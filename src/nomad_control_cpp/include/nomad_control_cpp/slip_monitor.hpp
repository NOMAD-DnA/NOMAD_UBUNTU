#pragma once
// Slip / stuck detection (2026-11-17, "4a"). Display and logs only: no control decision reads it, the output is
// unchanged. ROS-free so it can be unit tested (test/test_slip_monitor.cpp).
//
// Why: on 11-16 (Umyeonsan dirt and leaf slopes) the wheels spun in place at 25-49 A while VIO showed the body
// still. The sensorless ERPM swung 0-1000, so the stall watchdog (ERPM < 50 for 2 s) never fired. Once the car slid
// 0.53 m back unseen. The controller only knew the wheel speed; this compares it with the body speed from the fused
// localization pose (Pi5, over Wi-Fi in the Jetson-planner setup).
//
// Body speed is the pose displacement over the last kSpeedWindowS. The pose frame is the VIO's own frame. Speed
// magnitude does not depend on that frame. The direction comes from the last forward drive: a reference unit vector
// taken while driving forward at a clear body speed.
//
// Detected (coarse: no verified ERPM to m/s factor is needed):
// - Stuck (no progress): commanded forward at kStuckCommandErpm or more, mean iq >= kStuckIqA over kSpeedWindowS,
//   and the body below kStuckBodyMps. Becomes an event after kStuckHoldS; ends after kEndHoldS without the condition.
//   The measured ERPM is not a condition (review 11-17): on the 11-16 spins it swung 0-1000, so a mean-ERPM gate
//   missed every replayed case. The event line reports the mean ERPM, so a spin (ERPM up) and a blocked push (ERPM
//   near 0) can still be told apart.
// - Back slide: driving forward or holding, the body moves backward (along the reference) faster than
//   kBackslideMps. Becomes an event after kBackslideHoldS; ends after kEndHoldS without it. The distance is reported.
//
// Off (no detection) when:
// - the pose is missing, or older than kPoseStaleS at receipt (Wi-Fi);
// - the localization status is not READY, or older than kStatusStaleS;
// - the pose jumped (a VIO reset), or the pose window is still filling.
// A pose step faster than kCorrectionMps (a fused-pose correction, not motion) restarts the speed window but keeps
// the reference. Back slide additionally needs a reference direction younger than kReferenceMaxAgeS.
// The reference is taken only when the body moves the way the wheels do: speed at least kReferenceAgree times the
// coarse wheel speed (nominal ERPM per m/s, not verified), and within 60 degrees of the previous reference. Otherwise
// a slide while driving forward would become the "forward" direction (review 11-17).
// ERPM samples within kHoldEdgeIgnoreS of a hold start or end are skipped. Right after the handbrake engages or
// releases, the reported ERPM reads +-180-360 for one sample.
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>

namespace nomad {

enum class SlipState { Off, Ok, Stuck, BackSlide };

inline const char* slipStateName(SlipState state) {
    switch (state) {
    case SlipState::Ok: return "ok";
    case SlipState::Stuck: return "stuck";
    case SlipState::BackSlide: return "backslide";
    case SlipState::Off: break;
    }
    return "off";
}

struct SlipEvent {          // a change worth one log line
    SlipState state = SlipState::Ok;  // the new state (Stuck/BackSlide start, Ok = the event ended)
    SlipState ended = SlipState::Ok;  // for an end: which event ended
    double duration_s = 0.0;          // for an end: how long it lasted
    double distance_m = 0.0;          // back slide: distance moved backward during the event
    double erpm = 0.0, iq_a = 0.0, body_mps = 0.0;  // means at the start
};

class SlipMonitor {
public:
    static constexpr double kSpeedWindowS = 0.5;
    static constexpr double kPoseStaleS = 0.3;
    static constexpr double kStatusStaleS = 2.5;     // more than two periods of a 1 Hz status
    static constexpr double kJumpM = 0.5;              // displacement between consecutive poses that means a VIO reset
    static constexpr double kStuckCommandErpm = 500.0;
    static constexpr double kCorrectionMps = 1.5;    // pose step speed that is a correction, not motion
    static constexpr double kNominalErpmPerMps = 3721.88; // planning.erpm_per_mps (not verified): reference check only
    static constexpr double kReferenceAgree = 0.5;
    static constexpr double kReferenceMinCos = 0.5;  // 60 degrees
    static constexpr double kStuckIqA = 15.0;
    static constexpr double kStuckBodyMps = 0.03;
    static constexpr double kStuckHoldS = 1.0;
    static constexpr double kBackslideMps = 0.05;
    static constexpr double kBackslideHoldS = 0.3;
    static constexpr double kEndHoldS = 0.3;
    static constexpr double kReferenceMinMps = 0.10;   // body speed that sets the forward reference
    static constexpr double kReferenceMaxAgeS = 20.0;
    static constexpr double kHoldEdgeIgnoreS = 0.1;

    // Fused pose (x, y, z in its own frame), stamped by the sender, received at steady time `received`.
    void onPose(double stamp_s, double x, double y, double z, double received) {
        if (!std::isfinite(stamp_s) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
        if (!poses_.empty()) {
            const Pose& last = poses_.back();
            if (stamp_s <= last.stamp) return;  // out of order or repeated
            const double step = distance(last, Pose{stamp_s, x, y, z});
            if (step > kJumpM) { poses_.clear(); reference_.reset(); jumped_ = true; }
            else if (step > 0.05 && step / (stamp_s - last.stamp) > kCorrectionMps) poses_.clear();  // keep reference
        }
        poses_.push_back({stamp_s, x, y, z});
        while (poses_.size() > 2 && poses_.back().stamp - poses_.front().stamp > 3.0 * kSpeedWindowS) poses_.pop_front();
        pose_received_ = received;
    }
    // Localization ModuleStatus: READY (1) or not, received at steady time `received`.
    void onStatus(bool ready, double received) { status_ready_ = ready; status_received_ = received; }
    // One VESC telemetry sample.
    void onTelemetry(double steady, double erpm, double iq_a) {
        if (!std::isfinite(erpm) || !std::isfinite(iq_a)) return;
        if (hold_edge_ && steady - *hold_edge_ < kHoldEdgeIgnoreS) return;
        telemetry_.push_back({steady, erpm, iq_a});
        while (!telemetry_.empty() && steady - telemetry_.front().t > kSpeedWindowS) telemetry_.pop_front();
    }
    // Once per control tick. commanded_erpm: the drive command now (0 when not driving forward).
    // holding: the handbrake hold is on. Returns an event when a detection starts or ends.
    std::optional<SlipEvent> tick(double steady, double commanded_erpm, bool holding) {
        const bool driving_forward = std::isfinite(commanded_erpm) && commanded_erpm > 0.0;
        if (holding != last_holding_) { hold_edge_ = steady; last_holding_ = holding; telemetry_.clear(); }
        reason_ = offReason(steady);
        const auto speed = bodyVelocity();
        body_mps_ = speed ? std::optional<double>{speed->norm} : std::nullopt;
        along_mps_.reset();
        if (reason_.empty() && !speed) reason_ = jumped_ ? "pose jumped (VIO reset)" : "collecting pose window";
        if (!reason_.empty() || !speed) {
            stuck_since_.reset(); back_since_.reset();
            return finish(steady, SlipState::Off);
        }
        jumped_ = false;
        // Forward reference: the direction of a clear forward drive that the body really follows.
        const double wheel_mps = mean(&Sample::erpm) / kNominalErpmPerMps;
        if (driving_forward && speed->norm >= kReferenceMinMps && wheel_mps > 0.08 &&
            speed->norm >= kReferenceAgree * wheel_mps) {
            const Dir next{speed->dx / speed->norm, speed->dy / speed->norm, speed->dz / speed->norm};
            const bool agrees = !reference_ || next.x * reference_->x + next.y * reference_->y + next.z * reference_->z >=
                                kReferenceMinCos;
            if (agrees) { reference_ = next; reference_at_ = steady; }
        }
        if (reference_ && reference_at_ && steady - *reference_at_ <= kReferenceMaxAgeS)
            along_mps_ = speed->dx * reference_->x + speed->dy * reference_->y + speed->dz * reference_->z;
        const bool stuck_now = commanded_erpm >= kStuckCommandErpm && telemetry_.size() >= 5 &&
                               mean(&Sample::iq) >= kStuckIqA && speed->norm < kStuckBodyMps;
        const bool back_now = (driving_forward || holding) && along_mps_ && *along_mps_ < -kBackslideMps;
        track(stuck_since_, stuck_now, steady);
        track(back_since_, back_now, steady);
        if (state_ == SlipState::BackSlide && along_mps_ && *along_mps_ < 0.0 && last_tick_)
            back_distance_ += -*along_mps_ * (steady - *last_tick_);
        last_tick_ = steady;
        if (state_ == SlipState::Ok || state_ == SlipState::Off) {
            if (back_since_ && steady - *back_since_ >= kBackslideHoldS) return start(steady, SlipState::BackSlide, *speed);
            if (stuck_since_ && steady - *stuck_since_ >= kStuckHoldS) return start(steady, SlipState::Stuck, *speed);
            return finish(steady, SlipState::Ok);
        }
        const bool still = state_ == SlipState::Stuck ? stuck_now : back_now;
        if (still) { clear_since_.reset(); return std::nullopt; }
        if (!clear_since_) clear_since_ = steady;
        if (steady - *clear_since_ >= kEndHoldS) return finish(steady, SlipState::Ok);
        return std::nullopt;
    }

    SlipState state() const { return state_; }
    const std::string& offReason() const { return reason_; }    // why Off ("" when on)
    std::optional<double> bodySpeed() const { return body_mps_; }
    std::optional<double> alongSpeed() const { return along_mps_; }
    double since() const { return state_since_; }                // steady time the current state began
    unsigned events() const { return events_; }
    std::size_t telemetrySamples() const { return telemetry_.size(); }  // in the current window (tests)
    double lastBackslideM() const { return last_backslide_m_; }

private:
    struct Pose { double stamp, x, y, z; };
    struct Sample { double t, erpm, iq; };
    struct Dir { double x, y, z; };
    struct Velocity { double dx, dy, dz, norm; };

    static double distance(const Pose& a, const Pose& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    }
    std::string offReason(double steady) {
        if (poses_.empty() || !pose_received_) return jumped_ ? "pose jumped (VIO reset)" : "no localization pose";
        if (steady - *pose_received_ > kPoseStaleS) return "localization pose stale";
        if (!status_received_ || steady - *status_received_ > kStatusStaleS) return "no localization status";
        if (!status_ready_) return "localization not READY";
        return "";
    }
    // Velocity over the last kSpeedWindowS of poses (needs at least half a window of history).
    std::optional<Velocity> bodyVelocity() const {
        if (poses_.size() < 2) return std::nullopt;
        const Pose& last = poses_.back();
        const Pose* first = &poses_.front();
        for (const Pose& p : poses_) { if (last.stamp - p.stamp <= kSpeedWindowS) { first = &p; break; } }
        const double dt = last.stamp - first->stamp;
        if (dt < 0.5 * kSpeedWindowS) return std::nullopt;
        Velocity v{(last.x - first->x) / dt, (last.y - first->y) / dt, (last.z - first->z) / dt, 0.0};
        v.norm = std::sqrt(v.dx * v.dx + v.dy * v.dy + v.dz * v.dz);
        return v;
    }
    double mean(double Sample::*field) const {
        if (telemetry_.empty()) return 0.0;
        double sum = 0.0;
        for (const Sample& s : telemetry_) sum += s.*field;
        return sum / static_cast<double>(telemetry_.size());
    }
    static void track(std::optional<double>& since, bool now, double steady) {
        if (!now) since.reset(); else if (!since) since = steady;
    }
    std::optional<SlipEvent> start(double steady, SlipState state, const Velocity& v) {
        state_ = state; state_since_ = steady; clear_since_.reset(); back_distance_ = 0.0; ++events_;
        SlipEvent e; e.state = state; e.erpm = mean(&Sample::erpm); e.iq_a = mean(&Sample::iq); e.body_mps = v.norm;
        return e;
    }
    // Leaves an event (if one runs) for `next` (Ok or Off).
    std::optional<SlipEvent> finish(double steady, SlipState next) {
        std::optional<SlipEvent> e;
        if (state_ == SlipState::Stuck || state_ == SlipState::BackSlide) {
            e = SlipEvent{};
            e->state = SlipState::Ok; e->ended = state_; e->duration_s = steady - state_since_;
            e->distance_m = state_ == SlipState::BackSlide ? back_distance_ : 0.0;
            if (state_ == SlipState::BackSlide) last_backslide_m_ = back_distance_;
        }
        if (state_ != next) { state_ = next; state_since_ = steady; }
        clear_since_.reset();
        return e;
    }

    std::deque<Pose> poses_;
    std::deque<Sample> telemetry_;
    std::optional<double> pose_received_, status_received_, stuck_since_, back_since_, clear_since_, reference_at_,
        hold_edge_, last_tick_, body_mps_, along_mps_;
    std::optional<Dir> reference_;
    bool status_ready_ = false, last_holding_ = false, jumped_ = false;
    SlipState state_ = SlipState::Off;
    double state_since_ = 0.0, back_distance_ = 0.0, last_backslide_m_ = 0.0;
    unsigned events_ = 0;
    std::string reason_ = "no localization pose";
};

} // namespace nomad
