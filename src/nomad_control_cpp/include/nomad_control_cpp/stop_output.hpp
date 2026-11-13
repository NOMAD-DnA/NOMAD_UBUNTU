#pragma once
// Stop output policy (2026-11-13). Decides what the controller sends while it owns the VESC but is
// not driving: 0A coast, VESC brake current, or VESC handbrake hold. ROS-free so it can be unit
// tested (test/test_stop_output.cpp).
//
// - Brake current is speed-dependent damping: it slows a rolling motor but cannot hold at standstill.
// - The handbrake (fixed current vector) holds at standstill. Firmware 5.03 forces the motor phase to
//   0 while it is applied, so the VESC then reports ERPM ~0 even if the rotor slips by whole poles.
//   The hold is therefore latched until the next drive command, and the reported ERPM is marked
//   invalid while it lasts.
// - The hold is entered only from healthy telemetry at or below hold_below_erpm, and only when the
//   handbrake can be delivered (the driver subscribes to the handbrake topic).
// - A real hold reads ~0 ERPM. A recently received reading (even one the controller rejected, e.g.
//   an overspeed sample) above holdReleaseErpm() after kHoldGraceS, or a handbrake topic that lost its
//   subscriber, means the hold is not in effect: fall back to the brake current (0A if the brake is
//   disabled) and select again on later samples.
// - A latched VESC fault code releases with 0A so a power stage that reported a fault is not driven
//   again. Software overcurrent, overspeed, stall and other latches keep the brake/hold.
//
// Load-sized hold current (2026-11-13, hold_min_current_a > 0). Only the VESC telemetry this controller already
// reads is used; nothing is published as a terrain/slope estimate. The handbrake current that holds the steepest
// allowed slope (11-13 VESC Tool: 30 A held 14 and 20 deg without sliding, below 30 A slid a little on 14 deg
// first) heats the motor on every stop, also on flat ground: two phases carry it, about 1.5 * R * I^2 = ~15 W
// at 30 A with R = 11.2 mOhm (~2.6 W at 12.5 A). So the hold is sized from the drive load (VESC avg_iq, the
// q-axis current) just before the stop:
//   s * iq = gravity + rolling  (uphill, s = sign of the commanded ERPM)
//   s * iq = rolling - gravity  (downhill, the speed PID brakes)
// so the load above rolling, L = |s * iq - rolling|, is the part a hold must resist (0 on flat ground).
// That balance only holds at steady speed. Starting, reaching a slope, or a speed PID that cannot keep up
// leaves the current below the load (fw 5.03 restarts the speed PID integrator from 0 at every handover),
// and a too-small hold slips without any ERPM sign. So:
// - Load (settled end): the measured ERPM stayed within kSettledFraction of the command for the last kSettledS.
//   hold = clamp(load_gain * L + margin, hold_min_current_a, hold_current_a), L the largest median-of-
//   kMedianSamples value among the settled samples of the last kLoadWindowS.
// - SamePlace: not settled, and the car moved at most kSamePlaceErev electrical revolutions since the last hold
//   whose current was set by Load/Unsettled/Unknown/Escalated (a short hop or a stalled restart; chained
//   SamePlace holds do not restart the count). hold = max(previous hold, the same formula with this drive's
//   largest L, settled or not), so a stalled push against a slope raises it.
// - Otherwise hold_current_a (the maximum): Unsettled (moved, or no earlier hold), Unknown (no usable samples),
//   Escalated (a hold was given up for reading high ERPM since the last drive).
// The first kSkipSamples avg_iq samples of a drive are not used: avg_iq is the mean since the previous telemetry
// read, so they still carry the stop output (+handbrake or -brake current).
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>

namespace nomad {

enum class StopOutputKind { Coast, Brake, Hold };

inline const char* stopOutputKindName(StopOutputKind kind) {
    switch (kind) {
    case StopOutputKind::Brake: return "brake";
    case StopOutputKind::Hold: return "hold";
    case StopOutputKind::Coast: break;
    }
    return "coast";
}

// Why the current hold uses its current (diagnostics).
enum class HoldBasis { None, Fixed, Load, SamePlace, Unsettled, Unknown, Escalated };

inline const char* holdBasisName(HoldBasis basis) {
    switch (basis) {
    case HoldBasis::Fixed: return "fixed";           // load sizing disabled: hold_current_a
    case HoldBasis::Load: return "load";             // from the settled end of the last drive
    case HoldBasis::SamePlace: return "same_place";  // not settled, hardly moved: at least the previous hold current
    case HoldBasis::Unsettled: return "unsettled";   // not settled and moved (or no earlier hold): hold_current_a
    case HoldBasis::Unknown: return "unknown";       // no usable drive samples: hold_current_a
    case HoldBasis::Escalated: return "escalated";   // a hold was given up since the last drive: hold_current_a
    case HoldBasis::None: break;
    }
    return "none";
}

struct StopOutputConfig {
    double brake_current_a = 0.0;   // ROS parameter stop_brake_current_a; 0 = no brake current
    double hold_current_a = 0.0;    // stop_handbrake_current_a; 0 = no handbrake hold. Fixed hold current,
                                    // or the maximum when load sizing is on
    double hold_below_erpm = 200.0; // stop_handbrake_below_erpm; enter the hold at or below this |ERPM|
    double hold_min_current_a = 0.0;     // stop_handbrake_min_current_a; > 0 turns load sizing on
    double hold_load_gain = 1.45;        // stop_handbrake_load_gain; hold amps per amp of load above rolling
                                         // (11-13: 14 deg climb 27 A = 21 A + ~6 A rolling, held at 30 A)
    double hold_rolling_current_a = 6.0; // stop_handbrake_rolling_current_a; drive current on flat ground
    double hold_margin_a = 8.0;          // stop_handbrake_margin_a; added to the scaled load
};

// Inputs for one decision.
struct StopOutputInput {
    double steady_now = 0.0;
    bool telemetry_healthy = false; // last sample fresh and passed the controller's checks (hold entry)
    bool erpm_recent = false;       // a sample (accepted or not) arrived recently (hold release check)
    double erpm = 0.0;              // last reported ERPM
    bool vesc_fault = false;        // latched VESC fault code
    bool hold_available = true;     // the handbrake topic has a subscriber (patched vesc_driver)
};

struct StopOutputCommand {
    StopOutputKind kind = StopOutputKind::Coast;
    double current_a = 0.0;
};

class StopOutput {
public:
    // Brake/handbrake current limit; the Pi4 vesc_driver yaml clips at brake_max/handbrake_max 30 A.
    static constexpr double kMaxCurrentA = 30.0;
    static constexpr double kMaxHoldBelowErpm = 2000.0;
    static constexpr double kMaxLoadGain = 5.0;
    static constexpr double kMaxRollingCurrentA = 30.0;
    // A hold that still reads high this long after it started is treated as not holding.
    static constexpr double kHoldGraceS = 0.10;
    // Load sizing. A sample is settled when |s * ERPM - |command|| <= kSettledFraction * |command|; the drive must
    // end with kSettledS of settled samples. L looks back kLoadWindowS. Samples further apart than kMaxSampleGapS
    // do not count as movement in between. kSamePlaceErev: 15 electrical revolutions is ~0.24 m on this car
    // (3721.88 ERPM per m/s = 62 per m), under the 0.335 m wheelbase.
    static constexpr double kSettledFraction = 0.25;
    static constexpr double kSettledS = 0.30;
    static constexpr double kLoadWindowS = 1.0;
    static constexpr double kMaxSampleGapS = 0.5;
    static constexpr double kSamePlaceErev = 15.0;
    static constexpr unsigned kSkipSamples = 3;   // 60 ms at the 50 Hz driver poll
    static constexpr std::size_t kMedianSamples = 5;

    explicit StopOutput(StopOutputConfig config = {}) : config_(config) {
        const auto within = [](double value, double low, double high) {
            return std::isfinite(value) && value >= low && value <= high;
        };
        if (!within(config_.brake_current_a, 0.0, kMaxCurrentA))
            throw std::invalid_argument("stop_brake_current_a must be within 0..30 A");
        if (!within(config_.hold_current_a, 0.0, kMaxCurrentA))
            throw std::invalid_argument("stop_handbrake_current_a must be within 0..30 A");
        if (!within(config_.hold_below_erpm, 0.0, kMaxHoldBelowErpm))
            throw std::invalid_argument("stop_handbrake_below_erpm must be within 0..2000 ERPM");
        if (!within(config_.hold_min_current_a, 0.0, config_.hold_current_a))
            throw std::invalid_argument("stop_handbrake_min_current_a must be within 0..stop_handbrake_current_a");
        if (!within(config_.hold_load_gain, 0.0, kMaxLoadGain))
            throw std::invalid_argument("stop_handbrake_load_gain must be within 0..5");
        if (!within(config_.hold_rolling_current_a, 0.0, kMaxRollingCurrentA))
            throw std::invalid_argument("stop_handbrake_rolling_current_a must be within 0..30 A");
        if (!within(config_.hold_margin_a, 0.0, kMaxCurrentA))
            throw std::invalid_argument("stop_handbrake_margin_a must be within 0..30 A");
    }

    const StopOutputConfig& config() const { return config_; }
    bool brakeEnabled() const { return config_.brake_current_a > 0.0; }
    bool holdEnabled() const { return config_.hold_current_a > 0.0; }
    bool loadHoldEnabled() const { return holdEnabled() && config_.hold_min_current_a > 0.0; }
    // Above this recently received |ERPM| (accepted or rejected sample), kHoldGraceS after the hold began, a
    // running hold is considered ineffective (a real hold reads ~0).
    double holdReleaseErpm() const { return std::max(2.0 * config_.hold_below_erpm, config_.hold_below_erpm + 200.0); }

    // A nonzero drive command was sent (every control tick while driving, with the ERPM sent to the VESC):
    // the hold ends and the next stop selects again. The first call after a stop, or a change of direction,
    // starts a new drive record.
    void onDrive(double commanded_erpm) {
        const int direction = commanded_erpm < 0.0 ? -1 : 1;
        if (!in_drive_ || direction != direction_) {
            in_drive_ = true;
            direction_ = direction;
            samples_.clear();
            skip_ = kSkipSamples;
            last_sample_s_.reset();
            drive_peak_load_a_.reset();
            escalated_ = false;
        }
        command_erpm_ = std::isfinite(commanded_erpm) ? std::abs(commanded_erpm) : 0.0;
        hold_since_.reset();
        last_.reset();
    }

    // One VESC telemetry sample while driving: avg_iq (A) and the reported ERPM. Ignored outside a drive.
    void observeDrive(double steady_now, double iq_a, double erpm) {
        if (!in_drive_ || !std::isfinite(steady_now)) return;
        if (last_sample_s_ && steady_now < *last_sample_s_) return;
        if (last_sample_s_ && std::isfinite(erpm))
            moved_erev_ += std::abs(erpm) * std::min(steady_now - *last_sample_s_, kMaxSampleGapS) / 60.0;
        last_sample_s_ = steady_now;
        if (skip_ > 0) {
            --skip_;
            return;
        }
        if (!std::isfinite(iq_a) || !std::isfinite(erpm)) return;
        const bool settled = command_erpm_ > 0.0 &&
                             std::abs(direction_ * erpm - command_erpm_) <= kSettledFraction * command_erpm_;
        samples_.push_back({steady_now, direction_ * iq_a, settled});
        const double load = loadAt(samples_.size() - 1);
        drive_peak_load_a_ = drive_peak_load_a_ ? std::max(*drive_peak_load_a_, load) : load;
        while (samples_.front().t < steady_now - kLoadWindowS - kSettledS) samples_.pop_front();
    }

    StopOutputCommand decide(const StopOutputInput& in) {
        in_drive_ = false;
        if (in.vesc_fault) {
            hold_since_.reset();
            return remember({StopOutputKind::Coast, 0.0});
        }
        const bool finite_erpm = std::isfinite(in.erpm);
        const bool reads_high = in.erpm_recent && finite_erpm && hold_since_ &&
                                in.steady_now - *hold_since_ >= kHoldGraceS && std::abs(in.erpm) > holdReleaseErpm();
        if (hold_since_ && (!in.hold_available || reads_high)) {
            if (reads_high) escalated_ = true;
            hold_since_.reset();
            ++holds_abandoned_;
        }
        if (!hold_since_ && holdEnabled() && in.hold_available && in.telemetry_healthy && finite_erpm &&
            std::abs(in.erpm) <= config_.hold_below_erpm) {
            hold_since_ = in.steady_now;
            hold_current_a_ = selectHoldCurrent();
        }
        if (hold_since_) return remember({StopOutputKind::Hold, hold_current_a_});
        if (brakeEnabled()) return remember({StopOutputKind::Brake, config_.brake_current_a});
        return remember({StopOutputKind::Coast, 0.0});
    }

    bool holding() const { return hold_since_.has_value(); }
    // While the handbrake is held the reported ERPM is a frozen ~0, not a measurement.
    bool erpmValid() const { return !holding(); }
    double holdSeconds(double steady_now) const {
        return hold_since_ ? std::max(0.0, steady_now - *hold_since_) : 0.0;
    }
    // Holds given up because they read high or lost the handbrake subscriber (diagnostics).
    unsigned holdsAbandoned() const { return holds_abandoned_; }
    // Last stop output since the last drive command, if any.
    const std::optional<StopOutputCommand>& last() const { return last_; }
    // Basis and load L (A above rolling) that sized the hold in effect; None/empty otherwise.
    HoldBasis holdBasis() const { return holding() ? hold_basis_ : HoldBasis::None; }
    std::optional<double> holdLoadA() const { return holding() ? hold_load_a_ : std::nullopt; }
    // Electrical revolutions driven since the last hold whose current was not reused (SamePlace); ~62 per metre.
    double movedErev() const { return moved_erev_; }

private:
    struct DriveSample {
        double t = 0.0;
        double along_iq = 0.0; // avg_iq in the commanded direction (A)
        bool settled = false;
    };

    // |median of samples_[index] and up to kMedianSamples - 1 before it - rolling|: one odd sample cannot set L.
    double loadAt(std::size_t index) const {
        std::array<double, kMedianSamples> window{};
        std::size_t n = 0;
        for (std::size_t j = index + 1; j-- > 0 && n < kMedianSamples;) window[n++] = samples_[j].along_iq;
        std::sort(window.begin(), window.begin() + n);
        const double median = n % 2 ? window[n / 2] : 0.5 * (window[n / 2 - 1] + window[n / 2]);
        return std::abs(median - config_.hold_rolling_current_a);
    }

    // L from a drive that ended settled; empty otherwise.
    std::optional<double> settledLoadA() const {
        if (samples_.empty() || !samples_.back().settled) return std::nullopt;
        const double end = samples_.back().t;
        double settled_from = end;
        for (auto it = samples_.rbegin(); it != samples_.rend() && it->settled; ++it) settled_from = it->t;
        if (end - settled_from < kSettledS) return std::nullopt;
        std::optional<double> peak;
        for (std::size_t i = 0; i < samples_.size(); ++i) {
            if (!samples_[i].settled || samples_[i].t < end - kLoadWindowS) continue;
            const double load = loadAt(i);
            peak = peak ? std::max(*peak, load) : load;
        }
        return peak;
    }

    double sized(double load) const {
        return std::clamp(config_.hold_load_gain * load + config_.hold_margin_a, config_.hold_min_current_a,
                          config_.hold_current_a);
    }

    double selectHoldCurrent() {
        hold_load_a_.reset();
        double amps = config_.hold_current_a;
        if (!loadHoldEnabled()) {
            hold_basis_ = HoldBasis::Fixed;
        } else if (escalated_) {
            hold_basis_ = HoldBasis::Escalated;
        } else if (const auto settled = settledLoadA()) {
            hold_load_a_ = settled;
            amps = sized(*settled);
            hold_basis_ = HoldBasis::Load;
        } else if (previous_hold_a_ && moved_erev_ <= kSamePlaceErev) {
            hold_load_a_ = drive_peak_load_a_;
            amps = std::max(*previous_hold_a_, drive_peak_load_a_ ? sized(*drive_peak_load_a_) : 0.0);
            hold_basis_ = HoldBasis::SamePlace;
        } else {
            hold_basis_ = samples_.empty() ? HoldBasis::Unknown : HoldBasis::Unsettled;
        }
        previous_hold_a_ = amps;
        // SamePlace reuses a current, so it does not restart the distance: a chain of short hops adds up.
        if (hold_basis_ != HoldBasis::SamePlace) moved_erev_ = 0.0;
        return amps;
    }

    StopOutputCommand remember(StopOutputCommand command) {
        last_ = command;
        return command;
    }

    StopOutputConfig config_;
    std::optional<double> hold_since_;
    std::optional<StopOutputCommand> last_;
    unsigned holds_abandoned_ = 0;
    double hold_current_a_ = 0.0;
    HoldBasis hold_basis_ = HoldBasis::None;
    std::optional<double> hold_load_a_;
    std::optional<double> previous_hold_a_;
    bool escalated_ = false;
    bool in_drive_ = false;
    int direction_ = 1;
    double command_erpm_ = 0.0;
    unsigned skip_ = 0;
    double moved_erev_ = 0.0;
    std::optional<double> last_sample_s_;
    std::optional<double> drive_peak_load_a_; // largest L of the current/last drive record, settled or not
    std::deque<DriveSample> samples_;
};

} // namespace nomad
