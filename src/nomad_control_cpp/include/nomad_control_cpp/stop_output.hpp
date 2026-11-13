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
// Usable samples (11-13 evening). s * iq - rolling = load + inertia * dERPM/dt, so the current is the load only at
// near-constant speed. Flat launches (11-13, kp 0.010) draw 15-25 A at +2,000-5,000 ERPM/s against 4-7 A at
// constant speed, and the overshoot recovery runs below rolling (0-4 A); both would read as slope. A sample is
// usable when, over kAccelWindowS centred on it (trailing for the last half window at the stop):
// - steady: the speed changed by at most kSteadyAccel and stayed within kSteadyRange (the range rejects the launch,
//   where the reported speed jumps to ~600 ERPM and drops back at 20-27 A); its L is the load; or
// - a lower bound, from kLaunchIgnoreS after the drive start: the speed changed against its excess (slowing while
//   above rolling, as when meeting a slope or obstacle, or speeding up while below it, downhill). The inertia
//   current then hides part of the load, so L plus kInertiaLowA * |accel| (a low inertia value) stays below it.
// - Load (settled end): the measured ERPM stayed within kSettledFraction of the command for the last kSettledS.
//   hold = clamp(load_gain * L + margin, hold_min_current_a, hold_current_a), L the largest median-of-
//   kMedianSamples value among the settled and usable samples of the last kLoadWindowS.
// - SamePlace: not settled, and the car moved at most kSamePlaceErev electrical revolutions since the last hold
//   whose current was set by Load/Unsettled/Unknown/Escalated (a short hop or a stalled restart; chained
//   SamePlace holds do not restart the count). hold = max(previous hold, the same formula with this drive's
//   largest usable L, settled or not), so a stalled push against a slope raises it. A drive with no usable sample
//   (it kept accelerating) uses its largest L from kLaunchIgnoreS on instead, acceleration current included (as
//   before 11-13 evening: conservative, e.g. a hop onto a slope), unless it is a tiny creep (at most kTinyHopErev
//   since that hold and shorter than kTinyHopMaxS), which keeps the previous hold.
// - Otherwise hold_current_a (the maximum): Unsettled (moved, or no earlier hold), Unknown (no usable samples),
//   Escalated (a hold was given up for reading high ERPM since the last drive).
// A drive that ends slowing faster than kEndDecel with its last current kEndExcessA above rolling (it just met a slope
// or obstacle) uses at least its launch current (Load and SamePlace). A stop within ~20-30 ms of the front axle
// touching a slope shows nothing in the current yet and can be held for the flat; a slip then only rolls the axle
// back to the foot of the slope (~1 cm in the 11-13 review model).
// The first kSkipSamples avg_iq samples of a drive are not used: avg_iq is the mean since the previous telemetry
// read, so they still carry the stop output (+handbrake or -brake current).
// Stop ramp (2026-11-13 night): an LPP STOP or lost LPP input ramps the speed command down first (raw_trial
// beginStopRamp). beginStopRamp() ends the drive record for the hold where the ramp begins: the settled end, the end
// rule and the tiny-creep time use the drive up to there, since a falling command is not an unsettled drive and its
// lower current is not less load. Ramp samples from kRampIgnoreS on can only raise L (Load: max with the usable ramp
// samples; SamePlace: the drive's largest usable L); movement during the ramp still counts (conservative for the
// same-place and tiny-creep distances). A slope edge crossed in the last ~0.15 s of the ramp (~2-3 cm) shows nothing
// yet, as one crossed while braking or just before an immediate stop did before; the 11-13 review model found the
// ramp moves that band but does not widen it.
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
    // Steady sample: |speed change| over kAccelWindowS (centred on it) at most kSteadyAccel ERPM/s. 600 ERPM/s is
    // ~2 A of inertia current here (flat launch: ~0.003-0.004 A per ERPM/s) and above the HFI speed noise (~300).
    static constexpr double kAccelWindowS = 0.20;
    static constexpr double kMinAccelSpanS = 0.15;
    static constexpr double kSteadyAccel = 600.0;
    static constexpr double kSteadyRange = 300.0;   // ERPM within the window (noise + kp 0.010 ripple ~150-250)
    static constexpr double kTinyHopErev = 3.0;     // ~5 cm, chained since the last hold that set its current
    static constexpr double kTinyHopMaxS = 0.28;    // flat LPP hops of <= 0.24 s moved 0.5-1 erev (11-13 logs)
    static constexpr double kLaunchIgnoreS = 0.20;  // the handover jump of the reported speed (~0.06-0.18 s)
    // Inertia current per ERPM/s added back to a lower-bound sample. The 11-13 flat launches put the real value at
    // ~0.002-0.006 A per ERPM/s; the low end keeps the corrected value a lower bound of the load.
    static constexpr double kInertiaLowA = 0.002;
    // A drive that ends slowing faster than kEndDecel with its last current kEndExcessA above rolling just met more
    // load (a slope or obstacle) and has had no time to show it: no better than the launch current (conservative).
    // The drive ends where a stop ramp begins, so flat stops end at cruise or in the overshoot recovery.
    static constexpr double kEndDecel = 1000.0;
    static constexpr double kEndExcessA = 2.0;
    // Stop ramp samples this soon after the ramp began are not used: the speed PID (~20-60 ms) and the
    // kMedianSamples median (~40-60 ms) still show the cruise current while the speed already drops, which reads
    // like meeting a slope.
    static constexpr double kRampIgnoreS = 0.15;

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
            drive_launch_load_a_.reset();
            drive_start_s_.reset();
            drive_moved_erev_ = 0.0;
            escalated_ = false;
            ramp_start_s_.reset();
        }
        if (!ramp_start_s_) command_erpm_ = std::isfinite(commanded_erpm) ? std::abs(commanded_erpm) : 0.0;
        hold_since_.reset();
        last_.reset();
    }

    // The drive ends with a commanded speed ramp (a planning stop, 2026-11-13): the hold is selected as if the stop
    // happened now. Later samples (ramp samples) are never settled and do not count as launch current; from
    // kRampIgnoreS on, a usable one can only raise the load (a slope or obstacle met during the ramp, or a downhill
    // the speed PID cannot slow). Movement still counts. No effect outside a drive or when already ramping.
    void beginStopRamp(double steady_now) {
        if (!in_drive_ || ramp_start_s_ || !std::isfinite(steady_now)) return;
        evaluateTail();
        ramp_start_s_ = last_sample_s_ ? std::max(steady_now, *last_sample_s_) : steady_now;
    }

    // The ramp was cancelled and the drive goes on: the next drive command starts a new record, as after a stop.
    void cancelStopRamp() {
        if (!in_drive_ || !ramp_start_s_) return;
        in_drive_ = false;
    }
    bool stopRampActive() const { return in_drive_ && ramp_start_s_.has_value(); }

    // One VESC telemetry sample while driving: avg_iq (A) and the reported ERPM. Ignored outside a drive.
    void observeDrive(double steady_now, double iq_a, double erpm) {
        if (!in_drive_ || !std::isfinite(steady_now)) return;
        if (last_sample_s_ && steady_now < *last_sample_s_) return;
        if (!drive_start_s_) drive_start_s_ = steady_now;
        if (last_sample_s_ && std::isfinite(erpm)) {
            const double erev = std::abs(erpm) * std::min(steady_now - *last_sample_s_, kMaxSampleGapS) / 60.0;
            moved_erev_ += erev;
            drive_moved_erev_ += erev;
        }
        last_sample_s_ = steady_now;
        if (skip_ > 0) {
            --skip_;
            return;
        }
        if (!std::isfinite(iq_a) || !std::isfinite(erpm)) return;
        const bool ramp = ramp_start_s_.has_value();
        const bool settled = !ramp && command_erpm_ > 0.0 &&
                             std::abs(direction_ * erpm - command_erpm_) <= kSettledFraction * command_erpm_;
        DriveSample sample;
        sample.t = steady_now;
        sample.along_iq = direction_ * iq_a;
        sample.along_erpm = direction_ * erpm;
        sample.settled = settled;
        sample.ramp = ramp;
        samples_.push_back(sample);
        samples_.back().excess = excessAt(samples_.size() - 1);
        samples_.back().load = std::abs(samples_.back().excess);
        if (!ramp && steady_now - *drive_start_s_ >= kLaunchIgnoreS)
            drive_launch_load_a_ = std::max(drive_launch_load_a_.value_or(0.0), samples_.back().load);
        evaluateCentred();
        // Keep what the hold needs: kLoadWindowS + kSettledS before the stop, or before the ramp began.
        const double end = ramp ? std::min(steady_now, *ramp_start_s_) : steady_now;
        while (samples_.front().t < end - kLoadWindowS - kSettledS) samples_.pop_front();
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
        double along_iq = 0.0;   // avg_iq in the commanded direction (A)
        double along_erpm = 0.0; // reported ERPM in the commanded direction
        double excess = 0.0;     // median of it and up to kMedianSamples - 1 before it, minus rolling (A, signed)
        double load = 0.0;       // L = |excess|
        double evidence = 0.0;   // L used for sizing: load, plus kInertiaLowA * |accel| for a lower-bound sample
        bool settled = false;
        bool ramp = false;       // taken during a stop ramp (beginStopRamp)
        bool evaluated = false;  // usable decided
        bool usable = false;     // steady, or a lower bound of the load (see classify)
    };

    // Window [lo, hi] around sample i: speed change rate across it and its ERPM range. Empty if too short.
    struct Window {
        double accel = 0.0;
        double range = 0.0;
    };
    std::optional<Window> window(double lo, double hi) const {
        std::optional<std::size_t> first, last;
        for (std::size_t j = 0; j < samples_.size(); ++j) {
            if (samples_[j].t < lo - 1e-9 || samples_[j].t > hi + 1e-9) continue;
            if (!first) first = j;
            last = j;
        }
        if (!first) return std::nullopt;
        const double span = samples_[*last].t - samples_[*first].t;
        if (span < kMinAccelSpanS) return std::nullopt;
        Window w;
        w.accel = (samples_[*last].along_erpm - samples_[*first].along_erpm) / span;
        double low = samples_[*first].along_erpm, high = low;
        for (std::size_t j = *first; j <= *last; ++j) {
            low = std::min(low, samples_[j].along_erpm);
            high = std::max(high, samples_[j].along_erpm);
        }
        w.range = high - low;
        return w;
    }

    // Usable as L: steady (|accel| <= kSteadyAccel and range <= kSteadyRange: the current is the load), or, from
    // kLaunchIgnoreS after the drive start on, accelerating against its excess (slowing while above rolling, or
    // speeding up while below it): the inertia current then makes |excess| smaller than the load, a lower bound.
    // Excluded: the flat launch (speeding up above rolling) and the overshoot recovery (slowing below rolling).
    void classify(DriveSample& sample, const std::optional<Window>& w) {
        sample.evaluated = true;
        if (!w || (sample.ramp && sample.t - *ramp_start_s_ < kRampIgnoreS)) return;
        const bool steady = std::abs(w->accel) <= kSteadyAccel && w->range <= kSteadyRange;
        const bool lower_bound = drive_start_s_ && sample.t - *drive_start_s_ >= kLaunchIgnoreS &&
                                 sample.excess * w->accel < 0.0;
        sample.usable = steady || lower_bound;
        sample.evidence = steady ? sample.load : sample.load + kInertiaLowA * std::abs(w->accel);
        if (sample.usable)
            drive_peak_load_a_ = drive_peak_load_a_ ? std::max(*drive_peak_load_a_, sample.evidence) : sample.evidence;
    }

    // Every sample whose centred window (kAccelWindowS / 2 each side) is complete, once.
    void evaluateCentred() {
        const double newest = samples_.back().t;
        for (auto& sample : samples_) {
            if (sample.evaluated || newest - sample.t < 0.5 * kAccelWindowS - 1e-9) continue;
            classify(sample, window(sample.t - 0.5 * kAccelWindowS, sample.t + 0.5 * kAccelWindowS));
        }
    }

    // At the stop, the last kAccelWindowS / 2 of samples have no complete centred window: use a trailing one.
    void evaluateTail() {
        for (auto& sample : samples_)
            if (!sample.evaluated) classify(sample, window(sample.t - kAccelWindowS, sample.t));
    }

    // Median of samples_[index] and up to kMedianSamples - 1 before it, minus rolling: one odd sample cannot set L.
    double excessAt(std::size_t index) const {
        std::array<double, kMedianSamples> values{};
        std::size_t n = 0;
        for (std::size_t j = index + 1; j-- > 0 && n < kMedianSamples;) values[n++] = samples_[j].along_iq;
        std::sort(values.begin(), values.begin() + n);
        const double median = n % 2 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
        return median - config_.hold_rolling_current_a;
    }

    // Number of samples before the stop ramp (all of them without one): the drive "ends" at the last of these.
    std::size_t endIndex() const {
        std::size_t n = samples_.size();
        while (n > 0 && samples_[n - 1].ramp) --n;
        return n;
    }

    // The last sample's trailing speed change and raw current show a load rise at the very end.
    bool endsMeetingLoad() const {
        const std::size_t n = endIndex();
        if (n == 0) return false;
        const auto& last = samples_[n - 1];
        const auto w = window(last.t - kAccelWindowS, last.t);
        return w && w->accel < -kEndDecel && last.along_iq - config_.hold_rolling_current_a > kEndExcessA;
    }

    // Drive command time up to the stop, or up to the start of the stop ramp.
    double driveSpanS() const {
        if (!drive_start_s_ || !last_sample_s_) return 0.0;
        const double end = ramp_start_s_ ? std::min(*last_sample_s_, *ramp_start_s_) : *last_sample_s_;
        return std::max(0.0, end - *drive_start_s_);
    }

    // L from a drive that ended settled; empty otherwise.
    std::optional<double> settledLoadA() const {
        const std::size_t n = endIndex();
        if (n == 0 || !samples_[n - 1].settled) return std::nullopt;
        const double end = samples_[n - 1].t;
        double settled_from = end;
        for (std::size_t j = n; j-- > 0 && samples_[j].settled;) settled_from = samples_[j].t;
        if (end - settled_from < kSettledS) return std::nullopt;
        std::optional<double> peak;
        for (std::size_t j = 0; j < n; ++j) {
            const auto& sample = samples_[j];
            if (!sample.settled || !sample.usable || sample.t < end - kLoadWindowS) continue;
            peak = peak ? std::max(*peak, sample.evidence) : sample.evidence;
        }
        return peak;
    }

    // Largest usable L seen during the stop ramp; empty without one.
    std::optional<double> rampLoadA() const {
        std::optional<double> peak;
        for (std::size_t j = endIndex(); j < samples_.size(); ++j)
            if (samples_[j].usable) peak = peak ? std::max(*peak, samples_[j].evidence) : samples_[j].evidence;
        return peak;
    }

    double sized(double load) const {
        return std::clamp(config_.hold_load_gain * load + config_.hold_margin_a, config_.hold_min_current_a,
                          config_.hold_current_a);
    }

    double selectHoldCurrent() {
        evaluateTail();
        hold_load_a_.reset();
        double amps = config_.hold_current_a;
        if (!loadHoldEnabled()) {
            hold_basis_ = HoldBasis::Fixed;
        } else if (escalated_) {
            hold_basis_ = HoldBasis::Escalated;
        } else if (const auto settled = settledLoadA()) {
            hold_load_a_ = settled;
            if (endsMeetingLoad() && drive_launch_load_a_) hold_load_a_ = std::max(*hold_load_a_, *drive_launch_load_a_);
            if (const auto ramp = rampLoadA()) hold_load_a_ = std::max(*hold_load_a_, *ramp);
            amps = sized(*hold_load_a_);
            hold_basis_ = HoldBasis::Load;
        } else if (previous_hold_a_ && moved_erev_ <= kSamePlaceErev) {
            // Usable evidence first. A drive without any falls back to its launch current (conservative, e.g. a
            // short hop onto a slope), unless it was a tiny creep: chained distance <= kTinyHopErev and shorter than
            // kTinyHopMaxS (a slope launch is slow, so a longer drive that hardly moved was pushing against load).
            hold_load_a_ = drive_peak_load_a_;
            const bool tiny = moved_erev_ <= kTinyHopErev && driveSpanS() < kTinyHopMaxS;
            if ((!hold_load_a_ && !tiny) || (endsMeetingLoad() && drive_launch_load_a_ &&
                                              *drive_launch_load_a_ > hold_load_a_.value_or(0.0)))
                hold_load_a_ = drive_launch_load_a_;
            amps = std::max(*previous_hold_a_, hold_load_a_ ? sized(*hold_load_a_) : 0.0);
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
    std::optional<double> drive_peak_load_a_; // largest usable L of the current/last drive record, settled or not
    std::optional<double> drive_launch_load_a_; // largest L from kLaunchIgnoreS on, steady or not
    std::optional<double> drive_start_s_;
    std::optional<double> ramp_start_s_;      // the current/last drive record ended with a stop ramp from here
    double drive_moved_erev_ = 0.0;            // electrical revolutions of the current/last drive record
    std::deque<DriveSample> samples_;
};

} // namespace nomad
