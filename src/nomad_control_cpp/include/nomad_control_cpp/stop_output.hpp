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
#include <algorithm>
#include <cmath>
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

struct StopOutputConfig {
    double brake_current_a = 0.0;   // ROS parameter stop_brake_current_a; 0 = no brake current
    double hold_current_a = 0.0;    // stop_handbrake_current_a; 0 = no handbrake hold
    double hold_below_erpm = 200.0; // stop_handbrake_below_erpm; enter the hold at or below this |ERPM|
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
    // Parameter limits; the exception texts below repeat them for the operator.
    static constexpr double kMaxCurrentA = 30.0;
    static constexpr double kMaxHoldBelowErpm = 2000.0;
    // A hold that still reads high this long after it started is treated as not holding.
    static constexpr double kHoldGraceS = 0.10;

    explicit StopOutput(StopOutputConfig config = {}) : config_(config) {
        const auto valid_current = [](double amps) { return std::isfinite(amps) && amps >= 0.0 && amps <= kMaxCurrentA; };
        if (!valid_current(config_.brake_current_a))
            throw std::invalid_argument("stop_brake_current_a must be within 0..30 A");
        if (!valid_current(config_.hold_current_a))
            throw std::invalid_argument("stop_handbrake_current_a must be within 0..30 A");
        if (!std::isfinite(config_.hold_below_erpm) || config_.hold_below_erpm < 0.0 ||
            config_.hold_below_erpm > kMaxHoldBelowErpm)
            throw std::invalid_argument("stop_handbrake_below_erpm must be within 0..2000 ERPM");
    }

    const StopOutputConfig& config() const { return config_; }
    bool brakeEnabled() const { return config_.brake_current_a > 0.0; }
    bool holdEnabled() const { return config_.hold_current_a > 0.0; }
    // Above this recently received |ERPM| (accepted or rejected sample), kHoldGraceS after the hold began, a
    // running hold is considered ineffective (a real hold reads ~0).
    double holdReleaseErpm() const { return std::max(2.0 * config_.hold_below_erpm, config_.hold_below_erpm + 200.0); }

    // A nonzero drive command was sent: the hold ends and the next stop selects again.
    void onDrive() {
        hold_since_.reset();
        last_.reset();
    }

    StopOutputCommand decide(const StopOutputInput& in) {
        if (in.vesc_fault) {
            hold_since_.reset();
            return remember({StopOutputKind::Coast, 0.0});
        }
        const bool finite_erpm = std::isfinite(in.erpm);
        if (hold_since_ && (!in.hold_available ||
                            (in.erpm_recent && finite_erpm && in.steady_now - *hold_since_ >= kHoldGraceS &&
                             std::abs(in.erpm) > holdReleaseErpm()))) {
            hold_since_.reset();
            ++holds_abandoned_;
        }
        if (!hold_since_ && holdEnabled() && in.hold_available && in.telemetry_healthy && finite_erpm &&
            std::abs(in.erpm) <= config_.hold_below_erpm)
            hold_since_ = in.steady_now;
        if (hold_since_) return remember({StopOutputKind::Hold, config_.hold_current_a});
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

private:
    StopOutputCommand remember(StopOutputCommand command) {
        last_ = command;
        return command;
    }

    StopOutputConfig config_;
    std::optional<double> hold_since_;
    std::optional<StopOutputCommand> last_;
    unsigned holds_abandoned_ = 0;
};

} // namespace nomad
