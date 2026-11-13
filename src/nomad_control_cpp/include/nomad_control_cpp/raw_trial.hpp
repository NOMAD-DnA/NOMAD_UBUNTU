#pragma once

#include "nomad_control_cpp/trial_controller.hpp"
#include <deque>
#include <optional>
#include <string>

namespace nomad {

enum class RawAxis { Motor = 1, Steering = 2, Manual = 3 };

// 미보정 공중 시험용 후보 한계입니다. m/s·rad 보정이나 기계적 안전 범위를 증명하지 않습니다.
struct RawOptions {
    bool enabled = false;
    bool setup_acknowledged = false;
    bool hardware_timeout_verified = false;
    double motor_max_erpm = 1850.0;
    double overspeed_erpm = 2500.0;
    double servo_center = 0.50;
    double servo_min = 0.49;
    double servo_max = 0.51;
    double max_duration_s = 3.0;
    double telemetry_timeout_s = 0.30;
    double manual_timeout_s = 0.35;
    double max_tick_gap_s = 0.20;
    double stationary_hold_s = 0.50;
    double stationary_erpm = 50.0;
    // Battery input current limit. Low-speed starts draw large motor (phase) current but little
    // input current, so the two have separate limits.
    double current_limit_a = 30.0;
    double motor_current_limit_a = 30.0;
    // Above a limit: a short spike only releases output (headless planning resumes); it latches
    // when it lasts overcurrent_sustain_s, exceeds overcurrent_hard_a once, or repeats
    // overcurrent_spike_limit times within overcurrent_spike_window_s.
    double overcurrent_hard_a = 45.0;
    double overcurrent_sustain_s = 0.30;
    int overcurrent_spike_limit = 3;
    double overcurrent_spike_window_s = 10.0;
    double motor_ramp_erpm_s = 1200.0;
    double motor_deceleration_erpm_s = 2400.0;
    bool stall_watchdog_enabled = false;
    double stall_timeout_s = 2.0;
    double stall_command_erpm = 50.0;
    double stall_feedback_erpm = 50.0;
    double servo_rate_s = 0.02;
    double manual_servo_rate_s = 0.40;
};

struct RawOutput {
    OutputKind kind = OutputKind::None;
    double motor_erpm = 0.0;
    std::optional<double> servo_position; // 비어 있으면 서보 명령을 발행하면 안 됩니다.
};

// ROS·변환 계수와 독립적인 raw 상태 머신. Manual은 heartbeat가 있는 동안 두 축을 함께 제어합니다.
class RawTrialController {
public:
    explicit RawTrialController(RawOptions options);
    void observe(const TelemetrySample& sample, double steady_now, double ros_now);
    bool start(RawAxis axis, double erpm, double servo, double duration,
               double steady_now, double ros_now, bool exclusive_driver_ready, double command_age_s = 0.0);
    bool update(RawAxis axis, double erpm, double servo, double steady_now, double ros_now,
                double command_age_s = 0.0);
    // Caller must authorize a soft planning pause and validate the fresh command.
    // Explicit STOP/fault/session expiry must never call this continuation path.
    bool resumePlanning(double erpm, double servo, double steady_now, double ros_now,
                        bool exclusive_driver_ready);
    // Enabled only by the headless planner; manual/commissioning keeps its latch policy.
    void setPlanningRecovery(bool enabled) { planning_recovery_ = enabled; }
    bool pausePlanning(const std::string& reason, double steady_now, double ros_now);
    bool planningWaitActive() const { return planning_wait_; }
    double telemetryReceiveAge(double steady) const { return sample_ ? steady - received_at_ : -1.0; }
    double telemetryStampAge(double ros) const { return sample_ ? ros - sample_->stamp_s : -1.0; }
    const std::string& telemetryIssue() const { return telemetry_issue_; }
    void stop(const std::string& reason, bool fault = false);
    bool reset(double steady_now, double ros_now);
    bool setSteeringTestSpan(double span, double steady_now, double ros_now);
    double steeringTestMin() const { return steering_test_min_; }
    double steeringTestMax() const { return steering_test_max_; }
    bool outsideDriveRange() const { return commanded_servo_<options_.servo_min-1e-9||commanded_servo_>options_.servo_max+1e-9; }
    RawOutput tick(double steady_now, double ros_now, bool exclusive_driver_ready);
    // Caller enables this floor only for an authorized planning drive tick.
    // The original overload preserves manual/bench ramps and its binary ABI.
    RawOutput tick(double steady_now, double ros_now, bool exclusive_driver_ready,
                   double minimum_drive_erpm);
    bool healthy(double steady_now, double ros_now) const;
    bool stationary(double steady_now, double ros_now) const;
    TrialState state() const { return state_; }
    // A latched fault reported by the VESC itself (nonzero fault code). The node then releases with
    // 0A instead of brake/hold. Cleared by an accepted reset().
    bool vescFault() const { return vesc_fault_; }
    const std::string& reason() const { return reason_; }
    bool ownsOutput() const { return output_owned_; }
    bool manualActive() const { return state_ == TrialState::Running; }
    bool continuousManualActive() const { return manualActive() && axis_ == RawAxis::Manual; }
    double commandedServoPosition() const { return commanded_servo_; }
    double commandedMotorErpm() const { return commanded_erpm_; }
    double targetMotorErpm() const { return target_erpm_; }
    double requestedServoPosition() const { return target_servo_; }
    const RawOptions& options() const { return options_; }
    double remaining(double steady_now) const;

private:
    bool validTarget(RawAxis axis, double erpm, double servo) const;
    bool validCommandAge(RawAxis axis, double age) const;
    double manualTimeout(RawAxis axis) const;
    bool checkClock(double steady_now, double ros_now);
    bool checkRunning(double steady_now, double ros_now);
    bool checkMotorProgress(double steady_now);
    // Returns true when the sample is within the current limit.
    bool checkCurrent(const TelemetrySample& sample, double steady_now);
    // recoverable: a planning pause may resume without RESET. vesc_fault: the VESC reported a fault code.
    void invalidateTelemetry(const std::string& reason, bool recoverable = false, bool vesc_fault = false);
    RawOutput restingOutput() const;

    RawOptions options_;
    TrialState state_;
    std::string reason_ = "waiting for raw telemetry; calibration is unverified";
    bool output_owned_ = false;
    bool telemetry_ok_ = false;
    std::optional<TelemetrySample> sample_;
    std::optional<double> last_steady_now_, last_ros_now_;
    std::optional<double> stationary_since_, last_tick_;
    std::optional<double> stalled_since_;
    std::optional<double> overcurrent_since_;
    std::deque<double> overcurrent_spikes_;
    RawAxis axis_ = RawAxis::Motor;
    double received_at_ = 0.0;
    double started_at_ = 0.0;
    double last_manual_at_ = 0.0;
    double duration_s_ = 0.0;
    double target_erpm_ = 0.0;
    double target_servo_ = 0.50;
    double commanded_erpm_ = 0.0;
    // 마지막 송신 명령을 보존합니다. 최초 값은 센서값이 아닌 중립 후보 가정입니다.
    double commanded_servo_ = 0.50;
    // 사용자가 정지 중 설정하는 조향 전용 시험 한계. 일반 주행 한계와 분리하며 재시작 시 초기화합니다.
    double steering_test_min_, steering_test_max_;
    bool planning_recovery_ = false, planning_wait_ = false;
    bool vesc_fault_ = false;
    std::string telemetry_issue_ = "NO_TELEMETRY";
};

} // namespace nomad
