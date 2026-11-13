#pragma once

#include "nomad_control_cpp/vesc_mapping.hpp"
#include <optional>
#include <cmath>
#include <stdexcept>
#include <string>

namespace nomad {

enum class TrialState { Monitor, Idle, Running, Stopped, Fault };
enum class OutputKind { None, Drive, Release };

struct TrialOptions {
    bool actuation_enabled = false;
    bool calibration_verified = false;
    bool hardware_timeout_verified = false;
    double target_speed_mps = 0.0;
    double target_steering_rad = 0.0;
    double duration_s = 1.0; // 조향 준비를 포함한 전체 시험 시간. 최대 5초.
    double telemetry_timeout_s = 0.30;
    double max_tick_gap_s = 0.20;
    double stationary_hold_s = 0.50;
    double stationary_erpm = 50.0; // 실제 정지 잡음에 맞춰 확인할 raw ERPM 문턱.
    double stationary_speed_mps = 0.03;
    double current_limit_a = 30.0; // 소프트웨어 감시값이며 VESC 펌웨어 제한을 대체하지 않음.
    double acceleration_mps2 = 0.20;
    double steering_rate_radps = 0.30;
    double steering_settle_s = 0.30; // 명령각 도달 뒤 대기. 실제 조향 센서 확인은 아님.
    double manual_timeout_s = 0.35; // GUI 제어 메시지 단절 시 중단. 전체 시험 시간과 별개입니다.
};

struct TelemetrySample {
    double stamp_s; // ROS 시각. steady 시각과 직접 빼지 않습니다.
    double raw_erpm;
    double current_motor_a;
    double current_input_a;
    double voltage_v;
    int fault_code;
};

struct TrialOutput {
    OutputKind kind = OutputKind::None;
    double motor_erpm = 0.0;
    double servo_position = 0.0;
};

// ROS IO와 분리한 시험 상태 머신. 시간은 호출자가 공급하므로 가짜 시각으로 시험할 수 있습니다.
class TrialController {
public:
    TrialController(Calibration calibration, TrialOptions options);
    void observe(const TelemetrySample& sample, double steady_now, double ros_now);
    bool start(double steady_now, double ros_now, bool exclusive_driver_ready);
    bool startManual(double speed, double steering, double duration, double steady_now,
                     double ros_now, bool exclusive_driver_ready);
    bool updateManual(double speed, double steering, double steady_now, double ros_now);
    void stop(const std::string& reason, bool fault = false);
    bool reset(double steady_now, double ros_now);
    TrialOutput tick(double steady_now, double ros_now, bool exclusive_driver_ready);
    bool healthy(double steady_now, double ros_now) const;
    bool stationary(double steady_now, double ros_now) const;
    TrialState state() const { return state_; }
    const std::string& reason() const { return reason_; }
    // A latched fault reported by the VESC itself (nonzero fault code). The node then releases with
    // 0A instead of brake/hold. Cleared by an accepted reset().
    bool vescFault() const { return vesc_fault_; }
    std::optional<double> measuredSpeed() const { return measured_speed_; }
    bool ownsOutput() const { return output_owned_; }
    bool manualActive() const { return manual_active_; }
    const TrialOptions& options() const { return options_; }
    double commandedSteering() const { return commanded_steering_; }
    double remaining(double steady_now) const;
    // 외부 안정성 감시가 계산한 물리 단위 한계. 기본 시험 동작에는 적용하지 않습니다.
    void setStabilityLimits(double speed, double acceleration) {
        if (!std::isfinite(speed) || !std::isfinite(acceleration) || speed<0 || acceleration<=0)
            throw std::invalid_argument("invalid stability motion limits");
        stability_speed_cap_ = speed; stability_accel_cap_ = acceleration;
    }

private:
    bool checkClock(double steady_now, double ros_now);
    void invalidateTelemetry(const std::string& reason, bool vesc_fault = false);
    TrialOutput restingOutput() const;
    Calibration calibration_;
    TrialOptions options_;
    TrialState state_;
    std::string reason_ = "waiting for telemetry";
    bool output_owned_ = false;
    bool telemetry_ok_ = false;
    bool vesc_fault_ = false;
    std::optional<TelemetrySample> sample_;
    std::optional<double> measured_speed_;
    std::optional<double> last_ros_now_;
    std::optional<double> last_steady_now_;
    std::optional<double> stationary_since_;
    std::optional<double> steering_reached_at_;
    std::optional<double> last_tick_;
    double received_at_ = 0.0;
    double started_at_ = 0.0;
    double commanded_speed_ = 0.0;
    double commanded_steering_ = 0.0;
    bool manual_active_ = false;
    double last_manual_received_ = 0.0;
    double stability_speed_cap_ = 1e9, stability_accel_cap_ = 1e9;
};

const char* trialStateName(TrialState state);
} // namespace nomad
