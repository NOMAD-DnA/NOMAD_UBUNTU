#include "nomad_control_cpp/trial_controller.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
using namespace std;

namespace nomad {
namespace {
bool positive(double value) { return isfinite(value) && value > 0.0; }

// 목표를 향한 변화량을 제한합니다. 비상 정지는 이 함수를 거치지 않습니다.
double approach(double previous, double target, double rate, double dt) {
    return previous + clamp(target - previous, -rate * dt, rate * dt);
}
} // namespace

TrialController::TrialController(Calibration calibration, TrialOptions options)
    : calibration_(calibration), options_(options),
      state_(options.actuation_enabled ? TrialState::Idle : TrialState::Monitor) {
    if (validateCalibration(calibration_) != Error::None ||
        !convertTarget(options_.target_speed_mps, options_.target_steering_rad, calibration_).value)
        throw invalid_argument("invalid calibration or trial target");
    if (!positive(options_.duration_s) || options_.duration_s > 5.0 ||
        !positive(options_.telemetry_timeout_s) || options_.telemetry_timeout_s > 0.5 ||
        !positive(options_.max_tick_gap_s) || options_.max_tick_gap_s > 0.25 ||
        !positive(options_.stationary_hold_s) || options_.stationary_hold_s < 0.2 ||
        !positive(options_.stationary_erpm) || !positive(options_.stationary_speed_mps) ||
        !positive(options_.current_limit_a) || !positive(options_.acceleration_mps2) ||
        !positive(options_.steering_rate_radps) || !positive(options_.steering_settle_s) ||
        !positive(options_.manual_timeout_s) || options_.manual_timeout_s > 0.5)
        throw invalid_argument("invalid trial timing or limits");
}

// ROS 시계 역행은 기존 패킷을 다시 최신으로 만들 수 있으므로 현재 시험을 중단합니다.
bool TrialController::checkClock(double steady_now, double ros_now) {
    const bool valid = isfinite(steady_now) && isfinite(ros_now) && ros_now > 0.0 &&
        (!last_steady_now_ || steady_now >= *last_steady_now_) &&
        (!last_ros_now_ || ros_now >= *last_ros_now_ - 0.05);
    if (!valid) {
        invalidateTelemetry("clock invalid or moved backwards; restart after clock reset");
        return false;
    }
    last_steady_now_ = steady_now;
    last_ros_now_ = ros_now;
    return true;
}

void TrialController::invalidateTelemetry(const string& reason, bool vesc_fault) {
    telemetry_ok_ = false;
    stationary_since_.reset();
    reason_ = reason;
    // A VESC fault also latches a stopped trial that still owns the output (it may be holding).
    if (state_ == TrialState::Running || (vesc_fault && output_owned_ &&
        (state_ == TrialState::Stopped || state_ == TrialState::Idle))) stop(reason, true);
    if (vesc_fault && state_ == TrialState::Fault) vesc_fault_ = true;
}

// 같은 stamp 재전송은 수신 시각이나 정지 유지 시간을 갱신하지 않습니다.
void TrialController::observe(const TelemetrySample& s, double steady_now, double ros_now) {
    if (!checkClock(steady_now, ros_now)) return;
    const double age = ros_now - s.stamp_s;
    if (!isfinite(s.stamp_s) || s.stamp_s <= 0.0 ||
        age < -0.05 || age > options_.telemetry_timeout_s ||
        !isfinite(s.raw_erpm) || !isfinite(s.current_motor_a) ||
        !isfinite(s.current_input_a) || !positive(s.voltage_v) || s.fault_code != 0 ||
        abs(s.current_motor_a) > options_.current_limit_a ||
        abs(s.current_input_a) > options_.current_limit_a) {
        invalidateTelemetry("invalid/stale telemetry, VESC fault or current limit", /*vesc_fault=*/s.fault_code != 0);
        return;
    }
    if (sample_ && s.stamp_s <= sample_->stamp_s) return;
    const auto speed = convertFeedback(s.raw_erpm, calibration_);
    if (!speed.value) {
        invalidateTelemetry(errorMessage(speed.error));
        return;
    }
    if (!telemetry_ok_ || steady_now - received_at_ > options_.telemetry_timeout_s)
        stationary_since_.reset();
    sample_ = s;
    measured_speed_ = *speed.value;
    received_at_ = steady_now;
    telemetry_ok_ = true;
    if (abs(s.raw_erpm) <= options_.stationary_erpm &&
        abs(*speed.value) <= options_.stationary_speed_mps) {
        if (!stationary_since_) stationary_since_ = steady_now;
    } else {
        stationary_since_.reset();
    }
    if (state_ == TrialState::Monitor || state_ == TrialState::Idle)
        reason_ = "telemetry received; start requires stationary hold and explicit service";
}

bool TrialController::healthy(double steady_now, double ros_now) const {
    if (!telemetry_ok_ || !sample_ || !isfinite(steady_now) || !isfinite(ros_now)) return false;
    const double receipt_age = steady_now - received_at_;
    const double stamp_age = ros_now - sample_->stamp_s;
    return receipt_age >= 0.0 && receipt_age <= options_.telemetry_timeout_s &&
        stamp_age >= -0.05 && stamp_age <= options_.telemetry_timeout_s;
}

// raw ERPM과 환산속도 모두 정지 문턱 안에서 연속 유지되어야 합니다.
bool TrialController::stationary(double steady_now, double ros_now) const {
    return healthy(steady_now, ros_now) && stationary_since_ &&
        steady_now - *stationary_since_ >= options_.stationary_hold_s;
}

bool TrialController::start(double steady_now, double ros_now, bool exclusive_driver_ready) {
    if (!checkClock(steady_now, ros_now)) return false;
    if (!options_.actuation_enabled || !options_.calibration_verified || !options_.hardware_timeout_verified) {
        reason_ = "start refused: monitor mode or unverified calibration/hardware timeout";
        return false;
    }
    if (state_ != TrialState::Idle) return false; // 중복 호출로 시험 시간을 연장하지 않습니다.
    if (!exclusive_driver_ready || !stationary(steady_now, ros_now)) {
        reason_ = "start refused: require exclusive driver connection and fresh stationary telemetry";
        return false;
    }
    state_ = TrialState::Running;
    reason_ = "bounded trial running";
    output_owned_ = true;
    started_at_ = steady_now;
    last_tick_ = steady_now;
    steering_reached_at_.reset();
    commanded_speed_ = 0.0;
    // 정지 때 유지한 마지막 명령각에서 이어갑니다. 실제 조향 센서값은 아니며,
    // 프로세스의 최초 시작은 초기값 0을 가정하므로 중립 확인이 별도로 필요합니다.
    return true;
}

// GUI가 정한 목표를 한 번의 제한된 시험에 적용합니다. 확인 플래그·정지 조건은 기존 start와 같습니다.
bool TrialController::startManual(double speed, double steering, double duration, double steady_now,
                                   double ros_now, bool exclusive_driver_ready) {
    if (state_ != TrialState::Idle || !positive(duration) || duration > 5.0 ||
        !convertTarget(speed, steering, calibration_).value) {
        reason_ = "manual start refused: invalid target/duration or reset required";
        return false;
    }
    const auto previous = options_;
    options_.target_speed_mps = speed;
    options_.target_steering_rad = steering;
    options_.duration_s = duration;
    if (!start(steady_now, ros_now, exclusive_driver_ready)) { options_ = previous; return false; }
    manual_active_ = true;
    last_manual_received_ = steady_now;
    return true;
}

// 늦게 도착한 메시지로 이미 만료된 GUI 제어를 되살리지 않습니다. 시작 시각도 갱신하지 않습니다.
bool TrialController::updateManual(double speed, double steering, double steady_now, double ros_now) {
    if (!checkClock(steady_now, ros_now) || !manual_active_ || state_ != TrialState::Running) return false;
    if (steady_now - last_manual_received_ > options_.manual_timeout_s ||
        steady_now - started_at_ >= options_.duration_s) {
        stop("manual command lease or trial duration expired", true);
        return false;
    }
    if (!convertTarget(speed, steering, calibration_).value) {
        stop("invalid manual target", true);
        return false;
    }
    if (commanded_speed_ == 0.0 && steering != options_.target_steering_rad) steering_reached_at_.reset();
    options_.target_speed_mps = speed;
    options_.target_steering_rad = steering;
    last_manual_received_ = steady_now;
    return true;
}

double TrialController::remaining(double steady_now) const {
    return state_ == TrialState::Running ? max(0.0, options_.duration_s - (steady_now - started_at_)) : 0.0;
}

// 중단 뒤에는 자동 재시작하지 않습니다. Release는 0A 구동 해제이며, 정지 브레이크/핸드브레이크를 설정하면
// 제어기 노드가 그 출력으로 바꿔 보냅니다(stop_brake_current_a, stop_handbrake_current_a). VESC 고장 코드로
// 잠기면 0A 그대로입니다.
void TrialController::stop(const string& reason, bool fault) {
    if (state_ == TrialState::Monitor) return;
    if (state_ == TrialState::Fault && !fault) return;
    state_ = fault ? TrialState::Fault : TrialState::Stopped;
    reason_ = reason;
    commanded_speed_ = 0.0;
    manual_active_ = false;
}

bool TrialController::reset(double steady_now, double ros_now) {
    if (!checkClock(steady_now, ros_now) || !options_.actuation_enabled ||
        state_ == TrialState::Running || !stationary(steady_now, ros_now)) return false;
    state_ = TrialState::Idle;
    vesc_fault_ = false;
    reason_ = "reset; waiting for explicit start";
    return true; // 이미 취득한 출력권은 유지하며 정지 중에는 정지 출력(0A 또는 브레이크/핸드브레이크)을 계속 보냅니다.
}

TrialOutput TrialController::restingOutput() const {
    return {output_owned_ ? OutputKind::Release : OutputKind::None, 0.0, 0.0};
}

TrialOutput TrialController::tick(double steady_now, double ros_now, bool exclusive_driver_ready) {
    if (!checkClock(steady_now, ros_now)) return restingOutput();
    if (state_ != TrialState::Running) return restingOutput();
    if (!healthy(steady_now, ros_now)) stop("telemetry timeout or invalid sample", true);
    else if (manual_active_ && steady_now - last_manual_received_ > options_.manual_timeout_s)
        stop("GUI command timeout", true);
    else if (!exclusive_driver_ready) stop("driver disconnected or competing command publisher", true);
    else if (last_tick_ && steady_now - *last_tick_ > options_.max_tick_gap_s)
        stop("control timer stalled", true);
    else if (steady_now - started_at_ >= options_.duration_s)
        stop("trial duration elapsed");
    if (state_ != TrialState::Running) return restingOutput();

    const double dt = steady_now - *last_tick_; // steady 시각 기준 실제 제어 간격.
    last_tick_ = steady_now;
    commanded_steering_ = approach(commanded_steering_, options_.target_steering_rad,
                                   options_.steering_rate_radps, dt);
    if (abs(commanded_steering_ - options_.target_steering_rad) < 1e-9 && !steering_reached_at_)
        steering_reached_at_ = steady_now;
    const bool steering_wait_done = steering_reached_at_ &&
        steady_now - *steering_reached_at_ >= options_.steering_settle_s;
    if (steering_wait_done)
        commanded_speed_ = approach(commanded_speed_,
                                     clamp(options_.target_speed_mps, -stability_speed_cap_, stability_speed_cap_),
                                     min(options_.acceleration_mps2, stability_accel_cap_), dt);
    const auto output = convertTarget(commanded_speed_, commanded_steering_, calibration_);
    if (!output.value) {
        stop(errorMessage(output.error), true);
        return restingOutput();
    }
    return {OutputKind::Drive, output.value->motor_erpm, output.value->servo_position};
}

const char* trialStateName(TrialState state) {
    switch (state) {
        case TrialState::Monitor: return "MONITOR";
        case TrialState::Idle: return "IDLE";
        case TrialState::Running: return "RUNNING";
        case TrialState::Stopped: return "STOPPED_OUTPUT";
        case TrialState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}
} // namespace nomad
