#include "nomad_control_cpp/raw_trial.hpp"
#include "nomad_control_cpp/control_limits.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstdlib>

using namespace std;

namespace nomad {
namespace {
bool positive(double value) { return isfinite(value) && value > 0.0; }

double approach(double previous, double target, double rate, double dt) {
    return previous + clamp(target - previous, -rate * dt, rate * dt);
}
} // namespace

// 서보 명령은 프로토콜 범위 0~1 이내입니다. 실제 바퀴 각도나 기계적 끝단을 뜻하지 않습니다.
RawTrialController::RawTrialController(RawOptions options)
    : options_(options), state_(options.enabled ? TrialState::Idle : TrialState::Monitor),
      target_servo_(options.servo_center), commanded_servo_(options.servo_center),
      steering_test_min_(options.servo_min),steering_test_max_(options.servo_max) {
    if (!positive(options_.motor_max_erpm) || options_.motor_max_erpm > limits::kMotorMaxErpm ||
        !positive(options_.overspeed_erpm) || options_.overspeed_erpm > 2500.0 ||
        options_.overspeed_erpm <= options_.motor_max_erpm ||
        !isfinite(options_.servo_center) || !isfinite(options_.servo_min) ||
        !isfinite(options_.servo_max) || options_.servo_min < 0.0 || options_.servo_max > 1.0 ||
        options_.servo_min >= options_.servo_max || options_.servo_center < options_.servo_min ||
        options_.servo_center > options_.servo_max ||
        !positive(options_.max_duration_s) || options_.max_duration_s > 3.0 ||
        !positive(options_.telemetry_timeout_s) || options_.telemetry_timeout_s > 0.30 ||
        !positive(options_.manual_timeout_s) || options_.manual_timeout_s > 2.0 ||
        !positive(options_.max_tick_gap_s) || options_.max_tick_gap_s > 0.20 ||
        !positive(options_.stationary_hold_s) || options_.stationary_hold_s < 0.20 ||
        !positive(options_.stationary_erpm) || options_.stationary_erpm > 50.0 ||
        !positive(options_.current_limit_a) || options_.current_limit_a > 30.0 ||
        !positive(options_.motor_ramp_erpm_s) || options_.motor_ramp_erpm_s > 1200.0 ||
        !positive(options_.motor_deceleration_erpm_s) || options_.motor_deceleration_erpm_s > 2400.0 ||
        !positive(options_.stall_timeout_s) || options_.stall_timeout_s < 0.5 || options_.stall_timeout_s > 2.0 ||
        !positive(options_.stall_command_erpm) || options_.stall_command_erpm > 100.0 ||
        !positive(options_.stall_feedback_erpm) || options_.stall_feedback_erpm > options_.stall_command_erpm ||
        !positive(options_.servo_rate_s) || options_.servo_rate_s > 0.04 ||
        !positive(options_.manual_servo_rate_s) || options_.manual_servo_rate_s > 0.40)
        throw invalid_argument("invalid raw commissioning limits; outside candidate bounds");
    // 실행 파일 교체 때 마지막 소프트웨어 명령을 이어받습니다. 센서 실측값은 아닙니다.
    if(const char* saved=getenv("NOMAD_INITIAL_SERVO")) {
        char* end=nullptr;const double value=strtod(saved,&end);
        if(end==saved||*end!='\0'||!isfinite(value)||value<options_.servo_min||value>options_.servo_max)
            throw invalid_argument("invalid restored servo command");
        commanded_servo_=target_servo_=value;
    }
}

// Steering 시험은 모터 0을 강제하고 Manual은 두 축을 허용합니다. 모든 숫자와 끝점을 검사합니다.
bool RawTrialController::validTarget(RawAxis axis, double erpm, double servo) const {
    const double low=axis==RawAxis::Steering?steering_test_min_:options_.servo_min;
    const double high=axis==RawAxis::Steering?steering_test_max_:options_.servo_max;
    if ((axis != RawAxis::Motor && axis != RawAxis::Steering && axis != RawAxis::Manual) ||
        !isfinite(erpm) || erpm < 0.0 || erpm > options_.motor_max_erpm ||
        !isfinite(servo) || servo < low || servo > high) return false;
    return axis != RawAxis::Steering || erpm == 0.0;
}

// 한계 설정은 출력하지 않습니다. 현재 위치를 포함하지 않는 축소도 거부해 갑작스러운 복귀를 막습니다.
bool RawTrialController::setSteeringTestSpan(double span,double steady_now,double ros_now) {
    if(!checkClock(steady_now,ros_now)||!options_.enabled||!options_.setup_acknowledged||!options_.hardware_timeout_verified||
       (state_!=TrialState::Idle&&state_!=TrialState::Stopped)||!stationary(steady_now,ros_now)||
       !isfinite(span)||span<.005||span>min(options_.servo_center,1.0-options_.servo_center))return false;
    const double low=options_.servo_center-span,high=options_.servo_center+span;
    if(commanded_servo_<low-1e-9||commanded_servo_>high+1e-9)return false;
    steering_test_min_=low;steering_test_max_=high;
    return true;
}

// Wi-Fi 여유는 연속 MANUAL에만 적용합니다. 짧은 단축 시험의 감시는 350ms 이하로 유지합니다.
double RawTrialController::manualTimeout(RawAxis axis) const {
    return axis == RawAxis::Manual ? options_.manual_timeout_s : min(limits::kShortAxisWatchdogMaxS, options_.manual_timeout_s);
}

bool RawTrialController::validCommandAge(RawAxis axis, double age) const {
    const double max_age = limits::commandMaxAge(axis == RawAxis::Manual, options_.manual_timeout_s);
    return isfinite(age) && age >= -limits::kFutureStampToleranceS && age <= max_age && age <= manualTimeout(axis);
}

// 시계 역행은 이전 메시지를 다시 유효하게 만들 수 있어 현재 시험을 중단합니다.
bool RawTrialController::checkClock(double steady_now, double ros_now) {
    const bool valid = isfinite(steady_now) && steady_now >= 0.0 &&
        isfinite(ros_now) && ros_now > 0.0 &&
        (!last_steady_now_ || steady_now >= *last_steady_now_) &&
        (!last_ros_now_ || ros_now >= *last_ros_now_);
    if (!valid) {
        invalidateTelemetry("raw clock invalid or moved backwards");
        return false;
    }
    last_steady_now_ = steady_now;
    last_ros_now_ = ros_now;
    return true;
}

void RawTrialController::invalidateTelemetry(const string& reason, bool recoverable) {
    telemetry_ok_ = false;
    stationary_since_.reset();
    telemetry_issue_ = reason;
    if (state_ == TrialState::Running) {
        const bool wait = planning_recovery_ && recoverable;
        stop(reason, !wait);
        planning_wait_ = wait;
    } else if (planning_recovery_ && state_ == TrialState::Stopped && !recoverable) {
        stop(reason, true); // Physical faults during a pause must still latch.
    } else if (state_ == TrialState::Monitor || state_ == TrialState::Idle) reason_ = reason;
}

// ERPM과 전류를 원 단위로만 검사합니다. 중복·역순 stamp는 수신 시각을 갱신하지 않습니다.
void RawTrialController::observe(const TelemetrySample& sample, double steady_now, double ros_now) {
    if (!checkClock(steady_now, ros_now)) return;
    const double age = ros_now - sample.stamp_s;
    if (!isfinite(sample.raw_erpm) || abs(sample.raw_erpm) > options_.overspeed_erpm ||
        !isfinite(sample.current_motor_a) || !isfinite(sample.current_input_a) ||
        abs(sample.current_motor_a) > options_.current_limit_a ||
        abs(sample.current_input_a) > options_.current_limit_a ||
        !positive(sample.voltage_v) || sample.fault_code != 0) {
        invalidateTelemetry(sample.fault_code != 0 ? "raw VESC fault code nonzero" :
            "raw physical telemetry invalid, overspeed, current limit or voltage invalid");
        return;
    }
    if (!isfinite(sample.stamp_s) || sample.stamp_s <= 0.0) {
        invalidateTelemetry("raw telemetry stamp invalid"); return;
    }
    if (age < -limits::kFutureStampToleranceS || age > options_.telemetry_timeout_s) {
        invalidateTelemetry(age < -limits::kFutureStampToleranceS ? "raw telemetry stamp in future" : "raw telemetry source age expired", true);
        return;
    }
    if (sample_ && sample.stamp_s <= sample_->stamp_s) return;
    if (!telemetry_ok_ || steady_now - received_at_ > options_.telemetry_timeout_s)
        stationary_since_.reset();
    sample_ = sample;
    received_at_ = steady_now;
    telemetry_ok_ = true;
    telemetry_issue_ = "READY";
    if (abs(sample.raw_erpm) <= options_.stationary_erpm) {
        if (!stationary_since_) stationary_since_ = steady_now;
    } else stationary_since_.reset();
    if (state_ == TrialState::Monitor || state_ == TrialState::Idle)
        reason_ = "raw telemetry received; explicit axis start required; calibration unverified";
}

bool RawTrialController::healthy(double steady_now, double ros_now) const {
    if (!telemetry_ok_ || !sample_ || !isfinite(steady_now) || !isfinite(ros_now)) return false;
    const double receipt_age = steady_now - received_at_;
    const double stamp_age = ros_now - sample_->stamp_s;
    return receipt_age >= 0.0 && receipt_age <= options_.telemetry_timeout_s &&
        stamp_age >= -limits::kFutureStampToleranceS && stamp_age <= options_.telemetry_timeout_s;
}

bool RawTrialController::stationary(double steady_now, double ros_now) const {
    return healthy(steady_now, ros_now) && stationary_since_ &&
        steady_now - *stationary_since_ >= options_.stationary_hold_s;
}

// 부팅·reset만으로 출력하지 않습니다. 실제 보정 완료를 주장하는 플래그도 사용하지 않습니다.
bool RawTrialController::start(RawAxis axis, double erpm, double servo, double duration,
                               double steady_now, double ros_now, bool exclusive_driver_ready,
                               double command_age_s) {
    if (!checkClock(steady_now, ros_now)) return false;
    if (state_ != TrialState::Idle) return false;
    if (!options_.enabled || !options_.setup_acknowledged || !options_.hardware_timeout_verified) {
        reason_ = "raw start refused: commissioning setup or hardware timeout not confirmed";
        return false;
    }
    if(axis!=RawAxis::Steering&&outsideDriveRange()) {
        reason_="return steering inside normal drive range using steering-only mode first";
        return false;
    }
    const bool continuous = axis == RawAxis::Manual;
    const bool valid_duration = continuous ? duration == 0.0
                                          : positive(duration) && duration <= options_.max_duration_s;
    if (!validTarget(axis, erpm, servo) || !validCommandAge(axis, command_age_s) ||
        !valid_duration || (continuous && erpm != 0.0)) {
        reason_ = "raw start refused: invalid axis/target/duration; MANUAL requires 0 ERPM and duration 0";
        return false;
    }
    if (!exclusive_driver_ready || !stationary(steady_now, ros_now)) {
        reason_ = "raw start refused: need exclusive driver and fresh stationary hold";
        return false;
    }
    axis_ = axis;
    target_erpm_ = erpm;
    target_servo_ = servo;
    duration_s_ = duration;
    started_at_ = steady_now;
    // 수신 시각이 아니라 추정 발행 시각부터 감시하여 지연 큐가 구동 시간을 늘리지 못하게 합니다.
    last_manual_at_ = steady_now - max(0.0, command_age_s);
    last_tick_ = steady_now;
    commanded_erpm_ = 0.0;
    stalled_since_.reset();
    // commanded_servo_는 마지막 명령에서 이어집니다. Motor 시작은 서보를 전혀 송신하지 않습니다.
    state_ = TrialState::Running;
    output_owned_ = true;
    reason_ = continuous ? "raw MANUAL continuous control; heartbeat required; angle unmeasured"
                         : axis == RawAxis::Motor ? "raw MOTOR trial; no servo output; calibration unverified"
                                                : "raw STEERING trial; motor released; angle unmeasured";
    return true;
}

// update도 먼저 만료를 검사하므로 늦은 heartbeat로 정체나 끝난 시험을 되살릴 수 없습니다.
bool RawTrialController::checkRunning(double steady_now, double ros_now) {
    if (state_ != TrialState::Running) return false;
    if (!healthy(steady_now, ros_now)) invalidateTelemetry("raw telemetry timeout or invalid sample", true);
    else if (axis_ != RawAxis::Manual && steady_now - started_at_ >= duration_s_) stop("raw trial duration elapsed");
    else if (steady_now - last_manual_at_ > manualTimeout(axis_)) stop("raw GUI command timeout", true);
    else if (last_tick_ && steady_now - *last_tick_ > options_.max_tick_gap_s)
        stop("raw control timer stalled", true);
    return state_ == TrialState::Running;
}

bool RawTrialController::update(RawAxis axis, double erpm, double servo, double steady_now, double ros_now,
                                double command_age_s) {
    if (!checkClock(steady_now, ros_now) || !checkRunning(steady_now, ros_now)) return false;
    if (axis != axis_ || !validTarget(axis, erpm, servo) || !validCommandAge(axis, command_age_s)) {
        stop("raw axis switch or invalid target; reset and new session required", true);
        return false;
    }
    target_erpm_ = erpm;
    target_servo_ = servo;
    last_manual_at_ = max(last_manual_at_, steady_now - max(0.0, command_age_s));
    if (erpm < options_.stall_command_erpm) stalled_since_.reset();
    return true;
}

bool RawTrialController::pausePlanning(const string& reason, double steady_now, double ros_now) {
    if (!planning_recovery_ || !checkClock(steady_now, ros_now) || state_ == TrialState::Fault) return false;
    // Keep a real stalled control loop distinct from a missing module/telemetry sample.
    if (state_ == TrialState::Running && last_tick_ && steady_now - *last_tick_ > options_.max_tick_gap_s) {
        stop("raw control timer stalled", true); return false;
    }
    if (state_ == TrialState::Running || state_ == TrialState::Stopped) {
        stop(reason); planning_wait_ = true;
    }
    return true;
}

// A short planner pause may coast while fresh telemetry still reports rotation.
// This is not a manual START/reset bypass: its caller owns soft-pause permission.
bool RawTrialController::resumePlanning(double erpm, double servo, double steady_now, double ros_now,
                                        bool exclusive_driver_ready) {
    if (state_ != TrialState::Stopped) return false;
    if (!checkClock(steady_now, ros_now) || !options_.enabled || !options_.setup_acknowledged ||
        !options_.hardware_timeout_verified || !healthy(steady_now, ros_now) ||
        !validTarget(RawAxis::Manual, erpm, servo) || outsideDriveRange() ||
        !exclusive_driver_ready || sample_->raw_erpm < -options_.stationary_erpm) return false;
    axis_ = RawAxis::Manual;
    target_erpm_ = erpm;
    target_servo_ = servo;
    duration_s_ = 0.0;
    started_at_ = steady_now;
    last_manual_at_ = steady_now;
    last_tick_ = steady_now;
    // Seed from current measured forward rotation, never a previous target.
    // A lower target cannot inherit a larger command, and zero remains release.
    commanded_erpm_ = clamp(sample_->raw_erpm, 0.0, min(erpm, options_.motor_max_erpm));
    stalled_since_.reset();
    // Keep the last servo command; normal tick/update slew limits still apply.
    state_ = TrialState::Running;
    output_owned_ = true;
    planning_wait_ = false;
    reason_ = "raw planning continuation from fresh measured rotation";
    return true;
}

// Release는 모터 0A 해제입니다. 정지 시 서보를 중앙으로 돌리거나 새 값을 송신하지 않습니다.
void RawTrialController::stop(const string& reason, bool fault) {
    if (state_ == TrialState::Monitor || (state_ == TrialState::Fault && !fault)) return;
    planning_wait_ = false;
    state_ = fault ? TrialState::Fault : TrialState::Stopped;
    reason_ = reason;
    commanded_erpm_ = 0.0;
    target_erpm_ = 0.0;
    stalled_since_.reset();
}

bool RawTrialController::reset(double steady_now, double ros_now) {
    if (!checkClock(steady_now, ros_now) || !options_.enabled ||
        state_ == TrialState::Running || !stationary(steady_now, ros_now)) return false;
    state_ = TrialState::Idle;
    planning_wait_ = false;
    reason_ = "raw reset; waiting for explicit axis start; calibration unverified";
    return true;
}

RawOutput RawTrialController::restingOutput() const {
    return {output_owned_ ? OutputKind::Release : OutputKind::None, 0.0, nullopt};
}

// 명령이 실제로 출발 임계값에 도달한 뒤에만 정체 시간을 셉니다. 회전 확인 시 다시 시작합니다.
bool RawTrialController::checkMotorProgress(double steady_now) {
    const bool stalled = options_.stall_watchdog_enabled && sample_ &&
        target_erpm_ >= options_.stall_command_erpm && commanded_erpm_ >= options_.stall_command_erpm &&
        abs(sample_->raw_erpm) < options_.stall_feedback_erpm;
    if (!stalled) {
        stalled_since_.reset();
        return true;
    }
    if (!stalled_since_) stalled_since_ = steady_now;
    if (steady_now - *stalled_since_ < options_.stall_timeout_s) return true;
    stop("raw motor stalled: requested speed without measured rotation", true);
    return false;
}

// 연속 조작은 두 축을 함께 갱신하되 모터 가속과 서보 변화율을 제한합니다. STOP은 이 경로를 거치지 않습니다.
RawOutput RawTrialController::tick(double steady_now, double ros_now, bool exclusive_driver_ready) {
    return tick(steady_now, ros_now, exclusive_driver_ready, 0.0);
}

RawOutput RawTrialController::tick(double steady_now, double ros_now, bool exclusive_driver_ready,
                                  double minimum_drive_erpm) {
    if (!checkClock(steady_now, ros_now) || !checkRunning(steady_now, ros_now)) return restingOutput();
    if (!exclusive_driver_ready) {
        stop("raw driver disconnected or competing command publisher", true);
        return restingOutput();
    }
    if (!isfinite(minimum_drive_erpm) || minimum_drive_erpm < 0.0 ||
        minimum_drive_erpm > options_.motor_max_erpm) {
        stop("invalid planning minimum drive ERPM", true);
        return restingOutput();
    }
    const double dt = steady_now - *last_tick_;
    last_tick_ = steady_now;
    if (axis_ == RawAxis::Motor || axis_ == RawAxis::Manual) {
        // 0은 감속 목표가 아니라 출력 해제입니다. 낮은 RPM의 PID 구간에 오래 머물지 않습니다.
        const double rate = target_erpm_ < commanded_erpm_ ? options_.motor_deceleration_erpm_s
                                                         : options_.motor_ramp_erpm_s;
        commanded_erpm_ = target_erpm_ == 0.0 ? 0.0 : approach(commanded_erpm_, target_erpm_, rate, dt);
        // Skip the low-RPM ramp only for a positive planning target. STOP/zero
        // still releases immediately; overspeed reduction calls with floor=0.
        if (target_erpm_ > 0.0 && minimum_drive_erpm > 0.0)
            commanded_erpm_ = max(commanded_erpm_, minimum_drive_erpm);
        if (!checkMotorProgress(steady_now)) return restingOutput();
        if (axis_ == RawAxis::Motor) return {OutputKind::Drive, commanded_erpm_, nullopt};
    }
    const double servo_rate = axis_ == RawAxis::Manual ? options_.manual_servo_rate_s : options_.servo_rate_s;
    commanded_servo_ = approach(commanded_servo_, target_servo_, servo_rate, dt);
    return {OutputKind::Drive, axis_ == RawAxis::Manual ? commanded_erpm_ : 0.0, commanded_servo_};
}

double RawTrialController::remaining(double steady_now) const {
    if (state_ != TrialState::Running || axis_ == RawAxis::Manual ||
        !isfinite(steady_now) || steady_now < started_at_) return 0.0;
    return max(0.0, duration_s_ - (steady_now - started_at_));
}

} // namespace nomad
