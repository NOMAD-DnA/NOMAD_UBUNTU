#include "nomad_control_cpp/raw_trial.hpp"
#include "nomad_control_cpp/control_limits.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace std;

namespace nomad {
namespace {
bool positive(double value) { return isfinite(value) && value > 0.0; }

double approach(double previous, double target, double rate, double dt) {
    return previous + clamp(target - previous, -rate * dt, rate * dt);
}

template <typename... Args>
string format(const char* pattern, Args... args) {
    char text[200];
    snprintf(text, sizeof(text), pattern, args...);
    return text;
}

// 2026-11-15: which commissioning.* value is out of bounds, for the startup error. The constructor's own check stays
// the authority (unchanged); this only names the first failed bound so a bad yaml value is not found by bisecting.
string invalidOptionsDetail(const RawOptions& o) {
    struct Bound { bool bad; const char* name; double value; const char* rule; };
    const Bound bounds[] = {
        {!positive(o.motor_max_erpm) || o.motor_max_erpm > limits::kMotorMaxErpm, "motor_max_erpm", o.motor_max_erpm, "0 < x <= 1850"},
        {!isfinite(o.manual_reverse_max_erpm) || o.manual_reverse_max_erpm < 0.0 || o.manual_reverse_max_erpm > o.motor_max_erpm,
         "manual_reverse_max_erpm", o.manual_reverse_max_erpm, "0 <= x <= motor_max_erpm"},
        {!positive(o.overspeed_erpm) || o.overspeed_erpm > 2500.0 || o.overspeed_erpm <= o.motor_max_erpm,
         "overspeed_erpm", o.overspeed_erpm, "motor_max_erpm < x <= 2500"},
        {!isfinite(o.servo_min) || o.servo_min < 0.0 || o.servo_min >= o.servo_max, "servo_min", o.servo_min, "0 <= x < servo_max"},
        {!isfinite(o.servo_max) || o.servo_max > 1.0 || o.servo_min >= o.servo_max, "servo_max", o.servo_max, "servo_min < x <= 1"},
        {!isfinite(o.servo_center) || o.servo_center < o.servo_min || o.servo_center > o.servo_max,
         "servo_center", o.servo_center, "servo_min <= x <= servo_max"},
        {!positive(o.max_duration_s) || o.max_duration_s > 3.0, "max_duration_s", o.max_duration_s, "0 < x <= 3"},
        {!positive(o.telemetry_timeout_s) || o.telemetry_timeout_s > 0.30, "telemetry_timeout_s", o.telemetry_timeout_s, "0 < x <= 0.30"},
        {!positive(o.manual_timeout_s) || o.manual_timeout_s > 2.0, "manual_timeout_s", o.manual_timeout_s, "0 < x <= 2"},
        {!positive(o.max_tick_gap_s) || o.max_tick_gap_s > 0.20, "max_tick_gap_s", o.max_tick_gap_s, "0 < x <= 0.20"},
        {!positive(o.stationary_hold_s) || o.stationary_hold_s < 0.20, "stationary_hold_s", o.stationary_hold_s, "x >= 0.20"},
        {!positive(o.stationary_erpm) || o.stationary_erpm > 50.0, "stationary_erpm", o.stationary_erpm, "0 < x <= 50"},
        {!positive(o.current_limit_a) || o.current_limit_a > 30.0, "current_limit_a", o.current_limit_a, "0 < x <= 30"},
        {!positive(o.motor_current_limit_a) || o.motor_current_limit_a > 60.0, "motor_current_limit_a", o.motor_current_limit_a, "0 < x <= 60"},
        {!positive(o.overcurrent_hard_a) || o.overcurrent_hard_a <= max(o.current_limit_a, o.motor_current_limit_a) ||
         o.overcurrent_hard_a > 60.0, "overcurrent_hard_a", o.overcurrent_hard_a, "max(current limits) < x <= 60"},
        {!positive(o.overcurrent_sustain_s) || o.overcurrent_sustain_s > 1.0, "overcurrent_sustain_s", o.overcurrent_sustain_s, "0 < x <= 1"},
        {o.overcurrent_spike_limit < 1 || o.overcurrent_spike_limit > 10, "overcurrent_spike_limit",
         static_cast<double>(o.overcurrent_spike_limit), "1 <= x <= 10"},
        {!positive(o.overcurrent_spike_window_s) || o.overcurrent_spike_window_s > 60.0, "overcurrent_spike_window_s",
         o.overcurrent_spike_window_s, "0 < x <= 60"},
        {!positive(o.motor_ramp_erpm_s) || o.motor_ramp_erpm_s > 1200.0, "motor_ramp_erpm_s", o.motor_ramp_erpm_s, "0 < x <= 1200"},
        {!positive(o.motor_deceleration_erpm_s) || o.motor_deceleration_erpm_s > 2400.0, "motor_deceleration_erpm_s",
         o.motor_deceleration_erpm_s, "0 < x <= 2400"},
        {!positive(o.stall_timeout_s) || o.stall_timeout_s < 0.5 || o.stall_timeout_s > 2.0, "stall_timeout_s", o.stall_timeout_s, "0.5 <= x <= 2"},
        {!positive(o.stall_command_erpm) || o.stall_command_erpm > 100.0, "stall_command_erpm", o.stall_command_erpm, "0 < x <= 100"},
        {!positive(o.stall_feedback_erpm) || o.stall_feedback_erpm > o.stall_command_erpm, "stall_feedback_erpm",
         o.stall_feedback_erpm, "0 < x <= stall_command_erpm"},
        {!positive(o.servo_rate_s) || o.servo_rate_s > 0.04, "servo_rate_s", o.servo_rate_s, "0 < x <= 0.04"},
        {!positive(o.manual_servo_rate_s) || o.manual_servo_rate_s > 1.0, "manual_servo_rate_s", o.manual_servo_rate_s, "0 < x <= 1"},
    };
    for (const auto& b : bounds)
        if (b.bad) return format(": commissioning.%s=%g outside %s", b.name, b.value, b.rule);
    return "";
}
} // namespace

// 서보 명령은 프로토콜 범위 0~1 이내입니다. 실제 바퀴 각도나 기계적 끝단을 뜻하지 않습니다.
RawTrialController::RawTrialController(RawOptions options)
    : options_(options), state_(options.enabled ? TrialState::Idle : TrialState::Monitor),
      target_servo_(options.servo_center), commanded_servo_(options.servo_center),
      steering_test_min_(options.servo_min),steering_test_max_(options.servo_max) {
    if (!positive(options_.motor_max_erpm) || options_.motor_max_erpm > limits::kMotorMaxErpm ||
        !isfinite(options_.manual_reverse_max_erpm) || options_.manual_reverse_max_erpm < 0.0 ||
        options_.manual_reverse_max_erpm > options_.motor_max_erpm ||
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
        !positive(options_.motor_current_limit_a) || options_.motor_current_limit_a > 60.0 ||
        !positive(options_.overcurrent_hard_a) ||
        options_.overcurrent_hard_a <= max(options_.current_limit_a, options_.motor_current_limit_a) ||
        options_.overcurrent_hard_a > 60.0 ||
        !positive(options_.overcurrent_sustain_s) || options_.overcurrent_sustain_s > 1.0 ||
        options_.overcurrent_spike_limit < 1 || options_.overcurrent_spike_limit > 10 ||
        !positive(options_.overcurrent_spike_window_s) || options_.overcurrent_spike_window_s > 60.0 ||
        !positive(options_.motor_ramp_erpm_s) || options_.motor_ramp_erpm_s > 1200.0 ||
        !positive(options_.motor_deceleration_erpm_s) || options_.motor_deceleration_erpm_s > 2400.0 ||
        !positive(options_.stall_timeout_s) || options_.stall_timeout_s < 0.5 || options_.stall_timeout_s > 2.0 ||
        !positive(options_.stall_command_erpm) || options_.stall_command_erpm > 100.0 ||
        !positive(options_.stall_feedback_erpm) || options_.stall_feedback_erpm > options_.stall_command_erpm ||
        !positive(options_.servo_rate_s) || options_.servo_rate_s > 0.04 ||
        !positive(options_.manual_servo_rate_s) || options_.manual_servo_rate_s > 1.0)
        throw invalid_argument("invalid raw commissioning limits; outside candidate bounds" + invalidOptionsDetail(options_));
    // 실행 파일 교체 때 마지막 소프트웨어 명령을 이어받습니다. 센서 실측값은 아닙니다.
    if(const char* saved=getenv("NOMAD_INITIAL_SERVO")) {
        char* end=nullptr;const double value=strtod(saved,&end);
        if(end==saved||*end!='\0'||!isfinite(value)||value<options_.servo_min||value>options_.servo_max)
            throw invalid_argument("invalid restored servo command");
        commanded_servo_=target_servo_=value;
    }
}

// Steering 시험은 모터 0을 강제하고 Manual은 두 축을 허용합니다. 모든 숫자와 끝점을 검사합니다.
bool RawTrialController::validTarget(RawAxis axis, double erpm, double servo, bool allow_reverse) const {
    const double low=axis==RawAxis::Steering?steering_test_min_:options_.servo_min;
    const double high=axis==RawAxis::Steering?steering_test_max_:options_.servo_max;
    // 2026-11-15: reverse only for an explicitly allowed (GUI) MANUAL update, bounded by manual_reverse_max_erpm.
    const double min_erpm = allow_reverse && axis == RawAxis::Manual ? -options_.manual_reverse_max_erpm : 0.0;
    if ((axis != RawAxis::Motor && axis != RawAxis::Steering && axis != RawAxis::Manual) ||
        !isfinite(erpm) || erpm < min_erpm || erpm > options_.motor_max_erpm ||
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

// An invalid clock value is a fault. A clock that moved backwards (time sync) stops the
// current output, rebases the references and drops the old sample so fresh telemetry is
// required; headless planning then resumes on its own, manual sessions still latch.
bool RawTrialController::checkClock(double steady_now, double ros_now) {
    if (!isfinite(steady_now) || steady_now < 0.0 || !isfinite(ros_now) || ros_now <= 0.0) {
        invalidateTelemetry("raw clock value invalid");
        return false;
    }
    if ((last_steady_now_ && steady_now < *last_steady_now_) || (last_ros_now_ && ros_now < *last_ros_now_)) {
        last_steady_now_ = steady_now;
        last_ros_now_ = ros_now;
        sample_.reset();
        recent_count_ = recent_next_ = 0; // the ring restarts at slot 0 (recentMaxErpm reads slots 0..count-1)
        invalidateTelemetry("raw clock moved backwards; rebased, waiting for fresh telemetry", true);
        return false;
    }
    last_steady_now_ = steady_now;
    last_ros_now_ = ros_now;
    return true;
}

void RawTrialController::invalidateTelemetry(const string& reason, bool recoverable, bool vesc_fault) {
    telemetry_ok_ = false;
    stationary_since_.reset();
    telemetry_issue_ = reason;
    if (state_ == TrialState::Running) {
        pauseOrLatch(reason, recoverable);
    } else if (planning_recovery_ && !recoverable &&
               (state_ == TrialState::Stopped || state_ == TrialState::Idle)) {
        // Physical faults during a pause, or before headless planning auto-starts, must
        // still latch so a clean sample cannot start the motor without RESET.
        stop(reason, true);
    } else if (vesc_fault && output_owned_ && (state_ == TrialState::Stopped || state_ == TrialState::Idle)) {
        // A VESC fault while we still own the output (e.g. a stopped GUI session holding the brake)
        // latches too, so the node releases with 0A and RESET is required.
        stop(reason, true);
    } else if (state_ == TrialState::Monitor || state_ == TrialState::Idle) reason_ = reason;
    if (vesc_fault && state_ == TrialState::Fault) vesc_fault_ = true;
}

// ERPM과 전류를 원 단위로만 검사합니다. 중복·역순 stamp는 수신 시각을 갱신하지 않습니다.
void RawTrialController::observe(const TelemetrySample& sample, double steady_now, double ros_now) {
    if (!checkClock(steady_now, ros_now)) return;
    const double age = ros_now - sample.stamp_s;
    // Each physical check reports its own reason. Fault code, voltage and overspeed latch;
    // a non-finite sample or a short current spike only releases output.
    if (sample.fault_code != 0) {
        invalidateTelemetry(format("raw VESC fault code %d", sample.fault_code), /*recoverable=*/false, /*vesc_fault=*/true); return;
    }
    if (!isfinite(sample.raw_erpm) || !isfinite(sample.current_motor_a) ||
        !isfinite(sample.current_input_a) || !isfinite(sample.voltage_v)) {
        invalidateTelemetry("raw telemetry value not finite", true); return;
    }
    if (sample.voltage_v <= 0.0) {
        invalidateTelemetry(format("raw voltage invalid: %.2f V", sample.voltage_v)); return;
    }
    if (abs(sample.raw_erpm) > options_.overspeed_erpm) {
        invalidateTelemetry(format("raw overspeed: %.0f ERPM > %.0f ERPM",
                                   sample.raw_erpm, options_.overspeed_erpm)); return;
    }
    if (!checkCurrent(sample, steady_now)) return;
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
    recent_erpm_[recent_next_] = RecentErpm{sample.stamp_s, sample.raw_erpm};
    recent_next_ = (recent_next_ + 1) % kStopRampStartWindow;
    recent_count_ = min(recent_count_ + 1, kStopRampStartWindow);
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
    // 수신 시각이 아니라 추정 발행 시각부터 감시하여 지연 큐가 구동 시간을 늘리지 못하게 합니다.
    // commanded_servo_는 마지막 명령에서 이어집니다. Motor 시작은 서보를 전혀 송신하지 않습니다.
    beginRunning(axis, erpm, servo, duration, steady_now, steady_now - max(0.0, command_age_s));
    reason_ = continuous ? "raw MANUAL continuous control; heartbeat required; angle unmeasured"
                         : axis == RawAxis::Motor ? "raw MOTOR trial; no servo output; calibration unverified"
                                                : "raw STEERING trial; motor released (0A or stop brake/hold); angle unmeasured";
    return true;
}

// update도 먼저 만료를 검사하므로 늦은 heartbeat로 정체나 끝난 시험을 되살릴 수 없습니다.
bool RawTrialController::checkRunning(double steady_now, double ros_now) {
    if (state_ != TrialState::Running) return false;
    if (!healthy(steady_now, ros_now)) invalidateTelemetry("raw telemetry timeout or invalid sample", true);
    else if (axis_ != RawAxis::Manual && steady_now - started_at_ >= duration_s_) stop("raw trial duration elapsed");
    // A stop ramp is the controller's own stop; it no longer needs a fresh command.
    else if (!stop_ramp_ && steady_now - last_manual_at_ > manualTimeout(axis_)) stop("raw GUI command timeout", true);
    else if (timerStalled(steady_now)) pauseOrLatch(timerStallReason(steady_now), true);
    return state_ == TrialState::Running;
}

bool RawTrialController::update(RawAxis axis, double erpm, double servo, double steady_now, double ros_now,
                                double command_age_s, bool allow_reverse) {
    if (!checkClock(steady_now, ros_now) || !checkRunning(steady_now, ros_now)) return false;
    if (axis != axis_ || !validTarget(axis, erpm, servo, allow_reverse) || !validCommandAge(axis, command_age_s)) {
        stop("raw axis switch or invalid target; reset and new session required", true);
        return false;
    }
    target_erpm_ = erpm;
    target_servo_ = servo;
    last_manual_at_ = max(last_manual_at_, steady_now - max(0.0, command_age_s));
    if (abs(erpm) < options_.stall_command_erpm) stalled_since_.reset();
    return true;
}

bool RawTrialController::pausePlanning(const string& reason, double steady_now, double ros_now) {
    if (!planning_recovery_ || state_ == TrialState::Fault) return false;
    if (!checkClock(steady_now, ros_now)) {
        // An invalid clock value faulted. A backwards step was rebased by checkClock and has
        // already moved a running trial to a planning wait; keep that wait instead of failing.
        if (state_ == TrialState::Fault) return false;
        if (state_ == TrialState::Stopped) planning_wait_ = true;
        return true;
    }
    // A stalled control loop is reported as such but, like a missing input, only pauses.
    if (state_ == TrialState::Running && timerStalled(steady_now)) {
        pauseOrLatch(timerStallReason(steady_now), true);
        return true;
    }
    if (state_ == TrialState::Running || state_ == TrialState::Stopped) pauseOrLatch(reason, true);
    return true;
}

// 2026-11-13: LPP pauses stopped the car from ~1000 ERPM within ~50 ms (brake, then handbrake) and it lurched.
// The ramp starts from the measured speed when that is lower (a blocked or slowed car is not pushed on).
bool RawTrialController::beginStopRamp(const string& reason, double steady_now, double ros_now, double ramp_s,
                                       double end_erpm, optional<double> command_end_erpm) {
    if (stop_ramp_) return state_ == TrialState::Running;
    // No side effects when refused: the caller's pausePlanning() then reports any clock, timer or telemetry issue.
    const bool clock_ok = isfinite(steady_now) && isfinite(ros_now) &&
        (!last_steady_now_ || steady_now >= *last_steady_now_) && (!last_ros_now_ || ros_now >= *last_ros_now_);
    if (!planning_recovery_ || state_ != TrialState::Running || axis_ != RawAxis::Manual || !clock_ok ||
        !healthy(steady_now, ros_now) || !last_tick_ || timerStalled(steady_now) ||
        !isfinite(ramp_s) || ramp_s <= 0.0 || ramp_s > kMaxStopRampS || !isfinite(end_erpm) || end_erpm < 0.0)
        return false;
    const double command_end = command_end_erpm.value_or(end_erpm);
    if (command_end_erpm && (!isfinite(command_end) || command_end <= 0.0 || command_end > end_erpm)) return false;
    // 2026-11-20: from the recent samples, not the newest alone (kStopRampStartWindow): a lone HFI low no longer
    // turns a moving car's planning stop into a handbrake at speed.
    const double start = min(commanded_erpm_, max(0.0, recentMaxErpm()));
    if (!(start > end_erpm)) return false;
    commanded_erpm_ = start;
    stop_ramp_ = StopRamp{reason, (start - command_end) / ramp_s, end_erpm, command_end, nullopt, 0u, nullopt};
    stalled_since_.reset();
    reason_ = "speed ramp before planning stop: " + reason;
    return true;
}

double RawTrialController::recentMaxErpm() const {
    if (!sample_) return NAN;
    double highest = sample_->raw_erpm;
    for (int i = 0; i < recent_count_; ++i) {
        const auto& r = recent_erpm_[static_cast<size_t>(i)];
        const double before = sample_->stamp_s - r.stamp_s; // newer than the newest: a slot from before a rebase
        if (before >= -1e-9 && before <= kStopRampStartSpanS + 1e-9) highest = max(highest, r.erpm);
    }
    return highest;
}

void RawTrialController::cancelStopRamp(double steady_now) {
    if (!stop_ramp_) return;
    stop_ramp_.reset();
    // The fresh LPP command that cancels the ramp is the heartbeat; update() follows in the same tick.
    if (isfinite(steady_now)) last_manual_at_ = max(last_manual_at_, steady_now);
    reason_ = "raw planning drive continues: fresh command during the stop ramp";
}

// A short planner pause may still be rolling (coast or brake) while fresh telemetry reports rotation.
// This is not a manual START/reset bypass: its caller owns soft-pause permission.
bool RawTrialController::resumePlanning(double erpm, double servo, double steady_now, double ros_now,
                                        bool exclusive_driver_ready) {
    if (state_ != TrialState::Stopped) return false;
    if (!checkClock(steady_now, ros_now) || !options_.enabled || !options_.setup_acknowledged ||
        !options_.hardware_timeout_verified || !healthy(steady_now, ros_now) ||
        !validTarget(RawAxis::Manual, erpm, servo) || outsideDriveRange() ||
        !exclusive_driver_ready || sample_->raw_erpm < -options_.stationary_erpm) return false;
    // Keep the last servo command; normal tick/update slew limits still apply.
    beginRunning(RawAxis::Manual, erpm, servo, 0.0, steady_now, steady_now);
    // Seed from current measured forward rotation, never a previous target.
    // A lower target cannot inherit a larger command, and zero remains release.
    commanded_erpm_ = clamp(sample_->raw_erpm, 0.0, min(erpm, options_.motor_max_erpm));
    planning_wait_ = false;
    reason_ = "raw planning continuation from fresh measured rotation";
    return true;
}

// Release는 모터 구동 해제입니다(0A, 또는 설정 시 노드가 브레이크/핸드브레이크로 보냄. VESC 고장 코드 잠금 중에는 0A). 정지 시 서보를 중앙으로
// 돌리거나 새 값을 송신하지 않습니다.
void RawTrialController::stop(const string& reason, bool fault) {
    if (state_ == TrialState::Monitor || (state_ == TrialState::Fault && !fault)) return;
    planning_wait_ = false;
    state_ = fault ? TrialState::Fault : TrialState::Stopped;
    reason_ = reason;
    if (reversal_waiting_) reversal_wait_end_ = "stopped";
    clearDriveState();
}

// 11-16: the one place for 'headless planning waits for fresh input, everything else latches'. recoverable = the
// cause may clear by itself (timer, telemetry, driver endpoints, LPP pause); a GUI session still latches.
void RawTrialController::pauseOrLatch(const string& reason, bool recoverable) {
    const bool wait = planning_recovery_ && recoverable;
    stop(reason, !wait);
    planning_wait_ = wait;
}

bool RawTrialController::timerStalled(double steady_now) const {
    return last_tick_ && steady_now - *last_tick_ > options_.max_tick_gap_s;
}

string RawTrialController::timerStallReason(double steady_now) const {
    return format("raw control timer stalled %.2f s", steady_now - *last_tick_);
}

// Everything a drive must not inherit from the previous one (11-15: start, resumePlanning and stop each had their
// own copy of these lines, and the reverse work had to add to all three).
void RawTrialController::clearDriveState() {
    commanded_erpm_ = 0.0;
    target_erpm_ = 0.0;
    last_drive_dir_ = 0;
    reversal_since_.reset();
    reversal_waiting_ = false;
    stalled_since_.reset();
    stop_ramp_.reset();
}

// The common part of start() and resumePlanning(); each sets its own reason and seeds afterwards.
void RawTrialController::beginRunning(RawAxis axis, double erpm, double servo, double duration, double steady_now,
                                      double manual_at) {
    clearDriveState();
    axis_ = axis;
    target_erpm_ = erpm;
    target_servo_ = servo;
    duration_s_ = duration;
    started_at_ = steady_now;
    last_manual_at_ = manual_at;
    last_tick_ = steady_now;
    state_ = TrialState::Running;
    output_owned_ = true;
}

bool RawTrialController::reset(double steady_now, double ros_now) {
    if (!checkClock(steady_now, ros_now) || !options_.enabled ||
        state_ == TrialState::Running || !stationary(steady_now, ros_now)) return false;
    state_ = TrialState::Idle;
    planning_wait_ = false;
    vesc_fault_ = false;
    // 2026-11-15: an accepted RESET (car still) is a fresh start for the current spike rule too; otherwise one more
    // spike within the 10 s window latched again right after RESET.
    overcurrent_spikes_.clear();
    overcurrent_since_.reset();
    reason_ = "raw reset; waiting for explicit axis start; calibration unverified";
    return true;
}

RawOutput RawTrialController::restingOutput() const {
    return {output_owned_ ? OutputKind::Release : OutputKind::None, 0.0, nullopt};
}

bool RawTrialController::checkCurrent(const TelemetrySample& sample, double steady_now) {
    const double motor = abs(sample.current_motor_a), input = abs(sample.current_input_a);
    const double peak = max(motor, input);
    if (motor <= options_.motor_current_limit_a && input <= options_.current_limit_a) {
        overcurrent_since_.reset();
        return true;
    }
    const bool new_episode = !overcurrent_since_;
    if (new_episode) overcurrent_since_ = steady_now;
    while (!overcurrent_spikes_.empty() &&
           steady_now - overcurrent_spikes_.front() > options_.overcurrent_spike_window_s)
        overcurrent_spikes_.pop_front();
    if (new_episode) overcurrent_spikes_.push_back(steady_now);
    const double motor_limit = options_.motor_current_limit_a, input_limit = options_.current_limit_a;
    if (peak > options_.overcurrent_hard_a) {
        invalidateTelemetry(format("raw overcurrent: motor %.1f A, input %.1f A > hard limit %.0f A",
                                   motor, input, options_.overcurrent_hard_a));
    } else if (steady_now - *overcurrent_since_ >= options_.overcurrent_sustain_s) {
        invalidateTelemetry(format("raw overcurrent sustained %.2f s: motor %.1f A (limit %.0f), input %.1f A (limit %.0f)",
                                   steady_now - *overcurrent_since_, motor, motor_limit, input, input_limit));
    } else if (static_cast<int>(overcurrent_spikes_.size()) >= options_.overcurrent_spike_limit) {
        invalidateTelemetry(format("raw current spikes repeated %d times in %.0f s: motor %.1f A (limit %.0f), input %.1f A (limit %.0f)",
                                   static_cast<int>(overcurrent_spikes_.size()), options_.overcurrent_spike_window_s,
                                   motor, motor_limit, input, input_limit));
    } else {
        invalidateTelemetry(format("raw current spike: motor %.1f A (limit %.0f), input %.1f A (limit %.0f); output released",
                                   motor, motor_limit, input, input_limit), true);
    }
    return false;
}

// 명령이 실제로 출발 임계값에 도달한 뒤에만 정체 시간을 셉니다. 회전 확인 시 다시 시작합니다.
bool RawTrialController::checkMotorProgress(double steady_now) {
    const bool stalled = options_.stall_watchdog_enabled && sample_ &&
        abs(target_erpm_) >= options_.stall_command_erpm && abs(commanded_erpm_) >= options_.stall_command_erpm &&
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
        // Competing publishers are latched by the node before this point; a missing driver
        // endpoint only pauses headless planning.
        pauseOrLatch("raw driver endpoints unavailable or competing command publisher", true);
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
        if (stop_ramp_) {
            if (tickStopRamp(steady_now, dt)) return restingOutput();
        } else {
            const int target_dir = updateReversalWait(steady_now);
            rampMotorCommand(target_dir, dt, minimum_drive_erpm);
            if (!checkMotorProgress(steady_now)) return restingOutput();
        }
        if (axis_ == RawAxis::Motor) return {OutputKind::Drive, commanded_erpm_, nullopt};
    }
    const double servo_rate = axis_ == RawAxis::Manual ? options_.manual_servo_rate_s : options_.servo_rate_s;
    commanded_servo_ = approach(commanded_servo_, target_servo_, servo_rate, dt);
    return {OutputKind::Drive, axis_ == RawAxis::Manual ? commanded_erpm_ : 0.0, commanded_servo_};
}

// Planning stop ramp: the running floor and the stall check do not apply. The command stays at command_end_erpm
// (end_erpm unless set) until the measured speed is at or below end_erpm (then the stop output starts with the
// handbrake, not a brake pulse), at most
// kStopRampSettleS; the measured speed may also get there earlier (a slope or an obstacle). "There" means
// kStopRampEndLow of the last kStopRampEndWindow telemetry samples (2026-11-18; 3 in a row on 11-17), so a lone HFI
// ERPM drop does not end the ramp.
// True when it handed over to the planning wait.
bool RawTrialController::tickStopRamp(double steady_now, double dt) {
    commanded_erpm_ = max(stop_ramp_->command_end_erpm, commanded_erpm_ - stop_ramp_->rate_erpm_s * dt);
    if (commanded_erpm_ <= stop_ramp_->command_end_erpm + 1e-9 && !stop_ramp_->at_end_since)
        stop_ramp_->at_end_since = steady_now;
    const bool settled_too_long = stop_ramp_->at_end_since &&
        steady_now - *stop_ramp_->at_end_since >= kStopRampSettleS - 1e-9;
    // The control tick (20 ms) and the telemetry (50 Hz) are not in step: count each new sample once.
    if (!stop_ramp_->counted_stamp || sample_->stamp_s != *stop_ramp_->counted_stamp) {
        stop_ramp_->counted_stamp = sample_->stamp_s;
        const unsigned window = (1u << kStopRampEndWindow) - 1u;
        stop_ramp_->recent_low = ((stop_ramp_->recent_low << 1) |
                                  (sample_->raw_erpm <= stop_ramp_->end_erpm ? 1u : 0u)) & window;
    }
    int low = 0;
    for (unsigned bits = stop_ramp_->recent_low; bits; bits >>= 1) low += static_cast<int>(bits & 1u);
    if (low >= kStopRampEndLow || settled_too_long) {
        const string reason = stop_ramp_->reason;
        pauseOrLatch(reason, true); // a stop ramp exists only with planning recovery: always the wait
        return true;
    }
    return false;
}

// 2026-11-15 (GUI reverse): a reversal is the operator's: the target points against the last drive direction of
// this session (kept across a key release). While the wheel still turns the old way faster than stationary_erpm the
// output is released (stop output brake -> handbrake), at most kReversalWaitMaxS; the wait also ends early when the
// old-way speed rises kReversalLosingErpm above its lowest value since the press (the brake is losing on a steep
// slope; the speed PID brakes harder). A measured roll-back or a one-sample jump of the reported speed with a
// same-direction target keeps driving as before reverse existed (11-15 review: the first version released on the
// measured sign alone, so a forward hill start that rolled back cycled handbrake/drive or stayed on the 20 A brake,
// even with manual_reverse_max_erpm 0). Returns the target direction (-1, 0, +1).
int RawTrialController::updateReversalWait(double steady_now) {
    const int target_dir = target_erpm_ > 0.0 ? 1 : target_erpm_ < 0.0 ? -1 : 0;
    const bool was_waiting = reversal_waiting_;
    reversal_waiting_ = false;
    if (target_dir == 0 || target_dir == last_drive_dir_) {
        reversal_since_.reset();
        if (was_waiting) reversal_wait_end_ = target_dir == 0 ? "released" : "cancelled";
    } else if (last_drive_dir_ != 0 && !reversal_since_) {
        reversal_since_ = steady_now;
        reversal_min_old_erpm_ = numeric_limits<double>::infinity();
    }
    const double measured = sample_ ? sample_->raw_erpm : 0.0;
    if (reversal_since_) {
        const double old_way = -target_dir * measured; // speed still in the old direction
        reversal_min_old_erpm_ = min(reversal_min_old_erpm_, old_way);
        const char* end = old_way <= options_.stationary_erpm ? "still"
            : old_way > reversal_min_old_erpm_ + kReversalLosingErpm ? "brake losing"
            : steady_now - *reversal_since_ >= kReversalWaitMaxS - 1e-9 ? "timeout" : nullptr;
        reversal_waiting_ = end == nullptr;
        if (was_waiting && end) reversal_wait_end_ = end;
    }
    return target_dir;
}

// 0은 감속 목표가 아니라 출력 해제입니다. 낮은 RPM의 PID 구간에 오래 머물지 않습니다. Speeding up = larger magnitude in
// the same direction; a reversal never ramps through zero under drive (it restarts from 0, after the wait above).
void RawTrialController::rampMotorCommand(int target_dir, double dt, double minimum_drive_erpm) {
    if (target_erpm_ * commanded_erpm_ < 0.0) commanded_erpm_ = 0.0;
    const bool speeding_up = abs(target_erpm_) > abs(commanded_erpm_) && target_erpm_ * commanded_erpm_ >= 0.0;
    const double rate = speeding_up ? options_.motor_ramp_erpm_s : options_.motor_deceleration_erpm_s;
    commanded_erpm_ = (target_dir == 0 || reversal_waiting_) ? 0.0 : approach(commanded_erpm_, target_erpm_, rate, dt);
    if (commanded_erpm_ != 0.0) last_drive_dir_ = commanded_erpm_ > 0.0 ? 1 : -1;
    // Skip the low-RPM ramp only for a positive planning target. STOP/zero
    // still releases immediately; overspeed reduction calls with floor=0.
    if (target_erpm_ > 0.0 && minimum_drive_erpm > 0.0)
        commanded_erpm_ = max(commanded_erpm_, minimum_drive_erpm);
}

double RawTrialController::remaining(double steady_now) const {
    if (state_ != TrialState::Running || axis_ == RawAxis::Manual ||
        !isfinite(steady_now) || steady_now < started_at_) return 0.0;
    return max(0.0, duration_s_ - (steady_now - started_at_));
}

} // namespace nomad
