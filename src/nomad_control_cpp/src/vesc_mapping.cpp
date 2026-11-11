#include "nomad_control_cpp/vesc_mapping.hpp"

#include <cmath>

namespace nomad {

// 계수의 유한값, 환산 부호, 입력·출력 범위의 일관성을 검사합니다.
Error validateCalibration(const Calibration& c) {
    const double numbers[] = {
        c.speed_gain, c.speed_offset, c.steering_gain, c.steering_offset,
        c.speed_min_mps, c.speed_max_mps, c.steering_min_rad, c.steering_max_rad,
        c.servo_min, c.servo_max, c.motor_erpm_min, c.motor_erpm_max
    };
    for (double value : numbers) {
        if (!std::isfinite(value)) return Error::InvalidCalibration;
    }
    if (c.speed_gain <= 0.0 || c.steering_gain == 0.0)
        return Error::InvalidCalibration;
    if ((c.command_erpm_sign != 1 && c.command_erpm_sign != -1) ||
        (c.feedback_erpm_sign != 1 && c.feedback_erpm_sign != -1))
        return Error::InvalidCalibration;
    // 속도 범위는 정지값 0을, 조향 범위는 직진각 0을 포함해야 합니다.
    if (c.speed_min_mps >= c.speed_max_mps || c.speed_min_mps > 0.0 || c.speed_max_mps < 0.0)
        return Error::InvalidCalibration;
    if (c.steering_min_rad >= c.steering_max_rad || c.steering_min_rad > 0.0 || c.steering_max_rad < 0.0)
        return Error::InvalidCalibration;
    if (c.servo_min < 0.0 || c.servo_max > 1.0 || c.servo_min >= c.servo_max ||
        c.steering_offset < c.servo_min || c.steering_offset > c.servo_max)
        return Error::InvalidCalibration;
    if (c.motor_erpm_min >= c.motor_erpm_max || c.motor_erpm_min > 0.0 || c.motor_erpm_max < 0.0)
        return Error::InvalidCalibration;
    return Error::None;
}

// 목표를 장치 단위로 변환합니다. 검사 실패 시 실행 가능한 값을 반환하지 않습니다.
Result<ActuatorCommand> convertTarget(double speed_mps, double steering_rad, const Calibration& c) {
    const Error config_error = validateCalibration(c);
    if (config_error != Error::None) return {std::nullopt, config_error};
    if (!std::isfinite(speed_mps) || !std::isfinite(steering_rad))
        return {std::nullopt, Error::NonFiniteInput};
    if (speed_mps < c.speed_min_mps || speed_mps > c.speed_max_mps)
        return {std::nullopt, Error::SpeedOutOfRange};
    if (steering_rad < c.steering_min_rad || steering_rad > c.steering_max_rad)
        return {std::nullopt, Error::SteeringOutOfRange};

    // 정지 목표에는 offset을 더하지 않습니다. ERPM 0 계산이 물리적 제동을 보장하지는 않습니다.
    const double normalized_erpm = speed_mps == 0.0
        ? 0.0 : c.speed_gain * speed_mps + c.speed_offset;
    const double motor_erpm = c.command_erpm_sign * normalized_erpm; // VESC raw 명령 방향 적용
    const double servo = c.steering_gain * steering_rad + c.steering_offset; // gain에 후륜 부호가 포함되므로 다시 반전하지 않음
    if (!std::isfinite(motor_erpm) || !std::isfinite(servo))
        return {std::nullopt, Error::NonFiniteOutput};
    // 큰 offset이 작은 속도 목표의 방향을 뒤집거나 0으로 만드는 경우를 거부합니다.
    if ((speed_mps > 0.0 && normalized_erpm <= 0.0) ||
        (speed_mps < 0.0 && normalized_erpm >= 0.0))
        return {std::nullopt, Error::DirectionMismatch};
    // 출력을 조용히 clipping하면 LPP가 요청한 궤적과 달라질 수 있으므로 오류를 반환합니다.
    if (motor_erpm < c.motor_erpm_min || motor_erpm > c.motor_erpm_max)
        return {std::nullopt, Error::MotorOutOfRange};
    if (servo < c.servo_min || servo > c.servo_max)
        return {std::nullopt, Error::ServoOutOfRange};
    return {ActuatorCommand{motor_erpm, servo}, Error::None};
}

// 측정 ERPM을 속도로 환산합니다. 측정 최신성은 상위 제어기에서 검사해야 합니다.
Result<double> convertFeedback(double raw_erpm, const Calibration& c) {
    const Error config_error = validateCalibration(c);
    if (config_error != Error::None) return {std::nullopt, config_error};
    if (!std::isfinite(raw_erpm)) return {std::nullopt, Error::NonFiniteInput};
    const double normalized_erpm = c.feedback_erpm_sign * raw_erpm; // 명령 부호와 독립적으로 확인한 측정 부호
    const double speed_mps = (normalized_erpm - c.speed_offset) / c.speed_gain;
    if (!std::isfinite(speed_mps)) return {std::nullopt, Error::NonFiniteOutput};
    // 초과 속도·역방향 움직임을 숨기지 않습니다. 보정 범위 밖 환산의 정확도는 별도 판단합니다.
    return {speed_mps, Error::None};
}

// 변환 오류를 실행 화면과 로그에 표시할 문장으로 바꿉니다.
const char* errorMessage(Error error) {
    switch (error) {
        case Error::None: return "OK";
        case Error::InvalidCalibration: return "Invalid calibration settings";
        case Error::NonFiniteInput: return "Input is NaN or infinity";
        case Error::SpeedOutOfRange: return "Target speed outside calibrated range";
        case Error::SteeringOutOfRange: return "Target steering outside calibrated range";
        case Error::NonFiniteOutput: return "Conversion produced a non-finite result";
        case Error::DirectionMismatch: return "Speed offset produces wrong ERPM direction";
        case Error::MotorOutOfRange: return "Motor ERPM outside output range";
        case Error::ServoOutOfRange: return "Servo position outside output range";
    }
    return "Unknown conversion error";
}

}
