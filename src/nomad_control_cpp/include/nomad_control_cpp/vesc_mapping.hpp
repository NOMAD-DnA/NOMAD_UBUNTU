#pragma once

#include <optional>

namespace nomad {

// 실측 환산 계수와 검증된 명령 범위. 실제 구동 허가와는 별개입니다.
struct Calibration {
    double speed_gain; // ERPM/(m/s), 새 전방 기준 양수
    double speed_offset; // 정규화 ERPM 영점
    double steering_gain; // 서보값/rad, 후륜 조향 방향 부호 포함
    double steering_offset; // 직진 서보 중립
    int command_erpm_sign; // 정규화 ERPM → raw 명령, ±1
    int feedback_erpm_sign; // raw 측정 → 정규화 ERPM, ±1
    // 보정·검증한 속도(m/s)와 등가 조향각(rad)의 사용 범위입니다.
    double speed_min_mps;
    double speed_max_mps;
    double steering_min_rad;
    double steering_max_rad;
    // 서보 범위는 0~1 안에, 모터 출력 범위는 raw ERPM 기준으로 설정합니다.
    double servo_min;
    double servo_max;
    double motor_erpm_min;
    double motor_erpm_max;
};

// 장치에 보내기 전의 계산 결과입니다. 이 구조체는 통신을 수행하지 않습니다.
struct ActuatorCommand {
    double motor_erpm; // raw 목표 ERPM
    double servo_position; // 정규화 서보 명령, 실제 타이어 각도가 아님
};

enum class Error {
    None,
    InvalidCalibration,
    NonFiniteInput,
    SpeedOutOfRange,
    SteeringOutOfRange,
    NonFiniteOutput,
    DirectionMismatch,
    MotorOutOfRange,
    ServoOutOfRange
};

// 실패 시 value가 비어 있고 error에 원인이 들어갑니다. 접근 전에 value를 확인하세요.
template <typename T>
struct Result {
    std::optional<T> value;
    Error error;
};

// 보정 설정의 수치·부호·범위 검증. 실차 보정 완료 여부를 판정하지는 않습니다.
Error validateCalibration(const Calibration& cfg);
// m/s·등가각 rad를 raw ERPM·서보값으로 변환하며, 범위 밖 명령은 거부합니다.
Result<ActuatorCommand> convertTarget(double speed_mps, double steering_rad,
                                     const Calibration& cfg);
// raw ERPM 측정을 m/s로 환산합니다. 측정값은 명령 범위로 clipping하지 않습니다.
Result<double> convertFeedback(double raw_erpm, const Calibration& cfg);
// 로그에 표시할 오류 설명을 반환합니다.
const char* errorMessage(Error error);

}
