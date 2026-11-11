#pragma once
#include "nomad_control_cpp/vesc_mapping.hpp"

namespace nomad::config {
// 보정 출처 표시용입니다. true로 바꿔도 구동 허가는 생기지 않습니다.
inline constexpr bool kMeasuredOnVehicle = false;
inline constexpr const char* kCalibrationNote = "EXAMPLE ONLY: no vehicle measurements";

// 가상 예시값입니다. 실측값으로 교체하고 재빌드하세요. Calibration 필드 순서를 유지합니다.
inline constexpr Calibration kCalibration {
    6000.0, // speed_gain: ERPM/(m/s)
    0.0, // speed_offset: 정규화 ERPM
    -0.60, // steering_gain: 서보값/rad, 후륜 방향 포함
    0.52, // steering_offset: 직진 중립
    1, // command_erpm_sign: raw 양수 명령이 전진이라는 가정
    -1, // feedback_erpm_sign: raw 음수 측정이 전진이라는 가정
    0.0, // speed_min_mps: 후진 미보정
    0.40, // speed_max_mps
    -0.20, // steering_min_rad
    0.20, // steering_max_rad
    0.40, // servo_min
    0.64, // servo_max
    -3000.0, // motor_erpm_min: raw ERPM
    3000.0 // motor_erpm_max: raw ERPM
};
}
