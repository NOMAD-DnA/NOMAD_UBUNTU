#pragma once
namespace nomad {
// dd.mp4의 순차 촬영한 양쪽 바퀴 평균. 목표 1000 ERPM 기준이며 지면 속도 실측은 아닙니다.
inline constexpr double reference_erpm_per_mps=3721.882617894008;
inline constexpr double referenceSpeedMps(double target_erpm) {
    return target_erpm/reference_erpm_per_mps;
}
}
