#!/usr/bin/env bash
set -eo pipefail
REAL_DRIVER_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  printf '%s\n' '먼저 ROS setup.bash를 source하세요.' >&2; exit 2
fi
real_sdk_args=()
if [[ -n "${YDLIDAR_SDK_ROOT:-}" ]]; then
  [[ -f "$YDLIDAR_SDK_ROOT/lib/libydlidar_sdk.a" ]] || { printf '%s\n' 'SDK 정적 라이브러리를 찾을 수 없습니다.' >&2; exit 2; }
  real_sdk_args+=("-DG2_SDK_ROOT=$YDLIDAR_SDK_ROOT")
fi
cd "$REAL_DRIVER_WS"
colcon build --base-paths .nomad/real_sensors/ydlidar_ros2_driver \
  --packages-select ydlidar_ros2_driver --cmake-args -DCMAKE_BUILD_TYPE=Release "${real_sdk_args[@]}"
