#!/usr/bin/env bash
set -eo pipefail
REAL_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  printf '%s\n' '먼저 설치한 ROS 환경의 setup.bash를 source하세요.' >&2; exit 2
fi
if [[ ! -f "$REAL_WS_ROOT/install/local_setup.bash" ]]; then
  printf '%s\n' '저장소에서 nomad_interfaces와 nomad_perception을 먼저 빌드하세요.' >&2; exit 2
fi
source "$REAL_WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${NOMAD_ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${NOMAD_DISCOVERY_RANGE:-SUBNET}"
export ROS_LOCALHOST_ONLY=0
exec ros2 launch nomad_perception real_perception.launch.py "$@"
