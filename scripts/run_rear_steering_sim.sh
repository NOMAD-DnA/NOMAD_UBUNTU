#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export NOMAD_ROS_DOMAIN_ID="${NOMAD_ROS_DOMAIN_ID:-152}"
export NOMAD_GZ_PARTITION="${NOMAD_GZ_PARTITION:-nomad_rear_steer_$(hostname)_${NOMAD_ROS_DOMAIN_ID}}"
source "$ROOT/.nomad/env.sh"
exec 9>"/tmp/nomad-rear-steering-${UID}-${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo '같은 도메인의 후륜 조향 시뮬레이션이 이미 실행 중입니다.' >&2; exit 1; }
exec ros2 launch nomad_gazebo rear_steering.launch.py "$@"
