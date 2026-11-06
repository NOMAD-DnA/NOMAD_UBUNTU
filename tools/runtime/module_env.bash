#!/usr/bin/env bash
set -eo pipefail
NOMAD_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -n ${ROS_DISTRO:-} && $ROS_DISTRO != humble ]]; then
    echo '새 터미널에서 실행하세요: ROS 2 Humble 전용입니다.' >&2
    exit 1
fi
source /opt/ros/humble/install/setup.bash
if [[ ! -f "$NOMAD_WS/install/setup.bash" ]]; then
    echo "빌드가 필요합니다: $NOMAD_WS/install/setup.bash" >&2
    exit 1
fi
source "$NOMAD_WS/install/setup.bash"
export PYTHONPATH="$NOMAD_WS/tools/runtime/python${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export PATH=/usr/bin:/home/nvidia/.local/bin:$PATH

nomad_run() {
    local module=$1
    shift
    exec 9>"${XDG_RUNTIME_DIR:-/tmp}/nomad-${UID}-${module}.lock"
    if ! flock -n 9; then
        echo "$module 모듈이 이미 실행 중입니다." >&2
        exit 1
    fi
    echo "[$module] ROS_DOMAIN_ID=$ROS_DOMAIN_ID — 종료: Ctrl+C"
    exec "$@"
}
