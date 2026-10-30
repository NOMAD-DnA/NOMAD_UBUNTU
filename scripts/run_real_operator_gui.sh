#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
else
  echo 'ROS 2 설치를 찾지 못했습니다.' >&2
  exit 1
fi
if [[ ! -f "$root/install/nomad_operator_gui/share/nomad_operator_gui/package.bash" ]]; then
  echo '먼저 colcon build --base-paths src --packages-select nomad_operator_gui --symlink-install 을 실행하세요.' >&2
  exit 1
fi
source "$root/install/local_setup.bash"
set -u
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR="${ROS_LOG_DIR:-$root/log/operator_gui}"
mkdir -p "$ROS_LOG_DIR"
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  echo '먼저 Jetson과 같은 ROS_DOMAIN_ID를 지정하세요. 예: ROS_DOMAIN_ID=42 ./scripts/run_real_operator_gui.sh' >&2
  exit 1
fi
boards_env="${NOMAD_BOARDS_ENV:-$HOME/.config/nomad/boards.env}"
if [[ -f "$boards_env" ]]; then
  set -a
  source "$boards_env"
  set +a
fi
exec ros2 run nomad_operator_gui operator_gui "$@"
