#!/usr/bin/env bash
# Native costmap entrypoint. The manager starts only its existing child modules.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_runtime="${NOMAD_RUNTIME_DIR:-$task_root/runtime-semantic-test}"
task_ros="${NOMAD_ROS_PREFIX:-/opt/ros/humble/install}"
task_binary="$task_runtime/lib/nomad_costmap_cpp/nomad_runtime_manager"
[[ -x "$task_binary" ]] || { echo "Native manager is missing: $task_binary" >&2; exit 2; }
export LD_LIBRARY_PATH="$task_runtime/lib:$task_ros/lib"
export AMENT_PREFIX_PATH="$task_runtime:$task_ros"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
exec "$task_binary" --prefix "$task_runtime" --ros-prefix "$task_ros" "$@"
