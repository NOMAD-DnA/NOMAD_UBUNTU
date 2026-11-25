#!/usr/bin/env bash
set -euo pipefail
# One planner owner on this board. No sensor/controller starts here.
ws="${NOMAD_PLANNING_WS:-/home/user/nomad_cpp_ws}"
set +u
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RCUTILS_LOGGING_USE_STDOUT=0
lock_root=/tmp
exec 9>"$lock_root/nomad-planning-${UID}.lock"
if ! flock -n 9; then
  printf '%s\n' 'NOMAD planning is already running. Use the existing GUI owner.' >&2
  exit 73
fi
printf '%s\n' "$$" > "$lock_root/nomad-planning-${UID}.pid"
# Keep these wrappers in the source package; config is deployed by colcon.
config="$ws/install/nomad_planning_cpp/share/nomad_planning_cpp/config/planning_real.yaml"
exec "$ws/install/nomad_planning_cpp/lib/nomad_planning_cpp/planner_node" "$@" --ros-args --params-file "$config"
