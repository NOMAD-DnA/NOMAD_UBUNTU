#!/usr/bin/env bash
# Build source-only trial helpers. This does not start or switch any module.
set -eo pipefail
task_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_trial="$task_ws/tools/visual_trial_20261106"
task_ros="${NOMAD_ROS_PREFIX:-/opt/ros/humble/install}"
task_jobs="${NOMAD_BUILD_JOBS:-2}"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --with-fcn ) ]]; then
  echo 'Usage: build_jetson_trial_tools.sh [--with-fcn]' >&2
  exit 2
fi
[[ "$task_jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'NOMAD_BUILD_JOBS must be positive' >&2; exit 2; }
source "$task_ros/setup.bash"
source "$task_ws/install/local_setup.bash"
for task_pair in external_cloud:build_external_cloud external_frame_alias:build_external_frame_alias monitor_probe:build_monitor_probe; do
  task_source=${task_pair%:*}
  task_build=${task_pair#*:}
  cmake -S "$task_trial/$task_source" -B "$task_trial/$task_build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$task_trial/$task_build" -j "$task_jobs"
done
if [[ ${1:-} == --with-fcn ]]; then
  task_fcn="$task_ws/tools/indoor_fcn_20261106"
  cmake -S "$task_fcn/fcn_runtime" -B "$task_fcn/build_512x512" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$task_fcn/build_512x512" -j "$task_jobs"
fi
