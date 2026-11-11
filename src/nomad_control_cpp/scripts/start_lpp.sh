#!/usr/bin/env bash
# One entry point in an already-built team workspace; never starts a second VESC driver.
set -eo pipefail
if ! command -v ros2 >/dev/null; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash;
    else echo 'ROS Humble is missing. Run this inside the team ROS environment.' >&2; exit 1; fi
fi
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# Standard colcon layout: workspace/src/nomad_control_cpp. Preserve a sourced custom workspace.
workspace=$(dirname -- "$(dirname -- "$project")")
if [[ -f "$project/CMakeLists.txt" && $(basename -- "$(dirname -- "$project")") == src ]]; then
    command -v colcon >/dev/null || { echo 'colcon is missing from the team ROS environment.' >&2; exit 1; }
    (cd "$workspace" && colcon build --packages-up-to nomad_control_cpp --cmake-args -DNOMAD_ENABLE_PLANNING=ON)
fi
if [[ -f "$workspace/install/setup.bash" ]]; then source "$workspace/install/setup.bash"; fi
if ! ros2 pkg prefix nomad_control_cpp >/dev/null 2>&1; then
    echo 'Build the team workspace once with colcon build, then rerun this file.' >&2; exit 1
fi
exec ros2 launch nomad_control_cpp lpp.launch.xml "$@"
