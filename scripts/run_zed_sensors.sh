#!/usr/bin/env bash
set -eo pipefail
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-59}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
profile=${FASTRTPS_DEFAULT_PROFILES_FILE:-$ws/config/oak_fastdds_wired.xml}
if [ -f "$profile" ]; then export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"; fi
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
# Use the existing SDK instance and ROS camera component for both video and IMU.
source "$ws/tools/runtime/module_env.bash"
nomad_run zed ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2 camera_name:=zed node_name:=zed_node \
  publish_tf:=false publish_map_tf:=false \
  ros_params_override_path:="$ws/config/modules/zed.yaml" "$@"
