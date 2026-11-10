#!/usr/bin/env bash
set -eo pipefail
[[ "$(id -un)" = nvidia ]] || exit 2
source /home/nvidia/nomad_ws/tools/runtime/module_env.bash
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/nomad_ws/config/oak_fastdds_wired.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
exec /home/nvidia/nomad_ws/tools/visual_trial_20261106/build_external_cloud/cloud_source_control status
