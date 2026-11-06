#!/bin/bash
set -eo pipefail
exec 9>/tmp/nomad_oak_cpp.lock
flock -n 9 || { printf 'OAK C++ publisher is already running.\n' >&2; exit 1; }
source /home/nvidia/nomad_ws/tools/runtime/module_env.bash
export LD_LIBRARY_PATH="/home/nvidia/sdk/depthai-v2.29.0/lib:/home/nvidia/sdk/depthai-v2.29.0/lib/cmake/depthai/dependencies/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/nomad_ws/tools/visual_trial_20261106/config/oak_wired_udp1400.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
unset ROS_DISCOVERY_SERVER
printf '%s\n' "$$" > /tmp/nomad_oak_cpp.pid
exec /home/nvidia/nomad_ws/install/nomad_oak_cpp/lib/nomad_oak_cpp/oak_publisher --ros-args --params-file /home/nvidia/nomad_ws/config/oak_cpp.yaml "$@"
