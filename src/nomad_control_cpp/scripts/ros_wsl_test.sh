#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
cd /workspace
colcon test --packages-select nomad_control_cpp --event-handlers console_direct+
colcon test-result --verbose
