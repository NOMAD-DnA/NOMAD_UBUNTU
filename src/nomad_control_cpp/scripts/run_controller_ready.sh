#!/usr/bin/env bash
set -e
script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/ros_env.sh"
package_prefix=$(ros2 pkg prefix nomad_control_cpp)
package_share=$(ros2 pkg prefix --share nomad_control_cpp)
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
ack=false
if [[ ${NOMAD_SETUP_ACK:-false} == true ]]; then ack=true; fi
exec "$package_prefix/lib/nomad_control_cpp/controller_node" --ros-args --params-file "$package_share/config/commissioning.yaml" --params-file "$package_share/config/planning.yaml" -p commissioning_setup_acknowledged:=$ack -p hardware_timeout_verified:=true -p speed_pid_min_erpm_verified:=true
