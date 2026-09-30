# Source this file in bash for the Gazebo Harmonic ROS workspace.
export NOMAD_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset LD_LIBRARY_PATH PYTHONPATH
unset GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${NOMAD_ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${NOMAD_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="$NOMAD_WS_ROOT/log/ros"
export GZ_PARTITION="${NOMAD_GZ_PARTITION:-nomad_gazebo_${ROS_DOMAIN_ID}}"
if [ -f "$NOMAD_WS_ROOT/install/nomad_gazebo/share/nomad_gazebo/local_setup.bash" ]; then
    source "$NOMAD_WS_ROOT/install/local_setup.bash"
fi
