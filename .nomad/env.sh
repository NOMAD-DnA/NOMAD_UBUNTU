# Source this file in bash before using this workspace.
export NOMAD_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -d "$NOMAD_WS_ROOT/.nomad/deps/opt/ros/jazzy" ]; then
    export AMENT_PREFIX_PATH="$NOMAD_WS_ROOT/.nomad/deps/opt/ros/jazzy${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="$NOMAD_WS_ROOT/.nomad/deps/opt/ros/jazzy:$NOMAD_WS_ROOT/.nomad/deps/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$NOMAD_WS_ROOT/.nomad/deps/opt/ros/jazzy/lib:$NOMAD_WS_ROOT/.nomad/deps/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export LIBRARY_PATH="$NOMAD_WS_ROOT/.nomad/deps/usr/lib/x86_64-linux-gnu${LIBRARY_PATH:+:$LIBRARY_PATH}"
    export CPATH="$NOMAD_WS_ROOT/.nomad/deps/usr/include${CPATH:+:$CPATH}"
fi
export ROS_DOMAIN_ID="${NOMAD_ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${NOMAD_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="$NOMAD_WS_ROOT/log/ros"
if [ -f "$NOMAD_WS_ROOT/install/mvsim/share/mvsim/local_setup.bash" ]; then
    source "$NOMAD_WS_ROOT/install/local_setup.bash"
fi
