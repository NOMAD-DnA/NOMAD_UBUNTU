#!/usr/bin/env bash
set -e
TASK_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_SKIP_DEPS=false
TASK_SOURCES_ONLY=false
TASK_CACHE_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --skip-deps) TASK_SKIP_DEPS=true; shift ;;
    --sources-only) TASK_SOURCES_ONLY=true; shift ;;
    --cache-root) TASK_CACHE_ARGS=(--cache-root "$2"); shift 2 ;;
    *) echo "Usage: $0 [--skip-deps] [--sources-only] [--cache-root PATH]" >&2; exit 2 ;;
  esac
done
cd "$TASK_WS"
python3 .nomad/vio/import_sources.py "${TASK_CACHE_ARGS[@]}"
if "$TASK_SOURCES_ONLY"; then exit 0; fi
source .nomad/env.sh
if ! "$TASK_SKIP_DEPS"; then
  sudo apt-get update
  sudo apt-get install -y git cmake ninja-build build-essential pkg-config \
    libeigen3-dev libtbb-dev libfmt-dev libopencv-dev libopencv-contrib-dev \
    libceres-dev libboost-filesystem-dev libboost-program-options-dev \
    libboost-system-dev libboost-date-time-dev libglew-dev libglfw3-dev \
    libjpeg-dev libpng-dev liblz4-dev python3-rosdep python3-numpy
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi
  rosdep update
  # Do not scan Basalt's bundled ROS1 internals as independent ROS2 packages.
  rosdep install --from-paths src/nomad_interfaces src/nomad_gazebo \
    src/nomad_vio src/nomad_localization src/nomad_bringup \
    src/nomad_perception src/nomad_control src/nomad_path_planning \
    src/third_party/vio/basalt_ros src/third_party/vio/basalt_vio_msgs \
    src/third_party/vio/open_vins/ov_core src/third_party/vio/open_vins/ov_init \
    src/third_party/vio/open_vins/ov_msckf --ignore-src --skip-keys basalt -r -y
fi
# Sequential packages and bounded compiler jobs (also on Pi5).
export MAKEFLAGS='-j2 -l2'
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --symlink-install --packages-select basalt basalt_vio_msgs basalt_ros \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF -DBUILD_SANDBOX=OFF \
  -DJUST_INSTALL_CEREAL=ON \
  '-DCMAKE_CXX_FLAGS=-include cstdint -Wno-error -Wno-error=dangling-reference'
colcon build --symlink-install --packages-up-to ov_msckf nomad_vio nomad_bringup nomad_gazebo \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0'
echo 'Ready: source .nomad/env.sh; ros2 launch nomad_vio fusion.launch.py method:=basalt manage_vio:=true'
