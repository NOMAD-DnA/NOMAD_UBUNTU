#!/usr/bin/env bash
# Python is allowed here for ROS interface generation and package metadata.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_ros="${NOMAD_ROS_PREFIX:-/opt/ros/humble/install}"
task_build="${NOMAD_BUILD_DIR:-$task_root/build-jetson}"
task_prefix="${NOMAD_INSTALL_PREFIX:-$task_root/install-jetson}"
task_jobs="${NOMAD_BUILD_JOBS:-2}"
task_runtime="${NOMAD_RUNTIME_DIR:-$task_root/runtime-jetson}"
python3 "$task_root/tools/verify_sources.py"
if [[ ! -f "$task_ros/setup.bash" ]]; then
  echo "ROS C++ underlay is missing: $task_ros" >&2; exit 2
fi
set +u
source "$task_ros/setup.bash"
set -u
cmake -S "$task_root/interfaces" -B "$task_build/interfaces" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_prefix"
cmake --build "$task_build/interfaces" -j "$task_jobs"
cmake --install "$task_build/interfaces"
export AMENT_PREFIX_PATH="$task_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$task_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cmake -S "$task_root" -B "$task_build/costmap" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_prefix"
cmake --build "$task_build/costmap" -j "$task_jobs"
ctest --test-dir "$task_build/costmap" --output-on-failure
cmake --install "$task_build/costmap"
mkdir -p "$task_build/lidar"
tar --no-same-owner -xzf "$task_root/vendor/source.tar.gz" -C "$task_build/lidar"
cmake -S "$task_build/lidar/sdk" -B "$task_build/lidar/sdk-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_prefix" -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE -DSWIG_FOUND=FALSE
cmake --build "$task_build/lidar/sdk-build" -j "$task_jobs"
cmake --install "$task_build/lidar/sdk-build"
cmake -S "$task_root/cmake/lidar_driver" -B "$task_build/lidar/driver-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_prefix" -DCMAKE_PREFIX_PATH="$task_prefix;$task_ros" -DG2_SDK_ROOT="$task_prefix" -DNOMAD_LIDAR_SOURCE="$task_build/lidar/ros_driver"
cmake --build "$task_build/lidar/driver-build" -j "$task_jobs"
cmake --install "$task_build/lidar/driver-build"
# No Python launcher or build-generated setup helper belongs to the deployment.
find "$task_prefix" -type f \( -name '*.py' -o -name '*.pyc' \) -delete
find "$task_prefix" -type d -name __pycache__ -empty -delete
python3 "$task_root/tools/package_runtime.py" "$task_prefix" "$task_runtime"
"$task_runtime/lib/nomad_costmap_cpp/nomad_runtime_manager" --action check --prefix "$task_runtime"
echo "Built native costmap at $task_runtime"
