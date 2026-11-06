#!/usr/bin/env bash
# Python is used by build tools only. No sensor or runtime is started here.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_source="$task_root/source"
task_ros="${NOMAD_ROS_PREFIX:-/opt/ros/humble/install}"
task_ws="${NOMAD_WORKSPACE_ROOT:-$(cd -- "$task_root/../../.." && pwd)}"
task_build="${NOMAD_BUILD_DIR:-$task_root/build}"
task_install="${NOMAD_INSTALL_PREFIX:-$task_root/install}"
task_runtime="${NOMAD_RUNTIME_DIR:-$task_root/runtime-semantic-test}"
task_jobs="${NOMAD_BUILD_JOBS:-2}"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check ) ]]; then
  echo 'Usage: ./build.sh [--check]' >&2
  exit 2
fi
if [[ ! "$task_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo 'NOMAD_BUILD_JOBS must be a positive integer' >&2
  exit 2
fi
python3 "$task_root/tools/verify_layout.py"
python3 "$task_source/tools/verify_sources.py"
if [[ ! -f "$task_ros/setup.bash" || ! -f "$task_ws/install/local_setup.bash" ]]; then
  echo 'ROS underlay or existing nomad_ws install is missing' >&2
  exit 2
fi
# The workspace contains the TF2 C++ development packages absent from the underlay.
# Sourcing setup helpers is allowed during this build, never required at runtime.
set +u
source "$task_ros/setup.bash"
source "$task_ws/install/local_setup.bash"
set -u
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 "$task_root/tools/check_build_environment.py" \
  --root "$task_root" --ros-prefix "$task_ros" --workspace "$task_ws" \
  --build "$task_build" --install "$task_install" --runtime "$task_runtime"
if [[ ${1:-} == --check ]]; then
  echo 'Preflight passed. Compilation, runtime packaging and sensor startup were not performed.'
  exit 0
fi
if [[ -d "$task_runtime" && -n "$(find "$task_runtime" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo 'Runtime destination is not empty. Preserve it and choose NOMAD_RUNTIME_DIR inside traversability.' >&2
  exit 2
fi
mkdir -p "$task_build" "$task_install"
exec > >(tee -a "$task_build/build.log") 2>&1
cmake -S "$task_source/interfaces" -B "$task_build/interfaces" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_install"
cmake --build "$task_build/interfaces" -j "$task_jobs"
cmake --install "$task_build/interfaces"
export AMENT_PREFIX_PATH="$task_install${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$task_install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$task_install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cmake -S "$task_source" -B "$task_build/costmap" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_install"
cmake --build "$task_build/costmap" -j "$task_jobs"
(
  cd "$task_build/costmap"
  python3 "$task_root/tools/check_test_catalog.py"
  # CTest 3.16 has no --test-dir option.
  ctest --output-on-failure
)
cmake --install "$task_build/costmap"
cmake -S "$task_root/semantic_adapter" -B "$task_build/semantic_adapter" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_install"
cmake --build "$task_build/semantic_adapter" -j "$task_jobs"
(
  cd "$task_build/semantic_adapter"
  # Explicit tests: CTest must not silently pass with an empty catalog.
  ctest -N | tee "$task_build/semantic_adapter_test_catalog.txt"
  test "$(ctest -N | sed -n 's/Total Tests: //p')" = 2
  ctest --output-on-failure
)
cmake --install "$task_build/semantic_adapter"
mkdir -p "$task_build/lidar"
tar --no-same-owner -xzf "$task_source/vendor/source.tar.gz" -C "$task_build/lidar"
cmake -S "$task_build/lidar/sdk" -B "$task_build/lidar/sdk-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_install" \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE -DSWIG_FOUND=FALSE
cmake --build "$task_build/lidar/sdk-build" -j "$task_jobs"
cmake --install "$task_build/lidar/sdk-build"
cmake -S "$task_source/cmake/lidar_driver" -B "$task_build/lidar/driver-build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_install" \
  -DCMAKE_PREFIX_PATH="$task_install;$task_ros;${CMAKE_PREFIX_PATH:-}" \
  -DG2_SDK_ROOT="$task_install" -DNOMAD_LIDAR_SOURCE="$task_build/lidar/ros_driver"
cmake --build "$task_build/lidar/driver-build" -j "$task_jobs"
cmake --install "$task_build/lidar/driver-build"
python3 "$task_source/tools/package_runtime.py" "$task_install" "$task_runtime"
python3 "$task_root/tools/finalize_runtime.py" \
  --runtime "$task_runtime" --ros-prefix "$task_ros" --workspace "$task_ws" \
  --install "$task_install"
python3 "$task_source/tools/make_manifest.py" "$task_runtime"
LD_LIBRARY_PATH="$task_runtime/lib:$task_ros/lib" \
  "$task_runtime/lib/nomad_costmap_cpp/nomad_runtime_manager" --action check
echo "Built native runtime at $task_runtime"
echo 'No sensor or costmap run was started.'
