#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd -- "$NOMAD_WS_ROOT"
export CMAKE_BUILD_PARALLEL_LEVEL="${NOMAD_BUILD_JOBS:-4}"
export MAKEFLAGS="-j${NOMAD_BUILD_JOBS:-4}"
colcon build --base-paths src --symlink-install --packages-select nomad_gazebo \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
