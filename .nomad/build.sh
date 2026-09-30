#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd -- "$NOMAD_WS_ROOT"
export CMAKE_BUILD_PARALLEL_LEVEL="${NOMAD_BUILD_JOBS:-4}"
export MAKEFLAGS="-j${NOMAD_BUILD_JOBS:-4}"
colcon build --base-paths src --symlink-install --packages-up-to nomad_sim \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DMVSIM_UNIT_TESTS=OFF -DBUILD_TESTING=OFF
