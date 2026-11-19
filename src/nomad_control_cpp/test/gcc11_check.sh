#!/usr/bin/env bash
# 2026-11-19: build this package with g++-11 in Release, as the Pi4 does (Ubuntu 22.04 / GCC 11.4), and fail on any
# compiler warning. The PC has GCC 13, which missed a Pi4-only warning on 11-17 (an aggregate initializer, f7d4382).
#   bash src/nomad_control_cpp/test/gcc11_check.sh            # offline (no ROS) build only
#   source /opt/ros/<distro>/setup.bash; source <ws>/install/setup.bash   # with vesc_msgs and nomad_interfaces
#   bash src/nomad_control_cpp/test/gcc11_check.sh            # then also the ROS build (node, tests)
# Needs g++-11 (Ubuntu 24.04: sudo apt-get install g++-11). Builds in a temporary folder; changes nothing else.
set -euo pipefail
package=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command -v g++-11 >/dev/null || { echo 'g++-11 is missing (sudo apt-get install g++-11)' >&2; exit 2; }
work=$(mktemp -d -t nomad-gcc11-XXXXXX)
trap 'rm -rf "$work"' EXIT
failed=0

echo "== offline build (NOMAD_BUILD_ROS2=OFF, Release, g++-11)"
if CC=gcc-11 CXX=g++-11 cmake -S "$package" -B "$work/offline" -DNOMAD_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release \
       > "$work/offline.log" 2>&1 && cmake --build "$work/offline" -j"$(nproc)" >> "$work/offline.log" 2>&1; then
    warnings=$(grep -c 'warning:' "$work/offline.log" || true)
    echo "   built, warnings: $warnings"
    if [ "$warnings" != 0 ]; then grep -A3 'warning:' "$work/offline.log" | head -40; failed=1; fi
    ctest --test-dir "$work/offline" -L unit --output-on-failure | tail -2 || failed=1
else
    tail -30 "$work/offline.log"; failed=1
fi

if command -v colcon >/dev/null && ros2 pkg prefix vesc_msgs >/dev/null 2>&1; then
    echo "== ROS build (planning on, Release, g++-11; tests built, not run)"
    mkdir -p "$work/ws/src"
    ln -s "$package" "$work/ws/src/nomad_control_cpp"
    if (cd "$work/ws" && CC=gcc-11 CXX=g++-11 colcon build --packages-select nomad_control_cpp --cmake-args \
            -DNOMAD_BUILD_ROS2=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 \
            > "$work/ros.log" 2>&1); then
        warnings=$(grep -c 'warning:' "$work/ros.log" || true)
        echo "   built, warnings: $warnings"
        if [ "$warnings" != 0 ]; then grep -A3 'warning:' "$work/ros.log" | head -40; failed=1; fi
    else
        tail -40 "$work/ros.log"; failed=1
    fi
else
    echo "== ROS build skipped (source ROS and a workspace with vesc_msgs and nomad_interfaces first)"
fi
[ "$failed" = 0 ] && echo "GCC 11 CHECK PASS" || { echo "GCC 11 CHECK FAIL"; exit 1; }
