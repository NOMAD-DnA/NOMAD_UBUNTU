#!/usr/bin/env bash
# Run on Jetson with this shared source folder available. GUI collector only.
# This does not start/restart the mapper, sensors, monitor or vehicle controls.
set -eo pipefail
findmnt /mnt/ssd >/dev/null || { echo 'SSD /mnt/ssd is required' >&2; exit 2; }
src=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
build=/mnt/ssd/nomad/gui_slope_preview_20261116/build
export TMPDIR=/mnt/ssd/tmp
mkdir -p "$TMPDIR"
source /opt/ros/humble/install/setup.bash
source "$HOME/nomad_ws/install/setup.bash"
cmake -S "$src" -B "$build" -DCMAKE_BUILD_TYPE=Release -DNOMAD_SLOPE_PREVIEW_TEST=ON
cmake --build "$build" -j1
"$build/nomad_slope_preview_test"
target="$HOME/nomad_ws/tools/visual_trial_20261106/build_monitor_probe/nomad_monitor_probe"
cp "$build/nomad_monitor_probe" "$target.slope-preview-new"
chmod 755 "$target.slope-preview-new"
mv -f "$target.slope-preview-new" "$target"
sha256sum "$target"
