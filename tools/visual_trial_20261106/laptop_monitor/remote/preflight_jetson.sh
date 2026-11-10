#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
[[ "$(id -un)" = nvidia ]] || { echo 'WRONG_BOARD: expected Jetson nvidia user' >&2; exit 1; }
ws=/home/nvidia/nomad_ws
root="$ws/tools/visual_trial_20261106"
[[ -r /proc/device-tree/model ]] || { echo 'WRONG_BOARD: Jetson model unavailable' >&2; exit 1; }
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = *Jetson* || "$model" = *Xavier* ]] || { echo "WRONG_BOARD: expected Jetson, got $model" >&2; exit 1; }
for file in /opt/ros/humble/install/setup.bash "$ws/install/setup.bash" \
  "$ws/config/oak_fastdds_wired.xml" "$ws/scripts/run_oak_cpp.sh" \
  "$ws/tools/runtime/module_env.bash" "$ws/semantic.sh" \
  "$root/launch/zed_camera.preview.launch.py" "$root/run_monitor.sh" \
  "$root/runtime-map-preview/runtime_manifest.json"; do
  [[ -r "$file" ]] || { echo "MISSING: $file" >&2; exit 1; }
done
printf 'ROLE=jetson\nMODEL=%s\nWORKSPACE=%s\n' "$model" "$ws"
printf 'MODULES=OAK,ZED,LiDAR,semantic,traversability,costmap\n'
printf 'RUNTIME=%s\n' "$root/runtime-map-preview"
