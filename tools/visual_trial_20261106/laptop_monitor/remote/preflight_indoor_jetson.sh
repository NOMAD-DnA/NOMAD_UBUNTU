#!/usr/bin/env bash
set -eo pipefail
exec 2>&1
[[ "$(id -un)" = nvidia ]] || { echo WRONG_BOARD; exit 1; }
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = *Jetson* || "$model" = *Xavier* ]] || { echo WRONG_BOARD; exit 1; }
ws=/home/nvidia/nomad_ws
iw=/home/nvidia/nomad_indoor_ws
prefix="$ws/tools/visual_trial_20261106/runtime-map-preview"
for file in /opt/ros/humble/install/setup.bash "$ws/install/setup.bash" "$ws/config/oak_fastdds_wired.xml" \
  "$ws/scripts/run_oak_cpp.sh" "$iw/active_config/indoor.json" \
  "$prefix/share/nomad_costmap_cpp/config/g2b.yaml"; do
  [[ -r "$file" ]] || { echo "MISSING=$file"; exit 1; }
done
for binary in "$iw/install/nomad_indoor_mapper/lib/nomad_indoor_mapper/nomad_indoor_mapper" \
  "$iw/install/nomad_indoor_mapper/lib/nomad_indoor_mapper/indoor_probe" \
  "$ws/install/nomad_oak_cpp/lib/nomad_oak_cpp/oak_publisher" \
  "$prefix/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node"; do
  [[ -x "$binary" ]] || { echo "MISSING=$binary"; exit 1; }
done
"$iw/install/nomad_indoor_mapper/lib/nomad_indoor_mapper/indoor_probe" "$iw/active_config/indoor.json" </dev/null
display_helper="$ws/tools/visual_trial_20261106/indoor_display_helpers.sh"
[[ -r "$display_helper" ]] || { echo INDOOR_DISPLAY_HELPER_MISSING; exit 1; }
source "$display_helper"
nomad_indoor_display_check || { echo INDOOR_DISPLAY_PREFLIGHT_FAILED; exit 1; }
printf 'ROLE=jetson\nMODEL=%s\nWORKSPACE=%s\nMODULES=VIO_camera_IMU,LiDAR,indoor_mapper,OAK_RGB_display,ZED_indoor5_display\n' "$model" "$iw"
