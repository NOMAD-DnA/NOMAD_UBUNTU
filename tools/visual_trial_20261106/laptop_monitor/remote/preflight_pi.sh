#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
[[ "$(id -un)" = user ]] || { echo 'WRONG_BOARD: expected Pi5 user' >&2; exit 1; }
ws=/home/user/nomad_ws
[[ -r /proc/device-tree/model ]] || { echo 'WRONG_BOARD: Raspberry Pi model unavailable' >&2; exit 1; }
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = *'Raspberry Pi 5'* ]] || { echo "WRONG_BOARD: expected Raspberry Pi 5, got $model" >&2; exit 1; }
for file in /home/user/nomad_ros.sh "$ws/install/setup.bash" \
  "$ws/config/oak_fastdds_wired.xml" "$ws/config/fusion_independent.yaml" \
  "$ws/scripts/run_gnss.sh" "$ws/scripts/run_basalt_cpp.sh" \
  "$ws/scripts/run_fusion.sh" "$ws/tools/visual_trial_20261106/run_monitor.sh"; do
  [[ -r "$file" ]] || { echo "MISSING: $file" >&2; exit 1; }
done
printf 'ROLE=pi5\nMODEL=%s\nWORKSPACE=%s\n' "$model" "$ws"
printf 'MODULES=GNSS,Basalt,VIO-GNSS fusion\n'
printf 'OUTPUT=/nomad/localization/odometry,/nomad/localization/status\n'
