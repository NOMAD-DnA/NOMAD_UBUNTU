#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
ws=/home/nvidia/nomad_ws
root="$ws/tools/visual_trial_20261106"
run=$(cat /tmp/nomad_integrated_jetson_run_dir)
[[ "$run" = "$ws/log/visual_trial_"* && -d "$run" ]] || { echo 'INVALID_JETSON_RUN' >&2; exit 1; }
for name in oak zed lidar semantic frame_alias semantic_adapter external_cloud; do
  pid=$(cat "$run/$name.pid")
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]] || { echo "EXITED=$name" >&2; exit 1; }
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline")
  [[ "$cmd" = *"$ws/"* ]] || { echo "STALE_PID=$name" >&2; exit 1; }
  printf 'RUNNING=%s pid=%s\n' "$name" "$pid"
done
id=$(cat "$run/costmap_run_id")
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
NOMAD_RUNTIME_DIR="$root/runtime-map-preview" \
  bash "$ws/src/nomad_perception/traversability/run_costmap.sh" \
    --action status --run-id "$id" --state-root "$run/costmap_runs" > "$run/costmap_status.json"
cat "$run/costmap_status.json"
if grep -Eq '"state"[[:space:]]*:[[:space:]]*"(FAILED|STOPPED)"' "$run/costmap_status.json"; then
  echo 'COSTMAP_NOT_RUNNING' >&2
  exit 1
fi
printf 'STARTED_ONLY=jetson\nRUN=%s\n' "$run"
