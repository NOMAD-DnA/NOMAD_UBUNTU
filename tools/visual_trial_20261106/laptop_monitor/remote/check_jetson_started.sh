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
if [[ "$(cat "$run/mapper_kind" 2>/dev/null)" == mapper2 ]]; then
  # 2026-11-13 new mapper: one process, started by start_jetson.sh with a pid file.
  pid=$(cat "$run/mapper2.pid")
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]] || { tail -n 6 "$run/mapper2.log" >&2 || true; echo 'EXITED=mapper2' >&2; exit 1; }
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline")
  [[ "$cmd" = *nomad_mapper2* ]] || { echo 'STALE_PID=mapper2' >&2; exit 1; }
  printf 'RUNNING=mapper2 pid=%s\nSTARTED_ONLY=jetson\nRUN=%s\n' "$pid" "$run"
  exit 0
fi
if [[ "$(cat "$run/mapper_kind" 2>/dev/null)" == mapper3 ]]; then
  # 2026-11-15 new mapper3 v3: one process (exec'd node, binary on the Jetson SSD) with a pid file.
  pid=$(cat "$run/mapper3.pid")
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]] || { tail -n 6 "$run/mapper3.log" >&2 || true; echo 'EXITED=mapper3' >&2; exit 1; }
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline")
  [[ "$cmd" = */mnt/ssd/nomad/nomad_mapper3_ws/*nomad_mapper3* ]] || { echo 'STALE_PID=mapper3' >&2; exit 1; }
  printf 'RUNNING=mapper3 pid=%s\nSTARTED_ONLY=jetson\nRUN=%s\n' "$pid" "$run"
  exit 0
fi
if [[ "$(cat "$run/mapper_kind" 2>/dev/null)" == terrain ]]; then
  # 2026-11-13 improved legacy mapper: one process with a pid file.
  pid=$(cat "$run/terrain.pid")
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]] || { tail -n 6 "$run/terrain.log" >&2 || true; echo 'EXITED=terrain' >&2; exit 1; }
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline")
  [[ "$cmd" = *nomad_terrain_mapper* ]] || { echo 'STALE_PID=terrain' >&2; exit 1; }
  printf 'RUNNING=terrain pid=%s\nSTARTED_ONLY=jetson\nRUN=%s\n' "$pid" "$run"
  exit 0
fi
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
