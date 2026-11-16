#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
[[ "$(id -un)" = user && -f /home/user/nomad_ros.sh ]] || { echo "WRONG_BOARD: Pi5 native ROS environment missing" >&2; exit 1; }
ws=/home/user/nomad_ws
run="$ws/log/visual_trial_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run/ros"
printf '%s\n' "$run" > /tmp/nomad_integrated_pi_run_dir
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 2026-11-16: GUI 모듈 세부 설정 (~/nomad_settings/<module>/overrides.yaml, changed keys only).
# Copied into the run folder and added as a later --params-file. Without saved changes nothing is
# copied and the start command is unchanged.
nomad_module_override() {
  local src="$HOME/nomad_settings/$1/overrides.yaml"
  [[ -f "$src" && ! -L "$src" ]] && grep -q 'ros__parameters:' "$src" || return 1
  if ! grep -qxF "$2:" "$src"; then echo "MODULE_SETTINGS invalid=$1"; return 1; fi
  cp -- "$src" "$3" && echo "MODULE_SETTINGS applied=$1"
}
gnss_args=()
if nomad_module_override gnss nomad_gnss "$run/gnss_overrides.yaml"; then gnss_args=(--params-file "$run/gnss_overrides.yaml"); fi
setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_gnss.sh" "${gnss_args[@]}" </dev/null > "$run/gnss.log" 2>&1 &
echo $! > "$run/gnss.pid"
sleep 3
printf 'RUN=%s\n' "$run"
kill -0 "$(cat "$run/gnss.pid")"
tail -n 7 "$run/gnss.log"
