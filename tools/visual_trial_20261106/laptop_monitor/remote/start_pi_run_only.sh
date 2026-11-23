#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
# GNSS excluded by the operator (2026-11-11): create this trial's Pi5 run
# directory and pointer exactly like start_pi_gnss.sh, but start no GNSS.
[[ "$(id -un)" = user && -f /home/user/nomad_ros.sh ]] || { echo "WRONG_BOARD: Pi5 native ROS environment missing" >&2; exit 1; }
ws=/home/user/nomad_ws
run="$ws/log/visual_trial_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run/ros"
printf '%s\n' "$run" > /tmp/nomad_integrated_pi_run_dir
printf 'GNSS excluded by operator at trial start\n' > "$run/gnss_disabled"
printf 'RUN=%s\n' "$run"
printf 'GNSS=DISABLED\n'
