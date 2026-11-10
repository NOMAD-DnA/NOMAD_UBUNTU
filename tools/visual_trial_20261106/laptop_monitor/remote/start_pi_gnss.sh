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
setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_gnss.sh" </dev/null > "$run/gnss.log" 2>&1 &
echo $! > "$run/gnss.pid"
sleep 3
printf 'RUN=%s\n' "$run"
kill -0 "$(cat "$run/gnss.pid")"
tail -n 7 "$run/gnss.log"
