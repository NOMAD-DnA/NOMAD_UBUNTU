#!/usr/bin/env bash
# 2026-11-15 GUI '센서 녹화'용 Pi5 GPS 수신기 켜기. 이미 돌고 있으면(시험 등) 건드리지 않는다.
# start_pi_gnss.sh와 같은 실행(scripts/run_gnss.sh, 도메인 59, 유선 DDS)이지만 시험 포인터(/tmp/nomad_integrated_pi_run_dir)는 쓰지 않는다.
exec 2>&1
MARK=NOMAD_RECORD_RESULT=
[[ "$(id -un)" == user && -f /home/user/nomad_ros.sh ]] || { printf '%s{"ok":false,"reason":"wrong_board"}\n' "$MARK"; exit 0; }
ws=/home/user/nomad_ws
if pgrep -f "$ws/install/nomad_gnss/lib/nomad_gnss/receiver" >/dev/null; then
  printf '%s{"ok":true,"reason":"already_running","started":false}\n' "$MARK"; exit 0
fi
[[ -e /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 ]] || { printf '%s{"ok":false,"reason":"gnss_device_missing"}\n' "$MARK"; exit 0; }
run="$ws/log/record_gnss_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run/ros"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_gnss.sh" </dev/null > "$run/gnss.log" 2>&1 &
echo $! > "$run/gnss.pid"
printf '%s\n' "$run" > /tmp/nomad_record_gnss_run
sleep 3
if kill -0 "$(cat "$run/gnss.pid")" 2>/dev/null; then
  printf '%s{"ok":true,"reason":"started","started":true}\n' "$MARK"
else
  rm -f /tmp/nomad_record_gnss_run
  printf '%s{"ok":false,"reason":"gnss_exited"}\n' "$MARK"
fi
