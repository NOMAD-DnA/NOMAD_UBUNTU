#!/usr/bin/env bash
# 2026-11-15 GUI '센서 녹화' 상태 (Jetson, 읽기만, 1초 안): 녹화 세션·경과·크기·녹화기·센서·SSD 여유.
exec 2>&1
MARK=NOMAD_RECORD_RESULT=
SSD=/mnt/ssd
[[ "$(id -un)" == nvidia ]] || { printf '%s{"ok":false,"reason":"wrong_board"}\n' "$MARK"; exit 0; }
findmnt -n "$SSD" >/dev/null || { printf '%s{"ok":false,"reason":"no_ssd"}\n' "$MARK"; exit 0; }
session=$(cat "$SSD/bags/RECORDING" 2>/dev/null || true)
alive=false elapsed=null size=null gps=0
if [[ -n "$session" ]]; then
  kill -0 "$(cat "$session/record.pid" 2>/dev/null)" 2>/dev/null && alive=true
  start=$(sed -n 's/^start_utc=//p' "$session/info.txt" 2>/dev/null)
  [[ -n "$start" ]] && elapsed=$(( $(date -u +%s) - $(date -u -d "$start" +%s) ))
  size=$(du -sb "$session/bag" 2>/dev/null | cut -f1)
  gps=$(sed -n 's/^gps=//p' "$session/info.txt" 2>/dev/null)
fi
sensors=""
for p in oak_publisher ydlidar_ros2_driver_node external_stereo_cloud; do pgrep -f "/$p( |$)" >/dev/null && sensors+="$p,"; done
pgrep -f zed_camera.preview.launch.py >/dev/null && sensors+="zed,"
free=$(df -BG --output=avail "$SSD" | tail -1 | tr -dc 0-9)
offset=$(chronyc -n tracking 2>/dev/null | sed -n 's/^System time *: *\([0-9.]*\) seconds.*/\1/p')
printf '%s{"ok":true,"recording":%s,"session":"%s","recorder_alive":%s,"elapsed_s":%s,"bytes":%s,"gps":"%s","sensors":"%s","gui_sensors":%s,"ssd_free_gb":%s,"chrony_offset_s":%s}\n' \
  "$MARK" "$([[ -n "$session" ]] && echo true || echo false)" "$session" "$alive" "${elapsed:-null}" "${size:-null}" "${gps:-0}" \
  "${sensors%,}" "$([[ -f "$SSD/bags/GUI_SENSORS" ]] && echo true || echo false)" "${free:-null}" "${offset:-null}"
