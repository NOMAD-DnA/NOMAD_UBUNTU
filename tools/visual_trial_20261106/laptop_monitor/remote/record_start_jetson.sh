#!/usr/bin/env bash
# 2026-11-15 GUI '센서 녹화' 시작 (Jetson). /mnt/ssd/ws/recording/record.sh 와 같은 세션 형식
# (RECORDING 표시·record.pid·calib/·topics.txt → status.sh·stop_record.sh 그대로 사용) + 선택 시 Pi5 GPS 토픽.
# GUI가 맨 앞에 SCENE=<장면> GPS=0|1 줄을 붙여 보낸다. 센서가 꺼져 있으면 start_sensors.sh로 켜고 표시를 남긴다(정지 때 끔).
set -o pipefail
exec 2>&1
MARK=NOMAD_RECORD_RESULT=
SSD=/mnt/ssd; WS=$SSD/ws/rosbag2_ws; REC=$SSD/ws/recording
result() { printf '%s{"ok":%s,"reason":"%s","session":"%s","sensors_started":%s,"message":"%s"}\n' "$MARK" "$1" "$2" "${3:-}" "${4:-false}" "$(printf '%s' "${5:-}" | tr '"\\\n' "'/ " | cut -c1-240)"; exit 0; }
[[ "$(id -un)" == nvidia && -d "$REC" ]] || result false wrong_board
# 이름 글자 검사는 GUI(sensor_record.SCENE_RE)가 한다. 보드 로케일(C)에서는 한글 범위 정규식이 안 맞으므로 여기서는 경로 안전만 본다.
[[ -n "${SCENE:-}" && "$SCENE" != */* && "$SCENE" != .* && "$SCENE" != *[[:space:]]* && ${#SCENE} -le 160 ]] || result false invalid_scene
findmnt -n "$SSD" >/dev/null || result false no_ssd
[[ -f "$SSD/bags/RECORDING" ]] && result false already_recording "$(cat "$SSD/bags/RECORDING")"
free_gb=$(df -BG --output=avail "$SSD" | tail -1 | tr -dc 0-9); (( free_gb >= 30 )) || result false ssd_full "" false "SSD ${free_gb}GB"
(( $(date -u +%Y) >= 2026 )) || result false clock_year
# 시계: Pi5(chrony 기준)에 맞춰진 뒤에만 시작한다. 녹화 도중 시계가 건너뛰면 센서 시간축이 어긋난다.
tracking=$(chronyc -n tracking 2>/dev/null)
leap=$(sed -n 's/^Leap status *: *//p' <<<"$tracking")
offset=$(sed -n 's/^System time *: *\([0-9.]*\) seconds.*/\1/p' <<<"$tracking")
[[ "$leap" == Normal ]] || result false clock_not_synced "" false "chrony ${leap:-없음}"
awk -v o="${offset:-1}" 'BEGIN{exit !(o < 0.005)}' || result false clock_offset "" false "offset ${offset}s"
[[ -f "$WS/install/setup.bash" ]] || result false rosbag_missing
started=false
if ! pgrep -f '/oak_publisher( |$)' >/dev/null; then
  before=$(cat "$SSD/experiments/sensor_runs/LATEST" 2>/dev/null || true)
  if ! out=$(bash "$REC/start_sensors.sh" 2>&1); then
    # 2026-11-16: start_sensors.sh may fail after starting some sensors. Leave the
    # GUI_SENSORS marker for that new run so '녹화 정지' can stop them.
    now=$(cat "$SSD/experiments/sensor_runs/LATEST" 2>/dev/null || true)
    if [[ -n "$now" && "$now" != "$before" ]]; then
      printf '%s\n' "$now" > "$SSD/bags/GUI_SENSORS"
      result false sensors_failed "" true "$(tail -n 2 <<<"$out")"
    fi
    result false sensors_failed "" false "$(tail -n 2 <<<"$out")"
  fi
  started=true
  cat "$SSD/experiments/sensor_runs/LATEST" > "$SSD/bags/GUI_SENSORS"
fi
for p in oak_publisher ydlidar_ros2_driver_node; do pgrep -f "/$p( |$)" >/dev/null || result false sensors_missing "" "$started" "$p"; done
pgrep -f zed_camera.preview.launch.py >/dev/null || result false sensors_missing "" "$started" zed

ws=/home/nvidia/nomad_ws; root="$ws/tools/visual_trial_20261106"; prefix="$root/runtime-map-preview"
session="$SSD/bags/$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$SCENE"
mkdir -p "$session/calib" "$session/ros_log"
sensor_run=$(cat "$SSD/experiments/sensor_runs/LATEST" 2>/dev/null || true)
for f in "$root/config/oak_basalt_calib.json" "$root/config/factory_eeprom.json" "$root/config/ade20k.preview.json" \
         "$prefix/share/nomad_costmap_cpp/config/oak_g2b_selected_20261030.json" "$prefix/share/nomad_costmap_cpp/config/semantic.real.json" \
         "$prefix/share/nomad_costmap_cpp/config/slope.json" "$prefix/share/nomad_costmap_cpp/config/g2b.yaml" \
         "$ws/config/oak_cpp.yaml" "$ws/config/oak_fastdds_wired.xml" "${sensor_run:+$sensor_run/zed_preview.yaml}"; do
  [[ -n "$f" && -f "$f" ]] && cp -p -- "$f" "$session/calib/"
done
{ echo "scene=$SCENE"; echo "start_utc=$(date -u +%FT%TZ)"; echo "sensor_run=$sensor_run"; echo "host=$(hostname)"
  echo "gps=${GPS:-0}"; echo "started_by=GUI"; echo "chrony_offset_s=$offset"; } > "$session/info.txt"
TOPICS=(
  /oak/left/image_rect /oak/right/image_rect /oak/imu/data /oak/publish_rates
  /oak/color/image/compressed /oak/device_depth/image_rect /oak/device_depth/camera_info
  /oak/depth/points /oak/calibration/factory /oak/cloud/source_status
  /zed/zed_node/left/image_rect_color /zed/zed_node/left/camera_info /nomad_preview/zed/raw_camera_info
  /zed/zed_node/imu/data /zed/zed_node/imu/mag /zed/zed_node/atm_press /zed/zed_node/temperature/imu
  /scan /point_cloud
  /nomad_preview/field12/class_ids /perception/semantic/class_ids
  /tf_static
)
# Pi5 GPS (유선 DDS로 수신). /gps/signal_quality는 Jetson에 메시지 정의(nomad_gnss_msgs)가 없어 빼었다.
[[ "${GPS:-0}" == 1 ]] && TOPICS+=(/gps/fix /gps/quality)
printf '%s\n' "${TOPICS[@]}" > "$session/topics.txt"
set +u
source /opt/ros/humble/install/setup.bash; source "$ws/install/setup.bash"; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ws/config/oak_fastdds_wired.xml" RMW_FASTRTPS_USE_QOS_FROM_XML=0
export ROS_LOG_DIR="$session/ros_log" TMPDIR="$SSD/tmp"
setsid ros2 bag record -s sqlite3 -o "$session/bag" --max-bag-duration 120 --max-cache-size 209715200 \
  "${TOPICS[@]}" </dev/null > "$session/record.log" 2>&1 &
pid=$!
printf '%s\n' "$pid" > "$session/record.pid"
printf '%s\n' "$session" > "$SSD/bags/RECORDING"
sleep 4
if ! kill -0 "$pid" 2>/dev/null; then
  rm -f "$SSD/bags/RECORDING"
  result false recorder_exited "$session" "$started" "$(tail -n 2 "$session/record.log")"
fi
result true recording "$session" "$started"
