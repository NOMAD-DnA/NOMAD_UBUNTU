#!/usr/bin/env bash
# Existing final preview startup. Run on the verified Jetson after LAN setup.
# Reuses existing binaries/private calibration. No GUI, control, or Planning.
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
# Embedded helper: SSH supplies this script on stdin, with no installed sibling.
nomad_write_planning_output_profile() (
  set -e
  output_file=$1
  wired_local=$2
  wired_peer=$3
  planning_peer=${NOMAD_PLANNING_PEER:-}
  if [[ -z "$planning_peer" ]]; then
    if [[ -r /etc/nomad/network-peers.json ]]; then
      planning_peer=$(python3 -c 'import json,ipaddress; print(ipaddress.IPv4Address(json.load(open("/etc/nomad/network-peers.json"))["pi4"]))') || return 2
    else
      planning_peer=$(getent ahostsv4 lpp.local | awk 'NR==1 {print $1}' || true)
      planning_peer=${planning_peer:-192.168.3.142}
    fi
  fi
  wlan_local=$(ip -4 -o addr show dev wlan0 scope global | awk 'NR == 1 {split($4,a,"/"); print a[1]}')
  for address in "$wired_local" "$wired_peer" "$planning_peer" "$wlan_local"; do
    if ! awk -v address="$address" 'BEGIN {
      if (split(address,a,".") != 4) exit 1;
      for (i=1; i<=4; i++) if (a[i] !~ /^[0-9]+$/ || a[i]+0 > 255) exit 1;
    }'; then
      echo 'PLANNING_DDS_ERROR: wired, planning peer and wlan0 addresses must be IPv4' >&2
      exit 2
    fi
  done
  umask 077
  cat > "$output_file" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
 <transport_descriptors><transport_descriptor>
  <transport_id>planning_output_udp</transport_id><type>UDPv4</type>
  <maxInitialPeersRange>32</maxInitialPeersRange>
  <interfaceWhiteList><address>127.0.0.1</address><address>$wired_local</address><address>$wlan_local</address></interfaceWhiteList>
 </transport_descriptor></transport_descriptors>
 <participant profile_name="planning_output_default" is_default_profile="true"><rtps>
  <builtin>
   <metatrafficUnicastLocatorList><locator/></metatrafficUnicastLocatorList>
   <initialPeersList>
    <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
    <locator><udpv4><address>$wired_peer</address></udpv4></locator>
    <locator><udpv4><address>$planning_peer</address></udpv4></locator>
   </initialPeersList>
  </builtin>
  <useBuiltinTransports>false</useBuiltinTransports>
  <userTransports><transport_id>planning_output_udp</transport_id></userTransports>
 </rtps></participant>
</profiles>
XML
)

[[ "$(id -un)" = nvidia ]] || { echo "WRONG_BOARD: Jetson user must be nvidia" >&2; exit 1; }
ws=/home/nvidia/nomad_ws
root="$ws/tools/visual_trial_20261106"
prefix="$root/runtime-map-preview"
launch="$root/launch/zed_camera.preview.launch.py"
run=''

fail() { printf 'RESUME_FAILED: %s\n' "$*" >&2; exit 1; }
trap 'rc=$?; printf "RESUME_FAILED exit=%s run=%s; existing/new processes were not killed\n" "$rc" "${run:-not-created}" >&2' ERR

[[ -d "$root" && -d "$prefix" ]] || fail 'Existing external tools/private runtime missing'
exec 9>/tmp/nomad_resume_jetson_final_preview.lock
flock -n 9 || fail 'Another final-preview startup is active'

# Fail before starting anything if a required module already exists. Match
# executable/module tokens, not grep command lines. Never kill a prior module.
duplicate=0
for cmdline in /proc/[0-9]*/cmdline; do
  [[ -r "$cmdline" ]] || continue
  args=()
  mapfile -d '' -t args < "$cmdline" 2>/dev/null || continue
  [[ ${#args[@]} -gt 0 ]] || continue
  for arg in "${args[@]}"; do
    token=${arg##*/}
    case "$token" in
      oak_publisher|run_oak_cpp.sh|run_zed_sensors.sh|zed_camera.launch.py|zed_camera.preview.launch.py|ydlidar_ros2_driver_node|nomad_external_frame_alias|nomad_semantic_id_adapter|nomad_fcn_preview|external_stereo_cloud|nomad_runtime_manager|nomad_mapper|nomad_monitor|semantic.sh|nomad_perception.modules.semantic)
        printf 'DUPLICATE_MODULE pid=%s token=%s\n' "${cmdline#/proc/}" "$token" >&2
        duplicate=1
        break
        ;;
      component_container|component_container_isolated)
        if [[ " ${args[*]} " == *zed_container* ]]; then
          printf 'DUPLICATE_ZED_CONTAINER pid=%s\n' "${cmdline#/proc/}" >&2
          duplicate=1
          break
        fi
        ;;
    esac
  done
done
[[ "$duplicate" -eq 0 ]] || fail 'Required module already running; parent must inspect/stop it first'

for file in \
  "$launch" \
  "$root/slope_control/slope_control.py" \
  "$ws/scripts/run_oak_cpp.sh" \
  "$ws/tools/runtime/module_env.bash" \
  "$ws/semantic.sh" \
  "$ws/src/nomad_perception/traversability/run_semantic_adapter.sh" \
  "$ws/src/nomad_perception/traversability/run_costmap.sh" \
  "$ws/config/oak_fastdds_wired.xml" \
  "$root/config/ade20k.preview.json" \
  "$root/config/factory_eeprom.json" \
  "$root/config/oak_basalt_calib.json" \
  "$prefix/runtime_manifest.json" \
  "$prefix/share/nomad_costmap_cpp/config/g2b.yaml" \
  "$prefix/share/nomad_costmap_cpp/config/semantic.real.json" \
  "$prefix/share/nomad_costmap_cpp/config/oak_g2b_selected_20261030.json"; do
  [[ -f "$file" ]] || fail "Required existing file missing: $file"
done
for binary in \
  "$root/build_external_frame_alias/nomad_external_frame_alias" \
  "$root/build_external_cloud/external_stereo_cloud" \
  "$root/build_external_cloud/cloud_source_control" \
  "$prefix/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node" \
  "$prefix/lib/nomad_costmap_cpp/nomad_runtime_manager"; do
  [[ -x "$binary" ]] || fail "Existing executable missing: $binary"
done
for binary in "$ws/install/nomad_oak_cpp/lib/nomad_oak_cpp/oak_publisher" "$root/build_external_cloud/external_stereo_cloud"; do
  grep -aFq 'oak-cloud-select-20261110-1' "$binary" || { echo "OAK_CLOUD_FEATURE_MISSING: $binary" >&2; exit 1; }
done
# The external copy must already contain the approved single CameraInfo route.
grep -Fq "('/zed/zed_node/left/camera_info', '/nomad_preview/zed/raw_camera_info')" "$launch" \
  || fail 'Approved CameraInfo route is absent from external launch copy'

# Preserve the previous 2Hz/3.333333Hz YAML; only the new run's RGB publish rate changes.
zed_config=${NOMAD_ZED_PREVIEW_YAML:-}
if [[ -z "$zed_config" && -r /tmp/nomad_integrated_jetson_run_dir ]]; then
  previous_run=$(cat /tmp/nomad_integrated_jetson_run_dir)
  [[ ! -f "$previous_run/zed_preview.yaml" ]] || zed_config="$previous_run/zed_preview.yaml"
fi
if [[ -z "$zed_config" ]]; then
  shopt -s nullglob
  previous_configs=("$ws"/log/visual_trial_20261106_*/zed_preview.yaml)
  shopt -u nullglob
  [[ ${#previous_configs[@]} -eq 0 ]] || zed_config=${previous_configs[${#previous_configs[@]}-1]}
fi
[[ -n "$zed_config" && -f "$zed_config" ]] || fail 'Previous copied ZED YAML missing; set NOMAD_ZED_PREVIEW_YAML explicitly'
awk '/^[[:space:]]*pub_frame_rate[[:space:]]*:/ {rate=$2+0; delta=rate-3.3333333333; if (rate == 2 || (delta > -0.000001 && delta < 0.000001)) ok=1; else bad=1} END {exit (!ok || bad)}' "$zed_config" \
  || fail 'Existing copied ZED YAML must specify RGB pub_frame_rate=2 or approximately 3.333333'

port=${NOMAD_LIDAR_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}
[[ -c "$port" && -r "$port" && -w "$port" ]] || fail "LiDAR port absent/inaccessible: $port"
if command -v fuser >/dev/null 2>&1 && fuser "$port" >/dev/null 2>&1; then
  fail 'LiDAR serial port is already in use'
fi

source /opt/ros/humble/install/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ws/config/oak_fastdds_wired.xml" RMW_FASTRTPS_USE_QOS_FROM_XML=0
export LD_LIBRARY_PATH="$prefix/lib:/opt/ros/humble/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

run="$ws/log/visual_trial_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
mkdir "$run"
mkdir -p "$run/ros" "$run/oak/ros"
planning_output_profile="$run/planning_output_fastdds.xml"
nomad_write_planning_output_profile "$planning_output_profile" 10.77.0.1 10.77.0.2
cp -- "$zed_config" "$run/zed_preview.yaml"
sed -i -E '/^[[:space:]]*pub_frame_rate[[:space:]]*:/ s/:[[:space:]]*[^[:space:]#]+/: 3.3333333333/' "$run/zed_preview.yaml"
printf '%s\n' "$run" > /tmp/nomad_integrated_jetson_run_dir
printf '%s\n' "$run/oak" > /tmp/nomad_oak_cpp_run_dir
printf '%s\n' "$zed_config" > "$run/zed_preview_source"
printf '%s\n' "$prefix" > "$run/private_runtime_path"
printf '%s\n' "$port" > "$run/lidar_port"

start_module() {
  name=$1
  shift
  setsid env ROS_LOG_DIR="$run/ros" "$@" 9>&- </dev/null > "$run/$name.log" 2>&1 &
  printf '%s\n' "$!" > "$run/$name.pid"
}
check_module() {
  name=$1
  pid=$(cat "$run/$name.pid")
  if ! kill -0 "$pid" 2>/dev/null; then
    tail -n 6 "$run/$name.log" >&2 || true
    fail "$name exited; run logs/PIDs preserved for parent cleanup"
  fi
  printf '%s RUNNING pid=%s\n' "$name" "$pid"
}

# Sensor stage: OAK 10Hz/IMU100 (oak_cpp.yaml), copied ZED RGB every 0.3s, native G2B5Hz.
setsid env ROS_LOG_DIR="$run/oak/ros" bash "$ws/scripts/run_oak_cpp.sh" \
  -p enable_device_depth:=true -p device_depth_output_hz:=1.0 \
  -p rectified_calibration_json:="$root/config/oak_basalt_calib.json" \
  -p factory_calibration_json:="$root/config/factory_eeprom.json" 9>&- </dev/null > "$run/oak/oak.log" 2>&1 &
printf '%s\n' "$!" > "$run/oak/oak.pid"
printf '%s\n' "$!" > "$run/oak.pid"
ln -s oak/oak.log "$run/oak.log"
start_module zed bash -c '
  source /home/nvidia/nomad_ws/tools/runtime/module_env.bash
  nomad_run zed ros2 launch /home/nvidia/nomad_ws/tools/visual_trial_20261106/launch/zed_camera.preview.launch.py \
    camera_model:=zed2 camera_name:=zed node_name:=zed_node \
    publish_tf:=false publish_map_tf:=false ros_params_override_path:="$1"
' zed-preview "$run/zed_preview.yaml"
start_module lidar env AMENT_PREFIX_PATH="$prefix:/opt/ros/humble/install" \
  "$prefix/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node" \
  --ros-args --params-file "$prefix/share/nomad_costmap_cpp/config/g2b.yaml" -p "port:=$port"
sleep 8
for name in oak zed lidar; do check_module "$name"; done

# Existing independent inference and external adapters. No SDK/ROS rebuilds.
start_module semantic bash "$ws/semantic.sh" -p rate_hz:=1.0
start_module frame_alias "$root/build_external_frame_alias/nomad_external_frame_alias" --ros-args \
  -p confirmed_camera_serial:=23472639 -p same_left_rectified_optical_axis_confirmed:=true \
  -p input_camera_info_topic:=/nomad_preview/zed/raw_camera_info \
  -p output_camera_info_topic:=/zed/zed_node/left/camera_info
start_module semantic_adapter env NOMAD_SEMANTIC_MAPPING="$root/config/ade20k.preview.json" \
  bash "$ws/src/nomad_perception/traversability/run_semantic_adapter.sh"
start_module external_cloud "$root/build_external_cloud/external_stereo_cloud" --ros-args \
  -p factory_json:="$root/config/factory_eeprom.json" \
  -p vio_calib_json:="$root/config/oak_basalt_calib.json" -p output_hz:=1.0 -p initial_source:=opencv
sleep 3
for name in semantic frame_alias semantic_adapter external_cloud; do check_module "$name"; done

# Existing private native map: preserve mount guess, 2px tolerance and all K.
# LiDAR is already owned above; manager must not start another serial driver.
id="manual_map_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
NOMAD_RUNTIME_DIR="$prefix" ROS_LOG_DIR="$run/ros" FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" \
  bash "$ws/src/nomad_perception/traversability/run_costmap.sh" \
  --action start --run-id "$id" --state-root "$run/costmap_runs" \
  --external-lidar --startup-timeout 900 > "$run/costmap_start.json" 2> "$run/costmap_start.err"
printf '%s\n' "$id" > "$run/costmap_run_id"
sed -n 's/.*"supervisor_pid":[[:space:]]*\([0-9][0-9]*\).*/\1/p' "$run/costmap_start.json" > "$run/costmap.pid"
sleep 1
NOMAD_RUNTIME_DIR="$prefix" FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" \
  bash "$ws/src/nomad_perception/traversability/run_costmap.sh" \
  --action status --run-id "$id" --state-root "$run/costmap_runs" > "$run/costmap_status.json"

printf 'RUN=%s\nLIDAR_PORT=%s\nCOSTMAP_RUN=%s\n' "$run" "$port" "$id"
cat "$run/costmap_status.json"
for name in oak zed lidar semantic frame_alias semantic_adapter external_cloud; do
  printf '%s LOG\n' "$name"
  tail -n 2 "$run/$name.log" || true
done
printf 'STARTED_ONLY: readiness/physical accuracy must be checked separately; no GUI/motor/Planning started\n'
