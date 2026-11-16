#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
# Embedded helper: SSH supplies this script on stdin, with no installed sibling.
nomad_write_planning_output_profile() (
  set -e
  output_file=$1
  wired_local=$2
  wired_peer=$3
  wlan_local=$(ip -4 -o addr show dev wlan0 scope global | awk 'NR == 1 {split($4,a,"/"); print a[1]}')
  planning_peer=${NOMAD_PLANNING_PEER:-}
  if [[ -z "$planning_peer" ]]; then
    if [[ -r /etc/nomad/network-peers.json ]]; then
      # Laptop hotspot networks carry their own fixed Pi4 address; otherwise
      # the classroom (top-level) address. Chosen from this board's wlan0.
      planning_peer=$(python3 -c 'import ipaddress,json,sys
v=json.load(open("/etc/nomad/network-peers.json"))
peer=v.get("pi4")
for n in v.get("networks",[]):
    try:
        if sys.argv[1] and ipaddress.IPv4Address(sys.argv[1]) in ipaddress.ip_network(n["cidr"]):
            peer=n["pi4"]
    except (ValueError,KeyError,TypeError):
        pass
print(ipaddress.IPv4Address(peer))' "$wlan_local") || return 2
    else
      planning_peer=$(getent ahostsv4 lpp.local | awk 'NR==1 {print $1}' || true)
      planning_peer=${planning_peer:-192.168.3.142}
    fi
  fi
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
[[ "$(id -un)" = user && -f /home/user/nomad_ros.sh ]] || { echo "WRONG_BOARD: Pi5 native ROS environment missing" >&2; exit 1; }
# Board-owned time readiness (2026-11-11): the installed checker decides.
if [[ -x /usr/local/lib/nomad-network/time_agent.py ]]; then
  nomad_time=$(python3 /usr/local/lib/nomad-network/time_agent.py --check || true)
  [[ "$nomad_time" == *'"ok": true'* ]] || { printf 'TIME_NOT_READY: %s\n' "${nomad_time#NOMAD_TIME_RESULT=}" >&2; exit 1; }
fi
ws=/home/user/nomad_ws
run=$(cat /tmp/nomad_integrated_pi_run_dir)
source /home/user/nomad_ros.sh
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ws/config/oak_fastdds_wired.xml" RMW_FASTRTPS_USE_QOS_FROM_XML=0
planning_output_profile="$run/planning_output_fastdds.xml"
nomad_write_planning_output_profile "$planning_output_profile" 10.77.0.2 10.77.0.1
sed 's/base_to_imu_extrinsics_confirmed: false/base_to_imu_extrinsics_confirmed: true/' "$ws/config/fusion_independent.yaml" > "$run/fusion_preview.yaml"
cat >> "$run/fusion_preview.yaml" <<'NOTE'
# USER APPROVED VISUAL PREVIEW ONLY: base_link temporarily equals OAK IMU.
# This identity transform is not a measured vehicle mounting calibration.
NOTE
# 2026-11-16: GUI 모듈 세부 설정 (~/nomad_settings/<module>/overrides.yaml, changed keys only).
# Copied into the run folder and added as a later --params-file. Without saved changes nothing is
# copied and the start command is unchanged.
nomad_module_override() {
  local src="$HOME/nomad_settings/$1/overrides.yaml"
  [[ -f "$src" && ! -L "$src" ]] && grep -q 'ros__parameters:' "$src" || return 1
  if ! grep -qxF "$2:" "$src"; then echo "MODULE_SETTINGS invalid=$1"; return 1; fi
  cp -- "$src" "$3" && echo "MODULE_SETTINGS applied=$1"
}
fusion_args=()
if nomad_module_override fusion fusion "$run/fusion_overrides.yaml"; then fusion_args=(--params-file "$run/fusion_overrides.yaml"); fi
# Basalt: whole JSON copy from the GUI. Exported before Basalt and the supervisor so a supervised
# restart uses the same file. Needs run_basalt_cpp.sh to read NOMAD_BASALT_VIO_CONFIG (preflight_pi.sh reports it).
vio_override="$HOME/nomad_settings/vio/basalt_vio_config.json"
if [[ -f "$vio_override" && ! -L "$vio_override" ]] && grep -q '"value0"' "$vio_override"; then
  # Rebuilt from the board's current basalt_vio_config.json each Start: only the GUI keys
  # (vio_config/vio.json) come from the saved copy, so later board edits to other keys are kept.
  if python3 -I - "$ws/config/basalt_vio_config.json" "$vio_override" "$run/basalt_vio_config.json" 2>/dev/null <<'PY'
import json, sys
KEYS = ("optical_flow_detection_grid_size", "optical_flow_max_recovered_dist2", "optical_flow_levels",
        "optical_flow_skip_frames", "optical_flow_epipolar_error", "vio_max_states", "vio_max_kfs",
        "vio_min_frames_after_kf", "vio_new_kf_keypoints_thresh", "vio_obs_std_dev", "vio_obs_huber_thresh",
        "vio_outlier_threshold", "vio_filter_iteration", "vio_max_iterations", "vio_min_triangulation_dist",
        "vio_enforce_realtime", "vio_kf_marg_feature_ratio")
with open(sys.argv[1], encoding="utf-8") as f:
    base = json.load(f)
with open(sys.argv[2], encoding="utf-8") as f:
    saved = json.load(f)["value0"]
for key in KEYS:
    key = "config." + key
    if key in base["value0"] and key in saved:
        base["value0"][key] = saved[key]
with open(sys.argv[3], "w", encoding="utf-8") as f:
    json.dump(base, f, indent=4)
    f.write("\n")
PY
  then
    vio_merged=1
    export NOMAD_BASALT_VIO_CONFIG="$run/basalt_vio_config.json"
  else
    vio_merged=0
    rm -f -- "$run/basalt_vio_config.json"
    echo 'VIO_SETTINGS_IGNORED: saved VIO settings could not be merged with the board basalt_vio_config.json'
  fi
  if [[ "$vio_merged" != 1 ]]; then
    :
  elif grep -q NOMAD_BASALT_VIO_CONFIG "$ws/scripts/run_basalt_cpp.sh"; then
    echo 'MODULE_SETTINGS applied=vio'
  else
    echo 'VIO_SETTINGS_IGNORED: run_basalt_cpp.sh does not read NOMAD_BASALT_VIO_CONFIG'
  fi
fi
setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_basalt_cpp.sh" </dev/null > "$run/vio.log" 2>&1 &
echo $! > "$run/vio.pid"
# 2026-11-12: wait for Basalt's own start messages (max 6 s, as before), not a fixed 6 s.
for ((i=0;i<60;i++)); do
  grep -q 'backend started' "$run/vio.log" 2>/dev/null && grep -q 'loaded calibration' "$run/vio.log" 2>/dev/null && break
  kill -0 "$(cat "$run/vio.pid")" 2>/dev/null || break
  sleep 0.1
done
sleep 0.5
setsid env ROS_LOG_DIR="$run/ros" FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" NOMAD_FUSION_CONFIG="$run/fusion_preview.yaml" bash "$ws/scripts/run_fusion.sh" "${fusion_args[@]}" </dev/null > "$run/fusion.log" 2>&1 &
echo $! > "$run/fusion.pid"
# VIO supervisor (2026-11-11): restarts only the tracked Basalt group after fusion
# reports VIO_FAILED, rate limited. Skipped when the package is not installed.
supervisor="$ws/install/nomad_vio_supervisor/lib/nomad_vio_supervisor/vio_supervisor"
supervisor_params="$ws/install/nomad_vio_supervisor/share/nomad_vio_supervisor/config/vio_supervisor.yaml"
if [[ -x "$supervisor" && -f "$supervisor_params" ]]; then
  setsid env ROS_LOG_DIR="$run/ros" "$supervisor" --ros-args --params-file "$supervisor_params" -p run_dir:="$run" </dev/null > "$run/vio_supervisor.log" 2>&1 &
  echo $! > "$run/vio_supervisor.pid"
fi
# Front video recorder (2026-11-11): saves the OAK color JPEG stream the Jetson sends
# (/oak/color/image/compressed, about 10 Hz) byte-for-byte under $run/front_video.
# Skipped when the package is not installed.
recorder="$ws/install/nomad_front_recorder/lib/nomad_front_recorder/front_recorder"
recorder_params="$ws/install/nomad_front_recorder/share/nomad_front_recorder/config/front_recorder.yaml"
if [[ -x "$recorder" && -f "$recorder_params" ]]; then
  setsid env ROS_LOG_DIR="$run/ros" "$recorder" --ros-args --params-file "$recorder_params" -p run_dir:="$run" </dev/null > "$run/front_recorder.log" 2>&1 &
  echo $! > "$run/front_recorder.pid"
fi
sleep 3
printf 'RUN=%s\n' "$run"
for name in vio fusion; do
 if kill -0 "$(cat "$run/$name.pid")" 2>/dev/null; then printf '%s RUNNING pid=%s\n' "$name" "$(cat "$run/$name.pid")"; else printf '%s EXITED\n' "$name" >&2; tail -n 8 "$run/$name.log" >&2; exit 1; fi
 tail -n 8 "$run/$name.log"
done
if [[ -f "$run/vio_supervisor.pid" ]]; then
 if kill -0 "$(cat "$run/vio_supervisor.pid")" 2>/dev/null; then printf 'vio_supervisor RUNNING pid=%s\n' "$(cat "$run/vio_supervisor.pid")"; else printf 'vio_supervisor EXITED (VIO runs without automatic restart)\n'; tail -n 4 "$run/vio_supervisor.log"; fi
else
 printf 'vio_supervisor NOT_INSTALLED (VIO runs without automatic restart)\n'
fi
if [[ -f "$run/front_recorder.pid" ]]; then
 if kill -0 "$(cat "$run/front_recorder.pid")" 2>/dev/null; then printf 'front_recorder RUNNING pid=%s\n' "$(cat "$run/front_recorder.pid")"; else printf 'front_recorder EXITED (trial runs without front video)\n'; tail -n 4 "$run/front_recorder.log"; fi
else
 printf 'front_recorder NOT_INSTALLED (trial runs without front video)\n'
fi
