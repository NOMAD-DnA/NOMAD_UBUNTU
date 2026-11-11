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
setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_basalt_cpp.sh" </dev/null > "$run/vio.log" 2>&1 &
echo $! > "$run/vio.pid"
sleep 6
setsid env ROS_LOG_DIR="$run/ros" FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" NOMAD_FUSION_CONFIG="$run/fusion_preview.yaml" bash "$ws/scripts/run_fusion.sh" </dev/null > "$run/fusion.log" 2>&1 &
echo $! > "$run/fusion.pid"
sleep 3
printf 'RUN=%s\n' "$run"
for name in vio fusion; do
 if kill -0 "$(cat "$run/$name.pid")" 2>/dev/null; then printf '%s RUNNING pid=%s\n' "$name" "$(cat "$run/$name.pid")"; else printf '%s EXITED\n' "$name" >&2; tail -n 8 "$run/$name.log" >&2; exit 1; fi
 tail -n 8 "$run/$name.log"
done
