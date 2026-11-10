#!/usr/bin/env bash
# The function is embedded in the two startup scripts because SSH sends each
# script on stdin; no helper file must be preinstalled on either board.
nomad_write_planning_output_profile() (
  set -e
  output_file=$1
  wired_local=$2
  wired_peer=$3
  planning_peer=${NOMAD_PLANNING_PEER:-192.168.3.142}
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

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  [[ $# -eq 3 ]] || { echo 'Usage: planning_output_profile.sh OUTPUT_XML WIRED_LOCAL WIRED_PEER' >&2; exit 2; }
  nomad_write_planning_output_profile "$@"
fi
