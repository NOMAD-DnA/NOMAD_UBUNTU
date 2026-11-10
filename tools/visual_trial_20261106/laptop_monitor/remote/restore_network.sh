#!/usr/bin/env bash
# Existing direct-cable settings only; no persistent sysctl or default route.
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
[[ "$(id -u)" = 0 ]] || { echo 'Root is required for temporary LAN settings' >&2; exit 1; }
model=$(tr -d '\0' < /proc/device-tree/model)
case "$model" in
  *'Raspberry Pi 5'*) addr=10.77.0.2; peer=10.77.0.1; role=pi5 ;;
  *Jetson*|*Xavier*) addr=10.77.0.1; peer=10.77.0.2; role=jetson ;;
  *) echo "WRONG_BOARD: unsupported model $model" >&2; exit 1 ;;
esac
out="/tmp/nomad_network_resume_$(date +%Y%m%d_%H%M%S)_$$.log"
keys='net.core.rmem_max net.core.wmem_max net.core.rmem_default net.core.wmem_default net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_time net.ipv4.ipfrag_max_dist'
sysctl $keys > "$out"
if ! nmcli -g connection.id connection show nomad-direct-20261105 >/dev/null 2>&1; then
  nmcli connection add save no type ethernet ifname eth0 con-name nomad-direct-20261105 \
    ipv4.method manual ipv4.addresses "$addr/30" ipv4.never-default yes \
    ipv6.method disabled connection.autoconnect no
else
  # Reject a mismatched profile instead of rewriting any existing profile.
  actual=$(nmcli -g ipv4.addresses connection show nomad-direct-20261105)
  [[ "$actual" = "$addr/30" ]] || { echo "WRONG_LAN_PROFILE: $actual" >&2; exit 1; }
fi
nmcli -w 15 connection up nomad-direct-20261105
ethtool -s eth0 speed 100 duplex full autoneg on advertise 0x008 >> "$out" 2>&1 || true
sysctl -w net.core.rmem_max=33554432 net.core.wmem_max=33554432 \
  net.core.rmem_default=4194304 net.core.wmem_default=4194304 \
  net.ipv4.ipfrag_high_thresh=67108864 net.ipv4.ipfrag_time=5 \
  net.ipv4.ipfrag_max_dist=0 >> "$out"
sleep 4
ip -br -4 addr show dev eth0
ethtool eth0 | sed -n '/Speed:/p;/Duplex:/p;/Auto-negotiation:/p;/Link detected:/p'
printf 'NETWORK_CONFIGURED=%s\nNETWORK_LOG=%s\nPEER=%s\n' "$role" "$out" "$peer"
