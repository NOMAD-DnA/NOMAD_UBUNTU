#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = *'Raspberry Pi 5'* ]] || { echo 'WRONG_BOARD: expected Pi5' >&2; exit 1; }
ping -c 2 -W 2 10.77.0.1
printf 'LAN_PEER_REACHABLE=pi5-to-jetson\n'
