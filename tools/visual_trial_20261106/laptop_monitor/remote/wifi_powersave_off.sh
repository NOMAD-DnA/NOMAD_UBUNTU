#!/usr/bin/env bash
# 2026-11-16 (user request): turn Wi-Fi power saving off on this board for this boot only.
# Field runs on the laptop hotspot showed planner VEHICLE_STATE_STALE (Pi4 state > 0.3 s late)
# while all three boards had wlan0 power management on. Runtime only: no NetworkManager
# profile or config file is changed, the link is not dropped, a reboot restores the default.
# The GUI runs it at every Start, so it is applied again after a reboot.
exec 2>&1
[[ "$(id -u)" = 0 ]] || { echo 'Root is required to change Wi-Fi power saving'; exit 1; }
iface=$(ls /sys/class/net 2>/dev/null | grep -E '^wl' | head -n 1)
if [[ -z "$iface" ]]; then
  echo 'WIFI_POWERSAVE=none'   # no Wi-Fi device (wired only): nothing to do
  exit 0
fi
state() {
  if command -v iw >/dev/null 2>&1; then
    iw dev "$iface" get power_save 2>/dev/null | awk -F': ' '/Power save/{print $2; exit}'
  elif command -v iwconfig >/dev/null 2>&1; then
    iwconfig "$iface" 2>/dev/null | sed -n 's/.*Power Management:\([a-z]*\).*/\1/p' | head -n 1
  fi
}
before=$(state)
if command -v iw >/dev/null 2>&1; then
  iw dev "$iface" set power_save off || true
fi
if command -v iwconfig >/dev/null 2>&1; then  # Pi4 has iwconfig only
  iwconfig "$iface" power off || true
fi
after=$(state)
printf 'WIFI_IFACE=%s\nWIFI_POWERSAVE_BEFORE=%s\nWIFI_POWERSAVE=%s\n' "$iface" "${before:-unknown}" "${after:-unknown}"
[[ "$after" == off ]]
