#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${NOMAD_ENABLE_ACTUATION:-}" != 1 ]]; then
  echo 'Set NOMAD_ENABLE_ACTUATION=1 only for a supervised, lifted-wheel trial.' >&2
  exit 1
fi
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  echo 'Set ROS_DOMAIN_ID to the integration domain.' >&2
  exit 1
fi
port=/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00
if [[ ! -e "$port" ]]; then
  echo "VESC USB device missing: $port" >&2
  exit 1
fi
if fuser "$port" >/dev/null 2>&1; then
  echo 'VESC USB is already owned by another process.' >&2
  exit 1
fi
python3 "$root/scripts/configure_vesc_current.py" --port "$port" --require-current-a 30 --require-timeout-ms 300
set +u
source /opt/ros/humble/setup.bash
source /home/user/vesc_ws/install/setup.bash
source "$root/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=0
"/home/user/vesc_ws/install/vesc_driver/lib/vesc_driver/vesc_driver_node" --ros-args --params-file "$root/src/nomad_control/config/real_vesc_driver_trial.yaml" &
driver_pid=$!
cleanup() {
  kill "$control_pid" "$driver_pid" 2>/dev/null || true
  wait "$control_pid" "$driver_pid" 2>/dev/null || true
}
control_pid=''
trap cleanup EXIT INT TERM
sleep 2
if ! kill -0 "$driver_pid" 2>/dev/null; then
  echo 'VESC driver exited before Control started.' >&2
  exit 1
fi
"$root/install/nomad_control/lib/nomad_control/vesc_controller" --ros-args \
  --params-file "$root/src/nomad_control/config/real_vesc.yaml" \
  -p actuation_enabled:=true \
  -p trial_candidate_conversion:=true \
  -p watchdog_verified:=true \
  -p current_limit_verified:=true &
control_pid=$!
wait "$control_pid"
