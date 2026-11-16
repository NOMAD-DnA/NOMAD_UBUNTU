#!/usr/bin/env bash
# 2026-11-12 "Jetson planner" mode: Pi4 runs only the VESC driver and the vehicle
# controller; the planner runs on the Jetson and sends /nomad/planning/drive_command.
# The GUI sends this text over SSH (bash -c); nothing is installed or changed on Pi4.
# It holds the same Pi4 planning/output/VESC locks as the Pi4 launchers, so the Pi4
# planner cannot start at the same time. It never takes over a running vehicle
# process. SSH closing (stdin EOF) stops the controller and driver.
# Usage: pi4_output_only.sh rpm|vio_duty
set -eo pipefail
mode="${1:-rpm}"
ws=/home/user/nomad_cpp_ws
source /opt/ros/humble/setup.bash
source /home/user/vesc_ws/install/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export RCUTILS_LOGGING_USE_STDOUT=0

exec 9>"/tmp/nomad-planning-${UID}.lock"
flock -n 9 || { echo 'Pi4 planner session already running; close it before the Jetson planner mode.' >&2; exit 73; }
exec 8>"/tmp/nomad-output-${UID}.lock"
flock -n 8 || { echo 'Vehicle output session already owned.' >&2; exit 73; }
exec 7>/home/user/vesc_ws/device.lock
flock -n 7 || { echo 'VESC is owned by another app. Close that app before reconnecting.' >&2; exit 73; }

driver=/home/user/vesc_ws/install/vesc_driver/lib/vesc_driver/vesc_driver_node
control_config="$ws/install/nomad_control_cpp/share/nomad_control_cpp/config"
driver_config="$ws/commission_session/vesc_commission.yaml"
case "$mode" in
  rpm)
    controller="$ws/install/nomad_control_cpp/lib/nomad_control_cpp/controller_node"
    extra_driver=()
    controller_args=(--params-file "$control_config/commissioning.yaml" --params-file "$control_config/planning.yaml"
                     --params-file "$control_config/planning_headless.yaml" --params-file "$control_config/nomad_vehicle_ready.yaml")
    ;;
  vio_duty)
    controller="$ws/install/nomad_vio_duty_control/lib/nomad_vio_duty_control/duty_controller_node"
    duty_share="$ws/install/nomad_vio_duty_control/share/nomad_vio_duty_control/config"
    extra_driver=(--params-file "$duty_share/driver_duty.yaml")
    controller_args=(-r __node:=nomad_vesc_trial --params-file "$duty_share/vio_duty.yaml")
    ;;
  *) echo "Unknown motor control mode: $mode (rpm or vio_duty)." >&2; exit 2 ;;
esac
for file in "$controller" "$driver" "$driver_config" "$control_config/vesc_steering_test.yaml"; do
  test -r "$file" || { echo "Missing required output file: $file" >&2; exit 2; }
done
# Never signal an existing process: the operator must end that session first.
while read -r existing_pid; do
  actual=$(readlink "/proc/$existing_pid/exe" 2>/dev/null || true)
  actual="${actual% (deleted)}"
  case "${actual##*/}" in
    controller_node|duty_controller_node|vesc_driver_node|planner_node|duty_keyboard|keyboard)
      echo "Existing vehicle process PID $existing_pid ($actual); refusing output takeover." >&2
      exit 73 ;;
  esac
done < <(ps -u "$UID" -o pid=)

run="$ws/log/planning_output_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run"
echo "planner=jetson mode=$mode" > "$run/placement.txt"
children=()
cleanup() {
  trap - EXIT TERM INT HUP
  for owned_pid in "${children[@]}"; do kill -INT "$owned_pid" 2>/dev/null || true; done
  for ((i=0;i<30;i++)); do
    owned_alive=false
    for owned_pid in "${children[@]}"; do kill -0 "$owned_pid" 2>/dev/null && owned_alive=true; done
    "$owned_alive" || break
    sleep 0.1
  done
  for owned_pid in "${children[@]}"; do kill -TERM "$owned_pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
trap 'exit 129' HUP

"$driver" --ros-args -r __ns:=/nomad_bench -r __node:=vesc_driver_lpp \
  --params-file "$driver_config" --params-file "$control_config/vesc_steering_test.yaml" "${extra_driver[@]}" \
  >"$run/driver.log" 2>&1 </dev/null &
children+=("$!")
"$controller" --ros-args "${controller_args[@]}" >"$run/controller.log" 2>&1 </dev/null &
children+=("$!")
# SSH session end = stdin EOF: stop this session (the GUI closes stdin to stop).
( cat >/dev/null; kill -TERM $$ 2>/dev/null ) &
children+=("$!")
echo "NOMAD_PI4_OUTPUT_READY mode=$mode planner=jetson logs=$run"
set +e
wait -n "${children[@]}"
exit $?
