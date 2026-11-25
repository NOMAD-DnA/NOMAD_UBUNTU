#!/usr/bin/env bash
# User-started prototype output session. No arming and no nonzero command here.
set -eo pipefail
ws="${NOMAD_PLANNING_WS:-/home/user/nomad_cpp_ws}"
# Explicit opt-in only. No saved selection means the original RPM launcher.
selected_mode="${NOMAD_CONTROL_MODE:-}"
if [[ -z "$selected_mode" && -f "$ws/var/motor_control_mode" ]]; then
  IFS= read -r selected_mode < "$ws/var/motor_control_mode" || true
fi
selected_mode="${selected_mode:-rpm}"
case "$selected_mode" in
  rpm) ;;
  vio_duty)
    duty_launcher="$ws/install/nomad_vio_duty_control/lib/nomad_vio_duty_control/run_planning_vio_duty.sh"
    [[ -x "$duty_launcher" ]] || { echo "Missing VIO-duty launcher: $duty_launcher" >&2; exit 2; }
    exec "$duty_launcher" "$@"
    ;;
  *) echo "Unknown motor control mode: $selected_mode (choose rpm or vio_duty)." >&2; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source /home/user/vesc_ws/install/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export RCUTILS_LOGGING_USE_STDOUT=0
exec 9>"/tmp/nomad-planning-${UID}.lock"
flock -n 9 || { echo 'Planner session already owned by another window.' >&2; exit 73; }
exec 8>"/tmp/nomad-output-${UID}.lock"
flock -n 8 || { echo 'Vehicle output session already owned.' >&2; exit 73; }

planner="$ws/install/nomad_planning_cpp/lib/nomad_planning_cpp/planner_node"
controller="$ws/install/nomad_control_cpp/lib/nomad_control_cpp/controller_node"
driver=/home/user/vesc_ws/install/vesc_driver/lib/vesc_driver/vesc_driver_node
control_config="$ws/install/nomad_control_cpp/share/nomad_control_cpp/config"
planning_config="$ws/install/nomad_planning_cpp/share/nomad_planning_cpp/config/planning_output.yaml"
driver_config="$ws/commission_session/vesc_commission.yaml"
for file in "$planner" "$controller" "$driver" "$planning_config" "$driver_config" "$control_config/planning_headless.yaml"; do
  test -r "$file" || { echo "Missing required output file: $file" >&2; exit 2; }
done
if test -n "$(ps -C duty_keyboard,keyboard -o pid= 2>/dev/null)"; then
  echo 'Close the keyboard controller before starting planner output.' >&2
  exit 73
fi

# Replace only this vehicle's known executables, never arbitrary process names.
stop_existing() {
  local name="$1" expected="$2" pid actual
  expected=$(readlink -f "$expected")
  for pid in $(ps -C "$name" -o pid= 2>/dev/null); do
    actual=$(readlink "/proc/$pid/exe" 2>/dev/null || true)
    actual="${actual% (deleted)}"
    actual=$(readlink -f "$actual" 2>/dev/null || true)
    test "$actual" = "$expected" || { echo "Unexpected $name owner; refusing takeover." >&2; return 73; }
    kill -INT "$pid"
    for ((i=0;i<40;i++)); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
      echo "$name did not stop; refusing a second output process." >&2
      return 73
    fi
  done
}
stop_existing controller_node "$controller"
stop_existing vesc_driver_node "$driver"

run="$ws/log/planning_output_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run"
children=()
pidfile="/tmp/nomad-planning-${UID}.pid"
cleanup() {
  trap - EXIT TERM INT HUP
  for pid in "${children[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for ((i=0;i<30;i++)); do
    local_alive=false
    for pid in "${children[@]}"; do kill -0 "$pid" 2>/dev/null && local_alive=true; done
    "$local_alive" || break
    sleep 0.1
  done
  for pid in "${children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  rm -f "$pidfile"
}
trap cleanup EXIT TERM INT HUP
"$driver" --ros-args -r __ns:=/nomad_bench -r __node:=vesc_driver_lpp \
  --params-file "$driver_config" --params-file "$control_config/vesc_steering_test.yaml" \
  >"$run/driver.log" 2>&1 &
children+=("$!")
"$controller" --ros-args \
  --params-file "$control_config/commissioning.yaml" \
  --params-file "$control_config/planning.yaml" \
  --params-file "$control_config/planning_headless.yaml" \
  --params-file "$control_config/nomad_vehicle_ready.yaml" \
  >"$run/controller.log" 2>&1 &
children+=("$!")
echo "Pi4 domain 59 · prototype mapping · output awaits explicit arm · logs: $run" >&2
NOMAD_PLANNING_LOG_DIR="$run" "$planner" "$@" --ros-args --params-file "$planning_config" <&0 2>"$run/planner.log" &
children+=("$!")
printf '%s\n' "$!" > "$pidfile"
set +e
wait -n "${children[@]}"
result=$?
exit "$result"
