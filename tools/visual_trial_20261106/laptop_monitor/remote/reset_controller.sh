#!/usr/bin/env bash
# Pi4: ask the running controller_node to clear a critical latch (std_srvs/Trigger ~/reset).
# Sends no drive, servo or arm command. The controller itself accepts RESET only when it is
# not running and fresh telemetry shows the car stationary for 0.5 s.
# The ROS domain and DDS profile are copied from the running controller process so the
# request reaches the same graph the planner started.
set -u
marker=NOMAD_CONTROLLER_RESET_RESULT=
result() {
  python3 - "$marker" "$1" "$2" "${3:-}" <<'PY'
import json, sys
marker, ok, reason, message = sys.argv[1:5]
print(marker + json.dumps({"ok": ok == "true", "reason": reason, "message": message[:300]},
                          ensure_ascii=False, separators=(",", ":")))
PY
  exit 0
}

ws=/home/user/nomad_cpp_ws
[[ -d "$ws/install/nomad_control_cpp" ]] || result false wrong_board
pid=$(pgrep -u "$(id -u)" -f "$ws/install/nomad_control_cpp/lib/nomad_control_cpp/controller_node" | head -n 1)
[[ -n "$pid" && -r "/proc/$pid/environ" ]] || result false controller_not_running

# Copy only the ROS/DDS selection variables of the running controller.
while IFS= read -r -d '' entry; do
  case "${entry%%=*}" in
    ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE|RMW_IMPLEMENTATION|\
    FASTRTPS_DEFAULT_PROFILES_FILE|RMW_FASTRTPS_USE_QOS_FROM_XML|CYCLONEDDS_URI)
      export "$entry" ;;
  esac
done < "/proc/$pid/environ"

set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || result false environment_unavailable
[[ -f /home/user/vesc_ws/install/setup.bash ]] && source /home/user/vesc_ws/install/setup.bash >/dev/null 2>&1
source "$ws/install/setup.bash" >/dev/null 2>&1 || result false environment_unavailable
set -u

service=$(timeout 12 ros2 service list --no-daemon --spin-time 3 2>/dev/null | grep -E '(^|/)nomad_vesc_trial/reset$' | head -n 1)
[[ -n "$service" ]] || result false service_unavailable

reply=$(timeout 12 ros2 service call "$service" std_srvs/srv/Trigger "{}" 2>&1)
status=$?
[[ $status -eq 0 ]] || result false call_timeout
message=$(printf '%s\n' "$reply" | sed -n "s/.*message='\(.*\)')\$/\1/p" | tail -n 1)
if printf '%s\n' "$reply" | grep -q 'success=True'; then
  result true reset_accepted "$message"
fi
result false reset_refused "$message"
