#!/usr/bin/env bash
# 2026-11-12 "Jetson planner" mode: run the C++ planner (Pi4 source copy built in
# ~/nomad_planning_ws) next to the Jetson map. The GUI sends this text over SSH
# (bash -c) and talks to the planner on stdin/stdout exactly as on Pi4 (--stdio).
# Stdin EOF/SSH loss disarms and stops the planner. Uses the DDS profile of the
# running Jetson trial (Pi5 wired + Pi4 Wi-Fi peers). One planner per Jetson.
set -eo pipefail
planning_ws=$HOME/nomad_planning_ws
source /opt/ros/humble/install/setup.bash
source "$HOME/nomad_ws/install/setup.bash"
source "$planning_ws/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=0
export RCUTILS_LOGGING_USE_STDOUT=0

run_dir=$(cat /tmp/nomad_integrated_jetson_run_dir 2>/dev/null || true)
profile="$run_dir/planning_output_fastdds.xml"
[[ -n "$run_dir" && -r "$profile" ]] || { echo 'Jetson trial (map) is not running: start the trial first.' >&2; exit 2; }
export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"

exec 9>"/tmp/nomad-jetson-planning-${UID}.lock"
flock -n 9 || { echo 'Jetson planner already running in another window.' >&2; exit 73; }

planner="$planning_ws/install/nomad_planning_cpp/lib/nomad_planning_cpp/planner_node"
config="$planning_ws/install/nomad_planning_cpp/share/nomad_planning_cpp/config/planning_output.yaml"
for file in "$planner" "$config"; do
  test -r "$file" || { echo "Missing Jetson planner file: $file (build ~/nomad_planning_ws)" >&2; exit 2; }
done
log="$planning_ws/log/planning_jetson_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$log"
echo "trial=$run_dir" > "$log/placement.txt"
# 2026-11-13: GUI planner settings arrive as a base64 ROS parameter file; it is applied after
# the installed config (later --params-file wins) and kept in the log folder for the record.
params=(--params-file "$config")
# 2026-11-13 (later): values saved on the Jetson from the GUI settings dialog (board_settings.py).
board_overrides="$HOME/nomad_settings/planner/overrides.yaml"
if [[ -r "$board_overrides" ]]; then
  grep -q '^nomad_planning_cpp:' "$board_overrides" || { echo "Invalid saved planner settings: $board_overrides" >&2; exit 2; }
  # An older planner silently ignores undeclared ROS overrides. Refuse custom
  # candidate arrays until its reader is present; unchanged legacy lists emit no array.
  if grep -Eq '^[[:space:]]*rpp\.lookahead_candidates:[[:space:]]*\[[[:space:]]*[^][:space:]]' "$board_overrides" &&
     ! LC_ALL=C grep -aFq 'rpp.lookahead_candidates' "$planner"; then
    echo '캐럿 후보 배열을 읽는 Jetson 플래너 업데이트가 필요합니다. 저장한 후보를 무시한 채 시작하지 않습니다.' >&2
    exit 2
  fi
  cp "$board_overrides" "$log/planner_board_settings.yaml"
  params+=(--params-file "$log/planner_board_settings.yaml")
fi
# Older GUI copies still send their laptop values; the Jetson's saved values win when present.
if [[ "${1:-}" == --overrides-b64 ]]; then
  printf '%s' "${2:?}" | base64 -d > "$log/planner_overrides.yaml" || { echo 'Invalid planner settings from GUI' >&2; exit 2; }
  grep -q '^nomad_planning_cpp:' "$log/planner_overrides.yaml" || { echo 'Invalid planner settings from GUI' >&2; exit 2; }
  if [[ -r "$board_overrides" ]]; then
    echo 'Laptop planner settings ignored: using the values saved on the Jetson (update the GUI).' >&2
  else
    params+=(--params-file "$log/planner_overrides.yaml")
  fi
  shift 2
fi
echo "Jetson domain 59 · planner on Jetson · output awaits explicit arm · logs: $log" >&2
NOMAD_PLANNING_LOG_DIR="$log" exec "$planner" "$@" --ros-args "${params[@]}" 2>"$log/planner.log"
