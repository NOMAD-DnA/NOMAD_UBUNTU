#!/usr/bin/env bash
# Stop the CURRENT visual-trial modules only. Never power off a board.
# No Python, process-name-wide kill, controller, Pi4, or product-source edits.
set -uo pipefail
shopt -s nullglob
declare -A starts=() groups=() labels=() known_groups=()
rejected=0
found=0
current_uid=$(id -u)
boot_id=$(cat /proc/sys/kernel/random/boot_id)

result() {
  printf 'NOMAD_STOP_RESULT={"state":"%s","reason":"%s","targets":%s,"remaining":%s,"rejected":%s}\n' "$1" "$2" "${found:-0}" "${remaining:-0}" "${rejected:-0}"
}

identity() {
  local pid=$1 line rest
  [[ "$pid" =~ ^[1-9][0-9]*$ && -r "/proc/$pid/stat" ]] || return 1
  IFS= read -r line < "/proc/$pid/stat" 2>/dev/null || return 1
  rest=${line##*) }
  read -r -a proc_fields <<< "$rest"
  [[ ${#proc_fields[@]} -ge 20 ]] || return 1
  process_state=${proc_fields[0]}
  process_group=${proc_fields[2]}
  process_start=${proc_fields[19]}
  process_uid=$(stat -c %u "/proc/$pid" 2>/dev/null) || return 1
  [[ "$process_state" != Z && "$process_state" != X ]]
}

same_process() {
  local pid=$1
  identity "$pid" && [[ "$process_uid" == "$current_uid" && "$process_start" == "${starts[$pid]}" && "$process_group" == "${groups[$pid]}" ]]
}

capture_group() {
  local leader=$1 label=$2 member member_group member_uid group=$process_group
  [[ "$group" == "$leader" && "$leader" != "$$" ]] || { rejected=$((rejected+1)); return 1; }
  [[ -z ${known_groups[$group]+yes} ]] || return 0
  known_groups[$group]=1
  while read -r member member_group member_uid; do
    [[ "$member_group" == "$group" ]] || continue
    identity "$member" || continue
    [[ "$process_group" == "$group" ]] || continue
    if [[ "$process_uid" != "$current_uid" || "$member" == "$$" ]]; then
      rejected=$((rejected+1)); continue
    fi
    starts[$member]=$process_start
    groups[$member]=$group
    labels[$member]=$label
  done < <(ps -eo pid=,pgid=,uid=)
}

valid_module_command() {
  local pid=$1 label=$2 command cwd token
  local -a argv=()
  command=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null) || return 1
  [[ "$command" == *"$ws/"* ]] || return 1
  mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || return 1
  [[ ${#argv[@]} -gt 0 ]] || return 1
  # The alternate semantic producer and its passthrough adapter are native
  # binaries. Accept their exact executable token only, inside this workspace.
  case "$label:${argv[0]##*/}" in
    semantic:nomad_fcn_preview|semantic_adapter:nomad_external_frame_alias)
      [[ "${argv[0]}" == "$ws/"* ]] && return 0;;
  esac
  case "$label:$command" in
    oak:*oak*|zed:*zed*|lidar:*ydlidar_ros2_driver_node*|semantic:*semantic*|frame_alias:*nomad_external_frame_alias*|semantic_adapter:*semantic*adapter*|external_cloud:*external_stereo_cloud*|gnss:*gnss*|gnss:*ublox*|vio:*basalt*|fusion:*fusion*|costmap:*nomad_runtime_manager*|mapper:*nomad_mapper*|monitor:*nomad_monitor*) return 0;;
  esac
  return 1
}

candidate() {
  local pid=$1 label=$2 expected_start=${3:-} expected_group=${4:-} expected_boot=${5:-}
  [[ "$pid" =~ ^[1-9][0-9]*$ ]] || { rejected=$((rejected+1)); return; }
  identity "$pid" || return 0 # already ended, including old-boot PID files
  if [[ "$process_uid" != "$current_uid" ]] || ! valid_module_command "$pid" "$label"; then
    rejected=$((rejected+1)); return
  fi
  if [[ -n "$expected_start" && ( "$process_start" != "$expected_start" || "$process_group" != "$expected_group" || "$boot_id" != "$expected_boot" ) ]]; then
    rejected=$((rejected+1)); return
  fi
  capture_group "$pid" "$label"
}

json_number() {
  sed -nE 's/.*"'"$1"'"[[:space:]]*:[[:space:]]*([0-9]+).*/\1/p' <<< "$2"
}
json_string() {
  sed -nE 's/.*"'"$1"'"[[:space:]]*:[[:space:]]*"([A-Za-z0-9_-]+)".*/\1/p' <<< "$2"
}
state_identity() {
  local file=$1 key=$2 label=$3 object pid start pg boot
  # The manager writes flat identity objects containing only numeric fields,
  # boot UUID, and process_state. Do not parse or execute arbitrary JSON text.
  object=$(sed -nE 's/.*"'"$key"'"[[:space:]]*:[[:space:]]*(\{[^{}]*\}).*/\1/p' "$file")
  [[ -n "$object" ]] || return 0
  pid=$(json_number pid "$object"); start=$(json_number start_ticks "$object")
  pg=$(json_number pgid "$object"); boot=$(json_string boot_id "$object")
  [[ -n "$pid" && -n "$start" && -n "$pg" && -n "$boot" ]] || { rejected=$((rejected+1)); return; }
  candidate "$pid" "$label" "$start" "$pg" "$boot"
}

signal_stage() {
  local signal_name=$1 pid
  for pid in "${!starts[@]}"; do
    same_process "$pid" || continue
    # Signal the captured, revalidated task, not an unchecked whole PGID.
    kill -s "$signal_name" -- "$pid" 2>/dev/null || true
  done
}

trial_process_exists() {
  local pid uid argv token command
  while read -r pid uid; do
    [[ "$uid" == "$current_uid" && "$pid" != "$$" && -r "/proc/$pid/cmdline" ]] || continue
    argv=()
    mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || continue
    for token in "${argv[@]}"; do
      case "${token##*/}" in
        run_oak_cpp.sh|oak_publisher|run_basalt_cpp.sh|vio_frontend_node|vio_backend_node|run_vio_cpp.sh|vio_session|run_gnss.sh|run_fusion.sh|fusion_node|run_zed_sensors.sh|zed_camera.launch.py|zed_camera.preview.launch.py|ydlidar_ros2_driver_node|nomad_external_frame_alias|nomad_semantic_id_adapter|nomad_fcn_preview|external_stereo_cloud|nomad_runtime_manager|nomad_mapper|nomad_monitor|semantic.sh|nomad_perception.modules.semantic) return 0;;
        receiver) [[ "$token" != "$ws/install/nomad_gnss/lib/nomad_gnss/receiver" ]] || return 0;;
        component_container|component_container_isolated)
          command=" ${argv[*]} "
          [[ "$command" != *zed_container* ]] || return 0;;
      esac
    done
  done < <(ps -eo pid=,uid=)
  return 1
}

main() {
  local user model pointer run file name pid state count group member_group member_uid
  user=$(id -un)
  model=$(tr '\0' ' ' < /proc/device-tree/model 2>/dev/null || true)
  case "$user:$model" in
    user:*Raspberry\ Pi\ 5*) ws=/home/user/nomad_ws; pointer=/tmp/nomad_integrated_pi_run_dir; board=pi5;;
    nvidia:*Jetson*Xavier*|nvidia:*NVIDIA*Xavier*) ws=/home/nvidia/nomad_ws; pointer=/tmp/nomad_integrated_jetson_run_dir; board=jetson;;
    *) result FAILED board_identity_mismatch; return 2;;
  esac
  [[ -d "$ws" && $(stat -c %u "$ws") == "$current_uid" ]] || { result FAILED workspace_identity_mismatch; return 2; }
  exec 9>"/tmp/nomad_visual_trial_stop_${current_uid}.lock"
  flock -n 9 || { result FAILED stop_already_running; return 2; }
  if [[ ! -e "$pointer" && ! -L "$pointer" ]]; then
    # A fresh boot legitimately has no run pointer. Prove that there are no
    # installed trial-module tokens for this account before permitting a start.
    # If even one remains, do not guess its ownership or manufacture a pointer.
    if trial_process_exists; then
      result FAILED current_run_unknown; return 2
    fi
    result STOPPED no_trial_processes_board_power_on; return 0
  fi
  if [[ ! -f "$pointer" || -L "$pointer" || $(stat -c %u "$pointer") != "$current_uid" ]]; then
    result FAILED current_run_unknown; return 2
  fi
  run=$(cat "$pointer")
  [[ "$run" == "$ws"/log/visual_trial_* && -d "$run" && $(readlink -f "$run") == "$run" && $(stat -c %u "$run") == "$current_uid" ]] || { result FAILED current_run_invalid; return 2; }
  for file in "$run"/*.pid "$run"/oak/oak.pid; do
    [[ -f "$file" && ! -L "$file" && $(stat -c %u "$file") == "$current_uid" ]] || continue
    name=${file##*/}; name=${name%.pid}
    case "$board:$name" in
      pi5:gnss|pi5:vio|pi5:fusion|jetson:oak|jetson:zed|jetson:lidar|jetson:semantic|jetson:frame_alias|jetson:semantic_adapter|jetson:external_cloud|jetson:costmap) ;;
      *) continue;;
    esac
    pid=$(cat "$file")
    [[ -n "$pid" ]] || continue
    candidate "$pid" "$name"
  done
  if [[ "$board" == jetson ]]; then
    for state in "$run"/costmap_runs/*/state.json; do
      [[ -f "$state" && ! -L "$state" && $(stat -c %u "$state") == "$current_uid" ]] || continue
      state_identity "$state" manager costmap
      state_identity "$state" mapper mapper
      state_identity "$state" monitor monitor
      state_identity "$state" lidar lidar
    done
  fi
  found=${#starts[@]}
  signal_stage INT
  sleep 2
  signal_stage TERM
  sleep 2
  signal_stage KILL
  sleep 0.3
  remaining=0
  for pid in "${!starts[@]}"; do
    same_process "$pid" && remaining=$((remaining+1))
  done
  # Detect late/untracked descendants rather than reporting a false success.
  while read -r pid member_group member_uid; do
    [[ -n ${known_groups[$member_group]+yes} && "$member_uid" == "$current_uid" && -z ${starts[$pid]+yes} ]] || continue
    identity "$pid" && remaining=$((remaining+1))
  done < <(ps -eo pid=,pgid=,uid=)
  if (( remaining || rejected )); then
    result FAILED owned_cleanup_incomplete; return 2
  fi
  result STOPPED trial_modules_stopped_board_power_on
}

# MAIN_ENTRY: local tests load the functions above without calling this entry.
main "$@"
