#!/usr/bin/env bash
# Stop the current visual-trial modules, then this account's leftover NOMAD modules. Never power off a board.
# No Python, controller, Pi4, or product-source edits.
# 2026-11-16: after the pid-file stop, a sweep also stops this account's remaining
# NOMAD modules found by the known module names (module_label), one revalidated pid
# at a time (INT -> TERM -> KILL). Survivors are reported. During a Jetson sensor
# recording (/mnt/ssd/bags/RECORDING) sensor modules are kept: recording_active.
# Review fix: the sweep matches only the executed program (sweep_label: argv[0], or the
# script/launch file of bash/python3/ros2) under the NOMAD workspaces, never a module
# name that only appears in arguments (colcon, vim, tail stay untouched). On the Pi5 the
# GUI sensor-recording GPS receiver (ROS_LOG_DIR=$ws/log/record_gnss_*/ros) is kept too.
set -uo pipefail
shopt -s nullglob
declare -A starts=() groups=() labels=() known_groups=() kept_pids=() survivors=() sweep_groups=()
rejected=0
found=0
swept=0
kept=0
board=""
recording_marker=/mnt/ssd/bags/RECORDING
current_uid=$(id -u)
boot_id=$(cat /proc/sys/kernel/random/boot_id)

result() {
  local list="" pid n=0
  for pid in "${!survivors[@]}"; do
    (( n++ < 12 )) && list+="${list:+,}${survivors[$pid]}:$pid"
  done
  printf 'NOMAD_STOP_RESULT={"state":"%s","reason":"%s","targets":%s,"remaining":%s,"rejected":%s,"swept":%s,"kept":%s,"recording":%s,"survivors":"%s"}\n' "$1" "$2" "${found:-0}" "${remaining:-0}" "${rejected:-0}" "${swept:-0}" "${kept:-0}" "$( (recording_active || (( kept ))) && echo true || echo false)" "$list"
}

# 2026-11-16: a Jetson sensor recording is running; its sensors must stay up.
recording_active() {
  [[ "$board" == jetson && -f "$recording_marker" ]]
}
sensor_label() {
  case "$1" in oak|zed|lidar|semantic|frame_alias|semantic_adapter|external_cloud) return 0;; esac
  return 1
}
# Pi5: started under a GUI sensor-recording run folder (record_gnss_start_pi.sh).
record_gnss_owned() {
  local pid=$1 item
  local -a envs=()
  mapfile -d '' -t envs < "/proc/$pid/environ" 2>/dev/null || return 1
  for item in "${envs[@]}"; do
    case "$item" in
      ROS_LOG_DIR="$ws"/log/record_gnss_*/ros) return 0;;
    esac
  done
  return 1
}
# Keep (do not signal) a recording sensor. Sets kept/kept_pids.
keep_for_recording() {
  local pid=$1 label=$2
  if [[ "$board" == pi5 ]]; then
    record_gnss_owned "$pid" || return 1
  else
    recording_active && sensor_label "$label" || return 1
  fi
  [[ -n ${kept_pids[$pid]+yes} ]] || { kept_pids[$pid]=$label; kept=$((kept+1)); }
  return 0
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
    # e.g. a lidar started inside the costmap group while a recording uses it.
    if [[ "$board" == pi5 ]] && keep_for_recording "$member" ""; then
      continue
    fi
    if recording_active && module_label "$member" && keep_for_recording "$member" "$module"; then
      continue
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
  # 2026-11-13: the new mapper lives in its own workspace next to nomad_ws.
  [[ "$command" == *"$ws/"* || ( "$label" == mapper2 && "$command" == *"${ws%/nomad_ws}/nomad_mapper2_ws/"* ) ||
     ( "$label" == terrain && "$command" == *"${ws%/nomad_ws}/nomad_terrain_ws/"* ) ||
     ( "$label" == mapper3 && "$command" == *"/mnt/ssd/nomad/nomad_mapper3_ws/"* ) ]] || return 1
  mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || return 1
  [[ ${#argv[@]} -gt 0 ]] || return 1
  # The alternate semantic producer and its passthrough adapter are native
  # binaries. Accept their exact executable token only, inside this workspace.
  case "$label:${argv[0]##*/}" in
    semantic:nomad_fcn_preview|semantic_adapter:nomad_external_frame_alias)
      [[ "${argv[0]}" == "$ws/"* ]] && return 0;;
  esac
  case "$label:$command" in
    oak:*oak*|zed:*zed*|lidar:*ydlidar_ros2_driver_node*|semantic:*semantic*|frame_alias:*nomad_external_frame_alias*|semantic_adapter:*semantic*adapter*|external_cloud:*external_stereo_cloud*|gnss:*gnss*|gnss:*ublox*|vio:*basalt*|fusion:*fusion*|vio_supervisor:*nomad_vio_supervisor*|front_recorder:*nomad_front_recorder*|costmap:*nomad_runtime_manager*|mapper:*nomad_mapper*|mapper2:*nomad_mapper2*|mapper3:*nomad_mapper3*|terrain:*nomad_terrain_mapper*|monitor:*nomad_monitor*) return 0;;
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
  keep_for_recording "$pid" "$label" && return
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

# 2026-11-12: return as soon as every captured process is gone (max $1 seconds).
wait_gone() {
  local tenths=$1 i pid alive
  for ((i=0;i<tenths;i++)); do
    alive=0
    for pid in "${!starts[@]}"; do same_process "$pid" && { alive=1; break; }; done
    (( alive )) || return 0
    sleep 0.1
  done
}

signal_stage() {
  local signal_name=$1 pid
  for pid in "${!starts[@]}"; do
    same_process "$pid" || continue
    # Signal the captured, revalidated task, not an unchecked whole PGID.
    kill -s "$signal_name" -- "$pid" 2>/dev/null || true
  done
}

# Label of one installed trial-module token. Sets $module.
token_label() {
  local token=$1
  module=""
  case "${token##*/}" in
    run_oak_cpp.sh|oak_publisher) module=oak;;
    run_basalt_cpp.sh|vio_frontend_node|vio_backend_node|run_vio_cpp.sh|vio_session) module=vio;;
    run_gnss.sh) module=gnss;;
    run_fusion.sh|fusion_node) module=fusion;;
    vio_supervisor|front_recorder) module=${token##*/};;
    run_zed_sensors.sh|zed_camera.launch.py|zed_camera.preview.launch.py) module=zed;;
    ydlidar_ros2_driver_node) module=lidar;;
    nomad_external_frame_alias) module=frame_alias;;
    nomad_semantic_id_adapter) module=semantic_adapter;;
    nomad_fcn_preview|semantic.sh|nomad_perception.modules.semantic) module=semantic;;
    external_stereo_cloud) module=external_cloud;;
    nomad_runtime_manager) module=costmap;;
    nomad_mapper|nomad_mapper2|nomad_mapper3|nomad_terrain_mapper|nomad_monitor) module=${token##*/}; module=${module#nomad_}
      [[ "$module" != terrain_mapper ]] || module=terrain;;
    receiver) [[ "$token" != "$ws/install/nomad_gnss/lib/nomad_gnss/receiver" ]] || module=gnss;;
  esac
  [[ -n "$module" ]]
}

# Known installed trial-module tokens anywhere in argv (detection only, e.g.
# trial_process_exists). Sets $module to the module label.
module_label() {
  local pid=$1 token command
  local -a argv=()
  module=""
  mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || return 1
  for token in "${argv[@]}"; do
    case "${token##*/}" in
      component_container|component_container_isolated)
        command=" ${argv[*]} "
        [[ "$command" != *zed_container* ]] || module=zed;;
      *) token_label "$token";;
    esac
    [[ -z "$module" ]] || return 0
  done
  return 1
}

# Workspaces whose programs the sweep may stop (literal and resolved forms).
sweep_roots=()
under_sweep_roots() {
  local path=$1 root resolved
  if (( ! ${#sweep_roots[@]} )); then
    for root in "$ws" "${ws%/nomad_ws}/nomad_mapper2_ws" "${ws%/nomad_ws}/nomad_terrain_ws" /mnt/ssd/nomad; do
      sweep_roots+=("$root/")
      resolved=$(readlink -f "$root" 2>/dev/null) && [[ -n "$resolved" ]] && sweep_roots+=("$resolved/")
    done
  fi
  resolved=$(readlink -f "$path" 2>/dev/null || true)
  for root in "${sweep_roots[@]}"; do
    [[ "$path" == "$root"* || ( -n "$resolved" && "$resolved" == "$root"* ) ]] && return 0
  done
  return 1
}

# 2026-11-16 review: the sweep (which signals) matches the executed program only:
# argv[0], or for bash/sh/python3 the script (or -m module), for ros2 launch/run the
# launch file / executable. The program must be under a NOMAD workspace, except the
# ZED component container (/opt/ros) which needs its own zed_container node name.
sweep_label() {
  local pid=$1 path arg cwd i j n
  local -a argv=() rest=()
  module=""
  mapfile -d '' -t argv < "/proc/$pid/cmdline" 2>/dev/null || return 1
  n=${#argv[@]}
  (( n )) || return 1
  i=0
  case "${argv[0]##*/}" in
    bash|sh|dash|python|python3|python3.[0-9]*)
      for ((i=1;i<n;i++)); do
        arg=${argv[i]}
        case "$arg" in
          -c) return 1;;
          -m) [[ "${argv[i+1]:-}" == nomad_perception.modules.semantic ]] || return 1
              module=semantic; return 0;;
          -*) continue;;
        esac
        break
      done
      (( i < n )) || return 1;;
  esac
  path=${argv[i]}
  if [[ "${path##*/}" == ros2 ]]; then
    case "${argv[i+1]:-}" in launch|run) ;; *) return 1;; esac
    for ((j=i+2;j<n && ${#rest[@]}<2;j++)); do
      arg=${argv[j]}
      [[ "$arg" == -* || "$arg" == *:=* ]] && break
      rest+=("$arg")
    done
    (( ${#rest[@]} )) || return 1
    path=${rest[${#rest[@]}-1]}
    [[ "${rest[0]}" != /* ]] || path=${rest[0]}
  fi
  case "${path##*/}" in
    component_container|component_container_isolated)
      for ((j=i+1;j<n;j++)); do
        [[ "${argv[j]}" != *zed_container* ]] || { module=zed; return 0; }
      done
      return 1;;
  esac
  if [[ "$path" != */* ]]; then
    (( i == 0 )) || return 1
    path=$(readlink "/proc/$pid/exe" 2>/dev/null) || return 1
  elif [[ "$path" != /* ]]; then
    cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null) || return 1
    path="$cwd/$path"
  fi
  token_label "$path" || return 1
  under_sweep_roots "$path"
}

# Calls $1 <pid> <label> for every live module process of this account.
# Same token list as before 2026-11-16, e.g. |vio_supervisor|front_recorder| and |nomad_mapper|nomad_mapper2|nomad_mapper3|.
# Optional $2: the matcher (default module_label; the sweep uses sweep_label).
each_module_process() {
  local pid uid matcher=${2:-module_label}
  while read -r pid uid; do
    [[ "$uid" == "$current_uid" && "$pid" != "$$" && -r "/proc/$pid/cmdline" ]] || continue
    "$matcher" "$pid" || continue
    "$1" "$pid" "$module" || return 0
  done < <(ps -eo pid=,uid=)
}

trial_process_exists() {
  local hit=1
  # shellcheck disable=SC2329
  first_hit() { hit=0; return 1; }
  each_module_process first_hit
  return $hit
}

# Sweep: capture one remaining module process by its own identity (no PGID kill).
sweep_capture() {
  local pid=$1 label=$2
  [[ -z ${starts[$pid]+yes} ]] || return 0
  identity "$pid" && [[ "$process_uid" == "$current_uid" ]] || return 0
  keep_for_recording "$pid" "$label" && return 0
  starts[$pid]=$process_start
  groups[$pid]=$process_group
  labels[$pid]=$label
  # A swept group leader (ros2 launch, run_*.sh): its leftover children are reported.
  [[ "$process_group" != "$pid" ]] || sweep_groups[$pid]=1
  swept=$((swept+1))
}
note_survivor() {
  local pid=$1 label=$2
  [[ -z ${kept_pids[$pid]+yes} ]] || return 0
  keep_for_recording "$pid" "$label" && return 0
  identity "$pid" && [[ "$process_uid" == "$current_uid" ]] && survivors[$pid]=$label
  return 0
}
collect_survivors() {
  local pid member_group member_uid
  for pid in "${!starts[@]}"; do
    same_process "$pid" && survivors[$pid]=${labels[$pid]}
  done
  # Detect late/untracked descendants rather than reporting a false success.
  while read -r pid member_group member_uid; do
    [[ ( -n ${known_groups[$member_group]+yes} || -n ${sweep_groups[$member_group]+yes} ) && "$member_uid" == "$current_uid" &&
       "$pid" != "$$" && -z ${starts[$pid]+yes} && -z ${kept_pids[$pid]+yes} ]] || continue
    identity "$pid" && [[ "$process_uid" == "$current_uid" ]] && survivors[$pid]=child
  done < <(ps -eo pid=,pgid=,uid=)
  each_module_process note_survivor sweep_label
  remaining=${#survivors[@]}
}

stop_captured() {
  local int_tenths=$1
  signal_stage INT
  wait_gone "$int_tenths"
  signal_stage TERM
  wait_gone 20
  signal_stage KILL
  wait_gone 3
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
    # A fresh boot legitimately has no run pointer. 2026-11-16: remaining module
    # processes of this account (no pid file, e.g. a recording's sensors or an old
    # run) are stopped by the sweep, never by a guessed pointer.
    sweep_and_report no_pointer; return
  fi
  if [[ ! -f "$pointer" || -L "$pointer" || $(stat -c %u "$pointer") != "$current_uid" ]]; then
    result FAILED current_run_unknown; return 2
  fi
  run=$(cat "$pointer")
  # 2026-11-15: ~/nomad_ws is a symlink to /mnt/ssd/nomad/nomad_ws on the Jetson (SSD move 11-14), so compare the
  # resolved run folder with the resolved workspace; the run folder itself must still not be a symlink.
  [[ "$run" == "$ws"/log/visual_trial_* && -d "$run" && ! -L "$run" && $(readlink -f "$run") == "$(readlink -f "$ws")/log/${run##*/}" && $(stat -c %u "$run") == "$current_uid" ]] || { result FAILED current_run_invalid; return 2; }
  for file in "$run"/*.pid "$run"/oak/oak.pid; do
    [[ -f "$file" && ! -L "$file" && $(stat -c %u "$file") == "$current_uid" ]] || continue
    name=${file##*/}; name=${name%.pid}
    case "$board:$name" in
      pi5:gnss|pi5:vio|pi5:fusion|pi5:vio_supervisor|pi5:front_recorder|jetson:oak|jetson:zed|jetson:lidar|jetson:semantic|jetson:frame_alias|jetson:semantic_adapter|jetson:external_cloud|jetson:costmap|jetson:mapper2|jetson:mapper3|jetson:terrain) ;;
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
  (( found )) && stop_captured 20
  sweep_and_report pointer
}

# Time budget (run_remote 18 s): pid-file stop <= 4.3 s + sweep <= 5.3 s + scans.
sweep_and_report() {
  local mode=$1
  each_module_process sweep_capture sweep_label
  (( swept )) && stop_captured 30
  collect_survivors
  # A running sensor recording wins over leftovers: never let it look power-off-safe.
  if (( kept )); then
    result FAILED recording_active; return 2
  fi
  if (( remaining || rejected )); then
    result FAILED owned_cleanup_incomplete; return 2
  fi
  if [[ "$mode" == no_pointer ]]; then
    if (( swept )); then result STOPPED orphan_modules_stopped_board_power_on
    else result STOPPED no_trial_processes_board_power_on; fi
    return 0
  fi
  result STOPPED trial_modules_stopped_board_power_on
}

# MAIN_ENTRY: local tests load the functions above without calling this entry.
main "$@"
