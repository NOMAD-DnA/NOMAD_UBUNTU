#!/usr/bin/env bash
# Swap inference only in the CURRENT visual trial; sensors/VIO/map stay running.
set -eo pipefail
exec 2>&1
ws=/home/nvidia/nomad_ws
root="$ws/tools/indoor_fcn_20261106"
preview="$ws/tools/visual_trial_20261106"
model=${1:-}
fail() { printf 'MODEL_SWITCH_FAILED: %s\n' "$*"; exit 1; }
[[ "$model" == b2 || "$model" == fcn ]] || fail invalid_model
[[ $(id -un) == nvidia ]] || fail wrong_account
uid=$(id -u)
exec 9>"/tmp/nomad_visual_trial_stop_${uid}.lock"
flock -n 9 || fail another_start_stop_or_switch_active
pointer=/tmp/nomad_integrated_jetson_run_dir
[[ -f "$pointer" && ! -L "$pointer" && $(stat -c %u "$pointer") == "$uid" ]] || fail no_current_trial
run=$(cat "$pointer")
[[ "$run" == "$ws"/log/visual_trial_* && -d "$run" && ! -L "$run" && $(readlink -f "$run") == "$run" && $(stat -c %u "$run") == "$uid" ]] || fail invalid_trial_path
alias_bin="$preview/build_external_frame_alias/nomad_external_frame_alias"
[[ -x "$alias_bin" && -f "$ws/config/oak_fastdds_wired.xml" ]] || fail missing_common_runtime
if [[ "$model" == fcn ]]; then
  for f in "$root/build_512x512/nomad_fcn_preview" "$root/model/fcn_resnet18_512x512_fp16.engine" "$root/model/classes.txt" "$root/config/sunrgbd_to_field12.json"; do
    [[ -s "$f" ]] || fail "FCN missing: $f"
  done
else
  [[ -s /home/nvidia/models/segformer/ade20k_b2/model_fp16.engine && -s "$preview/config/ade20k.preview.json" && -x "$ws/semantic.sh" ]] || fail B2_runtime_missing
fi

identity() {
  local pid=$1 line rest
  [[ "$pid" =~ ^[1-9][0-9]*$ && -r /proc/$pid/stat ]] || return 1
  IFS= read -r line < "/proc/$pid/stat" || return 1
  rest=${line##*) }; read -r -a fields <<< "$rest"
  [[ ${#fields[@]} -ge 20 && ${fields[0]} != Z && ${fields[0]} != X ]] || return 1
  proc_group=${fields[2]}; proc_start=${fields[19]}
  [[ $(stat -c %u "/proc/$pid") == "$uid" ]]
}
valid() {
  local pid=$1 label=$2; local -a argv=()
  mapfile -d '' -t argv < "/proc/$pid/cmdline" || return 1
  [[ ${#argv[@]} -gt 0 && " ${argv[*]} " == *"$ws/"* ]] || return 1
  case "$label:${argv[0]##*/}:${argv[*]}" in
    semantic:nomad_fcn_preview:*|semantic:*:*semantic.sh*|semantic:*:*nomad_perception.modules.semantic*|semantic_adapter:nomad_external_frame_alias:*|semantic_adapter:*:*nomad_semantic_id_adapter*|semantic_adapter:*:*run_semantic_adapter.sh*) return 0;;
  esac
  return 1
}
check_owned_leader() {
  local label=$1 pid
  [[ -f "$run/$label.pid" && ! -L "$run/$label.pid" ]] || return 1
  pid=$(cat "$run/$label.pid")
  identity "$pid" && [[ "$proc_group" == "$pid" ]] && valid "$pid" "$label"
}
# Require an active ZED/camera-info alias before touching inference.
for label in zed frame_alias; do
  [[ -r "$run/$label.pid" ]] || fail "inactive trial: $label"
  pid=$(cat "$run/$label.pid")
  identity "$pid" || fail "inactive trial: $label"
done
previous=b2
if [[ -r "$run/semantic.pid" ]]; then
  p=$(cat "$run/semantic.pid")
  if identity "$p" && valid "$p" semantic; then
    command=$(tr '\0' ' ' < "/proc/$p/cmdline")
    [[ "$command" != *nomad_fcn_preview* ]] || previous=fcn
  fi
fi
stop_pair() {
  local label pid member pg owner sig round
  local -A starts=() groups=()
  for label in semantic semantic_adapter; do
    [[ -r "$run/$label.pid" ]] || continue
    pid=$(cat "$run/$label.pid")
    identity "$pid" || continue
    [[ "$proc_group" == "$pid" ]] && valid "$pid" "$label" || fail "unrecognized inference PID: $label"
    while read -r member pg owner; do
      [[ "$pg" == "$pid" && "$owner" == "$uid" ]] || continue
      identity "$member" || continue
      [[ "$proc_group" == "$pid" ]] || continue
      starts[$member]=$proc_start; groups[$member]=$proc_group
    done < <(ps -eo pid=,pgid=,uid=)
  done
  for sig in INT TERM KILL; do
    for member in "${!starts[@]}"; do
      identity "$member" || continue
      [[ "$proc_start" == "${starts[$member]}" && "$proc_group" == "${groups[$member]}" ]] || continue
      kill -s "$sig" "$member" 2>/dev/null || true
    done
    sleep 1
  done
  for member in "${!starts[@]}"; do
    if identity "$member" && [[ "$proc_start" == "${starts[$member]}" && "$proc_group" == "${groups[$member]}" ]]; then
      fail previous_inference_not_stopped
    fi
  done
}
source /opt/ros/humble/install/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ws/config/oak_fastdds_wired.xml" RMW_FASTRTPS_USE_QOS_FROM_XML=0
export LD_LIBRARY_PATH="$preview/runtime-map-preview/lib:/opt/ros/humble/install/lib:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
launch_one() {
  local label=$1; shift
  setsid env ROS_LOG_DIR="$run/ros" "$@" 9>&- </dev/null > "$run/${label}_${selected}_${stamp}.log" 2>&1 &
  printf '%s\n' "$!" > "$run/$label.pid"
  printf '%s\n' "$run/${label}_${selected}_${stamp}.log" > "$run/${label}_current_log"
}
launch_pair() {
  selected=$1
  stamp=$(date +%Y%m%d_%H%M%S)_$$
  if [[ "$selected" == b2 ]]; then
    launch_one semantic bash "$ws/semantic.sh" -p rate_hz:=1.0
    launch_one semantic_adapter env NOMAD_SEMANTIC_MAPPING="$preview/config/ade20k.preview.json" bash "$ws/src/nomad_perception/traversability/run_semantic_adapter.sh"
  else
    launch_one semantic "$root/build_512x512/nomad_fcn_preview" --ros-args \
      -p engine_path:="$root/model/fcn_resnet18_512x512_fp16.engine" \
      -p classes_path:="$root/model/classes.txt" -p mapping_path:="$root/config/sunrgbd_to_field12.json" \
      -p field12_output_topic:=/nomad_preview/sunrgbd/field12_raw -p rate_hz:=1.0
    launch_one semantic_adapter "$alias_bin" --ros-args -r __node:=nomad_sunrgbd_field12_frame_alias \
      -p confirmed_camera_serial:=23472639 -p same_left_rectified_optical_axis_confirmed:=true \
      -p input_label_topic:=/nomad_preview/sunrgbd/field12_raw -p output_label_topic:=/perception/semantic/class_ids \
      -p input_camera_info_topic:=/nomad_preview/sunrgbd/unused_camera_info_input \
      -p output_camera_info_topic:=/nomad_preview/sunrgbd/unused_camera_info_output
  fi
  for round in {1..20}; do
    sleep 1
    check_owned_leader semantic && check_owned_leader semantic_adapter || return 1
    if [[ "$selected" == b2 ]]; then
      (( round < 5 )) || return 0
    elif grep -q 'Published raw FCN and field12 IDs' "$(cat "$run/semantic_current_log")" && \
         grep -Eq 'aliased labels=[1-9][0-9]*' "$(cat "$run/semantic_adapter_current_log")"; then
      return 0
    fi
  done
  return 1
}
stop_pair
if launch_pair "$model"; then
  printf '{"model":"%s","status":"running","switched_at":"%s"}\n' "$model" "$stamp" > "$run/semantic_model.json"
  printf 'NOMAD_MODEL_SWITCH_RESULT={"state":"MODEL_SWITCHED","model":"%s"}\n' "$model"
else
  printf 'New inference failed; restoring previous model=%s\n' "$previous"
  for label in semantic semantic_adapter; do
    [[ ! -r "$run/${label}_current_log" ]] || tail -n 8 "$(cat "$run/${label}_current_log")"
  done
  stop_pair
  if launch_pair "$previous"; then
    printf '{"model":"%s","status":"restored"}\n' "$previous" > "$run/semantic_model.json"
    printf 'Previous model restored: %s\n' "$previous"
  fi
  fail selected_model_failed
fi
