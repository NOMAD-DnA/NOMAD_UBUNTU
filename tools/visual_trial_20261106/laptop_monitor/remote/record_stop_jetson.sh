#!/usr/bin/env bash
# 2026-11-15 GUI '센서 녹화' 정지 (Jetson): stop_record.sh(SIGINT → metadata.yaml) 후,
# GUI 녹화가 켠 센서(GUI_SENSORS 표시)만 끈다. GUI 시험이 켠 센서는 그대로 둔다.
# 2026-11-16: 센서 정지는 보드 stop_sensors.sh 대신 이 스크립트가 직접 한다(보드 파일은 그대로).
#  - 대상: 이 계정 프로세스 중 환경 ROS_LOG_DIR가 그 센서 run 폴더 아래이거나, 프로세스 그룹이
#    run/*.pid(이번 부팅)에 적힌 그룹인 것. 리더가 먼저 끝나 남은 ZED 컨테이너도 포함.
#  - 리뷰 반영: pid 파일 그룹은 그 그룹에 ROS_LOG_DIR가 run 폴더인 프로세스가 하나 이상 있을 때만 인정
#    (리더가 끝난 뒤 같은 번호가 다른 그룹에 재사용된 경우 제외).
#  - INT → 5초 → TERM → 2초 → KILL, 매 신호 전 pid 신원(시작 시각·그룹) 재확인. 남으면 survivors로 알린다.
#  - RECORDING이 없어도 GUI_SENSORS가 있으면 끈다. 남은 게 있으면 표시를 지우지 않는다(다시 정지 가능).
set -o pipefail
exec 2>&1
MARK=NOMAD_RECORD_RESULT=
SSD=/mnt/ssd; REC=$SSD/ws/recording
declare -A starts=() groups=() leaders=() env_groups=()
current_uid=$(id -u)

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
# Group leaders written by start_sensors.sh in this boot only (older pid numbers may be reused).
load_leaders() {
  local run=$1 file pid btime
  btime=$(awk '/^btime/{print $2}' /proc/stat)
  for file in "$run"/*.pid; do
    [[ -f "$file" && ! -L "$file" && $(stat -c %u "$file") == "$current_uid" && $(stat -c %Y "$file") -ge ${btime:-0} ]] || continue
    pid=$(head -c 32 "$file" | tr -dc 0-9)
    [[ -n "$pid" ]] && leaders[$pid]=1
  done
}
run_env() {
  local pid=$1 run=$2 item
  local -a envs=()
  mapfile -d '' -t envs < "/proc/$pid/environ" 2>/dev/null || return 1
  for item in "${envs[@]}"; do
    [[ "$item" == "ROS_LOG_DIR=$run/"* ]] && return 0
  done
  return 1
}
run_owned() {
  local pid=$1 group=$2 run=$3
  [[ -n ${leaders[$group]+yes} && -n ${env_groups[$group]+yes} ]] && return 0
  run_env "$pid" "$run"
}
capture_run() {
  local run=$1 pid group uid
  # Groups proven to belong to this run by at least one member's environment.
  while read -r pid group uid; do
    [[ "$uid" == "$current_uid" && -n ${leaders[$group]+yes} ]] || continue
    run_env "$pid" "$run" && env_groups[$group]=1
  done < <(ps -eo pid=,pgid=,uid=)
  while read -r pid group uid; do
    [[ "$uid" == "$current_uid" && "$pid" != "$$" && "$pid" != "$BASHPID" && -z ${starts[$pid]+yes} ]] || continue
    run_owned "$pid" "$group" "$run" || continue
    identity "$pid" && [[ "$process_uid" == "$current_uid" ]] || continue
    starts[$pid]=$process_start
    groups[$pid]=$process_group
  done < <(ps -eo pid=,pgid=,uid=)
}
signal_stage() {
  local pid
  for pid in "${!starts[@]}"; do
    same_process "$pid" && kill -s "$1" -- "$pid" 2>/dev/null
  done
  return 0
}
wait_gone() {
  local tenths=$1 i pid alive
  for ((i=0;i<tenths;i++)); do
    alive=0
    for pid in "${!starts[@]}"; do same_process "$pid" && { alive=1; break; }; done
    (( alive )) || return 0
    sleep 0.1
  done
}
# Stops the sensors of one run folder; prints the survivors ("name:pid,…", empty when all gone).
stop_run_sensors() {
  local run=$1 pid list="" name
  load_leaders "$run"
  capture_run "$run"
  if (( ${#starts[@]} )); then
    signal_stage INT; wait_gone 50
    signal_stage TERM; wait_gone 20
    signal_stage KILL; wait_gone 5
  fi
  capture_run "$run"   # children started meanwhile
  for pid in "${!starts[@]}"; do
    same_process "$pid" || continue
    name=$(tr -dc 'A-Za-z0-9_.-' < "/proc/$pid/comm" 2>/dev/null)
    list+="${list:+,}${name:-pid}:$pid"
  done
  printf '%s' "$list"
}

main() {
  local session out rc info duration messages gps size sensors survivors="" run ok reason
  [[ "$(id -un)" == nvidia && -d "$REC" ]] || { printf '%s{"ok":false,"reason":"wrong_board"}\n' "$MARK"; return 0; }
  session=$(cat "$SSD/bags/RECORDING" 2>/dev/null || true)
  if [[ -n "$session" ]]; then
    out=$(bash "$REC/stop_record.sh" 2>&1); rc=$?
    info="$session/bag_info.txt"
    duration=$(sed -n 's/^Duration: *\([0-9.]*\)s.*/\1/p' "$info" 2>/dev/null | head -1)
    messages=$(sed -n 's/^Messages: *\([0-9]*\).*/\1/p' "$info" 2>/dev/null | head -1)
    gps=$(grep -E 'Topic: /gps/fix ' "$info" 2>/dev/null | sed -n 's/.*Count: *\([0-9]*\).*/\1/p' | head -1)
    size=$(du -sb "$session" 2>/dev/null | cut -f1)
  fi
  sensors=kept
  if [[ -f "$SSD/bags/GUI_SENSORS" ]]; then
    run=$(head -c 512 "$SSD/bags/GUI_SENSORS" | head -1)
    if [[ -f "$SSD/bags/RECORDING" ]]; then
      sensors=recording_active          # never stop sensors under a running recorder
    elif [[ "$run" == "$SSD"/experiments/sensor_runs/* && "$run" != *..* && -d "$run" && ! -L "$run" ]]; then
      survivors=$(stop_run_sensors "$run")
      if [[ -n "$survivors" ]]; then sensors=stop_incomplete
      else sensors=stopped; rm -f "$SSD/bags/GUI_SENSORS"; fi
    else
      sensors=invalid_marker; rm -f "$SSD/bags/GUI_SENSORS"
    fi
  fi
  if [[ -z "$session" ]]; then
    printf '%s{"ok":false,"reason":"not_recording","sensors":"%s","survivors":"%s"}\n' "$MARK" "$sensors" "$survivors"
    return 0
  fi
  ok=false; [[ $rc == 0 && -f "$session/bag/metadata.yaml" ]] && ok=true
  reason=stop_incomplete
  if [[ $ok == true ]]; then
    reason=stopped; [[ -n "$survivors" ]] && reason=sensors_remaining
  fi
  printf '%s{"ok":%s,"reason":"%s","session":"%s","duration_s":%s,"messages":%s,"gps_fix":%s,"bytes":%s,"sensors":"%s","survivors":"%s"}\n' \
    "$MARK" "$ok" "$reason" "$session" \
    "${duration:-null}" "${messages:-null}" "${gps:-0}" "${size:-null}" "$sensors" "$survivors"
}

# MAIN_ENTRY: local tests load the functions above without calling this entry.
main "$@"
