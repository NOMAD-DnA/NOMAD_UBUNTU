#!/usr/bin/env bash
# 2026-11-15 GUI '센서 녹화'가 켠 Pi5 GPS 수신기만 끈다.
# 2026-11-16: 대상 = 이 계정 프로세스 중 환경 ROS_LOG_DIR가 $ws/log/record_gnss_*/ros 인 것
# (record_gnss_start_pi.sh가 켠 수신기와 그 자식). 표시(/tmp/nomad_record_gnss_run)가 없어도 정리한다.
# 시험 수신기(visual_trial_*)는 건드리지 않는다. INT → TERM → KILL, 매 신호 전 pid 신원(시작 시각·그룹) 재확인.
# 표시는 수신기가 모두 사라진 것을 확인한 뒤에만 지운다. 남으면 survivors로 알린다.
exec 2>&1
MARK=NOMAD_RECORD_RESULT=
ws=/home/user/nomad_ws
marker=/tmp/nomad_record_gnss_run
declare -A starts=() groups=()
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
# True when the process was started under a record_gnss_* run folder.
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
capture() {
  local pid uid
  while read -r pid uid; do
    [[ "$uid" == "$current_uid" && "$pid" != "$$" && "$pid" != "$BASHPID" && -z ${starts[$pid]+yes} ]] || continue
    record_gnss_owned "$pid" || continue
    identity "$pid" && [[ "$process_uid" == "$current_uid" ]] || continue
    starts[$pid]=$process_start
    groups[$pid]=$process_group
  done < <(ps -eo pid=,uid=)
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
# Comma list of captured (or newly started) receiver processes still alive.
survivor_list() {
  local pid list=""
  capture
  for pid in "${!starts[@]}"; do
    same_process "$pid" && list+="${list:+,}gnss:$pid"
  done
  printf '%s' "$list"
}

main() {
  [[ "$(id -un)" == user ]] || { printf '%s{"ok":false,"reason":"wrong_board"}\n' "$MARK"; return 0; }
  local had_marker=false found left
  [[ -f "$marker" ]] && had_marker=true
  capture
  found=${#starts[@]}
  if (( found == 0 )); then
    rm -f "$marker"
    printf '%s{"ok":true,"reason":"%s","survivors":""}\n' "$MARK" "$([[ $had_marker == true ]] && echo stopped || echo not_started_by_gui)"
    return 0
  fi
  # Budget (GUI 20 s): INT 3 s + TERM 1 s + KILL 0.5 s.
  signal_stage INT; wait_gone 30
  signal_stage TERM; wait_gone 10
  signal_stage KILL; wait_gone 5
  left=$(survivor_list)
  if [[ -n "$left" ]]; then
    printf '%s{"ok":false,"reason":"stop_failed","targets":%s,"survivors":"%s"}\n' "$MARK" "$found" "$left"
    return 0
  fi
  rm -f "$marker"
  printf '%s{"ok":true,"reason":"stopped","targets":%s,"marker":%s,"survivors":""}\n' "$MARK" "$found" "$had_marker"
}

# MAIN_ENTRY: local tests load the functions above without calling this entry.
main "$@"
