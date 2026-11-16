#!/usr/bin/env bash
# 2026-11-15 GUI '키보드 조종' 연결 끊기 (Pi4): 팀원 commission_session/stop.sh로 수동 조종 서비스(driver·controller·gui)를 끈다.
# VESC 출력은 제어기 종료 뒤 VESC timeout(300 ms)으로 풀린다. 자율주행 출력(run_planning_output)은 건드리지 않는다.
exec 2>&1
MARK=NOMAD_DRIVE_RESULT=
session=/home/user/nomad_cpp_ws/commission_session
[[ "$(id -un)" == user && -f "$session/stop.sh" ]] || { printf '%s{"ok":false,"reason":"wrong_board"}\n' "$MARK"; exit 0; }
bash "$session/stop.sh" >/dev/null 2>&1
for i in 1 2 3 4 5 6; do
  left=0; for role in driver controller gui; do p=$(cat "$session/$role.pid" 2>/dev/null); [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null && left=1; done
  [[ $left == 0 ]] && break; sleep 0.5
done
if [[ $left == 0 ]]; then printf '%s{"ok":true,"reason":"stopped"}\n' "$MARK"
else printf '%s{"ok":false,"reason":"still_running"}\n' "$MARK"; fi
