#!/usr/bin/env bash
# 2026-11-15 GUI '키보드 조종' 준비 (Pi4): 팀원 제어기의 수동 조종 서비스를 조종 가능 모드로 켠다.
# 팀원 실행기(linux/start.sh)와 같은 명령. START·속도·조향은 보내지 않는다. 지상 사용은 사용자 결정(2026-11-15).
exec 2>&1
MARK=NOMAD_DRIVE_RESULT=
ensure=/home/user/nomad_cpp_ws/commission_session/ensure_gui.sh
[[ "$(id -un)" == user && -f "$ensure" ]] || { printf '%s{"ok":false,"reason":"wrong_board"}\n' "$MARK"; exit 0; }
out=$(NOMAD_CONTROL_READY=YES NOMAD_LIFTED_TEST=YES bash "$ensure" 2>&1); rc=$?
last=$(printf '%s' "$out" | tail -n 1 | tr '"\\' "'/" | cut -c1-200)
case $rc in
  0) printf '%s{"ok":true,"reason":"ready","message":"%s"}\n' "$MARK" "$last" ;;
  3) printf '%s{"ok":true,"reason":"session_active","message":"%s"}\n' "$MARK" "$last" ;;
  *) printf '%s{"ok":false,"reason":"prepare_failed","message":"%s"}\n' "$MARK" "$last" ;;
esac
