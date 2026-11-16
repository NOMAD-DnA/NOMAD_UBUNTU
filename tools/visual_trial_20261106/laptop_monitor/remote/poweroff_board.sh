#!/usr/bin/env bash
# 2026-11-16 GUI '보드 전원 끄기' (보드에서 sudo로 실행, control.poweroff_boards).
# GUI가 맨 앞에 EXPECTED=pi4|pi5|jetson 줄을 붙인다. 이 보드가 그 기종이 아니면 거부한다
# (잘못된 주소 — 예: 노트북 자신 — 은 여기서 멈춘다). 확인되면 디스크를 sync하고
# 3초 뒤 꺼지도록 예약한 뒤 결과 줄을 내고 바로 끝난다(SSH가 정상 종료되도록).
exec 2>&1
MARK=NOMAD_POWER_RESULT=
result() { printf '%s{"ok":%s,"reason":"%s","board":"%s"}\n' "$MARK" "$1" "$2" "${EXPECTED:-}"; exit 0; }
model=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)
case "${EXPECTED:-}" in
  pi4) [[ "$model" == *"Raspberry Pi 4"* ]] || result false wrong_board;;
  pi5) [[ "$model" == *"Raspberry Pi 5"* ]] || result false wrong_board;;
  jetson) [[ -f /etc/nv_tegra_release ]] || result false wrong_board;;
  *) result false invalid_expected_board;;
esac
[[ "$(id -u)" == 0 ]] || result false not_root
sync
if [[ "$EXPECTED" == jetson ]] && findmnt -n /mnt/ssd >/dev/null 2>&1; then
  sync -f /mnt/ssd 2>/dev/null || sync
fi
if command -v systemd-run >/dev/null 2>&1 &&
   systemd-run --quiet --on-active=3 --unit="nomad-poweroff-$$" /bin/systemctl poweroff >/dev/null 2>&1; then
  result true scheduled
fi
# Fallback: shutdown(8) from a detached helper a few seconds later.
if command -v shutdown >/dev/null 2>&1; then
  setsid bash -c 'sleep 3; shutdown -h +0' </dev/null >/dev/null 2>&1 &
  result true scheduled_shutdown
fi
result false no_poweroff_command
