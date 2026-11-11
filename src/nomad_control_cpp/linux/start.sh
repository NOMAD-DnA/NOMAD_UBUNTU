#!/usr/bin/env bash
set -euo pipefail
NOMAD_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$NOMAD_DIR/common.sh"
no_browser=false check_only=false desktop=false created=false success=false
for arg in "$@"; do
    case "$arg" in
        --no-browser) no_browser=true ;;
        --check) check_only=true ;;
        --desktop) desktop=true ;;
        *) echo '사용법: ./start.sh [--no-browser] [--check]' >&2; exit 1 ;;
    esac
done
cleanup() {
    local result=$?
    # 실패한 새 연결만 닫습니다. 기존 연결/다른 프로그램은 건드리지 않습니다.
    if $created && ! $success; then nomad_close_master || true; fi
    if ((result != 0)) && $desktop && [[ -t 0 ]]; then read -r -p 'Enter로 닫기' _ || true; fi
    return "$result"
}
trap cleanup EXIT
nomad_load_config
nomad_dependencies
if $check_only; then echo "설정 확인 완료: $PI_USER@$PI_HOST (연결/차량 명령 없음)"; exit 0; fi
nomad_connection
exec 9>"$CONNECTION_LOCK"
flock -n 9 || { echo '다른 NOMAD 실행기가 연결 중입니다. 잠시 후 다시 실행하세요.' >&2; exit 1; }
if ! nomad_master_alive; then
    echo "NOMAD 연결: $TARGET"
    echo 'SSH 비밀번호를 입력하세요. 키 로그인이 설정되어 있으면 자동 연결됩니다.'
    # 로그인은 터미널에서 직접 처리하고, 인증 성공 후 터널만 백그라운드로 유지합니다.
    ssh "${SSH_OPTIONS[@]}" -M -S "$CONTROL_SOCKET" -fNT -o ExitOnForwardFailure=yes \
        -L 127.0.0.1:8765:127.0.0.1:8765 "$TARGET" 9>&-
    created=true
fi
remote="NOMAD_CONTROL_READY=YES NOMAD_LIFTED_TEST=YES bash $PI_WORKSPACE/commission_session/ensure_gui.sh"
# 준비 스크립트가 종료될 때까지 기다립니다. START/속도/조향 명령은 전송하지 않습니다.
result=0
ssh "${SSH_OPTIONS[@]}" -S "$CONTROL_SOCKET" "$TARGET" "$remote" 9>&- || result=$?
if ((result != 0 && result != 3)); then
    echo 'Pi 준비 실패. 위 메시지와 Pi/VESC 전원·USB 연결을 확인하세요.' >&2; exit "$result"
fi
curl --fail --silent --show-error --max-time 5 --output /dev/null http://127.0.0.1:8765/api/status
success=true
if ((result == 3)); then
    echo '차량이 이미 조종 중입니다. 기존 조종자가 STOP한 뒤 제어를 시작하세요.'
else
    echo '연결 완료. GUI에서 수동 제어 시작을 누르세요.'
fi
echo 'http://127.0.0.1:8765/'
if ! $no_browser; then
    if command -v xdg-open >/dev/null; then
        xdg-open http://127.0.0.1:8765/ >/dev/null 2>&1 9>&- &
    elif command -v gio >/dev/null; then
        gio open http://127.0.0.1:8765/ >/dev/null 2>&1 9>&- &
    else
        echo '브라우저에서 위 주소를 여세요.'
    fi
fi
