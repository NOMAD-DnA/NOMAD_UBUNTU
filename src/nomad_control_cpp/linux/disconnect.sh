#!/usr/bin/env bash
set -euo pipefail
NOMAD_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$NOMAD_DIR/common.sh"
nomad_load_config
nomad_dependencies
nomad_connection
exec 9>"$CONNECTION_LOCK"
flock -n 9 || { echo '연결 준비가 끝난 후 다시 실행하세요.' >&2; exit 1; }
if nomad_master_alive; then nomad_close_master; echo '이 PC의 NOMAD SSH 연결을 종료했습니다.'; else echo '종료할 NOMAD 연결이 없습니다.'; fi
