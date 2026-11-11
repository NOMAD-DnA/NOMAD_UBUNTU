#!/usr/bin/env bash
set -euo pipefail
NOMAD_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$NOMAD_DIR/common.sh"
nomad_load_config
echo 'NOMAD 연결 설정 (비밀번호는 저장하지 않습니다)'
read -r -p "Pi 주소 [$PI_HOST]: " value; PI_HOST=${value:-$PI_HOST}
read -r -p "Pi 계정 [$PI_USER]: " value; PI_USER=${value:-$PI_USER}
read -r -p "Pi 작업 폴더 [$PI_WORKSPACE]: " value; PI_WORKSPACE=${value:-$PI_WORKSPACE}
read -r -p 'SSH 개인 키 절대 경로 (비밀번호 로그인은 Enter): ' SSH_KEY
nomad_validate_config
umask 077
printf 'PI_HOST=%s\nPI_USER=%s\nPI_WORKSPACE=%s\nSSH_KEY=%s\n' "$PI_HOST" "$PI_USER" "$PI_WORKSPACE" "$SSH_KEY" > "$NOMAD_DIR/connection.local.conf"
echo '저장했습니다. ./start.sh로 연결하세요.'
