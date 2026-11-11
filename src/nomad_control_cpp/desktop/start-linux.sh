#!/usr/bin/env bash
set -euo pipefail
desktop_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_dir=$(dirname -- "$desktop_dir")
if [[ ${1:-} == --check ]]; then
    bash "$project_dir/linux/start.sh" --check
    command -v cmake >/dev/null
    echo 'Qt 소스 및 실행기 준비 완료 (차량 명령 없음)'
    exit 0
fi
# 해당 Linux에서 직접 빌드하여 다른 배포판의 glibc/Qt 버전을 묶지 않습니다.
if ! command -v cmake >/dev/null || ! command -v c++ >/dev/null; then
    echo '최초 설치: sudo apt install build-essential cmake qt6-base-dev openssh-client curl util-linux' >&2
    exit 1
fi
cmake -S "$desktop_dir" -B "$desktop_dir/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$desktop_dir/build" -j2
if [[ ${1:-} == --simulate ]]; then exec "$desktop_dir/build/nomad_desktop" --simulate; fi
bash "$project_dir/linux/start.sh" --no-browser
# 기존 SSH 연결을 재사용해 읽기 전용 기록기를 준비합니다. 실패해도 조종기를 종료하지 않습니다.
if ! (
    NOMAD_DIR="$project_dir/linux"
    source "$NOMAD_DIR/common.sh"
    nomad_load_config
    nomad_connection
    recorder="$PI_WORKSPACE/diagnostics/start_recording.sh"
    ssh -n -T -o BatchMode=yes -o NumberOfPasswordPrompts=0 -o ConnectTimeout=3 \
        -o ConnectionAttempts=1 -o ServerAliveInterval=2 -o ServerAliveCountMax=1 \
        "${SSH_OPTIONS[@]}" -S "$CONTROL_SOCKET" "$TARGET" \
        "if test -f '$recorder'; then timeout 3s bash '$recorder'; else echo 'Pi recording helper is not installed.' >&2; exit 4; fi"
); then
    echo '시험 자동 기록을 시작하지 못했습니다. Pi 기록기 설치와 SSH 연결을 확인하세요.' >&2
fi
exec "$desktop_dir/build/nomad_desktop"
