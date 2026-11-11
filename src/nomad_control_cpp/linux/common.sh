#!/usr/bin/env bash
# 설정을 source/eval하지 않습니다. 설정 파일의 문자를 명령으로 실행하지 않습니다.
nomad_load_config() {
    local file="$NOMAD_DIR/connection.conf" key value
    [[ ! -f "$NOMAD_DIR/connection.local.conf" ]] || file="$NOMAD_DIR/connection.local.conf"
    PI_HOST= PI_USER= PI_WORKSPACE= SSH_KEY=
    while IFS='=' read -r key value || [[ -n "$key" ]]; do
        value=${value%$'\r'}
        case "$key" in
            ''|'#'*) continue ;;
            PI_HOST) PI_HOST=$value ;;
            PI_USER) PI_USER=$value ;;
            PI_WORKSPACE) PI_WORKSPACE=$value ;;
            SSH_KEY) SSH_KEY=$value ;;
            *) echo "알 수 없는 설정: $key" >&2; return 1 ;;
        esac
    done < "$file"
    nomad_validate_config
}
nomad_validate_config() {
    [[ "$PI_HOST" =~ ^[a-zA-Z0-9][a-zA-Z0-9.-]*$ ]] || { echo 'Pi 주소를 확인하세요.' >&2; return 1; }
    [[ "$PI_USER" =~ ^[a-z_][a-z0-9_-]*$ ]] || { echo 'Pi 계정을 확인하세요.' >&2; return 1; }
    [[ "$PI_WORKSPACE" =~ ^/[a-zA-Z0-9_./-]+$ && ! "$PI_WORKSPACE" =~ (^|/)\.\.(/|$) ]] || { echo 'Pi 작업 폴더를 확인하세요.' >&2; return 1; }
    [[ -z "$SSH_KEY" || -f "$SSH_KEY" ]] || { echo 'SSH 키 파일이 없습니다.' >&2; return 1; }
}
nomad_dependencies() {
    local program
    for program in ssh curl flock sha256sum; do
        command -v "$program" >/dev/null || { echo "필요한 프로그램: $program (Ubuntu/Debian: sudo apt install openssh-client curl util-linux coreutils)" >&2; return 1; }
    done
}
nomad_connection() {
    # 짧은 Unix 소켓 경로를 사용하고 현재 사용자만 접근하도록 합니다.
    local runtime="${XDG_RUNTIME_DIR:-/tmp}/nomad-gui-$UID" profile
    if [[ -e "$runtime" || -L "$runtime" ]]; then
        [[ -d "$runtime" && -O "$runtime" && ! -L "$runtime" ]] || { echo 'NOMAD 연결 폴더의 소유권을 확인하세요.' >&2; return 1; }
    else
        (umask 077; mkdir "$runtime") || return 1
    fi
    chmod 700 "$runtime"
    profile=$(printf '%s\n' "$PI_HOST" "$PI_USER" "$PI_WORKSPACE" "$SSH_KEY" | sha256sum)
    CONTROL_SOCKET="$runtime/ctl-${profile:0:20}"
    CONNECTION_LOCK="$runtime/start.lock"
    TARGET="$PI_USER@$PI_HOST"
    SSH_OPTIONS=(-o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 -o StrictHostKeyChecking=ask)
    if [[ -n "$SSH_KEY" ]]; then SSH_OPTIONS+=(-i "$SSH_KEY" -o IdentitiesOnly=yes); fi
}
nomad_master_alive() { ssh "${SSH_OPTIONS[@]}" -S "$CONTROL_SOCKET" -O check "$TARGET" >/dev/null 2>&1; }
nomad_close_master() { ssh "${SSH_OPTIONS[@]}" -S "$CONTROL_SOCKET" -O exit "$TARGET" >/dev/null 2>&1; }
