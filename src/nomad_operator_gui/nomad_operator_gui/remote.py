"""SSH control for board scripts. Only scripts started by this GUI are stopped."""
import json
import os
import shlex
import subprocess
import time
import uuid
from dataclasses import dataclass


@dataclass(frozen=True)
class Board:
    name: str
    host: str
    user: str
    script: str


class RemoteError(RuntimeError):
    pass


def invoke(board, action, domain, run_id=None, actuation=False):
    if not board.host:
        raise RemoteError(f'{board.name}: SSH 주소를 입력하세요')
    if not board.script.startswith('/'):
        raise RemoteError(f'{board.name}: 실행 스크립트의 절대 경로가 필요합니다')
    if action not in ('start', 'status', 'stop'):
        raise ValueError(action)
    args = ['env', f'ROS_DOMAIN_ID={domain}', 'ROS_LOCALHOST_ONLY=0']
    if actuation:
        args.append('NOMAD_ENABLE_ACTUATION=1')
    if board.name == 'Jetson':
        # The current Jetson deployment owns one OAK process and requires a run ID.
        args.insert(2, f'NOMAD_ROS_DOMAIN_ID={domain}')
        if action == 'start':
            run_id = uuid.uuid4().hex[:12]
        elif not run_id:
            raise RemoteError('Jetson: 실행 ID가 필요합니다')
        args += [board.script, action, '--run-id', run_id]
    else:
        args += [board.script, action]
        if run_id is not None:
            args.append(run_id)
    command = ' '.join(shlex.quote(arg) for arg in args)
    target = f'{board.user}@{board.host}'
    try:
        key = os.environ.get('NOMAD_SSH_KEY', '')
        ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
               '-o', 'StrictHostKeyChecking=accept-new']
        if key:
            ssh += ['-i', key, '-o', 'IdentitiesOnly=yes']
        result = subprocess.run(
            ssh + [target, command],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=25 if action == 'start' else 12, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RemoteError(f'{board.name}: SSH 실행 실패: {exc}') from exc
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    payload = None
    for line in reversed(lines):
        try:
            candidate = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(candidate, dict) and 'state' in candidate:
            payload = candidate
            break
    if result.returncode or payload is None:
        detail = (result.stderr.strip() or result.stdout.strip() or f'exit {result.returncode}')[-500:]
        raise RemoteError(f'{board.name}: {detail}')
    return payload


def start_sequence(boards, domain, on_progress, owned):
    try:
        for board in boards:
            on_progress(f'{board.name} 시작 중…')
            result = invoke(board, 'start', domain, actuation=board.name == 'Pi 4')
            if result['state'] != 'STARTING':
                raise RemoteError(f"{board.name}: 시작 결과 {result['state']}")
            owned[board.name] = result['run_id']
            on_progress(f'{board.name} 시작 요청 완료')
            if board.name == 'Jetson':
                deadline = time.monotonic() + 130
                last_reason = None
                while True:
                    status = invoke(board, 'status', domain, result['run_id'])
                    if status.get('state') == 'READY' and status.get('ready') is True:
                        on_progress('Jetson 영상·지도 준비 완료')
                        break
                    if status.get('state') in ('FAILED', 'STOPPED'):
                        raise RemoteError(f"Jetson 시작 실패: {status.get('reason', status['state'])}")
                    reason = status.get('reason', '센서·위치·지도 준비 중')
                    if reason != last_reason:
                        on_progress(f'Jetson 준비 중: {reason}')
                        last_reason = reason
                    if time.monotonic() >= deadline:
                        raise RemoteError(f'Jetson 준비 시간 초과: {reason}')
                    time.sleep(2)
        return owned
    except RemoteError:
        for board in reversed(boards):
            if board.name in owned:
                try:
                    stopped = invoke(board, 'stop', domain, owned[board.name])
                    if stopped['state'] == 'STOPPED':
                        del owned[board.name]
                except RemoteError as exc:
                    on_progress(f'중지 확인 필요: {exc}')
        raise


def stop_sequence(boards, domain, owned, on_progress):
    errors = []
    for board in reversed(boards):
        if board.name not in owned:
            continue
        on_progress(f'{board.name} 중지 중…')
        try:
            result = invoke(board, 'stop', domain, owned[board.name])
            if result['state'] == 'STOPPED':
                del owned[board.name]
            else:
                errors.append(f"{board.name}: {result['state']}")
        except RemoteError as exc:
            errors.append(str(exc))
    return errors
