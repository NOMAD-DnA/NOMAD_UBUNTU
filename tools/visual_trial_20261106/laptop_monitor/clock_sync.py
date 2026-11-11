#!/usr/bin/env python3
"""Board time readiness before starting board modules (2026-11-11).

The laptop no longer sets board clocks. Pi5 is the time reference and Jetson/
Pi4 follow it with chrony; each board judges itself with the installed
board_network/time_agent.py --check (client |offset| <= 5 ms to Pi5, Pi5 must be
serving). This module only asks the board and reports its own answer. No sudo.
The laptop clock does not matter.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import os
from pathlib import Path
import shlex
import socket
import subprocess
import tempfile

from control import validate_host

MARKER = 'NOMAD_TIME_RESULT='
FEATURE = 'nomad-board-time-20261111-1'
CHECK = '/usr/local/lib/nomad-network/time_agent.py'
REMOTE = ('if [ -x ' + CHECK + ' ]; then exec python3 ' + CHECK + ' --check; '
          'else echo "' + MARKER + '{\\"ok\\": false, \\"feature\\": \\"' + FEATURE +
          '\\", \\"reason\\": \\"time_sync_not_installed\\"}"; fi')
REASON_TEXT = {
    'following_pi5_within_5ms': 'Pi5 기준 5ms 이내',
    'reference_serving': 'Pi5 기준 시계 동작 (인터넷 시간 동기화됨)',
    'reference_serving_local_clock': 'Pi5 기준 시계 동작 (인터넷 없음, 자체 시계)',
    'time_sync_not_installed': '보드 시간 동기화 미설치 / 연결 창에서 설치',
    'chrony_not_installed': 'chrony 미설치 / 연결 창에서 설치',
    'chrony_not_running': 'chrony가 꺼져 있음',
    'not_following_pi5': 'Pi5를 따라가지 못함 (연결 확인)',
    'pi5_update_stale': 'Pi5 시각을 최근에 받지 못함',
    'offset_over_5ms': 'Pi5와 5ms 넘게 차이 (잠시 뒤 다시 시도)',
    'time_connection_failed': '보드 접속 실패',
    'time_timeout': '보드 응답 시간 초과',
    'time_result_invalid': '보드 응답 확인 불가',
    'pi5_address_not_on_my_network': 'GUI의 Pi5 주소가 이 보드와 다른 망 (Pi5 주소 확인)',
    'invalid_pi5_address': 'Pi5 주소가 올바르지 않음',
    'pi5_address_unresolved': 'GUI의 Pi5 주소를 IP로 확인하지 못함',
    'pi5_update_needs_agent': 'Pi5 주소 갱신 기능 없음 / 연결 창에서 시간 동기화 다시 설치',
}


def describe(result):
    text = REASON_TEXT.get(result.get('reason', ''), result.get('reason', '미확인'))
    if result.get('offset_ms') is not None and result.get('board') != 'pi5':
        text += ' / 차이 {:+.2f}ms'.format(result['offset_ms'])
    return text


def parse(stdout):
    lines = [line[len(MARKER):] for line in stdout.splitlines() if line.startswith(MARKER)]
    if len(lines) != 1:
        return dict(ok=False, reason='time_result_invalid', state='FAILED')
    try:
        value = json.loads(lines[0])
        if not isinstance(value, dict) or value.get('feature') != FEATURE:
            raise ValueError()
    except ValueError:
        return dict(ok=False, reason='time_result_invalid', state='FAILED')
    result = {k: value[k] for k in ('board', 'reference', 'offset_ms', 'last_offset_ms', 'update_age_s', 'wired',
                                     'stratum', 'updated', 'pi5_address') if k in value}
    servers = value.get('servers')
    if isinstance(servers, list):
        result['servers'] = [str(s)[:64] for s in servers[:8]]
    result.update(ok=value.get('ok') is True, reason=str(value.get('reason', 'time_result_invalid'))[:80],
                  state='SUCCEEDED' if value.get('ok') is True else 'FAILED')
    result['message'] = describe(result)
    return result


def synchronize_clock(host, password, timeout_s=15, pi5_host=None):
    """Ask the board for its own time readiness (name kept for existing callers).
    pi5_host: for Pi4/Jetson, the Pi5 address the GUI uses; if the board fails
    because it still polls an old Pi5 address, it is given this one (sudo)."""
    try:
        host = validate_host(host)
        password = password or ''
        if not isinstance(password, str) or '\n' in password or '\r' in password:
            raise ValueError('Invalid password')
    except ValueError:
        return {'ok': False, 'reason': 'clock_arguments', 'state': 'FAILED'}
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'nomad-trial'
    state.mkdir(parents=True, exist_ok=True, mode=0o700)
    try:
        with tempfile.TemporaryDirectory(prefix='nomad-time-') as temporary:
            env = os.environ.copy()
            if password:
                askpass = Path(temporary) / 'askpass'
                askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_SSH_PASSWORD"\n')
                askpass.chmod(0o700)
                env.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force', NOMAD_SSH_PASSWORD=password)
                env.setdefault('DISPLAY', ':0')
            args = ['ssh', '-T', '-o', 'ConnectTimeout=5', '-o', 'NumberOfPasswordPrompts=1',
                    '-o', 'StrictHostKeyChecking=accept-new', '-o', 'LogLevel=ERROR',
                    '-o', 'UserKnownHostsFile="' + str(state / 'known_hosts') + '"',
                    '-o', 'BatchMode=' + ('no' if password else 'yes'), host, 'bash -c ' + shlex.quote(REMOTE)]
            completed = subprocess.run(args, stdin=subprocess.DEVNULL, capture_output=True, text=True,
                                       env=env, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return dict(ok=False, reason='time_timeout', state='FAILED', message=REASON_TEXT['time_timeout'])
    except OSError:
        return dict(ok=False, reason='time_connection_failed', state='FAILED', message=REASON_TEXT['time_connection_failed'])
    if completed.returncode == 255 and MARKER not in completed.stdout:
        return dict(ok=False, reason='time_connection_failed', state='FAILED', message=REASON_TEXT['time_connection_failed'])
    result = parse(completed.stdout)
    if pi5_host and not result['ok'] and result.get('board') in ('pi4', 'jetson'):
        result = _learn_pi5(result, result['board'], host, password, pi5_host)
    return result


def pi5_ipv4(pi5_host):
    """IPv4 the GUI uses for Pi5 (user@1.2.3.4 or user@name.local)."""
    name = validate_host(pi5_host).split('@', 1)[1]
    try:
        return str(ipaddress.IPv4Address(name))
    except ValueError:
        pass
    try:
        return socket.getaddrinfo(name, None, socket.AF_INET)[0][4][0]
    except (OSError, IndexError):
        return None


def _learn_pi5(result, board, host, password, pi5_host):
    """Follower does not track the Pi5 address the GUI just used: give it that
    address (sudo, board updates peers + chrony itself), then use its new check."""
    address = pi5_ipv4(pi5_host)
    if not address:
        return dict(result, reason='pi5_address_unresolved', message=describe({'reason': 'pi5_address_unresolved'}))
    if address in result.get('servers', []) or not password:
        return result
    import network_control
    updated = network_control.time_request(board, host, password, 'pi5_address', timeout_s=60, address=address)
    if updated.get('reason') == 'invalid_request':
        updated = dict(updated, reason='pi5_update_needs_agent', message=REASON_TEXT['pi5_update_needs_agent'])
    prefix = 'Pi5 주소 {} 로 갱신 · '.format(address) if updated.get('updated') else ''
    updated['message'] = prefix + updated.get('message', describe(updated))
    return updated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--pi5-host', default='', help='GUI의 Pi5 주소: 추종 보드가 옛 주소를 보면 알려 줌')
    args = parser.parse_args()
    result = synchronize_clock(args.host, os.environ.get('NOMAD_BOARD_PASSWORD1', ''), pi5_host=args.pi5_host or None)
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
