"""Credential-safe SSH transport for the installed common network agent."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import tempfile
from control import validate_host

FEATURE = 'nomad-gui-hotspot-20261110-1'
MARKER = 'NOMAD_NETWORK_RESULT='
BOARDS = {'pi4': 'Raspberry Pi 4', 'pi5': 'Raspberry Pi 5', 'jetson': 'Jetson'}
AGENT = '/usr/local/lib/nomad-network/network_agent.py'
WRAPPER = r'''set -eu
umask 077
IFS= read -r nomad_sudo_password
nomad_net_dir=$(mktemp -d /tmp/nomad-wifi-request.XXXXXXXX)
trap 'rm -rf -- "$nomad_net_dir"' EXIT
cat > "$nomad_net_dir/request.json"
printf '%s\n' "$nomad_sudo_password" | sudo -k -S -p '' python3 /usr/local/lib/nomad-network/network_agent.py --request-file "$nomad_net_dir/request.json" "${SSH_CONNECTION:-}"
'''
REASONS = {'invalid_request', 'wrong_board', 'wrong_machine', 'invalid_credentials', 'invalid_ssid',
           'invalid_wifi_password', 'networkmanager_unavailable', 'systemd_unavailable', 'select_wifi_interface',
           'wifi_radio_disabled', 'previous_wifi_not_connected', 'switch_pending', 'stale_request', 'not_prepared',
           'previous_connection_changed', 'schedule_failed', 'invalid_timeout', 'not_awaiting_confirmation',
           'reconnect_via_wifi_address', 'confirmation_expired', 'restore_unavailable', 'agent_error',
           'invalid_peer_addresses', 'peer_address_not_current', 'network_command_failed',
           'command_timeout_or_unavailable', 'root_required', 'unsupported_board', 'board_peer_unreachable', 'connection_not_confirmed'}
STATES = {'idle', 'prepared', 'queued', 'switching', 'awaiting_confirmation', 'committed',
          'restore_queued', 'rolling_back', 'rolled_back', 'rollback_failed', 'schedule_failed'}


def failed(reason):
    return {'ok': False, 'reason': reason}


def validate_credentials(ssid, password):
    from board_network.network_agent import credentials, NetworkError
    try:
        return credentials(ssid, password)
    except (NetworkError, AttributeError, UnicodeError):
        raise ValueError('SSID는 UTF-8 기준 1~32바이트, 비밀번호는 WPA2 기준 ASCII 8~63자 또는 64자리 16진수입니다.') from None


def parse_result(stdout, returncode, board, machine_id=None):
    lines = [line[len(MARKER):] for line in stdout.splitlines() if line.startswith(MARKER)]
    if len(lines) != 1:
        return failed('agent_missing_or_invalid_result')
    try:
        value = json.loads(lines[0])
        if not isinstance(value, dict) or value.get('feature') != FEATURE:
            raise ValueError()
        if value.get('ok') is not True:
            return failed(value.get('reason') if value.get('reason') in REASONS else 'remote_network_error')
        if returncode != 0 or value.get('board') != board or value.get('status') not in STATES:
            raise ValueError()
        machine = value.get('machine_id')
        if not isinstance(machine, str) or not re.fullmatch('[0-9a-f]{32}', machine):
            raise ValueError()
        if machine_id and machine_id != machine:
            return failed('wrong_machine')
        addresses = value.get('ipv4')
        if not isinstance(addresses, list) or len(addresses) > 16:
            raise ValueError()
        addresses = [str(ipaddress.IPv4Address(item)) for item in addresses]
        token = value.get('token')
        if token is not None and (not isinstance(token, str) or not re.fullmatch('[0-9a-f]{32}', token)):
            raise ValueError()
        result = {key: value[key] for key in ('ok', 'board', 'machine_id', 'status', 'feature')}
        result.update(ipv4=addresses, token=token, target_active=value.get('target_active') is True)
        for key in ('ssid', 'active_ssid', 'interface', 'mac'):
            if key in value:
                if not isinstance(value[key], str) or len(value[key]) > 128 or any(ord(c) < 32 for c in value[key]):
                    raise ValueError()
                result[key] = value[key]
        if type(value.get('seconds_until_restore')) is int:
            result['seconds_until_restore'] = max(0, min(900, value['seconds_until_restore']))
        if value.get('error') in ('activation_failed', 'confirmation_expired', 'user_restore', 'restore_failed', 'schedule_failed'):
            result['error'] = value['error']
        return result
    except (ValueError, TypeError, KeyError):
        return failed('agent_missing_or_invalid_result')


def request(board, host, ssh_password, action='status', *, timeout_s=24, **options):
    if board not in BOARDS or action not in ('status', 'prepare', 'switch', 'confirm', 'restore', 'peers'):
        return failed('invalid_request')
    try:
        host = validate_host(host)
        if host.split('@')[0] != ('nvidia' if board == 'jetson' else 'user'):
            return failed('invalid_board_address')
        if not isinstance(ssh_password, str) or not ssh_password or any(c in ssh_password for c in '\r\n\x00'):
            return failed('ssh_password_required')
        if action == 'prepare':
            validate_credentials(options.get('ssid'), options.get('password'))
        allowed = {'ssid', 'password', 'token', 'machine_id', 'interface', 'timeout', 'peers'}
        if set(options) - allowed or (action != 'prepare' and 'password' in options):
            return failed('invalid_request')
        payload = ssh_password + '\n' + json.dumps(dict(board=board, action=action, **options), ensure_ascii=False) + '\n'
        state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'nomad-trial'
        state.mkdir(parents=True, exist_ok=True, mode=0o700)
        with tempfile.TemporaryDirectory(prefix='nomad-wifi-ssh-') as temporary:
            askpass = Path(temporary) / 'askpass'
            askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_SSH_PASSWORD"\n')
            askpass.chmod(0o700)
            env = dict(os.environ, SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force', NOMAD_SSH_PASSWORD=ssh_password)
            env.setdefault('DISPLAY', ':0')
            # A new TCP/SSH connection proves reachability after a Wi-Fi move.
            args = ['ssh', '-T', '-o', 'ControlMaster=no', '-o', 'ControlPath=none',
                    '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1', '-o', 'ServerAliveInterval=3',
                    '-o', 'ServerAliveCountMax=1', '-o', 'NumberOfPasswordPrompts=1',
                    '-o', 'StrictHostKeyChecking=accept-new', '-o', 'UserKnownHostsFile="' + str(state / 'known_hosts') + '"',
                    '-o', 'LogLevel=ERROR', host, 'bash -c ' + shlex.quote(WRAPPER)]
            process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       text=True, env=env, start_new_session=True)
            try:
                out, err = process.communicate(payload, timeout=timeout_s)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.communicate()
                return failed('ssh_timeout_unconfirmed')
        if process.returncode == 255:
            return failed('authentication_failed' if 'permission denied' in err.lower() else
                          'host_key_check_failed' if 'identification has changed' in err.lower() or 'host key verification failed' in err.lower() else
                          'ssh_unreachable')
        return parse_result(out, process.returncode, board, options.get('machine_id'))
    except (OSError, TypeError, ValueError):
        return failed('invalid_credentials_or_local_ssh_error')


def connect_many(targets, passwords, ssid, password, timeout=300, progress=None):
    """All preparations must succeed before any board is scheduled to leave."""
    validate_credentials(ssid, password)
    prepared = {}
    for board, host in targets.items():
        value = request(board, host, passwords[board], 'prepare', ssid=ssid, password=password)
        prepared[board] = value
        if progress:
            progress(board, value)
    if not all(v.get('ok') and v.get('status') == 'prepared' and v.get('token') for v in prepared.values()):
        return {'ok': False, 'phase': 'prepare', 'boards': prepared}
    switched = {}
    # Schedule together so all requests finish before the 8-second grace period.
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures = {pool.submit(request, board, targets[board], passwords[board], 'switch',
                               token=value['token'], machine_id=value['machine_id'], timeout=timeout): board
                   for board, value in prepared.items()}
        for future in as_completed(futures):
            board = futures[future]
            try:
                value = future.result()
            except Exception:
                value = failed('local_network_control_error')
            # Retain preparation identity after an ambiguous/disconnected reply.
            switched[board] = dict(value, token=prepared[board]['token'], machine_id=prepared[board]['machine_id'])
            if progress:
                progress(board, switched[board])
    return {'ok': all(v.get('ok') and v.get('status') == 'queued' for v in switched.values()),
            'phase': 'switch', 'boards': switched}
