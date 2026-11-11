"""Board Wi-Fi agent transport and this laptop's own hotspot control.

Boards: credential-safe SSH to the installed laptop Wi-Fi agent
(board_network/wifi_agent.py). Secrets travel on SSH stdin only.
Laptop: NetworkManager hotspot of this PC via nmcli (no sudo; the active desktop
user's NetworkManager permission). Turning it on usually drops this PC's
classroom Wi-Fi because one Wi-Fi card serves either a client or an AP.
"""
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

FEATURE = 'nomad-gui-laptop-wifi-20261111-1'
MARKER = 'NOMAD_NETWORK_RESULT='
BOARDS = {'pi4': 'Raspberry Pi 4', 'pi5': 'Raspberry Pi 5', 'jetson': 'Jetson'}
AGENT_SOURCE = Path(__file__).resolve().parent / 'board_network' / 'wifi_agent.py'
INSTALLED = '/usr/local/lib/nomad-network/wifi_agent.py'
ACTIONS = ('status', 'install', 'register', 'remove', 'watchdog', 'indoor', 'classroom', 'peers')
HOST_OFFSETS = {'pi4': 14, 'pi5': 15, 'jetson': 16}
DEFAULT_GATEWAY = '10.42.0.1/24'  # NetworkManager "shared" hotspot default
LOCAL_PROFILE = 'nomad-laptop-hotspot'
# Request JSON on line 2; for install the agent source follows on stdin.
WRAPPER = r'''set -eu
umask 077
IFS= read -r nomad_sudo_password
IFS= read -r nomad_request
nomad_net_dir=$(mktemp -d /tmp/nomad-wifi-request.XXXXXXXX)
trap 'rm -rf -- "$nomad_net_dir"' EXIT
printf '%s\n' "$nomad_request" > "$nomad_net_dir/request.json"
nomad_agent=__AGENT__
if [ "$nomad_agent" = upload ]; then
  cat > "$nomad_net_dir/wifi_agent.py"
  nomad_agent="$nomad_net_dir/wifi_agent.py"
fi
printf '%s\n' "$nomad_sudo_password" | sudo -k -S -p '' python3 "$nomad_agent" --request-file "$nomad_net_dir/request.json"
'''
REASONS = {'invalid_request', 'wrong_board', 'invalid_credentials', 'invalid_ssid', 'invalid_wifi_password',
           'invalid_laptop_name', 'invalid_laptop_addresses', 'select_wifi_interface', 'home_wifi_unknown',
           'laptop_in_use', 'invalid_timeout', 'no_laptop_registered', 'not_on_classroom_wifi', 'schedule_failed',
           'invalid_peer_addresses', 'peer_address_not_current', 'network_command_failed',
           'command_timeout_or_unavailable', 'root_required', 'unsupported_board', 'agent_error'}
MODES = {'laptop', 'classroom', 'other', 'disconnected'}


def failed(reason):
    return {'ok': False, 'reason': reason}


def validate_credentials(ssid, password):
    from board_network.wifi_agent import credentials, NetworkError
    try:
        return credentials(ssid, password)
    except (NetworkError, AttributeError, UnicodeError):
        raise ValueError('SSID는 UTF-8 기준 1~32바이트, 비밀번호는 WPA2 기준 ASCII 8~63자 또는 64자리 16진수입니다.') from None


def board_addresses(gateway):
    """Fixed board addresses on a laptop hotspot network: .14 / .15 / .16."""
    try:
        network = ipaddress.IPv4Interface(gateway).network
        if not 16 <= network.prefixlen <= 30:
            raise ValueError()
    except (ValueError, TypeError):
        raise ValueError('노트북 핫스팟 주소는 10.42.0.1/24 형식이어야 합니다.') from None
    base = int(network.network_address)
    return {board: str(ipaddress.IPv4Address(base + offset)) for board, offset in HOST_OFFSETS.items()}


def clean_text(value, limit=128):
    if not isinstance(value, str) or len(value) > limit or any(ord(c) < 32 for c in value):
        raise ValueError()
    return value


def parse_result(stdout, returncode, board):
    lines = [line[len(MARKER):] for line in stdout.splitlines() if line.startswith(MARKER)]
    if len(lines) != 1:
        return failed('agent_missing_or_invalid_result')
    try:
        value = json.loads(lines[0])
        if not isinstance(value, dict) or value.get('feature') != FEATURE:
            raise ValueError()
        if value.get('ok') is not True:
            return failed(value.get('reason') if value.get('reason') in REASONS else 'remote_network_error')
        if returncode != 0 or value.get('board') != board or value.get('mode') not in MODES:
            raise ValueError()
        if not re.fullmatch('[0-9a-f]{32}', str(value.get('machine_id'))):
            raise ValueError()
        addresses = value.get('ipv4')
        if not isinstance(addresses, list) or len(addresses) > 16:
            raise ValueError()
        result = {'ok': True, 'board': board, 'machine_id': value['machine_id'], 'mode': value['mode'],
                  'ipv4': [str(ipaddress.IPv4Address(a)) for a in addresses]}
        for key in ('interface', 'active_ssid', 'active_laptop', 'home_ssid', 'watchdog', 'pending'):
            result[key] = clean_text(value.get(key, ''))
        seconds = value.get('watchdog_seconds')
        result['watchdog_seconds'] = seconds if type(seconds) is int else None
        laptops = value.get('laptops', [])
        if not isinstance(laptops, list) or len(laptops) > 32:
            raise ValueError()
        result['laptops'] = [{k: clean_text(item.get(k, '')) for k in ('name', 'ssid', 'address')} for item in laptops]
        removed = value.get('removed', [])
        if isinstance(removed, list):
            result['removed'] = [clean_text(r, 256) for r in removed[:64]]
        return result
    except (ValueError, TypeError, KeyError, AttributeError):
        return failed('agent_missing_or_invalid_result')


def request(board, host, ssh_password, action='status', *, timeout_s=40, **options):
    if board not in BOARDS or action not in ACTIONS:
        return failed('invalid_request')
    try:
        host = validate_host(host)
        if host.split('@')[0] != ('nvidia' if board == 'jetson' else 'user'):
            return failed('invalid_board_address')
        if not isinstance(ssh_password, str) or not ssh_password or any(c in ssh_password for c in '\r\n\x00'):
            return failed('ssh_password_required')
        allowed = {'name', 'ssid', 'password', 'gateway', 'addresses', 'seconds', 'peers'}
        if set(options) - allowed or (action != 'register' and 'password' in options):
            return failed('invalid_request')
        if action == 'register':
            validate_credentials(options.get('ssid'), options.get('password'))
        payload = ssh_password + '\n' + json.dumps(dict(board=board, action=action, **options), ensure_ascii=False) + '\n'
        upload = action == 'install'
        if upload:
            payload += AGENT_SOURCE.read_text(encoding='utf-8')
        wrapper = WRAPPER.replace('__AGENT__', 'upload' if upload else INSTALLED)
        done = _board_call(host, ssh_password, payload, wrapper, timeout_s)
        if 'reason' in done:
            return done
        out, err, returncode = done['out'], done['err'], done['returncode']
        if not upload and 'No such file' in err and MARKER not in out:
            return failed('agent_not_installed')
        return parse_result(out, returncode, board)
    except (OSError, TypeError, ValueError):
        return failed('invalid_credentials_or_local_ssh_error')


def _board_call(host, ssh_password, payload, wrapper, timeout_s):
    """One SSH call; secrets on stdin and in a private askpass only."""
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'nomad-trial'
    state.mkdir(parents=True, exist_ok=True, mode=0o700)
    with tempfile.TemporaryDirectory(prefix='nomad-wifi-ssh-') as temporary:
        askpass = Path(temporary) / 'askpass'
        askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_SSH_PASSWORD"\n')
        askpass.chmod(0o700)
        env = dict(os.environ, SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force', NOMAD_SSH_PASSWORD=ssh_password)
        env.setdefault('DISPLAY', ':0')
        args = ['ssh', '-T', '-o', 'ControlMaster=no', '-o', 'ControlPath=none',
                '-o', 'ConnectTimeout=5', '-o', 'ConnectionAttempts=1', '-o', 'ServerAliveInterval=3',
                '-o', 'ServerAliveCountMax=2', '-o', 'NumberOfPasswordPrompts=1',
                '-o', 'StrictHostKeyChecking=accept-new', '-o', 'UserKnownHostsFile="' + str(state / 'known_hosts') + '"',
                '-o', 'LogLevel=ERROR', host, 'bash -c ' + shlex.quote(wrapper)]
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
    return {'out': out, 'err': err, 'returncode': process.returncode}


# ---------------- board time (Pi5 reference, chrony) ----------------
TIME_SOURCE = Path(__file__).resolve().parent / 'board_network' / 'time_agent.py'
TIME_INSTALLED = '/usr/local/lib/nomad-network/time_agent.py'


def time_request(board, host, ssh_password, action='status', timeout_s=40, address=None):
    """install: upload time_agent.py and run it with sudo (chrony, config, cable).
    status: the board's own check. pi5_address: tell a follower Pi5's current
    address (installed agent, sudo). Returns clock_sync-style dict with 'message'."""
    import clock_sync
    if board not in BOARDS or action not in ('status', 'install', 'pi5_address'):
        return failed('invalid_request')
    request = {'board': board, 'action': action}
    if action == 'pi5_address':
        try:
            request['address'] = str(ipaddress.IPv4Address(address))
        except (ValueError, TypeError):
            return failed('invalid_request')
        if board == 'pi5':
            return failed('invalid_request')
    try:
        host = validate_host(host)
        if host.split('@')[0] != ('nvidia' if board == 'jetson' else 'user'):
            return failed('invalid_board_address')
        if not isinstance(ssh_password, str) or not ssh_password or any(c in ssh_password for c in '\r\n\x00'):
            return failed('ssh_password_required')
        payload = ssh_password + '\n' + json.dumps(request) + '\n'
        if action == 'install':
            payload += TIME_SOURCE.read_text(encoding='utf-8')
        wrapper = WRAPPER.replace('__AGENT__', 'upload' if action == 'install' else TIME_INSTALLED).replace(
            'wifi_agent.py', 'time_agent.py')
        done = _board_call(host, ssh_password, payload, wrapper, timeout_s)
        if 'reason' in done:
            return done
        if MARKER not in done['out'] and clock_sync.MARKER not in done['out']:
            return failed('time_sync_not_installed' if action == 'status' else 'agent_missing_or_invalid_result')
        result = clock_sync.parse(done['out'])
        if result.get('reason', '').startswith('trial_running'):
            result['message'] = '시험 모듈이 실행 중이라 설치하지 않음 / 시험 종료 후 다시'
        return result
    except (OSError, TypeError, ValueError):
        return failed('invalid_credentials_or_local_ssh_error')


# ---------------- this laptop's own hotspot ----------------

def _nmcli(*args, timeout=20):
    try:
        result = subprocess.run(['nmcli', '--colors', 'no'] + list(args), stdin=subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=timeout, env=dict(os.environ, LC_ALL='C'))
    except (OSError, subprocess.TimeoutExpired):
        raise RuntimeError('nmcli 실행 실패') from None
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout).strip().splitlines()[-1][:200] if (result.stderr or result.stdout).strip() else 'nmcli 오류')
    return result.stdout.strip()


def local_wifi_device():
    devices = [line.rsplit(':', 1)[0] for line in _nmcli('-t', '-f', 'DEVICE,TYPE', 'device', 'status').splitlines()
               if line.endswith(':wifi')]
    if not devices:
        raise RuntimeError('이 PC에서 Wi-Fi 장치를 찾지 못했습니다.')
    return devices[0]


def local_hotspots():
    """All AP-mode Wi-Fi profiles of this PC with SSID, active state and gateway."""
    out = []
    for line in _nmcli('-t', '-f', 'NAME,UUID,TYPE,ACTIVE', 'connection', 'show').splitlines():
        parts = line.replace('\\:', '\x00').split(':')
        if len(parts) != 4 or parts[2] != '802-11-wireless':
            continue
        name, uuid_text, _, active = (p.replace('\x00', ':') for p in parts)
        fields = _nmcli('-g', '802-11-wireless.mode,802-11-wireless.ssid,ipv4.method,ipv4.addresses,802-11-wireless.band',
                        'connection', 'show', 'uuid', uuid_text).splitlines()
        fields += [''] * (5 - len(fields))
        if fields[0] != 'ap':
            continue
        gateway = fields[3].split(',')[0].strip() or (DEFAULT_GATEWAY if fields[2] == 'shared' else '')
        if active == 'yes':
            current = _nmcli('-g', 'IP4.ADDRESS', 'connection', 'show', 'uuid', uuid_text).split('|')[0].strip()
            gateway = current or gateway
        out.append({'name': name, 'uuid': uuid_text, 'ssid': fields[1], 'active': active == 'yes',
                    'gateway': gateway, 'band': fields[4] or 'auto'})
    return out


def local_hotspot_password(uuid_text):
    return _nmcli('-s', '-g', '802-11-wireless-security.psk', 'connection', 'show', 'uuid', uuid_text)


def local_hotspot_create(ssid, password):
    validate_credentials(ssid, password)
    device = local_wifi_device()
    # nmcli takes the PSK as an argument; it exists only in this short local
    # process. 2.4 GHz for the widest board compatibility; fixed 10.42.0.1/24.
    _nmcli('connection', 'add', 'type', 'wifi', 'ifname', device, 'con-name', LOCAL_PROFILE, 'autoconnect', 'no',
           'ssid', ssid, '802-11-wireless.mode', 'ap', '802-11-wireless.band', 'bg',
           'ipv4.method', 'shared', 'ipv4.addresses', DEFAULT_GATEWAY, 'ipv6.method', 'ignore',
           'wifi-sec.key-mgmt', 'wpa-psk', 'wifi-sec.psk', password)
    return [h for h in local_hotspots() if h['name'] == LOCAL_PROFILE]


def local_hotspot_switch(uuid_text, on):
    if on:
        _nmcli('--wait', '30', 'connection', 'up', 'uuid', uuid_text, timeout=40)
    else:
        _nmcli('connection', 'down', 'uuid', uuid_text, timeout=30)
    return local_hotspots()
