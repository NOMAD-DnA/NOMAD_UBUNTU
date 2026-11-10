#!/usr/bin/env python3
"""Pi4/Pi5/Jetson Wi-Fi client control, derived from the 20261030 Switch.

Only NetworkManager profiles and this feature's state are changed. Independent
systemd guard precedes activation. Secrets never enter argv, stdout, or state.
"""
import contextlib
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time
import uuid

FEATURE = 'nomad-gui-hotspot-20261110-1'
MARKER = 'NOMAD_NETWORK_RESULT='
PENDING = {'queued', 'switching', 'awaiting_confirmation', 'restore_queued', 'rolling_back'}
BASE = Path('/var/lib/nomad-gui-network')
INSTALLED = Path('/usr/local/lib/nomad-network/network_agent.py')


class NetworkError(RuntimeError):
    pass


def command(args, timeout=15):
    try:
        result = subprocess.run(args, stdin=subprocess.DEVNULL, capture_output=True,
                                text=True, timeout=timeout, env=dict(os.environ, LC_ALL='C'))
    except (OSError, subprocess.TimeoutExpired):
        raise NetworkError('command_timeout_or_unavailable') from None
    if result.returncode:
        raise NetworkError('network_command_failed')
    return result.stdout.strip()


def private_write(path, text, mode=0o600):
    path = Path(path)
    temp = path.with_name(path.name + '.tmp-' + uuid.uuid4().hex)
    try:
        fd = os.open(str(temp), os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
        os.fchmod(fd, mode)
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(str(temp), str(path))
    finally:
        if temp.exists():
            temp.unlink()


def key_value(value):
    if not isinstance(value, str) or any(ord(c) < 32 or ord(c) == 127 for c in value):
        raise NetworkError('invalid_credentials')
    # GLib keyfile strings need escaped leading/trailing spaces as well.
    return value.replace('\\', '\\\\').replace(' ', '\\s')


def credentials(ssid, password):
    key_value(ssid)
    key_value(password)
    if not 1 <= len(ssid.encode('utf-8')) <= 32:
        raise NetworkError('invalid_ssid')
    if not (password.isascii() and (8 <= len(password) <= 63 or
                                   re.fullmatch(r'[0-9a-fA-F]{64}', password))):
        raise NetworkError('invalid_wifi_password')
    return ssid, password


def board_identity():
    model = Path('/proc/device-tree/model').read_text().replace('\x00', '').strip()
    if 'Raspberry Pi 4' in model:
        board = 'pi4'
    elif 'Raspberry Pi 5' in model:
        board = 'pi5'
    elif 'Jetson' in model or 'Xavier' in model:
        board = 'jetson'
    else:
        raise NetworkError('unsupported_board')
    return board, Path('/etc/machine-id').read_text().strip()


def mac_address(iface):
    return Path('/sys/class/net', iface, 'address').read_text().strip()


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


class Switch:
    def __init__(self, base=BASE, profiles=Path('/etc/NetworkManager/system-connections'),
                 worker=INSTALLED, peers=Path('/etc/nomad/network-peers.json')):
        self.base, self.profiles, self.worker, self.peers = map(Path, (base, profiles, worker, peers))
        self.base.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.state_path = self.base / 'state.json'

    @contextlib.contextmanager
    def locked(self):
        with (self.base / 'lock').open('a') as handle:
            os.chmod(str(self.base / 'lock'), 0o600)
            fcntl.flock(handle, fcntl.LOCK_EX)
            yield

    def read(self):
        return json.loads(self.state_path.read_text()) if self.state_path.exists() else {}

    def save(self, state):
        state['updated_at'] = time.time()
        private_write(self.state_path, json.dumps(state, ensure_ascii=False) + '\n')

    def nm(self, *args, timeout=15):
        return command(['nmcli', '--colors', 'no', '--escape', 'no'] + list(args), timeout)

    def active_uuid(self, iface):
        return self.nm('-g', 'GENERAL.CON-UUID', 'device', 'show', iface)

    def addresses(self, iface):
        result = []
        for line in self.nm('-g', 'IP4.ADDRESS', 'device', 'show', iface).splitlines():
            try:
                address = ipaddress.IPv4Interface(line).ip
            except ValueError:
                continue
            if not (address.is_loopback or address.is_link_local or address.is_unspecified):
                result.append(str(address))
        return result

    def interface(self, requested=None):
        if shutil.which('nmcli') is None:
            raise NetworkError('networkmanager_unavailable')
        command(['systemctl', 'is-active', 'NetworkManager'])
        lines = self.nm('-t', '-f', 'DEVICE,TYPE', 'device', 'status').splitlines()
        devices = [line.rsplit(':', 1)[0] for line in lines if line.endswith(':wifi')]
        if requested is None and len(devices) == 1:
            requested = devices[0]
        if requested not in devices or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', requested or ''):
            raise NetworkError('select_wifi_interface')
        return requested

    def preflight(self, requested=None):
        iface = self.interface(requested)
        for tool in ('systemd-run', 'systemctl'):
            if shutil.which(tool) is None:
                raise NetworkError('systemd_unavailable')
        if self.nm('radio', 'wifi') != 'enabled':
            raise NetworkError('wifi_radio_disabled')
        if not self.nm('-g', 'GENERAL.STATE', 'device', 'show', iface).startswith('100'):
            raise NetworkError('previous_wifi_not_connected')
        try:
            uuid.UUID(self.active_uuid(iface))
        except ValueError:
            raise NetworkError('previous_wifi_not_connected') from None
        return iface

    def ready(self, state):
        iface = state['interface']
        return (self.active_uuid(iface) == state['target_uuid'] and
                self.nm('-g', 'GENERAL.STATE', 'device', 'show', iface).startswith('100') and
                bool(self.addresses(iface)))

    def check_token(self, state, token):
        if not token or state.get('token') != token:
            raise NetworkError('stale_request')

    def prepare(self, ssid, password, requested=None):
        credentials(ssid, password)
        with self.locked():
            prior = self.read()
            if prior.get('status') in PENDING:
                raise NetworkError('switch_pending')
            iface = self.preflight(requested)
            target, token = str(uuid.uuid4()), uuid.uuid4().hex
            path = self.profiles / ('nomad-gui-' + target + '.nmconnection')
            text = ('[connection]\nid=nomad-gui-{target}\nuuid={target}\ntype=wifi\n'
                    'interface-name={iface}\nautoconnect=false\nautoconnect-retries=2\n\n'
                    '[wifi]\nmode=infrastructure\nssid={ssid}\nsecurity=802-11-wireless-security\n\n'
                    '[wifi-security]\nkey-mgmt=wpa-psk\npsk={password}\npsk-flags=0\n\n'
                    '[ipv4]\nmethod=auto\n\n[ipv6]\nmethod=auto\n').format(
                        target=target, iface=iface, ssid=''.join(str(b) + ';' for b in ssid.encode('utf-8')), password=key_value(password))
            self.profiles.mkdir(mode=0o700, parents=True, exist_ok=True)
            private_write(path, text)
            try:
                self.nm('connection', 'load', str(path))
            except Exception:
                path.unlink()
                raise
            self.save(dict(status='prepared', token=token, interface=iface, ssid=ssid,
                           target_uuid=target, profile_path=str(path), previous_uuid=self.active_uuid(iface),
                           boot_id=boot_id(), error=None))
            # Discard only unused preparations owned by this feature, after the
            # replacement loaded successfully. Committed profiles stay intact.
            if prior.get('status') in ('prepared', 'rolled_back', 'schedule_failed'):
                old = prior.get('target_uuid')
                if old and old != self.active_uuid(iface):
                    try:
                        self.nm('connection', 'delete', 'uuid', old)
                    except NetworkError:
                        pass
        return self.snapshot()

    def unit(self, state, kind):
        return 'nomad-gui-wifi-' + kind + '-' + state['token'][:12]

    def schedule(self, state, action, timer=None):
        kind = 'guard' if action == '_rollback' else 'restore' if action == '_restore' else 'switch'
        args = ['systemd-run', '--quiet', '--unit=' + self.unit(state, kind),
                '--property=Type=exec', '--property=StandardOutput=null', '--property=StandardError=null']
        if timer:
            args += ['--on-active={}s'.format(timer), '--timer-property=AccuracySec=1s']
        else:
            args += ['--property=RuntimeMaxSec=120s']
        command(args + [sys.executable, str(self.worker), action, state['token']])

    def stop_guard(self, state):
        try:
            command(['systemctl', 'stop', self.unit(state, 'guard') + '.timer'])
        except NetworkError:
            pass

    def queue(self, token, confirmation_seconds=300):
        if type(confirmation_seconds) is not int or not 120 <= confirmation_seconds <= 900:
            raise NetworkError('invalid_timeout')
        with self.locked():
            state = self.read()
            self.check_token(state, token)
            if state.get('status') != 'prepared':
                raise NetworkError('not_prepared')
            self.preflight(state['interface'])
            if self.active_uuid(state['interface']) != state['previous_uuid']:
                raise NetworkError('previous_connection_changed')
            state.update(status='queued', boot_id=boot_id(), deadline=time.monotonic() + confirmation_seconds)
            self.save(state)
            try:
                self.schedule(state, '_rollback', confirmation_seconds)
                self.schedule(state, '_worker')
            except Exception:
                # Guard failure never activates anything. A partially created
                # worker cannot acquire the lock until this terminal state saves.
                state.update(status='schedule_failed', error='schedule_failed')
                self.save(state)
                self.stop_guard(state)
                raise NetworkError('schedule_failed') from None
        return self.snapshot()

    def worker_run(self, token):
        time.sleep(8)
        with self.locked():
            state = self.read()
            if state.get('token') != token or state.get('status') != 'queued':
                return
            if state.get('boot_id') != boot_id() or time.monotonic() >= state['deadline']:
                self._rollback_locked(state, 'confirmation_expired')
                return
            state['status'] = 'switching'
            self.save(state)
            try:
                self.nm('--wait', '45', 'connection', 'up', 'uuid', state['target_uuid'],
                        'ifname', state['interface'], timeout=50)
                if not self.ready(state):
                    raise NetworkError('no_ipv4')
                if time.monotonic() >= state['deadline']:
                    raise NetworkError('confirmation_expired')
                state.update(status='awaiting_confirmation', error=None)
                self.save(state)
            except Exception:
                self._rollback_locked(state, 'activation_failed')

    def confirm(self, token, ssh_server):
        with self.locked():
            state = self.read()
            self.check_token(state, token)
            if state.get('status') not in ('awaiting_confirmation', 'committed'):
                raise NetworkError('not_awaiting_confirmation')
            if not self.ready(state) or ssh_server not in self.addresses(state['interface']):
                raise NetworkError('reconnect_via_wifi_address')
            if state['status'] == 'committed':
                return
            if state.get('boot_id') != boot_id() or time.monotonic() >= state['deadline']:
                raise NetworkError('confirmation_expired')
            old_priority = int(self.nm('-g', 'connection.autoconnect-priority', 'connection',
                                       'show', 'uuid', state['previous_uuid']) or '0')
            priority = str(min(999, max(100, old_priority + 10)))
            self.nm('connection', 'modify', 'uuid', state['target_uuid'],
                    'connection.autoconnect', 'yes', 'connection.autoconnect-priority', priority,
                    'connection.autoconnect-retries', '2')
            state.update(status='committed', error=None)
            self.save(state)
            self.stop_guard(state)

    def _rollback_locked(self, state, reason):
        state.update(status='rolling_back', error=reason)
        self.save(state)
        disable_failed = False
        try:
            self.nm('connection', 'modify', 'uuid', state['target_uuid'], 'connection.autoconnect', 'no')
        except Exception:
            disable_failed = True
        try:
            self.nm('--wait', '45', 'connection', 'up', 'uuid', state['previous_uuid'],
                    'ifname', state['interface'], timeout=50)
            if self.active_uuid(state['interface']) != state['previous_uuid'] or not self.addresses(state['interface']):
                raise NetworkError('restore_failed')
            state.update(status='rollback_failed' if disable_failed else 'rolled_back',
                         error='restore_failed' if disable_failed else reason)
        except Exception:
            state.update(status='rollback_failed', error='restore_failed')
        self.save(state)
        self.stop_guard(state)

    def rollback(self, token, force=False):
        if force:
            time.sleep(8)
        with self.locked():
            state = self.read()
            if state.get('token') != token:
                return
            if force and state.get('status') != 'restore_queued':
                return
            if not force and state.get('status') not in PENDING:
                return
            self._rollback_locked(state, 'user_restore' if force else 'confirmation_expired')

    def queue_restore(self, token):
        with self.locked():
            state = self.read()
            self.check_token(state, token)
            if not state.get('previous_uuid') or state.get('status') in ('rolling_back', 'restore_queued'):
                raise NetworkError('restore_unavailable')
            prior = state['status']
            state['status'] = 'restore_queued'
            self.save(state)
            try:
                self.schedule(state, '_restore')
            except Exception:
                state['status'] = prior
                self.save(state)
                raise NetworkError('schedule_failed') from None
        return self.snapshot()

    def snapshot(self):
        # Lock covers activations too, so status never reports a half-written transition.
        with self.locked():
            state = self.read()
            iface = self.interface(state.get('interface'))
            active = self.active_uuid(iface)
            ssid = self.nm('-g', '802-11-wireless.ssid', 'connection', 'show', 'uuid', active) if active not in ('', '--') else ''
            value = {k: state[k] for k in ('status', 'token', 'ssid', 'error') if k in state}
            value.update(status=state.get('status', 'idle'), interface=iface,
                         active_ssid=ssid, ipv4=self.addresses(iface), active_uuid=active,
                         target_active=bool(active and active == state.get('target_uuid')),
                         mac=mac_address(iface))
            if state.get('status') in PENDING:
                value['seconds_until_restore'] = max(0, int(state.get('deadline', 0) - time.monotonic())) if state.get('boot_id') == boot_id() else 0
            return value

    def save_peers(self, peers, board):
        state = self.read()
        if state.get('status') in PENDING:
            raise NetworkError('connection_not_confirmed')
        if not isinstance(peers, dict) or set(peers) != {'pi4', 'pi5', 'jetson'}:
            raise NetworkError('invalid_peer_addresses')
        try:
            parsed = {k: ipaddress.IPv4Address(v) for k, v in peers.items()}
        except (ValueError, TypeError):
            raise NetworkError('invalid_peer_addresses') from None
        if len(set(parsed.values())) != 3 or any(p.is_loopback or p.is_multicast or p.is_unspecified for p in parsed.values()):
            raise NetworkError('invalid_peer_addresses')
        iface = self.interface()
        if str(parsed[board]) not in self.addresses(iface):
            raise NetworkError('peer_address_not_current')
        for peer, address in parsed.items():
            if peer != board:
                try:
                    with socket.create_connection((str(address), 22), timeout=2):
                        pass
                except OSError:
                    raise NetworkError('board_peer_unreachable') from None
        if not self.peers.parent.exists():
            self.peers.parent.mkdir(mode=0o755, parents=True)
            self.peers.parent.chmod(0o755)
        private_write(self.peers, json.dumps({k: str(v) for k, v in parsed.items()}) + '\n', 0o644)


def handle(request, manager, ssh_connection=''):
    if not isinstance(request, dict) or request.get('action') not in ('status', 'prepare', 'switch', 'confirm', 'restore', 'peers'):
        raise NetworkError('invalid_request')
    board, machine = board_identity()
    if request.get('board') != board:
        raise NetworkError('wrong_board')
    if request.get('machine_id') and request['machine_id'] != machine:
        raise NetworkError('wrong_machine')
    action = request['action']
    if action == 'prepare':
        manager.prepare(request.get('ssid'), request.get('password'), request.get('interface') or None)
    elif action == 'switch':
        manager.queue(request.get('token'), request.get('timeout', 300))
    elif action == 'confirm':
        parts = ssh_connection.split()
        manager.confirm(request.get('token'), parts[2] if len(parts) == 4 else '')
    elif action == 'restore':
        manager.queue_restore(request.get('token'))
    elif action == 'peers':
        manager.save_peers(request.get('peers'), board)
    return dict(ok=True, feature=FEATURE, board=board, machine_id=machine, **manager.snapshot())


def main():
    os.umask(0o077)
    if os.geteuid() != 0:
        raise NetworkError('root_required')
    manager = Switch()
    if len(sys.argv) == 3 and sys.argv[1] in ('_worker', '_rollback', '_restore'):
        if not re.fullmatch('[0-9a-f]{32}', sys.argv[2]):
            raise NetworkError('invalid_token')
        if sys.argv[1] == '_worker':
            manager.worker_run(sys.argv[2])
        else:
            manager.rollback(sys.argv[2], force=sys.argv[1] == '_restore')
        return
    # Request file resides in a private temporary directory created by SSH shell.
    # No request content is ever part of argv or logs. Delete before dispatch.
    if len(sys.argv) != 4 or sys.argv[1] != '--request-file':
        raise NetworkError('invalid_request')
    path = Path(sys.argv[2])
    try:
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 8192:
            raise NetworkError('invalid_request')
        request = json.loads(path.read_text())
    finally:
        path.unlink(missing_ok=True)
    result = handle(request, manager, sys.argv[3])
    print(MARKER + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        reason = str(exc) if isinstance(exc, NetworkError) else 'agent_error'
        print(MARKER + json.dumps(dict(ok=False, feature=FEATURE, reason=reason)), flush=True)
        sys.exit(1)
