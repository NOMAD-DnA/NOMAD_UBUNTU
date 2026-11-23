#!/usr/bin/env python3
"""Pi4/Pi5/Jetson Wi-Fi: classroom Wi-Fi plus saved laptop hotspots.

Replaces the 2026-11-10 300-second switch. Each registered laptop hotspot is a
saved NetworkManager profile (fixed address, unlimited autoconnect retries,
lower priority than the classroom Wi-Fi). "Indoor" deactivates the classroom
connection so NetworkManager joins a laptop; a watchdog reconnects the
classroom Wi-Fi when the board stayed without Wi-Fi for N seconds and the
classroom network is visible. Secrets never enter argv, stdout or logs.
Python 3.8 compatible (Jetson).
"""
import contextlib
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import uuid

FEATURE = 'nomad-gui-laptop-wifi-20261111-1'
MARKER = 'NOMAD_NETWORK_RESULT='
INSTALL_DIR = Path('/usr/local/lib/nomad-network')
INSTALLED = INSTALL_DIR / 'wifi_agent.py'
BASE = Path('/var/lib/nomad-laptop-wifi')
CONFIG = Path('/etc/nomad/laptop-wifi.json')
PEERS = Path('/etc/nomad/network-peers.json')
PROFILES = Path('/etc/NetworkManager/system-connections')
UNIT = 'nomad-wifi-watchdog'
UNIT_PATH = Path('/etc/systemd/system') / (UNIT + '.service')
PREFIX = 'nomad-laptop-'
NAMESPACE = uuid.UUID('6f1d7c52-9a3e-4b8e-a1c0-6c0b6d1f2e11')
LAPTOP_PRIORITY = -10
DEFAULT_WATCHDOG = 120
BOARD_KEYS = ('pi4', 'pi5', 'jetson')
# Removed on install: the 300-second switch and the 2026-10-30 hotspot tool's profiles.
OLD_PROFILE_PREFIXES = ('nomad-gui-', 'nomad-hotspot-')
OLD_AGENT = INSTALL_DIR / 'network_agent.py'
OLD_STATE = Path('/var/lib/nomad-gui-network')


class NetworkError(RuntimeError):
    pass


def command(args, timeout=15):
    try:
        result = subprocess.run(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, timeout=timeout, env=dict(os.environ, LC_ALL='C'))
    except (OSError, subprocess.TimeoutExpired):
        raise NetworkError('command_timeout_or_unavailable') from None
    if result.returncode:
        raise NetworkError('network_command_failed')
    return result.stdout.strip()


def nm(*args, timeout=15):
    return command(['nmcli', '--colors', 'no', '--escape', 'no'] + list(args), timeout)


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
    return value.replace('\\', '\\\\').replace(' ', '\\s')


def credentials(ssid, password):
    key_value(ssid)
    key_value(password)
    if not 1 <= len(ssid.encode('utf-8')) <= 32:
        raise NetworkError('invalid_ssid')
    if not (password.isascii() and (8 <= len(password) <= 63 or re.fullmatch(r'[0-9a-fA-F]{64}', password))):
        raise NetworkError('invalid_wifi_password')
    return ssid, password


def laptop_name(name):
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,24}', name):
        raise NetworkError('invalid_laptop_name')
    return name


def network_plan(gateway, addresses):
    """gateway '10.42.0.1/24', addresses {'pi4': '10.42.0.14', ...} on that network."""
    try:
        gw = ipaddress.IPv4Interface(gateway)
        if not 16 <= gw.network.prefixlen <= 30:
            raise ValueError()
        if not isinstance(addresses, dict) or set(addresses) != set(BOARD_KEYS):
            raise ValueError()
        parsed = {k: ipaddress.IPv4Address(v) for k, v in addresses.items()}
    except (ValueError, TypeError):
        raise NetworkError('invalid_laptop_addresses') from None
    values = list(parsed.values()) + [gw.ip]
    if len(set(values)) != 4 or any(v not in gw.network or v in (gw.network.network_address, gw.network.broadcast_address)
                                    for v in values):
        raise NetworkError('invalid_laptop_addresses')
    return gw, parsed


def board_identity():
    model = Path('/proc/device-tree/model').read_text().replace('\x00', '').strip()
    for text, board in (('Raspberry Pi 4', 'pi4'), ('Raspberry Pi 5', 'pi5'), ('Jetson', 'jetson'), ('Xavier', 'jetson')):
        if text in model:
            return board, Path('/etc/machine-id').read_text().strip()
    raise NetworkError('unsupported_board')


def read_json(path, default):
    try:
        value = json.loads(Path(path).read_text())
        return value if isinstance(value, type(default)) else default
    except (OSError, ValueError):
        return default


class Agent:
    def __init__(self, base=BASE, config=CONFIG, peers=PEERS, profiles=PROFILES, nmcli=nm, runner=command):
        self.base, self.config_path, self.peers_path, self.profiles = map(Path, (base, config, peers, profiles))
        self.nm, self.run = nmcli, runner

    # ---------- inspection ----------
    def interface(self):
        devices = [line.rsplit(':', 1)[0] for line in self.nm('-t', '-f', 'DEVICE,TYPE', 'device', 'status').splitlines()
                   if line.endswith(':wifi')]
        if len(devices) != 1 or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', devices[0]):
            raise NetworkError('select_wifi_interface')
        return devices[0]

    def active_uuid(self, iface):
        value = self.nm('-g', 'GENERAL.CON-UUID', 'device', 'show', iface)
        return value if re.fullmatch(r'[0-9a-f-]{36}', value or '') else ''

    def connected(self, iface):
        return self.nm('-g', 'GENERAL.STATE', 'device', 'show', iface).startswith('100')

    def addresses(self, iface):
        result = []
        for line in self.nm('-g', 'IP4.ADDRESS', 'device', 'show', iface).splitlines():
            try:
                address = ipaddress.IPv4Interface(line.strip()).ip
            except ValueError:
                continue
            if not (address.is_loopback or address.is_link_local or address.is_unspecified):
                result.append(str(address))
        return result

    def connections(self):
        out = []
        for line in self.nm('-t', '-f', 'NAME,UUID,TYPE', 'connection', 'show').splitlines():
            parts = line.rsplit(':', 2)
            if len(parts) == 3:
                out.append({'name': parts[0], 'uuid': parts[1], 'type': parts[2]})
        return out

    def ssid_of(self, uuid_text):
        return self.nm('-g', '802-11-wireless.ssid', 'connection', 'show', 'uuid', uuid_text) if uuid_text else ''

    def config(self):
        value = read_json(self.config_path, {})
        seconds = value.get('watchdog_seconds')
        value['watchdog_seconds'] = seconds if type(seconds) is int and 30 <= seconds <= 900 else DEFAULT_WATCHDOG
        return value

    def save_config(self, value):
        self.config_path.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        private_write(self.config_path, json.dumps(value, ensure_ascii=False) + '\n')

    def laptops(self):
        out = []
        for item in self.connections():
            if item['type'] in ('802-11-wireless', 'wifi') and item['name'].startswith(PREFIX):
                address = ''
                try:
                    address = self.nm('-g', 'ipv4.addresses', 'connection', 'show', 'uuid', item['uuid']).split(',')[0].strip()
                except NetworkError:
                    pass
                out.append({'name': item['name'][len(PREFIX):], 'uuid': item['uuid'],
                            'ssid': self.ssid_of(item['uuid']), 'address': address})
        return out

    def watchdog_state(self):
        try:
            return self.run(['systemctl', 'is-active', UNIT]).strip() or 'unknown'
        except NetworkError:
            return 'inactive'

    @contextlib.contextmanager
    def locked(self):
        self.base.mkdir(mode=0o700, parents=True, exist_ok=True)
        with (self.base / 'lock').open('a') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            yield

    def remember_home(self, iface):
        """Record the classroom Wi-Fi: the active non-laptop Wi-Fi connection."""
        config = self.config()
        active = self.active_uuid(iface)
        names = {c['uuid']: c['name'] for c in self.connections()}
        if active and not names.get(active, '').startswith(PREFIX):
            config.update(home_uuid=active, home_ssid=self.ssid_of(active))
            self.save_config(config)
        if not config.get('home_uuid') or config['home_uuid'] not in names:
            raise NetworkError('home_wifi_unknown')
        return config

    def snapshot(self):
        iface = self.interface()
        active = self.active_uuid(iface)
        config = self.config()
        names = {c['uuid']: c['name'] for c in self.connections()}
        name = names.get(active, '')
        mode = ('laptop' if name.startswith(PREFIX) else
                'classroom' if active and active == config.get('home_uuid', active) else
                'other' if active else 'disconnected')
        return {'interface': iface, 'active_ssid': self.ssid_of(active) if active else '', 'ipv4': self.addresses(iface),
                'mode': mode, 'active_laptop': name[len(PREFIX):] if name.startswith(PREFIX) else '',
                'home_ssid': config.get('home_ssid', ''), 'watchdog_seconds': config['watchdog_seconds'],
                'watchdog': self.watchdog_state(), 'laptops': self.laptops(),
                'pending': read_json(self.base / 'pending.json', {}).get('action', '')}

    # ---------- changes ----------
    def register(self, board, name, ssid, password, gateway, addresses, seconds=None):
        laptop_name(name)
        credentials(ssid, password)
        gw, parsed = network_plan(gateway, addresses)
        with self.locked():
            iface = self.interface()
            config = self.remember_home(iface)
            if seconds is not None:
                if type(seconds) is not int or not 30 <= seconds <= 900:
                    raise NetworkError('invalid_timeout')
                config['watchdog_seconds'] = seconds
                self.save_config(config)
            target = str(uuid.uuid5(NAMESPACE, name))
            path = self.profiles / (PREFIX + name + '.nmconnection')
            text = ('[connection]\nid={id}\nuuid={uuid}\ntype=wifi\ninterface-name={iface}\n'
                    'autoconnect=true\nautoconnect-priority={priority}\nautoconnect-retries=0\n\n'
                    '[wifi]\nmode=infrastructure\nssid={ssid}\n\n'
                    '[wifi-security]\nkey-mgmt=wpa-psk\npsk={psk}\npsk-flags=0\n\n'
                    '[ipv4]\nmethod=manual\naddress1={address}/{prefix},{gateway}\ndns={gateway};\n\n'
                    '[ipv6]\nmethod=ignore\n').format(
                        id=PREFIX + name, uuid=target, iface=iface, priority=LAPTOP_PRIORITY,
                        ssid=''.join(str(b) + ';' for b in ssid.encode('utf-8')), psk=key_value(password),
                        address=parsed[board], prefix=gw.network.prefixlen, gateway=gw.ip)
            self.profiles.mkdir(mode=0o700, parents=True, exist_ok=True)
            private_write(path, text)
            try:
                self.nm('connection', 'load', str(path))
            except NetworkError:
                path.unlink()
                raise
            self.write_network_peers(str(gw.network), parsed)
        return self.snapshot()

    def remove(self, name):
        laptop_name(name)
        with self.locked():
            iface = self.interface()
            target = str(uuid.uuid5(NAMESPACE, name))
            if self.active_uuid(iface) == target:
                raise NetworkError('laptop_in_use')
            try:
                self.nm('connection', 'delete', 'uuid', target)
            except NetworkError:
                pass
            try:
                (self.profiles / (PREFIX + name + '.nmconnection')).unlink()
            except OSError:
                pass
        return self.snapshot()

    def set_watchdog(self, seconds):
        if type(seconds) is not int or not 30 <= seconds <= 900:
            raise NetworkError('invalid_timeout')
        with self.locked():
            config = self.config()
            config['watchdog_seconds'] = seconds
            self.save_config(config)
        return self.snapshot()

    def schedule(self, action):
        """Run after 8 s in its own unit so the SSH session can reply first."""
        if action not in ('_indoor', '_classroom'):
            raise NetworkError('invalid_request')
        with self.locked():
            iface = self.interface()
            config = self.config()
            if not config.get('home_uuid'):
                raise NetworkError('home_wifi_unknown')
            if action == '_indoor':
                if not self.laptops():
                    raise NetworkError('no_laptop_registered')
                if self.active_uuid(iface) != config['home_uuid']:
                    raise NetworkError('not_on_classroom_wifi')
            private_write(self.base / 'pending.json', json.dumps({'action': action[1:], 'at': time.time()}) + '\n')
            unit = 'nomad-wifi-' + action[1:] + '-' + uuid.uuid4().hex[:8]
            try:
                self.run(['systemd-run', '--quiet', '--unit=' + unit, '--on-active=8s', '--timer-property=AccuracySec=1s',
                          '--property=Type=exec', '--property=StandardOutput=null', '--property=StandardError=null',
                          sys.executable, str(INSTALLED), action])
            except NetworkError:
                (self.base / 'pending.json').unlink()
                raise NetworkError('schedule_failed') from None
        return self.snapshot()

    def run_scheduled(self, action):
        with self.locked():
            config = self.config()
            try:
                (self.base / 'pending.json').unlink()
            except OSError:
                pass
            home = config.get('home_uuid')
            if not home:
                return
            iface = self.interface()
            if action == '_indoor':
                if self.active_uuid(iface) == home:
                    # User deactivation also blocks autoconnect of the classroom
                    # profile, so NetworkManager picks a saved laptop instead.
                    self.nm('connection', 'down', 'uuid', home, timeout=30)
            elif self.active_uuid(iface) != home:
                self.nm('--wait', '45', 'connection', 'up', 'uuid', home, 'ifname', iface, timeout=50)

    def write_network_peers(self, cidr, parsed):
        value = read_json(self.peers_path, {})
        networks = [n for n in value.get('networks', []) if isinstance(n, dict) and n.get('cidr') != cidr]
        networks.append(dict(cidr=cidr, **{k: str(v) for k, v in parsed.items()}))
        value['networks'] = networks[-8:]
        self.peers_path.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        private_write(self.peers_path, json.dumps(value) + '\n', 0o644)

    def save_classroom_peers(self, board, peers):
        if not isinstance(peers, dict) or set(peers) != set(BOARD_KEYS):
            raise NetworkError('invalid_peer_addresses')
        try:
            parsed = {k: ipaddress.IPv4Address(v) for k, v in peers.items()}
        except (ValueError, TypeError):
            raise NetworkError('invalid_peer_addresses') from None
        if len(set(parsed.values())) != 3:
            raise NetworkError('invalid_peer_addresses')
        with self.locked():
            iface = self.interface()
            if str(parsed[board]) not in self.addresses(iface):
                raise NetworkError('peer_address_not_current')
            value = read_json(self.peers_path, {})
            value.update({k: str(v) for k, v in parsed.items()})
            self.peers_path.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
            private_write(self.peers_path, json.dumps(value) + '\n', 0o644)
        return self.snapshot()

    # ---------- watchdog ----------
    def visible(self, iface, ssid):
        try:
            # A disconnected device keeps only a stale cache; ask for a fresh scan.
            lines = self.nm('-g', 'SSID', 'device', 'wifi', 'list', 'ifname', iface, '--rescan', 'yes', timeout=30).splitlines()
        except NetworkError:
            return False
        return ssid in [line.strip() for line in lines]

    def watchdog_step(self, since, now):
        """Returns the new 'disconnected since' time (or None when connected)."""
        iface = self.interface()
        if self.connected(iface):
            return None
        since = now if since is None else since
        config = self.config()
        home, ssid = config.get('home_uuid'), config.get('home_ssid')
        if home and ssid and now - since >= config['watchdog_seconds']:
            if self.visible(iface, ssid):
                try:
                    self.nm('--wait', '45', 'connection', 'up', 'uuid', home, 'ifname', iface, timeout=50)
                except NetworkError:
                    pass
            return now  # wait another full interval before the next attempt
        return since

    def watchdog(self):
        since = None
        while True:
            try:
                since = self.watchdog_step(since, time.monotonic())
            except Exception:
                pass
            time.sleep(5)

    # ---------- install / cleanup ----------
    def install(self, source):
        if Path(source).resolve() != INSTALLED.resolve():
            INSTALL_DIR.mkdir(mode=0o755, parents=True, exist_ok=True)
            shutil.copyfile(source, str(INSTALLED))
            os.chmod(str(INSTALLED), 0o755)
        private_write(UNIT_PATH, '[Unit]\nDescription=NOMAD laptop Wi-Fi fallback watchdog\n'
                      'After=NetworkManager.service\nWants=NetworkManager.service\n\n'
                      '[Service]\nType=simple\nExecStart=/usr/bin/python3 {agent} --watchdog\n'
                      'Restart=always\nRestartSec=5\n\n[Install]\nWantedBy=multi-user.target\n'.format(agent=INSTALLED), 0o644)
        removed = self.cleanup()
        self.run(['systemctl', 'daemon-reload'])
        self.run(['systemctl', 'enable', '--now', UNIT], timeout=30)
        self.run(['systemctl', 'restart', UNIT], timeout=30)
        with self.locked():
            iface = self.interface()
            try:
                self.remember_home(iface)
            except NetworkError:
                pass
        return removed

    def cleanup(self):
        removed = []
        iface = self.interface()
        active = self.active_uuid(iface)
        for item in self.connections():
            if item['name'].startswith(OLD_PROFILE_PREFIXES) and item['uuid'] != active:
                try:
                    self.nm('connection', 'delete', 'uuid', item['uuid'])
                    removed.append(item['name'])
                except NetworkError:
                    pass
        try:
            units = self.run(['systemctl', 'list-units', '--all', '--plain', '--no-legend', 'nomad-gui-wifi-*'])
            for line in units.splitlines():
                unit = line.split()[0] if line.split() else ''
                if unit.startswith('nomad-gui-wifi-'):
                    try:
                        self.run(['systemctl', 'stop', unit])
                    except NetworkError:
                        pass
        except NetworkError:
            pass
        for path in (OLD_AGENT, INSTALL_DIR / '__pycache__'):
            if path.is_dir():
                shutil.rmtree(str(path), ignore_errors=True)
            elif path.exists():
                path.unlink()
                removed.append(str(path))
        if OLD_STATE.exists():
            shutil.rmtree(str(OLD_STATE), ignore_errors=True)
            removed.append(str(OLD_STATE))
        return removed


def handle(request, agent):
    if not isinstance(request, dict):
        raise NetworkError('invalid_request')
    action = request.get('action')
    board, machine = board_identity()
    if request.get('board') != board:
        raise NetworkError('wrong_board')
    extra = {}
    if action == 'status':
        pass
    elif action == 'register':
        agent.register(board, request.get('name'), request.get('ssid'), request.get('password'),
                       request.get('gateway'), request.get('addresses'), request.get('seconds'))
    elif action == 'remove':
        agent.remove(request.get('name'))
    elif action == 'watchdog':
        agent.set_watchdog(request.get('seconds'))
    elif action == 'indoor':
        agent.schedule('_indoor')
    elif action == 'classroom':
        agent.schedule('_classroom')
    elif action == 'peers':
        agent.save_classroom_peers(board, request.get('peers'))
    elif action == 'install':
        extra['removed'] = agent.install(request.get('_source', str(INSTALLED)))
    else:
        raise NetworkError('invalid_request')
    return dict(ok=True, feature=FEATURE, board=board, machine_id=machine, **agent.snapshot(), **extra)


def main():
    os.umask(0o077)
    if os.geteuid() != 0:
        raise NetworkError('root_required')
    agent = Agent()
    if sys.argv[1:] == ['--watchdog']:
        agent.watchdog()
        return
    if len(sys.argv) == 2 and sys.argv[1] in ('_indoor', '_classroom'):
        agent.run_scheduled(sys.argv[1])
        return
    if len(sys.argv) == 3 and sys.argv[1] == '--request-file':
        path = Path(sys.argv[2])
        try:
            if path.is_symlink() or not path.is_file() or path.stat().st_size > 8192:
                raise NetworkError('invalid_request')
            request = json.loads(path.read_text())
        finally:
            try:
                path.unlink()
            except OSError:
                pass
        if request.get('action') == 'install':
            request['_source'] = os.path.abspath(__file__)
        print(MARKER + json.dumps(handle(request, agent), ensure_ascii=False), flush=True)
        return
    raise NetworkError('invalid_request')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        reason = str(exc) if isinstance(exc, NetworkError) else 'agent_error'
        print(MARKER + json.dumps(dict(ok=False, feature=FEATURE, reason=reason)), flush=True)
        sys.exit(1)
