#!/usr/bin/env bash
set -euo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  printf '사용법: %s [--subnet 192.168.3.0/24]\n저장된 보드 계정으로 실제 로그인 후 기종과 접속 명령을 출력합니다.\n' "$0"
  exit 0
fi
python3 - "$@" <<'PY'
import argparse
import concurrent.futures
import ipaddress
import json
import os
import socket
import subprocess
import sys
import tempfile
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--subnet')
args = parser.parse_args()
# GUI supplies this in the child environment; never commit credentials.
password = os.environ.get("NOMAD_BOARD_PASSWORD", os.environ.get("NOMAD_BOARD_PASSWORD1", ""))

def ip_json(*arguments):
    try:
        return json.loads(subprocess.check_output(['ip', '-j', '-4', *arguments], stderr=subprocess.DEVNULL, timeout=3))
    except (OSError, ValueError, subprocess.SubprocessError):
        return []

addresses = ip_json('address', 'show')
local_ips = {a['local'] for i in addresses for a in i.get('addr_info', []) if a.get('family') == 'inet'}
routes = ip_json('route', 'show', 'default')
interface = min(routes, key=lambda r: r.get('metric', 0)).get('dev') if routes else None
networks = []
if args.subnet:
    networks = [ipaddress.ip_network(args.subnet, strict=False)]
else:
    networks = [ipaddress.ip_interface(f"{a['local']}/{a['prefixlen']}").network
                for i in addresses if i.get('ifname') == interface
                for a in i.get('addr_info', []) if a.get('family') == 'inet']
if any(n.version != 4 or n.num_addresses > 4096 for n in networks):
    parser.error('IPv4 /20 이상 크기의 대역은 지정할 수 없습니다. /24 등 작은 대역을 지정하세요.')

candidates = {'192.168.1.117', '192.168.1.82', '192.168.1.152', '192.168.55.1',
              '192.168.3.142', '192.168.3.144', '192.168.3.95', '10.42.0.224'}
candidates.update(r['dst'] for r in ip_json('neigh', 'show') if 'dst' in r)
for network in networks:
    candidates.update(str(host) for host in network.hosts())

def resolve(name):
    try:
        result = subprocess.check_output(['getent', 'ahostsv4', name], stderr=subprocess.DEVNULL, timeout=3).decode()
        return {line.split()[0] for line in result.splitlines() if line.split()}
    except (OSError, subprocess.SubprocessError):
        return set()

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    for found in pool.map(resolve, ['user-desktop.local', 'lpp.local', 'ubuntu.local', 'tegra-ubuntu.local']):
        candidates.update(found)
candidates.difference_update(local_ips)
candidates = {ip for ip in candidates if isinstance(ipaddress.ip_address(ip), ipaddress.IPv4Address)}

def ssh_open(ip):
    try:
        with socket.create_connection((ip, 22), timeout=0.5):
            return ip
    except OSError:
        return None

def get_model(ip, account, known_hosts, askpass):
    command = ['ssh', '-F', '/dev/null', '-o', 'ConnectTimeout=3', '-o', 'ConnectionAttempts=1',
               '-o', 'PreferredAuthentications=password,keyboard-interactive', '-o', 'PubkeyAuthentication=no',
               '-o', 'NumberOfPasswordPrompts=1', '-o', 'StrictHostKeyChecking=accept-new',
               '-o', f'UserKnownHostsFile={known_hosts}', '-o', 'LogLevel=ERROR',
               f'{account}@{ip}', "printf 'NOMAD_BOARD_MODEL='; cat /proc/device-tree/model 2>/dev/null"]
    environment = os.environ.copy()
    environment.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force',
                       DISPLAY=':nomad-discovery', NOMAD_BOARD_PASSWORD=password)
    try:
        result = subprocess.run(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, env=environment, timeout=8,
                                start_new_session=True)
        text = result.stdout.decode(errors='replace').replace('\x00', '')
        marker = 'NOMAD_BOARD_MODEL='
        if result.returncode == 0 and marker in text:
            return text.split(marker, 1)[1].splitlines()[0].strip()
        return ''
    except (OSError, subprocess.SubprocessError, IndexError):
        return ''

def identify(ip, directory):
    # 계정·과거 주소로 기종을 추정하지 않고 로그인 후 device-tree 모델을 확인한다.
    with tempfile.NamedTemporaryFile(dir=directory) as known_hosts:
        for account in ('user', 'nvidia'):
            model = get_model(ip, account, known_hosts.name, Path(directory) / 'askpass')
            if not model:
                continue
            if account == 'user' and 'Raspberry Pi 5' in model:
                return 'Pi 5', account, ip
            if account == 'user' and 'Raspberry Pi 4' in model:
                return 'Pi 4', account, ip
            if account == 'nvidia' and ('Jetson' in model or 'Xavier' in model):
                return 'Jetson', account, ip
    return None

print('보드 SSH 접속 확인 중…', file=sys.stderr, flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=96) as pool:
    reachable = [ip for ip in pool.map(ssh_open, sorted(candidates)) if ip]
with tempfile.TemporaryDirectory(prefix='nomad-board-discovery-') as directory:
    askpass = Path(directory) / 'askpass'
    askpass.write_text('#!/bin/sh\nprintf "%s\\n" "$NOMAD_BOARD_PASSWORD"\n')
    askpass.chmod(0o700)
    with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
        found = [board for board in pool.map(lambda ip: identify(ip, directory), reachable) if board]
order = {'Pi 5': 0, 'Jetson': 1, 'Pi 4': 2}
for label, account, ip in sorted(found, key=lambda item: (order[item[0]], item[2])):
    print(f'{label}: ssh {account}@{ip}')
for label in ('Pi 5', 'Jetson', 'Pi 4'):
    if not any(row[0] == label for row in found):
        print(f'{label}: 연결 안 됨')
sys.exit(0 if found else 1)
PY
