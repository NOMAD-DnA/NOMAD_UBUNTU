#!/usr/bin/env python3
"""Laptop-only clock handoff before starting board ROS modules."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import selectors
import shlex
import subprocess
import tempfile
import time

from control import validate_host

REMOTE = r'''set -eu
model=$(tr -d '\0' < /proc/device-tree/model)
case "$model" in *'Raspberry Pi'*|*Jetson*|*Xavier*) ;; *) exit 21 ;; esac
IFS= read -r nomad_password
printf '%s\n' 'NOMAD_CLOCK_READY'
IFS= read -r nomad_epoch
case "$nomad_epoch" in ''|*[!0-9.]*|.*) exit 22 ;; esac
if [ -n "$nomad_password" ]; then
  printf '%s\n' "$nomad_password" | sudo -k -S -p '' date -u --set="@$nomad_epoch" '+CLOCK_SET=%s.%N'
else
  sudo -n date -u --set="@$nomad_epoch" '+CLOCK_SET=%s.%N'
fi
'''


def synchronize_clock(host, password, timeout_s=12):
    """Send current host UTC after SSH is ready, excluding login latency.

    Board side uses existing shell/date only. Never alter sensor message stamps.
    Call only before starting the relevant ROS nodes.
    """
    try:
        host = validate_host(host)
        password = password or ""
        if not isinstance(password, str) or '\n' in password or '\r' in password:
            raise ValueError('Invalid password')
    except ValueError:
        return {'ok': False, 'reason': 'clock_arguments', 'state': 'FAILED'}
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state')))/'nomad-trial'
    state.mkdir(parents=True, exist_ok=True, mode=0o700)
    process = None
    deadline = time.monotonic()+timeout_s
    try:
        with tempfile.TemporaryDirectory(prefix='nomad-clock-') as temporary:
            env = os.environ.copy()
            if password:
                askpass=Path(temporary)/'askpass'
                askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_SSH_PASSWORD"\n')
                askpass.chmod(0o700)
                env.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE='force', NOMAD_SSH_PASSWORD=password)
                env.setdefault('DISPLAY', ':0')
            args=['ssh','-T','-o','ConnectTimeout=5','-o','NumberOfPasswordPrompts=1',
                  '-o','StrictHostKeyChecking=accept-new','-o','LogLevel=ERROR',
                  '-o','UserKnownHostsFile="'+str(state/'known_hosts')+'"',
                  '-o','BatchMode='+('no' if password else 'yes'),
                  host,'bash -c '+shlex.quote(REMOTE)]
            process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE,text=True,env=env)
            process.stdin.write(password+'\n')
            process.stdin.flush()
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout,selectors.EVENT_READ)
                while time.monotonic()<deadline:
                    if not selector.select(max(0,deadline-time.monotonic())):
                        raise subprocess.TimeoutExpired('clock handshake',timeout_s)
                    line=process.stdout.readline()
                    if line.strip()=='NOMAD_CLOCK_READY':
                        epoch=time.time_ns()
                        process.stdin.write(f'{epoch//1000000000}.{epoch%1000000000:09d}\n')
                        process.stdin.flush()
                        break
                    if not line:
                        raise OSError('Clock handshake closed')
                else:
                    raise subprocess.TimeoutExpired('clock handshake',timeout_s)
            stdout,_=process.communicate(timeout=max(.1,deadline-time.monotonic()))
            ok=process.returncode==0 and any(s.startswith('CLOCK_SET=') for s in stdout.splitlines())
            return {'ok':ok,'reason':'clock_synchronized' if ok else 'clock_set_failed',
                    'state':'SUCCEEDED' if ok else 'FAILED'}
    except (OSError,ValueError,subprocess.TimeoutExpired):
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        return {'ok':False,'reason':'clock_connection_failed','state':'FAILED'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True)
    args=parser.parse_args()
    result=synchronize_clock(args.host,os.environ.get('NOMAD_BOARD_PASSWORD1',''))
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return 0 if result['ok'] else 1


if __name__=='__main__':
    raise SystemExit(main())
