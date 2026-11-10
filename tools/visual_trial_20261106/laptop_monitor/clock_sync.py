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
# Complete privilege authentication before the laptop captures the reference time.
# Authentication after receiving the reference made the board clock lag by that wait.
if [ -n "$nomad_password" ]; then
  printf '%s\n' "$nomad_password" | sudo -k -S -p '' -v
else
  sudo -n -v
fi
unset nomad_password
printf '%s\n' 'NOMAD_CLOCK_READY'
IFS= read -r nomad_epoch
case "$nomad_epoch" in ''|*[!0-9.]*|.*) exit 22 ;; esac
sudo -n date -u --set="@$nomad_epoch" '+CLOCK_SET=%s.%N' >/dev/null
printf 'NOMAD_CLOCK_SAMPLE_READY\n'
IFS= read -r nomad_probe
[ "$nomad_probe" = NOMAD_CLOCK_PROBE ] || exit 23
date -u '+CLOCK_SAMPLE=%s.%N'
'''


def assess_clock_sample(sample, sent_wall_ns, sent_monotonic_ns,
                        received_wall_ns, received_monotonic_ns):
    """Bound clock skew from a short read-only probe, without changing stamps.

    A remote clock sample was taken between local send/receive.  Network and
    command scheduling uncertainty is half that interval, not zero.
    """
    try:
        seconds, fraction = sample.split(".")
        if not seconds.isdigit() or len(fraction) != 9 or not fraction.isdigit():
            raise ValueError("Invalid clock sample")
        remote_ns = int(seconds) * 1_000_000_000 + int(fraction)
        elapsed = received_monotonic_ns - sent_monotonic_ns
        if elapsed < 0 or remote_ns <= 0:
            raise ValueError("Invalid clock interval")
        if abs((received_wall_ns - sent_wall_ns) - elapsed) > 50_000_000:
            return dict(ok=False, reason="operator_clock_changed", state="FAILED")
        midpoint_ns = sent_wall_ns + elapsed // 2
        estimate_ns = remote_ns - midpoint_ns
        uncertainty_ns = elapsed // 2
        result = dict(offset_estimate_s=estimate_ns / 1e9,
                      uncertainty_s=uncertainty_ns / 1e9,
                      probe_round_trip_s=elapsed / 1e9)
        # Keep residual skew below the existing 0.2s future-stamp tolerance.
        bounded = abs(estimate_ns) + uncertainty_ns <= 150_000_000
        result.update(ok=bounded,
                      reason="clock_synchronized" if bounded else "clock_skew_or_probe_delay",
                      state="SUCCEEDED" if bounded else "FAILED")
        return result
    except (ValueError, TypeError, AttributeError):
        return dict(ok=False, reason="clock_sample_invalid", state="FAILED")


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
                # Probe only after sudo/date has completed.  This interval
                # excludes the clock-setting and authentication work.
                while time.monotonic() < deadline:
                    if not selector.select(max(0, deadline-time.monotonic())):
                        raise subprocess.TimeoutExpired('clock probe ready', timeout_s)
                    line = process.stdout.readline().strip()
                    if line == 'NOMAD_CLOCK_SAMPLE_READY':
                        break
                    elif not line:
                        raise OSError('Clock probe handshake closed')
                else:
                    raise subprocess.TimeoutExpired('clock probe ready', timeout_s)
                probe_wall_ns = time.time_ns()
                probe_monotonic_ns = time.monotonic_ns()
                process.stdin.write('NOMAD_CLOCK_PROBE\n')
                process.stdin.flush()
                if not selector.select(max(0, deadline-time.monotonic())):
                    raise subprocess.TimeoutExpired('clock probe', timeout_s)
                sample_line = process.stdout.readline().strip()
                received_monotonic_ns = time.monotonic_ns()
                received_wall_ns = time.time_ns()
            process.communicate(timeout=max(.1,deadline-time.monotonic()))
            if process.returncode != 0 or not sample_line.startswith('CLOCK_SAMPLE='):
                return dict(ok=False, reason='clock_set_failed', state='FAILED')
            return assess_clock_sample(sample_line.partition('=')[2], probe_wall_ns,
                                       probe_monotonic_ns, received_wall_ns,
                                       received_monotonic_ns)
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
