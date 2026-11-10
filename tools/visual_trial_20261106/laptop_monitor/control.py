"""Laptop SSH controls for the already installed NOMAD visual trial.

Passwords exist only in this process/SSH_ASKPASS environment. Remote scripts
are supplied on stdin; no password is embedded in a command line or log.
"""
from __future__ import annotations

import argparse
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
from typing import Callable

BASE_DIR = Path(__file__).resolve().parent
RESULT_MARKER = "NOMAD_STOP_RESULT="

# The transport wrapper ALWAYS consumes the password line itself. Passing
# password+script directly to `sudo -S bash -s` is unsafe when sudo credentials
# are cached or the account has NOPASSWD: bash would execute the password line.
# Here sudo's password pipe and the on-disk script are separate channels.
SUDO_WRAPPER = r'''set -eu
IFS= read -r nomad_password
nomad_action_dir=$(mktemp -d /tmp/nomad-action.XXXXXXXX)
trap 'rm -rf -- "$nomad_action_dir"' EXIT
chmod 700 "$nomad_action_dir"
cat > "$nomad_action_dir/action.sh"
chmod 600 "$nomad_action_dir/action.sh"
printf '%s\n' "$nomad_password" | sudo -k -S -p '' bash -c 'exec bash "$1" </dev/null' nomad-root "$nomad_action_dir/action.sh"
'''


def validate_host(host: str) -> str:
    """Return a safe SSH user@IPv4/hostname, or raise ValueError."""
    if not isinstance(host, str):
        raise ValueError("SSH address must be user@address")
    host = host.strip()
    if host.count("@") != 1 or len(host) > 280:
        raise ValueError("SSH address must be user@address")
    user, address = host.split("@", 1)
    if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", user):
        raise ValueError("Invalid SSH user")
    try:
        parsed = ipaddress.ip_address(address)
        if parsed.version != 4 or parsed.is_unspecified or parsed.is_multicast:
            raise ValueError("Use an IPv4 board address")
        address = str(parsed)
    except ValueError:
        # Do not reinterpret malformed dotted IPv4 values as host names.
        if re.fullmatch(r"[0-9.]+", address):
            raise ValueError("Invalid board IPv4 address") from None
        labels = address.split(".")
        if not labels or any(not re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?", part) for part in labels):
            raise ValueError("Invalid board host name") from None
    return f"{user}@{address}"


def _failure(reason: str, stdout: str = "", returncode=None) -> dict:
    return {"ok": False, "stdout": stdout, "reason": reason,
            "returncode": returncode, "state": "FAILED"}


def run_remote(host, script_path, password, timeout_s=18, *, sudo=False) -> dict:
    """Run one shipped shell script, bounded by timeout_s wall clock seconds."""
    try:
        host = validate_host(host)
        script = Path(script_path).read_text(encoding="utf-8")
        timeout_s = float(timeout_s)
        if not 0 < timeout_s <= 300:
            raise ValueError("Invalid timeout")
        if password is not None and (not isinstance(password, str) or "\n" in password or "\r" in password):
            raise ValueError("Invalid password")
        if sudo and not password:
            return _failure("sudo_password_required")
    except (OSError, ValueError, TypeError):
        return _failure("invalid_host_script_or_options")

    state_home = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial"
    process = None
    try:
        state_home.mkdir(mode=0o700, parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="nomad-ssh-") as temporary:
            env = os.environ.copy()
            if password:
                askpass = Path(temporary) / "askpass"
                askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_SSH_PASSWORD"\n', encoding="utf-8")
                askpass.chmod(0o700)
                env.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE="force",
                           NOMAD_SSH_PASSWORD=password)
                env.setdefault("DISPLAY", ":0")
            args = ["ssh", "-T", "-o", "ConnectTimeout=5", "-o", "ConnectionAttempts=1",
                    "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=1",
                    "-o", "NumberOfPasswordPrompts=1", "-o", "StrictHostKeyChecking=accept-new",
                    "-o", 'UserKnownHostsFile="' + str(state_home / "known_hosts") + '"',
                    "-o", "LogLevel=ERROR", "-o", "BatchMode=" + ("no" if password else "yes"),
                    host, "bash -c " + shlex.quote(SUDO_WRAPPER) if sudo else "bash -s"]
            payload = ((password + "\n") if sudo else "") + script + "\n"
            process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True, env=env, start_new_session=True)
            try:
                stdout, stderr = process.communicate(payload, timeout=timeout_s)
            except subprocess.TimeoutExpired:
                # This kills only our laptop SSH session, never arbitrary board processes.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.communicate()
                return _failure("timeout", returncode=process.returncode)
            returncode = process.returncode
            reason = "ok"
            if returncode:
                lowered = stderr.lower()
                if returncode == 255:
                    if "permission denied" in lowered or "incorrect password" in lowered:
                        reason = "authentication_failed"
                    elif "host key verification failed" in lowered or "identification has changed" in lowered:
                        reason = "host_key_check_failed"
                    else:
                        reason = "ssh_unreachable"
                else:
                    # A command can report permission denied AFTER SSH login
                    # succeeds, e.g. a readable but non-executable board script.
                    reason = "remote_script_failed"
            value = {"ok": returncode == 0, "stdout": stdout, "reason": reason,
                     "returncode": returncode, "state": "SUCCEEDED" if returncode == 0 else "FAILED"}
            # Stop scripts report actual process cleanup independently of SSH reachability.
            markers = [line[len(RESULT_MARKER):] for line in stdout.splitlines() if line.startswith(RESULT_MARKER)]
            if markers:
                try:
                    remote = json.loads(markers[-1])
                    if isinstance(remote, dict):
                        value["remote"] = remote
                        value["state"] = remote.get("state", "FAILED")
                        value["ok"] = returncode == 0 and remote.get("state") == "STOPPED"
                        if not value["ok"] and reason == "ok":
                            value["reason"] = "remote_cleanup_incomplete"
                except (ValueError, TypeError):
                    return _failure("invalid_remote_result", returncode=returncode)
            # Raw stderr is intentionally never saved/returned: askpass/sudo errors can
            # contain command text. The stable reason above is sufficient for the UI.
            return value
    except (OSError, ValueError):
        return _failure("local_ssh_failed")


def stop_boards(pi_host, jetson_host, password, progress_callback: Callable | None = None,
                timeout_s=18) -> dict:
    """Stop both current trial runs concurrently; board power stays on."""
    hosts = {"pi5": pi_host, "jetson": jetson_host}
    boards = {}
    expected_users = {"pi5": "user", "jetson": "nvidia"}
    for board, host in hosts.items():
        try:
            normalized = validate_host(host)
            if normalized.split("@", 1)[0] != expected_users[board]:
                raise ValueError("Wrong board account")
            hosts[board] = normalized
        except ValueError:
            boards[board] = _failure("invalid_board_address")
    # Partial success is retained and retryable; an unreachable board is never
    # reported as STOPPED merely because its SSH connection has disappeared.
    with ThreadPoolExecutor(max_workers=2) as executor:
        pending = {executor.submit(run_remote, host, BASE_DIR / "remote/stop_trial.sh", password,
                                   min(float(timeout_s), 18)): board
                   for board, host in hosts.items() if board not in boards}
        for future in as_completed(pending):
            board = pending[future]
            try:
                boards[board] = future.result()
                if boards[board].get("ok") and boards[board].get("state") != "STOPPED":
                    boards[board] = _failure("invalid_remote_stop_result")
            except Exception:
                boards[board] = _failure("local_control_error")
            if progress_callback:
                try:
                    progress_callback(board, dict(boards[board]))
                except Exception:
                    pass
    count = sum(bool(value.get("ok")) for value in boards.values())
    state = "STOPPED" if count == 2 else "PARTIAL_FAILURE" if count else "FAILED"
    return {"ok": count == 2, "state": state, "boards": boards}


def main():
    parser = argparse.ArgumentParser(description="Stop only the current NOMAD visual trial; keep board power on")
    parser.add_argument("action", choices=("stop",))
    parser.add_argument("--pi-host", required=True)
    parser.add_argument("--jetson-host", required=True)
    parser.add_argument("--state-dir")
    args = parser.parse_args()
    result = stop_boards(args.pi_host, args.jetson_host, os.environ.get("NOMAD_BOARD_PASSWORD1"))
    if args.state_dir:
        target = Path(args.state_dir)
        target.mkdir(parents=True, exist_ok=True)
        output = target / "stop_result.json"
        temporary = output.with_suffix(".tmp")
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, output)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
