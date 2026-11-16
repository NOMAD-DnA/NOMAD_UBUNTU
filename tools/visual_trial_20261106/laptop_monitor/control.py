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
# Fixed wired Pi5 address behind the Jetson (oak_fastdds_wired.xml 10.77.0.1/30).
# Used only when the Pi5 Wi-Fi address cannot be reached, e.g. after DHCP moved it.
PI5_WIRED_HOST = "user@10.77.0.2"
# A changed host key or login refusal at the old Wi-Fi address can mean another
# device now holds it; the wired retry still verifies the stored Pi5 host key.
DIRECT_UNREACHABLE = frozenset({"ssh_unreachable", "timeout", "host_key_check_failed", "authentication_failed"})

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


def run_remote(host, script_path, password, timeout_s=18, *, sudo=False, via=None, host_key_alias=None,
               follow=True) -> dict:
    """Run one shipped shell script, bounded by timeout_s wall clock seconds.

    via: optional user@address of a board to hop through (ssh -W). host_key_alias:
    verify the target against this known board address instead of the hop-side one.
    follow: use the board's recovered route (board_address) and, if the board is
    unreachable, look for it once and retry there. The result then carries
    "route" (mac, mdns or wired) and "target".
    """
    if not follow or via is not None or host_key_alias is not None:
        return _run_remote_once(host, script_path, password, timeout_s, sudo=sudo, via=via,
                                host_key_alias=host_key_alias)
    try:
        import board_address
        configured = validate_host(host)
    except (ImportError, ValueError):
        return _run_remote_once(host, script_path, password, timeout_s, sudo=sudo)
    target = board_address.route(configured)
    value = _run_remote_once(target["host"], script_path, password, timeout_s, sudo=sudo,
                             via=target.get("via"), host_key_alias=target.get("alias"))
    still_there = value.get("reason") == "timeout" and board_address._ssh_open(target["host"].split("@", 1)[1])
    # A slow script on a reachable board is never re-run on another route.
    if value.get("reason") in DIRECT_UNREACHABLE and not still_there:
        found = board_address.recover(configured)
        if found:
            target = found
            value = _run_remote_once(target["host"], script_path, password, timeout_s, sudo=sudo,
                                     via=target.get("via"), host_key_alias=target.get("alias"))
    if target.get("method"):
        value["route"] = target["method"]
        value["target"] = target["host"] + (" via " + target["via"] if target.get("via") else "")
    if value.get("reason") == "ok" or value.get("returncode") == 0:
        board_address.learn(configured, target)
    return value


def _connection_reuse_options(state_home):
    """2026-11-12: reuse one authenticated SSH connection per board for 60 s (startup
    runs ~7 scripts per board; each new login costs about 0.5-1.5 s). The socket must
    fit the 108-byte Unix path limit (%C = 40 characters); otherwise no reuse."""
    candidates = []
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime:
        candidates.append(Path(runtime) / "nomad-ssh")
    candidates.append(Path(state_home))
    for directory in candidates:
        try:
            if len(str(directory / "cm-")) + 40 >= 100:
                continue
            directory.mkdir(mode=0o700, parents=True, exist_ok=True)
            info = directory.stat()
            if info.st_uid != os.getuid() or info.st_mode & 0o077:
                continue  # never share a control socket folder with other users
            return ["-o", "ControlMaster=auto", "-o", "ControlPersist=60",
                    "-o", "ControlPath=" + str(directory / "cm-%C")]
        except OSError:
            continue
    return []


def _run_remote_once(host, script_path, password, timeout_s=18, *, sudo=False, via=None,
                     host_key_alias=None) -> dict:
    try:
        host = validate_host(host)
        if via is not None:
            via = validate_host(via)
        if host_key_alias is not None:
            host_key_alias = validate_host("x@" + str(host_key_alias)).split("@", 1)[1]
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
            common = ["-o", "ConnectTimeout=5", "-o", "ConnectionAttempts=1",
                      "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=1",
                      "-o", "NumberOfPasswordPrompts=1", "-o", "StrictHostKeyChecking=accept-new",
                      "-o", 'UserKnownHostsFile="' + str(state_home / "known_hosts") + '"',
                      "-o", "LogLevel=ERROR", "-o", "BatchMode=" + ("no" if password else "yes"),
                      *_connection_reuse_options(state_home)]
            route = []
            if via is not None:
                hop = " ".join(shlex.quote(part) for part in ["ssh", *common, "-W", "%h:%p", via])
                route += ["-o", "ProxyCommand=" + hop]
            if host_key_alias is not None:
                route += ["-o", "HostKeyAlias=" + host_key_alias]
            args = ["ssh", "-T", *common, *route,
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


def _stop_details(value):
    """2026-11-16: carry the board's survivor list and recording guard to the GUI."""
    remote = value.get("remote") if isinstance(value, dict) else None
    if isinstance(remote, dict):
        survivors = remote.get("survivors")
        if isinstance(survivors, str) and survivors:
            value["survivors"] = survivors[:300]
        if remote.get("reason") == "recording_active" and not value.get("ok"):
            value["reason"] = "recording_active"
        if remote.get("recording") is True:      # recording marker or kept recording sensors
            value["recording_active"] = True
    return value


def _jetson_recording_check(jetson_host, password):
    """Read-only Jetson sensor-recording status (remote/record_status_jetson.sh)."""
    import sensor_record
    return sensor_record.status(jetson_host, password)


def stop_boards(pi_host, jetson_host, password, progress_callback: Callable | None = None,
                timeout_s=18, only=None) -> dict:
    """Stop both current trial runs concurrently; board power stays on.

    only: boards to stop (e.g. {"pi5"} when retrying after a partial stop).
    A board that does not answer at its address is looked for again by
    run_remote (board_address: same MAC + host key, host name, or the
    Jetson <-> Pi5 wired link) and the stop is retried there.
    """
    hosts = {"pi5": pi_host, "jetson": jetson_host}
    try:
        import board_address
        board_address.register(pi5=pi_host, jetson=jetson_host)
    except Exception:
        pass
    if only is not None:
        hosts = {board: host for board, host in hosts.items() if board in only}
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
                boards[board] = _stop_details(future.result())
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
    total = len(boards)
    state = "STOPPED" if count == total else "PARTIAL_FAILURE" if count else "FAILED"
    return {"ok": count == total, "state": state, "boards": boards}


POWER_MARKER = "NOMAD_POWER_RESULT="
POWER_BOARDS = ("pi4", "pi5", "jetson")   # Jetson last: Pi5 may only be reachable through it
POWER_USERS = {"pi4": "user", "pi5": "user", "jetson": "nvidia"}


def _is_local_address(address):
    """True for this laptop's own names/addresses: the GUI must never power it off."""
    import socket
    name = address.lower().rstrip(".")
    if name in ("localhost", "localhost.localdomain") or name.endswith(".localhost"):
        return True
    try:
        if name in (socket.gethostname().lower(), socket.getfqdn().lower()):
            return True
    except OSError:
        pass
    try:
        parsed = ipaddress.ip_address(address)
    except ValueError:
        return False
    if parsed.is_loopback or parsed.is_unspecified:
        return True
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind((str(parsed), 0))   # succeeds only for an address of this machine
        return True
    except OSError:
        return False


def _power_one(board, host, password, timeout_s=25):
    """Ship remote/poweroff_board.sh with the expected board kind; sudo, no route retry."""
    with tempfile.TemporaryDirectory(prefix="nomad-power-") as folder:
        script = Path(folder) / "poweroff_board.sh"
        script.write_text(f"EXPECTED={board}\n" + (BASE_DIR / "remote/poweroff_board.sh").read_text(encoding="utf-8"),
                          encoding="utf-8")
        value = run_remote(host, script, password, timeout_s, sudo=True, follow=False)
    stdout = value.get("stdout", "") if isinstance(value.get("stdout", ""), str) else ""
    markers = [line[len(POWER_MARKER):] for line in stdout.splitlines() if line.startswith(POWER_MARKER)]
    remote = None
    if markers:
        try:
            remote = json.loads(markers[-1])
        except ValueError:
            remote = None
    if not isinstance(remote, dict):
        reason = value.get("reason") or "missing_result"
        if reason in ("ok", "remote_script_failed"):
            reason = "sudo_failed"      # sudo refused the password before the script ran
        return {"ok": False, "reason": reason}
    ok = remote.get("ok") is True
    return {"ok": ok, "reason": str(remote.get("reason") or ("scheduled" if ok else "failed"))}


def poweroff_boards(pi_host, jetson_host, password, pi4_host=None, boards=("pi5", "jetson"),
                    progress_callback: Callable | None = None, stop=None, recording_check=None) -> dict:
    """2026-11-16 '보드 전원 끄기': trial stop first, then Pi4 -> Pi5 -> Jetson (last).

    Refused while a Jetson sensor recording is running: checked read-only before the
    trial stop (whenever a Jetson address is given, even if the Jetson is not chosen;
    an unreadable status refuses), and again from the stop results. Each board checks its own
    model before scheduling poweroff (remote/poweroff_board.sh), so a wrong address
    is refused there; this laptop's own addresses are refused here. The laptop never
    runs poweroff/shutdown: the command only travels to the board inside SSH stdin.
    """
    stop = stop or stop_boards
    recording_check = recording_check or _jetson_recording_check
    hosts = {"pi4": pi4_host, "pi5": pi_host, "jetson": jetson_host}
    chosen = [board for board in POWER_BOARDS if board in set(boards or ())]
    results, valid = {}, {}
    for board in chosen:
        try:
            normalized = validate_host(hosts[board] or "")
            user, address = normalized.split("@", 1)
            if user != POWER_USERS[board]:
                raise ValueError("Wrong board account")
            if _is_local_address(address):
                results[board] = {"ok": False, "reason": "local_address_refused"}
                continue
            if normalized in valid.values():
                results[board] = {"ok": False, "reason": "duplicate_address"}
                continue
            valid[board] = normalized
        except ValueError:
            results[board] = {"ok": False, "reason": "invalid_board_address"}
    if not chosen:
        return {"ok": False, "state": "FAILED", "reason": "no_board_selected", "boards": {}}
    if results:
        return {"ok": False, "state": "REFUSED", "reason": "invalid_board_address", "boards": results}
    if not password:
        return {"ok": False, "state": "REFUSED", "reason": "sudo_password_required", "boards": {}}
    if jetson_host:
        try:
            status = recording_check(jetson_host, password)
        except Exception:
            status = {"ok": False, "reason": "local_control_error"}
        status = status if isinstance(status, dict) else {}
        if status.get("ok") and status.get("recording") is True:
            return {"ok": False, "state": "REFUSED", "reason": "recording_active", "boards": {}}
        # No SSD means no recording can run there.
        if not (status.get("ok") and status.get("recording") is False) and status.get("reason") != "no_ssd":
            return {"ok": False, "state": "REFUSED", "reason": "recording_check_failed",
                    "check": str(status.get("reason") or "unknown")[:60], "boards": {}}
    stop_targets = {board for board in ("pi5", "jetson") if board in valid}
    stopped = None
    if stop_targets:
        stopped = stop(pi_host, jetson_host, password, only=stop_targets)
        if progress_callback:
            try:
                progress_callback("stop", dict(stopped))
            except Exception:
                pass
        if any(value.get("reason") == "recording_active" or value.get("recording_active")
               for value in stopped.get("boards", {}).values()):
            return {"ok": False, "state": "REFUSED", "reason": "recording_active", "stop": stopped, "boards": {}}
    for board in chosen:
        try:
            results[board] = _power_one(board, valid[board], password)
        except Exception:
            results[board] = {"ok": False, "reason": "local_control_error"}
        if progress_callback:
            try:
                progress_callback(board, dict(results[board]))
            except Exception:
                pass
    count = sum(bool(value.get("ok")) for value in results.values())
    state = "SCHEDULED" if count == len(results) else "PARTIAL_FAILURE" if count else "FAILED"
    return {"ok": count == len(results), "state": state, "stop": stopped, "boards": results}


def main():
    parser = argparse.ArgumentParser(description="Stop the NOMAD visual trial (board power stays on), "
                                                 "or power off chosen boards (never this laptop)")
    parser.add_argument("action", choices=("stop", "poweroff"))
    parser.add_argument("--pi-host", required=True)
    parser.add_argument("--jetson-host", required=True)
    parser.add_argument("--pi4-host")
    parser.add_argument("--boards", default="pi5,jetson", help="poweroff only: comma list of pi4,pi5,jetson")
    parser.add_argument("--state-dir")
    args = parser.parse_args()
    if args.action == "poweroff":
        boards = [name for name in args.boards.split(",") if name in POWER_BOARDS]
        result = poweroff_boards(args.pi_host, args.jetson_host, os.environ.get("NOMAD_BOARD_PASSWORD1"),
                                 pi4_host=args.pi4_host, boards=boards)
    else:
        result = stop_boards(args.pi_host, args.jetson_host, os.environ.get("NOMAD_BOARD_PASSWORD1"))
    if args.state_dir:
        target = Path(args.state_dir)
        target.mkdir(parents=True, exist_ok=True)
        output = target / ("poweroff_result.json" if args.action == "poweroff" else "stop_result.json")
        temporary = output.with_suffix(".tmp")
        temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, output)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
