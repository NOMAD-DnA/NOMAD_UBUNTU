"""Offline controls tests. Never contact or stop a real board."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("trial_control", ROOT / "control.py")
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


class FakeSSH:
    returncode = 0
    pid = 99999999
    calls = []

    def __init__(self, args, **kwargs):
        self.args, self.kwargs = args, kwargs
        self.__class__.calls.append(self)

    def communicate(self, payload=None, timeout=None):
        self.payload = payload
        return 'NOMAD_STOP_RESULT={"state":"STOPPED","remaining":0}\n', ""


class Controls(unittest.TestCase):
    def test_remote_permission_error_is_not_ssh_authentication_failure(self):
        cases = ((126, "bash: /board/switch_model.sh: Permission denied", "remote_script_failed"),
                 (1, "Permission denied", "remote_script_failed"),
                 (255, "Permission denied (publickey,password)", "authentication_failed"),
                 (255, "Host key verification failed", "host_key_check_failed"),
                 (1, "Host key verification failed", "remote_script_failed"),
                 (255, "No route to host", "ssh_unreachable"))
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {"XDG_STATE_HOME": directory}):
            script = Path(directory) / "script.sh"
            script.write_text("exit 0\n")
            for code, stderr, expected in cases:
                with self.subTest(returncode=code, expected=expected):
                    class ErrorSSH(FakeSSH):
                        returncode = code

                        def communicate(self, payload=None, timeout=None):
                            return "", stderr

                    with patch.object(control.subprocess, "Popen", ErrorSSH):
                        result = control.run_remote("nvidia@192.168.1.152", script, "offline")
                    self.assertFalse(result["ok"])
                    self.assertEqual(result["reason"], expected)
                    self.assertNotIn("stderr", result)

    def test_address_validation(self):
        self.assertEqual(control.validate_host(" user@192.168.3.144 "), "user@192.168.3.144")
        self.assertEqual(control.validate_host("nvidia@ubuntu.local"), "nvidia@ubuntu.local")
        for value in ("-oProxyCommand=evil", "user@127.0.0.1;echo", "user@999.1.1.1", "a@b@c", "u@$(echo)"):
            with self.assertRaises(ValueError):
                control.validate_host(value)

    def test_ssh_password_only_in_askpass_environment_and_sudo_stdin(self):
        password = "offline-password-test"
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {"XDG_STATE_HOME": directory}):
            script = Path(directory) / "script.sh"
            script.write_text("printf 'finished\\n'\n")
            with patch.object(control.subprocess, "Popen", FakeSSH):
                result = control.run_remote("user@192.168.3.144", script, password, sudo=True)
            call = FakeSSH.calls[-1]
            self.assertTrue(result["ok"])
            self.assertNotIn(password, " ".join(call.args))
            self.assertEqual(call.kwargs["env"]["NOMAD_SSH_PASSWORD"], password)
            self.assertTrue(call.payload.startswith(password + "\n"))
            self.assertIn("sudo -k -S", call.args[-1])
            self.assertFalse(Path(call.kwargs["env"]["SSH_ASKPASS"]).exists())

    def test_sudo_wrapper_separates_password_even_when_sudo_does_not_read_it(self):
        # Emulate NOPASSWD sudo: it runs the command without consuming stdin.
        # A shell-shaped password must never be executed or become script input.
        with tempfile.TemporaryDirectory() as directory:
            fake_sudo = Path(directory) / "sudo"
            fake_sudo.write_text("#!/bin/bash\nwhile [[ $1 == -* ]]; do if [[ $1 == -p ]]; then shift 2; else shift; fi; done\nexec \"$@\"\n")
            fake_sudo.chmod(0o700)
            marker = Path(directory) / "password-was-executed"
            password = f"touch {marker}"
            payload = password + "\nprintf 'ACTION_OK\\n'\nif read -r value; then exit 31; fi\n"
            environment = os.environ.copy()
            environment["PATH"] = directory + ":" + environment["PATH"]
            run = subprocess.run(["bash", "-c", control.SUDO_WRAPPER], input=payload,
                                 text=True, capture_output=True, env=environment, timeout=3)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertEqual(run.stdout.strip(), "ACTION_OK")
            self.assertFalse(marker.exists())

    def test_partial_failure_and_concurrent_board_calls(self):
        def fake_remote(host, *_args, **_kwargs):
            time.sleep(0.15)
            return {"ok": host.startswith("user@"), "state": "STOPPED" if host.startswith("user@") else "FAILED", "reason": "ok" if host.startswith("user@") else "ssh_unreachable"}
        started = time.monotonic()
        with patch.object(control, "run_remote", fake_remote):
            result = control.stop_boards("user@192.168.3.144", "nvidia@192.168.1.152", "offline")
        self.assertEqual(result["state"], "PARTIAL_FAILURE")
        self.assertEqual(result["boards"]["jetson"]["reason"], "ssh_unreachable")
        self.assertLess(time.monotonic() - started, 0.27)

    def test_unreachable_board_is_looked_for_and_retried_there(self):
        calls = []
        def once(host, script, password, timeout_s=18, **kw):
            calls.append((host, kw.get("via"), kw.get("host_key_alias")))
            if host == "user@192.168.3.164":
                return dict(ok=False, reason="ssh_unreachable", state="FAILED", stdout="")
            return dict(ok=True, reason="ok", state="STOPPED", stdout="", returncode=0)
        found = dict(host="user@10.77.0.2", via="nvidia@192.168.1.152", alias="192.168.3.164", method="wired")
        import board_address
        with patch.object(control, "_run_remote_once", side_effect=once), \
                patch.object(board_address, "recover", return_value=found) as recover, \
                patch.object(board_address, "learn"), \
                patch.object(board_address, "route", side_effect=lambda h: dict(host=h, via=None, alias=None, method=None)):
            result = control.stop_boards("user@192.168.3.164", "nvidia@192.168.1.152", "pw")
        self.assertTrue(result["ok"])
        self.assertEqual(result["boards"]["pi5"]["route"], "wired")
        self.assertIn(("user@10.77.0.2", "nvidia@192.168.1.152", "192.168.3.164"), calls)
        recover.assert_called_once_with("user@192.168.3.164")

    def test_slow_script_on_reachable_board_is_not_rerun_elsewhere(self):
        import board_address
        with patch.object(control, "_run_remote_once", return_value=dict(ok=False, reason="timeout", stdout="")), \
                patch.object(board_address, "_ssh_open", return_value=True), \
                patch.object(board_address, "recover") as recover, \
                patch.object(board_address, "route", side_effect=lambda h: dict(host=h, via=None, alias=None, method=None)):
            result = control.run_remote("nvidia@192.168.1.152", control.BASE_DIR / "remote/stop_trial.sh", "pw")
        self.assertEqual(result["reason"], "timeout")
        recover.assert_not_called()

    def test_cleanup_failure_is_not_treated_as_a_moved_board(self):
        import board_address
        with patch.object(control, "_run_remote_once",
                          return_value=dict(ok=False, reason="remote_cleanup_incomplete", stdout="")), \
                patch.object(board_address, "recover") as recover, \
                patch.object(board_address, "route", side_effect=lambda h: dict(host=h, via=None, alias=None, method=None)):
            control.run_remote("user@192.168.3.164", control.BASE_DIR / "remote/stop_trial.sh", "pw")
        recover.assert_not_called()

    def test_retry_only_unconfirmed_board(self):
        seen = []
        def fake(host, script, password, timeout_s=18, **kw):
            seen.append(host)
            return dict(ok=True, reason="ok", state="STOPPED", stdout="")
        with patch.object(control, "run_remote", side_effect=fake):
            result = control.stop_boards("user@192.168.3.163", "nvidia@192.168.1.152", "pw", only={"pi5"})
        self.assertEqual(seen, ["user@192.168.3.163"])
        self.assertTrue(result["ok"])
        self.assertEqual(set(result["boards"]), {"pi5"})

    def test_hop_and_host_key_alias_reach_ssh_arguments(self):
        captured = {}
        class FakeSSH:
            def __init__(self, args, **kw):
                captured["args"] = args
                self.returncode = 0
                self.pid = 0
            def communicate(self, payload=None, timeout=None):
                return ("NOMAD_STOP_RESULT={\"state\":\"STOPPED\"}\n", "")
        with tempfile.TemporaryDirectory() as state, patch.dict(os.environ, {"XDG_STATE_HOME": state}), \
                patch.object(control.subprocess, "Popen", FakeSSH):
            result = control.run_remote(control.PI5_WIRED_HOST, control.BASE_DIR / "remote/stop_trial.sh", None,
                                        via="nvidia@192.168.1.152", host_key_alias="192.168.3.164")
        self.assertTrue(result["ok"])
        args = captured["args"]
        proxy = [a for a in args if a.startswith("ProxyCommand=")]
        self.assertEqual(len(proxy), 1)
        self.assertIn("-W %h:%p nvidia@192.168.1.152", proxy[0])
        self.assertIn("StrictHostKeyChecking=accept-new", proxy[0])
        self.assertIn("HostKeyAlias=192.168.3.164", args)
        self.assertEqual(args[-2], control.PI5_WIRED_HOST)
        for bad in ("nvidia@x;reboot", "nvidia@1.2.3"):
            self.assertFalse(control.run_remote(control.PI5_WIRED_HOST, control.BASE_DIR / "remote/stop_trial.sh",
                                                None, via=bad)["ok"])

    def test_wrong_accounts_never_run(self):
        with patch.object(control, "run_remote") as remote:
            result = control.stop_boards("root@192.168.3.144", "root@192.168.1.152", "offline")
        remote.assert_not_called()
        self.assertEqual(result["state"], "FAILED")

    def test_stop_requires_the_remote_stopped_marker(self):
        with patch.object(control, "run_remote", return_value={"ok": True, "state": "SUCCEEDED"}):
            result = control.stop_boards("user@192.168.3.144", "nvidia@192.168.1.152", "offline")
        self.assertFalse(result["ok"])
        self.assertEqual(result["boards"]["pi5"]["reason"], "invalid_remote_stop_result")

    def test_empty_array_and_already_ended_retry(self):
        functions = (ROOT / "remote/stop_trial.sh").read_text().split("# MAIN_ENTRY:")[0]
        # This is the post-main cleanup algorithm with no owned live targets.
        # A stale PID from a previous boot must not turn idempotent stop into
        # a shell nounset failure or a signal to another process.
        code = functions + '''\nws=/nonexistent/nomad_ws\ncandidate 2147483647 oak\nfound=${#starts[@]}\nsignal_stage INT\nsignal_stage TERM\nsignal_stage KILL\n[[ "$found" == 0 && "$rejected" == 0 ]] || exit 20\nremaining=0\nresult STOPPED already_stopped\n'''
        run = subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=3)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertIn('"state":"STOPPED"', run.stdout)
        self.assertIn('"targets":0', run.stdout)

    def test_target_process_group_isolation_and_stale_identity(self):
        functions = (ROOT / "remote/stop_trial.sh").read_text().split("# MAIN_ENTRY:")[0]
        with tempfile.TemporaryDirectory(prefix="nomad-stop-isolation-") as directory:
            workspace = Path(directory) / "nomad_ws"
            workspace.mkdir()
            program = workspace / "run_oak_cpp.sh"
            program.write_text("#!/bin/bash\ntrap 'exit 0' INT TERM\nwhile :; do sleep 0.1; done\n")
            target = subprocess.Popen(["bash", str(program)], start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            unrelated = subprocess.Popen(["sleep", "30"], start_new_session=True)
            try:
                time.sleep(0.08)
                code = functions + f'''\nws={workspace}\ncandidate {target.pid} oak 1 {target.pid} "$boot_id"\n[[ "$rejected" == 1 && "${{#starts[@]}}" == 0 ]] || exit 10\ncandidate {unrelated.pid} oak\n[[ "$rejected" == 2 && "${{#starts[@]}}" == 0 ]] || exit 11\ncandidate {target.pid} oak\n[[ -n ${{starts[{target.pid}]+yes}} ]] || exit 12\nsignal_stage TERM\nsleep 0.3\nsignal_stage KILL\nprintf 'ISOLATION_OK\\n'\n'''
                run = subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=6)
                self.assertEqual(run.returncode, 0, run.stderr + run.stdout)
                self.assertIn("ISOLATION_OK", run.stdout)
                target.wait(timeout=2)
                self.assertIsNone(unrelated.poll())
            finally:
                for process in (target, unrelated):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGKILL)
                    process.wait()

    def test_thousand_unrelated_process_rows_are_filtered_before_identity_reads(self):
        functions = (ROOT / "remote/stop_trial.sh").read_text().split("# MAIN_ENTRY:")[0]
        with tempfile.TemporaryDirectory(prefix="nomad-stop-load-") as directory:
            workspace = Path(directory) / "nomad_ws"
            workspace.mkdir()
            program = workspace / "run_oak_cpp.sh"
            program.write_text("#!/bin/bash\ntrap 'exit 0' TERM\nwhile :; do sleep 0.1; done\n")
            target = subprocess.Popen(["bash", str(program)], start_new_session=True,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                time.sleep(0.05)
                code = functions + f'''\nws={workspace}\nidentity_reads=0\neval "$(declare -f identity | sed '1s/identity/identity_real/')"\nidentity() {{ identity_reads=$((identity_reads+1)); identity_real "$@"; }}\nps() {{ for ((i=100000;i<101000;i++)); do printf '%s 99999999 %s\\n' "$i" "$current_uid"; done; command ps "$@"; }}\ncandidate {target.pid} oak\n[[ "$identity_reads" -lt 20 && -n ${{starts[{target.pid}]+yes}} ]] || exit 40\nprintf 'FILTERED_OK identities=%s\\n' "$identity_reads"\n'''
                began = time.monotonic()
                run = subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=3)
                self.assertEqual(run.returncode, 0, run.stderr)
                self.assertIn("FILTERED_OK", run.stdout)
                self.assertLess(time.monotonic() - began, 1.0)
            finally:
                if target.poll() is None:
                    os.killpg(target.pid, signal.SIGKILL)
                target.wait()

    def test_local_non_board_is_refused(self):
        result = subprocess.run(["bash", str(ROOT / "remote/stop_trial.sh")], text=True, capture_output=True, timeout=3)
        self.assertEqual(result.returncode, 2)
        self.assertIn("board_identity_mismatch", result.stdout)


if __name__ == "__main__":
    unittest.main()
