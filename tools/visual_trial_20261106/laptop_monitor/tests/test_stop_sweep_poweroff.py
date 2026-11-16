"""2026-11-16 items 2·9·1: stop sweep / recording guard / GNSS & sensor cleanup / board power off.

Offline only. Shell functions are loaded without their main entry, and `ps` is
replaced so the sweep can only ever see the fake processes this test starts
(never a real process of the laptop user). Power-off never runs on this laptop.
"""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import control  # noqa: E402
import sensor_record  # noqa: E402

STUBBORN = "#!/bin/bash\ntrap '' INT TERM\nwhile :; do sleep 0.1; done\n"   # needs KILL
POLITE = "#!/bin/bash\ntrap 'exit 0' INT TERM\nwhile :; do sleep 0.1; done\n"
NOT_RECORDING = Mock(return_value={"ok": True, "recording": False})   # Jetson status pre-check stub


def functions(name):
    return (ROOT / "remote" / name).read_text(encoding="utf-8").split("# MAIN_ENTRY:")[0]


def fake_ps(pids):
    """Shell `ps` override listing only the given pids (pid=,uid= and pid=,pgid=,uid= forms)."""
    listed = " ".join(str(pid) for pid in pids)
    return ('ps() { local p; for p in ' + listed + '; do [[ -r /proc/$p/stat ]] || continue; '
            'if [[ "$2" == *pgid* ]]; then printf "%s %s %s\\n" "$p" "$(command ps -o pgid= -p "$p" | tr -d " ")" "$(id -u)"; '
            'else printf "%s %s\\n" "$p" "$(id -u)"; fi; done; }\n')


class ProcessCase(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="nomad-sweep-")
        self.dir = Path(self.temp.name)
        self.processes = []

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            process.wait()
        self.temp.cleanup()

    def spawn(self, path, body, *args, env=None):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(body)
        process = subprocess.Popen(["bash", str(path), *args], start_new_session=True, env=env,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.processes.append(process)
        return process

    def wait_ready(self):
        time.sleep(0.15)


class StopTrialSweepTests(ProcessCase):
    def run_sweep(self, pids, setup, timeout=15):
        code = functions("stop_trial.sh") + fake_ps(pids) + setup
        return subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=timeout)

    def test_sweep_stops_leftover_zed_container_and_lidar_with_kill_escalation(self):
        ws = self.dir / "nomad_ws"
        zed = self.spawn(ws / "component_container_isolated", STUBBORN, "-r", "__node:=zed_container")
        lidar = self.spawn(ws / "ydlidar_ros2_driver_node", POLITE)
        decoy = self.spawn(ws / "other_tool.sh", POLITE)               # not a module name
        container = self.spawn(ws / "component_container_isolated", POLITE, "-r", "__node:=other")  # not ZED
        self.wait_ready()
        pids = [zed.pid, lidar.pid, decoy.pid, container.pid]
        run = self.run_sweep(pids, f"ws={ws}\nboard=jetson\nrecording_marker={self.dir}/none\n"
                                   "sweep_and_report no_pointer\n")
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertIn('"state":"STOPPED"', run.stdout)
        self.assertIn("orphan_modules_stopped_board_power_on", run.stdout)
        self.assertIn('"swept":2', run.stdout)
        self.assertIn('"survivors":""', run.stdout)
        zed.wait(timeout=2)
        lidar.wait(timeout=2)
        self.assertIsNone(decoy.poll())
        self.assertIsNone(container.poll())

    def test_no_processes_keeps_the_fresh_boot_result(self):
        run = self.run_sweep([], f"ws={self.dir}/nomad_ws\nboard=pi5\nsweep_and_report no_pointer\n")
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertIn("no_trial_processes_board_power_on", run.stdout)

    def test_recording_keeps_jetson_sensors_but_stops_other_modules(self):
        ws = self.dir / "nomad_ws"
        marker = self.dir / "RECORDING"
        marker.write_text("/mnt/ssd/bags/x\n")
        lidar = self.spawn(ws / "ydlidar_ros2_driver_node", POLITE)
        zed = self.spawn(ws / "component_container_isolated", POLITE, "__node:=zed_container")
        mapper = self.spawn(ws / "nomad_mapper2", POLITE)
        self.wait_ready()
        run = self.run_sweep([lidar.pid, zed.pid, mapper.pid],
                             f"ws={ws}\nboard=jetson\nrecording_marker={marker}\nsweep_and_report no_pointer\n")
        self.assertEqual(run.returncode, 2, run.stderr)
        self.assertIn('"reason":"recording_active"', run.stdout)
        self.assertIn('"kept":2', run.stdout)
        self.assertIn('"recording":true', run.stdout)
        mapper.wait(timeout=2)
        self.assertIsNone(lidar.poll())
        self.assertIsNone(zed.poll())

    def test_recording_marker_only_guards_the_jetson(self):
        ws = self.dir / "nomad_ws"
        marker = self.dir / "RECORDING"
        marker.write_text("x\n")
        lidar = self.spawn(ws / "ydlidar_ros2_driver_node", POLITE)
        self.wait_ready()
        run = self.run_sweep([lidar.pid], f"ws={ws}\nboard=pi5\nrecording_marker={marker}\nsweep_and_report no_pointer\n")
        self.assertEqual(run.returncode, 0, run.stderr)
        lidar.wait(timeout=2)

    def test_pid_file_stage_keeps_recording_sensor_groups(self):
        ws = self.dir / "nomad_ws"
        marker = self.dir / "RECORDING"
        marker.write_text("x\n")
        oak = self.spawn(ws / "run_oak_cpp.sh", POLITE)
        self.wait_ready()
        run = self.run_sweep([oak.pid], f"ws={ws}\nboard=jetson\nrecording_marker={marker}\n"
                                        f"candidate {oak.pid} oak\n"
                                        "[[ ${#starts[@]} == 0 && $kept == 1 ]] || exit 30\necho KEPT_OK\n")
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertIn("KEPT_OK", run.stdout)
        self.assertIsNone(oak.poll())

    def test_survivors_are_reported_with_label_and_pid(self):
        ws = self.dir / "nomad_ws"
        lidar = self.spawn(ws / "ydlidar_ros2_driver_node", POLITE)
        self.wait_ready()
        # Simulate a process that ignores every signal: signals become no-ops.
        run = self.run_sweep([lidar.pid], f"ws={ws}\nboard=jetson\nrecording_marker={self.dir}/none\n"
                                          "kill() { :; }\nwait_gone() { :; }\nsweep_and_report pointer\n")
        self.assertEqual(run.returncode, 2, run.stderr)
        self.assertIn("owned_cleanup_incomplete", run.stdout)
        self.assertIn(f'"survivors":"lidar:{lidar.pid}"', run.stdout)
        self.assertIsNone(lidar.poll())

    def test_trial_process_exists_still_detects_module_tokens(self):
        ws = self.dir / "nomad_ws"
        receiver = self.spawn(ws / "install/nomad_gnss/lib/nomad_gnss/receiver", POLITE)
        self.wait_ready()
        run = self.run_sweep([receiver.pid], f"ws={ws}\ntrial_process_exists && echo FOUND\n"
                                             "ws=/elsewhere\ntrial_process_exists || echo NONE\n")
        self.assertIn("FOUND", run.stdout)
        self.assertIn("NONE", run.stdout)

    def test_timing_budget_and_identity_rechecks_stay(self):
        text = (ROOT / "remote/stop_trial.sh").read_text()
        self.assertIn("stop_captured 30", text)   # sweep INT 3 s, TERM 2 s, KILL 0.3 s
        self.assertIn("same_process \"$pid\" || continue", text)
        self.assertNotIn("kill -s \"$signal_name\" -- \"-", text)   # never a whole PGID
        self.assertNotIn("poweroff", text.split("\n", 2)[2])


class RecordGnssStopTests(ProcessCase):
    def run_stop(self, pids, extra=""):
        ws = self.dir / "nomad_ws"
        code = (functions("record_gnss_stop_pi.sh") + fake_ps(pids) +
                f"ws={ws}\nmarker={self.dir}/marker\nid() {{ [[ $1 == -un ]] && echo user || command id \"$@\"; }}\n"
                + extra + "main\n")
        return subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=15)

    def receiver(self, run_name, body=STUBBORN):
        ws = self.dir / "nomad_ws"
        env = dict(os.environ, ROS_LOG_DIR=f"{ws}/log/{run_name}/ros")
        return self.spawn(ws / "install/nomad_gnss/lib/nomad_gnss/receiver", body, env=env)

    def result(self, run):
        line = [l for l in run.stdout.splitlines() if l.startswith("NOMAD_RECORD_RESULT=")][-1]
        return json.loads(line.split("=", 1)[1])

    def test_kill_escalation_then_marker_removed_and_trial_receiver_kept(self):
        (self.dir / "marker").write_text("x\n")
        recorded = self.receiver("record_gnss_20261116_1")
        trial = self.receiver("visual_trial_20261116_1", POLITE)
        self.wait_ready()
        value = self.result(self.run_stop([recorded.pid, trial.pid]))
        self.assertEqual(value["reason"], "stopped")
        self.assertTrue(value["ok"])
        recorded.wait(timeout=2)
        self.assertIsNone(trial.poll())
        self.assertFalse((self.dir / "marker").exists())

    def test_live_record_receiver_without_marker_is_cleaned(self):
        recorded = self.receiver("record_gnss_20261116_2", POLITE)
        self.wait_ready()
        value = self.result(self.run_stop([recorded.pid]))
        self.assertEqual(value["reason"], "stopped")
        self.assertFalse(value["marker"])
        recorded.wait(timeout=2)

    def test_marker_kept_while_receiver_survives(self):
        (self.dir / "marker").write_text("x\n")
        recorded = self.receiver("record_gnss_20261116_3", POLITE)
        self.wait_ready()
        value = self.result(self.run_stop([recorded.pid], "kill() { :; }\nwait_gone() { :; }\n"))
        self.assertFalse(value["ok"])
        self.assertEqual(value["reason"], "stop_failed")
        self.assertIn(f"gnss:{recorded.pid}", value["survivors"])
        self.assertTrue((self.dir / "marker").exists())

    def test_nothing_running_without_marker(self):
        value = self.result(self.run_stop([]))
        self.assertEqual(value["reason"], "not_started_by_gui")


class JetsonSensorStopTests(ProcessCase):
    def test_run_sensors_stop_including_orphan_container_and_group_members(self):
        run = self.dir / "sensor_runs/20261116_1"
        (run / "ros").mkdir(parents=True)
        env = dict(os.environ, ROS_LOG_DIR=f"{run}/ros")
        zed = self.spawn(self.dir / "bin/component_container_isolated", STUBBORN, "zed_container", env=env)
        # Leader (in the run's pid file, with the run env) and a group child without the env.
        leader = self.spawn(self.dir / "bin/lidar_leader.sh",
                            "#!/bin/bash\nenv -u ROS_LOG_DIR bash -c \"trap 'exit 0' INT TERM; while :; do sleep 0.1; done\" &\n"
                            "trap 'kill %1 2>/dev/null; exit 0' INT TERM\nwhile :; do sleep 0.1; done\n", env=env)
        (run / "lidar.pid").write_text(f"{leader.pid}\n")
        trial = self.spawn(self.dir / "bin/ydlidar_ros2_driver_node", POLITE,
                           env=dict(os.environ, ROS_LOG_DIR=f"{self.dir}/visual_trial/ros"))
        self.wait_ready()
        child = int(subprocess.run(["pgrep", "-g", str(leader.pid), "-f", "^bash -c trap"],
                                   capture_output=True, text=True).stdout.split()[0])
        code = (functions("record_stop_jetson.sh") + fake_ps([zed.pid, leader.pid, child, trial.pid]) +
                f'left=$(stop_run_sensors "{run}")\nprintf "LEFT=%s\\n" "$left"\n')
        result = subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=20)
        self.assertIn("LEFT=\n", result.stdout + "\n", result.stdout + result.stderr)
        zed.wait(timeout=2)
        leader.wait(timeout=2)
        for _ in range(30):
            if not Path(f"/proc/{child}").exists():
                break
            time.sleep(0.1)
        self.assertFalse(Path(f"/proc/{child}/cmdline").exists() and
                         Path(f"/proc/{child}/cmdline").read_bytes().startswith(b"bash\0-c\0trap"))
        self.assertIsNone(trial.poll())

    def test_reused_pid_file_number_without_run_env_is_not_stopped(self):
        run = self.dir / "sensor_runs/20261116_3"
        (run / "ros").mkdir(parents=True)
        other = self.spawn(self.dir / "bin/gui_trial_module.sh", POLITE)   # same number, other group owner
        (run / "lidar.pid").write_text(f"{other.pid}\n")
        self.wait_ready()
        code = (functions("record_stop_jetson.sh") + fake_ps([other.pid]) +
                f'left=$(stop_run_sensors "{run}")\nprintf "LEFT=%s\\n" "$left"\n')
        result = subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=10)
        self.assertIn("LEFT=\n", result.stdout + "\n", result.stdout + result.stderr)
        time.sleep(0.3)
        self.assertIsNone(other.poll())

    def test_survivors_listed(self):
        run = self.dir / "sensor_runs/20261116_2"
        (run / "ros").mkdir(parents=True)
        zed = self.spawn(self.dir / "bin/zed_like.sh", POLITE, env=dict(os.environ, ROS_LOG_DIR=f"{run}/ros"))
        self.wait_ready()
        code = (functions("record_stop_jetson.sh") + fake_ps([zed.pid]) + "kill() { :; }\nwait_gone() { :; }\n" +
                f'left=$(stop_run_sensors "{run}")\nprintf "LEFT=%s\\n" "$left"\n')
        result = subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=10)
        self.assertRegex(result.stdout, rf"LEFT=\S+:{zed.pid}")

    def test_script_contract(self):
        text = (ROOT / "remote/record_stop_jetson.sh").read_text()
        self.assertIn('"not_recording","sensors"', text)        # sensors stopped without RECORDING
        self.assertIn("sensors=recording_active", text)
        self.assertIn("sensors_remaining", text)
        self.assertLess(text.index('rm -f "$SSD/bags/GUI_SENSORS"'), text.index("invalid_marker"))
        start = (ROOT / "remote/record_start_jetson.sh").read_text()
        self.assertIn('result false sensors_failed "" true', start)
        for name in ("record_stop_jetson.sh", "record_start_jetson.sh", "record_gnss_stop_pi.sh",
                     "stop_trial.sh", "poweroff_board.sh"):
            subprocess.run(["bash", "-n", str(ROOT / "remote" / name)], check=True)


class StopResultDisplayTests(unittest.TestCase):
    def test_stop_boards_carries_survivors_and_recording_guard(self):
        def fake(host, script, password, timeout_s=18, **kw):
            if host.startswith("nvidia"):
                return dict(ok=False, reason="remote_cleanup_incomplete", state="FAILED",
                            remote=dict(state="FAILED", reason="recording_active", survivors=""))
            return dict(ok=False, reason="remote_cleanup_incomplete", state="FAILED",
                        remote=dict(state="FAILED", reason="owned_cleanup_incomplete", survivors="gnss:42"))
        with patch.object(control, "run_remote", side_effect=fake):
            value = control.stop_boards("user@192.168.3.10", "nvidia@192.168.3.11", "pw")
        self.assertEqual(value["boards"]["jetson"]["reason"], "recording_active")
        self.assertEqual(value["boards"]["pi5"]["survivors"], "gnss:42")

    def test_record_stop_description_lists_survivors(self):
        text = sensor_record.describe(dict(ok=True, reason="sensors_remaining", duration_s=3, bytes=1e9, gps_fix=2,
                                           session="/mnt/ssd/bags/a", survivors="component_cont:3607",
                                           gnss=dict(ok=False, reason="stop_failed", survivors="gnss:9")))
        self.assertIn("남은 센서 component_cont:3607", text)
        self.assertIn("Pi5 GPS 남음 gnss:9", text)
        self.assertIn("GPS 위치 2개", text)
        text = sensor_record.describe(dict(ok=False, reason="not_recording", sensors="stopped", survivors=""))
        self.assertIn("녹화가 켠 센서를 껐습니다", text)

    def test_monitor_shows_survivors(self):
        source = (ROOT / "monitor.py").read_text()
        self.assertIn('" · 남은 모듈 " + str(result["survivors"])', source)
        self.assertIn("센서 녹화 중이라 센서 유지", source)

    def test_drive_record_dialog_keeps_stop_enabled_for_leftover_sensors(self):
        from PyQt5 import QtWidgets
        import drive_record_dialog
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch.object(drive_record_dialog.DriveRecordDialog, "refresh_recording"):
            dialog = drive_record_dialog.DriveRecordDialog(None, "nvidia@192.168.3.11", None, None)
        try:
            dialog.timer.stop(); dialog.record_timer.stop()
            self.assertFalse(dialog.record_stop.isEnabled())
            dialog._record_stopped(dict(ok=False, reason="not_recording", sensors="stop_incomplete",
                                        survivors="zed:1"))
            self.assertTrue(dialog.record_stop.isEnabled())
            self.assertIn("zed:1", dialog.record_status.text())
            dialog._record_stopped(dict(ok=False, reason="not_recording", sensors="stopped", survivors=""))
            self.assertFalse(dialog.record_stop.isEnabled())
        finally:
            dialog.deleteLater()
            app.processEvents()


class PowerOffControlTests(unittest.TestCase):
    def stopped(self, *a, **kw):
        return {"ok": True, "state": "STOPPED", "boards": {"pi5": {"ok": True}, "jetson": {"ok": True}}}

    def test_order_sudo_no_route_retry_and_expected_header(self):
        calls = []

        def fake(host, script, password, timeout_s=18, **kw):
            calls.append((host, Path(script).read_text().splitlines()[0], kw))
            return dict(ok=True, reason="ok", stdout='NOMAD_POWER_RESULT={"ok":true,"reason":"scheduled"}\n')
        stop = Mock(side_effect=self.stopped)
        with patch.object(control, "run_remote", side_effect=fake), \
                patch.object(control, "_is_local_address", return_value=False):
            value = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "pw", pi4_host="user@10.42.0.14",
                                            boards=("jetson", "pi5", "pi4"), stop=stop,
                                            recording_check=NOT_RECORDING)
        self.assertTrue(value["ok"])
        self.assertEqual(value["state"], "SCHEDULED")
        self.assertEqual([c[0] for c in calls], ["user@10.42.0.14", "user@10.42.0.15", "nvidia@10.42.0.16"])
        self.assertEqual([c[1] for c in calls], ["EXPECTED=pi4", "EXPECTED=pi5", "EXPECTED=jetson"])
        for _host, _line, kw in calls:
            self.assertTrue(kw["sudo"])
            self.assertFalse(kw["follow"])
        stop.assert_called_once()
        self.assertEqual(stop.call_args.kwargs["only"], {"pi5", "jetson"})

    def test_recording_refuses_power_off(self):
        stop = Mock(return_value={"ok": False, "boards": {"jetson": {"ok": False, "reason": "recording_active"}}})
        with patch.object(control, "run_remote") as remote, patch.object(control, "_is_local_address", return_value=False):
            value = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "pw", stop=stop,
                                            boards=("pi5", "jetson"), recording_check=NOT_RECORDING)
        remote.assert_not_called()
        self.assertEqual(value["reason"], "recording_active")

    def test_local_wrong_account_and_missing_password_are_refused(self):
        stop = Mock(side_effect=self.stopped)
        with patch.object(control, "run_remote") as remote:
            local = control.poweroff_boards("user@127.0.0.1", "nvidia@10.42.0.16", "pw", stop=stop, boards=("pi5",))
            self.assertEqual(local["boards"]["pi5"]["reason"], "local_address_refused")
            self.assertEqual(control.poweroff_boards("user@localhost", "nvidia@x", "pw", stop=stop,
                                                     boards=("pi5",))["boards"]["pi5"]["reason"], "local_address_refused")
            wrong = control.poweroff_boards("user@10.42.0.15", "user@10.42.0.16", "pw", stop=stop, boards=("jetson",))
            self.assertEqual(wrong["boards"]["jetson"]["reason"], "invalid_board_address")
            with patch.object(control, "_is_local_address", return_value=False):
                nopass = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "", stop=stop)
            self.assertEqual(nopass["reason"], "sudo_password_required")
        remote.assert_not_called()
        stop.assert_not_called()

    def test_is_local_address_detects_loopback_only(self):
        self.assertTrue(control._is_local_address("127.0.0.1"))
        self.assertTrue(control._is_local_address("localhost"))
        self.assertFalse(control._is_local_address("192.0.2.77"))   # TEST-NET, never local

    def test_sudo_failure_and_wrong_board_results(self):
        def fake(host, script, password, timeout_s=18, **kw):
            if host.startswith("user"):
                return dict(ok=False, reason="remote_script_failed", stdout="")
            return dict(ok=True, reason="ok", stdout='NOMAD_POWER_RESULT={"ok":false,"reason":"wrong_board"}\n')
        with patch.object(control, "run_remote", side_effect=fake), \
                patch.object(control, "_is_local_address", return_value=False):
            value = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "pw", stop=self.stopped,
                                            recording_check=NOT_RECORDING)
        self.assertEqual(value["boards"]["pi5"]["reason"], "sudo_failed")
        self.assertEqual(value["boards"]["jetson"]["reason"], "wrong_board")
        self.assertEqual(value["state"], "FAILED")

    def test_laptop_only_runs_ssh_and_command_travels_in_stdin(self):
        seen = []
        secret = "NOMAD-TEST-SECRET-91"

        class FakeSSH:
            def __init__(self, args, **kw):
                seen.append({"args": list(args)})
                self.returncode, self.pid = 0, 0

            def communicate(self, payload=None, timeout=None):
                seen[-1]["payload"] = payload
                return ('NOMAD_POWER_RESULT={"ok":true,"reason":"scheduled"}\n', "")
        forbidden = Mock(side_effect=AssertionError("local command"))
        with tempfile.TemporaryDirectory() as state, patch.dict(os.environ, {"XDG_STATE_HOME": state}), \
                patch.object(control.subprocess, "Popen", FakeSSH), \
                patch.object(control.subprocess, "run", forbidden), \
                patch.object(control.subprocess, "call", forbidden), \
                patch.object(control.os, "system", forbidden), \
                patch.object(control, "_is_local_address", return_value=False):
            value = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", secret, stop=self.stopped,
                                            recording_check=NOT_RECORDING)
        self.assertTrue(value["ok"])
        self.assertEqual(len(seen), 2)
        for item in seen:
            self.assertEqual(item["args"][0], "ssh")
            # Path option values (-o UserKnownHostsFile=/tmp/tmpXXXX, ControlPath) are random: skip them.
            args = [arg for index, arg in enumerate(item["args"]) if index == 0 or item["args"][index - 1] != "-o"]
            joined = " ".join(args)
            self.assertNotIn(secret, " ".join(item["args"]))
            self.assertIsNone(re.search(r"\b(poweroff|shutdown|halt)\b", joined), joined)
            payload = item["payload"]
            self.assertTrue(payload.startswith(secret + "\nEXPECTED="))
            self.assertLess(payload.index("wrong_board"), payload.index("systemctl poweroff"))

    def test_gui_python_never_contains_a_local_power_command(self):
        pattern = re.compile(r"systemctl\s+(poweroff|halt|reboot)|shutdown\s+-[hHrP]|\binit\s+0\b|\bhalt\b")
        for path in ROOT.glob("*.py"):
            self.assertIsNone(pattern.search(path.read_text(encoding="utf-8")), path.name)


class PowerOffScriptTests(unittest.TestCase):
    """remote/poweroff_board.sh. Every power command is a stub in PATH; nothing can power off this laptop."""

    def stubs(self, folder):
        log = Path(folder) / "calls.log"
        for name in ("systemd-run", "systemctl", "shutdown", "poweroff", "halt", "sync", "findmnt"):
            stub = Path(folder) / name
            stub.write_text(f'#!/bin/sh\necho "{name} $*" >> "{log}"\n')
            stub.chmod(0o755)
        return log

    def run_script(self, text, folder):
        env = dict(os.environ, PATH=f"{folder}:{os.environ.get('PATH', '')}")
        return subprocess.run(["bash", "-s"], input=text, text=True, capture_output=True, timeout=10, env=env)

    def test_this_laptop_is_refused_before_anything_runs(self):
        script = (ROOT / "remote/poweroff_board.sh").read_text()
        with tempfile.TemporaryDirectory() as folder:
            log = self.stubs(folder)
            for board in ("pi4", "pi5", "jetson", "laptop"):
                run = self.run_script(f"EXPECTED={board}\n" + script, folder)
                self.assertIn("NOMAD_POWER_RESULT=", run.stdout)
                self.assertIn('"ok":false', run.stdout)
                self.assertRegex(run.stdout, r'"reason":"(wrong_board|invalid_expected_board|not_root)"')
            self.assertFalse(log.exists())

    def test_matching_board_schedules_delayed_poweroff_after_sync(self):
        script = (ROOT / "remote/poweroff_board.sh").read_text()
        with tempfile.TemporaryDirectory() as folder:
            log = self.stubs(folder)
            model = Path(folder) / "model"
            model.write_bytes(b"Raspberry Pi 5 Model B Rev 1.0\0")
            fake = (script.replace("/proc/device-tree/model", str(model))
                    .replace('[[ "$(id -u)" == 0 ]]', "true")
                    .replace("shutdown -h +0", "true"))
            run = self.run_script("EXPECTED=pi5\n" + fake, folder)
            self.assertIn('NOMAD_POWER_RESULT={"ok":true,"reason":"scheduled","board":"pi5"}', run.stdout)
            calls = log.read_text().splitlines()
            self.assertEqual(calls[0], "sync ")
            self.assertIn("--on-active=3", calls[1])
            self.assertIn("/bin/systemctl poweroff", calls[1])
            wrong = self.run_script("EXPECTED=pi4\n" + fake, folder)
            self.assertIn('"reason":"wrong_board"', wrong.stdout)


class LauncherPowerOffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        import launcher
        self.launcher = launcher
        self.temp = tempfile.TemporaryDirectory(prefix="nomad-launcher-power-")
        self.state_patch = patch.object(launcher, "STATE_ROOT", Path(self.temp.name) / "state")
        self.state_patch.start()
        self.window = launcher.Launcher()
        self.window.begin_process = Mock()
        self.window.save_hosts = Mock()
        self.window.targets = Mock(return_value=("user@10.42.0.15", "nvidia@10.42.0.16"))
        self.window.network_pi4 = "user@10.42.0.14"

    def tearDown(self):
        self.window.viewer = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.state_patch.stop()
        self.temp.cleanup()

    def test_dialog_defaults_leave_teammate_pi4_unchecked(self):
        dialog, boxes = self.launcher.poweroff_dialog(None, {"pi4": "user@10.42.0.14", "pi5": "user@10.42.0.15",
                                                             "jetson": "nvidia@10.42.0.16"})
        self.assertFalse(boxes["pi4"].isChecked())
        self.assertTrue(boxes["pi5"].isChecked())
        self.assertTrue(boxes["jetson"].isChecked())
        self.assertIn("10.42.0.14", boxes["pi4"].text())
        dialog.deleteLater()

    def test_button_runs_control_poweroff_with_password_in_env_only(self):
        self.window.password.setText("secret")
        self.window.ask_poweroff_targets = Mock(return_value=["pi5", "jetson"])
        self.window.poweroff_boards()
        program, arguments, env = self.window.begin_process.call_args.args[1:4]
        self.assertEqual(self.window.begin_process.call_args.args[0], "poweroff")
        self.assertEqual(program, sys.executable)
        self.assertEqual(arguments[1], "poweroff")
        self.assertIn("pi5,jetson", arguments)
        self.assertNotIn("--pi4-host", arguments)
        self.assertNotIn("secret", " ".join(arguments))
        self.assertEqual(env["NOMAD_BOARD_PASSWORD1"], "secret")
        self.assertTrue(arguments[0].endswith("control.py"))

    def test_pi4_only_when_chosen_and_cancel_does_nothing(self):
        self.window.password.setText("secret")
        self.window.ask_poweroff_targets = Mock(return_value=None)
        self.window.poweroff_boards()
        self.window.begin_process.assert_not_called()
        self.window.ask_poweroff_targets = Mock(return_value=["pi4", "pi5"])
        self.window.poweroff_boards()
        arguments = self.window.begin_process.call_args.args[2]
        self.assertEqual(arguments[arguments.index("--pi4-host") + 1], "user@10.42.0.14")

    def test_refused_without_password_spectator_replay_or_running_viewer(self):
        self.window.ask_poweroff_targets = Mock(return_value=["pi5"])
        self.window.poweroff_boards()                         # empty password
        self.window.password.setText("secret")
        self.window.viewer = Mock(); self.window.viewer.poll.return_value = None
        self.window.update_buttons()
        self.assertFalse(self.window.poweroff_button.isEnabled())
        self.window.poweroff_boards()
        self.window.viewer = None
        self.window.viewer_kind = "replay"
        self.window.update_buttons()
        self.assertFalse(self.window.poweroff_button.isEnabled())
        self.window.poweroff_boards()
        self.window.viewer_kind = None
        self.window.connection_mode.setCurrentIndex(1)       # spectator
        self.window.update_buttons()
        self.assertFalse(self.window.poweroff_button.isEnabled())
        self.window.poweroff_boards()
        self.window.begin_process.assert_not_called()
        self.window.connection_mode.setCurrentIndex(0)
        self.window.update_buttons()
        self.assertTrue(self.window.poweroff_button.isEnabled())

    def test_summary_lines(self):
        out = json.dumps({"ok": True, "state": "SCHEDULED", "boards": {"pi5": {"ok": True, "reason": "scheduled"},
                                                                      "jetson": {"ok": True, "reason": "scheduled"}}})
        self.assertIn("Pi5 꺼짐 예약", self.launcher.poweroff_summary(out, True))
        self.assertIn("노트북은 켜져 있습니다", self.launcher.poweroff_summary(out, True))
        refused = json.dumps({"ok": False, "reason": "recording_active", "boards": {}})
        self.assertIn("녹화", self.launcher.poweroff_summary(refused, False))
        self.assertIn("확인하지 못했습니다", self.launcher.poweroff_summary("", False))

    def test_launcher_never_starts_a_local_power_command(self):
        with patch("subprocess.Popen", side_effect=AssertionError("local process")), \
                patch("subprocess.run", side_effect=AssertionError("local process")), \
                patch("os.system", side_effect=AssertionError("local process")):
            self.window.password.setText("secret")
            self.window.ask_poweroff_targets = Mock(return_value=["pi4", "pi5", "jetson"])
            self.window.poweroff_boards()
        self.assertEqual(self.window.begin_process.call_count, 1)


class SweepReviewTests(ProcessCase):
    """Review fixes: narrow sweep matcher, Pi5 recording receiver kept, recording wins over survivors."""

    def run_sweep(self, pids, setup, timeout=15):
        code = functions("stop_trial.sh") + fake_ps(pids) + setup
        return subprocess.run(["bash", "-s"], input=code, text=True, capture_output=True, timeout=timeout)

    def test_module_names_in_arguments_are_not_swept(self):
        ws = self.dir / "nomad_ws"
        tools = self.dir / "tools"
        colcon = self.spawn(tools / "colcon", POLITE, "build", "--packages-select", "nomad_mapper2")
        vim = self.spawn(tools / "vim", POLITE, f"{ws}/semantic.sh")
        tail = self.spawn(tools / "tail", POLITE, "-f", f"{ws}/scripts/run_gnss.sh")
        outside = self.spawn(self.dir / "elsewhere/ydlidar_ros2_driver_node", POLITE)   # not a NOMAD workspace
        mapper = self.spawn(ws / "install/nomad_mapper2", POLITE)
        self.wait_ready()
        pids = [colcon.pid, vim.pid, tail.pid, outside.pid, mapper.pid]
        run = self.run_sweep(pids, f"ws={ws}\nboard=jetson\nrecording_marker={self.dir}/none\n"
                                   "sweep_and_report no_pointer\n")
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertIn('"swept":1', run.stdout)
        mapper.wait(timeout=2)
        for process in (colcon, vim, tail, outside):
            self.assertIsNone(process.poll())

    def test_sweep_label_forms(self):
        ws = self.dir / "nomad_ws"
        launch = ws / "tools/visual_trial_20261106/launch/zed_camera.preview.launch.py"
        launch.parent.mkdir(parents=True)
        launch.write_text("")
        ros2 = self.spawn(self.dir / "opt/ros2", POLITE, "launch", str(launch), "camera_model:=zedm")
        pkg = self.spawn(self.dir / "opt/ros2", POLITE, "launch", "zed_wrapper", "zed_camera.launch.py")
        bashc = subprocess.Popen(["bash", "-c", "trap 'exit 0' INT TERM; while :; do sleep 0.1; done",
                                  "nomad", f"{ws}/semantic.sh"], start_new_session=True)
        self.processes.append(bashc)
        self.wait_ready()
        code = (f"ws={ws}\nboard=jetson\n"
                f"sweep_label {ros2.pid} && echo ROS2=$module\n"
                f"sweep_label {pkg.pid} || echo PKG=none\n"
                f"sweep_label {bashc.pid} || echo BASHC=none\n"
                f"module_label {bashc.pid} && echo BROAD=$module\n")
        run = self.run_sweep([], code)
        self.assertIn("ROS2=zed", run.stdout, run.stderr)
        self.assertIn("PKG=none", run.stdout)
        self.assertIn("BASHC=none", run.stdout)
        self.assertIn("BROAD=semantic", run.stdout)     # detection (trial_process_exists) stays broad

    def test_pi5_recording_gps_receiver_is_kept(self):
        ws = self.dir / "nomad_ws"
        recv = ws / "install/nomad_gnss/lib/nomad_gnss/receiver"
        recorded = self.spawn(recv, POLITE, env=dict(os.environ, ROS_LOG_DIR=f"{ws}/log/record_gnss_20261116_1/ros"))
        trial = self.spawn(ws / "scripts/run_fusion.sh", POLITE,
                           env=dict(os.environ, ROS_LOG_DIR=f"{ws}/log/visual_trial_20261116_1/ros"))
        self.wait_ready()
        run = self.run_sweep([recorded.pid, trial.pid], f"ws={ws}\nboard=pi5\nsweep_and_report no_pointer\n")
        self.assertEqual(run.returncode, 2, run.stdout + run.stderr)
        self.assertIn('"reason":"recording_active"', run.stdout)
        self.assertIn('"kept":1', run.stdout)
        self.assertIn('"recording":true', run.stdout)
        trial.wait(timeout=2)
        self.assertIsNone(recorded.poll())

    def test_recording_wins_over_survivors(self):
        ws = self.dir / "nomad_ws"
        marker = self.dir / "RECORDING"
        marker.write_text("x\n")
        lidar = self.spawn(ws / "ydlidar_ros2_driver_node", POLITE)
        mapper = self.spawn(ws / "nomad_mapper2", POLITE)
        self.wait_ready()
        run = self.run_sweep([lidar.pid, mapper.pid], f"ws={ws}\nboard=jetson\nrecording_marker={marker}\n"
                                                      "kill() { :; }\nwait_gone() { :; }\nsweep_and_report pointer\n")
        self.assertEqual(run.returncode, 2, run.stderr)
        self.assertIn('"reason":"recording_active"', run.stdout)
        self.assertIn(f"mapper2:{mapper.pid}", run.stdout)
        self.assertIsNone(lidar.poll())

    def test_leftover_child_of_swept_leader_is_reported(self):
        ws = self.dir / "nomad_ws"
        leader = self.spawn(ws / "scripts/run_zed_sensors.sh",
                            "#!/bin/bash\nbash -c \"trap '' INT TERM; while :; do sleep 0.1; done\" &\n"
                            "trap 'exit 0' INT TERM\nwhile :; do sleep 0.1; done\n")
        self.wait_ready()
        child = int(subprocess.run(["pgrep", "-g", str(leader.pid), "-f", "^bash -c trap"],
                                   capture_output=True, text=True).stdout.split()[0])
        run = self.run_sweep([leader.pid, child], f"ws={ws}\nboard=jetson\nrecording_marker={self.dir}/none\n"
                                                  "sweep_and_report no_pointer\n")
        self.assertEqual(run.returncode, 2, run.stdout + run.stderr)
        self.assertIn(f"child:{child}", run.stdout)
        os.kill(child, signal.SIGKILL)


class PowerOffRecordingReviewTests(unittest.TestCase):
    def test_stop_details_flags_recording(self):
        value = control._stop_details({"ok": False, "reason": "remote_cleanup_incomplete",
                                       "remote": {"reason": "owned_cleanup_incomplete", "recording": True,
                                                  "survivors": "zed:5"}})
        self.assertTrue(value["recording_active"])
        self.assertEqual(value["survivors"], "zed:5")

    def test_recording_flag_with_survivors_refuses(self):
        stop = Mock(return_value={"ok": False, "boards": {"jetson": {"ok": False, "reason": "remote_cleanup_incomplete",
                                                                     "recording_active": True}}})
        with patch.object(control, "run_remote") as remote, patch.object(control, "_is_local_address", return_value=False):
            value = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "pw", stop=stop,
                                            recording_check=NOT_RECORDING)
        remote.assert_not_called()
        self.assertEqual(value["reason"], "recording_active")

    def test_recording_checked_before_trial_stop_even_for_pi5_only(self):
        stop = Mock()
        check = Mock(return_value={"ok": True, "recording": True})
        with patch.object(control, "run_remote") as remote, patch.object(control, "_is_local_address", return_value=False):
            value = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "pw", boards=("pi5",),
                                            stop=stop, recording_check=check)
        self.assertEqual(value["reason"], "recording_active")
        check.assert_called_once_with("nvidia@10.42.0.16", "pw")
        stop.assert_not_called()
        remote.assert_not_called()

    def test_unreadable_recording_status_refuses(self):
        stop = Mock()
        with patch.object(control, "run_remote") as remote, patch.object(control, "_is_local_address", return_value=False):
            value = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "pw", stop=stop,
                                            recording_check=Mock(return_value={"ok": False, "reason": "timeout"}))
            no_ssd = control.poweroff_boards("user@10.42.0.15", "nvidia@10.42.0.16", "pw", stop=Mock(return_value={
                "ok": True, "boards": {}}), boards=("pi5",),
                recording_check=Mock(return_value={"ok": False, "reason": "no_ssd"}))
        self.assertEqual(value["reason"], "recording_check_failed")
        stop.assert_not_called()
        self.assertNotEqual(no_ssd.get("reason"), "recording_check_failed")
        self.assertEqual(remote.call_count, 1)   # only the Pi5 power-off script (mocked)

    def test_summary_for_failed_check(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        import launcher
        line = launcher.poweroff_summary(json.dumps({"ok": False, "reason": "recording_check_failed", "boards": {}}), False)
        self.assertIn("녹화 상태를 확인하지 못해", line)


if __name__ == "__main__":
    unittest.main()
