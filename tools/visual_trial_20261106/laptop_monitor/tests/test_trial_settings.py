"""2026-11-12: trial settings before Start, popup changes, startup timing and speed-ups."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PyQt5 import QtWidgets
import trial_settings as ts


class SettingsModelTests(unittest.TestCase):
    def test_validate_save_load_and_arguments(self):
        with tempfile.TemporaryDirectory() as temp:
            self.assertEqual(ts.load(temp), ts.defaults())
            saved = ts.save({"semantic_model": "indoor", "cloud_source": "device", "slope_mode": "parallel",
                             "control_mode": "vio_duty", "placement": "jetson", "gnss": "off", "junk": 1}, temp)
            self.assertEqual(ts.load(temp), saved)
            self.assertNotIn("junk", saved)
            self.assertEqual(oct(os.stat(ts.path(temp)).st_mode & 0o777), "0o600")
            self.assertEqual(ts.start_arguments(saved), ["--semantic-model", "indoor", "--cloud-source", "device",
                                                         "--oak-cloud-hz", "3", "--slope-mode", "parallel", "--no-gnss"])
            self.assertEqual(ts.defaults()["oak_cloud_hz"], "3")  # 2026-11-13: OAK cloud rate choice
            self.assertEqual(ts.validate({"oak_cloud_hz": "9"})["oak_cloud_hz"], "3")
            jetson = (Path(__file__).resolve().parents[1] / "remote/start_jetson.sh").read_text()
            # ROS double parameters reject integer text ("3"): the script must send "3.0".
            self.assertIn('device_depth_output_hz:="$oak_cloud_hz.0"', jetson)
            self.assertIn('output_hz:="$oak_cloud_hz.0"', jetson)
            self.assertEqual(json.loads(ts.monitor_arguments(saved)[1]), saved)
            self.assertEqual(ts.validate({"semantic_model": "evil;rm"})["semantic_model"], "b2")
            Path(ts.path(temp)).write_text("{broken")
            self.assertEqual(ts.load(temp), ts.defaults())

    def test_choices_match_existing_controls(self):
        import model_control, planning_client, slope_control
        self.assertEqual(set(ts.SETTINGS["semantic_model"][1]), set(model_control.MODEL_NAMES))
        self.assertEqual(set(ts.SETTINGS["slope_mode"][1]), set(slope_control.MODES))
        self.assertEqual(set(ts.SETTINGS["control_mode"][1]), set(planning_client.CONTROL_MODES))
        self.assertEqual(set(ts.SETTINGS["placement"][1]), set(planning_client.PLACEMENTS))


class LauncherSettingsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_saved_choice_is_restored_and_passed_to_start_and_monitor(self):
        import launcher
        with tempfile.TemporaryDirectory() as temp, patch.object(launcher, "STATE_ROOT", Path(temp)):
            ts.save({"semantic_model": "fcn", "cloud_source": "device", "placement": "jetson", "gnss": "off"}, temp)
            window = launcher.Launcher()
            try:
                self.assertEqual(window.setting_combos["semantic_model"].currentData(), "fcn")
                self.assertTrue(window.no_gnss.isChecked())
                window.begin_process = Mock()
                window.relay_port_available = Mock(return_value=True)
                window.hosts["pi"].setCurrentText("user@192.168.1.5")
                window.hosts["jetson"].setCurrentText("nvidia@192.168.1.6")
                window.password.setText("test-only")
                window.setting_combos["slope_mode"].setCurrentIndex(1)
                window.start_trial()
                args = window.begin_process.call_args.args[2]
                for item in ("--semantic-model", "fcn", "--cloud-source", "device", "--slope-mode", "parallel", "--no-gnss"):
                    self.assertIn(item, args)
                self.assertEqual(ts.load(temp)["slope_mode"], "parallel")      # remembered
                monitor_args = window.monitor_arguments(("user@192.168.1.5", "nvidia@192.168.1.6"), Path(temp))
                values = json.loads(monitor_args[monitor_args.index("--trial-settings") + 1])
                self.assertEqual((values["placement"], values["semantic_model"]), ("jetson", "fcn"))
                self.assertIn("FCN · 실내", window.settings_summary.text())
                # Dialog cancel restores the previous choice; save remembers it.
                with patch.object(window.settings_dialog, "exec_",
                                  side_effect=lambda: window.setting_combos["cloud_source"].setCurrentIndex(0) or 0):
                    window.edit_trial_settings()
                self.assertEqual(window.setting_combos["cloud_source"].currentData(), "device")
                with patch.object(window.settings_dialog, "exec_",
                                  side_effect=lambda: window.setting_combos["cloud_source"].setCurrentIndex(0) or 1):
                    window.edit_trial_settings()
                self.assertEqual(ts.load(temp)["cloud_source"], "opencv")
                self.assertIn("OpenCV", window.settings_summary.text())
            finally:
                window.close()
                window.deleteLater()


class MonitorPopupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_launcher_choice_sets_popup_defaults(self):
        import monitor
        values = ts.validate({"semantic_model": "indoor", "cloud_source": "device", "slope_mode": "parallel",
                              "control_mode": "vio_duty", "placement": "jetson"})
        argv = ["monitor", "--no-ssh", "--enable-trial-control", "--trial-settings", json.dumps(values)]
        with patch.object(sys, "argv", argv):
            w = monitor.MonitorWindow(monitor.parse_args())
        try:
            self.assertIsNotNone(w.settings_dialog)
            for widget in (w.model_combo, w.cloud_combo, w.slope_combo, w.planning_mode, w.planning_placement):
                self.assertIs(widget.window(), w.settings_dialog)          # moved into the popup
            self.assertEqual((w.model_combo.currentData(), w.cloud_combo.currentData(), w.slope_combo.currentData(),
                              w.planning_mode.currentData(), w.planning_placement.currentData()),
                             ("indoor", "device", "parallel", "vio_duty", "jetson"))
            self.assertIn("VIO duty", w.planning_choice.text())
            self.assertIn("Jetson", w.planning_choice.text())
            w.open_settings()
            self.assertTrue(w.settings_dialog.isVisible())
        finally:
            w.close()
            self.app.processEvents()


class StartupSettingsTests(unittest.TestCase):
    def test_timing_cloud_slope_and_model_flow(self):
        import startup
        import time as _time
        for chosen in ("indoor", "fcn"):
            scripts, events = [], []
            order = []

            def fake_run(host, script, password, timeout_s=18, sudo=False):
                text = Path(script).read_text()
                scripts.append((host, Path(script).name, text.splitlines()[0], text))
                stdout = "ROLE=pi5\n" if "preflight_pi" in str(script) else ("ROLE=jetson\n" if "preflight_jetson" in str(script) else "")
                if "export NOMAD_CLOUD_SOURCE" in text:      # the Jetson start takes a while
                    order.append("jetson_start_begin"); _time.sleep(1.5); order.append("jetson_start_end")
                if "check_jetson_oak_ready" in text or "OAK_NOT_READY" in text:
                    stdout = "OAK_READY\n"
                if Path(script).name == "start_pi_vio_fusion.sh":
                    order.append("vio_start")
                return {"ok": True, "stdout": stdout, "reason": "ok", "returncode": 0, "state": "SUCCEEDED"}
            with patch.object(startup, "run_remote", fake_run), \
                    patch.object(startup, "stop_boards", return_value={"ok": True}), \
                    patch.object(startup, "synchronize_clock", return_value={"ok": True}), \
                    patch("slope_control.switch_slope_mode", return_value={"ok": True, "mode": "parallel"}) as slope, \
                    patch("model_control.switch_model", return_value={"ok": True, "model": chosen}) as model:
                value = startup.start_boards("user@192.168.1.5", "nvidia@192.168.1.6", "pw-test", progress=events.append,
                                             semantic_model=chosen, cloud_source="device", slope_mode="parallel")
            self.assertTrue(value["ok"], value)
            slope.assert_called_once()
            jetson_start = [s for s in scripts if s[0].startswith("nvidia@") and s[2].startswith("export NOMAD_CLOUD_SOURCE")]
            self.assertEqual(jetson_start[0][2], "export NOMAD_CLOUD_SOURCE=device")
            self.assertIn("export NOMAD_SEMANTIC_MODEL=" + chosen, jetson_start[0][3])
            self.assertTrue(all("elapsed_s" in e for e in events))
            self.assertIn("sensors", value["timings_s"])
            self.assertIn("total_s", value)
            stages = [e["stage"] for e in events]
            self.assertLess(stages.index("settings"), stages.index("sensors"))     # slope saved before the map starts
            # 2026-11-13: VIO starts once this Start's OAK streams, while the Jetson start is still running.
            self.assertLess(order.index("vio_start"), order.index("jetson_start_end"))
            if chosen == "indoor":   # started directly by start_jetson.sh, no switch afterwards
                model.assert_not_called()
            else:
                model.assert_called_once()
                self.assertGreater(stages.index("model"), stages.index("running"))

    def test_invalid_settings_rejected(self):
        import startup
        for kwargs in ({"semantic_model": "x"}, {"cloud_source": "x"}, {"slope_mode": "x"}):
            with self.assertRaises(ValueError):
                startup.start_boards("user@192.168.1.5", "nvidia@192.168.1.6", "pw-test", **kwargs)


class SpeedupScriptTests(unittest.TestCase):
    def test_fast_paths_keep_safety_checks(self):
        network = (ROOT / "remote/restore_network.sh").read_text()
        self.assertIn("NETWORK_ALREADY_UP=1", network)
        self.assertLess(network.index("NETWORK_ALREADY_UP"), network.index("nmcli -w 15 connection up"))
        for check in ("Speed: 100Mb/s", "Duplex: Full", "Link detected: yes", "net.ipv4.ipfrag_max_dist"):
            self.assertIn(check, network)
        stop = (ROOT / "remote/stop_trial.sh").read_text()
        self.assertIn("wait_gone 20", stop)
        self.assertNotIn("signal_stage INT\n  sleep 2", stop)
        vio = (ROOT / "remote/start_pi_vio_fusion.sh").read_text()
        self.assertIn("backend started", vio)
        self.assertNotIn('echo $! > "$run/vio.pid"\nsleep 6', vio)
        jetson = (ROOT / "remote/start_jetson.sh").read_text()
        # 2026-11-15: opencv3way maps to initial_source opencv + SGBM 3WAY parameters.
        self.assertIn('initial_source:="$cloud_initial"', jetson)
        import control
        with tempfile.TemporaryDirectory() as temp:
            with patch.dict(os.environ, {"XDG_RUNTIME_DIR": temp}):
                options = control._connection_reuse_options(Path(temp) / "state")
            self.assertIn("ControlMaster=auto", options)
            self.assertTrue(options[-1].startswith("ControlPath=" + temp))
            long = Path(temp) / ("x" * 120)
            with patch.dict(os.environ, {"XDG_RUNTIME_DIR": str(long)}):
                self.assertEqual(control._connection_reuse_options(long), [])   # too long: no reuse, no failure


if __name__ == "__main__":
    unittest.main()
