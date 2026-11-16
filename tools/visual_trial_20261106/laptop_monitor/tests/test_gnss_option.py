"""'GNSS 사용 안 함' at trial start: no Pi5 GNSS module, run directory still made."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import startup


def fake_remote(calls):
    def run(host, script, password, timeout_s=18, sudo=False):
        name = Path(script).name
        calls.append(name)
        role = "pi5" if host.startswith("user@") else "jetson"
        return {"ok": True, "stdout": f"ROLE={role}\n", "state": "OK", "reason": "", "returncode": 0}
    return run


class StartupGnssOptionTest(unittest.TestCase):
    def start(self, **kwargs):
        calls = []
        with patch.object(startup, "run_remote", side_effect=fake_remote(calls)), \
             patch.object(startup, "stop_boards", return_value={"ok": True}), \
             patch.object(startup, "synchronize_clock", return_value={"ok": True}):
            value = startup.start_boards("user@192.168.1.5", "nvidia@192.168.1.6", "test-only", **kwargs)
        return value, calls

    def test_default_starts_gnss(self):
        value, calls = self.start()
        self.assertTrue(value["ok"])
        self.assertIn("start_pi_gnss.sh", calls)
        self.assertNotIn("start_pi_run_only.sh", calls)

    def test_no_gnss_starts_run_directory_only(self):
        value, calls = self.start(use_gnss=False)
        self.assertTrue(value["ok"])
        self.assertIn("start_pi_run_only.sh", calls)
        self.assertNotIn("start_pi_gnss.sh", calls)
        # VIO/fusion and the started check still run after the sensor stage.
        self.assertLess(calls.index("start_pi_run_only.sh"), calls.index("start_pi_vio_fusion.sh"))
        self.assertIn("check_pi_started.sh", calls)

    def test_scripts(self):
        run_only = (ROOT / "remote/start_pi_run_only.sh").read_text()
        gnss = (ROOT / "remote/start_pi_gnss.sh").read_text()
        self.assertNotIn("run_gnss.sh", run_only)
        for line in ('run="$ws/log/visual_trial_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"',
                     "printf '%s\\n' \"$run\" > /tmp/nomad_integrated_pi_run_dir"):
            self.assertIn(line, gnss)
            self.assertIn(line, run_only)
        self.assertIn('gnss_disabled', run_only)
        check = (ROOT / "remote/check_pi_started.sh").read_text()
        self.assertIn('-f "$run/gnss_disabled" && ! -e "$run/gnss.pid"', check)

    def test_cli_flag(self):
        captured = {}
        def fake(pi, jetson, password, progress=None, use_gnss=True, **settings):
            captured["use_gnss"] = use_gnss
            captured["settings"] = settings
            return {"ok": True, "state": "STARTED", "stage": "started", "message": "", "ready": False}
        with patch.object(startup, "start_boards", side_effect=fake), patch("builtins.print"):
            startup.main(["--pi-host", "user@192.168.1.5", "--jetson-host", "nvidia@192.168.1.6", "--no-gnss"])
            self.assertFalse(captured["use_gnss"])
            startup.main(["--pi-host", "user@192.168.1.5", "--jetson-host", "nvidia@192.168.1.6"])
            self.assertTrue(captured["use_gnss"])
            self.assertEqual(captured["settings"], {"semantic_model": "b2", "cloud_source": "opencv", "slope_mode": None,
                                               "oak_cloud_hz": "1", "mapper": "legacy", "mapper2_configs": None, "terrain_configs": None,
                                               "pi4_host": None})  # 2026-11-13 mapper choice · 2026-11-16 pi4_host
            startup.main(["--pi-host", "user@192.168.1.5", "--jetson-host", "nvidia@192.168.1.6",
                          "--semantic-model", "indoor", "--cloud-source", "device", "--slope-mode", "parallel"])
            self.assertEqual(captured["settings"], {"semantic_model": "indoor", "cloud_source": "device", "slope_mode": "parallel",
                                               "oak_cloud_hz": "1", "mapper": "legacy", "mapper2_configs": None, "terrain_configs": None,
                                               "pi4_host": None})  # 2026-11-13 mapper choice · 2026-11-16 pi4_host


class LauncherGnssOptionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_checkbox_passes_no_gnss(self):
        import launcher
        with tempfile.TemporaryDirectory() as temp, patch.object(launcher, "STATE_ROOT", Path(temp)):
            window = launcher.Launcher()
            try:
                window.begin_process = Mock()
                window.relay_port_available = Mock(return_value=True)
                window.hosts["pi"].setCurrentText("user@192.168.1.5")
                window.hosts["jetson"].setCurrentText("nvidia@192.168.1.6")
                window.password.setText("test-only")
                self.assertFalse(window.no_gnss.isChecked())
                window.start_trial()
                self.assertNotIn("--no-gnss", window.begin_process.call_args.args[2])
                window.begin_process.reset_mock()
                window.no_gnss.setChecked(True)
                window.start_trial()
                self.assertIn("--no-gnss", window.begin_process.call_args.args[2])
            finally:
                window.close()
                window.deleteLater()


if __name__ == "__main__":
    unittest.main()
