"""Indoor selection, settings preservation and startup guards. Offline synthetic tests only."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import indoor_settings as indoor
import board_settings
import trial_settings
from module_settings import leaves


class IndoorSettingsTests(unittest.TestCase):
    def test_defaults_docs_and_no_camera_controls(self):
        c = indoor.defaults()["indoor"]
        self.assertEqual((c["floor_z_cm"], c["vio_fixed_z_m"], c["startup_radius_m"]), (5., 0., 1.5))
        keys = leaves(c)
        self.assertEqual(set(keys), set(indoor.SPEC.docs()["indoor"]))
        self.assertFalse(any(any(word in k for word in ("oak", "zed", "label", "camera")) for k in keys))
        self.assertIn("moving_free_region.radius_m", keys)

    def test_signed_z_values_save_and_reload(self):
        c = indoor.defaults()
        c["indoor"]["floor_z_cm"] = -3.5
        c["indoor"]["vio_fixed_z_m"] = .42
        with tempfile.TemporaryDirectory() as d:
            indoor.save(c, d)
            self.assertEqual(indoor.load(d), c)
        runtime = indoor.runtime_config(c)
        self.assertEqual((runtime["floor_z_cm"], runtime["vio_fixed_z_m"]), (-3.5, .42))

    def test_invalid_values_and_geometry(self):
        for key, value in (("floor_z_cm", float("nan")), ("vio_fixed_z_m", float("inf")),
                           ("max_pending_frames", 2.5), ("max_map_cells", 1), ("map_length_m", 1.)):
            c = indoor.defaults()
            c["indoor"][key] = value
            with self.assertRaises(ValueError):
                indoor.validate(c)
        c = indoor.defaults()
        c["indoor"]["moving_free_region"]["radius_m"] = .1
        with self.assertRaises(ValueError):
            indoor.validate(c)

    def test_board_roundtrip_retains_hidden_calibration(self):
        module = board_settings.IndoorModule()
        original = indoor.runtime_config(indoor.defaults())
        original["T_imu_from_lidar"][0][3] += .123
        original["imu_frame"] = "existing_vio_imu"
        module.configs_from_default([json.dumps(original).encode()])
        given, _ = module.configs_from_current([json.dumps(original).encode()])
        given["indoor"]["floor_z_cm"] = 8.
        given["indoor"]["vio_fixed_z_m"] = -.2
        saved = json.loads(module.current_files(given)[0])
        self.assertEqual(saved["T_imu_from_lidar"], original["T_imu_from_lidar"])
        self.assertEqual(saved["imu_frame"], "existing_vio_imu")
        self.assertEqual((saved["floor_z_cm"], saved["vio_fixed_z_m"]), (8., -.2))
        self.assertEqual(module.current_paths, ["nomad_indoor_ws/active_config/indoor.json"])

    def test_upload_does_not_overwrite_existing_board_values(self):
        script = indoor.upload_script(indoor.defaults(), if_missing=True)
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / indoor.REMOTE_DIR
            folder.mkdir(parents=True)
            target = folder / "indoor.json"
            target.write_text("preserve existing calibration and settings")
            subprocess.run(["bash"], input=script, text=True, env=dict(os.environ, HOME=d), check=True, capture_output=True)
            self.assertEqual(target.read_text(), "preserve existing calibration and settings")

    def test_selection_and_start_argument(self):
        args = trial_settings.start_arguments({"mapper": "indoor"})
        self.assertEqual(args[-2:], ["--mapper", "indoor"])
        summary = trial_settings.summary({"mapper": "indoor"})
        self.assertIn("실내 매퍼", summary)
        self.assertNotIn("OAK", summary)
        self.assertNotIn("SegFormer", summary)

    def test_indoor_start_exits_before_semantic_paths(self):
        script = (ROOT / "remote/start_jetson.sh").read_text()
        branch = script.split('# Indoor branch exits', 1)[1].split('\nexec 9>/tmp/nomad_resume_jetson_final_preview.lock', 1)[0]
        self.assertIn('"$iprobe" "$iw/active_config/indoor.json" </dev/null', branch)
        self.assertLess(branch.index('"$iprobe" "$iw/active_config/indoor.json" </dev/null'), branch.index('setsid env'))
        self.assertIn('enable_device_depth:=false', branch)
        self.assertNotIn('semantic.sh', branch)
        self.assertNotIn('external_stereo_cloud', branch)
        self.assertNotIn('ros2 launch', branch)
        self.assertIn('exit 0\nfi', branch)

    def test_start_check_stop_and_clock_guards_include_indoor(self):
        self.assertIn('== indoor', (ROOT / "remote/check_jetson_started.sh").read_text())
        stop = (ROOT / "remote/stop_trial.sh").read_text()
        for token in ('jetson:indoor', 'indoor:*nomad_indoor_mapper*', 'nomad_indoor_mapper) module=indoor'):
            self.assertIn(token, stop)
        self.assertIn('nomad_indoor_ma', (ROOT / 'board_network/time_agent.py').read_text())
        for name in ('start_jetson.sh', 'check_jetson_started.sh', 'stop_trial.sh'):
            subprocess.run(['bash', '-n', str(ROOT/'remote'/name)], check=True)


class IndoorStartupTests(unittest.TestCase):
    def test_ignores_saved_inference_and_slope_choices(self):
        import startup
        scripts = []
        def fake_run(host, script, password, **kwargs):
            scripts.append(Path(script).read_text())
            role = "pi5" if "preflight_pi.sh" in str(script) else "jetson"
            return {"ok": True, "stdout": f"ROLE={role}\n", "reason": "ok"}
        with patch.object(startup, "run_remote", fake_run), \
             patch.object(startup, "stop_boards", return_value={"ok": True}), \
             patch.object(startup, "synchronize_clock", return_value={"ok": True}), \
             patch("slope_control.switch_slope_mode") as slope, \
             patch("model_control.switch_model") as model:
            result = startup.start_boards("user@192.168.1.5", "nvidia@192.168.1.6", "synthetic",
                mapper="indoor", semantic_model="fcn", slope_mode="parallel")
        self.assertTrue(result["ok"], result)
        slope.assert_not_called()
        model.assert_not_called()
        self.assertTrue(any("NOMAD_INDOOR_EOF" in s for s in scripts))
        self.assertTrue(any("export NOMAD_MAPPER=indoor" in s for s in scripts))


class IndoorUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_form_hides_unrelated_controls_and_restores_them(self):
        form, combos = trial_settings.form(values={"mapper": "indoor"})
        self.assertTrue(combos["cloud_source"].isHidden())
        self.assertTrue(combos["semantic_model"].isHidden())
        self.assertFalse(combos["control_mode"].isHidden())
        combos["mapper"].setCurrentIndex(combos["mapper"].findData("terrain"))
        self.assertFalse(combos["cloud_source"].isHidden())
        form.deleteLater()

    def test_settings_dialog_has_two_independent_z_inputs(self):
        with tempfile.TemporaryDirectory() as d:
            dialog = indoor.SettingsDialog(None, d)
            try:
                floor = dialog.widgets[("indoor", "floor_z_cm")][0]
                fixed = dialog.widgets[("indoor", "vio_fixed_z_m")][0]
                floor.setValue(7.5)
                fixed.setValue(.25)
                c = indoor.validate(dialog.values())["indoor"]
                self.assertEqual((c["floor_z_cm"], c["vio_fixed_z_m"]), (7.5, .25))
            finally:
                dialog.dialog.close()
                dialog.dialog.deleteLater()

    def test_runtime_popup_cannot_start_unused_modules(self):
        import monitor
        with patch.object(sys, 'argv', ['monitor', '--no-ssh', '--enable-trial-control', '--trial-settings', '{"mapper":"indoor"}']):
            w = monitor.MonitorWindow(monitor.parse_args())
        try:
            self.assertTrue(w.model_combo.isHidden())
            self.assertTrue(w.cloud_combo.isHidden())
            self.assertTrue(w.slope_combo.isHidden())
            with patch.object(monitor.QtWidgets.QInputDialog, 'getText', side_effect=AssertionError("must not ask or connect")):
                w.apply_model()
                w.apply_cloud_source()
                w.apply_slope_mode()
        finally:
            w.close()
            self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
