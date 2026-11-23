"""2026-11-13: new mapper (mapper2) choice, per-part settings with descriptions, upload and Jetson scripts."""
import json
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
import mapper2_settings as m2
import trial_settings as ts


class ModelTests(unittest.TestCase):
    def test_every_setting_has_a_description_and_valid_default(self):
        base, notes = m2.defaults(), m2.docs()
        for part, _label in m2.PARTS:
            keys = m2.leaves(base[part])
            self.assertTrue(keys, part)
            for key, value in keys.items():
                doc = notes[part][key]
                self.assertTrue(doc.get("title") and doc.get("description"), (part, key))
                self.assertEqual(m2.check_value(value, doc, value), value)

    def test_validate_save_load_and_ranges(self):
        with tempfile.TemporaryDirectory() as temp:
            self.assertEqual(m2.load(temp), m2.defaults())
            cfg = m2.defaults()
            m2.set_dotted(cfg["cost"], "step_block.max_step_m", 0.08)
            m2.set_dotted(cfg["height"], "slope.points", "ground_only")
            cfg["cost"]["junk"] = 1
            saved = m2.save(cfg, temp)
            self.assertNotIn("junk", saved["cost"])
            self.assertEqual(m2.load(temp)["cost"]["step_block"]["max_step_m"], 0.08)
            self.assertEqual(oct(os.stat(m2.path(temp)).st_mode & 0o777), "0o600")
            self.assertEqual(len(m2.changed_keys(saved)), 2)
            bad = m2.defaults()
            m2.set_dotted(bad["common"], "map.length_m", 1000.0)
            with self.assertRaises(ValueError):
                m2.save(bad, temp)
            bad = m2.defaults()
            m2.set_dotted(bad["height"], "slope.points", "evil;rm")
            with self.assertRaises(ValueError):
                m2.validate(bad)
            m2.path(temp).write_text("{broken")
            self.assertEqual(m2.load(temp), m2.defaults())

    def test_upload_script_writes_four_files_atomically(self):
        script = m2.upload_script(m2.defaults())
        subprocess.run(["bash", "-n"], input=script, text=True, check=True)
        with tempfile.TemporaryDirectory() as home:
            subprocess.run(["bash", "-c", script], env={**os.environ, "HOME": home}, check=True, capture_output=True)
            out = Path(home) / m2.REMOTE_DIR
            for part, _ in m2.PARTS:
                self.assertEqual(json.loads((out / f"{part}.json").read_text()), m2.defaults()[part])

    def test_trial_setting_and_start_arguments(self):
        self.assertEqual(ts.defaults()["mapper"], "legacy")
        self.assertNotIn("--mapper", ts.start_arguments(ts.defaults()))
        self.assertEqual(ts.start_arguments({**ts.defaults(), "mapper": "mapper2"})[-2:], ["--mapper", "mapper2"])


class StartupTests(unittest.TestCase):
    def test_mapper2_uploads_settings_then_exports_choice(self):
        import startup
        scripts = []

        def fake_run(host, script, password, timeout_s=18, sudo=False):
            text = Path(script).read_text()
            scripts.append((host, text))
            stdout = "ROLE=pi5\n" if "preflight_pi" in str(script) else ("ROLE=jetson\n" if "preflight_jetson" in str(script) else "")
            return {"ok": True, "stdout": stdout, "reason": "ok", "returncode": 0, "state": "SUCCEEDED"}
        with patch.object(startup, "run_remote", fake_run), \
                patch.object(startup, "stop_boards", return_value={"ok": True}), \
                patch.object(startup, "synchronize_clock", return_value={"ok": True}):
            value = startup.start_boards("user@192.168.1.5", "nvidia@192.168.1.6", "pw-test",
                                         mapper="mapper2", mapper2_configs=m2.defaults())
        self.assertTrue(value["ok"], value)
        upload = [i for i, (_h, t) in enumerate(scripts) if "MAPPER2_CONFIG_UPLOADED" in t]
        start = [i for i, (_h, t) in enumerate(scripts) if "export NOMAD_MAPPER=mapper2" in t]
        self.assertEqual(len(upload), 1)
        self.assertEqual(len(start), 1)
        self.assertLess(upload[0], start[0])
        self.assertTrue(scripts[upload[0]][0].startswith("nvidia@"))
        with self.assertRaises(ValueError):
            startup.start_boards("user@192.168.1.5", "nvidia@192.168.1.6", "pw-test", mapper="x")

    def test_legacy_default_uploads_nothing(self):
        import startup
        texts = []

        def fake_run(host, script, password, timeout_s=18, sudo=False):
            texts.append(Path(script).read_text())
            stdout = "ROLE=pi5\n" if "preflight_pi" in str(script) else ("ROLE=jetson\n" if "preflight_jetson" in str(script) else "")
            return {"ok": True, "stdout": stdout, "reason": "ok", "returncode": 0, "state": "SUCCEEDED"}
        with patch.object(startup, "run_remote", fake_run), \
                patch.object(startup, "stop_boards", return_value={"ok": True}), \
                patch.object(startup, "synchronize_clock", return_value={"ok": True}):
            self.assertTrue(startup.start_boards("user@192.168.1.5", "nvidia@192.168.1.6", "pw-test")["ok"])
        self.assertFalse(any("MAPPER2_CONFIG_UPLOADED" in t for t in texts))
        self.assertTrue(any("export NOMAD_MAPPER=legacy" in t for t in texts))


class RemoteScriptTests(unittest.TestCase):
    def test_jetson_scripts_know_mapper2(self):
        for name in ("start_jetson.sh", "check_jetson_started.sh", "stop_trial.sh"):
            subprocess.run(["bash", "-n", str(ROOT / "remote" / name)], check=True)
        start = (ROOT / "remote/start_jetson.sh").read_text()
        self.assertIn('"${NOMAD_MAPPER:-legacy}" == mapper2', start)
        self.assertIn("|nomad_mapper|nomad_mapper2|", start)               # duplicate guard
        self.assertLess(start.index("NOMAD_MAPPER:-legacy"), start.index("run_costmap.sh\" \\\n  --action start"))
        check = (ROOT / "remote/check_jetson_started.sh").read_text()
        self.assertIn("EXITED=mapper2", check)
        stop = (ROOT / "remote/stop_trial.sh").read_text()
        for token in ("jetson:mapper2", "mapper2:*nomad_mapper2*", "|nomad_mapper2|", "nomad_mapper2_ws/"):
            self.assertIn(token, stop)


class DialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_dialog_edits_and_saves_values(self):
        with tempfile.TemporaryDirectory() as temp:
            dialog = m2.SettingsDialog(None, temp)
            self.assertEqual(dialog.tabs.count(), 4)
            widget, _default = dialog.widgets[("cost", "step_block.max_step_m")]
            widget.setValue(0.09)
            toggle, _default = dialog.widgets[("height", "clearing.enabled")]
            toggle.setChecked(False)
            dialog.accept()
            saved = m2.load(temp)
            self.assertAlmostEqual(saved["cost"]["step_block"]["max_step_m"], 0.09)
            self.assertFalse(saved["height"]["clearing"]["enabled"])
            dialog.tabs.setCurrentIndex(3)
            dialog.restore_tab()
            self.assertAlmostEqual(dialog.values()["cost"]["step_block"]["max_step_m"], 0.05)


if __name__ == "__main__":
    unittest.main()
