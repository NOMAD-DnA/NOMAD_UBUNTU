"""Ordered candidate editing, legacy settings and board round trips (offline only)."""
import json
import os
from pathlib import Path
import sys
import tempfile
import subprocess
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
import planning_settings as ps
import board_settings as bs
from test_board_settings import FakeJetson, jetson_home


class CandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_legacy_order_and_single_candidate(self):
        cfg = ps.defaults()["rpp"]["rpp"]
        self.assertEqual(ps.effective_lookaheads(cfg), [.8, 1., .6, 1.2, 1.5])
        cfg["adaptive_lookahead"] = False
        self.assertEqual(ps.effective_lookaheads(cfg), [.8])
        cfg["lookahead_candidates"] = [1.2, .9, .6]
        self.assertEqual(ps.effective_lookaheads(cfg), [1.2, .9, .6])

    def test_unchanged_array_keeps_runtime_parameters_identical(self):
        cfg = ps.defaults()
        before = ps.overrides_yaml(cfg)
        cfg["rpp"]["rpp"]["lookahead_candidates"] = ps.effective_lookaheads(cfg["rpp"]["rpp"])
        self.assertEqual(ps.overrides_yaml(cfg), before)

    def test_old_binary_rejects_custom_arrays_before_start(self):
        script = (Path(ps.__file__).parent / "remote/jetson_planner.sh").read_text()
        start = script.index("  if grep -Eq '^[[:space:]]*rpp")
        end = script.index("\n  fi", start) + len("\n  fi")
        guard = script[start:end]
        with tempfile.TemporaryDirectory() as temp:
            binary = Path(temp) / "planner"
            yaml = Path(temp) / "overrides.yaml"
            binary.write_text("older planner")
            for text, expected in (("    rpp.lookahead: 0.8\n", 0),
                                   ("    rpp.lookahead_candidates: []\n", 0),
                                   ("    rpp.lookahead_candidates: [0.9, 0.6]\n", 2)):
                yaml.write_text(text)
                done = subprocess.run(["bash", "-c", guard], env={**os.environ, "planner": str(binary), "board_overrides": str(yaml)}, capture_output=True)
                self.assertEqual(done.returncode, expected, text)
            binary.write_text("new planner rpp.lookahead_candidates")
            done = subprocess.run(["bash", "-c", guard], env={**os.environ, "planner": str(binary), "board_overrides": str(yaml)}, capture_output=True)
            self.assertEqual(done.returncode, 0)

    def test_edit_add_remove_save_and_preserve_other_values(self):
        with tempfile.TemporaryDirectory() as temp:
            cfg = ps.defaults()
            cfg["vehicle"]["body_length"] = 1.2
            ps.save(cfg, temp)
            gui = ps.SettingsDialog(None, temp)
            editor, _ = gui.widgets[("rpp", "rpp.lookahead_candidates")]
            self.assertEqual(editor.value(), [.8, 1., .6, 1.2, 1.5])
            with patch.object(QtWidgets.QInputDialog, "getDouble", return_value=(.9, True)):
                editor.add_button.click()
            self.assertEqual(editor.value(), [.8, 1., .6, 1.2, 1.5, .9])
            with patch.object(QtWidgets.QInputDialog, "getItem", return_value=("2번째: 1 m", True)):
                editor.remove_button.click()
            expected = [.8, .6, 1.2, 1.5, .9]
            self.assertEqual(editor.value(), expected)
            gui.accept()
            saved = ps.load(temp)
            self.assertEqual(saved["rpp"]["rpp"]["lookahead_candidates"], expected)
            saved["rpp"]["rpp"]["lookahead_candidates"] = []
            # Existing numeric widgets have their established four-decimal display precision.
            baseline = ps.SettingsDialog(None, temp)
            self.assertEqual(baseline.widgets[("vehicle", "body_length")][0].value(), 1.2)
            for part in ("cost", "timing", "guard"):
                self.assertEqual(saved[part], cfg[part])
            table = gui.tabs.widget(5)
            visible = [table.item(i, 0).toolTip() for i in range(table.rowCount()) if not table.isRowHidden(i)]
            self.assertEqual(visible, ["rpp.lookahead_candidates", "rpp.lateral_accel", "rpp.unknown_speed"])
            gui.dialog.close()
            baseline.dialog.close()

    def test_invalid_input_prevents_save(self):
        with tempfile.TemporaryDirectory() as temp:
            gui = ps.SettingsDialog(None, temp)
            editor, _ = gui.widgets[("rpp", "rpp.lookahead_candidates")]
            for text in ("[]", "[0]", "[3.1]", "[0.8, 0.8]", "[NaN]", "[true]", "oops", "[0.8,]"):
                editor.array.setText(text)
                gui.accept()
                self.assertTrue(gui.error.text(), text)
                self.assertFalse(ps.SPEC.path(temp).exists(), text)
            gui.dialog.close()

    def test_board_and_preset_roundtrip_keeps_order(self):
        with tempfile.TemporaryDirectory() as temp:
            home = jetson_home(temp)
            store = bs.BoardStore("planner", "nvidia@offline", None, Path(temp) / "state", runner=FakeJetson(home))
            gui = ps.SettingsDialog(None, Path(temp) / "state", board=store)
            editor, _ = gui.widgets[("rpp", "rpp.lookahead_candidates")]
            editor.array.setText("[1.2, 0.9, 0.6]")
            gui.accept()
            files = home / "nomad_settings/planner"
            self.assertEqual(bs.parse_ros_params((files / "overrides.yaml").read_text())["rpp.lookahead_candidates"], [1.2, .9, .6])
            data = store.read()
            self.assertEqual(data["values"]["rpp"]["rpp"]["lookahead_candidates"], [1.2, .9, .6])
            store.save_preset("후보 시험", data["values"])
            fresh = store.read()
            preset, problems = store.preset_values(fresh["presets"]["후보 시험"], fresh["defaults"])
            gui.set_values(preset)
            self.assertEqual(problems, [])
            self.assertEqual(editor.value(), [1.2, .9, .6])
            gui.tabs.setCurrentIndex(5)
            gui.restore_tab()
            self.assertEqual(editor.value(), [.8, 1., .6, 1.2, 1.5])
            legacy = ps.defaults()
            legacy["rpp"]["rpp"]["lookahead"] = 1.1
            legacy["rpp"]["rpp"]["adaptive_lookahead"] = False
            gui.set_values(legacy)
            self.assertEqual(editor.value(), [1.1])
            gui.dialog.close()


if __name__ == "__main__":
    unittest.main()
