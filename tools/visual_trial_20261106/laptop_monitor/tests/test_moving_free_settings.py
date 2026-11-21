"""Moving region GUI -> stored terrain settings -> per-run semantic config."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from test_mapper_settings import run, program, SEMANTIC_SAMPLE, FakeJetson
import test_mapper_settings as original
import terrain_settings as ts
import board_settings as bs

class MovingFreeTests(unittest.TestCase):
    def semantic(self):
        cfg = copy.deepcopy(SEMANTIC_SAMPLE)
        cfg["moving_front_free_region"].update(
            rear_length_m=0.35, rear_half_width_m=0.2,
            driven_path_memory=True, driven_front_m=0.025,
            override_policy="semantic_zero_measured_slope_kept_except_person",
            excluded_class_ids=[1])
        return cfg

    def write(self, td, name, data):
        p = Path(td) / name
        p.write_text(json.dumps(data))
        return p

    def test_inherit_preserves_old_upload_and_board_roundtrip(self):
        self.assertNotIn("gui_overrides", ts.board_json(ts.defaults()))
        with tempfile.TemporaryDirectory() as td:
            home = Path(td) / "home"
            target = home / ts.REMOTE_DIR / "terrain.json"
            target.parent.mkdir(parents=True)
            old = ts.board_json(ts.defaults())
            target.write_text(json.dumps(old))
            store = bs.BoardStore("terrain", "nvidia@fake", None, Path(td)/"state", runner=FakeJetson(home))
            cfg = store.read()["values"]
            self.assertEqual(cfg["terrain"]["gui_overrides"]["moving_front_enabled"], "보드값")
            cfg["terrain"]["gui_overrides"].update(moving_front_enabled="끔", moving_front_radius_m=0.6,
                                                  moving_rear_length_m=0.4, moving_rear_half_width_m=0.25)
            store.save(cfg)
            again = store.read()["values"]
            self.assertEqual(again["terrain"]["gui_overrides"], cfg["terrain"]["gui_overrides"])

    def test_toggle_and_dimensions_reach_only_moving_section(self):
        for choice, expected in (("켬", True), ("끔", False)):
            with self.subTest(choice=choice), tempfile.TemporaryDirectory() as td:
                before = self.semantic()
                target = self.write(td, "semantic.json", before)
                ov = self.write(td, "terrain.json", ts.board_json({"terrain": {"gui_overrides": {
                    "moving_front_enabled": choice, "moving_front_radius_m": 0.7,
                    "moving_rear_length_m": 0.45, "moving_rear_half_width_m": 0.3}}}))
                self.assertEqual(run("check", "terrain", ov, target).returncode, 0)
                done = run("apply", "terrain", ov, target)
                self.assertEqual(done.returncode, 0, done.stdout+done.stderr)
                after = json.loads(target.read_text())
                want = copy.deepcopy(before)
                want["moving_front_free_region"].update(enabled=expected, radius_m=0.7,
                                                       rear_length_m=0.45, rear_half_width_m=0.3)
                self.assertEqual(after, want)
                self.assertIn("MOVING_FREE_SETTINGS", done.stdout)
                self.assertEqual(run("strip", "terrain", ov).returncode, 0)
                self.assertNotIn("gui_overrides", json.loads(ov.read_text()))

    def test_merged_board_constraints_refuse_without_writing(self):
        for fields in ({"moving_front_radius_m": 0.1}, {"moving_rear_length_m": 0},
                       {"moving_rear_half_width_m": 0}, {"moving_front_enabled": "bad"},
                       {"moving_front_radius_m": 0}, {"moving_front_radius_m": True},
                       {"moving_front_radius_m": float("nan")}, {"moving_rear_length_m": 0.6}):
            with self.subTest(fields=fields), tempfile.TemporaryDirectory() as td:
                target = self.write(td,"semantic.json",self.semantic())
                original = target.read_bytes()
                ov = self.write(td,"terrain.json",{"gui_overrides": fields})
                self.assertEqual(run("check", "terrain", ov, target).returncode, 2)
                self.assertEqual(run("apply", "terrain", ov, target).returncode, 2)
                self.assertEqual(target.read_bytes(), original)

    def test_gui_rejects_ranges_and_inconsistent_explicit_dimensions(self):
        for fields in ({"moving_front_radius_m": 0}, {"moving_front_radius_m": float("inf")},
                       {"moving_rear_length_m": float("nan")}, {"moving_front_enabled": True},
                       {"moving_front_radius_m": 0.3, "moving_rear_half_width_m": 0.4}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                ts.validate({"terrain":{"gui_overrides":fields}})

    def test_actual_start_block_uses_copy(self):
        helper = original.StartScriptTests()
        cfg = ts.board_json({"terrain":{"gui_overrides":{"moving_front_enabled":"끔",
                                "moving_front_radius_m":0.6,"moving_rear_length_m":0.3,
                                "moving_rear_half_width_m":0.2}}})
        done, run_dir, shared = helper.run_terrain_block(cfg)
        self.assertIn("BLOCK_OK", done.stdout, done.stdout+done.stderr)
        actual = json.loads((run_dir/"terrain_legacy_cfg/semantic.real.json").read_text())
        self.assertFalse(actual["moving_front_free_region"]["enabled"])
        self.assertEqual(actual["moving_front_free_region"]["rear_half_width_m"],0.2)
        self.assertEqual(json.loads((shared/"semantic.real.json").read_text()),SEMANTIC_SAMPLE)
        self.assertNotIn("gui_overrides", json.loads((run_dir/"terrain_cfg/terrain.json").read_text()))

    def test_preflight_merged_constraints_before_sensors(self):
        from test_mapper_settings import START
        text = START.read_text()
        check = 'nomad_mapper_override check terrain "$HOME/nomad_terrain_ws/active_config/terrain.json" "$prefix/share/nomad_costmap_cpp/config/semantic.real.json"'
        self.assertIn(check,text)
        self.assertLess(text.index(check),text.index('start_module lidar'))

    def test_dialog_edits_and_saves_new_rows(self):
        os.environ.setdefault("QT_QPA_PLATFORM","offscreen")
        from PyQt5 import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as td:
            d = ts.SettingsDialog(None, Path(td))
            d.widgets[("terrain","gui_overrides.moving_front_enabled")][0].setCurrentText("끔")
            for key,value in (("moving_front_radius_m",0.6),("moving_rear_length_m",0.4),("moving_rear_half_width_m",0.25)):
                d.widgets[("terrain","gui_overrides."+key)][0].setValue(value)
            cfg = ts.board_json(d.values())
            self.assertEqual(cfg["gui_overrides"]["moving_front_enabled"],"끔")
            self.assertEqual(cfg["gui_overrides"]["moving_rear_half_width_m"],0.25)
            d.dialog.close()
