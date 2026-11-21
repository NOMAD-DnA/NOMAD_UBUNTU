"""2026-11-13: improved legacy mapper ("기존 매퍼 개선") settings and start wiring."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import board_settings
import terrain_settings
import trial_settings


class TerrainSettingsTest(unittest.TestCase):
    def test_defaults_and_ranges(self):
        cfg = terrain_settings.defaults()["terrain"]
        self.assertEqual(cfg["map_length_m"], 20.0)
        self.assertEqual(cfg["slope_angle_reference"], "gravity")
        self.assertFalse(cfg["freeze_slope_after_first_valid"])
        self.assertFalse(cfg["label_accumulation"]["enabled"])
        with self.assertRaises(ValueError):
            terrain_settings.validate({"terrain": {"slope_angle_reference": "sideways"}})
        with self.assertRaises(ValueError):
            terrain_settings.validate({"terrain": {"label_accumulation": {"max_labels_per_tick": 99}}})
        ok = terrain_settings.validate({"terrain": {"map_length_m": 30, "label_accumulation": {"enabled": True}}})
        self.assertEqual(ok["terrain"]["map_length_m"], 30.0)
        self.assertTrue(ok["terrain"]["label_accumulation"]["enabled"])

    def test_oak_horizontal_limit(self):
        key = "oak_max_horizontal_distance_m"
        self.assertEqual(terrain_settings.defaults()["terrain"][key], 2.0)
        self.assertEqual(terrain_settings.validate({"terrain": {}})["terrain"][key], 2.0)
        for value in (0.1, 2.0, 3.5, 30.0):
            cfg = {"terrain": {key: value}}
            self.assertEqual(terrain_settings.board_json(cfg)[key], value)
        for value in (0, -1, 30.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                terrain_settings.validate({"terrain": {key: value}})
        doc = terrain_settings.SPEC.docs()["terrain"][key]
        self.assertIn("XY", doc["description"])
        self.assertIn("LiDAR", doc["description"])
        self.assertIn('"oak_max_horizontal_distance_m": 2.0',
                      terrain_settings.upload_script({"terrain": {}}))

    def test_oak_height_threshold(self):
        import json
        key = "oak_lidar_max_abs_z_difference_cm"
        self.assertEqual(terrain_settings.defaults()["terrain"][key], 12.0)
        self.assertEqual(terrain_settings.board_json({"terrain": {}})[key], 12.0)
        for value in (0.1, 5.0, 12.0, 20.0, 100.0):
            values = {"terrain": {key: value}}
            self.assertEqual(terrain_settings.board_json(values)[key], value)
            module = board_settings.MODULES["terrain"]()
            self.assertEqual(json.loads(module.current_files(values)[0])[key], value)
        for value in (0, -1, 100.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                terrain_settings.validate({"terrain": {key: value}})
        doc = terrain_settings.SPEC.docs()["terrain"][key]
        self.assertEqual(doc["unit"], "cm")
        self.assertIn("이상", doc["description"])

    def test_oak_xy_search_radius(self):
        import json
        key = "oak_lidar_xy_radius_cm"
        self.assertEqual(terrain_settings.defaults()["terrain"][key], 20.0)
        for cm in (0.1, 12.0, 20.0, 100.0):
            cfg = {"terrain": {key: cm}}
            self.assertEqual(terrain_settings.board_json(cfg)[key], cm)
            self.assertEqual(json.loads(board_settings.MODULES["terrain"]().current_files(cfg)[0])[key], cm)
        for cm in (0, -1, 100.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                terrain_settings.validate({"terrain": {key: cm}})
        self.assertEqual(terrain_settings.SPEC.docs()["terrain"][key]["unit"], "cm")

    def test_docs_cover_every_value(self):
        docs = terrain_settings.SPEC.docs()["terrain"]
        from module_settings import leaves
        self.assertEqual(set(leaves(terrain_settings.defaults()["terrain"])), set(docs))

    def test_upload_only_when_missing(self):
        text = terrain_settings.upload_script(terrain_settings.defaults(), if_missing=True)
        self.assertIn('nomad_terrain_ws/active_config', text)
        self.assertIn('if [ ! -e "$dir/terrain.json" ]', text)

    def test_board_module_paths(self):
        module = board_settings.MODULES["terrain"]()
        self.assertEqual(module.current_paths, ["nomad_terrain_ws/active_config/terrain.json"])
        self.assertEqual(module.default_paths, ["nomad_terrain_ws/src/nomad_terrain_mapper/config/terrain.json",
                         "nomad_ws/tools/visual_trial_20261106/runtime-map-preview/share/nomad_costmap_cpp/config/semantic.real.json"])

    def test_trial_choice_and_scripts(self):
        values = dict(trial_settings.defaults(), mapper="terrain")
        self.assertEqual(trial_settings.start_arguments(values)[-2:], ["--mapper", "terrain"])
        start = (ROOT / "remote/start_jetson.sh").read_text()
        self.assertIn('NOMAD_MAPPER:-legacy}" == terrain', start)
        self.assertIn("terrain_mapper_started", start)
        self.assertIn("terrain", (ROOT / "remote/check_jetson_started.sh").read_text())
        self.assertIn("jetson:terrain", (ROOT / "remote/stop_trial.sh").read_text())

    def test_board_package_defaults_match(self):
        board = Path("/home/user/Documents/ChatGPT/lig 프로젝트/관련자료/기존매퍼개선_20261113/ws/src/nomad_terrain_mapper/config")
        if not board.exists():
            self.skipTest("board source not on this laptop")
        # 2026-11-16: slope k keys are GUI-only until k != 4 (terrain_settings.SLOPE_KEYS, slope_k.py).
        import json
        for name in ("terrain.json", "terrain_doc.json"):
            gui = json.loads((ROOT / "terrain_config" / name).read_text())
            for key in terrain_settings.SLOPE_KEYS:
                gui.pop(key)
            # 2026-11-16: GUI-only class-boundary / initial-area object (never sent to the terrain binary).
            gui.pop(terrain_settings.GUI_KEY, None)
            for key in [k for k in gui if k.startswith(terrain_settings.GUI_KEY + ".")]:
                gui.pop(key)
            self.assertEqual(json.loads((board / name).read_text()), gui)


if __name__ == "__main__":
    unittest.main()
