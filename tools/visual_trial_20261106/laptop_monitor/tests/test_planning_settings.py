"""2026-11-13: Jetson planner settings (per-part tables, override file sent with the start command)."""
import base64
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import module_settings as ms
import planning_settings as ps

# Values of the Jetson planner's planning_output.yaml on 2026-11-13 (after terrain_weight 5.0).
JETSON_YAML_VALUES = {
    "wheelbase": 0.335, "track": 0.6, "forward_speed": 0.25, "planning_hz": 5.0, "command_hz": 20.0,
    "search_expansions": 50000, "odometry_timeout": 0.5, "command_lease": 0.15, "cost.terrain_weight": 5.0,
    "cost.slope_weight": 5.0, "cost.clearance_weight": 27.0, "rpp.unknown_speed": 0.1,
    "observed_guard.enabled": True, "corridor.min_clearance": 0.15, "corridor.observation_guard": False,
    "corridor.observation_turn_weight": 80.0,
}


class ModelTests(unittest.TestCase):
    def test_legacy_zero_is_restored_once_and_other_settings_are_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            cfg = ps.defaults()
            cfg["corridor"]["corridor"]["observation_turn_weight"] = 0.0
            cfg["cost"]["cost"]["terrain_weight"] = 2.5
            ps.SPEC.path(temp).write_text(json.dumps(cfg))  # old GUI, without revision
            restored = ps.load(temp)
            self.assertEqual(restored["corridor"]["corridor"]["observation_turn_weight"], 80.0)
            self.assertEqual(restored["cost"]["cost"]["terrain_weight"], 2.5)
            self.assertIn("corridor.observation_turn_weight: 80.0", ps.overrides_yaml(restored))
            restored["corridor"]["corridor"]["observation_turn_weight"] = 0.0
            ps.save(restored, temp)  # an explicit later choice must be respected
            self.assertEqual(ps.load(temp)["corridor"]["corridor"]["observation_turn_weight"], 0.0)

    def test_legacy_custom_weight_is_not_reset(self):
        with tempfile.TemporaryDirectory() as temp:
            cfg = ps.defaults()
            cfg["corridor"]["corridor"]["observation_turn_weight"] = 40.0
            ps.SPEC.path(temp).write_text(json.dumps(cfg))
            self.assertEqual(ps.load(temp)["corridor"]["corridor"]["observation_turn_weight"], 40.0)

    def test_every_value_documented_and_defaults_match_board(self):
        base, notes = ps.SPEC.defaults(), ps.SPEC.docs()
        flat = {}
        for part, _ in ps.PARTS:
            for key, value in ms.leaves(base[part]).items():
                doc = notes[part][key]
                self.assertTrue(doc["title"] and doc["description"], key)
                self.assertEqual(ms.check_value(value, doc, value), value)
                self.assertNotIn(key, flat, "duplicate parameter " + key)
                flat[key] = value
        for key, value in JETSON_YAML_VALUES.items():
            self.assertEqual(flat[key], value, key)
        self.assertFalse(any(k.startswith("topics.") or k in ("plan_only", "prototype_output") for k in flat))
        self.assertTrue(notes["safety"]["command_lease"]["safety"])

    def test_override_yaml_roundtrip_and_range(self):
        cfg = ps.defaults()
        cfg["vehicle"]["forward_speed"] = 0.2
        text = base64.b64decode(ps.overrides_b64(cfg)).decode()
        self.assertTrue(text.startswith("nomad_planning_cpp:\n  ros__parameters:\n"))
        self.assertIn("    forward_speed: 0.2\n", text)
        self.assertIn("    cost.terrain_weight: 5.0\n", text)
        self.assertIn("    rear_steer: true\n", text)
        bad = ps.defaults()
        bad["vehicle"]["forward_speed"] = 9.0
        with self.assertRaises(ValueError):
            ps.overrides_yaml(bad)
        with tempfile.TemporaryDirectory() as temp:
            saved = ps.save(cfg, temp)
            self.assertEqual(ps.load(temp)["vehicle"]["forward_speed"], 0.2)
            self.assertEqual(ps.summary(saved), "플래너 설정: 1개 바꿈")


    def test_ranges_inside_planner_start_checks_and_gpp_grouping(self):
        # planner_node.cpp refuses to start beyond these (2026-11-13 source audit).
        notes = ps.SPEC.docs()
        limits = {("corridor", "search_seconds"): 2.0, ("corridor", "corridor_seconds"): 1.0,
                  ("safety", "command_lease"): 0.3, ("safety", "max_map_cells"): 4000000,
                  ("timing", "planning_hz"): 20.0, ("timing", "command_hz"): 50.0}
        for (part, key), high in limits.items():
            self.assertLessEqual(notes[part][key]["max"], high, key)
        self.assertEqual(notes["corridor"]["corridor.observation_turn_weight"]["title"], "관측 부족 방향 회전 비용")
        self.assertIn("footprint_planning", notes["guard"])
        self.assertIn("goal_tolerance", notes["corridor"])
        self.assertEqual(dict(ps.PARTS)["corridor"], "전역 경로(GPP)·중간 목표")


class ClientTests(unittest.TestCase):
    def test_jetson_start_command_uses_jetson_saved_settings(self):
        # 2026-11-13 (later): values live on the Jetson (board_settings.py); the laptop sends none.
        from PyQt5 import QtWidgets
        QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        import planning_client as pc
        with tempfile.TemporaryDirectory() as temp, patch.dict(os.environ, {"XDG_STATE_HOME": temp}):
            cfg = ps.defaults()
            cfg["rpp"]["rpp"]["lookahead"] = 1.1
            ps.save(cfg, Path(temp) / "nomad-trial")
            client = pc.PlanningClient("user@192.168.2.252", "CMD", temp, placement="jetson",
                                       jetson_host="nvidia@192.168.1.152")
            parts = shlex.split(client.command)
            self.assertEqual(parts[-1], "--stdio")
            self.assertNotIn("--overrides-b64", parts)

    def test_jetson_script_applies_override_after_installed_config(self):
        text = (ROOT / "remote/jetson_planner.sh").read_text()
        subprocess.run(["bash", "-n", str(ROOT / "remote/jetson_planner.sh")], check=True)
        self.assertIn('params=(--params-file "$config")', text)
        self.assertLess(text.index('params=(--params-file "$config")'), text.index('params+=(--params-file "$log/planner_overrides.yaml")'))
        self.assertIn('--ros-args "${params[@]}"', text)


class DialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_dialog_has_seven_tabs_and_marks_safety(self):
        with tempfile.TemporaryDirectory() as temp:
            dialog = ps.SettingsDialog(None, temp)
            self.assertEqual(dialog.tabs.count(), 7)
            table = dialog.tabs.widget(2)
            self.assertTrue(table.item(0, 0).text().startswith("[안전] "))
            widget, _ = dialog.widgets[("cost", "cost.terrain_weight")]
            widget.setValue(2.5)
            dialog.accept()
            self.assertEqual(ps.load(temp)["cost"]["cost"]["terrain_weight"], 2.5)


if __name__ == "__main__":
    unittest.main()
