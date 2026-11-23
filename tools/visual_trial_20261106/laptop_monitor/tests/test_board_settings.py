"""2026-11-13: Jetson-stored planner/mapper2 settings, presets and change check.

The board scripts run locally with HOME = a temporary folder that mimics the Jetson layout
(no board is contacted)."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import board_settings as bs
import mapper2_settings as m2
import planning_settings as ps

PLANNER_YAML = """# comment
nomad_planning_cpp:
  ros__parameters:
    use_sim_time: false
    planning_frame: odom
    wheelbase: 0.335
    forward_speed: 0.22   # board-only value
    search_expansions: 40000
    cost.terrain_weight: 5.0
    corridor.observation_guard: false
    preview_forward_axis_x: -0.004534
    topics: [a, b]
"""


class FakeJetson:
    """run_remote stand-in: runs the script with bash in a fake home."""

    def __init__(self, home):
        self.home = Path(home)
        self.calls = 0

    def __call__(self, host, script, password, timeout_s=18):
        self.calls += 1
        done = subprocess.run(["bash", "-s"], input=Path(script).read_text(), text=True, capture_output=True,
                              env={"PATH": os.environ["PATH"], "HOME": str(self.home)}, timeout=timeout_s)
        return {"ok": done.returncode == 0, "stdout": done.stdout, "reason": "ok" if done.returncode == 0 else
                "remote_script_failed", "returncode": done.returncode}


def jetson_home(temp):
    home = Path(temp) / "home"
    config = home / "nomad_planning_ws/install/nomad_planning_cpp/share/nomad_planning_cpp/config"
    config.mkdir(parents=True)
    (config / "planning_output.yaml").write_text(PLANNER_YAML)
    for part, package in bs.Mapper2Module.PACKAGES.items():
        folder = home / "nomad_mapper2_ws/src" / package / "config"
        folder.mkdir(parents=True)
        (folder / f"{part}.json").write_text(json.dumps(m2.defaults()[part]))
    return home


class ParseTests(unittest.TestCase):
    def test_ros_params_flat_scalars(self):
        flat = bs.parse_ros_params(PLANNER_YAML)
        self.assertEqual(flat["forward_speed"], 0.22)
        self.assertEqual(flat["search_expansions"], 40000)
        self.assertIs(flat["corridor.observation_guard"], False)
        self.assertEqual(flat["planning_frame"], "odom")
        self.assertNotIn("topics", flat)

    def test_out_of_range_board_value_is_reported_not_used(self):
        configs, problems = bs.merge_values(ps.SPEC, ps.defaults(), flat={"forward_speed": 50.0, "wheelbase": 0.3})
        self.assertEqual(configs["vehicle"]["forward_speed"], ps.defaults()["vehicle"]["forward_speed"])
        self.assertEqual(configs["vehicle"]["wheelbase"], 0.3)
        self.assertEqual(len(problems), 1)
        self.assertIn("forward_speed", problems[0])

    def test_preset_names(self):
        self.assertTrue(bs.preset_path("planner", "1113 야지 잘됨").endswith("/planner/1113 야지 잘됨.json"))
        for bad in ("../x", "a/b", "", " x", ".hidden", "x;rm", "a" * 60):
            with self.assertRaises(bs.BoardError, msg=bad):
                bs.preset_path("planner", bad)


class PlannerBoardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.mkdtemp()
        self.home = jetson_home(self.temp)
        self.state = Path(self.temp) / "state"
        self.fake = FakeJetson(self.home)

    def tearDown(self):
        shutil.rmtree(self.temp)

    def store(self):
        return bs.BoardStore("planner", "nvidia@192.168.1.152", None, self.state, runner=self.fake)

    def test_first_read_shows_installed_board_values_then_save_and_read_back(self):
        store = self.store()
        data = store.read()
        self.assertEqual(data["source"], "default")
        self.assertTrue(data["board_default"])
        self.assertEqual(data["values"]["vehicle"]["forward_speed"], 0.22)   # from the Jetson yaml, not the GUI copy
        self.assertEqual(data["values"]["corridor"]["search_expansions"], 40000)
        self.assertEqual(ps.load(self.state)["vehicle"]["forward_speed"], 0.22)  # offline cache
        values = data["values"]
        values["vehicle"]["forward_speed"] = 0.3
        store.save(values)
        saved = self.home / "nomad_settings/planner"
        self.assertIn("    forward_speed: 0.3\n", (saved / "overrides.yaml").read_text())
        self.assertTrue((saved / "overrides.yaml").read_text().startswith("nomad_planning_cpp:\n"))
        again = self.store().read()   # another laptop
        self.assertEqual(again["source"], "current")
        self.assertEqual(again["values"]["vehicle"]["forward_speed"], 0.3)
        self.assertEqual(again["defaults"]["vehicle"]["forward_speed"], 0.22)
        # Second save makes a backup of the first one.
        again["values"]["vehicle"]["forward_speed"] = 0.25
        self.store_from(again).save(again["values"])

    def store_from(self, _data):
        store = self.store()
        store.read()
        return store

    def test_backup_before_overwrite(self):
        store = self.store()
        values = store.read()["values"]
        store.save(values)
        values["vehicle"]["forward_speed"] = 0.3
        store.save(values)   # same laptop: fingerprint updated after its own save
        backups = sorted((self.home / "nomad_settings/backup/planner").iterdir())
        self.assertEqual(len(backups), 2)
        self.assertEqual(list(backups[0].iterdir()), [])            # nothing existed before the first save
        first = json.loads((backups[1] / "current.json").read_text())
        self.assertEqual(first["vehicle"]["forward_speed"], 0.22)

    def test_conflict_when_another_laptop_saved_in_between(self):
        a, b = self.store(), self.store()
        va, vb = a.read()["values"], b.read()["values"]
        vb["vehicle"]["forward_speed"] = 0.2
        b.save(vb)
        va["vehicle"]["forward_speed"] = 0.3
        with self.assertRaises(bs.BoardError) as ctx:
            a.save(va)
        self.assertIn("다른 노트북", str(ctx.exception))
        self.assertEqual(self.store().read()["values"]["vehicle"]["forward_speed"], 0.2)

    def test_save_refuses_out_of_range_and_without_read(self):
        store = self.store()
        with self.assertRaises(bs.BoardError):
            store.save(ps.defaults())
        values = store.read()["values"]
        values["vehicle"]["forward_speed"] = 9.0
        with self.assertRaises(ValueError):
            store.save(values)
        self.assertFalse((self.home / "nomad_settings/planner/current.json").exists())

    def test_presets_save_list_load_and_overwrite(self):
        store = self.store()
        values = store.read()["values"]
        values["cost"]["cost"]["terrain_weight"] = 2.5
        store.save_preset("1113 야지 잘됨", values, "맑음")
        with self.assertRaises(FileExistsError):
            store.save_preset("1113 야지 잘됨", values)
        values["cost"]["cost"]["terrain_weight"] = 3.0
        store.save_preset("1113 야지 잘됨", values, overwrite=True)
        data = self.store().read()
        preset = data["presets"]["1113 야지 잘됨"]
        self.assertEqual(preset["note"], "")
        loaded, problems = store.preset_values(preset, data["defaults"])
        self.assertEqual(loaded["cost"]["cost"]["terrain_weight"], 3.0)
        self.assertEqual(problems, [])
        # Loading a preset does not change the board's current values.
        self.assertEqual(data["source"], "default")

    def test_unreachable_board(self):
        def down(*_a, **_k):
            return {"ok": False, "stdout": "", "reason": "ssh_unreachable"}
        store = bs.BoardStore("planner", "nvidia@192.168.1.152", None, self.state, runner=down)
        with self.assertRaises(bs.BoardError) as ctx:
            store.read()
        self.assertIn("연결하지 못했습니다", str(ctx.exception))


class Mapper2BoardTests(unittest.TestCase):
    def test_read_defaults_save_active_config_and_start_keeps_board_values(self):
        with tempfile.TemporaryDirectory() as temp:
            home = jetson_home(temp)
            fake = FakeJetson(home)
            store = bs.BoardStore("mapper2", "nvidia@192.168.1.152", None, Path(temp) / "state", runner=fake)
            data = store.read()
            self.assertEqual(data["source"], "default")
            values = data["values"]
            m2.set_dotted(values["cost"], "step_block.max_step_m", 0.08)
            store.save(values)
            active = home / m2.REMOTE_DIR
            self.assertEqual(json.loads((active / "cost.json").read_text())["step_block"]["max_step_m"], 0.08)
            # The start upload only fills missing files: the saved board value stays.
            (active / "height.json").unlink()
            subprocess.run(["bash", "-c", m2.upload_script(m2.defaults(), if_missing=True)],
                           env={**os.environ, "HOME": str(home)}, check=True, capture_output=True)
            self.assertEqual(json.loads((active / "cost.json").read_text())["step_block"]["max_step_m"], 0.08)
            self.assertEqual(json.loads((active / "height.json").read_text()), m2.defaults()["height"])
            self.assertEqual(store.read()["source"], "current")


class PlannerScriptTests(unittest.TestCase):
    def test_board_values_applied_after_installed_and_win_over_old_laptop_values(self):
        text = (ROOT / "remote/jetson_planner.sh").read_text()
        board = text.index('params+=(--params-file "$log/planner_board_settings.yaml")')
        self.assertLess(text.index('params=(--params-file "$config")'), board)
        self.assertIn('$HOME/nomad_settings/planner/overrides.yaml', text)
        self.assertIn('if [[ -r "$board_overrides" ]]; then\n    echo \'Laptop planner settings ignored', text)


class DialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_dialog_shows_board_values_saves_to_board_and_loads_preset(self):
        with tempfile.TemporaryDirectory() as temp:
            home = jetson_home(temp)
            fake = FakeJetson(home)
            state = Path(temp) / "state"
            store = bs.BoardStore("planner", "nvidia@x", None, state, runner=fake)
            dialog = ps.SettingsDialog(None, state, board=store)
            widget, _ = dialog.widgets[("vehicle", "forward_speed")]
            self.assertAlmostEqual(widget.value(), 0.22)
            self.assertTrue(dialog.board_ok)
            widget.setValue(0.3)
            dialog.save_preset = dialog.save_preset  # (button path covered by store tests)
            store.save_preset("good", dialog.values())
            widget.setValue(0.25)
            dialog.accept()
            self.assertEqual(json.loads((home / "nomad_settings/planner/current.json").read_text())
                             ["vehicle"]["forward_speed"], 0.25)
            again = ps.SettingsDialog(None, state, board=bs.BoardStore("planner", "nvidia@x", None, state, runner=fake))
            self.assertEqual(again.preset_combo.count(), 1)
            again.load_preset()
            w, _ = again.widgets[("vehicle", "forward_speed")]
            self.assertAlmostEqual(w.value(), 0.3)
            again.restore_tab()   # first tab = vehicle: back to the Jetson's installed value
            self.assertAlmostEqual(w.value(), 0.22)

    def test_untouched_number_boxes_keep_full_precision(self):
        # 2026-11-13 real failure: max_steer 0.6108652381980153 (35 deg) saved as 0.6109 -> planner refused.
        with tempfile.TemporaryDirectory() as temp:
            home = jetson_home(temp)
            fake = FakeJetson(home)
            state = Path(temp) / "state"
            dialog = ps.SettingsDialog(None, state, board=bs.BoardStore("planner", "nvidia@x", None, state, runner=fake))
            exact = ps.defaults()["vehicle"]["max_steer"]
            self.assertEqual(dialog.values()["vehicle"]["max_steer"], exact)
            dialog.accept()
            saved = json.loads((home / "nomad_settings/planner/current.json").read_text())
            self.assertEqual(saved["vehicle"]["max_steer"], exact)
            self.assertIn(f"max_steer: {exact!r}", (home / "nomad_settings/planner/overrides.yaml").read_text())
            again = ps.SettingsDialog(None, state, board=bs.BoardStore("planner", "nvidia@x", None, state, runner=fake))
            widget, _ = again.widgets[("vehicle", "max_steer")]
            widget.setValue(0.5)
            self.assertEqual(again.values()["vehicle"]["max_steer"], 0.5)   # a real edit is saved as typed
            again.restore_tab()
            self.assertEqual(again.values()["vehicle"]["max_steer"], exact)

    def test_dialog_offline_shows_cache_and_refuses_save(self):
        with tempfile.TemporaryDirectory() as temp:
            def down(*_a, **_k):
                return {"ok": False, "stdout": "", "reason": "ssh_unreachable"}
            store = bs.BoardStore("mapper2", "nvidia@x", None, temp, runner=down)
            dialog = m2.SettingsDialog(None, temp, board=store)
            self.assertFalse(dialog.board_ok)
            self.assertIn("읽지 못했습니다", dialog.board_status.text())
            self.assertFalse(dialog.preset_save_button.isEnabled())
            dialog.accept()
            self.assertIn("저장하지 않았습니다", dialog.error.text())


if __name__ == "__main__":
    unittest.main()
