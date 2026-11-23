"""2026-11-16: '경사 점수 계수' k (slope_k.py) for mapper2, terrain (improved legacy) and mapper3.

Offline only: the Jetson is a temporary HOME run with bash; the programs embedded in
remote/start_jetson.sh run with the local python3 on sample config files."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import board_settings as bs
import mapper2_settings as m2
import mapper_settings as ms
import slope_k
import terrain_settings as ts

START = ROOT / "remote/start_jetson.sh"
MAPPER3_SAMPLE = {"_comment": "v3 샘플", "grid": {"resolution": 0.05, "size": 400},
                  "lidar": {"enable": True, "start_free_radius_m": 1.0}, "cost": {"inflate": False}}


def embedded(start_marker, end_marker):
    text = START.read_text(encoding="utf-8")
    return text.split(start_marker, 1)[1].split(end_marker, 1)[0]


def override(*args):
    prog = embedded("nomad_mapper_override_py=$(cat <<'NOMAD_MAPPER_PY'\n", "\nNOMAD_MAPPER_PY\n")
    return subprocess.run([sys.executable, "-I", "-c", prog, *map(str, args)], capture_output=True, text=True)


class FakeJetson:
    def __init__(self, home):
        self.home = Path(home)

    def __call__(self, host, script, password, timeout_s=18):
        done = subprocess.run(["bash", "-s"], input=Path(script).read_text(), text=True, capture_output=True,
                              env={"PATH": os.environ["PATH"], "HOME": str(self.home)}, timeout=timeout_s)
        return {"ok": done.returncode == 0, "stdout": done.stdout,
                "reason": "ok" if done.returncode == 0 else "remote_script_failed", "returncode": done.returncode}


class MathTests(unittest.TestCase):
    def test_block_angle(self):
        self.assertEqual(slope_k.block_deg(4), 20.0)
        self.assertEqual(slope_k.block_deg(8), 10.0)
        self.assertEqual(slope_k.block_deg(2), 40.0)
        self.assertEqual(slope_k.block_deg(16), 5.0)
        self.assertAlmostEqual(slope_k.block_deg(3), 26.6667)
        for k in (1.78, 3, 5.5, 16):
            self.assertLess(k * slope_k.block_deg(k), 100.0)      # terrain C++ patch requires k*B <= 100
        self.assertTrue(slope_k.is_default(4.0) and slope_k.is_default(None))
        self.assertFalse(slope_k.is_default(4.5))

    def test_range(self):
        for bad in (1.7, 16.5, 0, -4, True, "4", float("nan")):
            with self.assertRaises(ValueError, msg=bad):
                slope_k.check_k(bad)
        self.assertEqual(slope_k.check_k(1.78), 1.78)
        self.assertLessEqual(slope_k.block_deg(slope_k.K_MIN), 45.0)   # mapper2 limit_deg max

    def test_mapper3_cost_math(self):
        self.assertEqual(slope_k.mapper3_cost({}, 4), {"slope_free_deg": 8.0, "slope_mid_deg": 20.0, "slope_high_deg": 25.0})
        self.assertEqual(slope_k.mapper3_cost({}, 8), {"slope_free_deg": 4.0, "slope_mid_deg": 10.0, "slope_high_deg": 12.5})
        self.assertEqual(slope_k.mapper3_cost({"slope_high_deg": 30.0}, 2)["slope_high_deg"], 60.0)


class Mapper2Tests(unittest.TestCase):
    def test_default_unchanged(self):
        self.assertEqual(m2.validate(m2.defaults())["cost"]["slope"], {"limit_deg": 20.0, "score_per_deg": 4.0})
        self.assertEqual(m2.changed_keys(m2.validate(m2.defaults())), [])

    def test_limit_follows_k_and_range(self):
        cfg = m2.defaults()
        cfg["cost"]["slope"] = {"score_per_deg": 8.0, "limit_deg": 33.0}
        self.assertEqual(m2.validate(cfg)["cost"]["slope"], {"limit_deg": 10.0, "score_per_deg": 8.0})
        for bad in (1.0, 17.0, 0.0):
            cfg["cost"]["slope"]["score_per_deg"] = bad
            with self.assertRaises(ValueError):
                m2.validate(cfg)


class TerrainTests(unittest.TestCase):
    def test_default_writes_no_new_keys(self):
        cfg = ts.board_json(ts.defaults())
        for key in ts.SLOPE_KEYS:
            self.assertNotIn(key, cfg)
        text = ts.upload_script(ts.defaults(), if_missing=True)
        self.assertNotIn("slope_", text.replace("slope_angle_reference", "").replace("freeze_slope", "")
                         .replace("retry_slope", "").replace("keep_slope", ""))
        files = bs.MODULES["terrain"]().current_files(ts.defaults())
        self.assertNotIn("slope_score_per_deg", files[0])
        self.assertEqual(json.loads(files[0]), cfg)

    def test_k_not_4_writes_both(self):
        cfg = ts.defaults()
        cfg["terrain"]["slope_score_per_deg"] = 5.0
        cfg["terrain"]["slope_block_deg"] = 33.0          # ignored: derived
        out = ts.board_json(cfg)
        self.assertEqual((out["slope_score_per_deg"], out["slope_block_deg"]), (5.0, 16.0))
        with self.assertRaises(ValueError):
            cfg["terrain"]["slope_score_per_deg"] = 20.0
            ts.validate(cfg)

    def test_board_save_and_read_back(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp) / "home"
            home.mkdir()
            store = bs.BoardStore("terrain", "nvidia@x", None, Path(temp) / "state", runner=FakeJetson(home))
            data = store.read()
            self.assertEqual(data["values"]["terrain"]["slope_score_per_deg"], 4.0)
            store.save(data["values"])
            saved = json.loads((home / ts.REMOTE_DIR / "terrain.json").read_text())
            self.assertNotIn("slope_score_per_deg", saved)
            values = data["values"]
            values["terrain"]["slope_score_per_deg"] = 8.0
            store.save(values)
            saved = json.loads((home / ts.REMOTE_DIR / "terrain.json").read_text())
            self.assertEqual((saved["slope_score_per_deg"], saved["slope_block_deg"]), (8.0, 10.0))
            again = bs.BoardStore("terrain", "nvidia@x", None, Path(temp) / "state", runner=FakeJetson(home)).read()
            self.assertEqual(again["values"]["terrain"]["slope_block_deg"], 10.0)
            self.assertEqual(again["problems"], [])


class StartTerrainTests(unittest.TestCase):
    """The terrain-block slope check of start_jetson.sh, run with a fake HOME and fake binaries."""

    def run_block(self, terrain_cfg, binary_text):
        text = START.read_text(encoding="utf-8")
        helper = text[text.index("# BEGIN_TERRAIN_SLOPE_PY"):text.index("\n", text.index("nomad_terrain_slope() {"))]
        block = text[text.index('  cp -p "$tw/active_config/terrain.json" "$run/terrain_cfg/"'):
                     text.index('  # 2026-11-16: GUI \'기존 매퍼 개선 설정\' class-boundary px')]
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp) / "home"
            tw = home / "nomad_terrain_ws"
            (tw / "active_config").mkdir(parents=True)
            (tw / "active_config/terrain.json").write_text(json.dumps(terrain_cfg, indent=1) + "\n")
            binary = tw / "install/nomad_terrain_mapper/lib/nomad_terrain_mapper/nomad_terrain_mapper"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(binary_text)
            run = Path(temp) / "run"
            (run / "terrain_cfg").mkdir(parents=True)
            script = ("set -eo pipefail\nfail() { echo \"FAIL: $*\"; exit 1; }\n" + helper + "\n"
                      f'tw="$HOME/nomad_terrain_ws"\nrun={run}\n' + block + "echo BLOCK_OK\n")
            done = subprocess.run(["bash", "-s"], input=script, text=True, capture_output=True,
                                  env={"PATH": os.environ["PATH"], "HOME": str(home)})
            copy = json.loads((run / "terrain_cfg/terrain.json").read_text())
            board = json.loads((tw / "active_config/terrain.json").read_text())
            return done, copy, board

    def test_unsupported_binary_strips_keys_and_warns(self):
        cfg = ts.board_json({"terrain": {"slope_score_per_deg": 8.0}})
        done, copy, board = self.run_block(cfg, b"\x7fELF old build Unknown terrain.json key: ")
        self.assertIn("BLOCK_OK", done.stdout, done.stdout + done.stderr)
        self.assertIn("경사 계수 미적용: 개선 매퍼 재빌드 필요", done.stdout)
        self.assertNotIn("slope_score_per_deg", copy)
        self.assertNotIn("slope_block_deg", copy)
        self.assertEqual(copy["map_length_m"], cfg["map_length_m"])
        self.assertEqual(board["slope_score_per_deg"], 8.0)            # saved board file untouched
        note, ok = slope_k.start_note(done.stdout)
        self.assertFalse(ok)
        self.assertIn("재빌드 필요", note)

    def test_supported_binary_keeps_keys(self):
        cfg = ts.board_json({"terrain": {"slope_score_per_deg": 8.0}})
        done, copy, _ = self.run_block(cfg, b"\x7fELF\0slope_score_per_deg\0slope_block_deg\0")
        self.assertIn("BLOCK_OK", done.stdout, done.stdout + done.stderr)
        self.assertNotIn("재빌드", done.stdout)
        self.assertEqual((copy["slope_score_per_deg"], copy["slope_block_deg"]), (8.0, 10.0))
        note, ok = slope_k.start_note(done.stdout)
        self.assertTrue(ok)
        self.assertEqual(note, "경사 점수 계수 8 · 막힘 10° (기존 매퍼 개선)")

    def test_default_file_untouched_and_silent(self):
        cfg = ts.board_json(ts.defaults())
        done, copy, _ = self.run_block(cfg, b"\x7fELF old")
        self.assertIn("BLOCK_OK", done.stdout)
        self.assertNotIn("MAPPER_SLOPE", done.stdout)
        self.assertNotIn("WARNING", done.stdout)
        self.assertEqual(copy, cfg)


class Mapper3Tests(unittest.TestCase):
    def write(self, folder, name, data, indent=1):
        path = Path(folder) / name
        path.write_text(json.dumps(data, ensure_ascii=False, indent=indent) + "\n", encoding="utf-8")
        return path

    def test_overrides_spec(self):
        self.assertIsNone(ms.defaults()["mapper3"]["slope_k"])
        self.assertNotIn("terrain", ms.defaults())   # terrain values: terrain_settings gui_overrides
        self.assertEqual(ms.validate({"mapper3": {"slope_k": 6}})["mapper3"]["slope_k"], 6.0)
        for bad in (1.0, 17, "6"):
            with self.assertRaises(ValueError):
                ms.validate({"mapper3": {"slope_k": bad}})
        self.assertEqual(ms.summary(ms.validate({"mapper3": {"slope_k": 8}}), "mapper3"),
                         "경계 보드값 · 초기영역 보드값 · 경사계수 8")
        self.assertEqual(ms.summary(ms.validate({"mapper3": {"slope_k": 4}}), "mapper3"), "경계 보드값 · 초기영역 보드값")

    def test_k_4_or_null_leaves_copy_byte_identical(self):
        with tempfile.TemporaryDirectory() as temp:
            target = self.write(temp, "mapper3.json", MAPPER3_SAMPLE)
            raw = target.read_bytes()
            for k in (None, 4, 4.0):
                ov = self.write(temp, "ov.json", {"mapper3": {"erode_px": None, "start_free_radius_m": None, "slope_k": k}})
                self.assertEqual(override("check", "mapper3", ov).returncode, 1)
                done = override("apply", "mapper3", ov, target)
                self.assertEqual(done.returncode, 0, done.stdout)
                self.assertNotIn("MAPPER_SLOPE", done.stdout)
            self.assertEqual(target.read_bytes(), raw)

    def test_k_scales_cost_angles(self):
        with tempfile.TemporaryDirectory() as temp:
            target = self.write(temp, "mapper3.json", MAPPER3_SAMPLE)
            ov = self.write(temp, "ov.json", {"mapper3": {"slope_k": 8}})
            self.assertEqual(override("check", "mapper3", ov).returncode, 0)
            done = override("apply", "mapper3", ov, target)
            self.assertEqual(done.returncode, 0, done.stdout)
            after = json.loads(target.read_text())
            expected = json.loads(json.dumps(MAPPER3_SAMPLE))
            expected["cost"].update(slope_free_deg=4.0, slope_mid_deg=10.0, slope_high_deg=12.5)
            self.assertEqual(after, expected)
            self.assertIn("overridden=slope_k", done.stdout)
            self.assertIn("MAPPER_SLOPE kind=mapper3 slope_k=8 slope_free_deg=4 slope_mid_deg=10 slope_high_deg=12.5",
                          done.stdout)
            note, ok = slope_k.start_note(done.stdout)
            self.assertTrue(ok)
            self.assertIn("4/10/12.5°", note)
            # explicit board values are scaled instead of the code defaults
            target = self.write(temp, "m3b.json", {"cost": {"slope_high_deg": 30.0}})
            override("apply", "mapper3", self.write(temp, "ov2.json", {"mapper3": {"slope_k": 2}}), target)
            self.assertEqual(json.loads(target.read_text())["cost"],
                             {"slope_high_deg": 60.0, "slope_free_deg": 16.0, "slope_mid_deg": 40.0})

    def test_invalid_k_stops_start(self):
        with tempfile.TemporaryDirectory() as temp:
            target = self.write(temp, "mapper3.json", MAPPER3_SAMPLE)
            raw = target.read_bytes()
            for bad in (0.5, 20, "8", True):
                ov = self.write(temp, "ov.json", {"mapper3": {"slope_k": bad}})
                self.assertEqual(override("check", "mapper3", ov).returncode, 2, bad)
                self.assertEqual(override("apply", "mapper3", ov, target).returncode, 2, bad)
            self.assertEqual(target.read_bytes(), raw)
            # a terrain section slope_k is not a mapper3 value and is ignored
            ov = self.write(temp, "ov.json", {"terrain": {"slope_k": 8}})
            self.assertEqual(override("check", "terrain", ov).returncode, 1)

    def test_legacy_block_untouched(self):
        text = START.read_text(encoding="utf-8")
        legacy = text[text.index("# Existing private native map"):]
        self.assertNotIn("slope_k", legacy)
        self.assertNotIn("nomad_terrain_slope", legacy)
        subprocess.run(["bash", "-n", str(START)], check=True)


class DialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_terrain_block_box_follows_k(self):
        with tempfile.TemporaryDirectory() as temp:
            dialog = ts.SettingsDialog(None, temp, board=None)
            k_box = dialog.widgets[("terrain", "slope_score_per_deg")][0]
            b_box = dialog.widgets[("terrain", "slope_block_deg")][0]
            self.assertFalse(b_box.isEnabled())
            self.assertEqual(ts.board_json(dialog.values()), ts.board_json(ts.defaults()))
            k_box.setValue(8.0)
            self.assertEqual(b_box.value(), 10.0)
            out = ts.board_json(dialog.values())
            self.assertEqual((out["slope_score_per_deg"], out["slope_block_deg"]), (8.0, 10.0))
            dialog.dialog.deleteLater()

    def test_mapper2_limit_follows_k(self):
        with tempfile.TemporaryDirectory() as temp:
            dialog = m2.SettingsDialog(None, temp, board=None)
            k_box = dialog.widgets[("cost", "slope.score_per_deg")][0]
            b_box = dialog.widgets[("cost", "slope.limit_deg")][0]
            self.assertFalse(b_box.isEnabled())
            self.assertEqual(m2.validate(dialog.values()), m2.validate(m2.defaults()))
            k_box.setValue(2.0)
            self.assertEqual(b_box.value(), 40.0)
            self.assertEqual(m2.validate(dialog.values())["cost"]["slope"], {"limit_deg": 40.0, "score_per_deg": 2.0})
            dialog.dialog.deleteLater()


if __name__ == "__main__":
    unittest.main()
