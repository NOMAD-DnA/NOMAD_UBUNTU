"""2026-11-16: class-boundary px and initial safe area for terrain / mapper3 only.

Layout after the user's correction ("기존 매퍼설정에 그냥 넣으라고"): terrain values are in the existing
'기존 매퍼 개선 설정' dialog (terrain.json "gui_overrides"), mapper3 has its own '새 매퍼3 v3 설정' dialog
(mapper_settings.py, ~/nomad_settings/mapper_overrides/current.json), no separate '매퍼 설정' window.

Offline: the Jetson is a temporary HOME run with bash; the JSON patch program embedded in
remote/start_jetson.sh runs with the local python3 on copies of the board config files."""
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
import mapper_settings as ms
import terrain_settings as ts
import trial_settings

START = ROOT / "remote/start_jetson.sh"
BOARD_SEMANTIC = Path("/home/user/Documents/ChatGPT/lig 프로젝트/관련자료/보드코드_20261114/jetson/nomad_ws/tools/"
                      "visual_trial_20261106/runtime-map-preview/share/nomad_costmap_cpp/config/semantic.real.json")
MAPPER3_SAMPLE = {"_comment": "v3 샘플", "grid": {"resolution": 0.05, "size": 400},
                  "lidar": {"enable": True, "footprint_clear_radius": 0.25, "start_free_radius_m": 1.0},
                  "semantic": {"enable": True}}
SEMANTIC_SAMPLE = {"topics": {"a": "/x"}, "boundary_margin_px": 6,
                   "local_ground": {"enabled": True, "radius_m": 3.0},
                   "startup_free_region": {"enabled": True, "radius_m": 2.0, "field_of_view_deg": 360.0},
                   "moving_front_free_region": {"enabled": True, "radius_m": 0.5}}


def program() -> str:
    text = START.read_text(encoding="utf-8")
    body = text.split("nomad_mapper_override_py=$(cat <<'NOMAD_MAPPER_PY'\n", 1)[1]
    return body.split("\nNOMAD_MAPPER_PY\n", 1)[0]


def run(*args):
    return subprocess.run([sys.executable, "-I", "-c", program(), *map(str, args)], capture_output=True, text=True)


class FakeJetson:
    def __init__(self, home):
        self.home = Path(home)

    def __call__(self, host, script, password, timeout_s=18):
        done = subprocess.run(["bash", "-s"], input=Path(script).read_text(), text=True, capture_output=True,
                              env={"PATH": os.environ["PATH"], "HOME": str(self.home)}, timeout=timeout_s)
        return {"ok": done.returncode == 0, "stdout": done.stdout,
                "reason": "ok" if done.returncode == 0 else "remote_script_failed", "returncode": done.returncode}


class SpecTests(unittest.TestCase):
    def test_defaults_are_board_values(self):
        self.assertEqual(ms.defaults(), {"mapper3": {"erode_px": None, "start_free_radius_m": None,
                                                     "slope_k": None}})   # slope_k: 2026-11-16 (slope_k.py)
        self.assertEqual(ts.defaults()["terrain"]["gui_overrides"], dict(ts.MOVING_DEFAULTS, **{"boundary_margin_px": None, "startup_radius_m": None}))
        from module_settings import leaves
        for part, _label in ms.PARTS:
            self.assertEqual(set(leaves(ms.defaults()[part])), set(ms.SPEC.docs()[part]))
        self.assertFalse((ROOT / "mapper_config/terrain.json").exists())

    def test_ranges_and_int_px(self):
        ok = ms.validate({"terrain": {"boundary_margin_px": 6}, "mapper3": {"erode_px": 3.0}})
        self.assertNotIn("terrain", ok)          # an old terrain section is dropped
        self.assertEqual(ok["mapper3"]["erode_px"], 3)
        self.assertIsInstance(ok["mapper3"]["erode_px"], int)
        self.assertIsNone(ok["mapper3"]["start_free_radius_m"])
        for bad in ({"mapper3": {"erode_px": 6.5}}, {"mapper3": {"erode_px": 21}},
                    {"mapper3": {"start_free_radius_m": 5.5}}, {"mapper3": {"erode_px": -1}},
                    {"mapper3": {"erode_px": True}}):
            with self.assertRaises(ValueError):
                ms.validate(bad)
        t = ts.validate({"terrain": {"gui_overrides": {"boundary_margin_px": 6.0, "startup_radius_m": 1}}})
        self.assertEqual(t["terrain"]["gui_overrides"], dict(ts.MOVING_DEFAULTS, **{"boundary_margin_px": 6, "startup_radius_m": 1.0}))
        self.assertIsInstance(t["terrain"]["gui_overrides"]["boundary_margin_px"], int)
        for bad in ({"boundary_margin_px": 6.5}, {"boundary_margin_px": 21}, {"startup_radius_m": 5.5},
                    {"startup_radius_m": -0.1}, {"boundary_margin_px": True}):
            with self.assertRaises(ValueError):
                ts.validate({"terrain": {"gui_overrides": bad}})

    def test_summary_and_start_note(self):
        configs = ms.validate({"mapper3": {"erode_px": 6}})
        self.assertEqual(ms.summary(configs, "mapper3"), "경계 6px · 초기영역 보드값")
        self.assertEqual(ms.summary(configs, "terrain"), "")
        self.assertEqual(ms.summary(configs, "legacy"), "")
        self.assertEqual(ts.gui_summary(ts.validate({"terrain": {"gui_overrides": {"boundary_margin_px": 6}}})),
                         "경계 6px · 초기영역 보드값")
        self.assertEqual(ts.gui_summary(ts.defaults()), "경계 보드값 · 초기영역 보드값")
        note = ms.start_note("x\nMAPPER_SETTINGS kind=terrain boundary_margin_px=6 startup_radius_m=2.0 "
                             "overridden=boundary_margin_px\nRUN=/r\n")
        self.assertEqual(note, "매퍼 설정: 클래스 경계 6px · 초기 안전영역 2.0 m (GUI 값 적용)")
        note = ms.start_note("MAPPER_SETTINGS kind=mapper3 erode_px=- start_free_radius_m=1.0 overridden=none")
        self.assertEqual(note, "매퍼 설정: 클래스 경계 파일 기본 · 초기 안전영역 1.0 m (보드 파일 값)")
        self.assertEqual(ms.start_note("RUN=/r\nMAPPER=legacy"), "")

    def test_terrain_default_upload_unchanged(self):
        # defaults write exactly the terrain.json keys of before (no gui_overrides object)
        cfg = ts.board_json(ts.defaults())
        self.assertNotIn("gui_overrides", cfg)
        self.assertNotIn("gui_overrides", ts.upload_script(ts.defaults(), if_missing=True))
        files = bs.MODULES["terrain"]().current_files(ts.defaults())
        self.assertNotIn("gui_overrides", files[0])
        board_pkg = json.loads((ROOT / "terrain_config/terrain.json").read_text())
        for key in ts.SLOPE_KEYS + (ts.GUI_KEY,):
            board_pkg.pop(key)
        self.assertEqual(cfg, board_pkg)
        one = ts.board_json({"terrain": {"gui_overrides": {"startup_radius_m": 1.5}}})
        self.assertEqual(one["gui_overrides"], {"boundary_margin_px": None, "startup_radius_m": 1.5})

    def test_mapper_choice_unchanged(self):
        self.assertEqual(trial_settings.defaults()["mapper"], "legacy")
        self.assertEqual(set(trial_settings.SETTINGS["mapper"][1]), {"legacy", "terrain", "mapper2", "mapper3"})
        self.assertNotIn("--mapper", trial_settings.start_arguments(trial_settings.defaults()))
        self.assertEqual(trial_settings.start_arguments(dict(mapper="terrain"))[-2:], ["--mapper", "terrain"])


class BoardStoreTests(unittest.TestCase):
    def test_read_missing_then_save_and_read_back(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp) / "home"
            home.mkdir()
            store = bs.BoardStore("mapper_overrides", "nvidia@x", None, Path(temp) / "state", runner=FakeJetson(home))
            self.assertEqual(store.module.current_paths, ["nomad_settings/mapper_overrides/current.json"])
            data = store.read()
            self.assertEqual(data["source"], "default")
            self.assertEqual(data["values"], ms.defaults())
            store.save({"mapper3": {"erode_px": None, "start_free_radius_m": 1.5}})
            saved = json.loads((home / ms.REMOTE_PATH).read_text())
            self.assertEqual(saved, {"mapper3": {"erode_px": None, "start_free_radius_m": 1.5, "slope_k": None}})
            again = bs.BoardStore("mapper_overrides", "nvidia@x", None, Path(temp) / "state", runner=FakeJetson(home)).read()
            self.assertEqual(again["source"], "current")
            self.assertEqual(again["values"]["mapper3"]["start_free_radius_m"], 1.5)
            self.assertEqual(ms.load(Path(temp) / "state")["mapper3"]["start_free_radius_m"], 1.5)   # offline cache

    def test_old_file_with_terrain_section_still_reads(self):
        # backward compatibility: a file written by the earlier combined window
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp) / "home"
            (home / "nomad_settings/mapper_overrides").mkdir(parents=True)
            (home / ms.REMOTE_PATH).write_text(json.dumps(
                {"terrain": {"boundary_margin_px": 6, "startup_radius_m": None},
                 "mapper3": {"erode_px": 4, "start_free_radius_m": None, "slope_k": None}}))
            store = bs.BoardStore("mapper_overrides", "nvidia@x", None, Path(temp) / "state", runner=FakeJetson(home))
            data = store.read()
            self.assertEqual(data["values"], {"mapper3": {"erode_px": 4, "start_free_radius_m": None, "slope_k": None}})
            self.assertEqual(data["problems"], [])
            store.save(data["values"])
            self.assertNotIn("terrain", json.loads((home / ms.REMOTE_PATH).read_text()))

    def test_terrain_values_saved_in_terrain_json(self):
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp) / "home"
            home.mkdir()
            store = bs.BoardStore("terrain", "nvidia@x", None, Path(temp) / "state", runner=FakeJetson(home))
            data = store.read()
            self.assertEqual(data["values"]["terrain"]["gui_overrides"], dict(ts.MOVING_DEFAULTS, **{"boundary_margin_px": None, "startup_radius_m": None}))
            store.save(data["values"])
            saved = json.loads((home / ts.REMOTE_DIR / "terrain.json").read_text())
            self.assertNotIn("gui_overrides", saved)
            values = data["values"]
            values["terrain"]["gui_overrides"]["boundary_margin_px"] = 9.0
            store.save(values)
            saved = json.loads((home / ts.REMOTE_DIR / "terrain.json").read_text())
            self.assertEqual(saved["gui_overrides"], {"boundary_margin_px": 9, "startup_radius_m": None})
            again = bs.BoardStore("terrain", "nvidia@x", None, Path(temp) / "state", runner=FakeJetson(home)).read()
            self.assertEqual(again["values"]["terrain"]["gui_overrides"]["boundary_margin_px"], 9)
            self.assertEqual(again["problems"], [])


class PatchProgramTests(unittest.TestCase):
    def write(self, folder, name, data, indent=2):
        path = Path(folder) / name
        path.write_text(json.dumps(data, ensure_ascii=False, indent=indent) + "\n", encoding="utf-8")
        return path

    def test_board_semantic_copy_only_two_values_change(self):
        with tempfile.TemporaryDirectory() as temp:
            if BOARD_SEMANTIC.exists():
                target = Path(temp) / "semantic.real.json"
                target.write_bytes(BOARD_SEMANTIC.read_bytes())
            else:
                target = self.write(temp, "semantic.real.json", SEMANTIC_SAMPLE)
            before_text = target.read_text(encoding="utf-8")
            before = json.loads(before_text)
            overrides = self.write(temp, "ov.json", {"gui_overrides": {"boundary_margin_px": 9, "startup_radius_m": 1.5},
                                                     "mapper3": {"erode_px": 4, "start_free_radius_m": None}})
            self.assertEqual(run("check", "terrain", overrides).returncode, 0)
            done = run("apply", "terrain", overrides, target)
            self.assertEqual(done.returncode, 0, done.stdout)
            self.assertIn("MAPPER_SETTINGS kind=terrain boundary_margin_px=9 startup_radius_m=1.5 "
                          "overridden=boundary_margin_px,startup_radius_m", done.stdout)
            after = json.loads(target.read_text(encoding="utf-8"))
            expected = json.loads(before_text)
            expected["boundary_margin_px"] = 9
            expected["startup_free_region"]["radius_m"] = 1.5
            self.assertEqual(after, expected)
            # the other two radius_m blocks are untouched
            self.assertEqual(after["local_ground"]["radius_m"], before["local_ground"]["radius_m"])
            self.assertEqual(after["moving_front_free_region"]["radius_m"], before["moving_front_free_region"]["radius_m"])
            changed = [(a, b) for a, b in zip(before_text.splitlines(), target.read_text(encoding="utf-8").splitlines()) if a != b]
            self.assertEqual(len(changed), 2)

    def test_mapper3_sample(self):
        with tempfile.TemporaryDirectory() as temp:
            target = self.write(temp, "mapper3.json", MAPPER3_SAMPLE, indent=1)
            overrides = self.write(temp, "ov.json", {"mapper3": {"erode_px": 5, "start_free_radius_m": 0.5}})
            done = run("apply", "mapper3", overrides, target)
            self.assertEqual(done.returncode, 0, done.stdout)
            after = json.loads(target.read_text(encoding="utf-8"))
            expected = json.loads(json.dumps(MAPPER3_SAMPLE))
            expected["lidar"]["start_free_radius_m"] = 0.5
            expected["sem_fit"] = {"erode_px": 5}       # group added when the board file has none
            self.assertEqual(after, expected)
            self.assertIn('\n "grid"', target.read_text(encoding="utf-8"))   # board indent (1) kept
            self.assertIn("erode_px=5 start_free_radius_m=0.5 overridden=erode_px,start_free_radius_m", done.stdout)

    def test_empty_overrides_leave_files_byte_identical(self):
        with tempfile.TemporaryDirectory() as temp:
            m3 = self.write(temp, "mapper3.json", MAPPER3_SAMPLE, indent=1)
            sem = self.write(temp, "semantic.real.json", SEMANTIC_SAMPLE)
            raw_m3, raw_sem = m3.read_bytes(), sem.read_bytes()
            missing = Path(temp) / "none.json"
            nulls = self.write(temp, "nulls.json", dict(ms.defaults(), **ts.defaults()["terrain"]))
            old_terrain = self.write(temp, "old.json", {"terrain": {"boundary_margin_px": 6}})   # earlier window: ignored
            for overrides in (missing, nulls, old_terrain):
                self.assertEqual(run("check", "terrain", overrides).returncode, 1)
                self.assertEqual(run("check", "mapper3", overrides).returncode, 1)
                done = run("apply", "mapper3", overrides, m3)
                self.assertEqual(done.returncode, 0)
                self.assertIn("erode_px=- start_free_radius_m=1.0 overridden=none", done.stdout)
            shown = run("show", "terrain", sem)
            self.assertIn("boundary_margin_px=6 startup_radius_m=2.0 overridden=none", shown.stdout)
            self.assertEqual(m3.read_bytes(), raw_m3)
            self.assertEqual(sem.read_bytes(), raw_sem)
            self.assertEqual(sorted(p.name for p in Path(temp).iterdir()), ["mapper3.json", "nulls.json", "old.json", "semantic.real.json"])

    def test_invalid_values_refused(self):
        with tempfile.TemporaryDirectory() as temp:
            sem = self.write(temp, "semantic.real.json", SEMANTIC_SAMPLE)
            raw = sem.read_bytes()
            for bad in ({"gui_overrides": {"boundary_margin_px": 6.5}}, {"gui_overrides": {"boundary_margin_px": 30}},
                        {"gui_overrides": {"startup_radius_m": 9}}, {"gui_overrides": {"startup_radius_m": "2"}},
                        {"gui_overrides": 3}):
                overrides = self.write(temp, "ov.json", bad)
                self.assertEqual(run("check", "terrain", overrides).returncode, 2, bad)
                self.assertEqual(run("apply", "terrain", overrides, sem).returncode, 2, bad)
            (Path(temp) / "broken.json").write_text("{")
            self.assertEqual(run("check", "terrain", Path(temp) / "broken.json").returncode, 2)
            self.assertEqual(sem.read_bytes(), raw)

    def test_terrain_missing_key_is_an_error(self):
        with tempfile.TemporaryDirectory() as temp:
            sem = self.write(temp, "semantic.real.json", {"boundary_margin_px": 2})
            overrides = self.write(temp, "ov.json", {"gui_overrides": {"startup_radius_m": 1.0}})
            self.assertEqual(run("apply", "terrain", overrides, sem).returncode, 2)

    def test_program_runs_through_bash_definition(self):
        text = START.read_text(encoding="utf-8")
        start = text.index('mapper_overrides="$HOME/nomad_settings/mapper_overrides/current.json"')
        end = text.index("\n", text.index("nomad_mapper_override() {"))
        with tempfile.TemporaryDirectory() as temp:
            home = Path(temp)
            (home / "nomad_settings/mapper_overrides").mkdir(parents=True)
            (home / ms.REMOTE_PATH).write_text(json.dumps({"mapper3": {"erode_px": 2}}))
            target = self.write(temp, "mapper3.json", MAPPER3_SAMPLE, indent=1)
            script = ("set -eo pipefail\n" + text[start:end] + "\n" +
                      f'nomad_mapper_override apply mapper3 "$mapper_overrides" {target}\n'
                      'rc=0; nomad_mapper_override check terrain "$mapper_overrides" || rc=$?; echo "TERRAIN_RC=$rc"\n')
            done = subprocess.run(["bash", "-s"], input=script, text=True, capture_output=True,
                                  env={"PATH": os.environ["PATH"], "HOME": str(home)})
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
            self.assertIn("overridden=erode_px", done.stdout)
            self.assertIn("TERRAIN_RC=1", done.stdout)
            self.assertEqual(json.loads(target.read_text())["sem_fit"], {"erode_px": 2})


class StartScriptTests(unittest.TestCase):
    def test_syntax(self):
        subprocess.run(["bash", "-n", str(START)], check=True)

    def test_legacy_start_path_unchanged(self):
        text = START.read_text(encoding="utf-8")
        legacy = text[text.index("# Existing private native map"):]
        self.assertNotIn("mapper_override", legacy)
        self.assertNotIn("legacy_cfg", legacy)
        self.assertNotIn("gui_overrides", legacy)
        self.assertIn('NOMAD_RUNTIME_DIR="$prefix" ROS_LOG_DIR="$run/ros"', legacy)
        # the only users of the overrides are the mapper3 and terrain branches
        mapper3 = text[text.index('== mapper3 ]]; then'):text.index('== terrain ]]; then')]
        terrain = text[text.index('== terrain ]]; then'):text.index("# Existing private native map")]
        mapper2 = text[text.index('== mapper2 ]]; then'):text.index('== mapper3 ]]; then')]
        self.assertNotIn("nomad_mapper_override", mapper2)
        self.assertIn('nomad_mapper_override apply mapper3 "$mapper_overrides" "$run/mapper3_cfg/mapper3.json"', mapper3)
        self.assertIn('"$prefix/share/nomad_costmap_cpp/config" "$run/mapper3_summary.json"', mapper3)
        self.assertIn('t_legacy="$prefix/share/nomad_costmap_cpp/config"', terrain)
        self.assertIn('cp -pR -- "$t_legacy"/. "$run/terrain_legacy_cfg/"', terrain)
        self.assertIn('bash "$t_script" "$run/terrain_cfg" "$t_legacy"', terrain)
        # terrain values come from the run's terrain.json copy, not the mapper_overrides file
        self.assertIn('nomad_mapper_override check terrain "$run/terrain_cfg/terrain.json"', terrain)
        self.assertNotIn('apply terrain "$mapper_overrides"', terrain)
        # the per-run copy is made only when the check finds values (exit 0); strip before the mapper starts
        self.assertLess(terrain.index('if [[ "$override_rc" -eq 0 ]]'), terrain.index('mkdir "$run/terrain_legacy_cfg"'))
        self.assertLess(terrain.index("nomad_mapper_override strip terrain"), terrain.index("start_module terrain"))

    def precheck_block(self):
        text = START.read_text(encoding="utf-8")
        pre = text.index('nomad_mapper_override check mapper3 "$mapper_overrides" >/dev/null')
        start = text.index('case "${NOMAD_MAPPER:-legacy}" in', pre - 600)
        return text, pre, text[start:text.index("esac", pre) + 4]

    def test_overrides_checked_before_sensors_start(self):
        # review fix 2026-11-16: a broken file stops Start before any sensor/module is started
        text, pre, block = self.precheck_block()
        self.assertLess(text.index("case \"${NOMAD_MAPPER:-legacy}\" in", pre - 600), text.index('run="$ws/log/visual_trial_'))
        self.assertLess(pre, text.index("start_module "))
        self.assertIn('[[ "$override_rc" -le 1 ]] || fail', block)
        start = text.index('mapper_overrides="$HOME/nomad_settings/mapper_overrides/current.json"')
        end = text.index("\n", text.index("nomad_mapper_override() {"))
        terrain_path = "nomad_terrain_ws/active_config/terrain.json"
        cases = ((ms.REMOTE_PATH, json.dumps({"mapper3": {"erode_px": 99}}), "mapper3", False),
                 (ms.REMOTE_PATH, "{broken", "legacy", True), (None, None, "mapper3", True),
                 (None, None, "terrain", True),
                 (terrain_path, "{broken", "terrain", False),
                 (terrain_path, json.dumps({"gui_overrides": {"boundary_margin_px": 99}}), "terrain", False),
                 (terrain_path, json.dumps({"map_length_m": 20.0, "gui_overrides": {"boundary_margin_px": 4}}), "terrain", True),
                 # old combined-window file: its terrain section is not read any more
                 (ms.REMOTE_PATH, "{broken", "terrain", True))
        for rel, content, kind, ok in cases:
            with self.subTest(kind=kind, rel=rel, content=content), tempfile.TemporaryDirectory() as temp:
                home = Path(temp)
                if content is not None:
                    (home / rel).parent.mkdir(parents=True)
                    (home / rel).write_text(content)
                script = ("set -eo pipefail\nfail() { echo \"FAIL: $*\"; exit 1; }\n" + text[start:end] + "\n"
                          + block + "\necho PRECHECK_OK\n")
                done = subprocess.run(["bash", "-s"], input=script, text=True, capture_output=True,
                                      env={"PATH": os.environ["PATH"], "HOME": str(home), "NOMAD_MAPPER": kind})
                self.assertEqual("PRECHECK_OK" in done.stdout, ok, done.stdout + done.stderr)
                self.assertEqual("sensors were not started" in done.stdout, not ok)

    def run_terrain_block(self, terrain_cfg, old_overrides=None):
        """start_jetson.sh terrain gui_overrides block with a fake HOME, prefix and run folder."""
        text = START.read_text(encoding="utf-8")
        start = text.index('mapper_overrides="$HOME/nomad_settings/mapper_overrides/current.json"')
        end = text.index("\n", text.index("nomad_mapper_override() {"))
        terrain = text[text.index('== terrain ]]; then'):]
        block = terrain[terrain.index('  t_legacy="$prefix/share/nomad_costmap_cpp/config"'):
                        terrain.index("  printf 'terrain\\n' > \"$run/mapper_kind\"")]
        temp = tempfile.mkdtemp()
        self.addCleanup(__import__("shutil").rmtree, temp)
        home, prefix, run_dir = Path(temp) / "home", Path(temp) / "prefix", Path(temp) / "run"
        shared = prefix / "share/nomad_costmap_cpp/config"
        shared.mkdir(parents=True)
        (shared / "semantic.real.json").write_text(json.dumps(SEMANTIC_SAMPLE, indent=2) + "\n")
        (shared / "slope.json").write_text("{}\n")
        (run_dir / "terrain_cfg").mkdir(parents=True)
        home.mkdir()
        if terrain_cfg is not None:
            (run_dir / "terrain_cfg/terrain.json").write_text(json.dumps(terrain_cfg, indent=1) + "\n")
        if old_overrides is not None:
            (home / "nomad_settings/mapper_overrides").mkdir(parents=True)
            (home / ms.REMOTE_PATH).write_text(json.dumps(old_overrides))
        script = ("set -eo pipefail\nfail() { echo \"FAIL: $*\"; exit 1; }\n" + text[start:end] + "\n"
                  f"prefix={prefix}\nrun={run_dir}\n" + block + 'echo "T_LEGACY=$t_legacy"\necho BLOCK_OK\n')
        done = subprocess.run(["bash", "-s"], input=script, text=True, capture_output=True,
                              env={"PATH": os.environ["PATH"], "HOME": str(home)})
        return done, run_dir, shared

    def test_terrain_block_strips_gui_overrides_and_patches_copy(self):
        cfg = ts.board_json({"terrain": {"gui_overrides": {"boundary_margin_px": 9, "startup_radius_m": 1.5}}})
        done, run_dir, shared = self.run_terrain_block(cfg)
        self.assertIn("BLOCK_OK", done.stdout, done.stdout + done.stderr)
        self.assertIn(f"T_LEGACY={run_dir}/terrain_legacy_cfg", done.stdout)
        copy = json.loads((run_dir / "terrain_cfg/terrain.json").read_text())
        expected = dict(cfg)
        expected.pop("gui_overrides")
        self.assertEqual(copy, expected)                                   # binary never sees the GUI keys
        semantic = json.loads((run_dir / "terrain_legacy_cfg/semantic.real.json").read_text())
        self.assertEqual((semantic["boundary_margin_px"], semantic["startup_free_region"]["radius_m"]), (9, 1.5))
        self.assertTrue((run_dir / "terrain_legacy_cfg/slope.json").exists())
        self.assertEqual(json.loads((shared / "semantic.real.json").read_text()), SEMANTIC_SAMPLE)   # shared untouched
        self.assertEqual(ms.start_note(done.stdout), "매퍼 설정: 클래스 경계 9px · 초기 안전영역 1.5 m (GUI 값 적용)")

    def test_terrain_block_defaults_identical_to_before(self):
        cfg = ts.board_json(ts.defaults())
        done, run_dir, shared = self.run_terrain_block(cfg)
        self.assertIn("BLOCK_OK", done.stdout, done.stdout + done.stderr)
        self.assertIn(f"T_LEGACY={shared}", done.stdout)
        self.assertFalse((run_dir / "terrain_legacy_cfg").exists())
        self.assertEqual(json.loads((run_dir / "terrain_cfg/terrain.json").read_text()), cfg)
        self.assertNotIn("NOTE:", done.stdout)
        # all-null object written by hand: still removed, nothing copied
        nulls = dict(cfg, gui_overrides={"boundary_margin_px": None, "startup_radius_m": None})
        done, run_dir, shared = self.run_terrain_block(nulls)
        self.assertIn(f"T_LEGACY={shared}", done.stdout)
        self.assertEqual(json.loads((run_dir / "terrain_cfg/terrain.json").read_text()), cfg)
        # no terrain.json at all (board file missing): shared folder, no error
        done, run_dir, shared = self.run_terrain_block(None)
        self.assertIn("BLOCK_OK", done.stdout, done.stdout + done.stderr)
        self.assertIn(f"T_LEGACY={shared}", done.stdout)

    def test_terrain_block_ignores_old_window_values_with_note(self):
        cfg = ts.board_json(ts.defaults())
        done, run_dir, shared = self.run_terrain_block(cfg, {"terrain": {"boundary_margin_px": 6, "startup_radius_m": None},
                                                             "mapper3": {"erode_px": None}})
        self.assertIn("BLOCK_OK", done.stdout, done.stdout + done.stderr)
        self.assertIn(f"T_LEGACY={shared}", done.stdout)
        self.assertIn("NOTE: old terrain values", done.stdout)
        done, _, _ = self.run_terrain_block(cfg, {"terrain": {"boundary_margin_px": None}})
        self.assertNotIn("NOTE:", done.stdout)

    def test_terrain_block_invalid_values_fail(self):
        cfg = dict(ts.board_json(ts.defaults()), gui_overrides={"boundary_margin_px": 6.5})
        done, run_dir, _ = self.run_terrain_block(cfg)
        self.assertNotIn("BLOCK_OK", done.stdout)
        self.assertIn("FAIL: Invalid GUI values (gui_overrides)", done.stdout)

    def test_launcher_buttons(self):
        source = (ROOT / "launcher.py").read_text(encoding="utf-8")
        self.assertNotIn("매퍼 설정 (클래스 경계·초기 영역)", source)
        self.assertNotIn("edit_mapper_settings", source)
        self.assertIn('QtWidgets.QPushButton("새 매퍼3 v3 설정…")', source)
        order = [source.index(text) for text in ('QPushButton("새 매퍼 세부 설정…")', 'QPushButton("기존 매퍼 개선 설정…")',
                                                 'QPushButton("새 매퍼3 v3 설정…")', 'QPushButton("플래너 세부 설정…")')]
        self.assertEqual(order, sorted(order))
        body = source[source.index("    def edit_mapper3_settings(self):"):source.index("    def show_trial_settings(self):")]
        self.assertIn('board=self.board_settings_store("mapper_overrides")', body)
        self.assertNotIn("combo", body)
        summary = source[source.index("    def show_trial_settings(self):"):source.index("    def trial_settings(self, save=False):")]
        self.assertIn("terrain_settings.gui_summary(", summary)
        self.assertIn("mapper_settings.summary(", summary)

    def test_startup_reports_mapper_values(self):
        source = (ROOT / "startup.py").read_text(encoding="utf-8")
        self.assertIn('mapper_settings.start_note(jetson_sensors.get("stdout", ""))', source)


class DialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_mapper3_dialog_has_no_mapper_combo_or_terrain_tab(self):
        import inspect
        from PyQt5 import QtWidgets
        self.assertNotIn("mapper", inspect.signature(ms.SettingsDialog.__init__).parameters)
        with tempfile.TemporaryDirectory() as temp:
            dialog = ms.SettingsDialog(None, temp, board=None)
            self.assertFalse(hasattr(dialog, "mapper_combo"))
            self.assertEqual(dialog.dialog.findChildren(QtWidgets.QComboBox), [])
            self.assertEqual(dialog.tabs.count(), 1)
            self.assertEqual(dialog.tabs.tabText(0), "새 매퍼3 v3")
            self.assertIn("새 매퍼3 v3 설정", dialog.dialog.windowTitle())
            self.assertEqual({part for part, _key in dialog.widgets}, {"mapper3"})
            widget = dialog.widgets[("mapper3", "erode_px")][0]
            self.assertEqual(widget.decimals(), 0)
            self.assertEqual(dialog.values(), ms.defaults())   # empty = board values
            widget.setValue(6)
            self.assertEqual(ms.validate(dialog.values())["mapper3"]["erode_px"], 6)
            dialog.dialog.deleteLater()

    def test_terrain_dialog_shows_new_fields(self):
        with tempfile.TemporaryDirectory() as temp:
            dialog = ts.SettingsDialog(None, temp, board=None)
            px = dialog.widgets[("terrain", "gui_overrides.boundary_margin_px")][0]
            radius = dialog.widgets[("terrain", "gui_overrides.startup_radius_m")][0]
            self.assertEqual(px.decimals(), 0)
            self.assertEqual((px.maximum(), radius.maximum()), (20.0, 5.0))
            table = dialog.tabs.widget(0)
            titles = [table.item(row, 0).text() for row in range(table.rowCount())]
            self.assertIn("클래스 경계 px", titles)
            self.assertIn("초기 안전영역 반경", titles)
            # next to the slope k rows (after them, at the end of the table)
            self.assertEqual(titles.index("클래스 경계 px"), titles.index("경사 막힘 각도") + 1)
            self.assertEqual(ts.board_json(dialog.values()), ts.board_json(ts.defaults()))   # untouched = no change
            px.setValue(7)
            radius.setValue(1.5)
            out = ts.board_json(dialog.values())
            self.assertEqual(out["gui_overrides"], {"boundary_margin_px": 7, "startup_radius_m": 1.5})
            dialog.dialog.deleteLater()


if __name__ == "__main__":
    unittest.main()
