"""2026-11-16: 모듈 세부 설정 (VIO/Basalt, 위치 융합, GNSS, OAK·점군, 인지).

Board scripts run locally with HOME = a temporary folder shaped like the Pi5/Jetson home
(no board is contacted). The start-script parts are run from the shipped remote/*.sh text."""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
REMOTE = ROOT / "remote"
sys.path.insert(0, str(ROOT))
import board_settings as bs
import module_params as mp

FUSION_YAML = """fusion:
  ros__parameters:
    vio_topic: /basalt/odom
    base_to_imu_translation_m: [0.0, 0.0, 0.0]
    base_to_imu_extrinsics_confirmed: false
    publish_hz: 25.0
    solve_hz: 2.0
    max_iterations: 10
    gnss_fusion_mode: drift_correction
"""
BASALT = {"value0": {"config.optical_flow_type": "frame_to_frame", "config.optical_flow_detection_grid_size": 40,
                     "config.optical_flow_max_recovered_dist2": 0.04, "config.optical_flow_levels": 3,
                     "config.optical_flow_skip_frames": 1, "config.optical_flow_epipolar_error": 0.005,
                     "config.vio_max_states": 3, "config.vio_max_kfs": 7, "config.vio_min_frames_after_kf": 5,
                     "config.vio_new_kf_keypoints_thresh": 0.7, "config.vio_obs_std_dev": 0.5,
                     "config.vio_obs_huber_thresh": 1.0, "config.vio_min_triangulation_dist": 0.05,
                     "config.vio_outlier_threshold": 3.0, "config.vio_filter_iteration": 4,
                     "config.vio_max_iterations": 7, "config.vio_enforce_realtime": False,
                     "config.vio_lm_lambda_min": 1e-05, "config.vio_init_pose_weight": 100000000.0,
                     "config.vio_kf_marg_feature_ratio": 0.2, "config.mapper_min_matches": 20}}


class FakeBoard:
    def __init__(self, home):
        self.home = Path(home)
        self.hosts = []

    def __call__(self, host, script, password, timeout_s=18):
        self.hosts.append(host)
        done = subprocess.run(["bash", "-s"], input=Path(script).read_text(), text=True, capture_output=True,
                              env={"PATH": os.environ["PATH"], "HOME": str(self.home)}, timeout=timeout_s)
        return {"ok": done.returncode == 0, "stdout": done.stdout, "reason": "ok" if done.returncode == 0 else
                "remote_script_failed", "returncode": done.returncode}


def pi5_home(temp, basalt=True):
    home = Path(temp) / "home"
    config = home / "nomad_ws/config"
    config.mkdir(parents=True)
    (config / "fusion_independent.yaml").write_text(FUSION_YAML)
    (config / "gnss_receiver.yaml").write_text("nomad_gnss:\n  ros__parameters:\n    port: /dev/x\n    poll_hz: 1.0\n    max_age_s: 2.0\n")
    if basalt:
        (config / "basalt_vio_config.json").write_text(json.dumps(BASALT, indent=2))
    return home


class ModuleTargetTest(unittest.TestCase):
    def test_boards(self):
        self.assertEqual({k: bs.module_board(k) for k in ("vio", "fusion", "gnss", "oak", "perception", "planner")},
                         {"vio": "pi5", "fusion": "pi5", "gnss": "pi5", "oak": "jetson", "perception": "jetson",
                          "planner": "jetson"})

    def test_launcher_uses_pi5_address_for_pi5_modules(self):
        import launcher

        class Box:
            def __init__(self, text):
                self.text = text

            def currentText(self):
                return self.text

        fake = type("Fake", (), {})()
        fake.hosts = {"pi": Box("user@10.42.0.15"), "jetson": Box("nvidia@10.42.0.16")}
        fake.password = Box("")
        fake.password.text = lambda: ""
        self.assertEqual(launcher.Launcher.board_settings_store(fake, "fusion").host, "user@10.42.0.15")
        self.assertEqual(launcher.Launcher.board_settings_store(fake, "vio").host, "user@10.42.0.15")
        self.assertEqual(launcher.Launcher.board_settings_store(fake, "perception").host, "nvidia@10.42.0.16")
        self.assertEqual(launcher.Launcher.board_settings_store(fake, "planner").host, "nvidia@10.42.0.16")

    def test_failure_text_names_the_board(self):
        self.assertIn("Pi5에 연결하지", bs.BoardStore._failure({"reason": "ssh_unreachable"}, "Pi5"))
        self.assertIn("Jetson에 연결하지", bs.BoardStore._failure({"reason": "ssh_unreachable"}))


class RosOverrideTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.home = pi5_home(self.temp.name)
        self.state = Path(self.temp.name) / "state"
        self.board = FakeBoard(self.home)

    def tearDown(self):
        self.temp.cleanup()

    def store(self, key="fusion"):
        return bs.BoardStore(key, "user@10.42.0.15", None, self.state, runner=self.board)

    def test_board_values_and_only_changed_keys_written(self):
        store = self.store()
        data = store.read()
        self.assertEqual(data["source"], "default")
        self.assertTrue(data["board_default"])
        self.assertEqual(data["values"]["fusion"]["publish_hz"], 25.0)   # board value, not the GUI copy (20)
        configs = data["values"]
        configs["fusion"]["solve_hz"] = 3.0
        configs["fusion"]["max_iterations"] = 12
        configs["fusion"]["gnss_fusion_mode"] = "window"
        store.save(configs)
        text = (self.home / "nomad_settings/fusion/overrides.yaml").read_text()
        self.assertEqual(text, "fusion:\n  ros__parameters:\n    solve_hz: 3.0\n    max_iterations: 12\n"
                               '    gnss_fusion_mode: "window"\n')
        self.assertEqual(bs.parse_ros_params(text), {"solve_hz": 3.0, "max_iterations": 12, "gnss_fusion_mode": "window"})
        # Read back: the saved table wins, board config untouched.
        again = self.store().read()
        self.assertEqual(again["source"], "current")
        self.assertEqual(again["values"]["fusion"]["solve_hz"], 3.0)
        self.assertEqual((self.home / "nomad_ws/config/fusion_independent.yaml").read_text(), FUSION_YAML)

    def test_float_keeps_decimal_point(self):
        store = self.store()
        configs = store.read()["values"]
        configs["fusion"]["publish_hz"] = 20       # an int typed in a double box
        store.save(configs)
        text = (self.home / "nomad_settings/fusion/overrides.yaml").read_text()
        self.assertIn("    publish_hz: 20.0\n", text)

    def test_unchanged_writes_no_parameters(self):
        store = self.store()
        store.save(store.read()["values"])
        text = (self.home / "nomad_settings/fusion/overrides.yaml").read_text()
        self.assertEqual(text, mp.NO_CHANGES)
        self.assertNotIn("ros__parameters:", text)

    def test_conflict_names_pi5(self):
        first, second = self.store(), self.store()
        first.read()
        configs = second.read()["values"]
        configs["fusion"]["solve_hz"] = 4.0
        second.save(configs)
        with self.assertRaises(bs.BoardError) as caught:
            first.save(first.module.spec.defaults())
        self.assertIn("Pi5", str(caught.exception))

    def test_gnss_and_jetson_parts(self):
        store = self.store("gnss")
        configs = store.read()["values"]
        configs["gnss"]["max_future_s"] = 0.5
        store.save(configs)
        self.assertEqual((self.home / "nomad_settings/gnss/overrides.yaml").read_text(),
                         "nomad_gnss:\n  ros__parameters:\n    max_future_s: 0.5\n")
        oak = self.store("oak")
        data = oak.read()
        self.assertFalse(data["board_default"])   # no oak_cpp.yaml in this fake home: GUI defaults
        configs = data["values"]
        configs["cloud"]["max_depth_m"] = 6.0
        oak.save(configs)
        self.assertEqual((self.home / "nomad_settings/oak/overrides.yaml").read_text(), mp.NO_CHANGES)
        self.assertEqual((self.home / "nomad_settings/cloud/overrides.yaml").read_text(),
                         "nomad_external_stereo_cloud:\n  ros__parameters:\n    max_depth_m: 6.0\n")
        perception = self.store("perception")
        configs = perception.read()["values"]
        configs["semantic"]["rate_hz"] = 2.0
        perception.save(configs)
        self.assertEqual((self.home / "nomad_settings/perception/overrides.yaml").read_text(),
                         "/**:\n  ros__parameters:\n    rate_hz: 2.0\n")

    def test_cloud_rules(self):
        spec = mp.SPECS["oak"]
        for key, value in (("num_disparities", 100), ("block_size", 8), ("max_depth_m", 0.1)):
            configs = spec.defaults()
            configs["cloud"][key] = value
            if key == "max_depth_m":
                configs["cloud"]["min_depth_m"] = 1.0
                configs["cloud"]["max_depth_m"] = 1.0
            with self.assertRaises(ValueError):
                spec.validate(configs)

    def test_ranges_inside_board_node_checks(self):
        # fusion_node.cpp: these must be > 0; receiver_node.cpp: max_future_s in [0, 1].
        docs = mp.SPECS["fusion"].docs()["fusion"]
        for key in ("gps_jump_margin_m", "gps_reacquire_s", "vio_jump_margin_m", "vio_recovery_hold_s"):
            self.assertGreater(docs[key]["min"], 0.0, key)
            configs = mp.SPECS["fusion"].defaults()
            configs["fusion"][key] = 0.0
            with self.assertRaises(ValueError):
                mp.SPECS["fusion"].validate(configs)
        gnss = mp.SPECS["gnss"].docs()["gnss"]["max_future_s"]
        self.assertLessEqual(gnss["max"], 1.0)
        self.assertGreaterEqual(gnss["min"], 0.0)

    def test_fusion_window_limit(self):
        # fusion_node.cpp: window_s * keyframe_hz > 500 -> "Invalid window or integer limit".
        spec = mp.SPECS["fusion"]
        configs = spec.defaults()
        configs["fusion"]["window_s"], configs["fusion"]["keyframe_hz"] = 30.0, 20.0
        with self.assertRaisesRegex(ValueError, "500"):
            spec.validate(configs)
        configs["fusion"]["window_s"], configs["fusion"]["keyframe_hz"] = 25.0, 20.0
        self.assertEqual(spec.validate(configs)["fusion"]["window_s"], 25.0)
        self.assertEqual(spec.validate(spec.defaults()), spec.defaults())

    def test_no_forbidden_keys(self):
        forbidden = re.compile(r"topic|frame_id|_frame$|confirmed|calib|^port$|baud|quaternion|antenna|imu_rate|^fps$|initial_source|output_hz")
        for key, spec in mp.SPECS.items():
            for part, _label in spec.parts:
                defaults = spec.defaults()[part]
                docs = spec.docs()[part]
                self.assertEqual(set(defaults), set(docs), key)
                for name in defaults:
                    self.assertIsNone(forbidden.search(name), name)
                    self.assertEqual(docs[name]["default"], defaults[name])
                    self.assertNotIsInstance(defaults[name], list)


class BasaltTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.state = Path(self.temp.name) / "state"

    def tearDown(self):
        self.temp.cleanup()

    def test_full_copy_with_only_edited_key(self):
        home = pi5_home(self.temp.name)
        store = bs.BoardStore("vio", "user@10.42.0.15", None, self.state, runner=FakeBoard(home))
        configs = store.read()["values"]
        configs["vio"]["vio_max_kfs"] = 9
        configs["vio"]["vio_obs_std_dev"] = 0.8
        store.save(configs)
        saved_text = (home / "nomad_settings/vio/basalt_vio_config.json").read_text()
        saved = json.loads(saved_text)
        self.assertEqual(list(saved["value0"]), list(BASALT["value0"]))   # same keys, same order
        expected = dict(BASALT["value0"], **{"config.vio_max_kfs": 9, "config.vio_obs_std_dev": 0.8})
        self.assertEqual(saved["value0"], expected)
        self.assertIsInstance(saved["value0"]["config.vio_max_kfs"], int)
        self.assertIsInstance(saved["value0"]["config.vio_lm_lambda_min"], float)
        self.assertIn('"config.vio_init_pose_weight": 100000000.0', saved_text)
        self.assertIn('"config.vio_obs_huber_thresh": 1.0', saved_text)
        self.assertEqual(json.loads((home / "nomad_ws/config/basalt_vio_config.json").read_text()), BASALT)

    def test_unchanged_is_empty_object(self):
        home = pi5_home(self.temp.name)
        store = bs.BoardStore("vio", "user@10.42.0.15", None, self.state, runner=FakeBoard(home))
        store.save(store.read()["values"])
        self.assertEqual((home / "nomad_settings/vio/basalt_vio_config.json").read_text(), "{}\n")

    def test_change_refused_without_board_file(self):
        home = pi5_home(self.temp.name, basalt=False)
        store = bs.BoardStore("vio", "user@10.42.0.15", None, self.state, runner=FakeBoard(home))
        configs = store.read()["values"]
        configs["vio"]["vio_max_kfs"] = 9
        with self.assertRaises(bs.BoardError):
            store.save(configs)
        self.assertFalse((home / "nomad_settings/vio/basalt_vio_config.json").exists())


class HelperTest(unittest.TestCase):
    def test_yaml_values(self):
        self.assertEqual(mp.yaml_value(20.0), "20.0")
        self.assertEqual(mp.yaml_value(1e-05), "1e-05")
        self.assertEqual(mp.yaml_value(3), "3")
        self.assertEqual(mp.yaml_value(True), "true")
        self.assertEqual(mp.yaml_value("window"), '"window"')

    def test_start_notes(self):
        self.assertEqual(mp.start_notes("ROLE=pi5\n"), [])
        self.assertEqual(mp.start_notes("VIO_SETTINGS=unsupported\nROLE=pi5\n"), [mp.VIO_UNSUPPORTED])
        self.assertEqual(mp.VIO_UNSUPPORTED, "VIO 설정 미적용: Pi5 실행 스크립트 미지원")
        self.assertEqual(mp.start_notes("MODULE_SETTINGS applied=gnss\n", "MODULE_SETTINGS applied=cloud\n",
                                        "MODULE_SETTINGS applied=fusion\nMODULE_SETTINGS invalid=oak\n"),
                         ["모듈 세부 설정 적용: GNSS, OAK 점군, 위치 융합", "OAK 설정 파일 형식 오류: 보드 기본값으로 실행"])

    def test_startup_reports_unsupported_vio(self):
        text = (ROOT / "startup.py").read_text(encoding="utf-8")
        self.assertIn('module_params.start_notes(stages["preflight"]["pi5"]', text)
        self.assertIn("module_params.VIO_UNSUPPORTED", text)


def _function(script: str) -> str:
    text = (REMOTE / script).read_text()
    match = re.search(r"^nomad_module_override\(\) \{\n.*?^\}\n", text, re.S | re.M)
    return match.group(0)


class ScriptTest(unittest.TestCase):
    SCRIPTS = ("start_pi_vio_fusion.sh", "start_pi_gnss.sh", "start_jetson.sh", "restart_vio.sh", "preflight_pi.sh")

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.home = Path(self.temp.name) / "home"
        self.run = Path(self.temp.name) / "run"
        self.run.mkdir(parents=True)
        self.home.mkdir()

    def tearDown(self):
        self.temp.cleanup()

    def bash(self, text):
        return subprocess.run(["bash", "-c", text], text=True, capture_output=True,
                              env={"PATH": os.environ["PATH"], "HOME": str(self.home), "run": str(self.run)}, timeout=20)

    def put(self, module, text):
        folder = self.home / "nomad_settings" / module
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "overrides.yaml").write_text(text)

    def test_syntax(self):
        for script in self.SCRIPTS:
            done = subprocess.run(["bash", "-n", str(REMOTE / script)], capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, script + done.stderr)

    def test_same_helper_everywhere(self):
        self.assertEqual(_function("start_pi_gnss.sh"), _function("start_pi_vio_fusion.sh"))
        self.assertEqual(_function("start_pi_gnss.sh"), _function("start_jetson.sh"))

    def test_helper(self):
        helper = _function("start_pi_gnss.sh")
        call = helper + 'nomad_module_override gnss nomad_gnss "$run/out.yaml"; echo "rc=$?"\n'
        self.assertEqual(self.bash(call).stdout, "rc=1\n")                 # no file
        self.put("gnss", mp.NO_CHANGES)
        self.assertEqual(self.bash(call).stdout, "rc=1\n")                 # no changes
        self.assertFalse((self.run / "out.yaml").exists())
        self.put("gnss", "fusion:\n  ros__parameters:\n    poll_hz: 2.0\n")
        self.assertEqual(self.bash(call).stdout, "MODULE_SETTINGS invalid=gnss\nrc=1\n")
        self.put("gnss", "nomad_gnss:\n  ros__parameters:\n    poll_hz: 2.0\n")
        self.assertEqual(self.bash(call).stdout, "MODULE_SETTINGS applied=gnss\nrc=0\n")
        self.assertEqual((self.run / "out.yaml").read_text(), "nomad_gnss:\n  ros__parameters:\n    poll_hz: 2.0\n")

    def jetson_args(self):
        text = (REMOTE / "start_jetson.sh").read_text()
        start = text.index("nomad_module_override() {")
        end = text.index('setsid env ROS_LOG_DIR="$run/oak/ros"')
        out = self.bash(text[start:end] + 'printf "oak=%s|" "${oak_args[*]}"; printf "cloud=%s|" "${cloud_args[*]}"; '
                                          'printf "semantic=%s" "${semantic_args[*]}"\n')
        return out.stdout.splitlines()[-1]

    def test_jetson_unchanged_without_settings(self):
        self.assertEqual(self.jetson_args(), "oak=|cloud=|semantic=-p rate_hz:=1.0")
        text = (REMOTE / "start_jetson.sh").read_text()
        self.assertIn('-p factory_calibration_json:="$root/config/factory_eeprom.json" "${oak_args[@]}" 9>&-', text)
        self.assertIn('"${cloud_args[@]}" "${cloud_extra[@]}"', text)
        self.assertIn('start_module semantic bash "$ws/semantic.sh" --indoor "${semantic_args[@]}"', text)
        self.assertNotIn("semantic.sh\" -p rate_hz:=1.0", text)

    def test_jetson_with_settings(self):
        self.put("perception", "/**:\n  ros__parameters:\n    rate_hz: 2.0\n")
        self.put("cloud", "nomad_external_stereo_cloud:\n  ros__parameters:\n    max_depth_m: 6.0\n")
        self.put("oak", mp.NO_CHANGES)
        run = str(self.run)
        self.assertEqual(self.jetson_args(),
                         f"oak=|cloud=--params-file {run}/cloud_overrides.yaml|semantic=--params-file "
                         f"{run}/perception_params.yaml --params-file {run}/perception_overrides.yaml")
        self.assertEqual((self.run / "perception_params.yaml").read_text(), "/**:\n  ros__parameters:\n    rate_hz: 1.0\n")

    def test_pi5_launch_lines(self):
        text = (REMOTE / "start_pi_vio_fusion.sh").read_text()
        self.assertIn('bash "$ws/scripts/run_fusion.sh" "${fusion_args[@]}" </dev/null', text)
        self.assertLess(text.index("export NOMAD_BASALT_VIO_CONFIG"), text.index('bash "$ws/scripts/run_basalt_cpp.sh"'))
        self.assertLess(text.index("export NOMAD_BASALT_VIO_CONFIG"), text.index("vio_supervisor"))
        self.assertIn('bash "$ws/scripts/run_gnss.sh" "${gnss_args[@]}" </dev/null', (REMOTE / "start_pi_gnss.sh").read_text())
        restart = (REMOTE / "restart_vio.sh").read_text()
        self.assertLess(restart.index('export NOMAD_BASALT_VIO_CONFIG="$run/basalt_vio_config.json"'),
                        restart.index('setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_basalt_cpp.sh"'))

    def test_preflight_vio_support(self):
        text = (REMOTE / "preflight_pi.sh").read_text()
        block = text[text.index('vio_override="$HOME'):text.index("printf 'ROLE=pi5")]
        scripts = self.home / "nomad_ws/scripts"
        scripts.mkdir(parents=True)
        (scripts / "run_basalt_cpp.sh").write_text('-p "vio_config_file:=$config/basalt_vio_config.json"\n')
        probe = f'ws="{self.home}/nomad_ws"\n' + block
        self.assertEqual(self.bash(probe).stdout, "")                      # no saved VIO settings
        folder = self.home / "nomad_settings/vio"
        folder.mkdir(parents=True)
        (folder / "basalt_vio_config.json").write_text("{}\n")
        self.assertEqual(self.bash(probe).stdout, "")                      # saved but unchanged
        (folder / "basalt_vio_config.json").write_text(json.dumps(BASALT))
        self.assertEqual(self.bash(probe).stdout, "VIO_SETTINGS=unsupported\n")
        (scripts / "run_basalt_cpp.sh").write_text('-p "vio_config_file:=${NOMAD_BASALT_VIO_CONFIG:-$config/x}"\n')
        self.assertEqual(self.bash(probe).stdout, "VIO_SETTINGS=supported\n")

        (folder / "basalt_vio_config.json").unlink()
        (folder / "basalt_vio_config.json").symlink_to(folder / "elsewhere.json")
        (folder / "elsewhere.json").write_text(json.dumps(BASALT))
        self.assertEqual(self.bash(probe).stdout, "")                      # link ignored, as in start

    def vio_block(self):
        text = (REMOTE / "start_pi_vio_fusion.sh").read_text()
        block = text[text.index('vio_override="$HOME'):text.index('setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_basalt_cpp.sh"')]
        return f'ws="{self.home}/nomad_ws"\n' + block + 'echo "env=${NOMAD_BASALT_VIO_CONFIG:-}"\n'

    def test_vio_copy_rebuilt_from_board_file(self):
        config = self.home / "nomad_ws/config"
        scripts = self.home / "nomad_ws/scripts"
        config.mkdir(parents=True)
        scripts.mkdir(parents=True)
        (scripts / "run_basalt_cpp.sh").write_text('${NOMAD_BASALT_VIO_CONFIG:-x}\n')
        self.assertEqual(self.bash(self.vio_block()).stdout, "env=\n")       # nothing saved
        board = json.loads(json.dumps(BASALT))
        board["value0"]["config.mapper_min_matches"] = 33                   # later board edit, not a GUI key
        (config / "basalt_vio_config.json").write_text(json.dumps(board))
        saved = json.loads(json.dumps(BASALT))
        saved["value0"]["config.vio_max_kfs"] = 9
        folder = self.home / "nomad_settings/vio"
        folder.mkdir(parents=True)
        (folder / "basalt_vio_config.json").write_text(json.dumps(saved))
        out = self.bash(self.vio_block()).stdout
        self.assertEqual(out, f"MODULE_SETTINGS applied=vio\nenv={self.run}/basalt_vio_config.json\n")
        merged = json.loads((self.run / "basalt_vio_config.json").read_text())["value0"]
        self.assertEqual(merged["config.vio_max_kfs"], 9)
        self.assertEqual(merged["config.mapper_min_matches"], 33)
        self.assertIsInstance(merged["config.vio_obs_huber_thresh"], float)
        expected = dict(board["value0"], **{"config.vio_max_kfs": 9})
        self.assertEqual(merged, expected)
        # merge failure (board file unreadable) -> no env, clear line, Start continues
        (config / "basalt_vio_config.json").unlink()
        out = self.bash(self.vio_block()).stdout
        self.assertIn("VIO_SETTINGS_IGNORED", out)
        self.assertTrue(out.endswith("env=\n"))
        self.assertFalse((self.run / "basalt_vio_config.json").exists())

    def test_vio_merge_keys_match_gui(self):
        text = (REMOTE / "start_pi_vio_fusion.sh").read_text()
        keys = set(re.findall(r'"((?:optical_flow|vio)_[a-z0-9_]+)"', text[text.index("KEYS = ("):text.index("with open")]))
        self.assertEqual(keys, set(mp.SPECS["vio"].defaults()["vio"]))


if __name__ == "__main__":
    unittest.main()
