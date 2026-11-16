"""2026-11-15 'OAK 점군: OpenCV SGBM 3WAY' start option (offline only; never contacts boards)."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import trial_settings


class Cloud3wayOptionTests(unittest.TestCase):
    def test_choice_passes_to_startup(self):
        values = trial_settings.validate(dict(cloud_source="opencv3way"))
        self.assertEqual(values["cloud_source"], "opencv3way")
        args = trial_settings.start_arguments(values)
        self.assertEqual(args[args.index("--cloud-source") + 1], "opencv3way")
        src = (ROOT / "startup.py").read_text(encoding="utf-8")
        self.assertIn('choices=("opencv", "device", "opencv3way")', src)

    def test_jetson_start_maps_3way_and_refuses_unpatched_cloud(self):
        start = (ROOT / "remote/start_jetson.sh").read_text(encoding="utf-8")
        self.assertIn("opencv|device|opencv3way", start)
        self.assertIn("cloud_extra=(-p sgbm_mode:=3way -p pad_left:=true -p num_disparities:=128)", start)
        self.assertIn('-p initial_source:="$cloud_initial"', start)
        self.assertIn("grep -aq 'sgbm_mode'", start)
        subprocess.run(["bash", "-n", str(ROOT / "remote/start_jetson.sh")], check=True)

    def test_runtime_switch_names_unchanged(self):
        import cloud_control
        self.assertNotIn("opencv3way", cloud_control.SOURCE_NAMES)


if __name__ == "__main__":
    unittest.main()
