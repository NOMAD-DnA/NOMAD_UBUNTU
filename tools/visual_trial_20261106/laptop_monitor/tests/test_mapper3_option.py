"""2026-11-15 '새 매퍼3 v3' 지도 선택 연결 (offline only; never contacts boards)."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import trial_settings


class Mapper3OptionTests(unittest.TestCase):
    def test_choice_and_start_arguments(self):
        self.assertEqual(trial_settings.validate(dict(mapper="mapper3"))["mapper"], "mapper3")
        self.assertEqual(trial_settings.start_arguments(dict(mapper="mapper3"))[-2:], ["--mapper", "mapper3"])

    def test_startup_accepts_mapper3(self):
        src = (ROOT / "startup.py").read_text(encoding="utf-8")
        self.assertIn('choices=("legacy", "mapper2", "terrain", "mapper3", "indoor")', src)
        self.assertIn('mapper not in ("legacy", "mapper2", "terrain", "mapper3", "indoor")', src)

    def test_jetson_scripts_start_check_and_stop_mapper3(self):
        start = (ROOT / "remote/start_jetson.sh").read_text(encoding="utf-8")
        self.assertIn('"${NOMAD_MAPPER:-legacy}" == mapper3', start)
        self.assertIn("nomad_mapper3 started", start)
        self.assertIn("|nomad_mapper3|", start)  # duplicate-module guard
        check = (ROOT / "remote/check_jetson_started.sh").read_text(encoding="utf-8")
        self.assertIn("== mapper3 ]]", check)
        stop = (ROOT / "remote/stop_trial.sh").read_text(encoding="utf-8")
        for token in ('"$label" == mapper3', "mapper3:*nomad_mapper3*", "jetson:mapper3", "|nomad_mapper3|"):
            self.assertIn(token, stop)
        for name in ("start_jetson.sh", "check_jetson_started.sh", "stop_trial.sh"):
            subprocess.run(["bash", "-n", str(ROOT / "remote" / name)], check=True)


if __name__ == "__main__":
    unittest.main()
