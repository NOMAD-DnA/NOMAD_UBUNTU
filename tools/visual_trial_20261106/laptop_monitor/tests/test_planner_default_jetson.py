"""2026-11-17 user request: the planner runs on the Jetson unless a laptop saved another choice."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import trial_settings  # noqa: E402


class PlannerDefaultTest(unittest.TestCase):
    def test_default_placement_is_jetson(self):
        label, choices, default = trial_settings.SETTINGS["placement"]
        self.assertEqual(default, "jetson")
        self.assertIn("pi4", choices)  # Pi4 stays selectable


if __name__ == "__main__":
    unittest.main()
