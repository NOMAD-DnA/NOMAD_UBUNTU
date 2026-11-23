"""Planner control-mode selection (rpm / vio_duty) is passed explicitly per session."""
import os, sys, unittest
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))
import planning_client as pc


class Tests(unittest.TestCase):
    def test_mode_is_explicit_env_prefix(self):
        base = pc.DEFAULT_PLANNING_COMMAND
        self.assertEqual(pc.command_with_mode(base, 'rpm'), 'NOMAD_CONTROL_MODE=rpm ' + base)
        self.assertEqual(pc.command_with_mode(base, 'vio_duty'), 'NOMAD_CONTROL_MODE=vio_duty ' + base)
        for bad in ('', 'duty', 'rpm; reboot', None):
            with self.assertRaises(ValueError):
                pc.command_with_mode(base, bad)

    def test_monitor_uses_selector_and_locks_it_while_connected(self):
        text = (BASE / 'monitor.py').read_text()
        self.assertIn('command_with_mode(getattr(self.args, "planning_command", DEFAULT_PLANNING_COMMAND),', text)
        self.assertIn('self.planning_mode.currentData()', text)
        self.assertEqual(text.count('self.planning_mode.setEnabled(False)'), 2)
        self.assertIn('self.planning_mode.setEnabled(True)', text)
        self.assertEqual(list(pc.CONTROL_MODES)[0], 'rpm')  # default stays the existing controller


if __name__ == '__main__':
    unittest.main()
