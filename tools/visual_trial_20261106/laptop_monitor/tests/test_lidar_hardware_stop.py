"""Offline stop result and recording guard regressions. No board I/O."""
import json
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
FUNCTIONS = (ROOT / "remote/stop_trial.sh").read_text().split("# MAIN_ENTRY:")[0]

class LidarHardwareStopTests(unittest.TestCase):
    def run_shell(self, code):
        run = subprocess.run(["bash", "-s"], input=FUNCTIONS + "\n" + code,
                             text=True, capture_output=True, timeout=5)
        values = [json.loads(line.split("=", 1)[1]) for line in run.stdout.splitlines()
                  if line.startswith("NOMAD_STOP_RESULT=")]
        self.assertTrue(values, run.stderr + run.stdout)
        return run, values[-1]

    def test_no_driver_still_calls_hardware_stop(self):
        run, result = self.run_shell('''
board=jetson
recording_marker=/nonexistent/recording
ps() { :; }
stop_lidar_hardware() { lidar_hardware=stop_commands_verified; }
sweep_and_report no_pointer
''')
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(result["targets"], 0)
        self.assertEqual(result["lidar_hardware"], "stop_commands_verified")

    def test_hardware_failure_is_not_success(self):
        run, result = self.run_shell('''
board=jetson
recording_marker=/nonexistent/recording
ps() { :; }
stop_lidar_hardware() { lidar_hardware=port_busy; return 1; }
sweep_and_report no_pointer
''')
        self.assertEqual(run.returncode, 2)
        self.assertEqual(result["state"], "FAILED")
        self.assertEqual(result["reason"], "lidar_hardware_stop_failed")
        self.assertEqual(result["lidar_hardware"], "port_busy")

    def test_recording_without_recognized_processes_blocks_hardware_stop(self):
        run, result = self.run_shell('''
board=jetson
ps() { :; }
recording_active() { return 0; }
stop_lidar_hardware() { echo SHOULD_NOT_RUN; exit 77; }
sweep_and_report no_pointer
''')
        self.assertEqual(run.returncode, 2)
        self.assertEqual(result["reason"], "recording_active")
        self.assertNotIn("SHOULD_NOT_RUN", run.stdout)

    def test_pi5_does_not_touch_serial_port(self):
        run, result = self.run_shell('''
board=pi5
ps() { :; }
sweep_and_report no_pointer
''')
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(result["lidar_hardware"], "not_applicable")

if __name__ == "__main__":
    unittest.main()
