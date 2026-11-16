"""2026-11-12 planner placement: Pi4 (existing) or Jetson planner + Pi4 controller only."""
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore, QtWidgets
import planning_client as pc

APP = Path(__file__).resolve().parents[1]


class PlacementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = patch.dict(os.environ, {"XDG_STATE_HOME": self.tmp.name})
        self.env.start()

    def tearDown(self):
        self.env.stop()
        self.tmp.cleanup()

    def test_embedded_command_runs_shipped_text_with_arguments(self):
        parts = shlex.split(pc.embedded("pi4_output_only.sh", "vio_duty"))
        self.assertEqual(parts[:2], ["bash", "-c"])
        self.assertEqual(parts[2], (APP / "remote/pi4_output_only.sh").read_text())
        self.assertEqual(parts[3:], ["nomad", "vio_duty"])

    def test_scripts_are_valid_bash_and_hold_pi4_locks(self):
        for name in ("pi4_output_only.sh", "jetson_planner.sh"):
            subprocess.run(["bash", "-n", str(APP / "remote" / name)], check=True)
        pi4 = (APP / "remote/pi4_output_only.sh").read_text()
        for lock in ("/tmp/nomad-planning-${UID}.lock", "/tmp/nomad-output-${UID}.lock", "/home/user/vesc_ws/device.lock"):
            self.assertIn(lock, pi4)                  # same locks as the Pi4 launchers: no second planner
        self.assertNotIn("planner_node\" ", pi4)
        self.assertNotIn("kill -INT \"$pid\"", pi4)   # never takes over a running process
        self.assertIn("cat >/dev/null; kill -TERM $$", pi4)
        jetson = (APP / "remote/jetson_planner.sh").read_text()
        self.assertIn("/tmp/nomad_integrated_jetson_run_dir", jetson)
        self.assertIn("nomad-jetson-planning", jetson)

    def test_pi4_placement_keeps_existing_command(self):
        client = pc.PlanningClient("user@192.168.2.252", "CMD", self.tmp.name)
        self.assertEqual((client.placement, client.command, client.output_process), ("pi4", "CMD", None))

    def test_jetson_placement_validates_accounts(self):
        with self.assertRaises(ValueError):
            pc.PlanningClient("user@192.168.2.252", "CMD", self.tmp.name, placement="jetson", jetson_host="user@1.2.3.4")
        with self.assertRaises(ValueError):
            pc.PlanningClient("user@192.168.2.252", "CMD", self.tmp.name, placement="jetson",
                              jetson_host="nvidia@1.2.3.4", control_mode="bad")
        with self.assertRaises(ValueError):
            pc.PlanningClient("user@192.168.2.252", "CMD", self.tmp.name, placement="mars")

    def test_jetson_start_sequence_two_clocks_then_two_sessions(self):
        client = pc.PlanningClient("user@192.168.2.252", "CMD", self.tmp.name, placement="jetson",
                                   jetson_host="nvidia@192.168.1.152", control_mode="rpm")
        started, messages = [], []
        client.progress.connect(messages.append)
        with patch.object(client.clock_process, "start", lambda prog, args: started.append(("clock", args[2]))), \
                patch.object(client.clock_process, "readAllStandardOutput", return_value=QtCore.QByteArray(b'{"message":"ok"}\n')), \
                patch.object(client.process, "start", lambda prog, args: started.append(("planner", args[-2], args[-1]))), \
                patch.object(client.output_process, "start", lambda prog, args: started.append(("output", args[-2], args[-1]))):
            client.start()
            client.clock_finished(0, None)
            client.clock_finished(0, None)
        self.assertEqual([s[:2] for s in started],
                         [("clock", "user@192.168.2.252"), ("clock", "nvidia@192.168.1.152"),
                          ("output", "user@192.168.2.252"), ("planner", "nvidia@192.168.1.152")])
        self.assertIn("Pi4 runs only the VESC driver", started[2][2])
        self.assertIn("planner (Pi4 source copy built in", started[3][2])
        self.assertTrue(shlex.split(started[2][2])[-1] == "rpm")
        self.assertTrue(shlex.split(started[3][2])[-1] == "--stdio")
        self.assertTrue(any("Jetson 플래너" in m for m in messages))

    def test_jetson_clock_failure_starts_nothing(self):
        client = pc.PlanningClient("user@192.168.2.252", "CMD", self.tmp.name, placement="jetson",
                                   jetson_host="nvidia@192.168.1.152")
        started, closed, messages = [], [], []
        client.closed.connect(lambda: closed.append(1))
        client.progress.connect(messages.append)
        with patch.object(client.clock_process, "start", lambda prog, args: started.append(args[2])), \
                patch.object(client.clock_process, "readAllStandardOutput", return_value=QtCore.QByteArray(b'{"message":"x"}\n')), \
                patch.object(client.output_process, "start", lambda *a: self.fail("output started")), \
                patch.object(client.process, "start", lambda *a: self.fail("planner started")):
            client.start()
            client.clock_finished(0, None)
            client.clock_finished(1, None)
        self.assertEqual(closed, [1])
        self.assertTrue(any(m.startswith("Jetson 시간 동기화 확인 실패") for m in messages))

    def test_pi4_output_loss_closes_planner(self):
        client = pc.PlanningClient("user@192.168.2.252", "CMD", self.tmp.name, placement="jetson",
                                   jetson_host="nvidia@192.168.1.152")
        with patch.object(client, "stop") as stop:
            client.output_finished(1, None)
        stop.assert_called_once()
        client.read_output_session  # exists
        with patch.object(client.output_process, "readAllStandardOutput",
                          return_value=QtCore.QByteArray(b"NOMAD_PI4_OUTPUT_READY mode=rpm\n")):
            client.read_output_session()
        self.assertTrue(client.output_ready)


if __name__ == "__main__":
    unittest.main()
