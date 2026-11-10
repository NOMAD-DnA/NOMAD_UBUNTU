"""Goal/path display contracts. No board connection or actuator execution."""
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_planning_gui_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtGui, QtWidgets
import monitor
import planning_client
import relay


def packet(**changes):
    return {"type": "planning", "t": 1794000000., "stamp_ns": 1793999999900000000,
            "frame": "odom", "state": "TRACKING", "detail": "test fixture",
            "plan_only": True, "path": [[0., 0., 0.], [1., 0., 0.]],
            "local_path": [[0., 0., 0.], [.5, 0., 0.]], **changes}


class PlanningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with patch.object(sys, "argv", ["monitor", "--no-ssh"]):
            self.window = monitor.MonitorWindow(monitor.parse_args())
        self.window.timer.stop()

    def tearDown(self):
        self.window.planning_client = None
        self.window.close()
        self.app.processEvents()

    def test_goal_protocol_never_arms_and_only_separate_exact_commands_are_allowed(self):
        self.assertEqual(planning_client.goal_command(1., 2., "odom")["cmd"], "goal")
        for values in ((math.nan, 0., "odom"), (0., True, "odom"), (0., 0., "map")):
            with self.assertRaises(ValueError):
                planning_client.goal_command(*values)
        client = planning_client.PlanningClient("user@192.0.2.1", "not executed", Path("/tmp"))
        client.ready = True
        client.process = Mock()
        client.process.state.return_value = client.process.Running = planning_client.QtCore.QProcess.Running
        client.process.write.side_effect = len
        client.send(planning_client.goal_command(1., 2., "odom"))
        goal_body = client.process.write.call_args.args[0]
        self.assertEqual(__import__('json').loads(goal_body)["cmd"], "goal")
        self.assertNotIn(b'arm', goal_body)
        client.send({"cmd": "arm"})
        self.assertEqual(client.process.write.call_args.args[0], b'{"cmd":"arm"}\n')
        with self.assertRaises(ValueError):
            client.send({"cmd": "arm", "throttle": .5})
        client.stop()
        self.assertEqual([call.args[0] for call in client.process.write.call_args_list[-2:]],
                         [b'{"cmd":"disarm"}\n', b'{"cmd":"cancel"}\n'])
        client.force_timer.stop()
        client.deleteLater()

    def test_output_report_is_conservative_and_permission_is_not_physical_motion(self):
        report = planning_client.display_event(packet(plan_only=False, state="STOP", armed=True,
                  actuator_enabled=True, prototype_output=True, output_mode="prototype",
                  feedback_source="measured_speed_unmeasured_steering", output_stamp_ns=1793999999900000001,
                  output_inhibit="WAIT_ODOMETRY"))
        self.assertEqual(report["feedback_source"], "measured_speed_unmeasured_steering")
        self.assertEqual(report["output_stamp_ns"], 1793999999900000001)
        self.assertIn("현재 정지/대기", planning_client.output_summary(report))
        self.assertIn("실측 검증 전", planning_client.output_summary(report))
        self.assertIn("WAIT_ODOMETRY", planning_client.output_summary(report))
        missing = planning_client.display_event(packet(plan_only=False))
        self.assertFalse(missing["armed"])
        self.assertFalse(missing["actuator_enabled"])
        self.assertIn("출력 잠금", planning_client.output_summary(missing))
        self.assertEqual(relay._validate("pi4", report)[1], "planning")

    def test_explicit_output_buttons_use_fresh_native_permission_and_cancel_disarms(self):
        client = SimpleNamespace(send=Mock(return_value=True), ready=True, stopping=False)
        self.window.planning_client = client
        self.window.args.no_ssh = False
        self.window.on_packet("pi4", planning_client.display_event(packet(plan_only=False, armed=False)))
        self.window.refresh()
        self.assertTrue(self.window.output_arm.isEnabled())
        self.window.arm_output()
        client.send.assert_called_once_with({"cmd": "arm"})
        self.window.on_packet("pi4", planning_client.display_event(packet(plan_only=False, armed=True)))
        self.window.refresh()
        self.assertFalse(self.window.output_arm.isEnabled())
        self.window.cancel_goal()
        self.assertEqual([call.args[0] for call in client.send.call_args_list[-2:]],
                         [{"cmd": "disarm"}, {"cmd": "cancel"}])
        self.window.received["telemetry:planning"] -= 4
        client.send.reset_mock()
        self.window.refresh()
        self.window.refresh()
        self.assertFalse(self.window.output_arm.isEnabled())
        client.send.assert_called_once_with({"cmd": "disarm"})
        self.window.arm_output()
        client.send.assert_called_once_with({"cmd": "disarm"})

    def test_actual_native_packet_preserves_measurement_stamp_and_clears_invalid_path(self):
        event = planning_client.display_event(packet())
        self.assertEqual(event["stamp_ns"], 1793999999900000000)
        self.assertEqual(event["trajectory"], [[0., 0., 0.], [.5, 0., 0.]])
        self.assertIsNotNone(relay._validate("pi4", event))
        self.assertIsNone(relay._validate("pi4", {"kind": "status", "source": "localization", "t": 1., "state": "READY"}))
        self.window.on_packet("pi4", event)
        self.assertEqual(self.window.panels["costmap"].planning["path"], event["path"])
        self.window.on_packet("pi4", planning_client.display_event(packet(state="WAIT", path=[], local_path=[])))
        self.assertEqual(self.window.panels["costmap"].planning["path"], [])
        self.window.on_packet("pi4", {**event, "_relay_age_s": 4.})
        self.window.refresh()
        self.assertEqual(self.window.panels["costmap"].planning, {})

    def test_rotated_map_can_stage_and_send_stale_goal_without_arming(self):
        panel = self.window.panels["costmap"]
        image = QtGui.QImage(20, 20, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("gray"))
        geometry = {"georeferenced": True, "grid": {"frame_id": "odom", "width": 20, "height": 20,
                    "resolution": .1, "image_y": "high_to_low", "origin": {"x": 10., "y": 20., "z": 0.,
                    "qx": 0., "qy": 0., "qz": math.sin(math.pi/4), "qw": math.cos(math.pi/4)}}}
        panel.set_image(image, geometry)
        self.window.images["costmap"] = geometry
        panel.goal_selection_enabled = True
        self.window.args.no_ssh = False
        self.window.received["image:costmap"] = time.monotonic()
        panel.select_goal(SimpleNamespace(button=1, inaxes=panel.ax, xdata=9., ydata=21.))
        self.assertEqual(self.window.selected_goal, (9., 21., "odom"))
        panel.select_goal(SimpleNamespace(button=1, inaxes=panel.ax, xdata=30., ydata=30.))
        self.assertEqual(self.window.selected_goal, (30., 30., "odom"))
        self.window.received["image:costmap"] -= 4.
        panel.select_goal(SimpleNamespace(button=1, inaxes=panel.ax, xdata=9.5, ydata=20.5))
        self.assertEqual(self.window.selected_goal, (9.5, 20.5, "odom"))
        client = SimpleNamespace(send=Mock(return_value=True), ready=True, stopping=False)
        self.window.planning_client = client
        self.window.send_goal()
        client.send.assert_called_once_with(planning_client.goal_command(9.5, 20.5, "odom"))
        self.assertIn("지도 갱신 지연", self.window.planning_message.text())
        self.assertNotIn({"cmd": "arm"}, [call.args[0] for call in client.send.call_args_list])
        self.window.received["telemetry:planning"] = time.monotonic()
        self.window.refresh()
        self.assertTrue(self.window.goal_send.isEnabled())
        client.send.reset_mock()
        self.window.received["image:costmap"] = time.monotonic()
        self.window.send_goal()
        client.send.assert_called_once_with(planning_client.goal_command(9.5, 20.5, "odom"))

    def test_spectator_methods_never_send_and_large_paths_keep_endpoints(self):
        self.window.spectator_mode = True
        self.window.relay_address = ("192.0.2.2", 8768)
        self.window.planning_client = SimpleNamespace(send=Mock(side_effect=AssertionError("read-only")), ready=True, stopping=False)
        self.window.selected_goal = (1., 2., "odom")
        self.window.send_goal()
        self.window.cancel_goal()
        self.window.arm_output()
        self.window.disarm_output()
        self.window.connect_planner()
        self.window.refresh()
        self.assertFalse(self.window.goal_send.isEnabled())
        self.assertFalse(self.window.output_arm.isEnabled())
        self.assertFalse(self.window.output_disarm.isEnabled())
        points = [[float(i), 0., 0.] for i in range(3000)]
        bounded = planning_client.display_event(packet(path=points))["path"]
        self.assertEqual(len(bounded), 600)
        self.assertEqual((bounded[0], bounded[-1]), (points[0], points[-1]))
        with self.assertRaises(ValueError):
            planning_client.display_event(packet(path=[[float("nan"), 0., 0.]]))


if __name__ == "__main__":
    unittest.main()
