"""Manual odom x/y goal entry: staging only, no board connection."""
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_PLANNING_TIMING_DIR", tempfile.mkdtemp(prefix="nomad-manual-goal-test-"))
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_manual_goal_gui_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtGui, QtWidgets
import monitor
import planning_client
from developer_planning import PlanningWindow


GEOMETRY = {"georeferenced": True, "grid": {"frame_id": "odom", "width": 20, "height": 20,
            "resolution": .1, "image_y": "high_to_low", "origin": {"x": 10., "y": 20., "z": 0.,
            "qx": 0., "qy": 0., "qz": math.sin(math.pi/4), "qw": math.cos(math.pi/4)}}}


class ManualGoalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with patch.object(sys, "argv", ["monitor", "--no-ssh"]):
            self.window = monitor.MonitorWindow(monitor.parse_args())
        self.window.timer.stop()
        self.window.args.no_ssh = False
        self.window.refresh()

    def tearDown(self):
        self.window.planning_client = None
        self.window.close()
        self.app.processEvents()

    def add_map(self):
        panel = self.window.panels["costmap"]
        image = QtGui.QImage(20, 20, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("gray"))
        panel.set_image(image, GEOMETRY)
        self.window.images["costmap"] = GEOMETRY
        self.window.received["image:costmap"] = time.monotonic()
        panel.goal_selection_enabled = True
        return panel

    def test_manual_goal_stages_without_map_and_never_sends(self):
        client = SimpleNamespace(send=Mock(return_value=True), ready=True, stopping=False)
        self.window.planning_client = client
        self.window.goal_x_input.setValue(12.34)
        self.window.goal_y_input.setValue(-5.5)
        self.window.goal_manual_set.click()
        self.assertEqual(self.window.selected_goal, (12.34, -5.5, "odom"))
        self.assertEqual(self.window.selected_goal_source, "manual")
        self.assertIn("지도 없음", self.window.planning_message.text())
        self.assertIn("현재 odom 기준", self.window.planning_message.text())
        client.send.assert_not_called()
        self.window.send_goal()
        client.send.assert_called_once_with(planning_client.goal_command(12.34, -5.5, "odom"))
        self.assertNotIn({"cmd": "arm"}, [c.args[0] for c in client.send.call_args_list])

    def test_manual_goal_warns_outside_map_but_still_stages(self):
        self.add_map()
        # The map is rotated 90 degrees around (10, 20): it covers x 8..10, y 20..22.
        self.window.manual_goal(9., 21.)
        self.assertEqual(self.window.selected_goal, (9., 21., "odom"))
        self.assertNotIn("플래너가 판정", self.window.planning_message.text())
        self.window.manual_goal(30., 30.)
        self.assertEqual(self.window.selected_goal, (30., 30., "odom"))
        self.assertIn("지도 밖 목표", self.window.planning_message.text())
        self.assertIs(self.window.goal_inside_map(11., 21.), False)
        self.assertIs(self.window.goal_inside_map(8.5, 21.9), True)

    def test_invalid_values_and_guards_are_rejected(self):
        self.window.manual_goal(float("nan"), 1.)
        self.assertIsNone(self.window.selected_goal)
        self.window.manual_goal(1., float("inf"))
        self.assertIsNone(self.window.selected_goal)
        for spectator, no_ssh in ((True, False), (False, True)):
            self.window.spectator_mode, self.window.args.no_ssh = spectator, no_ssh
            self.window.relay_address = ("192.0.2.2", 8768) if spectator else None
            self.window.manual_goal(1., 2.)
            self.assertIsNone(self.window.selected_goal)
            self.window.refresh()
            self.assertFalse(self.window.goal_manual_set.isEnabled())
            self.assertFalse(self.window.goal_x_input.isEnabled())
        self.window.spectator_mode = self.window.args.no_ssh = False
        self.window.relay_address = None
        self.window.refresh()
        self.assertTrue(self.window.goal_manual_set.isEnabled())

    def test_map_click_fills_inputs_and_records_click_source(self):
        panel = self.add_map()
        panel.select_goal(SimpleNamespace(button=1, inaxes=panel.ax, xdata=9.25, ydata=21.5))
        self.assertEqual(self.window.selected_goal, (9.25, 21.5, "odom"))
        self.assertEqual((self.window.goal_x_input.value(), self.window.goal_y_input.value()), (9.25, 21.5))
        self.assertEqual(self.window._view_event()["selected_goal_source"], "click")
        self.assertTrue(self.window.planning_message.text().startswith("선택 목표 (9.25, 21.50)"))
        self.window.manual_goal()
        self.assertEqual(self.window._view_event()["selected_goal_source"], "manual")
        snapshot = self.window.developer_snapshot()["planning_ui"]
        self.assertEqual(snapshot["selected_goal_source"], "manual")
        self.assertTrue(snapshot["manual_enabled"])
        self.window.selected_goal = None
        self.assertIsNone(self.window._view_event()["selected_goal_source"])

    def test_replay_view_restores_goal_source(self):
        view = self.window._view_event()
        view.update(selected_goal=[1., 2., "odom"], selected_goal_source="manual")
        self.window._apply_replay_record({"seq": 1, "offset_s": 0., "board": "gui", "event": view})
        self.assertEqual(self.window.selected_goal, (1., 2., "odom"))
        self.assertEqual(self.window.selected_goal_source, "manual")
        view.update(selected_goal_source="bogus")
        self.window._apply_replay_record({"seq": 2, "offset_s": 0., "board": "gui", "event": view})
        self.assertIsNone(self.window.selected_goal_source)

    def test_developer_window_manual_entry_calls_controller_and_syncs(self):
        window = PlanningWindow(components={"grid_geometry": monitor.grid_geometry,
                                            "rotate_vector": monitor.rotate_vector})
        try:
            snapshot = {"now": 1., "planning_ui": {"spectator": False, "no_ssh": False, "manual_enabled": True,
                                                   "selected_goal": (3., 4., "odom")}}
            controller = SimpleNamespace(manual_goal=Mock(), developer_snapshot=lambda: snapshot)
            window.controller = controller
            window.update_snapshot(snapshot)
            self.assertTrue(window.manual_button.isEnabled())
            self.assertEqual((window.goal_x_input.value(), window.goal_y_input.value()), (3., 4.))
            window.goal_x_input.setValue(7.5)
            window.manual_button.click()
            controller.manual_goal.assert_called_once_with(7.5, 4.)
            controller.manual_goal.reset_mock()
            snapshot["planning_ui"].update(spectator=True)
            window.update_snapshot(snapshot)
            self.assertFalse(window.manual_button.isEnabled())
            window._manual_goal()
            controller.manual_goal.assert_not_called()
        finally:
            window.close()
            window.deleteLater()


if __name__ == "__main__":
    unittest.main()
