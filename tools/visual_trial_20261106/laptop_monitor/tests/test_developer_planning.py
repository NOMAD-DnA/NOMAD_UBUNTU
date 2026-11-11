"""Local planning-view checks with synthetic snapshots; no board or ROS I/O."""
import copy
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_planning_developer_ui_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtGui, QtTest, QtWidgets
import monitor
from developer_planning import PlanningMap, PlanningWindow


COMPONENTS = {"grid_geometry": monitor.grid_geometry, "rotate_vector": monitor.rotate_vector}


def fixture():
    image = QtGui.QImage(20, 10, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("#608040"))
    event = {"t": 1794190000., "georeferenced": True,
             "grid": {"frame_id": "odom", "width": 20, "height": 10,
                      "resolution": .1, "image_y": "high_to_low",
                      "origin": {"x": 10., "y": 20., "z": 0., "qx": 0., "qy": 0.,
                                 "qz": math.sin(math.pi/4), "qw": math.cos(math.pi/4)}}}
    pose = {"t": 1794190000., "frame_id": "odom", "x": 9.5, "y": 21., "z": 0.,
            "qx": 0., "qy": 0., "qz": 0., "qw": 1.}
    planning = {"frame": "odom", "state": "TRACKING", "path": [[9.5, 21., 0.], [30., 30., 0.]],
                "trajectory": [[9.5, 21., 0.], [10., 21., 0.]], "goal": [30., 30., 0.]}
    snapshot = {"now": 100., "images": {"costmap": image}, "image_meta": {"costmap": event},
                "received": {"image:costmap": 100., "pose:public": 100., "telemetry:planning": 100.},
                "histories": {"public": [pose]}, "telemetry": {"planning": planning},
                "forward_axis": (1., 0., 0.),
                "planning_ui": {"spectator": False, "no_ssh": False, "host": "user@192.0.2.1",
                                "client_active": True, "send_enabled": True, "cancel_enabled": True,
                                "arm_enabled": True, "disarm_enabled": True,
                                "message": "fixture", "selected_goal": (30., 30., "odom")}}
    return image, event, snapshot


def click(panel, **changes):
    return SimpleNamespace(button=1, inaxes=panel.ax, xdata=30., ydata=30., x=100., y=100., **changes)


class PlanningDeveloperViewTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.windows = []

    def tearDown(self):
        for window in self.windows:
            window.close()
            window.deleteLater()
        self.app.processEvents()

    def panel(self):
        panel = PlanningMap(COMPONENTS)
        self.windows.append(panel)
        image, event, _ = fixture()
        panel.set_image(image, event)
        panel.goal_selection_enabled = True
        return panel

    def window(self):
        window = PlanningWindow(components=COMPONENTS)
        self.windows.append(window)
        return window

    def test_rotated_map_outside_goal_is_unchanged_odom_and_invalid_clicks_are_ignored(self):
        panel = self.panel()
        selected = []
        panel.goal_selected.connect(lambda *goal: selected.append(goal))
        panel.select_goal(click(panel))
        self.assertEqual(selected, [(30., 30., "odom")])
        # Neither map origin translation nor its 90-degree rotation applies to an axes click again.
        panel.redraw()
        artist = panel.ax.images[0]
        displayed = panel.ax.transData.inverted().transform(artist.get_transform().transform([[0., 0.], [2., 1.]]))
        self.assertAlmostEqual(displayed[0][0], 10.)
        self.assertAlmostEqual(displayed[0][1], 20.)
        self.assertAlmostEqual(displayed[1][0], 9.)
        self.assertAlmostEqual(displayed[1][1], 22.)
        for attribute, value in (("button", 3), ("inaxes", None), ("xdata", math.nan), ("ydata", True)):
            event = click(panel)
            setattr(event, attribute, value)
            panel.select_goal(event)
        panel.goal_selection_enabled = False
        panel.select_goal(click(panel))
        panel.goal_selection_enabled = True
        panel.event = copy.deepcopy(panel.event)
        panel.event["grid"]["frame_id"] = "map"
        panel.select_goal(click(panel))
        self.assertEqual(len(selected), 1)

    def test_zoom_pan_limits_survive_redraw_and_drag_does_not_select_goal(self):
        panel = self.panel()
        selected = []
        panel.goal_selected.connect(lambda *goal: selected.append(goal))
        panel.zoom(.5, (30., 30.))
        self.assertEqual(panel._limits(), ((25., 35.), (25., 35.)))
        panel.canvas.draw()
        right = click(panel)
        right.button = 3
        panel._mouse_press(right)
        moved = click(panel)
        moved.x, moved.y = 140., 125.
        panel._mouse_move(moved)
        moved.button = 3
        panel._mouse_release(moved)
        limits = panel._limits()
        self.assertNotEqual(limits, ((25., 35.), (25., 35.)))
        _, _, snapshot = fixture()
        panel.history = snapshot["histories"]["public"]
        panel.planning = snapshot["telemetry"]["planning"]
        panel.selected_goal = (30., 30., "odom")
        panel.redraw()
        self.assertEqual(panel._limits(), limits)
        panel._mouse_press(click(panel))
        moved.button = 1
        panel._mouse_release(moved)
        self.assertEqual(selected, [])
        panel.fit_all()
        self.assertLess(panel.ax.get_xlim()[0], 9.)
        self.assertGreater(panel.ax.get_xlim()[1], 30.)

    def test_failed_fresh_image_never_refreshes_old_display_and_stale_planning_is_hidden(self):
        window = self.window()
        _, _, snapshot = fixture()
        window.update_snapshot(snapshot)
        pixels = window.map.pixels
        self.assertTrue(window.map.goal_selection_enabled)
        self.assertIs(window.map.planning, snapshot["telemetry"]["planning"])
        snapshot["now"] = 104.
        snapshot["received"]["image:costmap"] = 104.
        snapshot["image_meta"]["costmap"] = {**snapshot["image_meta"]["costmap"], "decode_error": True}
        window.update_snapshot(snapshot)
        self.assertIs(window.map.pixels, pixels)
        self.assertEqual(window.map.map_age, 4.)
        self.assertTrue(window.map.goal_selection_enabled)
        self.assertIn('지도 갱신 지연 · 목표 선택/전송 가능', window.detail.text())
        self.assertEqual(window.map.planning, {})
        snapshot["image_meta"]["costmap"] = {"georeferenced": False, "t": 1794190004.}
        window.update_snapshot(snapshot)
        self.assertIs(window.map.pixels, pixels)
        self.assertTrue(window.map.goal_selection_enabled)

    def test_stale_map_allows_staging_via_real_qt_click_but_send_stays_disabled(self):
        window = self.window()
        _, _, snapshot = fixture()
        snapshot['planning_ui']['send_enabled'] = False
        controller = SimpleNamespace(select_goal=Mock(), developer_snapshot=lambda: snapshot)
        window.controller = controller
        window.update_snapshot(snapshot)
        snapshot['now'] = 323.
        window.update_snapshot(snapshot)
        self.assertEqual(window.map.map_age, 223.)
        self.assertTrue(window.map.goal_selection_enabled)
        self.assertFalse(window.send_button.isEnabled())
        self.assertIn('목표 선택/전송 가능', window.detail.text())
        window.show()
        self.app.processEvents()
        window.map.redraw()
        window.map.canvas.draw()
        selected = QtTest.QSignalSpy(window.map.goal_selected)
        x, y = 11., 23.
        px, py = window.map.ax.transData.transform((x, y))
        point = QtCore.QPoint(round(px), round(window.map.canvas.height()-py))
        QtTest.QTest.mousePress(window.map.canvas, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, point)
        QtTest.QTest.mouseRelease(window.map.canvas, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, point)
        self.app.processEvents()
        self.assertEqual(len(selected), 1)
        goal_x, goal_y, frame = selected[0]
        self.assertAlmostEqual(goal_x, x, delta=.1)
        self.assertAlmostEqual(goal_y, y, delta=.1)
        self.assertEqual(frame, 'odom')
        controller.select_goal.assert_called_once_with(goal_x, goal_y, 'odom')
        self.assertFalse(window.send_button.isEnabled())

    def test_no_valid_displayed_geometry_never_allows_staging_and_bad_replacement_keeps_odom(self):
        window = self.window()
        _, _, snapshot = fixture()
        valid_meta = snapshot['image_meta']['costmap']
        snapshot['image_meta']['costmap'] = {'georeferenced': False, 't': 1794190000.}
        window.update_snapshot(snapshot)
        self.assertFalse(window.map.goal_selection_enabled)
        self.assertIsNone(window.map.pixels)
        self.assertIn('odom 지도 좌표 미수신 · 목표 선택 불가', window.detail.text())
        snapshot['image_meta']['costmap'] = valid_meta
        window.update_snapshot(snapshot)
        pixels = window.map.pixels
        self.assertTrue(window.map.goal_selection_enabled)
        invalid_frame = copy.deepcopy(valid_meta)
        invalid_frame['grid']['frame_id'] = 'other_map'
        snapshot['image_meta']['costmap'] = invalid_frame
        snapshot['received']['image:costmap'] = 101.
        snapshot['now'] = 104.
        window.update_snapshot(snapshot)
        self.assertIs(window.map.pixels, pixels)
        self.assertEqual(window.map._frame(), 'odom')
        self.assertEqual(window.map.map_age, 4.)
        self.assertTrue(window.map.goal_selection_enabled)
        self.assertIn('목표 선택/전송 가능', window.detail.text())

    def test_shared_snapshot_actions_use_existing_controller_and_keep_viewport(self):
        window = self.window()
        _, _, snapshot = fixture()
        controller = SimpleNamespace(planning_host=QtWidgets.QLineEdit(), connect_planner=Mock(),
                                     send_goal=Mock(), cancel_goal=Mock(), select_goal=Mock(),
                                     arm_output=Mock(), disarm_output=Mock(),
                                     developer_snapshot=lambda: snapshot)
        window.controller = controller
        window.update_snapshot(snapshot)
        window.map.zoom(.7)
        limits = window.map._limits()
        for name in ("connect_planner", "send_goal", "cancel_goal", "arm_output", "disarm_output"):
            window._action(name)
            getattr(controller, name).assert_called_once_with()
        window._select_goal(40., 50., "odom")
        controller.select_goal.assert_called_once_with(40., 50., "odom")
        self.assertEqual(window.map.selected_goal, snapshot["planning_ui"]["selected_goal"])
        self.assertIs(window.map.planning, snapshot["telemetry"]["planning"])
        window.update_snapshot(snapshot)
        window.map.redraw()
        self.assertEqual(window.map._limits(), limits)
        self.assertEqual(window.findChildren(QtCore.QProcess), [])
        self.assertEqual(window.findChildren(QtCore.QThread), [])

    def test_controller_reset_button_follows_snapshot_and_calls_monitor(self):
        window = self.window()
        _, _, snapshot = fixture()
        controller = SimpleNamespace(reset_controller=Mock())
        window.controller = controller
        snapshot["planning_ui"].update(spectator=False, no_ssh=False, reset_enabled=True, reset_busy=False)
        window.update_snapshot(snapshot)
        self.assertTrue(window.reset_button.isEnabled())
        window.reset_button.click()
        controller.reset_controller.assert_called_once_with()
        snapshot["planning_ui"].update(reset_enabled=False, reset_busy=True)
        window.update_snapshot(snapshot)
        self.assertFalse(window.reset_button.isEnabled())
        self.assertIn("복구 중", window.reset_button.text())

    def test_spectator_and_no_ssh_never_invoke_controller_or_enable_goal_selection(self):
        window = self.window()
        _, _, snapshot = fixture()
        denied = Mock(side_effect=AssertionError("read-only developer view attempted control"))
        window.controller = SimpleNamespace(connect_planner=denied, send_goal=denied,
                                            cancel_goal=denied, select_goal=denied,
                                            arm_output=denied, disarm_output=denied,
                                            reset_controller=denied)
        for mode in ("spectator", "no_ssh"):
            snapshot["planning_ui"].update(spectator=mode == "spectator", no_ssh=mode == "no_ssh")
            window.update_snapshot(snapshot)
            for name in ("connect_planner", "send_goal", "cancel_goal", "arm_output", "disarm_output",
                         "reset_controller"):
                window._action(name)
            window._select_goal(30., 30., "odom")
            self.assertFalse(window.reset_button.isEnabled())
            self.assertFalse(window.connect_button.isEnabled())
            self.assertFalse(window.send_button.isEnabled())
            self.assertFalse(window.cancel_button.isEnabled())
            self.assertFalse(window.arm_button.isEnabled())
            self.assertFalse(window.disarm_button.isEnabled())
            self.assertFalse(window.map.goal_selection_enabled)
            self.assertIn('목표 선택 불가', window.detail.text())
            self.assertIn('관전 · 읽기 전용' if mode == 'spectator' else 'SSH 비활성 · 읽기 전용', window.detail.text())
        denied.assert_not_called()
        self.assertEqual(window.findChildren(QtCore.QProcess), [])
        self.assertEqual(window.findChildren(QtCore.QThread), [])


if __name__ == "__main__":
    unittest.main()
