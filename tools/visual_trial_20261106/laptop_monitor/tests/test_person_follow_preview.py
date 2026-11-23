"""Offline preview contract. Synthetic frames only; no board or actuator I/O."""
import base64
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('NOMAD_VIEWER_STATE_DIR', '/tmp/nomad_follow_gui_test_state')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtGui, QtTest, QtWidgets
import monitor
from developer_person_follow import PersonFollowWindow
from test_developer_planning import fixture, click


COMPONENTS = {'grid_geometry': monitor.grid_geometry, 'rotate_vector': monitor.rotate_vector,
              'ImagePanel': monitor.ImagePanel}


class PersonFollowPreviewTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.window = PersonFollowWindow(components=COMPONENTS)
        _, _, self.snapshot = fixture()
        self.snapshot['images'].update(rgb=self.image('#607080'), semantic=self.image('#f04646'))
        for source in ('rgb', 'semantic'):
            self.snapshot['image_meta'][source] = {'t': 1794190000., 'width': 40, 'height': 30}
            self.snapshot['received']['image:'+source] = 100.

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()

    @staticmethod
    def image(color):
        image = QtGui.QImage(40, 30, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor(color))
        return image

    def test_connected_operator_is_still_display_only(self):
        self.window.update_snapshot(self.snapshot)
        self.assertIsNotNone(self.window.map.pixels)
        self.assertIs(self.window.map.planning, self.snapshot['telemetry']['planning'])
        self.assertEqual(self.window.panels['rgb'].pixmap.width(), 40)
        self.assertIn('미연결', self.window.target.text())
        for button in (self.window.select_button, self.window.start_button, self.window.stop_button):
            self.assertFalse(button.isEnabled())
        self.assertFalse(self.window.map.goal_selection_enabled)
        spy = QtTest.QSignalSpy(self.window.map.goal_selected)
        self.window.map.select_goal(click(self.window.map))
        self.assertEqual(len(spy), 0)

    def test_replay_spectator_and_no_ssh_cannot_enable_actions(self):
        for replay, spectator, no_ssh, text in ((True, False, False, '기록 재생'),
                                               (False, True, False, '관전'),
                                               (False, False, True, 'SSH 비활성')):
            with self.subTest(text=text):
                self.snapshot['replay'] = replay
                self.snapshot['planning_ui'].update(spectator=spectator, no_ssh=no_ssh)
                self.snapshot['follow_ui'] = {'connected': True, 'can_start': True}
                self.window.update_snapshot(self.snapshot)
                self.assertIn(text, self.window.connection.text())
                self.assertFalse(self.window.select_button.isEnabled())
                self.assertFalse(self.window.start_button.isEnabled())
                self.assertFalse(self.window.stop_button.isEnabled())
                self.assertFalse(self.window.map.goal_selection_enabled)

    def test_invalid_new_image_does_not_refresh_last_good_frame(self):
        self.window.update_snapshot(self.snapshot)
        first = self.window.panels['semantic'].pixmap.cacheKey()
        self.snapshot['now'] = 106.
        self.snapshot['received']['image:semantic'] = 106.
        self.snapshot['image_meta']['semantic']['decode_error'] = True
        self.snapshot['images']['semantic'] = self.image('#000000')
        self.window.update_snapshot(self.snapshot)
        self.assertEqual(self.window.panels['semantic'].pixmap.cacheKey(), first)
        self.assertIn('오래됨', self.window.panels['semantic'].meta.text())
        self.assertIn('6.0초', self.window.panels['semantic'].meta.text())
        self.assertIn('오류', self.window.panels['semantic'].meta.text())
        self.assertEqual(self.window.map.planning, {})

    def test_time_seek_clears_future_images_and_map(self):
        self.window.show()
        self.window.update_snapshot(self.snapshot)
        self.app.processEvents()
        self.window.update_snapshot({'now': 10., 'replay': True})
        self.assertIsNone(self.window.map.pixels)
        self.assertEqual(self.window.map.history, ())
        self.assertEqual(self.window.map.planning, {})
        self.assertFalse(self.window.map._image_artist.get_visible())
        for panel in self.window.panels.values():
            self.assertIsNone(panel.pixmap)
            self.assertIn('미수신', panel.meta.text())
        # A new timeline can display frames again, including the same QImage.
        self.window.update_snapshot(self.snapshot)
        self.assertIsNotNone(self.window.map.pixels)
        self.assertEqual(self.window.panels['rgb'].pixmap.width(), 40)

    def test_shared_window_lifecycle_adds_no_readers_or_commands(self):
        with patch.object(sys, 'argv', ['monitor', '--no-ssh']):
            parent = monitor.MonitorWindow(monitor.parse_args())
        try:
            parent.open_developer_window('planning')
            parent.select_goal = Mock()
            parent.send_goal = Mock()
            parent.cancel_goal = Mock()
            parent.arm_output = Mock()
            parent.disarm_output = Mock()
            with patch.object(parent, 'ensure_log_reader') as logs, \
                 patch.object(parent, 'ensure_slope_cpu_reader') as slopes:
                entry = parent.developer_windows['planning'].follow_button
                entry.click()
                child = parent.developer_windows['person_follow']
                self.assertTrue(child.isVisible())
                child.close()
                entry.click()
                self.assertIs(parent.developer_windows['person_follow'], child)
                logs.assert_not_called()
                slopes.assert_not_called()
            self.assertEqual(parent.workers, [])
            self.assertIsNone(parent.planning_client)
            # Shared camera packets reach the new window through refresh().
            image = self.image('#607080')
            buf = QtCore.QBuffer()
            buf.open(QtCore.QIODevice.WriteOnly)
            image.save(buf, 'PNG')
            parent.on_packet('jetson', {'kind': 'image', 'source': 'rgb', 't': parent.wall_now(),
                'encoding': 'png', 'width': 40, 'height': 30,
                'data': base64.b64encode(bytes(buf.data())).decode('ascii')})
            parent.refresh()
            self.assertEqual(child.panels['rgb'].pixmap.width(), 40)
            child.map.select_goal(click(child.map))
            child.close()
            for method in (parent.select_goal, parent.send_goal, parent.cancel_goal,
                           parent.arm_output, parent.disarm_output):
                method.assert_not_called()
            parent.close()
            self.assertFalse(child.isVisible())
        finally:
            parent.close()
            parent.deleteLater()
            self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
