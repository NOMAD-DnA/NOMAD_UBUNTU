"""2026-11-13: map panel zoom (wheel), pan (right/middle drag) and return to the automatic view."""
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('NOMAD_VIEWER_STATE_DIR', '/tmp/nomad_map_view_test')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtGui, QtWidgets
from matplotlib.backend_bases import MouseEvent
import monitor


def grid(t=100.):
    return dict(t=t, georeferenced=True, grid=dict(frame_id='odom', width=20, height=10,
        resolution=.1, image_y='high_to_low', origin=dict(x=-1., y=-.5, z=0., qx=0., qy=0., qz=0., qw=1.)))


def image():
    result = QtGui.QImage(20, 10, QtGui.QImage.Format_RGB888)
    result.fill(QtGui.QColor('green'))
    return result


def pose(x=0.):
    return dict(t=100., x=x, y=0., z=0., qx=0., qy=0., qz=0., qw=1., frame_id='odom')


class MapViewTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.panel = monitor.MapPanel('fixture')
        self.panel.resize(400, 400)
        self.panel.show()
        self.app.processEvents()
        self.panel.set_image(image(), grid())
        self.panel.set_pose([pose()], 0.)
        self.panel.redraw()
        self.panel.canvas.draw()

    def tearDown(self):
        self.panel.close()
        self.panel.deleteLater()
        self.app.processEvents()

    def event(self, name, xdata, ydata, button=None, step=0, dblclick=False):
        x, y = self.panel.ax.transData.transform((xdata, ydata))
        return MouseEvent(name, self.panel.canvas, x, y, button=button, step=step, dblclick=dblclick)

    def spans(self):
        (x0, x1), (y0, y1) = self.panel.ax.get_xlim(), self.panel.ax.get_ylim()
        return x1 - x0, y1 - y0

    def test_wheel_zooms_at_cursor_and_stays_manual_until_auto(self):
        auto = self.panel.ax.get_xlim(), self.panel.ax.get_ylim()
        before = self.spans()
        self.panel.on_scroll(self.event('scroll_event', .5, 0., button='up', step=1))
        after = self.spans()
        self.assertAlmostEqual(after[0], before[0] / 1.25, places=6)
        self.assertTrue(self.panel.manual_view)
        self.assertTrue(self.panel.fit_button.isEnabled())
        # The point under the cursor did not move.
        x0, x1 = self.panel.ax.get_xlim()
        self.assertAlmostEqual((.5 - auto[0][0]) / before[0], (.5 - x0) / after[0], places=6)
        # New data does not reset a manual view, even when the vehicle leaves it.
        manual = self.panel.ax.get_xlim()
        self.panel.set_pose([pose(), pose(30.)], 0.)
        self.panel.redraw()
        self.assertEqual(self.panel.ax.get_xlim(), manual)
        self.panel.auto_view()
        self.assertFalse(self.panel.manual_view)
        self.assertFalse(self.panel.fit_button.isEnabled())
        self.assertGreater(self.panel.ax.get_xlim()[1], 30.)   # automatic view includes the vehicle again

    def test_zoom_limits(self):
        for _ in range(40):
            self.panel.on_scroll(self.event('scroll_event', 0., 0., button='up', step=1))
        self.assertGreaterEqual(min(self.spans()), monitor.MapPanel.MIN_SPAN_M - 1e-9)
        for _ in range(80):
            self.panel.on_scroll(self.event('scroll_event', 0., 0., button='down', step=-1))
        self.assertLessEqual(max(self.spans()), monitor.MapPanel.MAX_SPAN_M + 1e-9)

    def test_right_drag_pans_and_left_click_still_picks_goals(self):
        picked = []
        self.panel.goal_selected.connect(lambda x, y, f: picked.append((x, y, f)))
        self.panel.goal_selection_enabled = True
        x_before = self.panel.ax.get_xlim()
        start = self.event('button_press_event', 0., 0., button=3)
        self.panel.on_press(start)
        self.panel.select_goal(start)
        end = MouseEvent('motion_notify_event', self.panel.canvas, start.x + 40, start.y, button=3)
        self.panel.on_motion(end)
        self.panel.on_release(end)
        moved = self.panel.ax.get_xlim()
        self.assertLess(moved[0], x_before[0])          # dragged right -> view moves left
        self.assertAlmostEqual(moved[1] - moved[0], x_before[1] - x_before[0], places=6)
        self.assertEqual(picked, [])                     # right drag never picks a goal
        self.panel.select_goal(self.event('button_press_event', .2, .1, button=1))
        self.assertEqual(len(picked), 1)
        self.panel.select_goal(self.event('button_press_event', .2, .1, button=1, dblclick=True))
        self.assertEqual(len(picked), 1)                 # double-click does not pick a second goal

    def test_double_click_returns_to_auto_only_without_goal_picking(self):
        self.panel.on_scroll(self.event('scroll_event', 0., 0., button='up', step=1))
        self.panel.goal_selection_enabled = True
        self.panel.on_press(self.event('button_press_event', 0., 0., button=1, dblclick=True))
        self.assertTrue(self.panel.manual_view)
        self.panel.goal_selection_enabled = False
        self.panel.on_press(self.event('button_press_event', 0., 0., button=1, dblclick=True))
        self.assertFalse(self.panel.manual_view)

    def test_clear_data_returns_to_auto(self):
        self.panel.on_scroll(self.event('scroll_event', 0., 0., button='up', step=1))
        self.panel.clear_data()
        self.assertFalse(self.panel.manual_view)


if __name__ == '__main__':
    unittest.main()
