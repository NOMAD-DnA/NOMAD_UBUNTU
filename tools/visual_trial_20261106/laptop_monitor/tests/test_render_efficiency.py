"""Rendering work stays bounded; synthetic packets never command hardware."""
from collections import deque
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('NOMAD_VIEWER_STATE_DIR', '/tmp/nomad_render_efficiency')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtGui, QtWidgets
import monitor
import launcher
from developer_planning import PlanningWindow
from module_logs import LogHistory, attach_module_logs, update_module_logs
from planning_client import display_event, planning_for_display, stage_summary
from render_utils import HistorySampler


def pose(n=0):
    return dict(t=100.+n, x=n*.001, y=0., z=0., qx=0., qy=0., qz=0., qw=1., frame_id='odom')


def grid():
    return dict(t=100., georeferenced=True, grid=dict(frame_id='odom', width=20, height=10,
        resolution=.1, image_y='high_to_low', origin=dict(x=-1., y=-.5, z=0., qx=0., qy=0., qz=0., qw=1.)))


def image():
    result = QtGui.QImage(20, 10, QtGui.QImage.Format_RGB888)
    result.fill(QtGui.QColor('green'))
    return result


class RenderEfficiencyTest(unittest.TestCase):
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

    def test_exact_sample_cap_endpoints_and_unchanged_cache(self):
        points = deque((pose(n) for n in range(10001)), maxlen=20000)
        sampler = HistorySampler(4000)
        sampled = sampler.sample(points)
        self.assertEqual(len(sampled), 4000)
        self.assertIs(sampled[0], points[0])
        self.assertIs(sampled[-1], points[-1])
        self.assertIs(sampler.sample(points), sampled)
        points.append(pose(10001))
        self.assertIsNot(sampler.sample(points), sampled)
        self.assertEqual(len(sampler.points), 4000)

    def test_identical_map_pixels_with_new_capture_do_not_redraw_or_replace_pixels(self):
        panel = monitor.MapPanel('fixture')
        self.windows.append(panel)
        panel.set_image(image(), grid())
        panel.redraw()
        previous_pixels = panel.pixels
        panel.canvas.draw_idle = Mock()
        panel.set_image(image(), {**grid(), 't':101.})
        panel.redraw()
        self.assertIs(panel.pixels, previous_pixels)
        self.assertEqual(panel.event['t'], 101.)
        panel.canvas.draw_idle.assert_not_called()

    def test_main_plot_updates_only_changed_source_and_keeps_padded_limits(self):
        with patch.object(sys, 'argv', ['monitor', '--no-ssh']):
            window = monitor.MonitorWindow(monitor.parse_args())
        self.windows.append(window)
        window.timer.stop(); window.ingest_timer.stop()
        window.histories['vio'].append(pose())
        window.histories['public'].append(pose())
        window.update_plot()
        limits = window.ax.get_xlim(), window.ax.get_ylim(), window.ax.get_zlim()
        window.lines['vio'].set_data_3d = Mock()
        window.histories['public'].append(pose(1))
        window.update_plot()
        window.lines['vio'].set_data_3d.assert_not_called()
        self.assertEqual(limits, (window.ax.get_xlim(), window.ax.get_ylim(), window.ax.get_zlim()))

    def test_gpp_stays_visible_when_lpp_blocked_and_stale_path_is_explicit(self):
        report = display_event(dict(t=1794000000., frame='odom', state='WAIT', plan_only=False,
            path=[[0.,0.,0.],[1.,0.,0.]], trajectory=[], gpp_state='READY', gpp_reason='path clear',
            lpp_state='BLOCKED', lpp_reason='no valid RPP', path_valid=True, executable=False,
            gpp_stamp_ns=1793999999000000000))
        self.assertIs(planning_for_display(report, .2), report)
        stale = planning_for_display(report, 4.)
        self.assertEqual(stale['path'], report['path'])
        self.assertFalse(stale['path_valid'])
        self.assertFalse(stale['executable'])
        self.assertEqual(stale['gpp_state'], 'STALE')
        self.assertEqual(stale['gpp_stamp_ns'], 1793999999000000000)
        self.assertIn('기준 시각 1.0초 전', stage_summary(report)[0])
        self.assertIn('기준 시각 11.0초 전', stage_summary({**report, 't':report['t']+10.})[0])
        window = PlanningWindow(components={'grid_geometry':monitor.grid_geometry, 'rotate_vector':monitor.rotate_vector})
        self.windows.append(window)
        snapshot = dict(now=100., images={'costmap':image()}, image_meta={'costmap':grid()},
            received={'image:costmap':100., 'telemetry:planning':100.}, telemetry={'planning':report})
        window.update_snapshot(snapshot); window.map.redraw()
        self.assertIn('GPP · READY', window.gpp_status.text())
        self.assertIn('LPP · BLOCKED', window.lpp_status.text())
        self.assertEqual(list(window.map._global_line.get_xdata()), [0.,1.])
        self.assertEqual(window.map._global_line.get_linestyle(), '-')
        snapshot['now'] = 104.
        window.update_snapshot(snapshot); window.map.redraw()
        self.assertEqual(window.map._global_line.get_linestyle(), '--')
        self.assertIn('참고용', window.gpp_status.text())

    def test_log_display_batch_appends_text_once_and_unchanged_does_nothing(self):
        window = QtWidgets.QMainWindow(); self.windows.append(window)
        attach_module_logs(window)
        window.append_module_log = window.log_panel.append_event
        history = LogHistory()
        for seq in range(30):
            history.append(dict(board='pi', source='localization', session='fixture', seq=seq, text=f'line {seq}'))
        window.log_panel.text.appendPlainText = Mock(wraps=window.log_panel.text.appendPlainText)
        snapshot = {'module_logs':{'localization':history}}
        update_module_logs(window, snapshot, 'localization')
        self.assertEqual(window.log_panel.text.appendPlainText.call_count, 1)
        update_module_logs(window, snapshot, 'localization')
        self.assertEqual(window.log_panel.text.appendPlainText.call_count, 1)

    def test_launcher_does_not_request_automatic_screenshots(self):
        with tempfile.TemporaryDirectory() as temporary, patch.object(launcher, 'STATE_ROOT', Path(temporary)):
            window = launcher.Launcher(); self.windows.append(window)
            arguments = window.monitor_arguments(('user@192.0.2.1','nvidia@192.0.2.2'), Path(temporary))
            self.assertNotIn('--snapshot', arguments)
            self.assertNotIn('--snapshot-interval-s', arguments)
            window.connection_mode.setCurrentIndex(1)
            window.relay_host.setText('192.0.2.3:8768')
            arguments = window.monitor_arguments(None, Path(temporary))
            self.assertNotIn('--snapshot', arguments)
            self.assertNotIn('--snapshot-interval-s', arguments)


if __name__ == '__main__':
    unittest.main()
