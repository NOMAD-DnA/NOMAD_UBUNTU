"""Control display contracts using local fixtures only; no board/output access."""
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('NOMAD_VIEWER_STATE_DIR', '/tmp/nomad_control_developer_test')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtWidgets
from developer_control import ControlWindow, trace_from_line
from module_logs import LogHistory
import monitor


def trace(stamp=None, fields=None):
    stamp = time.time() if stamp is None else stamp
    values = fields or ('source=LPP phase=STOPPED_OUTPUT requested_erpm=206.8 '
                       'target_erpm=0.0 output_erpm=0.0 measured_erpm=-1.0 '
                       'current_a=0.02 start_gate=planning target below minimum start ERPM; '
                       'target not increased reset_required=0')
    return f'[controller] [INFO] [{stamp:.9f}] [nomad_vesc_trial]: control_trace {values}'


def log(text, seq=1, session='fixture'):
    return dict(kind='log', board='pi4', source='planning', t=time.time(),
                seq=seq, session=session, text=text)


class ControlDeveloperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.windows = []

    def tearDown(self):
        for window in self.windows:
            window.close()
        self.app.processEvents()

    def child(self):
        child = ControlWindow()
        self.windows.append(child)
        return child

    def test_open_reopen_and_planning_share_one_reader_without_commands(self):
        with patch.object(sys, 'argv', ['monitor', '--no-ssh']), \
                patch('module_log_client.ModuleLogClient') as reader:
            window = monitor.MonitorWindow(monitor.parse_args())
            self.windows.append(window)
            window.timer.stop()
            window.ingest_timer.stop()
            command = Mock()
            window.planning_client = type('Client', (), {'send': command})()
            window.on_packet('pi4', log(trace()))
            window.open_developer_window('control')
            child = window.developer_windows['control']
            window.open_developer_window('planning')
            child.close()
            window.open_developer_window('control')
            self.assertIs(child, window.developer_windows['control'])
            self.assertEqual(child.values['requested_erpm'].text(), '206.8')
            self.assertIs(child.log_dock.parent(), child)
            self.assertIn('control_trace', child.log_panel.text.toPlainText())
            command.assert_not_called()
            reader.assert_not_called()
            self.assertEqual(window.workers, [])
            window.planning_client = None

    def test_controller_values_are_atomic_and_logs_filter_other_modules(self):
        child = self.child()
        history = LogHistory()
        history.append(log('[planner] unrelated planner line\n' + trace() +
                           '\n[driver] connected to VESC\n[log] helper note'))
        snapshot = dict(module_logs={'planning': history}, now=time.monotonic())
        child.update_snapshot(snapshot)
        self.assertEqual(child.values['measured_erpm'].text(), '-1.0')
        self.assertEqual(child.values['reset_required'].text(), '0')
        self.assertIn('target not increased', child.values['start_gate'].text())
        text = child.log_panel.text.toPlainText()
        self.assertIn('[driver] connected', text)
        self.assertNotIn('unrelated planner', text)
        self.assertNotIn('helper note', text)
        self.assertIsNone(child.servo.command)
        self.assertFalse(child.servo.isEnabled())
        # A later trace with missing fields cannot mix old and new samples.
        child.append_module_log(log(trace(time.time()+.01, 'phase=IDLE requested_erpm=5'), seq=2))
        child.update_snapshot({})
        self.assertEqual(child.values['requested_erpm'].text(), '5.0')
        self.assertEqual(child.values['measured_erpm'].text(), '미수신')
        self.assertEqual(child.values['reset_required'].text(), '미수신')

    def test_old_original_timestamp_is_not_made_fresh_by_new_receipt(self):
        child = self.child()
        child.append_module_log(log(trace(time.time()-45)))
        child.update_snapshot({})
        self.assertIn('오래된 값', child.trace_status.text())
        # A backward timestamp within a session never replaces the newer trace.
        child.append_module_log(log(trace(time.time()-60, 'requested_erpm=900'), seq=2))
        child.update_snapshot({})
        self.assertEqual(child.values['requested_erpm'].text(), '206.8')
        child.append_module_log(log('[controller] mode=commission; controller startup',
                                    seq=1, session='new-session'))
        child.update_snapshot({})
        self.assertEqual(child.values['requested_erpm'].text(), '미수신')

    def test_permission_is_received_not_inferred_from_trace_or_path(self):
        child = self.child()
        child.append_module_log(log(trace()))
        child.update_snapshot({})
        self.assertEqual(child.telemetry_values['armed'].text(), '미수신')
        received = time.monotonic()-8
        child.update_snapshot({'now': time.monotonic(),
             'received': {'telemetry:planning': received},
             'planning_ui': {'spectator': True, 'host': 'user@192.0.2.1'},
             'telemetry': {'planning': {'armed': True, 'actuator_enabled': False,
                 'state': 'WAIT', 'output_inhibit': 'LOCALIZATION_NOT_READY'}}})
        self.assertIn('관전', child.host.text())
        self.assertIn('오래된 상태', child.status.text())
        self.assertEqual(child.telemetry_values['armed'].text(), 'true')
        self.assertEqual(child.telemetry_values['actuator_enabled'].text(), 'false')
        self.assertEqual(child.telemetry_values['output_inhibit'].text(), 'LOCALIZATION_NOT_READY')
        self.assertEqual(child.findChildren(QtWidgets.QPushButton)[0].text(), '비우기')
        self.assertEqual(len(child.findChildren(QtWidgets.QPushButton)), 1)

    def test_log_display_is_incremental_and_limited_to_one_update_per_second(self):
        child = self.child()
        history = LogHistory()
        history.append(log(trace()))
        snapshot = {'module_logs': {'planning': history}}
        with patch('module_logs.time.monotonic', return_value=100.):
            child.update_snapshot(snapshot)
        initial = child.log_panel.text.toPlainText()
        history.append(log('[controller] later row', seq=2))
        with patch('module_logs.time.monotonic', return_value=100.5):
            child.update_snapshot(snapshot)
        self.assertEqual(child.log_panel.text.toPlainText(), initial)
        with patch('module_logs.time.monotonic', return_value=101.1):
            child.update_snapshot(snapshot)
        self.assertEqual(child.log_panel.text.toPlainText().count('later row'), 1)
        with patch('module_logs.time.monotonic', return_value=102.2):
            child.update_snapshot(snapshot)
        self.assertEqual(child.log_panel.text.toPlainText().count('later row'), 1)


if __name__ == '__main__':
    unittest.main()
