"""Operator UI state reaches multiple read-only spectators over real loopback TCP."""
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
_state = tempfile.TemporaryDirectory(prefix='nomad-planner-sync-')
os.environ['NOMAD_VIEWER_STATE_DIR'] = _state.name
os.environ['XDG_STATE_HOME'] = _state.name
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore, QtWidgets
import monitor
import relay


class FakePlanningClient(QtCore.QObject):
    packet = QtCore.pyqtSignal(dict)
    progress = QtCore.pyqtSignal(str)
    closed = QtCore.pyqtSignal()

    def __init__(self, *_args, **_kwargs):
        super().__init__()
        self.ready = False
        self.stopping = False

    def start(self):
        self.ready = True

    def stop(self):
        self.ready = False
        self.stopping = True
        self.closed.emit()

    def send(self, *_args, **_kwargs):
        raise AssertionError('No vehicle commands during observer tests')


class PlannerObserverSyncTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.windows = []
        self.addCleanup(self.close_windows)

    def close_windows(self):
        for window in reversed(self.windows):
            window.close()
        self.app.processEvents()

    def make_window(self, *args):
        with mock.patch.object(sys, 'argv', ['monitor', *args]):
            window = monitor.MonitorWindow(monitor.parse_args())
        self.windows.append(window)
        return window

    def wait_for(self, condition):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            self.app.processEvents()
            if condition():
                return
            time.sleep(.01)
        self.fail('Timed out waiting for mirrored planner UI')

    def test_operator_connect_close_and_late_join_follow_without_commands(self):
        # A real relay with a fake planner session; never access any board.
        operator = self.make_window('--no-ssh')
        server = relay.RelayServer('127.0.0.1', 0).start()
        operator.relay_server = server
        operator.publish_planning_ui(force=True)
        address = '127.0.0.1:' + str(server.port)
        first = self.make_window('--no-ssh', '--relay-host', address)
        self.wait_for(lambda: first.planning_connect.text() == '플래너 연결')
        operator.args.no_ssh = False
        with mock.patch('planning_client.PlanningClient', FakePlanningClient), \
                mock.patch.object(operator, 'ensure_log_reader'):
            operator.connect_planner()
            self.wait_for(lambda: first.planning_connect.text() == operator.planning_connect.text() == '플래너 종료')
            second = self.make_window('--no-ssh', '--relay-host', address)
            self.wait_for(lambda: second.planning_connect.text() == '플래너 종료')
            for observer in (first, second):
                observer.open_developer_window('planning')
                observer.refresh()
                self.app.processEvents()
                self.assertFalse(observer.planning_connect.isEnabled())
                self.assertTrue(observer.developer_snapshot()['planning_ui']['client_active'])
                self.assertEqual(observer.developer_windows['planning'].connect_button.text(), '플래너 종료')
                self.assertFalse(observer.developer_windows['planning'].connect_button.isEnabled())
                self.assertIsNone(observer.planning_client)
                observer.connect_planner()
                self.assertIsNone(observer.planning_client)
                self.assertFalse(observer.goal_send.isEnabled())
                self.assertFalse(observer.output_arm.isEnabled())
            operator.connect_planner()
            self.wait_for(lambda: all(w.planning_connect.text() == '플래너 연결' for w in (first, second)))
        operator.args.no_ssh = True

    def test_stale_and_disconnected_operator_state_is_unknown(self):
        observer = self.make_window('--no-ssh', '--relay-host', '127.0.0.1:1')
        observer.relay_receiver.stop()
        self.assertTrue(observer.relay_receiver.wait(2000))
        event = {'kind': 'telemetry', 'source': 'planning_ui', 't': time.time(),
                 'client_active': True, 'button_text': '플래너 종료'}
        self.assertTrue(observer.on_packet('pi4', event))
        observer.relay_progress = {'state': 'connected'}
        observer.refresh()
        self.assertEqual(observer.planning_connect.text(), '플래너 종료')
        observer.received['telemetry:planning_ui'] -= 4
        observer.refresh()
        self.assertEqual(observer.planning_connect.text(), '플래너 상태 미확인')
        self.assertTrue(observer.on_packet('pi4', {**event, 't': time.time()}))
        observer.relay_progress = {'state': 'disconnected'}
        observer.refresh()
        self.assertEqual(observer.planning_connect.text(), '플래너 상태 미확인')
        self.assertFalse(observer.planning_connect.isEnabled())

    def test_malformed_and_wrong_owner_ui_packets_are_rejected(self):
        server = relay.RelayServer('127.0.0.1', 0).start()
        self.addCleanup(server.close)
        event = {'kind': 'telemetry', 'source': 'planning_ui', 't': time.time(),
                 'client_active': True, 'button_text': '플래너 종료'}
        self.assertTrue(server.publish('pi4', event))
        self.assertFalse(server.publish('jetson', event))
        self.assertFalse(server.publish('pi4', {**event, 'client_active': 'true'}))
        self.assertFalse(server.publish('pi4', {**event, 'button_text': '플래너 연결'}))


if __name__ == '__main__':
    unittest.main()
