"""Exercise the actual spectator UI over a loopback TCP relay, with no boards."""
import base64
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
_state = tempfile.TemporaryDirectory(prefix='nomad-windows-test-')
os.environ['NOMAD_VIEWER_STATE_DIR'] = _state.name
os.environ['XDG_STATE_HOME'] = _state.name
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore, QtGui, QtWidgets
import monitor
from relay import RelayServer
from windows_spectator import connection_args, spectator_args


class WindowsSpectatorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_demo_skips_dialog_and_ignores_saved_manual_address(self):
        settings = mock.Mock()
        settings.value.return_value = '192.0.2.123:9999'
        with mock.patch.object(QtWidgets.QInputDialog, 'getText', side_effect=AssertionError('Demo must not prompt')):
            args = connection_args(settings, demo=True)
        self.assertEqual(args.relay_host, '10.42.0.1:8768')
        self.assertTrue(args.no_ssh)
        self.assertFalse(args.enable_trial_control)
        settings.value.assert_not_called()
        settings.setValue.assert_not_called()

    def test_manual_address_and_cancel_still_work(self):
        settings = mock.Mock()
        settings.value.return_value = '192.0.2.123:9999'
        with mock.patch.object(QtWidgets.QInputDialog, 'getText', return_value=('192.0.2.2:8888', True)):
            self.assertEqual(connection_args(settings).relay_host, '192.0.2.2:8888')
        settings.setValue.assert_called_once_with('relay', '192.0.2.2:8888')
        with mock.patch.object(QtWidgets.QInputDialog, 'getText', return_value=('', False)):
            self.assertIsNone(connection_args(settings))

    def test_forces_readonly_and_does_not_forward_operator_options(self):
        with mock.patch.object(sys, 'argv', ['viewer', '--enable-trial-control', '--relay-listen', '8768']):
            args = spectator_args('127.0.0.1')
            self.assertEqual(sys.argv[1], '--enable-trial-control')
        self.assertEqual(args.relay_host, '127.0.0.1:8768')
        self.assertTrue(args.no_ssh)
        self.assertFalse(args.enable_trial_control)
        self.assertIsNone(args.relay_listen)
        self.assertEqual(args.pi_command, '')
        self.assertEqual(args.jetson_command, '')
        for invalid in ('', 'host:0', 'host:65536', 'a b:8768', '::1:8768'):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                spectator_args(invalid)

    def test_receives_pose_image_and_status_via_actual_tcp_and_closes_cleanly(self):
        server = RelayServer(host='127.0.0.1', port=0)
        server.start()
        self.addCleanup(server.close)
        address = server.status()
        args = spectator_args('127.0.0.1:' + str(address['port']))
        with mock.patch.object(monitor, 'Collector', side_effect=AssertionError('No SSH')), \
                mock.patch('control.run_remote', side_effect=AssertionError('No remote commands')):
            window = monitor.MonitorWindow(args)
            try:
                window.show()
                image = QtGui.QImage(20, 16, QtGui.QImage.Format_RGB888)
                image.fill(QtGui.QColor('#405060'))
                buffer = QtCore.QBuffer()
                buffer.open(QtCore.QIODevice.WriteOnly)
                self.assertTrue(image.save(buffer, 'PNG'))
                now = time.time()
                events = [('pi', {'kind': 'pose', 'source': 'public', 't': now, 'x': 1., 'y': 2., 'z': 0.,
                                  'qx': 0., 'qy': 0., 'qz': 0., 'qw': 1., 'frame_id': 'odom'}),
                          ('jetson', {'kind': 'image', 'source': 'rgb', 't': now, 'width': 20, 'height': 16,
                                      'encoding': 'png', 'data': base64.b64encode(bytes(buffer.data())).decode()}),
                          ('pi', {'kind': 'status', 'source': 'localization', 't': now, 'state': 'READY'})]
                for board, event in events:
                    self.assertTrue(server.publish(board, event))
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    self.app.processEvents()
                    if window.histories['public'] and 'rgb' in window.latest_qimages and window.statuses:
                        break
                    time.sleep(.02)
                self.assertTrue(window.histories['public'])
                self.assertIn('rgb', window.latest_qimages)
                self.assertTrue(window.statuses)
                self.assertEqual(window.workers, [])
                self.assertFalse(window.state()['trial_control']['enabled'])
                self.assertFalse(window.panels['costmap'].goal_selection_enabled)
                for part in ('localization', 'perception', 'traversability', 'planning', 'control', 'resources'):
                    window.open_developer_window(part)
                self.app.processEvents()
                self.assertEqual(window.workers, [])
                if os.environ.get('NOMAD_TEST_SNAPSHOT'):
                    self.assertTrue(window.grab().save(os.environ['NOMAD_TEST_SNAPSHOT']))
            finally:
                window.close()
                self.app.processEvents()
            self.assertFalse(window.relay_receiver.isRunning())


if __name__ == '__main__':
    unittest.main()
