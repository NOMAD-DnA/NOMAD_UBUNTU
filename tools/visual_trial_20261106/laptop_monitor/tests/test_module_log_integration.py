"""Log routing/relay/display contracts; no boards or vehicle modules."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('NOMAD_VIEWER_STATE_DIR', '/tmp/nomad_log_integration_test')
import sys
import time
from pathlib import Path
import unittest
from unittest import mock
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
import monitor
from relay import LatestPacketBuffer, RelayClient


class ModuleLogsIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with mock.patch.object(sys, 'argv', ['monitor', '--no-ssh']):
            self.window = monitor.MonitorWindow(monitor.parse_args())
        self.window.timer.stop()
        self.window.ingest_timer.stop()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def log(self, part='localization', seq=1, text='[vio] actual stdout: tracking 정상'):
        return dict(kind='log', source=part, t=time.time(), seq=seq, session='fixture', text=text)

    def test_each_window_receives_its_log_group_without_extra_board_reader(self):
        for part, board in [('localization', 'pi'), ('perception', 'jetson'),
                            ('traversability', 'jetson'), ('planning', 'pi4')]:
            self.window.on_packet(board, self.log(part, text='['+part+'] original stdout'))
            self.window.open_developer_window(part)
            panel = self.window.developer_windows[part].log_panel
            self.assertIn('['+part+'] original stdout', panel.text.toPlainText())
        self.assertEqual(self.window.log_workers, {})

    def test_wrong_owner_and_oversized_logs_rejected_history_bounded(self):
        self.window.on_packet('pi4', self.log())
        self.assertEqual(len(self.window.module_logs['localization']), 0)
        for seq in range(200):
            self.window.on_packet('pi', self.log(seq=seq))
        self.assertEqual(len(self.window.module_logs['localization']), 128)
        before = self.window.accepted_events
        self.window.on_packet('pi', self.log(seq=201, text='x'*17000))
        self.assertEqual(self.window.accepted_events, before)

    def test_relay_keeps_real_log_and_receipt_age_without_board_access(self):
        import json
        event = self.log('planning', text='[controller] ready')
        delivered = []
        client = RelayClient('127.0.0.1', 1, lambda b, e: delivered.append((b, e)), lambda e: None)
        client._deliver(json.dumps({'board': 'pi4', 'event': {**event, '_relay_age_s': 2.}}).encode())
        self.assertEqual(len(delivered), 1)
        inbox = LatestPacketBuffer(clock=lambda: time.monotonic()-1.)
        self.assertTrue(inbox.put(*delivered[0]))
        board, packet = inbox.drain()[0]
        self.window.on_packet(board, packet)
        self.assertGreaterEqual(self.window.age('log', 'planning'), 3.)
        self.assertEqual(self.window.module_logs['planning'][-1]['text'], '[controller] ready')


if __name__ == '__main__':
    unittest.main()
