"""Board scan must also update the Pi4 (planner) address, not keep a stale saved one."""
import os, sys, tempfile, unittest
from unittest.mock import MagicMock, patch
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))
import launcher

SCAN = """Pi 5: ssh user@192.168.3.163
Jetson: ssh nvidia@192.168.1.152
Pi 4: ssh user@192.168.2.252
"""


class Tests(unittest.TestCase):
    def test_scan_output_includes_pi4(self):
        found = launcher.parse_discovery(SCAN)
        self.assertEqual(found['pi'], ['user@192.168.3.163'])
        self.assertEqual(found['jetson'], ['nvidia@192.168.1.152'])
        self.assertEqual(found['pi4'], ['user@192.168.2.252'])

    def test_choose_pi4_replaces_stale_hotspot_address(self):
        self.assertEqual(launcher.choose_pi4('user@10.42.0.14', ['user@192.168.2.252']), 'user@192.168.2.252')
        self.assertEqual(launcher.choose_pi4('user@192.168.2.252', ['user@10.0.0.9', 'user@192.168.2.252']),
                         'user@192.168.2.252')
        self.assertEqual(launcher.choose_pi4('user@lpp.local', ['user@10.0.0.9', 'user@10.0.0.8']), 'user@10.0.0.9')
        self.assertIsNone(launcher.choose_pi4('user@lpp.local', []))

    def test_discovery_handler_updates_network_pi4(self):
        from PyQt5 import QtWidgets, QtCore
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as tmp, patch.object(launcher, 'STATE_ROOT', Path(tmp)):
            w = launcher.Launcher()
            w.network_pi4 = 'user@10.42.0.14'  # stale hotspot address from an earlier session
            w.process, w.process_kind, w.output, w.pending_text, w.cancel_reason = MagicMock(), 'discover', SCAN, '', ''
            w.read_output = lambda: None
            w.process_finished(0, QtCore.QProcess.NormalExit)
            self.assertEqual(w.network_pi4, 'user@192.168.2.252')
            self.assertEqual(w.hosts['pi'].currentText(), 'user@192.168.3.163')
            self.assertIn('Pi4(플래닝): user@192.168.2.252', w.status.text())
            w.process, w.process_kind, w.output = MagicMock(), 'discover', 'Pi 5: ssh user@192.168.3.163\n'
            w.process_finished(0, QtCore.QProcess.NormalExit)
            self.assertEqual(w.network_pi4, 'user@192.168.2.252')
            self.assertIn('Pi4를 찾지 못해', w.status.text())
            w.close()


if __name__ == '__main__':
    unittest.main()
