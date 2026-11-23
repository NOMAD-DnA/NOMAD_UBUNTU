"""2026-11-16: board reads in settings dialogs run off the GUI thread (no app freeze)."""
import os
from pathlib import Path
import sys
import threading
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from PyQt5 import QtCore, QtWidgets  # noqa: E402
import module_settings  # noqa: E402


class _Probe:
    """Minimal stand-in exposing what _busy uses."""
    def __init__(self, parent):
        self.QtWidgets, self.QtCore = QtWidgets, QtCore
        self.dialog = QtWidgets.QDialog(parent)

    _busy = module_settings.SettingsDialog._busy


class BusyThreadTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_call_runs_off_gui_thread_and_events_keep_flowing(self):
        parent = QtWidgets.QWidget()
        probe = _Probe(parent)
        ticks = []
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: ticks.append(1))
        timer.start(10)
        seen = {}

        def slow():
            seen["thread"] = threading.current_thread() is not threading.main_thread()
            seen["disabled"] = not probe.dialog.isEnabled() and not parent.isEnabled()
            time.sleep(0.3)
            return 42

        self.assertEqual(probe._busy(slow), 42)
        timer.stop()
        self.assertTrue(seen["thread"])
        self.assertTrue(seen["disabled"])
        self.assertGreater(len(ticks), 5)  # the GUI event loop kept running during the call
        self.assertTrue(probe.dialog.isEnabled() and parent.isEnabled())

    def test_error_is_raised_on_caller_and_widgets_reenabled(self):
        parent = QtWidgets.QWidget()
        probe = _Probe(parent)

        def boom():
            raise ValueError("board unreachable")

        with self.assertRaises(ValueError):
            probe._busy(boom)
        self.assertTrue(probe.dialog.isEnabled() and parent.isEnabled())


if __name__ == "__main__":
    unittest.main()
