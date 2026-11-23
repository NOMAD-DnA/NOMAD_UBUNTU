"""Log display delta updates and right-column layout; no board access."""
from collections import deque
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtWidgets
from module_logs import LogHistory, ModuleLogPanel, attach_module_logs, update_module_logs


def fixture(seq):
    return {"kind": "log", "board": "pi", "source": "localization", "session": "fixture",
            "seq": seq, "text": f"[vio] frame {seq}"}


class CountedDeque:
    def __init__(self, items):
        self.items = deque(items, maxlen=128)
        self.visited = 0

    def append(self, value):
        self.items.append(value)

    def __len__(self):
        return len(self.items)

    def __iter__(self):
        for item in self.items:
            self.visited += 1
            yield item

    def __reversed__(self):
        for item in reversed(self.items):
            self.visited += 1
            yield item


class LogHistoryTest(unittest.TestCase):
    def test_unchanged_is_constant_and_increment_visits_new_records_only(self):
        history = LogHistory(maxlen=128)
        for seq in range(128):
            history.append(fixture(seq))
        cursor, first = history.events_after(None)
        self.assertEqual(len(first), 128)
        counted = history._events = CountedDeque(history)
        for _ in range(100):
            self.assertEqual(history.events_after(cursor), (cursor, []))
        self.assertEqual(counted.visited, 0)
        newest = fixture(128)
        history.append(newest)
        cursor, new = history.events_after(cursor)
        self.assertEqual(new, [newest])
        self.assertIs(new[0], newest)
        self.assertEqual(counted.visited, 1)

    def test_evicted_records_are_skipped_and_sequence_access_remains_available(self):
        history = LogHistory(maxlen=3)
        for seq in range(8):
            history.append(fixture(seq))
        cursor, records = history.events_after(1)
        self.assertEqual(cursor, 8)
        self.assertEqual([e['seq'] for e in records], [5, 6, 7])
        self.assertEqual(len(history), 3)
        self.assertEqual(history[-1]['seq'], 7)
        self.assertEqual(history[:2], list(history)[:2])


class LogPanelDeltaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_right_column_and_repeated_snapshot_never_replays_history(self):
        window = QtWidgets.QMainWindow()
        attach_module_logs(window)
        window.append_module_log = Mock(side_effect=window.log_panel.append_event)
        history = LogHistory(maxlen=128)
        for seq in range(128):
            history.append(fixture(seq))
        snapshot = {'module_logs': {'localization': history}}
        with patch('module_logs.time.monotonic', return_value=10.):
            update_module_logs(window, snapshot, 'localization')
        self.assertEqual(window.append_module_log.call_count, 128)
        self.assertEqual(window.dockWidgetArea(window.log_dock), QtCore.Qt.RightDockWidgetArea)
        self.assertEqual(window.log_dock.minimumWidth(), 260)
        window.append_module_log.reset_mock()
        with patch('module_logs.time.monotonic', return_value=10.2):
            for _ in range(50):
                update_module_logs(window, snapshot, 'localization')
        window.append_module_log.assert_not_called()
        history.append(fixture(128))
        with patch('module_logs.time.monotonic', return_value=10.2):
            update_module_logs(window, snapshot, 'localization')
        window.append_module_log.assert_not_called()
        with patch('module_logs.time.monotonic', return_value=11.1):
            update_module_logs(window, snapshot, 'localization')
        window.append_module_log.assert_called_once_with(fixture(128))
        window.log_panel.clear()
        with patch('module_logs.time.monotonic', return_value=11.5):
            update_module_logs(window, snapshot, 'localization')
        self.assertEqual(window.log_panel.text.toPlainText(), '')
        history.append(fixture(129))
        with patch('module_logs.time.monotonic', return_value=12.2):
            update_module_logs(window, snapshot, 'localization')
        self.assertIn('frame 129', window.log_panel.text.toPlainText())
        self.assertNotIn('frame 128', window.log_panel.text.toPlainText())
        window.close()
        window.deleteLater()
        self.app.processEvents()

    def test_manual_scroll_up_is_held_until_view_returns_to_bottom(self):
        panel = ModuleLogPanel(max_lines=800)
        panel.resize(400, 220)
        panel.show()
        for seq in range(100):
            panel.append_event(fixture(seq))
        self.app.processEvents()
        scroll = panel.text.verticalScrollBar()
        self.assertGreater(scroll.maximum(), 10)
        scroll.setValue(scroll.maximum()-10)
        previous = scroll.value()
        self.assertTrue(panel.auto_scroll.isChecked())
        panel.append_event(fixture(100))
        self.app.processEvents()
        self.assertEqual(scroll.value(), previous)
        self.assertLess(scroll.value(), scroll.maximum())
        scroll.setValue(scroll.maximum())
        panel.append_event(fixture(101))
        self.app.processEvents()
        self.assertEqual(scroll.value(), scroll.maximum())
        panel.close()
        panel.deleteLater()
        self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
