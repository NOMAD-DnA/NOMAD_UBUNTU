"""기록 열기: shared folder shown when a local copy can reach it (2026-11-12)."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
from PyQt5 import QtWidgets
import local_install as li
import session_ui


class RecordingOpenSharedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.local = root / 'local'
        self.local.mkdir()
        self.source = root / 'share/NOMAD/app'
        (self.source / 'laptop_monitor').mkdir(parents=True)
        (self.source / 'laptop_monitor/launcher.py').write_text('')
        self.shared = self.source.parent / '주행기록'
        self.shared.mkdir()

    def tearDown(self):
        self.tmp.cleanup()

    def run_dialog(self, shared):
        seen = {}

        def fake_exec(dialog):
            seen['dir'] = dialog.directory().absolutePath()
            seen['sidebar'] = [u.toLocalFile() for u in dialog.sidebarUrls()]
            seen['native'] = not dialog.testOption(QtWidgets.QFileDialog.DontUseNativeDialog)
            seen['title'] = dialog.windowTitle()
            buttons = {b.text(): b for b in dialog.findChildren(QtWidgets.QPushButton)}
            if '이 노트북 주행기록' in buttons:
                buttons['이 노트북 주행기록'].click()
                seen['after_click'] = dialog.directory().absolutePath()
            return QtWidgets.QDialog.Rejected
        with patch.object(session_ui, 'recording_directory', return_value=self.local), \
                patch.object(QtWidgets.QFileDialog, 'exec_', fake_exec):
            self.assertEqual(session_ui.choose_recording(None, shared=shared), '')
        return seen

    def test_shared_connected_opens_shared_and_lists_both(self):
        seen = self.run_dialog(self.shared)
        self.assertEqual(seen['dir'], str(self.shared))
        self.assertEqual(seen['sidebar'], [str(self.shared), str(self.local)])
        self.assertFalse(seen['native'])
        self.assertIn('공유폴더', seen['title'])
        self.assertEqual(seen['after_click'], str(self.local))

    def test_not_connected_keeps_local(self):
        with patch.object(session_ui, 'shared_recordings_folder', return_value=None), \
                patch.object(session_ui, 'recording_directory', return_value=self.local), \
                patch.object(QtWidgets.QFileDialog, 'getOpenFileName', return_value=('', '')) as chooser:
            self.assertEqual(session_ui.choose_recording(None), '')
        self.assertEqual(chooser.call_args[0][2], str(self.local))

    def test_reachable_shared_recordings(self):
        info = {'source': str(self.source)}
        self.assertEqual(li.reachable_shared_recordings(info), self.shared)
        self.assertIsNone(li.reachable_shared_recordings({'source': str(Path(self.tmp.name) / 'gone')}))
        import share_mount
        with patch.object(share_mount, 'find_mounted', return_value=(self.source, '10.42.0.1')):
            self.assertEqual(li.reachable_shared_recordings({'source': '/gone'}), self.shared)

    def test_launcher_button_uses_shared_chooser(self):
        import launcher
        seen = []
        with tempfile.TemporaryDirectory() as state, patch.object(launcher, 'STATE_ROOT', Path(state)), \
                patch.object(session_ui, 'shared_recordings_folder', return_value=self.shared), \
                patch.object(session_ui, 'recording_directory', return_value=self.local), \
                patch.object(QtWidgets.QFileDialog, 'exec_',
                             lambda d: seen.append(d.directory().absolutePath()) or QtWidgets.QDialog.Rejected):
            w = launcher.Launcher()
            w.open_recording()
            w.deleteLater()
        self.assertEqual(seen, [str(self.shared)])

    def test_not_a_local_copy(self):
        with patch.object(li, 'installed_info', return_value=None):
            self.assertIsNone(li.reachable_shared_recordings())


if __name__ == '__main__':
    unittest.main()
