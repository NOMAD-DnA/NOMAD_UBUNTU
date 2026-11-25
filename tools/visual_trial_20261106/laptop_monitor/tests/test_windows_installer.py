"""Verify the shipped CMD payload in isolation from the shared source tree."""
import base64
import io
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile


class WindowsInstallerTest(unittest.TestCase):
    def test_extracted_installer_opens_spectator_and_developer_windows(self):
        gui = Path(__file__).resolve().parents[1]
        installer = gui.parent.parent / 'NOMAD_Windows_Demo_Setup.cmd'
        parts = re.split(rb'\r?\n:NOMAD_PAYLOAD\r?\n', installer.read_bytes(), maxsplit=1)
        self.assertEqual(len(parts), 2)
        with tempfile.TemporaryDirectory(prefix='nomad-installed-app-test-') as stage:
            with zipfile.ZipFile(io.BytesIO(base64.b64decode(parts[1]))) as archive:
                self.assertIsNone(archive.testzip())
                archive.extractall(stage)
            installed = Path(stage) / 'app/laptop_monitor'
            tests = installed / 'tests'
            tests.mkdir()
            shutil.copy2(gui / 'tests/test_windows_spectator.py', tests)
            shutil.copy2(gui / 'tests/test_planner_observer_sync.py', tests)
            env = os.environ.copy()
            env.pop('PYTHONPATH', None)
            env['QT_QPA_PLATFORM'] = 'offscreen'
            env['LOCALAPPDATA'] = str(Path(stage) / 'user')
            result = subprocess.run(
                [sys.executable, '-m', 'unittest', 'discover', '-s', str(tests), '-p', 'test_*.py', '-v'],
                cwd=stage, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout)
            self.assertIn('Ran 9 tests', result.stdout)
            result = subprocess.run(
                [sys.executable, str(installed / 'windows_spectator.py'), '--check-installation'],
                cwd=stage, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout)


if __name__ == '__main__':
    unittest.main()
