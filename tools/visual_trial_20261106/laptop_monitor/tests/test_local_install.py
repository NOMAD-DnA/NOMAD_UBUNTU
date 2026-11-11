"""Shared source -> ~/nomad_gui local copy. Uses temporary copies only."""
import json, os, shutil, subprocess, sys, tempfile, time, unittest
from pathlib import Path
from unittest import mock

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
import local_install as li


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        base = Path(self.tmp.name)
        # Minimal shared tree: integration root / app dir / {laptop_monitor, config}
        self.integration = base / 'NOMAD_모듈별_통합_20261104'
        self.source = self.integration / 'NOMAD_실물시험앱_20261106'
        shutil.copytree(str(APP), str(self.source / 'laptop_monitor'),
                        ignore=shutil.ignore_patterns('__pycache__', 'tests'))
        shutil.copytree(str(APP.parent / 'config'), str(self.source / 'config'))
        shutil.copy2(str(APP.parent / 'GUI_설치_업데이트.sh'), str(self.source / 'GUI_설치_업데이트.sh'))
        (self.integration / '주행기록').mkdir()
        self.home = base / 'home'
        self.home.mkdir()
        self.env = dict(os.environ, HOME=str(self.home), XDG_DATA_HOME=str(self.home / '.local/share'))
        self.env.pop('NOMAD_GUI_HOME', None)
        self.env.pop('XDG_STATE_HOME', None)

    def tearDown(self):
        self.tmp.cleanup()

    def run_installer(self):
        return subprocess.run(['bash', str(self.source / 'GUI_설치_업데이트.sh')], env=self.env,
                              capture_output=True, text=True, timeout=60)

    def test_install_creates_versioned_copy_marker_and_app_entry(self):
        result = self.run_installer()
        self.assertEqual(result.returncode, 0, result.stderr)
        root = self.home / 'nomad_gui'
        current = (root / 'current').resolve()
        marker = json.loads((current / li.MARKER).read_text())
        self.assertEqual(marker['source'], str(self.source))
        self.assertEqual(marker['version'], li.tree_version(self.source))
        self.assertTrue((current / 'laptop_monitor' / 'launcher.py').is_file())
        self.assertTrue((current / 'config').is_dir())
        self.assertIn('수정 금지', (root / 'AGENTS.md').read_text())
        entry = (self.home / '.local/share/applications' / li.DESKTOP_ID).read_text()
        self.assertIn(str(root / 'current' / 'laptop_monitor' / 'run_app.sh'), entry)
        self.assertTrue((root / '주행기록').is_dir())

    def test_reinstall_same_content_keeps_version_and_change_makes_new_one(self):
        self.run_installer()
        first = (self.home / 'nomad_gui' / 'current').resolve().name
        self.run_installer()
        self.assertEqual((self.home / 'nomad_gui' / 'current').resolve().name, first)
        (self.source / 'laptop_monitor' / 'README.md').write_text('changed\n')
        self.run_installer()
        second = (self.home / 'nomad_gui' / 'current').resolve().name
        self.assertNotEqual(first, second)
        self.assertTrue((self.home / 'nomad_gui' / 'versions' / first).is_dir())  # old copy kept for a running GUI

    def test_only_three_versions_kept(self):
        for i in range(5):
            (self.source / 'laptop_monitor' / 'README.md').write_text('v%d\n' % i)
            self.run_installer()
            time.sleep(0.02)
        versions = [p for p in (self.home / 'nomad_gui' / 'versions').iterdir() if not p.name.startswith('.')]
        self.assertEqual(len(versions), 3)

    def test_local_copy_refuses_to_install_itself(self):
        self.run_installer()
        copy = (self.home / 'nomad_gui' / 'current').resolve()
        result = subprocess.run([sys.executable, '-I', str(copy / 'laptop_monitor' / 'local_install.py'), 'install',
                                 '--source', str(copy)], env=self.env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)

    def test_local_recordings_and_update_detection(self):
        self.run_installer()
        copy = (self.home / 'nomad_gui' / 'current').resolve() / 'laptop_monitor'
        code = ('import sys,json;sys.path.insert(0,sys.argv[1]);import session_recording,local_install as l;'
                'i=l.installed_info();print(json.dumps([str(session_recording.recording_directory()),'
                'l.update_available(i)]))')
        out = subprocess.run([sys.executable, '-I', '-c', code, str(copy)], env=self.env, capture_output=True,
                             text=True, cwd=str(copy))
        recording, newer = json.loads(out.stdout)
        self.assertEqual(recording, str(self.home / 'nomad_gui' / '주행기록'))
        self.assertIsNone(newer)
        (self.source / 'laptop_monitor' / 'README.md').write_text('newer\n')
        out = subprocess.run([sys.executable, '-I', '-c', code, str(copy)], env=self.env, capture_output=True,
                             text=True, cwd=str(copy))
        self.assertEqual(json.loads(out.stdout)[1], li.tree_version(self.source))

    def test_unreachable_source_means_no_update_question(self):
        info = {'feature': li.FEATURE, 'version': 'aaaaaaaaaaaa', 'source': str(self.source / 'missing'),
                'root': str(self.home / 'nomad_gui')}
        self.assertIsNone(li.update_available(info))
        with mock.patch.object(li, '_bounded', return_value=(None, '')):
            self.assertIsNone(li.source_version(self.source))

    def test_shared_folder_run_uses_shared_recordings(self):
        out = subprocess.run([sys.executable, '-I', '-c',
                              'import sys;sys.path.insert(0,sys.argv[1]);import session_recording;'
                              'print(session_recording.recording_directory())', str(self.source / 'laptop_monitor')],
                             env=self.env, capture_output=True, text=True)
        self.assertEqual(out.stdout.strip(), str(self.integration / '주행기록'))

    def test_upload_copies_finished_recordings_only(self):
        self.run_installer()
        info = li.installed_info((self.home / 'nomad_gui' / 'current').resolve() / 'laptop_monitor')
        local = li.local_recordings(info)
        old = local / 'a.nomad'
        old.write_bytes(b'x' * 10)
        os.utime(str(old), (time.time() - 600, time.time() - 600))
        busy = local / 'b.nomad'
        busy.write_bytes(b'y')
        os.utime(str(busy), (time.time() - 600, time.time() - 600))
        (local / 'b.nomad-wal').write_bytes(b'')
        fresh = local / 'c.nomad'
        fresh.write_bytes(b'z')
        pending = li.pending_uploads(info)
        self.assertEqual([p.name for p in pending], ['a.nomad'])
        self.assertEqual(li.upload(info, pending), ['a.nomad'])
        self.assertTrue((self.integration / '주행기록' / 'a.nomad').is_file())
        self.assertTrue(old.is_file())
        self.assertEqual(li.pending_uploads(info), [])


if __name__ == '__main__':
    unittest.main()
