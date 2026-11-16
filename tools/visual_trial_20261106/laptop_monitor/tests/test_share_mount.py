"""Shared-folder finding/mounting for local GUI copies (no network, fake gio)."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
import share_mount as s

CONFIG = {'share': 'shared', 'servers': ['lab.local', '10.42.0.1'], 'relative': 'A/B', 'user': 'user',
          'domain': 'WORKGROUP'}


class ShareMountTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.gvfs = Path(self.tmp.name) / 'gvfs'
        self.gvfs.mkdir()
        self.calls = []
        self.patches = [patch.object(s, 'gvfs_root', return_value=self.gvfs),
                        patch.object(s, 'load_config', return_value=dict(CONFIG)),
                        patch.object(s, 'keyring_lookup', return_value=None),
                        patch.object(s, 'keyring_store', side_effect=lambda c, p: self.calls.append(('store', p)) or True)]
        for p in self.patches:
            p.start()

    def tearDown(self):
        for p in self.patches:
            p.stop()
        self.tmp.cleanup()

    def make_mount(self, server, with_source=True):
        base = self.gvfs / 'smb-share:server={},share=shared'.format(server)
        if with_source:
            (base / 'A/B/laptop_monitor').mkdir(parents=True)
            (base / 'A/B/laptop_monitor/launcher.py').write_text('')
        else:
            base.mkdir()
        return base

    def fake_run(self, reachable=('10.42.0.1',), password_ok='secret', has_source=True):
        def run(args, timeout, stdin_text=None):
            self.calls.append(tuple(args[:2]))
            if args[0] == 'stat':
                return (0, '') if Path(args[2]).exists() else (1, '')
            if args[0] == 'which':
                return 0, '/usr/bin/gio'
            if args[0] == 'getent':
                return (0, '10.42.0.1 STREAM x\n') if args[2] in reachable else (2, '')
            if args[:2] == ['gio', 'mount']:
                server = args[2].split('/')[2]
                user, domain, password = stdin_text.split('\n')[:3]
                self.calls.append(('stdin_user', user))
                if password == password_ok:
                    self.make_mount(server, has_source)
                    return 0, ''
                return 2, 'Authentication Required\nPassword: '
            raise AssertionError(args)
        return run

    def test_existing_mount_needs_no_password(self):
        base = self.make_mount('10.42.0.1')
        with patch.object(s, '_run', self.fake_run()):
            result = s.connect(interactive=False)
        self.assertTrue(result['ok'])
        self.assertEqual(result['source'], str(base / 'A/B'))
        self.assertNotIn(('gio', 'mount'), self.calls)

    def test_unreachable_does_not_ask(self):
        with patch.object(s, '_run', self.fake_run(reachable=())), patch.object(s, 'reachable', return_value=False):
            result = s.resolve_source({'source': '/nonexistent'}, ask_password=lambda r: self.fail('asked'))
        self.assertEqual(result['reason'], 'unreachable')

    def test_password_asked_once_mounted_and_saved(self):
        asked = []
        with patch.object(s, '_run', self.fake_run()), patch.object(s, 'reachable', side_effect=lambda h: h == '10.42.0.1'):
            result = s.resolve_source({'source': '/nonexistent'},
                                      ask_password=lambda r: asked.append(r['server']) or ('secret', True))
        self.assertEqual(asked, ['10.42.0.1'])
        self.assertTrue(result['ok'])
        self.assertIn('server=10.42.0.1,share=shared', result['source'])
        self.assertIn(('store', 'secret'), self.calls)
        self.assertIn(('stdin_user', 'user'), self.calls)

    def test_wrong_password_not_saved(self):
        with patch.object(s, '_run', self.fake_run()), patch.object(s, 'reachable', return_value=True):
            result = s.connect(password='bad', save=True)
        self.assertEqual(result['reason'], 'auth_failed')
        self.assertNotIn(('store', 'bad'), self.calls)
        # stops after the first auth failure instead of trying every address
        self.assertEqual(sum(1 for c in self.calls if c == ('gio', 'mount')), 1)

    def test_keyring_password_used_silently(self):
        with patch.object(s, '_run', self.fake_run()), patch.object(s, 'reachable', return_value=True), \
                patch.object(s, 'keyring_lookup', return_value='secret'):
            result = s.resolve_source({'source': '/nonexistent'}, ask_password=lambda r: self.fail('asked'))
        self.assertTrue(result['ok'])
        self.assertIn('server=lab.local', result['source'])

    def test_recorded_source_preferred(self):
        recorded = Path(self.tmp.name) / 'team-shared/A/B'
        (recorded / 'laptop_monitor').mkdir(parents=True)
        (recorded / 'laptop_monitor/launcher.py').write_text('')
        with patch.object(s, '_run', self.fake_run()):
            result = s.resolve_source({'source': str(recorded)})
        self.assertEqual((result['reason'], result['source']), ('local_source', str(recorded)))

    def test_share_without_gui_folder(self):
        with patch.object(s, '_run', self.fake_run(has_source=False)), patch.object(s, 'reachable', return_value=True):
            result = s.connect(password='secret')
        self.assertEqual(result['reason'], 'folder_missing')

    def test_password_with_newline_rejected(self):
        self.assertEqual(s.mount('x', CONFIG, 'a\nb'), (False, 'password_needed'))

    def test_mounted_path_case_insensitive(self):
        base = self.gvfs / 'smb-share:server=lab.local,share=shared'
        base.mkdir()
        self.assertEqual(s.mounted_path('LAB.local', 'Shared'), base)
        self.assertIsNone(s.mounted_path('other', 'shared'))


class ConfigTest(unittest.TestCase):
    def test_shipped_config(self):
        value = s.load_config()
        self.assertEqual(value['share'], 'shared')
        self.assertIn('10.42.0.1', value['servers'])
        self.assertTrue(value['relative'].endswith('NOMAD_실물시험앱_20261106'))

    def test_bad_config(self):
        with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
            json.dump({'share': 'shared', 'servers': []}, f)
            f.flush()
            self.assertIsNone(s.load_config(f.name))


if __name__ == '__main__':
    unittest.main()
