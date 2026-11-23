"""2026-11-16: 주행 기록 올리기 "0개 복사" — no cp -p, size check, mv -n / gio move fallback, reasons."""
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import local_install as li  # noqa: E402


class UploadRecordingsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        base = Path(self.tmp.name)
        (base / 'share' / 'src').mkdir(parents=True)
        self.remote = base / 'share' / '주행기록'
        self.remote.mkdir()
        self.local = base / 'home' / '주행기록'
        self.local.mkdir(parents=True)
        self.bin = base / 'bin'
        self.bin.mkdir()
        self.info = {'source': str(base / 'share' / 'src'), 'root': str(base / 'home')}

    def tearDown(self):
        self.tmp.cleanup()

    def recording(self, name='a.nomad', data=b'x' * 10):
        path = self.local / name
        path.write_bytes(data)
        os.utime(str(path), (time.time() - 600, time.time() - 600))
        return path

    def fake(self, name, body):
        path = self.bin / name
        path.write_text('#!/bin/bash\n' + body + '\n')
        path.chmod(0o755)

    def fake_path(self):
        return mock.patch.dict(os.environ, {'PATH': str(self.bin) + os.pathsep + os.environ.get('PATH', '')})

    def test_plain_copy_renames_and_keeps_local(self):
        path = self.recording()
        self.assertEqual(li.upload(self.info, [path]), (['a.nomad'], []))
        self.assertEqual((self.remote / 'a.nomad').read_bytes(), b'x' * 10)
        self.assertFalse((self.remote / 'a.nomad.part').exists())
        self.assertTrue(path.is_file())
        self.assertAlmostEqual((self.remote / 'a.nomad').stat().st_mtime, path.stat().st_mtime, delta=2)

    def test_symlinked_recording_is_not_pending(self):
        """2026-11-22: ~/nomad_gui/주행기록/vio12.nomad -> share original failed "크기 다름 (…/82)"."""
        target = Path(self.tmp.name) / 'share' / 'elsewhere.nomad'
        target.write_bytes(b'y' * 10)
        link = self.local / 'vio12.nomad'
        link.symlink_to(target)
        os.utime(str(link), (time.time() - 600, time.time() - 600), follow_symlinks=False)
        plain = self.recording()
        self.assertEqual(li.pending_uploads(self.info), [plain])

    def test_mv_failure_falls_back_to_gio_move(self):
        path = self.recording()
        self.fake('mv', 'echo "mv: Operation not supported" >&2; exit 1')
        self.fake('gio', 'shift; shift; shift; /bin/mv -- "$1" "$2"')
        with self.fake_path():
            self.assertEqual(li.upload(self.info, [path]), (['a.nomad'], []))
        self.assertTrue((self.remote / 'a.nomad').is_file())
        self.assertFalse((self.remote / 'a.nomad.part').exists())

    def test_rename_failure_is_reported_with_reason(self):
        path = self.recording()
        self.fake('mv', 'exit 1')
        self.fake('gio', 'echo "gio: 지원하지 않음" >&2; exit 1')
        with self.fake_path():
            done, failed = li.upload(self.info, [path])
        self.assertEqual(done, [])
        self.assertEqual(failed[0][0], 'a.nomad')
        self.assertIn('이름 바꾸기 실패', failed[0][1])
        self.assertIn('지원하지 않음', failed[0][1])
        self.assertLessEqual(len(failed[0][1]), 80)
        self.assertTrue((self.remote / 'a.nomad.part').is_file())

    def test_copy_failure_and_short_copy_are_failures(self):
        path = self.recording()
        self.fake('cp', 'echo "cp: preserving times: Operation not supported" >&2; exit 1')
        with self.fake_path():
            done, failed = li.upload(self.info, [path])
        self.assertEqual(done, [])
        self.assertIn('복사 실패', failed[0][1])
        self.fake('cp', 'head -c 3 -- "$2" > "$3"')
        with self.fake_path():
            done, failed = li.upload(self.info, [path])
        self.assertEqual(done, [])
        self.assertIn('크기 다름', failed[0][1])
        self.assertFalse((self.remote / 'a.nomad').exists())

    def test_existing_final_is_never_overwritten(self):
        path = self.recording()
        (self.remote / 'a.nomad').write_bytes(b'team')
        done, failed = li.upload(self.info, [path])
        self.assertEqual((done, failed), ([], [('a.nomad', '이미 있음')]))
        self.assertEqual((self.remote / 'a.nomad').read_bytes(), b'team')

    def test_remote_part_only_is_pending_and_same_size_is_renamed_without_copy(self):
        path = self.recording()
        (self.remote / 'a.nomad.part').write_bytes(b'x' * 10)
        self.assertEqual(li.pending_uploads(self.info), [path])
        self.fake('cp', 'echo "cp should not run" >&2; exit 1')
        with self.fake_path():
            self.assertEqual(li.upload(self.info, [path]), (['a.nomad'], []))
        self.assertFalse((self.remote / 'a.nomad.part').exists())
        self.assertEqual(li.pending_uploads(self.info), [])

    def test_remote_part_of_other_size_is_copied_again(self):
        path = self.recording()
        (self.remote / 'a.nomad.part').write_bytes(b'x' * 4)
        self.assertEqual(li.upload(self.info, [path]), (['a.nomad'], []))
        self.assertEqual((self.remote / 'a.nomad').read_bytes(), b'x' * 10)

    def test_timeout_is_a_failure(self):
        path = self.recording()
        with mock.patch.object(li, '_bounded_group', return_value=(None, '')):
            self.assertEqual(li.upload(self.info, [path]), ([], [('a.nomad', '시간 초과')]))

    def test_first_timeout_stops_the_rest(self):
        paths = [self.recording('a.nomad'), self.recording('b.nomad'), self.recording('c.nomad')]
        with mock.patch.object(li, '_bounded_group', return_value=(None, '')) as run:
            done, failed = li.upload(self.info, paths)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(done, [])
        self.assertEqual(failed, [('a.nomad', '시간 초과'), ('b.nomad', '중단: 앞 파일 시간 초과'),
                                  ('c.nomad', '중단: 앞 파일 시간 초과')])

    def test_total_deadline_limits_each_timeout_and_stops(self):
        paths = [self.recording('a.nomad'), self.recording('b.nomad')]
        clock = iter([0.0, 0.0, 400.0])
        with mock.patch.object(li.time, 'monotonic', lambda: next(clock)), \
                mock.patch.object(li, '_bounded_group', return_value=(0, 'ok')) as run:
            done, failed = li.upload(self.info, paths, timeout=120, total_timeout=100)
        self.assertEqual(run.call_args_list[0][0][1], 100)
        self.assertEqual(done, ['a.nomad'])
        self.assertEqual(failed, [('b.nomad', '중단: 전체 시간 초과')])

    def test_group_timeout_kills_children(self):
        marker = Path(self.tmp.name) / 'child_alive'
        script = 'sleep 30 & echo $! > {0}; wait'.format(marker)
        start = time.monotonic()
        self.assertEqual(li._bounded_group(['bash', '-c', script], 1), (None, ''))
        self.assertLess(time.monotonic() - start, 10)
        pid = int(marker.read_text())
        time.sleep(0.2)
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_message_counts_failures(self):
        self.assertEqual(li.upload_message(['a', 'b'], []), '2개 복사했습니다.')
        self.assertEqual(li.upload_message(['a'], [('b.nomad', '이미 있음'), ('c.nomad', '시간 초과')]),
                         '1개 복사, 2개 실패(b.nomad: 이미 있음)')

    def test_offer_shows_failures(self):
        from PyQt5 import QtWidgets
        QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        import share_mount
        info = dict(self.info, feature=li.FEATURE, version='aaaaaaaaaaaa')
        shown = []
        found = {'ok': True, 'reason': 'mounted', 'source': self.info['source'], 'server': '', 'message': ''}
        with mock.patch.object(li, 'installed_info', return_value=info), \
                mock.patch.object(share_mount, 'resolve_source', return_value=found), \
                mock.patch.object(li, 'update_check', return_value=(None, 'same')), \
                mock.patch.object(li, 'pending_uploads', return_value=[Path('a.nomad'), Path('b.nomad')]), \
                mock.patch.object(li, 'upload', return_value=(['a.nomad'], [('b.nomad', '시간 초과')])), \
                mock.patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes), \
                mock.patch.object(QtWidgets.QMessageBox, 'warning', lambda *a: shown.append(a[2])):
            li.offer(None)
        self.assertEqual(shown, ['1개 복사, 1개 실패(b.nomad: 시간 초과)'])


if __name__ == '__main__':
    unittest.main()
