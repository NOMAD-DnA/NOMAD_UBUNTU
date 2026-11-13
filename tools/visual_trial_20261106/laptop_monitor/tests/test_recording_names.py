"""Real filesystem rename checks, using only temporary synthetic files."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from recording_names import rename_trial_recording, trial_filename_stem


class RecordingNamesTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def source(self, name='timestamp.nomad', data=b'untouched synthetic recording'):
        path = self.root/name
        path.write_bytes(data)
        return path

    def test_korean_name_preserves_bytes_and_repeated_save_is_idempotent(self):
        source = self.source()
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        dest = Path(rename_trial_recording(source, '(대성공) 강의실 앞 ㄹ자'))
        self.assertEqual(dest.name, '(대성공) 강의실 앞 ㄹ자.nomad')
        self.assertFalse(source.exists())
        self.assertEqual(hashlib.sha256(dest.read_bytes()).hexdigest(), digest)
        self.assertEqual(rename_trial_recording(dest, '(대성공) 강의실 앞 ㄹ자'), str(dest))

    def test_existing_name_and_concurrent_renames_never_overwrite(self):
        occupied = self.source('동일 시험.nomad', b'existing')
        sources = [self.source('input%d.nomad'%i, str(i).encode()) for i in range(5)]
        with ThreadPoolExecutor(max_workers=5) as pool:
            outputs = list(pool.map(lambda p: rename_trial_recording(p, '동일 시험'), sources))
        self.assertEqual(len(set(outputs)), 5)
        self.assertEqual(occupied.read_bytes(), b'existing')
        self.assertEqual({Path(p).read_bytes() for p in outputs}, {str(i).encode() for i in range(5)})
        self.assertEqual({Path(p).name for p in outputs}, {'동일 시험 (%d).nomad'%i for i in range(2,7)})

    def test_separators_reserved_names_and_long_utf8_remain_in_same_folder(self):
        for name in ['../escape', '/tmp/overwrite', 'CON', 'a\\b:*?"<>|\n', '가'*200, '...']:
            with self.subTest(name=name):
                source = self.source()
                dest = Path(rename_trial_recording(source, name))
                self.assertEqual(dest.parent, self.root)
                self.assertLessEqual(len(dest.name.encode()), 240)
                self.assertNotIn('/', dest.name)
                self.assertNotIn('\\', dest.name)
        with self.assertRaises(ValueError):
            trial_filename_stem(' \n')

    def test_busy_file_or_symlink_remains_untouched(self):
        source = self.source()
        journal = Path(str(source)+'-journal');journal.touch()
        with self.assertRaises(RuntimeError):
            rename_trial_recording(source, '아직 기록 중')
        self.assertTrue(source.exists())
        journal.unlink()
        link = self.root/'link.nomad';link.symlink_to(source)
        with self.assertRaises(ValueError):
            rename_trial_recording(link, '링크')
        self.assertTrue(source.exists())


if __name__ == '__main__':
    unittest.main()
