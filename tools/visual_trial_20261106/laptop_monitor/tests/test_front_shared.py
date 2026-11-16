"""Front videos in the team shared folder + launcher share status (2026-11-12)."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
import front_recording as fr
import local_install as li

RUN = "visual_trial_20261112_014140_8428"
SHARED_ONLY = "visual_trial_20261111_235200_36434"


class FrontSharedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = patch.dict(os.environ, {"XDG_STATE_HOME": self.tmp.name})
        self.env.start()
        self.local = fr.front_video_root() / RUN
        (self.local / "원본").mkdir(parents=True)
        (self.local / "전체.avi").write_bytes(b"v" * 3000)
        (self.local / "원본/seg_0000.csv").write_text("t\n")
        self.shared = Path(self.tmp.name) / "share/주행기록/전방녹화"
        (self.shared / SHARED_ONLY).mkdir(parents=True)
        (self.shared / SHARED_ONLY / "전체.avi").write_bytes(b"s")
        (self.shared / "메모_모음.json").write_text("{}")

    def tearDown(self):
        self.env.stop()
        self.tmp.cleanup()

    def test_shared_runs_and_root(self):
        self.assertEqual(fr.shared_runs(self.shared), [SHARED_ONLY])
        self.assertEqual(fr.shared_runs(None), [])
        self.assertEqual(fr.shared_runs(self.shared / "gone"), [])
        with patch.object(li, "reachable_shared_recordings", return_value=self.shared.parent):
            self.assertEqual(fr.shared_front_root(), self.shared)
        with patch.object(li, "reachable_shared_recordings", return_value=fr.front_video_root().parent):
            self.assertIsNone(fr.shared_front_root())          # running from the shared folder itself
        with patch.object(li, "reachable_shared_recordings", return_value=None):
            self.assertIsNone(fr.shared_front_root())

    def test_upload_copies_keeps_local_and_never_overwrites(self):
        seen = []
        value = fr.upload_run(RUN, self.shared, progress=seen.append, chunk=1000)
        self.assertTrue(value["ok"], value)
        self.assertEqual((self.shared / RUN / "전체.avi").read_bytes(), b"v" * 3000)
        self.assertTrue((self.shared / RUN / "원본/seg_0000.csv").is_file())
        self.assertTrue((self.local / "전체.avi").is_file())       # laptop copy kept
        self.assertFalse((self.shared / (RUN + ".part")).exists())
        self.assertEqual(seen[-1], 3002)
        self.assertEqual(fr.upload_run(RUN, self.shared)["reason"], "already_shared")
        self.assertEqual(fr.upload_run("../x", self.shared)["reason"], "bad_run")
        self.assertEqual(fr.upload_run(SHARED_ONLY, self.shared)["reason"], "not_downloaded")

    def test_upload_cancel_removes_partial(self):
        value = fr.upload_run(RUN, self.shared, cancelled=lambda: True)
        self.assertEqual(value["reason"], "cancelled")
        self.assertFalse((self.shared / RUN).exists())
        self.assertFalse((self.shared / (RUN + ".part")).exists())

    def test_dialog_lists_shared_plays_and_uploads(self):
        import front_recording_dialog as frd
        with patch.object(frd.QtCore.QTimer, "singleShot"):
            dialog = frd.FrontRecordingDialog("user@192.168.3.163")
        dialog.show_runs({"ok": False, "reason": "pi5_off", "shared_root": str(self.shared),
                          "shared_runs": [SHARED_ONLY]})
        names = [row["name"] for row in dialog.rows]
        self.assertEqual(set(names), {RUN, SHARED_ONLY})
        column = frd.FrontRecordingDialog.COLUMNS.index("공유폴더")
        shared_row, local_row = names.index(SHARED_ONLY), names.index(RUN)
        self.assertEqual(dialog.table.item(shared_row, column).text(), "있음")
        self.assertEqual(dialog.table.item(local_row, column).text(), "")
        self.assertTrue(dialog.shared_button.isEnabled())
        dialog.table.selectRow(shared_row)
        self.assertTrue(dialog.play_button.isEnabled())             # only in the shared folder
        self.assertFalse(dialog.upload_button.isEnabled())
        with patch("front_player.play") as play:
            dialog.play()
        self.assertEqual(Path(play.call_args.args[0]), self.shared / SHARED_ONLY)
        dialog.table.selectRow(local_row)
        self.assertTrue(dialog.upload_button.isEnabled())
        dialog.upload()
        dialog.task.wait(5000)
        self.app.processEvents()
        self.assertTrue((self.shared / RUN / "전체.avi").is_file())
        self.assertEqual(dialog.table.item(names.index(RUN), column).text(), "있음")
        self.assertIn("공유폴더에 올림", dialog.status.text())
        dialog.close()

    def test_dialog_without_share_hides_shared_actions(self):
        import front_recording_dialog as frd
        with patch.object(frd.QtCore.QTimer, "singleShot"):
            dialog = frd.FrontRecordingDialog("user@192.168.3.163")
        dialog.show_runs({"ok": False, "reason": "pi5_off", "shared_root": "", "shared_runs": []})
        dialog.table.selectRow(0)
        self.assertFalse(dialog.upload_button.isEnabled())
        self.assertFalse(dialog.shared_button.isEnabled())
        dialog.close()


class ShareStatusLineTests(unittest.TestCase):
    def test_texts(self):
        ok = {"ok": True, "server": "10.42.0.1", "source": "/run/x"}
        self.assertEqual(li.share_state_text(ok, None, "same", "abc"), "연결됨 · smb://10.42.0.1/ · GUI 최신 버전(abc)")
        self.assertIn("새 버전 있음(def)", li.share_state_text(ok, "def", "newer", "abc"))
        local = {"ok": True, "server": "", "source": "/home/u/team-shared/A"}
        self.assertIn("/home/u/team-shared/A", li.share_state_text(local, None, "same", "abc"))
        text = li.share_state_text({"ok": False, "message": "공유폴더 PC를 찾지 못함"}, None, None, "abc")
        self.assertIn("연결 안 됨", text)
        self.assertIn("공유폴더 연결", text)

    def test_offer_fills_launcher_label(self):
        QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        import share_mount
        label = QtWidgets.QLabel()
        window = type("W", (), {"share_label": label})()
        info = {"feature": li.FEATURE, "version": "aaaaaaaaaaaa", "source": "/gone", "root": "/tmp"}
        found = {"ok": True, "reason": "mounted", "source": "/run/x", "server": "user-su.local", "message": ""}
        with patch.object(li, "installed_info", return_value=info), \
                patch.object(share_mount, "resolve_source", return_value=found), \
                patch.object(li, "update_check", return_value=(None, "same")), \
                patch.object(li, "pending_uploads", return_value=[]):
            li.offer(window)
        self.assertEqual(label.text(), "공유폴더: 연결됨 · smb://user-su.local/ · GUI 최신 버전(aaaaaaaaaaaa)")
        with patch.object(li, "installed_info", return_value=None):
            li.offer(window)
        self.assertIn("원본에서 직접 실행", label.text())


if __name__ == "__main__":
    unittest.main()
