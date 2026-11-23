"""Mock-only front recording check contract. Never contacts boards or processes."""
import json
import os
from pathlib import Path
import re
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from front_recording import RESULT_MARKER, check_front_recording, runs_text, summary_text

STATUS = {"state": "RECORDING", "topic": "/oak/left/image_rect", "dir": "/x/front_video", "file": "seg_0003.avi",
          "backend": "ffmpeg", "segments": 4, "frames": 2140, "dropped": 0, "rejected": 0, "duration_s": 214.3,
          "bytes": 81000000, "free_gb": 87.6, "last_frame_age_s": 0.08, "started_unix": 1, "updated_unix": 2,
          "pid": 10, "error": ""}


RUNS = [dict(name="visual_trial_20261111_203955_55181", state="RECORDING", duration_s=129.78, frames=1297,
             dropped=0, segments=3, bytes=56515871, current=True),
        dict(name="visual_trial_20261111_185506_32604", state="STOPPED", duration_s=600.0, frames=5990,
             dropped=4, segments=10, bytes=260000000, current=False)]


def packet(state="CHECKED", status=STATUS, alive=True, runs=RUNS, **extra):
    data = dict(ok=True, state=state, reason="status_read", alive=alive, previous_runs=2,
                previous_bytes=3_200_000_000, status=status, runs=runs)
    data.update(extra)
    return dict(ok=True, stdout=RESULT_MARKER + json.dumps(data) + "\n")


class FrontRecordingTests(unittest.TestCase):
    def test_recording_summary(self):
        with patch("front_recording.run_remote", return_value=packet()) as call:
            value = check_front_recording("user@192.168.3.164", "private")
        self.assertTrue(value["ok"])
        self.assertEqual(call.call_args.args[1].name, "check_front_recording.sh")
        self.assertNotIn("private", json.dumps(value))
        text = summary_text(value)
        for part in ("녹화 중", "3.6분", "2140프레임", "0.08GB", "파일 4개", "여유 88GB", "이전 시험 2개"):
            self.assertIn(part, text)

    def test_lists_each_recording(self):
        with patch("front_recording.run_remote", return_value=packet()):
            lines = runs_text(check_front_recording("user@pi5")).splitlines()
        self.assertEqual(len(lines), 2)
        for part in ("11/11 20:39:55", "2.2분", "1297프레임", "0.06GB", "파일 3개", "녹화 중", "현재 시험"):
            self.assertIn(part, lines[0])
        for part in ("11/11 18:55:06", "10.0분", "녹화 종료", "누락 4"):
            self.assertIn(part, lines[1])
        self.assertNotIn("현재 시험", lines[1])
        with patch("front_recording.run_remote", return_value=packet(alive=False)):
            self.assertNotIn("녹화 중", runs_text(check_front_recording("user@pi5")).splitlines()[0])
        with patch("front_recording.run_remote", return_value=packet(runs=[])):
            self.assertEqual(runs_text(check_front_recording("user@pi5")), "저장된 전방 녹화 없음")
        with patch("front_recording.run_remote", return_value=packet(runs=[dict(name=5)])):
            self.assertFalse(check_front_recording("user@pi5")["ok"])

    def test_dead_recorder_is_not_reported_as_recording(self):
        with patch("front_recording.run_remote", return_value=packet(alive=False)):
            self.assertIn("녹화기 종료됨", summary_text(check_front_recording("user@pi5")))
        stopped = dict(STATUS, state="STOPPED")
        with patch("front_recording.run_remote", return_value=packet(status=stopped, alive=False)):
            self.assertIn("녹화 종료", summary_text(check_front_recording("user@pi5")))

    def test_states_without_recorder_status(self):
        for state, words in (("NOT_INSTALLED", "미설치"), ("NO_TRIAL_RUN", "시험 없음"), ("NOT_STARTED", "시작 안 됨")):
            with patch("front_recording.run_remote", return_value=packet(state=state, status=None, alive=False)):
                self.assertIn(words, summary_text(check_front_recording("user@pi5")))

    def test_bad_host_never_reaches_transport(self):
        for host in ("nvidia@jetson", "root@pi5", "user@pi5;reboot", "user@pi5\necho"):
            with patch("front_recording.run_remote") as call:
                self.assertFalse(check_front_recording(host)["ok"])
                call.assert_not_called()

    def test_malformed_or_failed_results(self):
        bad_numbers = dict(STATUS, frames="many")
        for result in (dict(ok=True, stdout=""), dict(ok=True, stdout=RESULT_MARKER + "bad"),
                       dict(ok=True, stdout=packet()["stdout"] * 2), dict(ok=False, stdout=packet()["stdout"]),
                       packet(status=bad_numbers), packet(ok=False, reason="wrong_board")):
            with patch("front_recording.run_remote", return_value=result):
                value = check_front_recording("user@pi5")
                self.assertFalse(value["ok"])
                self.assertIn("확인 실패", summary_text(value))
        with patch("front_recording.run_remote", side_effect=RuntimeError("password")):
            self.assertEqual(check_front_recording("user@pi5")["reason"], "local_control_error")

    def test_trial_scripts_start_and_stop_the_recorder(self):
        start = (ROOT / "remote/start_pi_vio_fusion.sh").read_text()
        self.assertIn('"$run/front_recorder.pid"', start)
        self.assertIn("front_recorder NOT_INSTALLED", start)
        stop = (ROOT / "remote/stop_trial.sh").read_text()
        self.assertIn("pi5:front_recorder", stop)
        self.assertIn("front_recorder:*nomad_front_recorder*", stop)
        self.assertRegex(stop, re.compile(r"\|vio_supervisor\|front_recorder\|"))

    def test_jetson_start_sends_oak_color_jpeg(self):
        start = (ROOT / "remote/start_jetson.sh").read_text()
        self.assertIn("-p enable_color_jpeg:=true", start)
        self.assertIn("-p enable_device_depth:=true", start)  # existing depth graph kept
        check = (ROOT / "remote/check_front_recording.sh").read_text()
        self.assertIn("seg_*.mjpeg", check)

    def test_check_script_is_read_only(self):
        script = (ROOT / "remote/check_front_recording.sh").read_text()
        for word in ("kill -", "rm ", "mv ", "setsid", "ros2 ", "> \"$"):
            self.assertNotIn(word, script)


class FrontDownloadTests(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.tmp = tempfile.TemporaryDirectory()
        self.env = patch.dict(os.environ, {"XDG_STATE_HOME": self.tmp.name})
        self.env.start()

    def tearDown(self):
        self.env.stop()
        self.tmp.cleanup()

    def fake_stream(self, members, returncode=0):
        import io, tarfile, tempfile
        buffer = io.BytesIO()
        with tarfile.open(fileobj=buffer, mode="w") as archive:
            for name, data in members:
                info = tarfile.TarInfo(name)
                info.size = len(data)
                archive.addfile(info, io.BytesIO(data))
        buffer.seek(0)
        class Proc:
            pid = 999999
            stdout = buffer
            stderr = io.BytesIO(b"")
            def __init__(self): self.returncode = None
            def poll(self): return self.returncode
            def wait(self): self.returncode = returncode; return returncode
        return lambda *a, **k: (Proc(), tempfile.TemporaryDirectory(), {"host": "user@x", "via": None, "method": None})

    def test_download_extracts_only_front_video_files(self):
        import front_recording as fr
        jpeg = bytes([0xFF, 0xD8]) + b"x" * 50 + bytes([0xFF, 0xD9])
        members = [("front_video/seg_0000.mjpeg", jpeg), ("front_video/seg_0000.csv", b"frame\n"),
                   ("front_video/status.json", b"{}"), ("../evil", b"no"), ("front_video/sub/x", b"no"),
                   ("other/file", b"no")]
        with patch.object(fr, "_ssh_stream", self.fake_stream(members)), patch.object(fr, "_combine", return_value=None):
            value = fr.download_run("user@192.168.3.163", "visual_trial_20261111_210814_62698")
        self.assertTrue(value["ok"], value)
        folder = Path(value["folder"])
        self.assertEqual(sorted(p.name for p in folder.iterdir()), ["seg_0000.csv", "seg_0000.mjpeg", "status.json"])
        self.assertTrue(fr.downloaded("visual_trial_20261111_210814_62698"))
        self.assertFalse((folder.parent.parent / "evil").exists())

    def test_bad_names_never_reach_ssh(self):
        import front_recording as fr
        with patch.object(fr, "_ssh_stream") as stream:
            for host, run in (("user@pi5", "visual_trial_x;rm -rf ~"), ("user@pi5", "../etc"),
                              ("nvidia@jetson", "visual_trial_1"), ("user@pi5\n", "visual_trial_1")):
                self.assertFalse(fr.download_run(host, run)["ok"])
            stream.assert_not_called()

    def test_cancel_and_failed_transfer_leave_nothing(self):
        import front_recording as fr
        members = [("front_video/seg_0000.mjpeg", b"a" * 10)]
        with patch.object(fr, "_ssh_stream", self.fake_stream(members)):
            value = fr.download_run("user@192.168.3.163", "visual_trial_1", cancelled=lambda: True)
        self.assertEqual(value["reason"], "cancelled")
        with patch.object(fr, "_ssh_stream", self.fake_stream([], returncode=2)):
            self.assertFalse(fr.download_run("user@192.168.3.163", "visual_trial_2")["ok"])
        root = fr.front_video_root()
        self.assertEqual([p.name for p in root.iterdir()], [])

    def test_combine_makes_a_real_video_without_ffmpeg(self):
        try:
            import cv2
            import numpy
        except ImportError:
            self.skipTest("OpenCV not installed")
        import tempfile
        import front_recording as fr
        with tempfile.TemporaryDirectory() as tmp, patch.object(fr.shutil, "which", return_value=None):
            folder = Path(tmp)
            stream, rows, offset = b"", ["frame,stamp_ns,received_ns,offset,bytes"], 0
            for i in range(25):
                ok, jpg = cv2.imencode(".jpg", numpy.full((360, 640, 3), i * 9, numpy.uint8))
                data = jpg.tobytes()
                rows.append(f"{i},{1_000_000_000 + i * 100_000_000},0,{offset},{len(data)}")
                stream += data
                offset += len(data)
            (folder / "seg_0000.mjpeg").write_bytes(stream)
            (folder / "seg_0000.csv").write_text("\n".join(rows) + "\n")
            (folder / "status.json").write_text("{}")
            combined = fr._combine(folder)
            self.assertEqual(combined.name, "전체.avi")
            self.assertEqual(sorted(p.name for p in (folder / "원본").iterdir()),
                             ["seg_0000.csv", "seg_0000.mjpeg", "status.json"])
            capture = cv2.VideoCapture(str(combined))
            frames = 0
            while capture.read()[0]:
                frames += 1
            self.assertEqual(frames, 25)
            self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 10.0, places=1)

    def test_dialog_lists_runs(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PyQt5 import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        import front_recording_dialog as frd
        with patch.object(frd.QtCore.QTimer, "singleShot"):
            dialog = frd.FrontRecordingDialog("user@192.168.3.163")
        with patch("front_recording.run_remote", return_value=packet()):
            import front_recording as fr
            dialog.show_runs(fr.check_front_recording("user@192.168.3.163"))
        self.assertEqual(dialog.table.rowCount(), 2)
        self.assertEqual(dialog.table.item(0, 1).text(), "11/11 20:39:55")
        self.assertFalse(dialog.download_button.isEnabled())
        dialog.table.selectRow(0)
        self.assertTrue(dialog.download_button.isEnabled())
        dialog.close()

    def test_notes_are_shared_cached_and_never_removed(self):
        import base64, front_recording as fr
        sent = {}
        def fake(host, script, password, timeout_s=18, **kw):
            text = Path(script).read_text()
            payload = json.loads(base64.b64decode(text.split("\n", 1)[0].split("=", 1)[1]))
            sent.update(payload)
            note = dict(title=payload["title"], comment=payload["comment"], updated=100)
            return dict(ok=True, stdout="NOMAD_FRONT_MANAGE_RESULT=" + json.dumps(dict(ok=True, state="NOTE_SAVED", note=note)))
        run = "visual_trial_20261111_214202_82207"
        (fr.front_video_root() / run).mkdir(parents=True)
        with patch.object(fr, "run_remote", side_effect=fake):
            value = fr.set_note("user@192.168.3.163", run, "우면산 S자 2회차", "사람 지나감")
        self.assertTrue(value["ok"])
        self.assertEqual((sent["action"], sent["run"], sent["title"]), ("note", run, "우면산 S자 2회차"))
        self.assertEqual(fr.local_notes()[run]["comment"], "사람 지나감")
        self.assertIn("우면산 S자 2회차", (fr.front_video_root() / run / "메모.txt").read_text())
        fr.remember_notes({})                                   # empty Pi5 notes: cache untouched
        fr.remember_notes({run: dict(title="old", comment="", updated=50)})  # older: ignored
        self.assertEqual(fr.local_notes()[run]["title"], "우면산 S자 2회차")
        self.assertFalse(fr.set_note("user@pi5", "../x", "a", "b")["ok"])
        self.assertFalse(fr.set_note("user@pi5", run, "t" * 121, "")["ok"])

    def test_delete_is_pi5_only_and_validated(self):
        import base64, front_recording as fr
        run = "visual_trial_20261111_214202_82207"
        folder = fr.front_video_root() / run
        folder.mkdir(parents=True)
        (folder / "전체.avi").write_bytes(b"x")
        sent = {}
        def fake(host, script, password, timeout_s=18, **kw):
            sent.update(json.loads(base64.b64decode(Path(script).read_text().split("\n", 1)[0].split("=", 1)[1])))
            return dict(ok=True, stdout='NOMAD_FRONT_MANAGE_RESULT={"ok":true,"state":"DELETED","bytes":5,"note_kept":true}')
        with patch.object(fr, "run_remote", side_effect=fake):
            self.assertTrue(fr.delete_on_pi5("user@192.168.3.163", run)["ok"])
        self.assertEqual(sent, {"action": "delete", "run": run})
        self.assertTrue((folder / "전체.avi").exists())         # laptop copy untouched
        self.assertIn(run, fr.local_runs())
        with patch.object(fr, "run_remote") as call:
            for bad in ("visual_trial_x;rm -rf ~", "../../etc", ""):
                self.assertFalse(fr.delete_on_pi5("user@pi5", bad)["ok"])
            self.assertFalse(fr.delete_on_pi5("nvidia@jetson", run)["ok"])
            call.assert_not_called()

    def test_manage_script_never_touches_laptop_or_other_folders(self):
        script = (ROOT / "remote/front_manage.sh").read_text()
        self.assertIn('ws / "log" / run / "front_video"', script)
        self.assertIn("recording_now", script)
        self.assertIn("notes survive the delete", script)
        self.assertNotIn("front_video_notes.json\")\n    shutil.rmtree", script)
        self.assertEqual(script.count("shutil.rmtree("), 1)

    def test_dialog_shows_laptop_only_rows_and_notes(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PyQt5 import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        import front_recording as fr, front_recording_dialog as frd
        kept = "visual_trial_20261110_100000_1"
        (fr.front_video_root() / kept).mkdir(parents=True)
        (fr.front_video_root() / kept / "전체.avi").write_bytes(b"x")
        fr.remember_notes({kept: dict(title="예전 시험", comment="Pi5에서 지움", updated=1)})
        with patch.object(frd.QtCore.QTimer, "singleShot"):
            dialog = frd.FrontRecordingDialog("user@192.168.3.163")
        with patch("front_recording.run_remote", return_value=packet()):
            dialog.show_runs(fr.check_front_recording("user@192.168.3.163"))
        names = [row["name"] for row in dialog.rows]
        self.assertIn(kept, names)
        index = names.index(kept)
        self.assertEqual(dialog.table.item(index, 0).text(), "예전 시험")
        self.assertIn("없음", dialog.table.item(index, 4).text())
        dialog.table.selectRow(index)
        self.assertFalse(dialog.delete_button.isEnabled())       # nothing on Pi5 to delete
        self.assertFalse(dialog.download_button.isEnabled())
        self.assertTrue(dialog.play_button.isEnabled())
        self.assertTrue(dialog.note_button.isEnabled())
        dialog.close()


if __name__ == "__main__":
    unittest.main()
