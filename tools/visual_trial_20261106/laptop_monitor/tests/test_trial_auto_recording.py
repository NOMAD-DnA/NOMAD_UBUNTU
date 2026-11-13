"""Offline fixtures only: never construct a board connection or actuator worker."""
import os
from pathlib import Path
import sqlite3
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad-auto-record-test-state")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore, QtWidgets
import monitor
from session_recording import SessionReader, SessionRecorder, save_trial_summary
from trial_summary import TrialSummaryDialog


class TrialAutoRecordingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="nomad-auto-record-fixture-")
        self.env = patch.dict(os.environ, {"XDG_STATE_HOME": self.temp.name,
                                          "NOMAD_BOARD_PASSWORD1": "offline-fixture"})
        self.env.start()
        self.windows = []

    def tearDown(self):
        for w in self.windows:
            w._trial_notes_required = False
            w.planning_client = None
            w.stop_worker = None
            if w._trial_notes_dialog is not None:
                QtWidgets.QDialog.accept(w._trial_notes_dialog)
            w.close()
        self.app.processEvents()
        self.env.stop()
        self.temp.cleanup()

    def window(self):
        with patch.object(sys, "argv", ["monitor", "--no-ssh", "--enable-trial-control"]):
            args = monitor.parse_args()
        w = monitor.MonitorWindow(args)
        w.timer.stop()
        w.ingest_timer.stop()
        w.args.no_ssh = False
        w.args.enable_trial_control = True
        w.telemetry["planning"] = {"plan_only": False, "armed": False}
        w.received["telemetry:planning"] = time.monotonic()
        w.planning_client = SimpleNamespace(send=Mock(return_value=True), stop=Mock(),
                                            ready=True, stopping=False)
        self.windows.append(w)
        return w

    def test_arm_records_before_send_and_reuses_one_file_until_trial_end(self):
        w = self.window()
        def sent(*args, **kwargs):
            self.assertIsNotNone(w.recorder)
            self.assertTrue(w.recorder.wait_ready(.1))
            return True
        w.planning_client.send.side_effect = sent
        w.arm_output()
        recorder = w.recorder
        path = recorder.path
        self.assertFalse(w.record_button.isEnabled())
        w.disarm_output()
        w.toggle_recording()  # Even programmatic/manual toggle cannot truncate it.
        w.arm_output()
        self.assertIs(w.recorder, recorder)
        self.assertEqual(len(list(Path(self.temp.name).rglob("*.nomad"))), 1)
        w.stop_recording()
        with_reader = SessionReader(path)
        self.addCleanup(with_reader.close)
        self.assertEqual(with_reader.metadata["recording_trigger"], "output_allow")
        actions = [r["event"] for r in with_reader.iter_until(with_reader.duration)
                   if r["event"].get("trial_action") == "output_allow_requested"]
        self.assertEqual(len(actions), 2)
        self.assertIn("click_wall_ns", actions[0])

    def test_existing_manual_recording_is_adopted(self):
        w = self.window()
        self.assertTrue(w.start_recording())
        recorder = w.recorder
        w.arm_output()
        self.assertIs(w.recorder, recorder)
        self.assertTrue(w._auto_trial_recording)

    def test_start_failure_or_disk_failure_never_sends_arm(self):
        w = self.window()
        with patch("session_ui.SessionRecorder", side_effect=OSError("disk full")):
            w.arm_output()
        w.planning_client.send.assert_not_called()
        self.assertIn("기록", w.planning_message.text())
        class FailedRecorder(SessionRecorder):
            def _connect(self):
                raise sqlite3.OperationalError("disk full")
        with patch("session_ui.SessionRecorder", FailedRecorder):
            w.arm_output()
        w.planning_client.send.assert_not_called()
        self.assertEqual(w.recorder.status()["state"], "FAILED")

    def test_stop_starts_worker_before_required_dialog_and_flushes_before_notes(self):
        w = self.window()
        w.arm_output()
        path = w.recorder.path
        worker = Mock()
        worker.isRunning.return_value = False
        def started():
            self.assertIsNone(w._trial_notes_dialog)
        worker.start.side_effect = started
        with patch.object(monitor, "TrialStopWorker", return_value=worker):
            w.stop_trial()
        worker.start.assert_called_once()
        dialog = w._trial_notes_dialog
        self.assertIsNotNone(dialog)
        self.assertTrue(w._trial_notes_required)
        self.assertFalse(dialog.save_button.isEnabled())
        dialog.name_edit.setText(" 우회전 비교 ")
        dialog.description_edit.setPlainText(" 목표는 오른쪽, 실제는 직진. ")
        self.assertFalse(dialog.save_button.isEnabled())
        result = {"ok": True, "boards": {"pi5": {"ok": True}, "jetson": {"ok": True}}}
        w.on_trial_stopped(result)
        self.assertIsNone(w.recorder)
        self.assertTrue(dialog.save_button.isEnabled())
        dialog.accept()
        self.assertFalse(w._trial_notes_required)
        reader = SessionReader(path)
        self.addCleanup(reader.close)
        trial = reader.metadata["trial"]
        self.assertEqual(trial["name"], "우회전 비교")
        self.assertEqual(trial["description"], "목표는 오른쪽, 실제는 직진.")
        self.assertTrue(trial["stop_result"]["ok"])
        self.assertTrue(reader.metadata["_recording"]["complete"])
        self.assertTrue(any(r["event"].get("trial_stop_result") == result
                            for r in reader.iter_until(reader.duration)))
        # Metadata survives copying/opening just one .nomad; playback sends nothing.
        with patch.object(sys, "argv", ["monitor", "--replay", path]):
            args = monitor.parse_args()
        replay = monitor.MonitorWindow(args)
        self.windows.append(replay)
        replay.timer.stop()
        replay.ingest_timer.stop()
        replay.update_recording_bar()
        self.assertIn("우회전 비교", replay.record_label.text())
        self.assertIn("실제는 직진", replay.record_label.toolTip())

    def test_notes_require_both_fields_and_cannot_be_dismissed_or_lost_on_error(self):
        save = Mock(side_effect=OSError("read-only disk"))
        dialog = TrialSummaryDialog(None, save)
        dialog.show()
        dialog.set_recording_state(True, "offline fixture")
        dialog.name_edit.setText("  ")
        dialog.description_edit.setPlainText("내용")
        dialog.accept()
        save.assert_not_called()
        dialog.name_edit.setText("시험명")
        dialog.description_edit.setPlainText("  ")
        self.assertFalse(dialog.save_button.isEnabled())
        dialog.description_edit.setPlainText("내용")
        dialog.reject()  # Esc follows reject too.
        dialog.close()  # Window X.
        self.assertTrue(dialog.isVisible())
        dialog.accept()
        self.assertTrue(dialog.isVisible())
        self.assertEqual(dialog.name_edit.text(), "시험명")
        self.assertIn("저장 실패", dialog.error_label.text())
        save.side_effect = None
        dialog.accept()
        self.assertEqual(dialog.result(), QtWidgets.QDialog.Accepted)
        dialog.deleteLater()

    def test_partial_stop_retry_keeps_same_recording_and_updates_summary(self):
        w = self.window()
        w.arm_output()
        path = w.recorder.path
        worker = Mock()
        worker.isRunning.return_value = False
        with patch.object(monitor, "TrialStopWorker", return_value=worker) as factory:
            w.stop_trial()
            w.on_trial_stopped({"ok": False, "boards": {"pi5": {"ok": True}, "jetson": {"ok": False}}})
            w._trial_notes_dialog.name_edit.setText("종료 실패 시험")
            w._trial_notes_dialog.description_edit.setPlainText("Jetson 종료를 재시도함")
            w._trial_notes_dialog.accept()
            w.stop_trial()
            w.on_trial_stopped({"ok": True, "boards": {"pi5": {"ok": True}, "jetson": {"ok": True}}})
            w._trial_notes_dialog.accept()
        self.assertEqual(w._trial_record_path, path)
        self.assertEqual(len(list(Path(self.temp.name).rglob("*.nomad"))), 1)
        reader = SessionReader(path)
        self.addCleanup(reader.close)
        self.assertTrue(reader.metadata["trial"]["stop_result"]["ok"])

    def test_stop_without_output_has_labelled_snapshot_and_main_close_is_blocked(self):
        w = self.window()
        worker = Mock()
        worker.isRunning.return_value = False
        with patch.object(monitor, "TrialStopWorker", return_value=worker):
            w.stop_trial()
        event = Mock()
        w.closeEvent(event)
        event.ignore.assert_called_once()
        w.on_trial_stopped({"ok": True, "boards": {}})
        w._trial_notes_dialog.name_edit.setText("출력 없는 시험")
        w._trial_notes_dialog.description_edit.setPlainText("상태만 확인")
        w._trial_notes_dialog.accept()
        reader = SessionReader(w._trial_record_path)
        self.addCleanup(reader.close)
        self.assertEqual(reader.metadata["recording_trigger"], "trial_end_without_output_allow")

    def test_invalid_freshness_spectator_and_replay_do_not_start_recording(self):
        w = self.window()
        w.received["telemetry:planning"] -= 4
        w.arm_output()
        self.assertIsNone(w.recorder)
        w.received["telemetry:planning"] = time.monotonic()
        for attribute in ("spectator_mode", "replay_mode"):
            setattr(w, attribute, True)
            if attribute == "replay_mode":
                w.args.no_ssh = True
            w.arm_output()
            w.stop_trial()
            self.assertIsNone(w.recorder)
            self.assertIsNone(w._trial_notes_dialog)
            setattr(w, attribute, False)
        w.planning_client.send.assert_not_called()

    def test_unwritable_recording_can_save_required_notes_to_selected_json(self):
        import json
        w = self.window()
        w._trial_record_closed = True
        w._trial_notes_required = True
        w._trial_record_path = str(Path(self.temp.name)/"failed.nomad")
        w.recording_result = {"state": "FAILED", "error": "offline disk error", "path": w._trial_record_path}
        w.trial_stop_result = {"ok": False}
        dialog = TrialSummaryDialog(w, w.save_trial_notes, save_copy=w.save_trial_notes_copy)
        dialog.set_recording_state(True, "offline failed recording fixture")
        dialog.name_edit.setText("기록 실패 시험")
        dialog.description_edit.setPlainText("내용은 다른 위치에 보관")
        dialog.accept()
        self.assertFalse(dialog.copy_button.isHidden())
        target = Path(self.temp.name)/"notes.json"
        with patch.object(QtWidgets.QFileDialog, "getSaveFileName", return_value=(str(target), "")):
            dialog.save_copy()
        self.assertFalse(w._trial_notes_required)
        saved = json.loads(target.read_text())
        self.assertEqual(saved["trial"]["name"], "기록 실패 시험")
        self.assertEqual(saved["recording"]["state"], "FAILED")
        self.assertFalse(saved["contains_driving_log"])

    def test_summary_transaction_does_not_change_events_or_accept_empty_fields(self):
        path = Path(self.temp.name)/"fixture.nomad"
        r = SessionRecorder(path)
        self.assertTrue(r.wait_ready())
        r.append("pi4", {"kind": "telemetry", "source": "planning", "path": [[0, 0, 0]]}, 0.)
        r.close()
        with sqlite3.connect(path) as db:
            before = db.execute("SELECT * FROM events").fetchall()
        for name, description in [(" ", "x"), ("x", "\n")]:
            with self.assertRaises(ValueError):
                save_trial_summary(path, name, description, {"ok": True}, time.time())
        save_trial_summary(path, "../시험", "첫줄\n둘째줄", {"ok": True}, time.time())
        with sqlite3.connect(path) as db:
            self.assertEqual(before, db.execute("SELECT * FROM events").fetchall())


if __name__ == "__main__":
    unittest.main()
