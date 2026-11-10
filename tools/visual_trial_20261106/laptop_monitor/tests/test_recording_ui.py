"""Offline end-to-end record/replay tests. Every input is a labelled fixture."""
import base64
import os
from pathlib import Path
import sqlite3
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_recording_gui_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore, QtGui, QtWidgets
import monitor
from session_recording import SessionReader, SessionRecorder

STAMP = 1700000000.


def image(source, color, stamp):
    value = QtGui.QImage(16, 12, QtGui.QImage.Format_RGB888)
    value.fill(QtGui.QColor(color))
    output = QtCore.QBuffer()
    output.open(QtCore.QIODevice.WriteOnly)
    assert value.save(output, "PNG")
    event = dict(kind="image", source=source, encoding="png", t=stamp, width=16, height=12,
                 data=base64.b64encode(bytes(output.data())).decode())
    if source in ("costmap", "slope"):
        event.update(georeferenced=True, grid={"frame_id": "odom", "resolution": .125,
            "width": 16, "height": 12, "image_y": "high_to_low",
            "origin": dict(x=-1., y=0., z=0., qx=0., qy=0., qz=0., qw=1.)})
    return event


def pose(x, seconds):
    return dict(kind="pose", source="public", t=STAMP+seconds, x=x, y=seconds, z=0.,
                qx=0., qy=0., qz=0., qw=1., frame_id="odom", child_frame_id="base_link")


def log(text, seq, seconds):
    return dict(kind="log", source="planning", board="pi4", t=STAMP+seconds,
                session="offline-fixture", seq=seq,
                text=f"[controller] [INFO] [{STAMP+seconds:.9f}] [nomad_vesc_trial]: {text}")


class RecordingUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="nomad-recording-ui-")
        self.windows = []
        self.root = Path(self.directory.name)
        self.env = patch.dict(os.environ, {"XDG_STATE_HOME": self.directory.name})
        self.env.start()

    def tearDown(self):
        for window in self.windows:
            window.close()
        self.app.processEvents()
        self.env.stop()
        self.directory.cleanup()

    def window(self, *options):
        with patch.object(sys, "argv", ["monitor", *options]):
            args = monitor.parse_args()
        window = monitor.MonitorWindow(args)
        window.timer.stop()
        window.ingest_timer.stop()
        self.windows.append(window)
        return window

    def fixture(self):
        path = self.root / "offline_fixture.nomad"
        recorder = SessionRecorder(path, {"started_wall_s": STAMP,
            "forward_axis": [0., 1., 0.], "heading_note": "offline fixture"})
        recorder.append("pi", dict(kind="status", source="localization", t=STAMP,
                                  state="READY", detail="offline fixture"), 0.)
        recorder.append("pi", {**pose(1., 1.), "_record_receive_age_s": .25}, 1.)
        for source, color in (("rgb", "red"), ("costmap", "green"),
                              ("slope", "green"), ("semantic", "yellow")):
            recorder.append("jetson", image(source, color, STAMP+1), 1.)
        recorder.append("pi4", log("FIRST", 1, 1.), 1.)
        recorder.append("pi", pose(2., 2.), 2.)
        recorder.append("jetson", image("rgb", "blue", STAMP+2), 2.)
        recorder.append("jetson", image("costmap", "red", STAMP+2), 2.)
        recorder.append("pi4", dict(kind="telemetry", source="planning", t=STAMP+2,
            frame="odom", state="DRIVING", reason="offline fixture", plan_only=False,
            path=[[0., 0., 0.], [2., 2., 0.]], trajectory=[[0., 0., 0.], [1., 1., 0.]],
            armed=True, actuator_enabled=True), 2.)
        recorder.append("pi4", log("SECOND", 2, 2.), 2.)
        self.assertEqual(recorder.close(duration_s=4.)["state"], "CLOSED")
        return path

    def seek(self, window, seconds):
        window.seek_replay(seconds)
        for _ in range(100):
            if not window._replay_seeking:
                break
            window.advance_replay()
        self.assertFalse(window._replay_seeking)
        window.refresh()

    def test_record_preserves_already_visible_baseline_and_new_packets(self):
        window = self.window("--no-ssh")
        window.on_packet("pi", pose(1., 1.))
        window.on_packet("jetson", image("rgb", "red", STAMP+1))
        window.on_packet("pi4", log("FIRST", 1, 1.))
        window.refresh()
        window.toggle_recording()
        window.on_packet("pi", pose(2., 2.))
        final_image = image("costmap", "green", STAMP+2)
        window.on_packet("jetson", final_image)
        window.selected_goal = (3., 4., "odom")
        window.refresh()
        window.stop_recording()
        self.assertTrue(window.recording_result["complete"])
        path = Path(window.recording_result["path"])
        self.assertTrue(path.is_relative_to(self.root))
        reader = SessionReader(path)
        records = list(reader.iter_until(reader.duration))
        reader.close()
        raw_map = next(row["event"] for row in records if row["event"].get("source") == "costmap")
        self.assertEqual(raw_map["data"], final_image["data"])
        self.assertEqual(raw_map["grid"], final_image["grid"])
        replay = self.window("--replay", str(path))
        self.seek(replay, replay.reader.duration)
        self.assertEqual([p["x"] for p in replay.histories["public"]], [1., 2.])
        self.assertEqual(replay.selected_goal, (3., 4., "odom"))
        self.assertEqual(replay.latest_qimages["rgb"].pixelColor(0, 0).name(), "#ff0000")
        self.assertEqual(len(replay.module_logs["planning"]), 1)

    def test_shared_cloud_slope_modes_and_cpu_history_survive_recording(self):
        window = self.window("--no-ssh")
        window.active_cloud_source = "device"
        window.active_slope_mode = "parallel"
        window.slope_cpu_history = {"parallel": {"percent_one_core": 120., "t": STAMP,
                                                 "received": time.monotonic() - .5}}
        window.toggle_recording()
        window.refresh()
        window.stop_recording()
        replay = self.window("--replay", window.recording_result["path"])
        self.seek(replay, replay.reader.duration)
        self.assertEqual(replay.active_cloud_source, "device")
        self.assertEqual(replay.active_slope_mode, "parallel")
        values = replay.slope_cpu_history["parallel"]
        self.assertEqual(values["percent_one_core"], 120.)
        self.assertGreaterEqual(replay.now() - values["received"], .5)

    def test_backward_seek_clears_future_images_poses_paths_and_child_logs(self):
        window = self.window("--replay", str(self.fixture()))
        self.seek(window, 2.)
        window.open_developer_window("control")
        self.assertIn("SECOND", window.developer_windows["control"].log_panel.text.toPlainText())
        self.assertEqual(window.latest_qimages["rgb"].pixelColor(0, 0).name(), "#0000ff")
        self.assertEqual(len(window.histories["public"]), 2)
        self.assertTrue(window.telemetry["planning"]["path"])
        self.seek(window, 1.)
        self.assertEqual(window.latest_qimages["rgb"].pixelColor(0, 0).name(), "#ff0000")
        self.assertEqual([p["x"] for p in window.histories["public"]], [1.])
        self.assertNotIn("planning", window.telemetry)
        self.assertNotIn("SECOND", window.developer_windows["control"].log_panel.text.toPlainText())
        self.seek(window, 0.)
        self.assertEqual(window.latest_qimages, {})
        self.assertFalse(window.panels["costmap"]._map_artist.get_visible())
        self.assertEqual(len(window.histories["public"]), 0)
        self.assertEqual(window.module_logs["planning"].events_after()[1], [])

    def test_pause_uses_recorded_time_for_freshness_and_speed_controls(self):
        window = self.window("--replay", str(self.fixture()))
        self.seek(window, 1.)
        self.assertAlmostEqual(window.age("pose", "public"), .25)
        self.assertEqual(window.wall_now(), STAMP+1.)
        for _ in range(3):
            window.advance_replay()
            window.refresh()
        self.assertAlmostEqual(window.age("pose", "public"), .25)
        window.replay_speed.setCurrentIndex(2)  # 2x
        window.toggle_replay()
        window._replay_wall_anchor -= .5
        window.advance_replay()
        self.assertGreaterEqual(window._replay_position, 2.)
        window.pause_replay()
        self.seek(window, 4.)
        self.assertAlmostEqual(window.age("pose", "public"), 2.)
        self.assertFalse(window._replay_playing)
        window.toggle_replay()
        self.assertTrue(window._replay_playing)

    def test_seek_retains_last_valid_frame_when_later_payload_is_invalid(self):
        path = self.root / "retained_frame.nomad"
        recorder = SessionRecorder(path, {"started_wall_s": STAMP})
        recorder.append("jetson", image("rgb", "red", STAMP+1), 1.)
        bad = image("rgb", "blue", STAMP+2)
        bad["data"] = "not-valid-base64"
        recorder.append("jetson", bad, 2.)
        recorder.close(duration_s=3.)
        window = self.window("--replay", str(path))
        self.seek(window, 3.)
        self.assertEqual(window.latest_qimages["rgb"].pixelColor(0, 0).name(), "#ff0000")
        self.assertTrue(window.images["rgb"]["decode_error"])

    def test_playback_never_constructs_network_or_command_workers(self):
        path = self.fixture()
        prohibited = AssertionError("offline playback must not connect to boards")
        with patch.object(monitor, "Collector", side_effect=prohibited), \
                patch.object(monitor, "RelayReceiver", side_effect=prohibited), \
                patch("relay.RelayServer", side_effect=prohibited), \
                patch("module_log_client.ModuleLogClient", side_effect=prohibited), \
                patch("planning_client.PlanningClient", side_effect=prohibited), \
                patch.object(monitor, "TrialStopWorker", side_effect=prohibited), \
                patch.object(monitor, "ModelSwitchWorker", side_effect=prohibited):
            window = self.window("--replay", str(path), "--pi-command", "NEVER",
                                 "--relay-listen", "8768", "--enable-trial-control")
            self.seek(window, 2.)
            for part in ("localization", "perception", "traversability", "planning", "control"):
                window.open_developer_window(part)
            window.connect_planner()
            window.arm_output()
            window.disarm_output()
            window.send_goal()
            window.cancel_goal()
            window.stop_trial()
            window.apply_model()
            self.assertEqual(window.workers, [])
            self.assertEqual(window.log_workers, {})
            self.assertIsNone(window.relay_server)
            self.assertIsNone(window.planning_client)
            self.assertFalse(window.goal_send.isEnabled())
            self.assertFalse(window.output_arm.isEnabled())

    def test_corrupt_event_error_survives_refresh_and_stops_playback(self):
        path = self.fixture()
        with sqlite3.connect(path) as db:
            db.execute("UPDATE events SET event_json=? WHERE seq=(SELECT MAX(seq) FROM events)", (b"broken-json",))
        window = self.window("--replay", str(path))
        window.seek_replay(4.)
        for _ in range(10):
            window.advance_replay()
        window.refresh()
        self.assertIn("기록 재생 오류", window.record_label.text())
        self.assertFalse(window.replay_play.isEnabled())
        self.assertFalse(window.replay_slider.isEnabled())


if __name__ == "__main__":
    unittest.main()
