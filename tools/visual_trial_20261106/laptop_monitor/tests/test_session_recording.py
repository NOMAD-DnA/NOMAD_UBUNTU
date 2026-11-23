"""Real SQLite round trips and explicit failure cases, without boards or Qt."""
import base64
from pathlib import Path
import sqlite3
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from session_recording import (APPLICATION_ID, RecordingError, SessionReader,
                               SessionRecorder)


class BlockedRecorder(SessionRecorder):
    """Hold disk startup to test an overflowing producer deterministically."""
    gate = None

    def _connect(self):
        self.gate.wait(3)
        return super()._connect()


class SessionRecordingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "trial.nomad"

    def tearDown(self):
        self.tmp.cleanup()

    def test_every_display_payload_roundtrips_without_timestamp_rewrite(self):
        image = base64.b64encode(b"\xff\xd8original-jpeg\xff\xd9").decode("ascii")
        events = [
            ("pi", {"kind": "pose", "source": "vio", "t": 10., "x": 1.,
                    "y": 2., "z": 3., "_local_received_monotonic": 42.}),
            ("jetson", {"kind": "image", "source": "rgb", "t": 8., "width": 640,
                        "height": 360, "encoding": "jpeg", "data": image}),
            ("jetson", {"kind": "grid", "source": "costmap", "t": 9.,
                        "origin": [-1.5, 0.], "cells": [[0, 0, -1], [1, 0, 100]],
                        "resolution": .125}),
            ("pi", {"kind": "status", "source": "localization", "state": "READY"}),
            ("jetson", {"kind": "rates", "rates": {"rgb": 1., "costmap": 1.}}),
            ("pi4", {"kind": "telemetry", "armed": True, "erpm": -2.}),
            ("pi4", {"kind": "log", "lines": ["한글 로그", "LPP pause"]}),
        ]
        recorder = SessionRecorder(self.path, {"started_wall_s": 1794270000.,
                                               "forward_axis": "+Y"})
        for index, (board, event) in enumerate(events):
            self.assertTrue(recorder.append(board, event, index * .5))
        # Accepted event is a copy, even when nested data changes immediately.
        events[2][1]["cells"][0][2] = 999
        status = recorder.close(duration_s=10.)
        self.assertEqual(status["state"], "CLOSED")
        self.assertTrue(status["complete"])
        self.assertEqual(status["events"], 7)
        self.assertEqual(status["queue_bytes"], 0)
        self.assertEqual(recorder.close(duration_s=20.), status)
        self.assertFalse(recorder.append("pi", {}, 12.))
        reader = SessionReader(self.path)
        self.addCleanup(reader.close)
        self.assertEqual(reader.duration, 10.)
        self.assertEqual(reader.metadata["forward_axis"], "+Y")
        self.assertTrue(reader.metadata["_recording"]["complete"])
        saved = list(reader.iter_until(10., chunk_size=2))
        self.assertEqual([item["seq"] for item in saved], list(range(1, 8)))
        self.assertEqual(saved[1]["event"]["data"], image)
        self.assertEqual(saved[2]["event"]["cells"][0][2], -1)
        self.assertEqual([item["event"].get("t") for item in saved[:3]], [10., 8., 9.])
        self.assertEqual(saved[0]["event"]["_local_received_monotonic"], 42.)
        self.assertEqual(saved[-1]["event"]["lines"][0], "한글 로그")
        self.assertEqual([item["seq"] for item in reader.events_until(.5)], [1, 2])
        self.assertEqual([item["seq"] for item in reader.events_until(1., after_seq=2)], [3])
        # Seek rebuild restarts at beginning, preserving earlier state.
        self.assertEqual([item["seq"] for item in reader.iter_until(0.)], [1])

    def test_equal_offsets_keep_sequence_and_large_stream_reads_in_chunks(self):
        recorder = SessionRecorder(self.path)
        for index in range(1500):
            self.assertTrue(recorder.append("pi", {"index": index}, 1.))
        self.assertEqual(recorder.close()["events"], 1500)
        reader = SessionReader(self.path)
        self.addCleanup(reader.close)
        first = reader.events_until(1.)
        self.assertEqual(len(first), 512)
        self.assertEqual(len(list(reader.iter_until(1.))), 1500)
        self.assertEqual(reader.events_until(.99), [])
        self.assertEqual(reader.events_until(1., after_seq=1499)[0]["event"]["index"], 1499)

    def test_overflow_is_visible_and_accepted_prefix_is_preserved_incomplete(self):
        BlockedRecorder.gate = threading.Event()
        recorder = BlockedRecorder(self.path, max_queue_bytes=180)
        try:
            self.assertTrue(recorder.append("pi", {"data": "a" * 40}, 0.))
            self.assertFalse(recorder.append("pi", {"data": "b" * 40}, 1.))
            self.assertEqual(recorder.status()["state"], "FAILED")
            self.assertIn("queue", recorder.status()["error"])
        finally:
            BlockedRecorder.gate.set()
            status = recorder.close()
        self.assertEqual(status["state"], "FAILED")
        self.assertEqual(status["events"], 1)
        reader = SessionReader(self.path)
        self.addCleanup(reader.close)
        self.assertFalse(reader.metadata["_recording"]["complete"])
        self.assertIn("queue", reader.metadata["_recording"]["error"])
        self.assertEqual(len(reader.events_until(10.)), 1)

    def test_bad_offset_or_serialization_fails_instead_of_silent_loss(self):
        recorder = SessionRecorder(self.path)
        self.assertTrue(recorder.append("pi", {"t": 100.}, 1.))
        self.assertFalse(recorder.append("pi", {"t": 101.}, .5))
        self.assertIn("backwards", recorder.close()["error"])
        self.assertEqual(recorder.status()["events"], 1)
        for index, bad in enumerate([{"bad": float("nan")}, {"bad": object()}]):
            other = SessionRecorder(Path(self.tmp.name) / ("bad%d.nomad" % index))
            self.assertFalse(other.append("pi", bad, 0.))
            self.assertEqual(other.close()["state"], "FAILED")

    def test_disk_error_and_close_timeout_are_explicit(self):
        class FailedDiskRecorder(SessionRecorder):
            def _connect(self):
                raise sqlite3.OperationalError("disk is full")
        failed = FailedDiskRecorder(self.path)
        result = failed.close()
        self.assertEqual(result["state"], "FAILED")
        self.assertIn("disk is full", result["error"])
        with self.assertRaises(RecordingError):
            SessionReader(self.path)
        BlockedRecorder.gate = threading.Event()
        blocked = BlockedRecorder(Path(self.tmp.name) / "blocked.nomad", close_timeout_s=.03)
        start = time.monotonic()
        status = blocked.close()
        self.assertLess(time.monotonic() - start, .3)
        self.assertEqual(status["state"], "FAILED")
        self.assertIn("timed out", status["error"])
        BlockedRecorder.gate.set()
        blocked.close()

    def test_existing_file_corrupt_file_unknown_version_and_missing_event(self):
        recorder = SessionRecorder(self.path)
        for index in range(3):
            self.assertTrue(recorder.append("pi", {"index": index}, index))
        recorder.close()
        with self.assertRaises(FileExistsError):
            SessionRecorder(self.path)
        db = sqlite3.connect(self.path)
        db.execute("PRAGMA user_version=99")
        db.commit()
        with self.assertRaisesRegex(RecordingError, "version"):
            SessionReader(self.path)
        db.execute("PRAGMA user_version=1")
        db.execute("DELETE FROM events WHERE seq=2")
        db.commit()
        db.close()
        with self.assertRaisesRegex(RecordingError, "sequence"):
            SessionReader(self.path)
        other = Path(self.tmp.name) / "garbage.nomad"
        other.write_bytes(b"not sqlite")
        with self.assertRaises(RecordingError):
            SessionReader(other)
        with self.assertRaises(RecordingError):
            SessionReader(Path(self.tmp.name) / "missing.nomad")

    def test_bad_event_payload_is_reported_when_read(self):
        recorder = SessionRecorder(self.path)
        recorder.append("pi", {"kind": "pose", "t": 1.}, 0.)
        recorder.close()
        db = sqlite3.connect(self.path)
        db.execute("UPDATE events SET event_json=? WHERE seq=1", (b'{"bad":NaN}',))
        db.commit()
        db.close()
        reader = SessionReader(self.path)
        self.addCleanup(reader.close)
        with self.assertRaisesRegex(RecordingError, "Non-finite"):
            reader.events_until(1.)


if __name__ == "__main__":
    unittest.main()
