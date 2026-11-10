"""Bounded ingestion/ordering contracts. Synthetic data; no board or SSH."""
import base64
import os
from pathlib import Path
import sys
import time
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_ingress_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtGui, QtWidgets
import monitor
from relay import CaptureOrder, LatestPacketBuffer


def pose(stamp, **extra):
    return {"kind": "pose", "source": "vio", "t": stamp,
            "x": stamp, "y": 0., "z": 0., "qx": 0., "qy": 0., "qz": 0., "qw": 1.,
            "frame_id": "odom", **extra}


class IngressTest(unittest.TestCase):
    def test_large_burst_has_one_compressed_image_and_bounded_pose_history(self):
        inbox = LatestPacketBuffer(clock=lambda: 100.)
        for index in range(1, 20001):
            inbox.put("jetson", {"kind": "image", "source": "rgb", "t": float(index),
                                "encoding": "jpeg", "data": "compressed-data", "width": 640, "height": 360})
            inbox.put("pi", pose(float(index)))
        state = inbox.status()
        self.assertEqual(state["pending"], 129)
        self.assertEqual(state["coalesced"], 19999)
        self.assertEqual(state["dropped_poses"], 20000 - 128)
        packets = inbox.drain()
        images = [event for board, event in packets if event["kind"] == "image"]
        poses = [event for board, event in packets if event["kind"] == "pose"]
        self.assertEqual(len(images), 1)
        self.assertEqual(images[0]["t"], 20000.)
        self.assertEqual([event["t"] for event in poses], list(map(float, range(19873, 20001))))
        self.assertEqual(inbox.status()["pending"], 0)

    def test_old_capture_and_wrong_board_never_replace_latest(self):
        inbox = LatestPacketBuffer(clock=lambda: 100.)
        event = {"kind": "image", "source": "rgb", "t": 20., "data": "new"}
        self.assertTrue(inbox.put("jetson", event))
        self.assertFalse(inbox.put("jetson", {**event, "t": 19., "data": "old"}))
        self.assertFalse(inbox.put("pi", {**event, "t": 21., "data": "wrong-owner"}))
        self.assertEqual(inbox.drain()[0][1]["data"], "new")

    def test_session_clock_reset_is_explicit_and_retired_session_is_rejected(self):
        order = CaptureOrder()
        key = ("pi", "pose", "vio")
        self.assertTrue(order.accept(key, pose(200., _source_session="a")))
        self.assertFalse(order.accept(key, pose(1., _source_session="a")))
        self.assertTrue(order.accept(key, pose(1., _source_session="b")))
        self.assertFalse(order.accept(key, pose(201., _source_session="a")))
        self.assertTrue(order.accept(key, pose(2., _source_session="b")))
        inbox = LatestPacketBuffer(clock=lambda: 100.)
        inbox.put("pi", pose(200.))
        old_session = inbox.session
        inbox.new_session()
        self.assertNotEqual(old_session, inbox.session)
        self.assertTrue(inbox.put("pi", pose(1.)))
        self.assertEqual(inbox.drain()[0][1]["t"], 1.)

    def test_local_queue_residence_adds_to_incoming_relay_age(self):
        now = [100.]
        inbox = LatestPacketBuffer(clock=lambda: now[0])
        inbox.put("pi", {**pose(20.), "_relay_age_s": 2., "_local_received_monotonic": 999.})
        now[0] = 105.
        event = inbox.drain()[0][1]
        self.assertEqual(event["_local_received_monotonic"], 98.)
        self.assertEqual(now[0] - event["_local_received_monotonic"], 7.)


class IngressQtTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        with mock.patch.object(sys, "argv", ["monitor", "--no-ssh"]):
            self.window = monitor.MonitorWindow(monitor.parse_args())
        self.window.timer.stop()
        self.window.ingest_timer.stop()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def test_delayed_gui_drain_and_relay_preserve_true_arrival_age(self):
        class Server:
            def __init__(self):
                self.events = []
            def publish(self, board, event):
                self.events.append((board, event))
            def close(self):
                pass
        server = self.window.relay_server = Server()
        inbox = LatestPacketBuffer(clock=lambda: time.monotonic() - 5.)
        inbox.put("pi", {**pose(time.time()), "_relay_age_s": 2.})
        board, event = inbox.drain()[0]
        self.window.on_packet(board, event)
        self.assertGreaterEqual(self.window.age("pose", "vio"), 7.)
        forwarded = server.events[0][1]
        self.assertGreaterEqual(forwarded["_relay_age_s"], 7.)
        self.assertNotIn("_local_received_monotonic", forwarded)

    def test_image_capture_regression_keeps_newest_pixels_and_session_reset_clears_pose_history(self):
        image = QtGui.QImage(20, 16, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("red"))
        buffer = QtCore.QBuffer()
        buffer.open(QtCore.QIODevice.WriteOnly)
        self.assertTrue(image.save(buffer, "PNG"))
        event = {"kind": "image", "source": "rgb", "t": 20., "width": 20, "height": 16,
                 "encoding": "png", "data": base64.b64encode(bytes(buffer.data())).decode("ascii")}
        self.window.on_packet("jetson", event)
        self.window.on_packet("jetson", {**event, "t": 19., "data": "old-invalid-image"})
        self.window.refresh()
        self.assertEqual(self.window.images["rgb"]["t"], 20.)
        self.assertEqual(self.window.latest_qimages["rgb"].pixelColor(0, 0).name(), "#ff0000")
        self.window.on_packet("pi", pose(200., _source_session="a"))
        self.window.on_packet("pi", pose(1., _source_session="b"))
        self.window.on_packet("pi", pose(201., _source_session="a"))
        self.assertEqual([p["t"] for p in self.window.histories["vio"]], [1.])


if __name__ == "__main__":
    unittest.main()
