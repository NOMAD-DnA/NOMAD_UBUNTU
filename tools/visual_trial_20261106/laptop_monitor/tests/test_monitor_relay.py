"""Offline Qt contracts for the operator/spectator split. No boards or SSH."""
import base64
import contextlib
import io
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_monitor_relay_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtGui, QtWidgets
import monitor


class FakeClient:
    """Block exactly like the backend run/stop API, without opening a socket."""
    instances = []

    def __init__(self, host, port, on_packet, on_progress):
        self.host, self.port = host, port
        self.on_packet, self.on_progress = on_packet, on_progress
        self.done, self.started = threading.Event(), threading.Event()
        self.instances.append(self)

    def run(self):
        self.started.set()
        self.on_progress({"state": "connected", "host": self.host, "port": self.port})
        self.done.wait(5)
        self.on_progress({"state": "stopped", "host": self.host, "port": self.port})

    def stop(self):
        self.done.set()


class FakeServer:
    instances = []

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.events = []
        self.running = False
        self.closed = False
        self.instances.append(self)

    def start(self):
        self.running = True

    def publish(self, board, event):
        self.events.append((board, dict(event)))
        return True

    def status(self):
        return {"host": self.host, "port": self.port, "client_count": 2,
                "stream_count": len(self.events), "running": self.running}

    def close(self):
        self.running = False
        self.closed = True


class MonitorRelayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.windows = []
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(mock.patch("relay.RelayClient", FakeClient))
        self.stack.enter_context(mock.patch("relay.RelayServer", FakeServer))
        self.stack.enter_context(mock.patch.object(monitor.QtNetwork.QNetworkInterface, "allAddresses", return_value=[]))
        FakeClient.instances.clear()
        FakeServer.instances.clear()

    def tearDown(self):
        for window in self.windows:
            window.close()
        self.app.processEvents()

    def args(self, *options):
        with mock.patch.object(sys, "argv", ["monitor", *options]):
            return monitor.parse_args()

    def window(self, *options):
        window = monitor.MonitorWindow(self.args(*options))
        self.windows.append(window)
        return window

    def test_spectator_never_constructs_collectors_and_cannot_control_boards(self):
        with mock.patch.object(monitor, "Collector", side_effect=AssertionError("SSH prohibited")), \
                mock.patch.object(monitor, "TrialStopWorker", side_effect=AssertionError("stop prohibited")), \
                mock.patch.object(monitor, "ModelSwitchWorker", side_effect=AssertionError("model prohibited")), \
                mock.patch.object(QtWidgets.QInputDialog, "getText", side_effect=AssertionError("password prohibited")):
            window = self.window("--relay-host", "192.0.2.10:8768", "--enable-trial-control",
                                 "--pi-command", "do-not-run", "--jetson-command", "do-not-run")
            self.assertTrue(FakeClient.instances[-1].started.wait(1))
            self.app.processEvents()
            self.assertEqual(window.workers, [])
            self.assertFalse(window.stop_button.isEnabled())
            self.assertFalse(window.model_button.isEnabled())
            self.assertFalse(window.model_combo.isEnabled())
            window.stop_trial()
            window.apply_model()
            # A late result callback also cannot re-enable remote controls.
            window.on_model_switch_finished()
            self.assertFalse(window.stop_button.isEnabled())
            self.assertFalse(window.model_button.isEnabled())
            self.assertIsNone(window.stop_worker)
            self.assertIsNone(window.model_worker)
            self.assertFalse(window.state()["trial_control"]["enabled"])
            window.refresh()
            self.assertIn("읽기 전용", window.relay_label.text())
            self.assertIn("192.0.2.10:8768", window.relay_label.text())
            self.assertEqual(FakeServer.instances, [])

    def test_spectator_uses_shared_developer_snapshot_and_closes_receiver(self):
        window = self.window("--relay-host", "192.0.2.10:8768")
        self.assertTrue(FakeClient.instances[-1].started.wait(1))
        for part in ("localization", "perception", "traversability"):
            window.open_developer_window(part)
        self.assertEqual(window.workers, [])
        self.assertIs(window.developer_snapshot()["histories"], window.histories)
        before = time.monotonic()
        window.close()
        self.assertLess(time.monotonic() - before, 1)
        self.assertFalse(window.relay_receiver.isRunning())
        self.assertTrue(FakeClient.instances[-1].done.is_set())
        self.assertTrue(all(not child.isVisible() for child in window.developer_windows.values()))

    def test_cached_ages_apply_to_all_streams_without_rewriting_capture_time(self):
        window = self.window("--relay-host", "192.0.2.10:8768")
        image = QtGui.QImage(20, 16, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("#405060"))
        buffer = QtCore.QBuffer()
        buffer.open(QtCore.QIODevice.WriteOnly)
        self.assertTrue(image.save(buffer, "PNG"))
        stamp = 1794000000.
        events = [
            {"kind": "pose", "source": "public", "t": stamp, "x": 1., "y": 2., "z": 0.,
             "qx": 0., "qy": 0., "qz": 0., "qw": 1., "frame_id": "odom"},
            {"kind": "image", "source": "rgb", "t": stamp, "width": 20, "height": 16,
             "encoding": "png", "data": base64.b64encode(bytes(buffer.data())).decode("ascii")},
            {"kind": "status", "source": "localization", "t": stamp, "state": "READY"},
            {"kind": "rates", "source": "pi", "rates": {"vio": 10.}},
            {"kind": "telemetry", "source": "gnss", "t": stamp, "received": True,
             "fix_age_s": 0., "fix_valid": True},
        ]
        for event in events:
            board = "jetson" if event["kind"] == "image" else "pi"
            window.on_packet(board, {**event, "_relay_age_s": 12.})
            self.assertGreaterEqual(window.age(event["kind"], event["source"]), 12.)
        self.assertEqual(window.histories["public"][-1]["t"], stamp)
        self.assertEqual(window.images["rgb"]["t"], stamp)
        self.assertEqual(window.statuses["localization"]["t"], stamp)
        self.assertEqual(window.telemetry["gnss"]["t"], stamp)
        self.assertEqual(window.panels["costmap"].pose_age, 12.)
        window.refresh()
        self.assertIn("STALE", window.status_label.text())
        self.assertNotIn("rgb", window.panels)
        self.assertIn("STALE", window.age_text("image", "rgb"))
        self.assertEqual(window.latest_qimages["rgb"].width(), 20)
        for age in (-1., float("nan"), True):
            window.on_packet("pi", {**events[0], "_relay_age_s": age})
        self.assertEqual(window.accepted_events, len(events))
        self.assertEqual(window.invalid_events, 3)

    def test_operator_fans_out_each_accepted_raw_packet_once_and_closes_server(self):
        window = self.window("--no-ssh", "--relay-listen", "8768")
        server = FakeServer.instances[-1]
        self.assertEqual((server.host, server.port), ("0.0.0.0", 8768))
        stamp = 1794000000.
        event = {"kind": "status", "source": "localization", "t": stamp,
                 "state": "READY", "detail": "test fixture"}
        window.on_packet("pi", event)
        window.on_packet("pi", {**event, "t": float("nan")})
        window.refresh()
        window.refresh()
        self.assertEqual(server.events, [("pi", event)])
        self.assertEqual(server.events[0][1]["t"], stamp)
        self.assertNotIn("_relay_age_s", event)
        self.assertIn("관전 PC 2대", window.relay_label.text())
        window.close()
        self.assertTrue(server.closed)

    def test_magnetometer_packet_reaches_operator_and_spectator_without_board_control(self):
        import relay
        event = {"kind": "telemetry", "source": "magnetometer", "t": 1794000000.,
                 "received": True, "mag": {"frame_id": "zed_imu_link", "x_ut": 20.,
                 "y_ut": 0., "z_ut": -30., "stamp": 1794000000., "age_s": -.345,
                 "received_age_s": 0.}, "imu": None}
        self.assertIsNotNone(relay._validate("jetson", event))
        operator = self.window("--no-ssh", "--relay-listen", "8768")
        operator.on_packet("jetson", event)
        self.assertEqual(FakeServer.instances[-1].events, [("jetson", event)])
        spectator = self.window("--relay-host", "192.0.2.10:8768", "--enable-trial-control")
        spectator.on_packet("jetson", {**event, "_relay_age_s": 2.})
        self.assertEqual(spectator.telemetry["magnetometer"]["mag"]["x_ut"], 20.)
        self.assertGreaterEqual(spectator.age("telemetry", "magnetometer"), 2.)
        self.assertFalse(spectator.stop_button.isEnabled())

    def test_bind_failure_happens_before_any_board_collector(self):
        args = self.args("--relay-listen", "8768", "--pi-command", "do-not-run")
        with mock.patch.object(FakeServer, "start", side_effect=OSError("occupied test port")), \
                mock.patch.object(monitor, "Collector", side_effect=AssertionError("SSH prohibited")):
            with self.assertRaises(OSError):
                monitor.MonitorWindow(args)
        self.assertTrue(FakeServer.instances[-1].closed)

    def test_legacy_no_ssh_and_older_argument_fixture_do_not_start_relay(self):
        args = self.args("--no-ssh")
        del args.relay_host
        del args.relay_listen
        window = monitor.MonitorWindow(args)
        self.windows.append(window)
        self.assertEqual(FakeServer.instances, [])
        self.assertEqual(FakeClient.instances, [])
        self.assertEqual(window.workers, [])
        self.assertEqual(window.state()["relay"]["mode"], "direct")

    def test_argument_modes_and_address_validation(self):
        for options in (("--relay-listen", "8768", "--relay-host", "host:8768"),
                        ("--relay-listen", "0"), ("--relay-listen", "65536"),
                        ("--relay-host", "missing-port"), ("--relay-host", "host:0"),
                        ("--relay-host", "bad host:8768")):
            with self.subTest(options=options), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    self.args(*options)
        self.assertEqual(monitor.relay_endpoint("host.local:8768"), ("host.local", 8768))


if __name__ == "__main__":
    unittest.main()
