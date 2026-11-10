"""Actual loopback socket tests; no Qt, SSH, board, or sensor processes."""
import base64
import json
from pathlib import Path
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import relay


def pose(index=1):
    return {"kind": "pose", "source": "vio", "t": 1700000000.0 + index,
            "x": float(index), "y": 2., "z": 3., "qx": 0., "qy": 0., "qz": 0., "qw": 1.}


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("condition did not become true")


class RelayTest(unittest.TestCase):
    def setUp(self):
        self.server = relay.RelayServer("127.0.0.1", 0).start()
        self.clients = []
        self.sockets = []

    def tearDown(self):
        for client, thread, _packets, _progress in self.clients:
            client.stop()
        self.server.close()
        for sock in self.sockets:
            sock.close()
        for _client, thread, _packets, _progress in self.clients:
            thread.join(2)
            self.assertFalse(thread.is_alive(), "relay client shutdown leaked a thread")
        self.assertEqual(self.server.status()["client_count"], 0)

    def client(self):
        packets, progress = [], []
        client = relay.RelayClient("127.0.0.1", self.server.port,
                                   lambda board, event: packets.append((board, event)), progress.append)
        thread = threading.Thread(target=client.run, daemon=True)
        item = client, thread, packets, progress
        self.clients.append(item)
        thread.start()
        wait_for(lambda: any(p["state"] == "connected" for p in progress))
        return item

    def test_two_clients_latest_snapshot_preserves_time_and_age(self):
        original = pose()
        self.assertTrue(self.server.publish("pi", original))
        original["x"] = 999.  # Caller mutation must not change the cached event.
        time.sleep(.12)
        first, first_thread, a, _ = self.client()
        _second, _second_thread, b, _ = self.client()
        wait_for(lambda: len(a) == 1 and len(b) == 1)
        for board, event in a + b:
            self.assertEqual(board, "pi")
            self.assertEqual(event["t"], 1700000001.)
            self.assertEqual(event["x"], 1.)
            self.assertGreaterEqual(event["_relay_age_s"], .1)
        self.assertTrue(self.server.publish("pi", pose(2)))
        wait_for(lambda: len(a) == 2 and len(b) == 2)
        self.assertEqual(a[-1][1]["t"], b[-1][1]["t"])
        first.stop(); first_thread.join(1)
        wait_for(lambda: self.server.status()["client_count"] == 1)
        # A later connection sees only each stream's latest capture, with its
        # true operator-cache age; a new TCP session does not make it fresh.
        time.sleep(.12)
        _third, _third_thread, c, _ = self.client()
        wait_for(lambda: len(c) == 1)
        self.assertEqual(c[0][1]["t"], pose(2)["t"])
        self.assertGreaterEqual(c[0][1]["_relay_age_s"], .1)
        self.assertEqual(self.server.status()["stream_count"], 1)

    def test_streams_age_accumulation_validation_and_bind_error(self):
        event = {"kind": "telemetry", "source": "gnss", "t": 1700000000.,
                 "received": False, "_relay_age_s": 8.}
        self.assertTrue(self.server.publish("pi", event))
        self.assertTrue(self.server.publish("jetson", {
            "kind": "rates", "source": "jetson", "rates": {"rgb": 3.}}))
        self.assertTrue(self.server.publish("pi", {"kind": "status", "source": "localization",
                                                  "t": 1700000000., "state": "WAITING"}))
        _client, _thread, packets, _progress = self.client()
        wait_for(lambda: len(packets) == 3)
        gnss = next(event for _, event in packets if event["source"] == "gnss")
        self.assertGreaterEqual(gnss["_relay_age_s"], 8.)
        self.assertNotIn("t", next(event for _, event in packets if event["kind"] == "rates"))
        rejected = [
            ("pi", {"kind": "control", "source": "motor", "t": 1.}),
            ("other", pose()), ("pi", {**pose(), "source": []}),
            ("pi", {**pose(), "qw": 0.}), ("pi", {**pose(), "t": float("nan")}),
            ("pi", {**event, "nested": {"bad": float("inf")}}),
            ("pi", {**event, "values": "x" * 70000}),
            ("pi", {"kind": "image", "source": "rgb", "t": 1., "width": 20,
                    "height": 10, "encoding": "jpeg", "data": "broken"}),
        ]
        for board, bad in rejected:
            self.assertFalse(self.server.publish(board, bad))
        self.assertEqual(self.server.status()["rejected"], len(rejected))
        duplicate = relay.RelayServer("127.0.0.1", self.server.port)
        with self.assertRaises(OSError):
            duplicate.start()
        duplicate.close()

    def test_slow_reader_is_bounded_and_does_not_block_publish(self):
        slow = socket.socket()
        slow.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        slow.connect(("127.0.0.1", self.server.port))
        self.sockets.append(slow)
        wait_for(lambda: self.server.status()["client_count"] == 1)
        # A valid JPEG envelope large enough to fill a deliberately unread
        # socket. Image decoding belongs to the viewer, not this TCP relay.
        payload = base64.b64encode(b"\xff\xd8" + b"a" * 1500000 + b"\xff\xd9").decode("ascii")
        image = {"kind": "image", "source": "rgb", "t": 1700000000., "width": 640,
                 "height": 480, "encoding": "jpeg", "data": payload}
        self.assertTrue(self.server.publish("jetson", image))
        start = time.monotonic()
        for i in range(300):
            self.assertTrue(self.server.publish("pi", pose(i)))
        self.assertLess(time.monotonic() - start, .5, "publish waited for slow reader")
        self.assertEqual(self.server.status()["stream_count"], 2)
        # A late healthy reader receives the latest pose, not 300 old poses.
        _client, _thread, packets, _progress = self.client()
        wait_for(lambda: any(e["kind"] == "pose" for _, e in packets))
        poses = [e for _, e in packets if e["kind"] == "pose"]
        self.assertEqual(poses[-1]["x"], 299.)
        self.assertLessEqual(len(poses), 2)
        wait_for(lambda: self.server.status()["slow_disconnects"] >= 1)

    def test_reconnect_and_stop_wait_are_interruptible(self):
        _client, _thread, packets, progress = self.client()
        self.server.publish("pi", pose())
        wait_for(lambda: len(packets) == 1)
        port = self.server.port
        self.server.close()
        wait_for(lambda: any(item["state"] == "disconnected" for item in progress))
        self.server = relay.RelayServer("127.0.0.1", port).start()
        self.server.publish("pi", pose(9))
        wait_for(lambda: any(event["x"] == 9. for _, event in packets))
        start = time.monotonic()
        _client.stop(); _thread.join(1)
        self.assertFalse(_thread.is_alive())
        self.assertLess(time.monotonic() - start, .6)

    def test_client_rejects_invalid_envelopes_and_oversize_line(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0)); listener.listen(1)
        self.sockets.append(listener)
        packets, progress = [], []
        client = relay.RelayClient("127.0.0.1", listener.getsockname()[1],
                                   lambda board, event: packets.append((board, event)), progress.append)
        thread = threading.Thread(target=client.run, daemon=True)
        self.clients.append((client, thread, packets, progress)); thread.start()
        peer, _ = listener.accept(); self.sockets.append(peer)
        peer.sendall(b'{"command":"start"}\n')
        peer.sendall(json.dumps({"board": "pi", "event": {**pose(), "_relay_age_s": -1}}).encode() + b"\n")
        peer.sendall(json.dumps({"board": "pi", "event": {**pose(), "_relay_age_s": 1.5}}).encode() + b"\n")
        wait_for(lambda: len(packets) == 1)
        self.assertEqual(sum(item["state"] == "invalid" for item in progress), 2)
        self.assertEqual(packets[0][1]["_relay_age_s"], 1.5)
        # Use a small temporary wire bound to exercise the exact socket parser
        # overflow branch without allocating a sixteen-MiB unit-test buffer.
        old_limit = relay.MAX_LINE_BYTES
        try:
            relay.MAX_LINE_BYTES = 1024
            peer.sendall(b"x" * 1025)
            wait_for(lambda: any("exceeds limit" in p.get("error", "") for p in progress))
        finally:
            relay.MAX_LINE_BYTES = old_limit

    def test_inbound_bytes_cannot_be_commands_and_close_is_idempotent(self):
        sock = socket.create_connection(("127.0.0.1", self.server.port))
        self.sockets.append(sock)
        wait_for(lambda: self.server.status()["client_count"] == 1)
        sock.sendall(b'{"command":"start_motor"}\n')
        wait_for(lambda: self.server.status()["client_count"] == 0)
        self.assertEqual(self.server.status()["published"], 0)
        start = time.monotonic()
        self.server.close(); self.server.close()
        self.assertLess(time.monotonic() - start, .7)
        self.assertFalse(self.server.publish("pi", pose()))


if __name__ == "__main__":
    unittest.main()
