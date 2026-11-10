"""Read-only operator-to-spectator TCP JSONL relay (standard library only).

Each client sees latest values, not an ever-growing replay queue. No command,
SSH, credential, or board-control protocol is implemented by this module.
"""
from __future__ import annotations

import base64
import binascii
from collections import deque
from dataclasses import dataclass
import json
import math
import select
import socket
import threading
import time
import uuid


MAX_LINE_BYTES = 16 * 1024 * 1024
MAX_CLIENTS = 8
SEND_TIMEOUT_S = 0.5
_SOURCES = {
    "pose": frozenset(("vio", "fusion", "public")),
    "image": frozenset(("rgb", "semantic", "slope", "costmap", "vio_tracking")),
    "status": frozenset(("localization", "perception")),
    "rates": frozenset(("pi", "jetson")),
    "telemetry": frozenset(("gnss", "fusion", "vio_features", "magnetometer", "planning", "slope_cpu")),
    "log": frozenset(("localization", "perception", "traversability", "planning")),
}

LOG_OWNERS = {"pi": frozenset(("localization",)),
              "jetson": frozenset(("localization", "perception", "traversability")),
              "pi4": frozenset(("planning",))}

STREAM_OWNERS = {
    **{("pose", source): "pi" for source in ("vio", "fusion", "public")},
    **{("image", source): "jetson" for source in ("rgb", "semantic", "slope", "costmap")},
    ("image", "vio_tracking"): "pi",
    ("status", "localization"): "pi", ("status", "perception"): "jetson",
    ("rates", "pi"): "pi", ("rates", "jetson"): "jetson",
    **{("telemetry", source): "pi" for source in ("gnss", "fusion", "vio_features")},
    ("telemetry", "slope_cpu"): "jetson",
    ("telemetry", "magnetometer"): "jetson", ("telemetry", "planning"): "pi4",
}


def owns_stream(board, event):
    kind, source = event.get("kind"), event.get("source")
    if kind == "log":
        return isinstance(source, str) and source in LOG_OWNERS.get(board, ())
    return (isinstance(kind, str) and isinstance(source, str) and
            STREAM_OWNERS.get((kind, source)) == board)


def valid_log_event(board, event):
    return (isinstance(event, dict) and event.get("kind") == "log" and owns_stream(board, event)
            and _finite(event.get("t")) and event["t"] > 0
            and isinstance(event.get("text"), str) and len(event["text"].encode("utf-8")) <= 16384
            and isinstance(event.get("session"), str) and 0 < len(event["session"]) <= 128
            and isinstance(event.get("seq"), int) and not isinstance(event["seq"], bool)
            and 0 <= event["seq"] < 2**63)


class CaptureOrder:
    """Reject capture regressions within a producer connection/session.

    A new explicitly labelled session may restart its clock. Retired sessions
    cannot replace it later. A camera/VIO module restart normally retains wall
    time; reconnecting the collector gives clock resets a new session as well.
    """
    def __init__(self):
        self.latest = {}
        self.retired = {}

    def accept(self, key, event, commit=True):
        session = event.get("_source_session", "legacy")
        if not isinstance(session, str) or not 0 < len(session) <= 128:
            return False
        stamp = event.get("t")
        if stamp is not None and (not _finite(stamp) or stamp <= 0):
            return False
        previous = self.latest.get(key)
        if previous is not None:
            old_session, old_stamp = previous
            if session != old_session:
                retired = self.retired.get(key, ())
                if session in retired:
                    return False
                if commit:
                    self.retired.setdefault(key, deque(maxlen=16)).append(old_session)
            elif stamp is not None and old_stamp is not None and stamp < old_stamp:
                return False
        if commit:
            self.latest[key] = (session, stamp)
        return True


class LatestPacketBuffer:
    """Bounded receive-thread mailbox, drained by one 30 Hz GUI timer.

    Compressed images/state have one slot per stream. Poses retain the newest
    128 samples per stream (6.4 s at 20 Hz); overflow drops oldest samples and is
    counted. No image payload is placed in Qt's unbounded queued-signal list.
    """
    def __init__(self, pose_limit=128, clock=time.monotonic):
        self.pose_limit = pose_limit
        self.clock = clock
        self.session = uuid.uuid4().hex
        self._lock = threading.Lock()
        self._latest, self._poses = {}, {}
        self._order = CaptureOrder()
        self.dropped_poses = self.coalesced = self.rejected = 0

    def new_session(self):
        # Called on an actual relay reconnect, not on timestamp regression.
        with self._lock:
            self.session = uuid.uuid4().hex
            self._latest.clear()
            self._poses.clear()

    def put(self, board, event):
        if not isinstance(event, dict) or not owns_stream(board, event):
            with self._lock:
                self.rejected += 1
            return False
        age = event.get("_relay_age_s", 0.)
        if not _finite(age) or age < 0:
            with self._lock:
                self.rejected += 1
            return False
        item = dict(event)
        # This field is always replaced locally, never trusted from the wire.
        item["_local_received_monotonic"] = self.clock() - age
        item.setdefault("_source_session", self.session)
        key = (board, item["kind"], item["source"])
        with self._lock:
            if not self._order.accept(key, item):
                self.rejected += 1
                return False
            if item["kind"] == "pose":
                queue = self._poses.setdefault(key, deque(maxlen=self.pose_limit))
                if queue and queue[-1]["_source_session"] != item["_source_session"]:
                    queue.clear()
                if len(queue) == self.pose_limit:
                    self.dropped_poses += 1
                queue.append(item)
            else:
                if key in self._latest:
                    self.coalesced += 1
                self._latest[key] = item
        return True

    def drain(self):
        with self._lock:
            packets = [(key[0], item) for key, queue in self._poses.items() for item in queue]
            packets.extend((key[0], item) for key, item in self._latest.items())
            self._latest.clear()
            self._poses.clear()
        return sorted(packets, key=lambda pair: pair[1]["_local_received_monotonic"])

    def status(self):
        with self._lock:
            return {"pending": len(self._latest) + sum(map(len, self._poses.values())),
                    "dropped_poses": self.dropped_poses, "coalesced": self.coalesced,
                    "rejected": self.rejected, "pose_limit_per_stream": self.pose_limit}


def _finite(value):
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


def _validate(board, event):
    """Return a bounded immutable JSON snapshot, or None for malformed data."""
    if board not in ("pi", "jetson", "pi4") or not isinstance(event, dict) or not owns_stream(board, event):
        return None
    kind, source = event.get("kind"), event.get("source")
    if board == "pi4" and (kind, source) not in (("telemetry", "planning"), ("log", "planning")):
        return None
    if (not isinstance(kind, str) or kind not in _SOURCES or
            not isinstance(source, str) or source not in _SOURCES[kind]):
        return None
    stamp = event.get("t")
    # Existing C++ rates events have no measurement timestamp. Preserve that.
    if kind != "rates" or "t" in event:
        if not _finite(stamp) or stamp <= 0:
            return None
    age = event.get("_relay_age_s", 0.0)
    if not _finite(age) or age < 0:
        return None
    try:
        if kind == "pose":
            if not all(_finite(event.get(key)) for key in ("x", "y", "z", "qx", "qy", "qz", "qw")):
                return None
            if abs(math.hypot(*(event[key] for key in ("qx", "qy", "qz", "qw"))) - 1) > .02:
                return None
        elif kind == "image":
            if event.get("encoding") not in ("jpeg", "png") or not isinstance(event.get("data"), str):
                return None
            if not event["data"] or len(event["data"]) > MAX_LINE_BYTES - 4096:
                return None
            for name in ("width", "height"):
                value = event.get(name)
                if not isinstance(value, int) or isinstance(value, bool) or not 0 < value <= 8192:
                    return None
            image = base64.b64decode(event["data"], validate=True)
            if event["encoding"] == "jpeg":
                if not image.startswith(b"\xff\xd8") or not image.endswith(b"\xff\xd9"):
                    return None
            elif not image.startswith(b"\x89PNG\r\n\x1a\n"):
                return None
        elif kind == "status":
            if not isinstance(event.get("state"), str) or len(event["state"]) > 128:
                return None
            if not isinstance(event.get("detail", ""), str) or len(event.get("detail", "")) > 4096:
                return None
        elif kind == "rates":
            rates = event.get("rates")
            if not isinstance(rates, dict) or len(rates) > 128:
                return None
            if not all(isinstance(k, str) and len(k) <= 128 and _finite(v) and v >= 0 for k, v in rates.items()):
                return None
        elif kind == "log":
            if not valid_log_event(board, event):
                return None
        elif kind == "telemetry" and source == "planning":
            if event.get("frame") != "odom" or not isinstance(event.get("plan_only"), bool):
                return None
            for name in ("path", "trajectory"):
                points = event.get(name)
                if not isinstance(points, list) or len(points) > 600:
                    return None
                if not all(isinstance(p, list) and len(p) == 3 and all(_finite(v) for v in p) for p in points):
                    return None
            for name in ("gpp_state", "gpp_reason", "lpp_state", "lpp_reason"):
                if name in event and (not isinstance(event[name], str) or len(event[name]) > 400):
                    return None
            for name in ("path_valid", "executable"):
                if name in event and not isinstance(event[name], bool):
                    return None
            if "gpp_stamp_ns" in event and (isinstance(event["gpp_stamp_ns"], bool)
                    or not isinstance(event["gpp_stamp_ns"], int) or event["gpp_stamp_ns"] < 0):
                return None
        # Serialize exactly once per operator publication, independent of readers.
        # This copies/fixes caller-owned data and rejects NaN even in nested fields.
        clean = dict(event)
        clean.pop("_relay_age_s", None)
        clean.pop("_local_received_monotonic", None)
        body = json.dumps(clean, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        limit = 65536 if kind == "telemetry" else MAX_LINE_BYTES - 1024
        if len(body) > limit:
            return None
        return kind, source, body, float(age)
    except (ValueError, TypeError, OverflowError, RecursionError, binascii.Error):
        return None


@dataclass(frozen=True)
class _Record:
    sequence: int
    board: str
    body: bytes
    received: float
    initial_age: float

    def wire(self):
        age = max(0.0, time.monotonic() - self.received) + self.initial_age
        # body is a JSON object. Add receipt age without re-serializing its image.
        return (b'{"board":"' + self.board.encode("ascii") + b'","event":' +
                self.body[:-1] + b',"_relay_age_s":' + repr(age).encode("ascii") + b"}}\n")


class RelayServer:
    def __init__(self, host="0.0.0.0", port=8768):
        self.host, self.port = host, int(port)
        self._condition = threading.Condition()
        self._listener = None
        self._accept_thread = None
        self._clients = {}  # socket -> writer thread; no per-client image queues
        self._latest = {}
        self._running = False
        self._sequence = 0
        self._rejected = 0
        self._slow_disconnects = 0
        self._capture_order = CaptureOrder()

    def start(self):
        """Bind synchronously; bind/listen failures propagate to the caller."""
        with self._condition:
            if self._running:
                return self
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self.host, self.port))
                listener.listen(MAX_CLIENTS)
                listener.settimeout(.25)
            except BaseException:
                listener.close()
                raise
            self.port = listener.getsockname()[1]
            self._listener = listener
            self._running = True
            self._accept_thread = threading.Thread(target=self._accept, name="relay-accept", daemon=True)
            self._accept_thread.start()
        return self

    def publish(self, board, event):
        """Accept one display event; slow socket writes never execute here."""
        received = time.monotonic()
        validated = _validate(board, event)
        with self._condition:
            if validated is None:
                self._rejected += 1
                return False
            if not self._running:
                return False
            kind, source, body, initial_age = validated
            if not self._capture_order.accept((board, kind, source), event):
                self._rejected += 1
                return False
            self._sequence += 1
            self._latest[(board, kind, source)] = _Record(
                self._sequence, board, body, received, initial_age)
            self._condition.notify_all()
        return True

    def status(self):
        with self._condition:
            return {"host": self.host, "port": self.port, "running": self._running,
                    "client_count": len(self._clients), "stream_count": len(self._latest),
                    "published": self._sequence, "rejected": self._rejected,
                    "slow_disconnects": self._slow_disconnects}

    def _accept(self):
        while True:
            with self._condition:
                listener = self._listener
                if not self._running or listener is None:
                    return
            try:
                connection, _address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            connection.settimeout(SEND_TIMEOUT_S)
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
            with self._condition:
                if not self._running or len(self._clients) >= MAX_CLIENTS:
                    connection.close()
                    continue
                thread = threading.Thread(target=self._send, args=(connection,), name="relay-reader", daemon=True)
                self._clients[connection] = thread
                thread.start()

    def _send(self, connection):
        cursors = {}
        try:
            while True:
                with self._condition:
                    if not self._running:
                        return
                    pending = [(key, value) for key, value in self._latest.items()
                               if value.sequence > cursors.get(key, 0)]
                    if not pending:
                        self._condition.wait(.25)
                        pending = [(key, value) for key, value in self._latest.items()
                                   if value.sequence > cursors.get(key, 0)]
                    if not self._running:
                        return
                    item = min(pending, key=lambda item: item[1].sequence) if pending else None
                # This is a one-way protocol. EOF or any incoming data closes the
                # spectator; no received bytes can become a board-control action.
                if select.select([connection], [], [], 0)[0]:
                    return
                if item is None:
                    continue
                key, record = item
                connection.sendall(record.wire())
                cursors[key] = record.sequence
        except socket.timeout:
            with self._condition:
                self._slow_disconnects += 1
        except (OSError, ValueError):
            pass
        finally:
            connection.close()
            with self._condition:
                self._clients.pop(connection, None)

    def close(self):
        with self._condition:
            self._running = False
            listener, self._listener = self._listener, None
            clients = list(self._clients.items())
            accept_thread = self._accept_thread
            self._condition.notify_all()
        if listener:
            listener.close()
        for connection, _thread in clients:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        current = threading.current_thread()
        for thread in [accept_thread] + [thread for _, thread in clients]:
            if thread and thread is not current:
                thread.join(timeout=1.5)
        with self._condition:
            self._latest.clear()


class RelayClient:
    def __init__(self, host, port, on_packet, on_progress):
        self.host, self.port = host, int(port)
        self.on_packet, self.on_progress = on_packet, on_progress
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._socket = None

    def _progress(self, state, **extra):
        try:
            self.on_progress({"state": state, "host": self.host, "port": self.port, **extra})
        except Exception:
            pass  # UI teardown must not keep a network worker alive.

    def run(self):
        backoff = .25
        try:
            while not self._stop.is_set():
                self._progress("connecting")
                connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                connection.settimeout(.5)
                with self._lock:
                    if self._stop.is_set():
                        connection.close()
                        break
                    self._socket = connection
                try:
                    connection.connect((self.host, self.port))
                    if self._stop.is_set():
                        break
                    self._progress("connected")
                    backoff = .25
                    buffer = bytearray()
                    while not self._stop.is_set():
                        try:
                            chunk = connection.recv(65536)
                        except socket.timeout:
                            continue
                        if not chunk:
                            raise ConnectionError("relay closed")
                        buffer.extend(chunk)
                        while True:
                            newline = buffer.find(b"\n")
                            if newline < 0:
                                if len(buffer) > MAX_LINE_BYTES:
                                    raise ValueError("relay frame exceeds limit")
                                break
                            if newline > MAX_LINE_BYTES:
                                raise ValueError("relay frame exceeds limit")
                            line = bytes(buffer[:newline])
                            del buffer[:newline + 1]
                            self._deliver(line)
                except (OSError, ValueError) as error:
                    if not self._stop.is_set():
                        self._progress("disconnected", error=str(error)[:240])
                finally:
                    connection.close()
                    with self._lock:
                        if self._socket is connection:
                            self._socket = None
                if self._stop.wait(backoff):
                    break
                backoff = min(backoff * 2, 3.0)
        finally:
            self._progress("stopped")

    def _deliver(self, line):
        try:
            packet = json.loads(line)
            if not isinstance(packet, dict) or set(packet) != {"board", "event"}:
                raise ValueError("invalid relay envelope")
            board, event = packet["board"], packet["event"]
            validated = _validate(board, event)
            if validated is None or "_relay_age_s" not in event:
                raise ValueError("invalid relay event")
        except (ValueError, TypeError, RecursionError, UnicodeError) as error:
            self._progress("invalid", error=str(error)[:240])
            return
        # Callbacks always run on the caller's run() thread, never on Qt directly.
        try:
            self.on_packet(board, event)
        except Exception as error:
            self._progress("callback_error", error=str(error)[:240])

    def stop(self):
        self._stop.set()
        with self._lock:
            connection = self._socket
        if connection:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
