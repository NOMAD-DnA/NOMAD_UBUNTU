"""Portable GUI event recording; no Qt, ROS, SSH, or vehicle commands.

A .nomad file is SQLite containing the original received JSON events.  Receipt
offsets determine playback order; timestamps inside events are never rewritten.
Only the writer thread accesses the recording database.  A failed recorder never
silently drops events and pretends the recording is complete.
"""
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sqlite3
import threading


FORMAT_VERSION = 1
APPLICATION_ID = 0x4E4F4D44  # NOMD
MAX_QUEUE_BYTES = 64 * 1024 * 1024
MAX_EVENT_BYTES = 32 * 1024 * 1024
READ_CHUNK_SIZE = 512
READ_CHUNK_BYTES = 16 * 1024 * 1024
CLOSE_TIMEOUT_S = 5.0


def recording_directory():
    """Use the shared trial folder, retaining explicit XDG and portable installs."""
    if "XDG_STATE_HOME" in os.environ:
        return (Path(os.environ["XDG_STATE_HOME"]).expanduser() /
                "nomad-trial" / "recordings")
    integration_root = Path(__file__).resolve().parent.parent.parent
    if integration_root.name == "NOMAD_모듈별_통합_20261104":
        return integration_root / "주행기록"
    return Path.home() / ".local" / "state" / "nomad-trial" / "recordings"


class RecordingError(ValueError):
    """The selected file is not a supported, readable NOMAD recording."""


def _json_bytes(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"),
                      allow_nan=False).encode("utf-8")


def _load_json(value):
    def reject_constant(constant):
        raise RecordingError("Non-finite JSON value: " + constant)
    return json.loads(value, parse_constant=reject_constant)


def _nonnegative_time(value):
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise ValueError("receipt offset must be finite and nonnegative")
    return result


class SessionRecorder:
    """Nonblocking producer and bounded background SQLite writer.

    ``events`` counts committed events; ``accepted_events`` additionally includes
    queued events.  ``queue_bytes`` includes the writer's uncommitted batch.
    On overflow recording stops, accepted data is flushed, and state remains
    FAILED.  Call close() when ending a session, even after an error.
    """

    def __init__(self, path, metadata=None, *, max_queue_bytes=MAX_QUEUE_BYTES,
                 close_timeout_s=CLOSE_TIMEOUT_S):
        self.path = str(Path(path).expanduser().absolute())
        self._metadata = _json_bytes(dict(metadata or {}))
        self._max_queue_bytes = int(max_queue_bytes)
        self._close_timeout_s = float(close_timeout_s)
        if self._max_queue_bytes < 1:
            raise ValueError("max_queue_bytes must be positive")
        if not math.isfinite(self._close_timeout_s) or self._close_timeout_s <= 0:
            raise ValueError("close_timeout_s must be finite and positive")
        self._condition = threading.Condition()
        self._queue = deque()
        self._queue_bytes = 0
        self._accepted_events = 0
        self._events = 0
        self._last_offset = 0.0
        self._duration = 0.0
        self._requested_duration = 0.0
        self._state = "RECORDING"
        self._error = ""
        self._close_requested = False
        self._created_utc = datetime.now(timezone.utc).isoformat()
        # Never replace another trial, even if a name is accidentally reused.
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        os.close(descriptor)
        self._thread = threading.Thread(target=self._run,
                                        name="nomad-session-writer", daemon=True)
        self._thread.start()

    def _fail_locked(self, message):
        if not self._error:
            self._error = str(message)
        self._state = "FAILED"
        self._close_requested = True
        self._condition.notify_all()

    def append(self, board, event, offset_s):
        """Copy one original event without waiting for disk; False is an error.

        Offset equality is allowed; ties retain acceptance sequence.  Clock
        jumps within sensor ``t``/header fields do not affect receipt ordering.
        """
        with self._condition:
            if self._state != "RECORDING":
                return False
        try:
            offset_s = _nonnegative_time(offset_s)
            if not isinstance(board, str) or not board:
                raise ValueError("board must be a nonempty string")
            if not isinstance(event, dict):
                raise ValueError("event must be a JSON object")
            payload = _json_bytes(event)
            if len(payload) > MAX_EVENT_BYTES:
                raise ValueError("event exceeds recording size limit")
            size = len(payload) + len(board.encode("utf-8")) + 64
        except (TypeError, ValueError, OverflowError, RecursionError) as exc:
            with self._condition:
                self._fail_locked("Event could not be recorded: " + str(exc))
            return False
        with self._condition:
            if self._state != "RECORDING":
                return False
            if offset_s < self._last_offset:
                self._fail_locked("Receipt offsets went backwards; recording stopped")
                return False
            if self._queue_bytes + size > self._max_queue_bytes:
                self._fail_locked("Recording queue limit exceeded; recording stopped. "
                                  "The event that exceeded the limit was not saved")
                return False
            self._accepted_events += 1
            self._queue.append((self._accepted_events, offset_s, board, payload, size))
            self._queue_bytes += size
            self._last_offset = offset_s
            self._condition.notify()
            return True

    def status(self):
        with self._condition:
            return {"state": self._state, "error": self._error,
                    "events": self._events, "accepted_events": self._accepted_events,
                    "path": self.path, "queue_bytes": self._queue_bytes,
                    "duration": self._duration,
                    "duration_s": max(self._duration, self._requested_duration),
                    "complete": self._state == "CLOSED" and not self._error}

    def close(self, duration_s=None):
        """Flush accepted events, with a bounded wait and explicit failure."""
        with self._condition:
            if self._state == "CLOSED":
                return self.status()
            if duration_s is not None:
                try:
                    self._requested_duration = max(
                        self._requested_duration, self._last_offset,
                        _nonnegative_time(duration_s))
                except (TypeError, ValueError, OverflowError) as exc:
                    self._fail_locked("Invalid recording end time: " + str(exc))
            self._close_requested = True
            if self._state == "RECORDING":
                self._state = "CLOSING"
            self._condition.notify_all()
        self._thread.join(self._close_timeout_s)
        if self._thread.is_alive():
            with self._condition:
                self._fail_locked("Recording close timed out; file may be incomplete")
        return self.status()

    def _connect(self):
        return sqlite3.connect(self.path, timeout=1.0)

    def _recording_info(self, complete=False):
        with self._condition:
            return {"format_version": FORMAT_VERSION, "created_utc": self._created_utc,
                    "complete": bool(complete), "error": self._error,
                    "events": self._events, "accepted_events": self._accepted_events,
                    "duration": self._duration,
                    "duration_s": max(self._duration, self._requested_duration),
                    "time_basis": "monotonic receipt offset; original event times retained"}

    def _run(self):
        connection = None
        try:
            connection = self._connect()
            connection.execute("PRAGMA application_id=%d" % APPLICATION_ID)
            connection.execute("PRAGMA user_version=%d" % FORMAT_VERSION)
            connection.execute("PRAGMA journal_mode=DELETE")
            connection.execute("PRAGMA synchronous=FULL")
            connection.execute("CREATE TABLE metadata (key TEXT PRIMARY KEY, value BLOB NOT NULL)")
            connection.execute("CREATE TABLE events (seq INTEGER PRIMARY KEY, "
                               "offset_s REAL NOT NULL CHECK(offset_s>=0), "
                               "board TEXT NOT NULL, event_json BLOB NOT NULL)")
            connection.execute("CREATE INDEX events_time ON events(offset_s,seq)")
            connection.executemany("INSERT INTO metadata VALUES (?,?)", [
                ("user", self._metadata),
                ("recording", _json_bytes(self._recording_info()))])
            connection.commit()
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._queue or self._close_requested)
                    if not self._queue:
                        break
                    batch = []
                    while self._queue and len(batch) < 128:
                        batch.append(self._queue.popleft())
                connection.executemany("INSERT INTO events VALUES (?,?,?,?)",
                                       [row[:4] for row in batch])
                connection.commit()
                with self._condition:
                    self._events += len(batch)
                    self._duration = batch[-1][1]
                    self._queue_bytes -= sum(row[4] for row in batch)
            with self._condition:
                complete = not self._error and self._events == self._accepted_events
            connection.execute("UPDATE metadata SET value=? WHERE key='recording'",
                               (_json_bytes(self._recording_info(complete)),))
            connection.commit()
            with self._condition:
                if complete and not self._error:
                    self._state = "CLOSED"
        except Exception as exc:
            with self._condition:
                self._fail_locked("Recording write failed: " + str(exc))
                self._queue.clear()
                self._queue_bytes = 0
            if connection is not None:
                try:
                    connection.rollback()
                    connection.execute("UPDATE metadata SET value=? WHERE key='recording'",
                                       (_json_bytes(self._recording_info(False)),))
                    connection.commit()
                except sqlite3.Error:
                    pass  # Original complete=false remains; caller sees FAILED.
        finally:
            if connection is not None:
                connection.close()


class SessionReader:
    """Read-only, indexed playback with bounded event batches.

    Incomplete sessions are readable after a crash; metadata['_recording'] shows
    complete=False.  Unsupported or malformed files raise RecordingError.
    """

    def __init__(self, path):
        self.path = str(Path(path).expanduser().absolute())
        self._connection = None
        try:
            self._connection = sqlite3.connect(Path(self.path).as_uri() + "?mode=ro",
                                              uri=True, timeout=1.0)
            self._connection.execute("PRAGMA query_only=ON")
            if self._connection.execute("PRAGMA application_id").fetchone()[0] != APPLICATION_ID:
                raise RecordingError("Not a NOMAD recording")
            if self._connection.execute("PRAGMA user_version").fetchone()[0] != FORMAT_VERSION:
                raise RecordingError("Unsupported NOMAD recording version")
            rows = dict(self._connection.execute("SELECT key,value FROM metadata"))
            self.metadata = _load_json(rows["user"])
            info = _load_json(rows["recording"])
            if not isinstance(self.metadata, dict) or not isinstance(info, dict):
                raise RecordingError("Invalid recording metadata")
            if info.get("format_version") != FORMAT_VERSION:
                raise RecordingError("Unsupported recording metadata version")
            self.metadata["_recording"] = info
            self.event_count, duration, first, last = self._connection.execute(
                "SELECT COUNT(*),MAX(offset_s),MIN(seq),MAX(seq) FROM events").fetchone()
            self.duration = max(float(duration or 0.0),
                                _nonnegative_time(info.get("duration_s", 0.0)))
            if self.event_count and (first != 1 or last != self.event_count):
                raise RecordingError("Recording event sequence is incomplete")
            if not math.isfinite(self.duration) or self.duration < 0:
                raise RecordingError("Invalid recording duration")
            bad = self._connection.execute(
                "SELECT 1 FROM events WHERE offset_s < 0 OR board='' OR "
                "typeof(offset_s) NOT IN ('real','integer') LIMIT 1").fetchone()
            backwards = self._connection.execute(
                "SELECT 1 FROM events a JOIN events b ON b.seq=a.seq+1 "
                "WHERE b.offset_s<a.offset_s LIMIT 1").fetchone()
            if bad or backwards:
                raise RecordingError("Invalid recording event order")
            if info.get("complete") and (info.get("events") != self.event_count or
                                          info.get("accepted_events") != self.event_count):
                raise RecordingError("Completed recording has missing events")
            if not info.get("complete"):
                info["recovered_events"] = self.event_count
        except (sqlite3.Error, OSError, KeyError, TypeError, ValueError, RecursionError) as exc:
            self.close()
            if isinstance(exc, RecordingError):
                raise
            raise RecordingError("Cannot open recording: " + str(exc)) from exc

    def events_until(self, time_s, after_seq=0, limit=READ_CHUNK_SIZE):
        """Return at most limit events in sequence order, inclusive of time_s."""
        if self._connection is None:
            raise RecordingError("Recording is closed")
        time_s = _nonnegative_time(time_s)
        after_seq, limit = int(after_seq), int(limit)
        if after_seq < 0 or limit < 1 or limit > 10000:
            raise ValueError("Invalid event cursor or batch size")
        try:
            rows = self._connection.execute(
                "SELECT seq,offset_s,board,event_json FROM events "
                "WHERE seq>? AND offset_s<=? ORDER BY seq LIMIT ?",
                (after_seq, time_s, limit))
            result = []
            read_bytes = 0
            for seq, offset, board, payload in rows:
                if result and read_bytes + len(payload) > READ_CHUNK_BYTES:
                    break
                event = _load_json(payload)
                if not isinstance(event, dict):
                    raise RecordingError("Recorded event is not an object at seq %d" % seq)
                result.append({"seq": seq, "offset_s": offset, "board": board, "event": event})
                read_bytes += len(payload)
            return result
        except (sqlite3.Error, ValueError, TypeError, UnicodeError, RecursionError) as exc:
            if isinstance(exc, RecordingError):
                raise
            raise RecordingError("Cannot read recording events: " + str(exc)) from exc

    def iter_until(self, time_s, chunk_size=READ_CHUNK_SIZE):
        """Stream from the beginning for deterministic state rebuild on seek."""
        cursor = 0
        while True:
            batch = self.events_until(time_s, cursor, chunk_size)
            if not batch:
                return
            for item in batch:
                yield item
            cursor = batch[-1]["seq"]

    def close(self):
        if self._connection is not None:
            self._connection.close()
            self._connection = None
