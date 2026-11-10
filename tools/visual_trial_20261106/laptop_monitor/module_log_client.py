"""Operator-only bounded SSH reader for the external C++ log tail helper.

The reader never starts or changes a vehicle module. No packet Qt signal is
emitted: the GUI timer drains at most 64 queued records, and spectators reuse
the operator's relay. Credentials only reach SSH's inherited environment.
"""
from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import selectors
import subprocess
import tempfile
import threading
import time
import uuid

from control import validate_host


MAX_LINE = 32768
MAX_EVENTS = 64
SOURCES = frozenset(("localization", "perception", "traversability", "planning"))
HELPERS = {
    "pi": '"$HOME/nomad_ws/tools/visual_trial_20261106/module_log_tail"',
    "jetson": '"$HOME/nomad_ws/tools/visual_trial_20261106/module_log_tail"',
    "pi4": '"$HOME/nomad_cpp_ws/tools/module_log_tail"',
}


class ModuleLogClient:
    def __init__(self, board, host, runtime_dir, parent=None):
        if board not in HELPERS:
            raise ValueError("Unknown module-log board")
        self.board, self.host = board, validate_host(host)
        self.runtime_dir = Path(runtime_dir)
        self._pending = deque(maxlen=MAX_EVENTS)
        self._lock = threading.Lock()
        self._process_lock = threading.Lock()
        self._process = None
        self._thread = None
        self._stop = threading.Event()
        self._error_session = "log-client-" + uuid.uuid4().hex
        self._error_seq = 0
        self.dropped_events = 0
        self.invalid_events = 0

    def start(self):
        """Start at most one reader thread; no remote mutation or ROS command."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="nomad-log-" + self.board,
                                        daemon=True)
        self._thread.start()

    def drain(self):
        """Return [(board,event)] with the original laptop arrival monotonic time."""
        with self._lock:
            events = list(self._pending)
            self._pending.clear()
        return [(self.board, event) for event in events]

    def stop(self):
        """Close/terminate only this reader's SSH session; never module processes."""
        self._stop.set()
        with self._process_lock:
            process = self._process
        if process is not None:
            self._end_process(process)
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=.8)
        return thread is None or not thread.is_alive()

    def _enqueue(self, event, arrived=None):
        value = dict(event)
        value["_local_received_monotonic"] = time.monotonic() if arrived is None else arrived
        with self._lock:
            if len(self._pending) == self._pending.maxlen:
                self.dropped_events += 1
            self._pending.append(value)

    def _accept_line(self, raw, arrived=None):
        """Validate before buffering, with bounded wire and decoded text sizes."""
        try:
            if len(raw) > MAX_LINE:
                raise ValueError("oversize")
            value = json.loads(raw)
            if not isinstance(value, dict) or value.get("kind") != "log" or value.get("board") != self.board:
                raise ValueError("envelope")
            source, stamp, sequence = value.get("source"), value.get("t"), value.get("seq")
            session, text = value.get("session"), value.get("text")
            if source not in SOURCES:
                raise ValueError("source")
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float))
                    or not math.isfinite(stamp) or stamp <= 0):
                raise ValueError("time")
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                raise ValueError("sequence")
            if not isinstance(session, str) or not 0 < len(session) <= 256:
                raise ValueError("session")
            if not isinstance(text, str) or len(text.encode("utf-8")) > 4096:
                raise ValueError("text")
            event = {"kind": "log", "board": self.board, "source": source,
                     "t": float(stamp), "seq": sequence, "session": session, "text": text}
        except (ValueError, TypeError, UnicodeError, OverflowError):
            self.invalid_events += 1
            return False
        self._enqueue(event, arrived)
        return True

    def _failure(self):
        self._error_seq += 1
        source = "planning" if self.board == "pi4" else "localization" if self.board == "pi" else "perception"
        self._enqueue({"kind": "log", "board": self.board, "source": source,
                       "t": time.time(), "seq": self._error_seq, "session": self._error_session,
                       "text": "로그 연결 실패 · 주소/접속/로그 도구 확인"})

    def _run(self):
        # One bounded retry permits a transient network interruption without an
        # unbounded reconnect loop or repeated password prompts.
        for attempt in range(2):
            if self._stop.is_set():
                return
            try:
                self._session()
            except (OSError, ValueError, subprocess.SubprocessError):
                pass
            if self._stop.is_set():
                return
            self._failure()
            if attempt == 0 and self._stop.wait(2.):
                return

    def _session(self):
        self.runtime_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".module-log-ssh-", dir=self.runtime_dir) as temporary:
            env = os.environ.copy()
            password = env.get("NOMAD_BOARD_PASSWORD1", "")
            if password:
                askpass = Path(temporary) / "askpass"
                askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_BOARD_PASSWORD1"\n', encoding="utf-8")
                askpass.chmod(0o700)
                env.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE="force")
                env.setdefault("DISPLAY", ":0")
            command = f"exec nice -n 15 {HELPERS[self.board]} --board {self.board}"
            args = ["ssh", "-T", "-o", "ConnectTimeout=5", "-o", "ConnectionAttempts=1",
                    "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=1",
                    "-o", "NumberOfPasswordPrompts=1", "-o", "StrictHostKeyChecking=accept-new",
                    "-o", 'UserKnownHostsFile="' + str(self.runtime_dir / "known_hosts") + '"',
                    "-o", "LogLevel=ERROR", "-o", "BatchMode=" + ("no" if password else "yes"),
                    self.host, command]
            process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.DEVNULL, env=env, bufsize=0,
                                       start_new_session=True)
            with self._process_lock:
                self._process = process
            try:
                if self._stop.is_set():
                    return
                self._read_output(process)
            finally:
                self._end_process(process)
                with self._process_lock:
                    if self._process is process:
                        self._process = None

    def _read_output(self, process):
        buffer = bytearray()
        discard_line = False
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            while not self._stop.is_set():
                if not selector.select(timeout=.2):
                    continue
                data = os.read(process.stdout.fileno(), 8192)
                if not data:
                    return
                arrived = time.monotonic()
                for chunk in data.splitlines(keepends=True):
                    ends = chunk.endswith(b"\n")
                    if not discard_line:
                        buffer.extend(chunk)
                        if len(buffer) > MAX_LINE:
                            buffer.clear()
                            discard_line = True
                            self.invalid_events += 1
                    if ends:
                        if not discard_line:
                            self._accept_line(bytes(buffer), arrived)
                        buffer.clear()
                        discard_line = False

    @staticmethod
    def _end_process(process):
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        for action, timeout in ((None, .15), ("terminate", .25), ("kill", .25)):
            try:
                if action is not None and process.poll() is None:
                    getattr(process, action)()
                process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
            except (OSError, ValueError):
                break
        if process.stdout is not None:
            try:
                process.stdout.close()
            except OSError:
                pass
