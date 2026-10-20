"""Opt-in metadata-only diagnostics. Disk IO never runs in ROS callbacks."""
import json
import os
import queue
import threading
import time
from pathlib import Path


class Diagnostics:
    def __init__(self, directory, capacity=8192, prefix="map"):
        self.path = None
        self.dropped = 0
        self.error = None
        self.closed = False
        if not directory:
            return
        folder = Path(directory).expanduser()
        folder.mkdir(parents=True, exist_ok=True)
        self.path = folder / f"{prefix}_{time.time_ns()}_{os.getpid()}.jsonl"
        self.file = self.path.open("x", buffering=65536)
        self.queue = queue.Queue(maxsize=capacity)
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def emit(self, event, **fields):
        if self.path is None or self.closed or self.error:
            return
        record = dict(event=event, wall_ns=time.time_ns(), mono_ns=time.monotonic_ns(), **fields)
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        last_flush = time.monotonic()
        try:
            while not self.closed or not self.queue.empty():
                try:
                    record = self.queue.get(timeout=.2)
                except queue.Empty:
                    record = None
                if record is not None:
                    self.file.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                if time.monotonic() - last_flush >= 1.:
                    self.file.flush()
                    last_flush = time.monotonic()
            self.file.write(json.dumps(dict(event="diagnostics_closed", dropped=self.dropped)) + "\n")
        except Exception as error:
            self.error = str(error)
        finally:
            self.file.close()

    def close(self):
        self.closed = True
        if self.path is not None:
            self.thread.join(timeout=2.)
