"""Bounded laptop log display. The operator supplies existing stdout/stderr.

No SSH, process, file, or ROS access lives here. A visible detail window is fed
from the monitor's shared buffer, including for spectator PCs.
"""
from __future__ import annotations

import re
import time
from collections import deque
from collections.abc import Mapping
from itertools import islice

from PyQt5 import QtCore, QtGui, QtWidgets
from render_utils import stable_metadata


_ANSI = re.compile(r"\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1b\\))")


class LogHistory:
    """Bounded operator/spectator memory with an incremental display cursor.

    The monitor appends already validated event dictionaries once. Each detail
    window independently consumes only events added after its local cursor.
    No history traversal occurs when no new record arrived.
    """

    def __init__(self, maxlen=128):
        self.maxlen = max(1, int(maxlen))
        self._events = deque(maxlen=self.maxlen)
        self._cursor = 0

    def append(self, event):
        self._events.append(event)
        self._cursor += 1

    def __len__(self):
        return len(self._events)

    def __iter__(self):
        return iter(self._events)

    def __getitem__(self, index):
        if isinstance(index, slice):
            return list(self._events)[index]
        return self._events[index]

    def events_after(self, cursor=None):
        latest = self._cursor
        if isinstance(cursor, int) and not isinstance(cursor, bool):
            count = latest - cursor
            if count <= 0:
                return latest, []
        else:
            count = len(self._events)
        if count >= len(self._events):
            return latest, list(self._events)
        events = list(islice(reversed(self._events), count))
        events.reverse()
        return latest, events


class ModuleLogPanel(QtWidgets.QWidget):
    """Render bounded plain text; optional sequence IDs make replay idempotent."""

    def __init__(self, parent=None, max_lines=800):
        super().__init__(parent)
        self.max_lines = max(1, min(1000, int(max_lines)))
        self._lines = deque(maxlen=self.max_lines)
        self._sequences = {}
        self._ids = deque()
        self._id_set = set()
        self._batch_rows = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(7, 3, 7, 5)
        layout.setSpacing(4)
        self.state = QtWidgets.QLabel("실행 로그 대기 · 기존 stdout/stderr")
        self.state.setWordWrap(True)
        self.state.setStyleSheet("font-size:11px;color:#94a3b8;")
        stable_metadata(self.state)
        layout.addWidget(self.state)
        controls = QtWidgets.QHBoxLayout()
        self.filter = QtWidgets.QLineEdit()
        self.filter.setPlaceholderText("로그 검색")
        self.filter.textChanged.connect(self._rebuild)
        controls.addWidget(self.filter, 1)
        clear = QtWidgets.QPushButton("비우기")
        clear.setToolTip("이 창의 표시만 비웁니다. 보드의 로그 파일과 모듈은 유지됩니다.")
        clear.clicked.connect(self.clear)
        controls.addWidget(clear)
        layout.addLayout(controls)
        options = QtWidgets.QHBoxLayout()
        self.auto_scroll = QtWidgets.QCheckBox("자동 스크롤")
        self.auto_scroll.setChecked(True)
        self.auto_scroll.setToolTip("맨 아래에서만 새 로그를 따라갑니다. 위로 스크롤하면 읽는 위치를 유지합니다.")
        options.addWidget(self.auto_scroll)
        self.paused = QtWidgets.QCheckBox("표시 일시정지")
        self.paused.toggled.connect(self._pause_changed)
        options.addWidget(self.paused)
        options.addStretch(1)
        layout.addLayout(options)
        self.text = QtWidgets.QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(self.max_lines)
        self.text.setLineWrapMode(QtWidgets.QPlainTextEdit.WidgetWidth)
        self.text.setWordWrapMode(QtGui.QTextOption.WrapAnywhere)
        self.text.setPlaceholderText("이 모듈의 기존 실행 로그를 기다립니다.")
        self.text.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.text.setMinimumHeight(60)
        self.text.setStyleSheet("QPlainTextEdit {background:#0b1220;color:#cbd5e1;border:1px solid #334155;font-size:12px;}")
        layout.addWidget(self.text, 1)

    def append_event(self, event):
        """Accept {board, source, text|lines, seq?, run_id?, id?}; return rows added.

        Events must be routed to the appropriate window by the monitor. A sequence
        resets with run_id/session, so a module restart need not reuse stale text.
        Rendering never interprets HTML or terminal control sequences.
        """
        if not isinstance(event, Mapping):
            return 0
        board = self._label(event.get("board", "보드"))
        source = self._label(event.get("source", event.get("module", "모듈")))
        session = str(event.get("run_id", event.get("session", "")))[:256]
        key = (board, source, session)
        sequence = event.get("seq")
        if isinstance(sequence, int) and not isinstance(sequence, bool):
            if sequence <= self._sequences.get(key, -1):
                return 0
        identifier = event.get("id", event.get("cursor"))
        id_key = (key, str(identifier)[:512]) if identifier is not None else None
        if id_key is not None and id_key in self._id_set:
            return 0
        raw = event.get("text")
        if not isinstance(raw, str):
            lines = event.get("lines")
            if not isinstance(lines, (tuple, list)):
                return 0
            # Bound work even if an external producer accidentally sends a huge
            # record. Keep recent lines, just as the screen's ring buffer does.
            raw = "\n".join(value[:4096] for value in lines[-200:] if isinstance(value, str))
        raw = _ANSI.sub("", raw[-65536:]).replace("\r\n", "\n").replace("\r", "\n")
        raw = "".join(char for char in raw if char in "\n\t" or ord(char) >= 32)
        rows = raw.splitlines()[-200:]
        if not rows:
            return 0
        if isinstance(sequence, int) and not isinstance(sequence, bool):
            self._sequences[key] = sequence
            # Producer sessions are bounded too, independent of QTextDocument.
            if len(self._sequences) > 64:
                self._sequences.pop(next(iter(self._sequences)))
        if id_key is not None:
            self._ids.append(id_key)
            self._id_set.add(id_key)
            while len(self._ids) > 2048:
                self._id_set.discard(self._ids.popleft())
        prefix = f"[{board}/{source}] "
        rows = [prefix + row[:4096] for row in rows]
        self._lines.extend(rows)
        if not self.paused.isChecked():
            selected = self._matching(rows)
            if selected:
                if self._batch_rows is not None:
                    self._batch_rows.extend(selected)
                    if len(self._batch_rows) > self.max_lines:
                        del self._batch_rows[:-self.max_lines]
                else:
                    self._append_rows(selected)
        if self._batch_rows is None:
            self._state()
        return len(rows)

    def _append_rows(self, rows):
        scroll = self.text.verticalScrollBar()
        previous = scroll.value()
        follow = self.auto_scroll.isChecked() and previous == scroll.maximum()
        removed = max(0, self.text.document().blockCount() + len(rows) - self.max_lines)
        removed_lines = 0
        if removed and not follow:
            block = self.text.document().firstBlock()
            for _ in range(removed):
                if not block.isValid():
                    break
                removed_lines += max(1, block.layout().lineCount())
                block = block.next()
        self.text.appendPlainText("\n".join(rows))
        scroll.setValue(scroll.maximum() if follow else max(0, previous-removed_lines))

    def begin_batch(self):
        self._batch_rows = []

    def end_batch(self):
        rows, self._batch_rows = self._batch_rows, None
        if rows:
            self._append_rows(rows)
        self._state()

    @staticmethod
    def _label(value):
        return " ".join(str(value).split())[:80]

    def _matching(self, rows):
        query = self.filter.text().casefold()
        return [row for row in rows if query in row.casefold()]

    def _state(self):
        mode = "표시 일시정지 · " if self.paused.isChecked() else ""
        self.state.setText(f"{mode}최근 {len(self._lines)}/{self.max_lines}행 · 표시 최대 1Hz")

    def _rebuild(self, *_):
        if self.paused.isChecked():
            return
        scroll = self.text.verticalScrollBar()
        previous = scroll.value()
        self.text.setPlainText("\n".join(self._matching(self._lines)))
        scroll.setValue(scroll.maximum() if self.auto_scroll.isChecked() else previous)

    def _pause_changed(self, paused):
        if not paused:
            self._rebuild()
        self._state()

    def clear(self):
        self._lines.clear()
        self.text.clear()
        # Keep cursors so the next shared-buffer replay cannot resurrect cleared
        # records. No board file is truncated and no module command is sent.
        self._state()


def attach_module_logs(window):
    """Keep maps/images left and add a horizontally resizable log column right."""
    panel = ModuleLogPanel(window)
    dock = QtWidgets.QDockWidget("실행 로그", window)
    dock.setObjectName("module_execution_logs")
    dock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
    dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
    dock.setWidget(panel)
    dock.setMinimumWidth(260)
    window.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
    window.resizeDocks([dock], [400], QtCore.Qt.Horizontal)
    window.log_panel, window.log_dock = panel, dock
    return panel


def update_module_logs(window, snapshot, source):
    """Append new laptop-memory records in batches at most once per second.

    The owning monitor already refreshes visible detail windows only. There is no
    separate timer here, and opening another window cannot create another reader.
    """
    groups = snapshot.get("module_logs", {})
    if not isinstance(groups, Mapping):
        return
    history = groups.get(source)
    if history is None:
        return
    cursors = getattr(window, "_module_log_cursors", None)
    if cursors is None:
        cursors = window._module_log_cursors = {}
    previous, cursor = cursors.get(source, (None, None))
    flushes = getattr(window, "_module_log_flush_times", None)
    if flushes is None:
        flushes = window._module_log_flush_times = {}
    if previous is not history:
        cursor = None
        flushes.pop(source, None)
    replay = bool(snapshot.get("replay"))
    now = snapshot.get("now", 0.) if replay else time.monotonic()
    previous_flush = flushes.get(source)
    if not replay and previous_flush is not None and now-previous_flush < 1.:
        # Leave the cursor unchanged. Received records remain in LogHistory
        # until the next display batch, subject only to its bounded retention.
        return
    if isinstance(history, LogHistory):
        latest, events = history.events_after(cursor)
    else:
        # Compatibility for small list/deque test fixtures. Live monitor storage
        # uses LogHistory, so its unchanged path never scans buffered records.
        latest = len(history)
        start = cursor if isinstance(cursor, int) and 0 <= cursor <= latest else 0
        events = () if start == latest else islice(history, start, latest)
    consumed = False
    panel = getattr(window, "log_panel", None)
    batching = panel is not None and bool(events)
    if batching:
        panel.begin_batch()
    try:
        for event in events:
            consumed = True
            window.append_module_log(event)
    finally:
        if batching:
            panel.end_batch()
    cursors[source] = (history, latest)
    if consumed:
        flushes[source] = now
