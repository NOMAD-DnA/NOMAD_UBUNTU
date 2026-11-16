"""Shared read-only debugging aids for the developer windows.

Everything here reads the monitor's existing snapshot only. There is no SSH,
ROS, board process, socket or timer. A saved snapshot goes to the laptop GUI
state folder (never a board), and copying only touches the local clipboard.
"""
from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from datetime import datetime
import json
import math
from pathlib import Path
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets


WINDOW_S = 60.
SCHEMA = "nomad_developer_debug_v1"
LEVELS = {
    # name, background, foreground
    "ok": ("정상", "#064e3b", "#a7f3d0"),
    "warn": ("주의", "#78350f", "#fde68a"),
    "fail": ("실패", "#7f1d1d", "#fecaca"),
    "unknown": ("미확인", "#1e293b", "#cbd5e1"),
}
SEVERITY = {"ok": 0, "unknown": 1, "warn": 2, "fail": 3}
PANEL_STYLE = """
QGroupBox { border:1px solid #334155; border-radius:5px; margin-top:12px;
            padding:12px 7px 7px; font-weight:600; }
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; }
QTableWidget { background:#0b1220; alternate-background-color:#111a2c; color:#e2e8f0;
               gridline-color:#273449; border:1px solid #334155; font-size:12px; }
QHeaderView::section { background:#1e293b; color:#cbd5e1; border:0; padding:4px 6px;
                       font-size:12px; font-weight:600; }
"""
TOOL_STYLE = ("QPushButton {background:#1e293b;border:1px solid #475569;padding:4px 9px;font-size:12px;"
              "border-radius:4px;color:#e2e8f0;} QPushButton:hover {background:#334155;}"
              "QPushButton:checked {background:#92400e;border-color:#f59e0b;}"
              "QPushButton:disabled {color:#64748b;}")

# Process groups reported by the existing board resource sampler.
GROUP_LABELS = {"planning_gpp_lpp": "플래닝 GPP+LPP", "control": "제어", "vesc_driver": "VESC 드라이버",
                "mapping": "지도", "semantic": "의미분할", "stereo_cloud": "점군 생성", "camera": "카메라",
                "lidar": "라이다", "slope": "경사 계산", "measurement": "계측/전달",
                "other_nomad": "기타 NOMAD", "other": "기타"}
BOARD_LABELS = {"jetson": "젯슨", "pi4": "라파4", "pi": "Pi5"}


# --------------------------------------------------------------------------- values

def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def age(snapshot, key):
    """Seconds since the monitor received `key`, or None when never received."""
    if not isinstance(snapshot, Mapping):
        return None
    received = snapshot.get("received", {})
    received = received.get(key) if isinstance(received, Mapping) else None
    now = snapshot.get("now")
    if not finite(now):
        now = time.monotonic()
    if not finite(received):
        return None
    return max(0., now - received)


def reported_rate(snapshot, board, name):
    rates = snapshot.get("rates", {}) if isinstance(snapshot, Mapping) else {}
    values = rates.get(board, {}) if isinstance(rates, Mapping) else {}
    value = values.get(name) if isinstance(values, Mapping) else None
    return float(value) if finite(value) and value >= 0 else None


def age_text(value):
    return "미수신" if value is None else f"{value:.1f}초"


def number_text(value, digits=1, unit=""):
    return f"{value:.{digits}f}{unit}" if finite(value) else "—"


def plain(value, limit=300):
    if value is None or value == "":
        return "—"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.4g}" if finite(value) else "—"
    return " ".join(str(value).split())[:limit]


def wall_clock(stamp):
    if not finite(stamp) or stamp <= 0:
        return "—"
    try:
        return datetime.fromtimestamp(stamp).strftime("%H:%M:%S")
    except (OverflowError, OSError, ValueError):
        return "—"


def freshness(name, value, warn_s, fail_s, missing="fail", note=""):
    """One checklist row for a receive age. The exact numbers stay visible."""
    suffix = (" · " + note) if note else ""
    if value is None:
        return (name, missing, "미수신" + suffix)
    level = "ok" if value <= warn_s else "warn" if value <= fail_s else "fail"
    words = {"ok": "최근 수신", "warn": "늦음", "fail": "끊김"}[level]
    return (name, level, f"{words} · {value:.1f}초 전 (기준 {warn_s:g}초){suffix}")


def rate_check(name, value):
    if value is None:
        return (name, "unknown", "보드 보고 없음")
    if value <= 0.:
        return (name, "fail", "0 Hz · 보드에서 이 토픽 수신 없음")
    return (name, "ok", f"{value:.1f} Hz (보드 집계)")


def status_check(name, status, status_age, stale_s=3.):
    if not isinstance(status, Mapping) or not status:
        return (name, "fail", "상태 미수신")
    state = str(status.get("state", "UNKNOWN"))
    detail = plain(status.get("detail"), 400)
    text = f"{state} · {detail}" if detail != "—" else state
    if status_age is not None and status_age > stale_s:
        return (name, "warn", f"오래됨 {status_age:.1f}초 · 마지막 {text}")
    level = {"READY": "ok", "WAITING": "warn", "STOPPED": "warn", "FAULT": "fail"}.get(state.upper(), "unknown")
    return (name, level, text)


def log_check(panel, name="최근 오류 로그"):
    summary = panel.problem_summary() if panel is not None and hasattr(panel, "problem_summary") else None
    if summary is None:
        return (name, "unknown", "실행 로그 없음")
    errors, warnings, last = summary["errors"], summary["warnings"], summary["last"]
    if errors:
        return (name, "warn", f"오류 {errors} · 경고 {warnings} · 마지막: {last}")
    if warnings:
        return (name, "ok", f"오류 0 · 경고 {warnings} · 마지막: {last}")
    return (name, "ok", "오류·경고 줄 없음 (이 창에 받은 로그 기준)")


def flat_scalars(event, limit=80):
    """Every scalar field of a received telemetry dict, nested keys joined by '.'."""
    rows = []

    def walk(prefix, value, depth):
        if len(rows) >= limit:
            return
        if isinstance(value, Mapping) and depth < 3:
            for key in sorted(value, key=str):
                if str(key).startswith("_") or key in ("kind",):
                    continue
                walk(f"{prefix}.{key}" if prefix else str(key), value[key], depth + 1)
        elif isinstance(value, (list, tuple)):
            if value and all(isinstance(v, Mapping) and isinstance(v.get("key"), str) for v in value):
                for item in value:
                    walk(f"{prefix}.{item['key']}" if prefix else item["key"], item.get("value"), depth + 1)
            elif len(value) <= 12 and all(finite(v) or isinstance(v, str) for v in value):
                rows.append((prefix, "[" + ", ".join(plain(v, 40) for v in value) + "]"))
            else:
                rows.append((prefix, f"목록 {len(value)}개"))
        else:
            rows.append((prefix, plain(value)))

    if isinstance(event, Mapping):
        walk("", event, 0)
    return rows


def path_length(points):
    total = 0.
    for a, b in zip(points, points[1:]):
        if len(a) >= 2 and len(b) >= 2 and all(finite(v) for v in (*a[:2], *b[:2])):
            total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def stats(values):
    values = [v for v in values if finite(v)]
    if not values:
        return None
    return min(values), sum(values) / len(values), max(values)


def stats_text(values, digits=1, unit=""):
    result = stats(values)
    if result is None:
        return "—"
    return " / ".join(f"{v:.{digits}f}" for v in result) + unit


# ------------------------------------------------------------------ board processes

def module_processes(snapshot, groups):
    """Rows of already received board resource telemetry for these process groups."""
    telemetry = snapshot.get("telemetry", {}) if isinstance(snapshot, Mapping) else {}
    history = snapshot.get("resource_history", {}) if isinstance(snapshot, Mapping) else {}
    now = snapshot.get("now") if isinstance(snapshot, Mapping) else None
    rows = []
    for source, board in (("resources_jetson", "jetson"), ("resources_pi4", "pi4")):
        event = telemetry.get(source, {}) if isinstance(telemetry, Mapping) else {}
        if not isinstance(event, Mapping) or not event:
            continue
        received_age = age(snapshot, "telemetry:" + source)
        live = received_age is not None and received_age <= 3. and event.get("cpu", {}).get("state") == "live"
        samples = [r for r in history.get(board, ()) if finite(now) and isinstance(r, Mapping)
                   and finite(r.get("received")) and 0 <= now - r["received"] <= WINDOW_S] if isinstance(history, Mapping) else []
        for process in event.get("processes", ()):
            if not isinstance(process, Mapping) or process.get("group") not in groups:
                continue
            cpu = process.get("cpu_percent_one_core")
            group = process.get("group")
            spread = stats_text([r.get("groups", {}).get(group) for r in samples], 1, "%")
            rows.append([GROUP_LABELS.get(group, str(group)) + " · " + BOARD_LABELS.get(board, board),
                         plain(process.get("pid")) + " " + plain(process.get("name"), 60),
                         (f"{cpu:.1f}%" if finite(cpu) else "준비 중") + ("" if live else " (지연)"),
                         spread, number_text(process.get("rss_mib"), 1), plain(process.get("threads"))])
    return rows


PROCESS_HEADERS = ["작업 · 보드", "PID 프로세스", "CPU (한 코어=100%)", "60초 그룹 CPU", "RSS MiB", "스레드"]
PROCESS_NOTE = ("보드 부하 창과 같은 기존 계측값입니다. 60초 그룹 CPU = 같은 작업 그룹 합계의 최근 60초 최소/평균/최대. "
                "계측 도구가 없거나 해당 프로세스가 없으면 비어 있습니다.")


# ------------------------------------------------------------------ image analysis

def image_rgb(image):
    """Read-only RGB array copy of a QImage (no file or decode work)."""
    if not isinstance(image, QtGui.QImage) or image.isNull():
        return None
    rgb = image.convertToFormat(QtGui.QImage.Format_RGB888)
    bits = rgb.bits()
    bits.setsize(rgb.byteCount())
    array = np.frombuffer(bits, dtype=np.uint8).reshape(rgb.height(), rgb.bytesPerLine())
    return array[:, :rgb.width() * 3].reshape(rgb.height(), rgb.width(), 3).copy()


def heat_values(pixels):
    """Invert the board probe's green-yellow-red heat colour to 0..1, NaN = gray unknown.

    The probe paints unknown cells exactly (100,100,100) and known values as
    (R,G,B) = (255*min(1,2v), 255*min(1,2(1-v)), 0) in a lossless PNG.
    """
    rgb = pixels.astype(np.int16)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    value = np.where(r < 255, r / 510., 1. - g / 510.)
    heat = (b <= 8) & ((r >= 247) | (g >= 247))
    return np.where(heat, value, np.nan)


def bucket_counts(values, edges):
    """values: 0..1 array or NaN. edges: [(label, low, high_inclusive)] in 0..1."""
    total = values.size
    unknown = int(np.count_nonzero(np.isnan(values)))
    rows = [("미확정 (회색)", unknown)]
    known = values[~np.isnan(values)]
    for label, low, high in edges:
        rows.append((label, int(np.count_nonzero((known >= low) & (known <= high)))))
    return total, rows


def palette_counts(pixels, palette, step=2, tolerance=70.):
    """Nearest palette colour per sampled pixel; far colours are 'unclassified'."""
    sample = pixels[::step, ::step].reshape(-1, 3).astype(np.float32)
    colors = np.asarray(palette, dtype=np.float32)
    distance = ((sample[:, None, :] - colors[None, :, :]) ** 2).sum(axis=2)
    nearest = distance.argmin(axis=1)
    close = distance[np.arange(len(sample)), nearest] <= tolerance ** 2
    counts = np.bincount(nearest[close], minlength=len(colors))
    return len(sample), counts.tolist(), int(len(sample) - close.sum())


# ------------------------------------------------------------------- history helpers

class StreamStats:
    """Last-60 s receive age and board-reported Hz, sampled at each UI update."""

    def __init__(self, specs, window_s=WINDOW_S):
        # spec: dict(key=received key, label, limit, rate=(board, name) | None,
        #            age=callable(snapshot) | None)
        self.specs = list(specs)
        self.window_s = window_s
        self.clear()

    def clear(self):
        self.samples = {spec["key"]: deque() for spec in self.specs}
        self.drops = {spec["key"]: deque() for spec in self.specs}
        self._stale = {}
        self._last_now = None

    def observe(self, snapshot):
        now = snapshot.get("now") if isinstance(snapshot, Mapping) else None
        if not finite(now):
            return False
        if self._last_now is not None and now < self._last_now - .5:
            self.clear()  # replay moved back in time: drop future samples
        if self._last_now is not None and now - self._last_now < .05:
            return False
        self._last_now = now
        for spec in self.specs:
            key = spec["key"]
            value = spec["age"](snapshot) if spec.get("age") else age(snapshot, key)
            rate = reported_rate(snapshot, *spec["rate"]) if spec.get("rate") else None
            queue = self.samples[key]
            queue.append((now, value, rate))
            while queue and queue[0][0] < now - self.window_s:
                queue.popleft()
            stale = value is None or value > spec.get("limit", 2.)
            if self._stale.get(key) is False and stale:
                self.drops[key].append(now)
            self._stale[key] = stale
            drops = self.drops[key]
            while drops and drops[0] < now - self.window_s:
                drops.popleft()
        return True

    def rows(self):
        result = []
        for spec in self.specs:
            key = spec["key"]
            queue = self.samples[key]
            ages = [a for _, a, _ in queue if a is not None]
            rates = [r for _, _, r in queue if r is not None]
            current = queue[-1][1] if queue else None
            limit = spec.get("limit", 2.)
            kept = (sum(1 for _, a, _ in queue if a is not None and a <= limit) / len(queue)) if queue else None
            span = (queue[-1][0] - queue[0][0]) if len(queue) > 1 else 0.
            result.append({
                "key": key, "label": spec.get("label", key), "limit_s": limit,
                "age_s": current, "age_stats": stats(ages), "fresh_ratio": kept,
                "dropouts": len(self.drops[key]), "span_s": span,
                "rate_hz": queue[-1][2] if queue else None, "rate_stats": stats(rates),
            })
        return result

    def series(self, key):
        return [(t, a) for t, a, _ in self.samples.get(key, ())]


class TransitionLog:
    """Bounded record of value changes for named fields (time, field, old, new)."""

    def __init__(self, maxlen=40):
        self.rows = deque(maxlen=maxlen)
        self.previous = {}
        self._last_now = None

    def clear(self):
        self.rows.clear()
        self.previous.clear()
        self._last_now = None

    def update(self, now, values, clock=None):
        if finite(now) and self._last_now is not None and now < self._last_now - .5:
            self.clear()
        if finite(now):
            self._last_now = now
        for field, value in values.items():
            text = plain(value, 160)
            old = self.previous.get(field)
            if old is not None and old != text:
                self.rows.append((clock or "—", field, old, text, now))
            self.previous[field] = text

    def count_since(self, now, field=None, window_s=WINDOW_S):
        return sum(1 for row in self.rows if finite(row[4]) and finite(now) and now - row[4] <= window_s
                   and (field is None or row[1] == field))

    def table(self):
        return [[clock, field, old, new] for clock, field, old, new, _ in reversed(self.rows)]


# ---------------------------------------------------------------------------- widgets

def _table(headers, stretch_last=True):
    table = QtWidgets.QTableWidget(0, len(headers))
    table.setHorizontalHeaderLabels(headers)
    table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    table.setAlternatingRowColors(True)
    table.verticalHeader().setVisible(False)
    table.setWordWrap(True)
    table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
    header = table.horizontalHeader()
    header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
    header.setStretchLastSection(stretch_last)
    if stretch_last:
        header.setSectionResizeMode(len(headers) - 1, QtWidgets.QHeaderView.Stretch)
    table.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
    return table


def fill_table(table, rows, level_column=None):
    """Update cells in place; only changed cells are touched."""
    rows = [list(row) for row in rows]
    if not rows:
        rows = [["기록 없음"] + [""] * (table.columnCount() - 1)]
    changed = table.rowCount() != len(rows) or table.property("_fitted") is None
    table.setProperty("_fitted", True)
    if changed:
        table.setRowCount(len(rows))
    for r, row in enumerate(rows):
        for c in range(table.columnCount()):
            value = row[c] if c < len(row) else ""
            level = None
            if c == level_column and value in LEVELS:
                level, value = value, LEVELS[value][0]
            text = str(value)
            item = table.item(r, c)
            if item is None:
                item = QtWidgets.QTableWidgetItem()
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                table.setItem(r, c, item)
            if item.text() != text:
                item.setText(text)
                item.setToolTip(text if len(text) > 40 else "")
                changed = True
            if level is not None:
                _, background, foreground = LEVELS[level]
                if item.data(QtCore.Qt.UserRole) != level:
                    item.setData(QtCore.Qt.UserRole, level)
                    item.setBackground(QtGui.QColor(background))
                    item.setForeground(QtGui.QColor(foreground))
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                    font = item.font()
                    font.setBold(True)
                    item.setFont(font)
    if changed:
        table.resizeRowsToContents()
        height = table.horizontalHeader().height() + 4
        height += sum(table.rowHeight(r) for r in range(table.rowCount()))
        if table.horizontalScrollBar().isVisible():
            height += table.horizontalScrollBar().height()
        table.setFixedHeight(max(height, table.horizontalHeader().height() + 26))


class AgePlot(QtWidgets.QWidget):
    """Small last-60 s receive-age plot. Redrawn at most once per second."""

    COLORS = ("#38bdf8", "#f472b6", "#34d399", "#fbbf24", "#a78bfa", "#fb923c", "#22d3ee", "#e879f9", "#94a3b8")

    def __init__(self, ylabel="Receive age (s)"):
        super().__init__()
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
        self.figure = Figure(figsize=(8, 2.3), facecolor="#111827")
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setMinimumHeight(190)
        self.ax = self.figure.add_subplot(111, facecolor="#0b1220")
        self.figure.subplots_adjust(left=.07, right=.99, bottom=.2, top=.95)
        self.ylabel = ylabel
        self._drawn = None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

    def plot(self, now, series, force=False):
        """series: [(label, [(t, value|None)], limit|None)]"""
        if not finite(now):
            return False
        if not force and self._drawn is not None and abs(now - self._drawn) < 1.:
            return False
        self._drawn = now
        ax = self.ax
        ax.clear()
        top = 1.
        for index, (label, points, limit) in enumerate(series):
            xs, ys = [], []
            for t, value in points:
                xs.append(t - now)
                ys.append(value if finite(value) else float("nan"))
            color = self.COLORS[index % len(self.COLORS)]
            if xs:
                ax.plot(xs, ys, color=color, lw=1.3, label=label)
                finite_values = [v for v in ys if finite(v)]
                if finite_values:
                    top = max(top, min(max(finite_values), 30.))
            if finite(limit):
                top = max(top, limit * 1.3)
        ax.set_xlim(-WINDOW_S, 0)
        ax.set_ylim(0, top * 1.1)
        ax.tick_params(colors="#94a3b8", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#475569")
        ax.set_xlabel("Seconds before now", color="#cbd5e1", fontsize=8)
        ax.set_ylabel(self.ylabel, color="#cbd5e1", fontsize=8)
        ax.grid(color="#64748b", alpha=.25, lw=.5)
        if ax.lines:
            ax.legend(loc="upper left", fontsize=7, facecolor="#111827", edgecolor="#475569",
                      labelcolor="#e2e8f0", ncol=min(4, len(ax.lines)))
        self.canvas.draw_idle()
        return True


class DebugPanel(QtWidgets.QWidget):
    """Checklist, receive statistics and named key/value or row tables."""

    def __init__(self, parent=None, plot_ylabel="Receive age (s)"):
        super().__init__(parent)
        # Always dark, also inside windows that keep the default light theme.
        self.setObjectName("debugPanel")
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.setStyleSheet("QWidget#debugPanel {background:#111827;} QGroupBox {color:#e2e8f0;} " + PANEL_STYLE)
        self.layout_ = QtWidgets.QVBoxLayout(self)
        self.layout_.setContentsMargins(6, 4, 6, 6)
        self.layout_.setSpacing(6)
        box = QtWidgets.QGroupBox("상태 점검 · 조건별 판정과 실제 사유")
        inner = QtWidgets.QVBoxLayout(box)
        self.checks = _table(["항목", "판정", "현재 값 · 사유"])
        self.checks.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        self.checks.setColumnWidth(1, 64)
        inner.addWidget(self.checks)
        self.layout_.addWidget(box)
        box = QtWidgets.QGroupBox("입력·출력 수신 기록 · 최근 60초")
        inner = QtWidgets.QVBoxLayout(box)
        note = QtWidgets.QLabel("나이 = 마지막 수신 후 지난 시간. 유지율 = 기준 시간 안에 수신된 비율. "
                                "끊김 = 정상에서 기준 초과로 바뀐 횟수. 보고 Hz = 보드 수신 프로그램이 1초마다 센 값.")
        note.setWordWrap(True)
        note.setStyleSheet("color:#94a3b8;font-size:11px;font-weight:400;")
        inner.addWidget(note)
        self.streams = _table(["항목", "현재 나이", "나이 최소/평균/최대", "유지율", "끊김",
                               "보고 Hz", "Hz 최소/평균/최대"])
        inner.addWidget(self.streams)
        self.plot = AgePlot(plot_ylabel)
        inner.addWidget(self.plot)
        self.layout_.addWidget(box)
        self.sections = {}
        self.extra_widgets = []
        self.layout_.addStretch(1)

    def add_widget(self, title, widget):
        box = QtWidgets.QGroupBox(title)
        inner = QtWidgets.QVBoxLayout(box)
        inner.addWidget(widget)
        self.layout_.insertWidget(self.layout_.count() - 1, box)
        self.extra_widgets.append(box)
        return box

    def section(self, name, headers, note=""):
        if name not in self.sections:
            box = QtWidgets.QGroupBox(name)
            inner = QtWidgets.QVBoxLayout(box)
            label = QtWidgets.QLabel(note)
            label.setWordWrap(True)
            label.setStyleSheet("color:#94a3b8;font-size:11px;font-weight:400;")
            label.setVisible(bool(note))
            inner.addWidget(label)
            table = _table(headers)
            inner.addWidget(table)
            self.layout_.insertWidget(self.layout_.count() - 1, box)
            self.sections[name] = (box, table, label, list(headers))
        return self.sections[name]


def stream_table_rows(stream_rows):
    rows = []
    for row in stream_rows:
        ages = row["age_stats"]
        rates = row["rate_stats"]
        rows.append([row["label"], age_text(row["age_s"]),
                     " / ".join(f"{v:.1f}" for v in ages) + "초" if ages else "—",
                     f"{row['fresh_ratio']*100:.0f}%" if row["fresh_ratio"] is not None else "—",
                     str(row["dropouts"]),
                     number_text(row["rate_hz"], 1, " Hz"),
                     " / ".join(f"{v:.1f}" for v in rates) + " Hz" if rates else "—"])
    return rows


class DebugTools(QtCore.QObject):
    """Toolbar + health strip + '점검·통계' tab wrapped around a window's view.

    A window calls observe(snapshot) on every update (history keeps running
    while the display is frozen) and publish(...) after drawing its view.
    """

    def __init__(self, window, module, title, streams, log_dock=None, extra_buttons=()):
        super().__init__(window)
        self.window, self.module, self.title = window, module, title
        self.stats = StreamStats(streams)
        self.frozen = False
        self.report = {}
        self.log_dock = log_dock
        self.clear_hooks = []
        self._last_snapshot = {}
        self.toolbar = QtWidgets.QWidget()
        self.toolbar.setStyleSheet(TOOL_STYLE)
        bar = QtWidgets.QHBoxLayout(self.toolbar)
        bar.setContentsMargins(4, 2, 6, 2)
        bar.setSpacing(5)
        self.freeze_button = QtWidgets.QPushButton("화면 멈춤")
        self.freeze_button.setCheckable(True)
        self.freeze_button.setToolTip("화면 값만 멈춥니다. 수신·주행 기록·최근 60초 통계는 계속 쌓입니다.")
        self.freeze_button.toggled.connect(self.set_frozen)
        self.copy_button = QtWidgets.QPushButton("현재 상태 복사")
        self.copy_button.setToolTip("점검표·수신 기록·주요 값을 글로 클립보드에 복사합니다.")
        self.copy_button.clicked.connect(self.copy_report)
        self.save_button = QtWidgets.QPushButton("JSON으로 저장")
        self.save_button.setToolTip("이 노트북의 GUI 상태 폴더에 저장합니다. 보드에는 아무것도 쓰지 않습니다.")
        self.save_button.clicked.connect(self.save_report)
        self.log_button = QtWidgets.QPushButton("로그 숨기기")
        self.log_button.setToolTip("오른쪽 실행 로그 칸을 보이거나 숨깁니다. 기존 로그 수신은 그대로입니다.")
        self.log_button.clicked.connect(self.toggle_log)
        self.log_button.setVisible(log_dock is not None)
        self.clear_button = QtWidgets.QPushButton("기록 초기화")
        self.clear_button.setToolTip("이 창의 최근 60초 통계·변화 기록만 비웁니다. 보드와 주행 기록은 그대로입니다.")
        self.clear_button.clicked.connect(self.clear_history)
        for button in (self.freeze_button, self.copy_button, self.save_button, self.log_button, self.clear_button):
            bar.addWidget(button)
        for button in extra_buttons:
            bar.addWidget(button)
        self.result = QtWidgets.QLabel("")
        self.result.setStyleSheet("color:#94a3b8;font-size:11px;")
        self.result.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.result.setWordWrap(True)
        self.result.setMaximumWidth(380)
        self.strip = QtWidgets.QLabel("상태 점검 대기")
        self.strip.setWordWrap(True)
        self.strip.setTextFormat(QtCore.Qt.PlainText)
        self.strip.setStyleSheet(self._strip_style("unknown"))
        self.panel = DebugPanel()
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidget(self.panel)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.tabBar().setUsesScrollButtons(False)
        self.tabs.setStyleSheet("QTabBar::tab {background:#1e293b;color:#cbd5e1;padding:6px 16px;margin-right:2px;}"
                                "QTabBar::tab:selected {background:#334155;color:#ffffff;font-weight:600;}"
                                "QTabWidget::pane {border:1px solid #334155;}")
        view = window.takeCentralWidget()
        if view is not None:
            self.tabs.addTab(view, "모듈 화면")
        self.tabs.addTab(scroll, "점검 · 통계")
        self.tabs.currentChanged.connect(lambda _index: self._render(force=True))
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)
        # Tool buttons share the tab-bar row so the module view keeps its height.
        self.tabs.setCornerWidget(self.toolbar, QtCore.Qt.TopRightCorner)
        strip_row = QtWidgets.QHBoxLayout()
        strip_row.setContentsMargins(6, 4, 6, 0)
        strip_row.addWidget(self.strip, 1)
        strip_row.addWidget(self.result)
        layout.addLayout(strip_row)
        layout.addWidget(self.tabs, 1)
        window.setCentralWidget(container)

    @staticmethod
    def _strip_style(level):
        _, background, foreground = LEVELS.get(level, LEVELS["unknown"])
        return (f"QLabel {{background:{background};color:{foreground};border-radius:4px;"
                "padding:5px 9px;font-size:12px;font-weight:600;}")

    # ------------------------------------------------------------------ actions
    def set_frozen(self, frozen):
        self.frozen = bool(frozen)
        if self.report:
            self.report["frozen"] = self.frozen
        self.freeze_button.setText("화면 재개" if self.frozen else "화면 멈춤")
        if self.freeze_button.isChecked() != self.frozen:
            self.freeze_button.setChecked(self.frozen)
        self._update_strip()
        if not self.frozen and self._last_snapshot and hasattr(self.window, "update_snapshot"):
            self.window.update_snapshot(self._last_snapshot)

    def toggle_log(self):
        if self.log_dock is None:
            return
        visible = not self.log_dock.isVisible()
        self.log_dock.setVisible(visible)
        self.log_button.setText("로그 숨기기" if visible else "로그 보기")

    def clear_history(self):
        self.stats.clear()
        for hook in self.clear_hooks:
            hook()
        self.result.setText("최근 기록 초기화 · " + time.strftime("%H:%M:%S"))
        self._render(force=True)

    def report_text(self):
        report = self.report or {}
        lines = [f"[{report.get('window', self.title)} · 상태 스냅샷] {report.get('captured_local', '—')}"
                 f" · {report.get('mode', '—')}" + (" · 화면 멈춤 중" if report.get("frozen") else "")]
        lines.append("")
        lines.append("■ 상태 점검")
        for row in report.get("checks", ()):
            lines.append(f"- {row['item']}: {LEVELS.get(row['level'], LEVELS['unknown'])[0]} · {row['detail']}")
        lines.append("")
        lines.append("■ 수신 기록 (최근 60초)")
        for row in stream_table_rows(report.get("_stream_rows", ())):
            lines.append(f"- {row[0]}: 현재 {row[1]} · 나이 {row[2]} · 유지율 {row[3]} · 끊김 {row[4]}"
                         f" · 보고 {row[5]} ({row[6]})")
        for name, section in report.get("sections", {}).items():
            lines.append("")
            lines.append("■ " + name)
            headers = section.get("headers", [])
            if len(headers) > 2:
                lines.append("  " + " | ".join(headers))
            for row in section.get("rows", ()):
                lines.append("- " + (f"{row[0]}: {row[1]}" if len(row) == 2 else " | ".join(str(v) for v in row)))
        return "\n".join(lines)

    def copy_report(self):
        text = self.report_text()
        QtWidgets.QApplication.clipboard().setText(text)
        self.result.setText(f"클립보드에 복사 · {len(text.splitlines())}줄 · " + time.strftime("%H:%M:%S"))
        return text

    def save_directory(self):
        from board_address import state_dir
        return state_dir() / "developer_snapshots"

    def save_report(self):
        folder = self.save_directory()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        try:
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / f"{self.module}_{stamp}.json"
            index = 1
            while path.exists():
                path = folder / f"{self.module}_{stamp}_{index}.json"
                index += 1
            data = {key: value for key, value in self.report.items() if not key.startswith("_")}
            data["streams"] = self.report.get("streams", [])
            path.write_text(json.dumps(data, ensure_ascii=False, indent=1, allow_nan=False), encoding="utf-8")
        except (OSError, ValueError, TypeError) as error:
            self.result.setText("저장 실패 · " + str(error)[:160])
            return None
        self.result.setText("저장: " + str(path))
        self.result.setToolTip(str(path))
        return path

    # ----------------------------------------------------------------- updates
    def observe(self, snapshot):
        if isinstance(snapshot, Mapping):
            self._last_snapshot = snapshot
            self.stats.observe(snapshot)

    def publish(self, snapshot, checks, sections=None):
        """checks: [(item, level, detail)], sections: {name: (headers, rows, note)}"""
        snapshot = snapshot if isinstance(snapshot, Mapping) else {}
        stream_rows = self.stats.rows()
        wall = snapshot.get("wall_now")
        captured = datetime.fromtimestamp(wall if finite(wall) else time.time())
        self.report = {
            "schema": SCHEMA, "module": self.module, "window": self.title,
            "captured_local": captured.strftime("%Y-%m-%d %H:%M:%S"),
            "mode": "기록 재생" if snapshot.get("replay") else "실시간",
            "frozen": self.frozen,
            "checks": [{"item": str(item), "level": level, "detail": str(detail)} for item, level, detail in checks],
            "streams": [{**{k: v for k, v in row.items() if k not in ("age_stats", "rate_stats")},
                         "age_min_avg_max_s": list(row["age_stats"]) if row["age_stats"] else None,
                         "rate_min_avg_max_hz": list(row["rate_stats"]) if row["rate_stats"] else None}
                        for row in stream_rows],
            "_stream_rows": stream_rows,
            "sections": {name: {"headers": list(value[0]), "rows": [[str(v) for v in row] for row in value[1]],
                                **({"note": value[2]} if len(value) > 2 and value[2] else {})}
                         for name, value in (sections or {}).items()},
        }
        self._update_strip()
        self._render()

    def _update_strip(self):
        checks = self.report.get("checks", []) if self.report else []
        counts = {level: 0 for level in LEVELS}
        worst = None
        for row in checks:
            counts[row["level"]] = counts.get(row["level"], 0) + 1
            if row["level"] in ("fail", "warn") and (worst is None or SEVERITY[row["level"]] > SEVERITY[worst["level"]]):
                worst = row
        level = max((row["level"] for row in checks), key=lambda v: SEVERITY.get(v, 0), default="unknown")
        text = (f"점검 {len(checks)}개 · 정상 {counts['ok']} · 주의 {counts['warn']} · 실패 {counts['fail']}"
                f" · 미확인 {counts['unknown']}")
        if worst is not None:
            text += f"  |  가장 먼저 볼 항목: {worst['item']} — {worst['detail']}"
        if self.frozen:
            text = "화면 멈춤 중 (수신과 통계는 계속) · " + text
            level = "warn" if level == "ok" else level
        if self.strip.text() != text:
            self.strip.setText(text[:600])
        style = self._strip_style(level)
        if self.strip.styleSheet() != style:
            self.strip.setStyleSheet(style)

    def _render(self, force=False):
        if not self.report:
            return
        if not force and not self.panel.isVisible():
            return
        fill_table(self.panel.checks, [[row["item"], row["level"], row["detail"]]
                                       for row in self.report["checks"]], level_column=1)
        stream_rows = self.report.get("_stream_rows", [])
        fill_table(self.panel.streams, stream_table_rows(stream_rows))
        now = self.stats._last_now
        self.panel.plot.plot(now, [(row["key"].split(":")[-1], self.stats.series(row["key"]), row["limit_s"])
                                   for row in stream_rows], force=force)
        for name, section in self.report.get("sections", {}).items():
            _, table, label, _headers = self.panel.section(name, section["headers"], section.get("note", ""))
            if label.text() != section.get("note", ""):
                label.setText(section.get("note", ""))
                label.setVisible(bool(section.get("note")))
            fill_table(table, section["rows"])
