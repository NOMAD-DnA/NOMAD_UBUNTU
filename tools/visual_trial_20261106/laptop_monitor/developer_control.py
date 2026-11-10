"""Read-only control developer view of existing planning telemetry and logs.

This window owns no receiver, SSH connection, timer, or actuator API. The
operator's existing planning log stream is reused by operator and spectators.
"""
from __future__ import annotations

from collections.abc import Mapping
import math
import re
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from module_logs import attach_module_logs, update_module_logs
from render_utils import set_text, stable_metadata


STYLE = "QWidget {background:#111827;color:#e2e8f0;} QLabel {font-size:12px;}"
_MODULE = re.compile(r"^\s*\[(controller|driver)\](?:\s|$)")
_ROS_STAMP = re.compile(r"\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*\[([0-9]+(?:\.[0-9]+)?)\]")
_KEY = re.compile(r"(?:^|\s)([A-Za-z_][A-Za-z0-9_]*)=")
_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
TRACE_FIELDS = ("requested_erpm", "target_erpm", "output_erpm", "measured_erpm",
                "current_a", "phase", "source", "reset_required", "start_gate")
_NUMERIC = frozenset(("requested_erpm", "target_erpm", "output_erpm", "measured_erpm", "current_a"))


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _number(value):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def trace_from_line(line):
    """Parse one real controller trace; absent fields never inherit old values."""
    line = _ANSI.sub("", line)
    module = _MODULE.match(line)
    marker = line.find("control_trace ")
    if module is None or module.group(1) != "controller" or marker < 0:
        return None
    payload = line[marker + len("control_trace "):]
    matches = list(_KEY.finditer(payload))
    values = {match.group(1): payload[match.end():matches[index+1].start()
              if index+1 < len(matches) else len(payload)].strip()
              for index, match in enumerate(matches)}
    trace = {key: None for key in TRACE_FIELDS}
    for key in TRACE_FIELDS:
        value = values.get(key)
        if key in _NUMERIC:
            trace[key] = _number(value)
        elif key == "reset_required":
            trace[key] = value if value in ("0", "1") else None
        else:
            trace[key] = value[:500] if value else None
    stamp = _ROS_STAMP.search(line[:marker])
    trace["stamp"] = _number(stamp.group(1)) if stamp else None
    return trace


class ServoCommandBar(QtWidgets.QProgressBar):
    """Read-only 0..1 reference with a fixed 0.50 center and no fake sample."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.command = None
        self.setRange(0, 1000)
        self.setMinimumHeight(64)
        self.setTextVisible(False)
        self.setEnabled(False)
        self.setToolTip("서보 명령값 전용입니다. 0.50은 중앙 참고값이며 실측 바퀴 조향각이 아닙니다.")

    def set_command(self, value):
        command = float(value) if finite(value) and 0. <= value <= 1. else None
        if command == self.command:
            return
        self.command = command
        self.setEnabled(command is not None)
        # The internal range exists for accessibility; a missing sample is
        # painted without a marker or numerical value, rather than as zero.
        if command is not None:
            self.setValue(round(command * 1000))
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        left, right, y = 22., max(23., self.width()-22.), 22.
        painter.setPen(QtGui.QPen(QtGui.QColor("#475569"), 4))
        painter.drawLine(QtCore.QPointF(left, y), QtCore.QPointF(right, y))
        painter.setPen(QtGui.QPen(QtGui.QColor("#94a3b8"), 1))
        for value, label in ((0., "0.00"), (.5, "0.50 · 중앙"), (1., "1.00")):
            x = left + value * (right-left)
            painter.drawLine(QtCore.QPointF(x, y-7), QtCore.QPointF(x, y+7))
            painter.drawText(QtCore.QRectF(x-48, y+12, 96, 18), QtCore.Qt.AlignCenter, label)
        if self.command is not None:
            x = left + self.command * (right-left)
            painter.setPen(QtGui.QPen(QtGui.QColor("#22d3ee"), 3))
            painter.drawLine(QtCore.QPointF(x, y-12), QtCore.QPointF(x, y+11))
        else:
            painter.setPen(QtGui.QColor("#cbd5e1"))
            painter.drawText(QtCore.QRectF(left, 0, right-left, 15), QtCore.Qt.AlignCenter, "명령값 미수신")


class ControlWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.setWindowTitle("NOMAD | 제어 개발")
        self.resize(1220, 770)
        self.setStyleSheet(STYLE)
        self._log_session = None
        self._trace = None
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setSpacing(9)
        self.host = QtWidgets.QLabel("Pi4 · 주소 미수신")
        self.host.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.status = QtWidgets.QLabel("플래너 출력 상태 · 미수신")
        self.status.setWordWrap(True)
        stable_metadata(self.status, rows=3)
        layout.addWidget(self.host)
        layout.addWidget(self.status)

        telemetry_box = QtWidgets.QGroupBox("플래너에서 받은 출력 상태")
        telemetry_grid = QtWidgets.QGridLayout(telemetry_box)
        self.telemetry_values = {}
        for row, (key, title) in enumerate((("state", "주행 상태"), ("reason", "상태 사유"),
                ("armed", "출력 허용 의도 · armed"), ("actuator_enabled", "출력 실행 허용 · actuator_enabled"),
                ("output_inhibit", "출력 차단 사유"), ("feedback_source", "피드백 출처"))):
            telemetry_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            label = QtWidgets.QLabel("미수신")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            telemetry_grid.addWidget(label, row, 1)
            self.telemetry_values[key] = label
        telemetry_grid.setColumnStretch(1, 1)
        layout.addWidget(telemetry_box)

        trace_box = QtWidgets.QGroupBox("Pi4 제어기 · 기존 control_trace 로그")
        trace_layout = QtWidgets.QVBoxLayout(trace_box)
        self.trace_status = QtWidgets.QLabel("제어기 trace · 미수신")
        self.trace_status.setWordWrap(True)
        stable_metadata(self.trace_status, rows=2)
        trace_layout.addWidget(self.trace_status)
        trace_grid = QtWidgets.QGridLayout()
        self.values = {}
        rows = (("requested_erpm", "요청 ERPM"), ("target_erpm", "목표 ERPM"),
                ("output_erpm", "출력 ERPM 명령"), ("measured_erpm", "측정 ERPM"),
                ("current_a", "모터 전류 · A"), ("phase", "제어 상태 · phase"),
                ("source", "명령 출처 · source"), ("reset_required", "재설정 필요 · reset_required"),
                ("start_gate", "출발 대기 사유 · start_gate"))
        for row, (key, title) in enumerate(rows):
            trace_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            label = QtWidgets.QLabel("미수신")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            trace_grid.addWidget(label, row, 1)
            self.values[key] = label
        trace_grid.setColumnStretch(1, 1)
        trace_layout.addLayout(trace_grid)
        layout.addWidget(trace_box)

        servo_box = QtWidgets.QGroupBox("서보 명령 · 읽기 전용")
        servo_layout = QtWidgets.QVBoxLayout(servo_box)
        self.servo = ServoCommandBar()
        servo_layout.addWidget(self.servo)
        self.servo_status = QtWidgets.QLabel("미수신 · 현재 제어기 로그에 서보 명령값이 없습니다.")
        self.servo_status.setWordWrap(True)
        servo_layout.addWidget(self.servo_status)
        layout.addWidget(servo_box)
        footer = QtWidgets.QLabel("기존 수신 공유 · 제어/드라이버 로그만 표시 · 이 창은 구동 명령을 보내지 않습니다.\n"
                                  "출력 ERPM은 명령값이며, 실제 피드백은 측정 ERPM입니다. 서보 중앙 0.50은 참고값입니다.")
        footer.setWordWrap(True)
        footer.setStyleSheet("color:#94a3b8;")
        layout.addWidget(footer)
        layout.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidget(body)
        self.setCentralWidget(scroll)
        attach_module_logs(self)

    def append_module_log(self, event):
        if not isinstance(event, Mapping) or event.get("board") != "pi4" or event.get("source") != "planning":
            return 0
        raw = event.get("text")
        if not isinstance(raw, str):
            values = event.get("lines")
            raw = "\n".join(value for value in values[-200:] if isinstance(value, str)) if isinstance(values, (tuple, list)) else ""
        rows = [line for line in raw.splitlines()[-200:] if _MODULE.match(_ANSI.sub("", line))]
        if not rows:
            return 0
        filtered = {**event, "text": "\n".join(rows)}
        added = self.log_panel.append_event(filtered)
        if not added:
            return 0
        session = str(event.get("run_id", event.get("session", "")))
        if session != self._log_session:
            self._log_session, self._trace = session, None
        for row in rows:
            clean = _ANSI.sub("", row)
            trace = trace_from_line(clean)
            if trace is not None:
                previous_stamp = self._trace.get("stamp") if self._trace else None
                if finite(previous_stamp) and (not finite(trace["stamp"]) or trace["stamp"] <= previous_stamp):
                    continue
                trace.update(event_stamp=event.get("t"),
                             received_monotonic=getattr(self, "_display_now", time.monotonic()), session=session)
                self._trace = trace
            elif clean.lstrip().startswith("[controller]") and re.search(r"\bmode=commission\b", clean):
                # The tail helper may keep a session when the controller restarts.
                # Do not carry a previous run's numbers over its startup line.
                stamp = _ROS_STAMP.search(clean)
                value = _number(stamp.group(1)) if stamp else None
                old = self._trace.get("stamp") if self._trace else None
                if not finite(old) or not finite(value) or value >= old:
                    self._trace = None
        return added

    def _trace_age(self, snapshot):
        if not self._trace:
            return None, "미수신"
        stamp, sent = self._trace.get("stamp"), self._trace.get("event_stamp")
        wall = snapshot.get("wall_now", time.time())
        now = snapshot.get("now", time.monotonic())
        if not finite(stamp) or stamp <= 0 or not finite(wall):
            return None, "원본 시각 미확인 · 최신 여부 확인 불가"
        if stamp > wall+2. or (finite(sent) and sent > wall+2.):
            return None, "보드/PC 시각 불일치 · 최신 여부 확인 불가"
        ages = [max(0., wall-stamp)]
        if finite(sent) and sent > 0:
            ages.append(max(0., wall-sent))
        received = self._trace.get("received_monotonic")
        if finite(now) and finite(received):
            ages.append(max(0., now-received))
        age = max(ages)
        return age, f"원본 로그 {age:.1f}초 전" + (" · 오래된 값" if age > 3. else "")

    def update_snapshot(self, snapshot):
        self._display_now = snapshot.get("now", time.monotonic())
        update_module_logs(self, snapshot, "planning")
        ui = snapshot.get("planning_ui", {})
        set_text(self.host, "Pi4 · " + str(ui.get("host") or "주소 미수신") +
                 (" · 관전" if ui.get("spectator") else "") + " · 읽기 전용")
        report = snapshot.get("telemetry", {}).get("planning", {})
        now = snapshot.get("now", time.monotonic())
        received = snapshot.get("received", {}).get("telemetry:planning")
        age = max(0., now-received) if finite(now) and finite(received) else None
        freshness = "미수신" if age is None else f"수신 후 {age:.1f}초" + (" · 오래된 상태" if age > 3. else "")
        for key, label in self.telemetry_values.items():
            value = report.get(key)
            if key in ("armed", "actuator_enabled"):
                text = str(value).lower() if isinstance(value, bool) else "미수신"
            elif value is None:
                text = "미수신"
            else:
                text = str(value) or ("없음" if key == "output_inhibit" else "미수신")
            set_text(label, text)
        set_text(self.status, "플래너 출력 상태 · " + freshness + "\n"
                 "armed=" + self.telemetry_values["armed"].text() +
                 " · actuator_enabled=" + self.telemetry_values["actuator_enabled"].text() +
                 " · " + self.telemetry_values["state"].text())
        trace_age, trace_note = self._trace_age(snapshot)
        set_text(self.trace_status, "제어기 trace · " + trace_note + "\n기존 실행 로그의 한 시점 값 · 실시간 전용 계측 아님")
        for key, label in self.values.items():
            value = self._trace.get(key) if self._trace else None
            text = (f"{value:.2f}" if key == "current_a" else f"{value:.1f}") if key in _NUMERIC and finite(value) else str(value) if value is not None else "미수신"
            set_text(label, text)
            style = "color:#e2e8f0;" if trace_age is not None and trace_age <= 3. else "color:#94a3b8;"
            if label.styleSheet() != style:
                label.setStyleSheet(style)
        # No servo sample is present in the current real control_trace format.
        # Neither a motor value nor the 0.50 reference is a steering command.
        self.servo.set_command(None)
