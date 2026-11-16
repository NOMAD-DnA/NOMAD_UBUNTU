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
import developer_debug as dbg
import vesc_status


STYLE = "QWidget {background:#111827;color:#e2e8f0;} QLabel {font-size:12px;}"
_MODULE = re.compile(r"^\s*\[(controller|driver)\](?:\s|$)")
_ROS_STAMP = re.compile(r"\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*\[([0-9]+(?:\.[0-9]+)?)\]")
_KEY = re.compile(r"(?:^|\s)([A-Za-z_][A-Za-z0-9_]*)=")
_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
TRACE_FIELDS = ("requested_erpm", "target_erpm", "output_erpm", "measured_erpm",
                "current_a", "phase", "source", "reset_required", "start_gate")
_NUMERIC = frozenset(("requested_erpm", "target_erpm", "output_erpm", "measured_erpm", "current_a"))
# 2026-11-16: control_trace에 이미 있던 값 (표시는 아래 '모터·배터리' 묶음)
EXTRA_TRACE_FIELDS = ("esc_temp_c", "thermal", "stop", "stop_a", "erpm_valid")


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
    for key in EXTRA_TRACE_FIELDS:
        value = values.get(key)
        trace[key] = _number(value) if key in ("esc_temp_c", "stop_a") else (value[:40] if value else None)
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


class ErpmPlot(dbg.AgePlot):
    """Last 60 s of received control_trace values on the board log clock."""

    SERIES = (("target_erpm", "Target ERPM", "#fbbf24"), ("output_erpm", "Output ERPM", "#22d3ee"),
              ("measured_erpm", "Measured ERPM", "#34d399"))

    def plot_traces(self, traces, force=False):
        stamps = [row["stamp"] for row in traces if finite(row.get("stamp"))]
        latest = max(stamps) if stamps else None
        if latest is None:
            if force or self._drawn is not None:
                self._drawn = None
                self.ax.clear()
                self.ax.text(.5, .5, "No control_trace yet", color="#94a3b8", ha="center", transform=self.ax.transAxes)
                self.canvas.draw_idle()
            return False
        if not force and self._drawn is not None and self._drawn == latest:
            return False
        self._drawn = latest
        ax = self.ax
        ax.clear()
        for key, label, color in self.SERIES:
            points = [(row["stamp"]-latest, row.get(key)) for row in traces if finite(row.get("stamp"))]
            ax.plot([p[0] for p in points], [p[1] if finite(p[1]) else float("nan") for p in points],
                    color=color, lw=1.3, label=label, marker="." if len(points) < 40 else None)
        ax.set_xlim(-dbg.WINDOW_S, 0)
        ax.tick_params(colors="#94a3b8", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#475569")
        ax.set_xlabel("Seconds before latest trace (board log clock)", color="#cbd5e1", fontsize=8)
        ax.set_ylabel("ERPM", color="#cbd5e1", fontsize=8)
        ax.grid(color="#64748b", alpha=.25, lw=.5)
        ax.legend(loc="upper left", fontsize=7, facecolor="#111827", edgecolor="#475569", labelcolor="#e2e8f0", ncol=3)
        self.canvas.draw_idle()
        return True


class ControlWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.controller = parent
        self.setWindowTitle("NOMAD | 제어 개발")
        self.resize(1220, 770)
        self.setStyleSheet(STYLE)
        self._log_session = None
        self._trace = None
        self._vesc = None
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        layout.setSpacing(9)
        self.host = QtWidgets.QLabel("Pi4 · 주소 미수신")
        self.host.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.status = QtWidgets.QLabel("플래너 출력 상태 · 미수신")
        self.status.setWordWrap(True)
        stable_metadata(self.status, rows=3)
        host_row = QtWidgets.QHBoxLayout()
        host_row.addWidget(self.host, 1)
        self.reset_button = QtWidgets.QPushButton("제어기 복구")
        self.reset_button.setToolTip("플래닝 창과 같은 기존 기능입니다. Pi4 제어기의 치명 잠금을 풉니다. 확인 창을 거치며, "
                                     "차가 0.5초 이상 멈춰 있어야 합니다. 구동 명령은 보내지 않습니다. 운용 PC에서만 사용할 수 있습니다.")
        self.reset_button.setStyleSheet("QPushButton {background:#92400e;color:white;padding:5px 12px;border-radius:4px;}"
                                        "QPushButton:disabled {background:#334155;color:#94a3b8;}")
        self.reset_button.setEnabled(False)
        self.reset_button.clicked.connect(self._request_reset)
        host_row.addWidget(self.reset_button)
        layout.addLayout(host_row)
        self.reset_result = QtWidgets.QLabel("")
        self.reset_result.setWordWrap(True)
        self.reset_result.setStyleSheet("color:#fbbf24;")
        self.reset_result.setVisible(False)
        layout.addWidget(self.reset_result)
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

        vesc_box = QtWidgets.QGroupBox("모터·배터리 · VESC 값 (vesc_trace 로그, 2026-11-16 이후 제어기)")
        vesc_layout = QtWidgets.QVBoxLayout(vesc_box)
        self.vesc_status = QtWidgets.QLabel("vesc_trace · 미수신")
        self.vesc_status.setWordWrap(True)
        vesc_layout.addWidget(self.vesc_status)
        vesc_grid = QtWidgets.QGridLayout()
        self.vesc_values = {}
        for index, (title, _text, _level) in enumerate(vesc_status.trace_rows({}, {})):
            row, column = index // 2, (index % 2) * 2
            vesc_grid.addWidget(QtWidgets.QLabel(title), row, column)
            label = QtWidgets.QLabel("미수신")
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            vesc_grid.addWidget(label, row, column + 1)
            self.vesc_values[title] = label
        vesc_grid.setColumnStretch(1, 1)
        vesc_grid.setColumnStretch(3, 1)
        vesc_layout.addLayout(vesc_grid)
        layout.addWidget(vesc_box)

        servo_box = QtWidgets.QGroupBox("서보 명령 · 읽기 전용")
        servo_layout = QtWidgets.QVBoxLayout(servo_box)
        self.servo = ServoCommandBar()
        servo_layout.addWidget(self.servo)
        self.servo_status = QtWidgets.QLabel("미수신 · 현재 제어기 로그에 서보 명령값이 없습니다.")
        self.servo_status.setWordWrap(True)
        servo_layout.addWidget(self.servo_status)
        layout.addWidget(servo_box)
        footer = QtWidgets.QLabel("기존 수신 공유 · 제어/드라이버 로그만 표시 · 이 창은 구동 명령을 보내지 않습니다 (제어기 복구는 확인 후 잠금 해제만).\n"
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
        self._traces = []
        self._trace_count = 0
        self.transitions = dbg.TransitionLog(maxlen=60)
        self._observed_now = None
        self.debug = dbg.DebugTools(self, "control", "NOMAD 제어 개발", (
            {"key": "telemetry:planning", "label": "플래너 출력 상태", "limit": 3.},
            {"key": "control_trace", "label": "제어기 trace (로그)", "limit": 3.,
             "age": lambda snapshot: self._trace_age(snapshot)[0]},
        ), log_dock=self.log_dock)
        self.debug.clear_hooks.append(self._clear_debug)
        self.erpm_plot = ErpmPlot()
        self.debug.panel.add_widget("ERPM 추이 · 최근 60초 (control_trace)", self.erpm_plot)

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
            self._log_session, self._trace, self._vesc = session, None, None
        for row in rows:
            clean = _ANSI.sub("", row)
            trace = trace_from_line(clean)
            vesc = vesc_status.vesc_trace_from_line(clean) if trace is None else None
            if vesc is not None:
                previous_stamp = self._vesc.get("stamp") if self._vesc else None
                if finite(previous_stamp) and (not finite(vesc["stamp"]) or vesc["stamp"] <= previous_stamp):
                    continue
                vesc.update(event_stamp=event.get("t"),
                            received_monotonic=getattr(self, "_display_now", time.monotonic()), session=session)
                self._vesc = vesc
            elif trace is not None:
                previous_stamp = self._trace.get("stamp") if self._trace else None
                if finite(previous_stamp) and (not finite(trace["stamp"]) or trace["stamp"] <= previous_stamp):
                    continue
                trace.update(event_stamp=event.get("t"),
                             received_monotonic=getattr(self, "_display_now", time.monotonic()), session=session)
                self._trace = trace
                self._record_trace(trace)
            elif clean.lstrip().startswith("[controller]") and re.search(r"\bmode=commission\b", clean):
                # The tail helper may keep a session when the controller restarts.
                # Do not carry a previous run's numbers over its startup line.
                stamp = _ROS_STAMP.search(clean)
                value = _number(stamp.group(1)) if stamp else None
                old = self._trace.get("stamp") if self._trace else None
                if not finite(old) or not finite(value) or value >= old:
                    self._trace = None
                old = self._vesc.get("stamp") if self._vesc else None
                if not finite(old) or not finite(value) or value >= old:
                    self._vesc = None
        return added

    def _clear_debug(self):
        self._traces = []
        self._trace_count = 0
        self.transitions.clear()

    def _record_trace(self, trace):
        self._trace_count += 1
        if finite(trace.get("stamp")):
            self._traces.append(dict(trace))
            latest = trace["stamp"]
            self._traces = [row for row in self._traces if latest - row["stamp"] <= dbg.WINDOW_S][-3000:]
        self.transitions.update(trace.get("stamp"), {"phase": trace.get("phase"), "source": trace.get("source"),
                                                     "reset_required": trace.get("reset_required"),
                                                     "start_gate": trace.get("start_gate") or "없음"},
                                dbg.wall_clock(trace.get("stamp")))

    def _request_reset(self):
        snapshot = getattr(self, "_last_snapshot", {}) or {}
        if self._reset_allowed(snapshot) and self.controller is not None and hasattr(self.controller, "reset_controller"):
            self.controller.reset_controller()

    @staticmethod
    def _reset_allowed(snapshot):
        ui = snapshot.get("planning_ui", {}) if isinstance(snapshot, Mapping) else {}
        return (not snapshot.get("replay") and not ui.get("spectator", True) and not ui.get("no_ssh", True)
                and bool(ui.get("reset_enabled")))

    def _update_actions(self, snapshot):
        self._last_snapshot = snapshot
        ui = snapshot.get("planning_ui", {})
        allowed = self._reset_allowed(snapshot) and self.controller is not None
        self.reset_button.setEnabled(allowed)
        self.reset_button.setText("제어기 복구 중…" if ui.get("reset_busy") else "제어기 복구")
        reset_text = str(ui.get("reset_result", "") or "")
        if self.reset_result.text() != reset_text:
            self.reset_result.setText(reset_text)
        self.reset_result.setVisible(bool(reset_text))

    def _debug_checks(self, snapshot):
        report = snapshot.get("telemetry", {}).get("planning", {})
        now = snapshot.get("now", time.monotonic())
        received = snapshot.get("received", {}).get("telemetry:planning")
        planning_age = max(0., now-received) if finite(now) and finite(received) else None
        checks = [dbg.freshness("플래너 출력 상태 수신", planning_age, 1., 3.)]
        armed, enabled, inhibit = report.get("armed"), report.get("actuator_enabled"), report.get("output_inhibit")
        checks.append(("출력 허용 의도 · armed", "ok" if armed is True else "warn" if armed is False else "unknown",
                       {True: "true", False: "false · 출력 잠금"}.get(armed, "미수신")))
        checks.append(("출력 실행 허용 · actuator_enabled", "ok" if enabled is True else "warn" if enabled is False else "unknown",
                       {True: "true", False: "false · 현재 정지/대기"}.get(enabled, "미수신")))
        checks.append(("출력 차단 사유", "warn" if inhibit else "ok" if report else "unknown",
                       inhibit or ("없음" if report else "미수신")))
        trace_age, trace_note = self._trace_age(snapshot)
        if not self._trace:
            checks.append(("제어기 trace 수신", "fail", "control_trace 로그 미수신 (Pi4 로그 연결 또는 제어기 실행 확인)"))
        else:
            level = "ok" if trace_age is not None and trace_age <= 3. else "warn"
            checks.append(("제어기 trace 수신", level, trace_note))
            reset = self._trace.get("reset_required")
            checks.append(("재설정 필요 · reset_required", "fail" if reset == "1" else "ok" if reset == "0" else "unknown",
                           {"1": "1 · 치명 잠금 상태 · 원인 확인 후 제어기 복구", "0": "0"}.get(reset, "값 없음")))
            gate = self._trace.get("start_gate")
            checks.append(("출발 대기 사유 · start_gate", "warn" if gate else "ok", gate or "비어 있음 (대기 사유 없음 또는 미보고)"))
            target, measured = self._trace.get("target_erpm"), self._trace.get("measured_erpm")
            if finite(target) and finite(measured):
                error = target - measured
                checks.append(("속도 추종 오차 (목표 − 측정)", "ok" if abs(error) <= max(300., abs(target)*.3) else "warn",
                               f"{error:+.0f} ERPM (마지막 trace · 목표의 30%와 300 중 큰 값 초과 시 주의)"))
        if self._vesc:
            code = self._vesc.get("fault_code")
            checks.append(("VESC 고장 코드", vesc_status.fault_level(code),
                           vesc_status.fault_text(code, self._vesc.get("fault"))))
        if self._trace and self._trace.get("thermal"):
            thermal = self._trace.get("thermal")
            checks.append(("ESC 온도", vesc_status.thermal_level(thermal),
                           f"{vesc_status._num(self._trace.get('esc_temp_c'), 1)} ℃ · {thermal}"))
        checks.append(dbg.log_check(self.log_panel, "최근 제어/드라이버 오류 로그"))
        return checks

    def _debug_sections(self, snapshot):
        report = snapshot.get("telemetry", {}).get("planning", {})
        traces = self._traces
        latest = traces[-1]["stamp"] if traces else None
        rows = []
        for key, title in (("requested_erpm", "요청 ERPM"), ("target_erpm", "목표 ERPM"), ("output_erpm", "출력 ERPM 명령"),
                           ("measured_erpm", "측정 ERPM"), ("current_a", "모터 전류 · A")):
            values = [row.get(key) for row in traces]
            digits = 2 if key == "current_a" else 0
            current = self._trace.get(key) if self._trace else None
            rows.append([title, dbg.number_text(current, digits), dbg.stats_text(values, digits),
                         f"{sum(1 for v in values if finite(v))}개"])
        errors = [row["target_erpm"]-row["measured_erpm"] for row in traces
                  if finite(row.get("target_erpm")) and finite(row.get("measured_erpm"))]
        abs_errors = [abs(v) for v in errors]
        rows.append(["추종 오차 (목표 − 측정)", dbg.number_text(errors[-1], 0) if errors else "—",
                     dbg.stats_text(errors, 0), f"평균 절댓값 {sum(abs_errors)/len(abs_errors):.0f}" if abs_errors else "—"])
        command_gap = [row["output_erpm"]-row["target_erpm"] for row in traces
                       if finite(row.get("output_erpm")) and finite(row.get("target_erpm"))]
        rows.append(["출력 − 목표 (명령 제한 효과)", dbg.number_text(command_gap[-1], 0) if command_gap else "—",
                     dbg.stats_text(command_gap, 0), "—"])
        span = traces[-1]["stamp"] - traces[0]["stamp"] if len(traces) > 1 else 0.
        info = [["trace 수 (최근 60초 · 보드 로그 시각)", f"{len(traces)}개" + (f" · 약 {(len(traces)-1)/span:.1f}개/초" if span > 0 else "")],
                ["이 창이 받은 trace 누적", f"{self._trace_count}개"],
                ["마지막 trace 시각", dbg.wall_clock(latest)],
                ["phase 변화 (최근 60초)", f"{self.transitions.count_since(latest, 'phase')}회"],
                ["reset_required=1 표본", f"{sum(1 for row in traces if row.get('reset_required') == '1')}개"],
                ["플래너 주행 상태 · 사유", f"{dbg.plain(report.get('state'))} · {dbg.plain(report.get('reason'))}" if report else "미수신"],
                ["출력 방식 · 피드백 출처", f"{dbg.plain(report.get('output_mode'))} · {dbg.plain(report.get('feedback_source'))}" if report else "미수신"],
                ["임시 환산 출력 (prototype_output)", dbg.plain(report.get("prototype_output")) if report else "미수신"]]
        return {
            "제어 값 통계 · 최근 60초": (["항목", "마지막 값", "최소 / 평균 / 최대", "표본"], rows,
                "기존 control_trace 로그 줄에서 읽은 값입니다. 로그가 1초 묶음으로 오므로 짧은 순간 값은 빠질 수 있습니다. "
                "출력 ERPM은 명령값, 측정 ERPM이 실제 피드백입니다."),
            "trace · 출력 정보": (["항목", "값"], info, ""),
            "제어 상태 변화 기록 · 최근 60건": (["보드 로그 시각", "항목", "이전", "현재"], self.transitions.table(),
                                       "phase/source/reset_required/start_gate가 바뀔 때만 한 줄 추가합니다. 위가 최신입니다."),
            "제어 관련 프로세스 · 보드 부하 계측": (dbg.PROCESS_HEADERS,
                dbg.module_processes(snapshot, ("control", "vesc_driver")), dbg.PROCESS_NOTE),
        }

    def _trace_age(self, snapshot):
        return self._record_age(self._trace, snapshot)

    def _record_age(self, record, snapshot):
        if not record:
            return None, "미수신"
        stamp, sent = record.get("stamp"), record.get("event_stamp")
        wall = snapshot.get("wall_now", time.time())
        now = snapshot.get("now", time.monotonic())
        if not finite(stamp) or stamp <= 0 or not finite(wall):
            return None, "원본 시각 미확인 · 최신 여부 확인 불가"
        if stamp > wall+2. or (finite(sent) and sent > wall+2.):
            return None, "보드/PC 시각 불일치 · 최신 여부 확인 불가"
        ages = [max(0., wall-stamp)]
        if finite(sent) and sent > 0:
            ages.append(max(0., wall-sent))
        received = record.get("received_monotonic")
        if finite(now) and finite(received):
            ages.append(max(0., now-received))
        age = max(ages)
        return age, f"원본 로그 {age:.1f}초 전" + (" · 오래된 값" if age > 3. else "")

    def update_snapshot(self, snapshot):
        now = snapshot.get("now")
        if finite(now) and finite(self._observed_now) and now < self._observed_now - .5:
            self._clear_debug()  # replay moved back in time
        if finite(now):
            self._observed_now = now
        self.debug.observe(snapshot)
        self._update_actions(snapshot)
        if self.debug.frozen:
            return
        self._update_view(snapshot)
        self.debug.publish(snapshot, self._debug_checks(snapshot), self._debug_sections(snapshot))
        if self.debug.panel.isVisible():
            self.erpm_plot.plot_traces(self._traces)

    def _update_view(self, snapshot):
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
        # 2026-11-16: vesc_trace carries the commanded servo (never a measured steering angle).
        vesc_age, vesc_note = self._record_age(self._vesc, snapshot)
        vesc_fresh = vesc_age is not None and vesc_age <= 3.
        set_text(self.vesc_status, "vesc_trace · " + (vesc_note if self._vesc else
                 "미수신 (2026-11-16 이전 제어기에는 없음)") + " · 입력 전류는 VESC 추정값")
        control_fresh = trace_age is not None and trace_age <= 3.
        for title, text, level in vesc_status.trace_rows(self._trace if control_fresh else None,
                                                         self._vesc if vesc_fresh else None):
            label = self.vesc_values.get(title)
            if label is None:
                continue
            set_text(label, text)
            style = "color:%s;" % vesc_status.LEVEL_COLORS[level]
            if label.styleSheet() != style:
                label.setStyleSheet(style)
        command = self._vesc.get("servo_cmd") if vesc_fresh else None
        self.servo.set_command(command)
        if self._vesc and vesc_fresh and finite(command):
            text = (f"명령 {command:.3f} · 목표 {vesc_status._num(self._vesc.get('servo_target'), 3)} · "
                    "제어기가 보낸 서보 명령값입니다(실측 조향각 아님).")
        elif self._vesc:
            text = "vesc_trace가 오래되어 서보 명령을 표시하지 않습니다."
        else:
            text = "미수신 · 2026-11-16 이전 제어기 로그에는 서보 명령값이 없습니다."
        set_text(self.servo_status, text)
