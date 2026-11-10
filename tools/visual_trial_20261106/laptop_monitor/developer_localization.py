"""Read-only laptop localization detail window. No sensor or process control."""
from __future__ import annotations

import math
import time
from collections.abc import Mapping

from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from magnetometer_panel import MagnetometerPanel
from module_logs import attach_module_logs, update_module_logs
from render_utils import HistorySampler, history_signature, fit_axes_3d, set_text, set_style, stable_metadata


SOURCES = ("vio", "fusion", "public")
COLORS = {"vio": "#38bdf8", "fusion": "#fbbf24", "public": "#34d399"}
TEXT = "#e2e8f0"
MUTED = "#94a3b8"
WARNING = "#fbbf24"
STALE = "#fb7185"


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _text(value):
    if value is None or value == "":
        return "—"
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)[:250]


def _flatten(event):
    """Accept direct telemetry or explicit DiagnosticStatus key/value records."""
    if not isinstance(event, Mapping):
        return {}
    result = dict(event)
    for key in ("data", "metrics", "fields"):
        if isinstance(event.get(key), Mapping):
            result.update(event[key])
    values = event.get("values")
    if isinstance(values, Mapping):
        result.update(values)
    elif isinstance(values, (tuple, list)):
        result.update({str(v["key"]): v.get("value") for v in values
                       if isinstance(v, Mapping) and isinstance(v.get("key"), str)})
    diagnostics = event.get("diagnostics", event.get("status"))
    if isinstance(diagnostics, (tuple, list)):
        for status in diagnostics:
            if isinstance(status, Mapping):
                result.update(_flatten(status))
    return result


def _pick(data, *keys):
    for key in keys:
        if key in data and data[key] is not None:
            return data[key]
    return None


def _metric(data, keys, unit="", digits=2):
    value = _pick(data, *keys)
    numeric = _number(value)
    if numeric is not None:
        return f"{numeric:.{digits}f}{unit}"
    return _text(value)


def _age(snapshot, kind, source):
    receipts = snapshot.get("received", {})
    if not isinstance(receipts, Mapping):
        return None
    receipt = receipts.get(f"{kind}:{source}")
    if receipt is None and isinstance(receipts.get(kind), Mapping):
        receipt = receipts[kind].get(source)
    number = _number(receipt)
    now = _number(snapshot.get("now"))
    if number is None:
        return None
    return max(0.0, (time.monotonic() if now is None else now) - number)


def _age_description(age, limit):
    if age is None:
        return "수신 시각 미확인", MUTED
    if age > limit:
        return f"오래됨 · {age:.1f}초 전 수신", STALE
    return f"수신 중 · {age:.1f}초 전", "#34d399"


def _stream_age(event, prefix, fallback):
    """Keep cached Fix and quality ages separate even in a merged GNSS event."""
    values = [_number(event.get(prefix + suffix))
              for suffix in ("_received_age_s", "_age_s")]
    valid = [value for value in values if value is not None and value >= 0.]
    # Ages in the packet are frozen at collector emission; keep advancing them
    # even after that source stops producing callbacks.
    return max(valid) + max(0., fallback or 0.) if valid else fallback


def _telemetry_age(event, fallback):
    measured = _number(event.get("measurement_age_s"))
    if measured is not None and measured >= 0.:
        return measured + max(0., fallback or 0.)
    return fallback


class _ImageFallback(QtWidgets.QWidget):
    def __init__(self, title):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        self.title = QtWidgets.QLabel(title)
        self.display = QtWidgets.QLabel("미수신")
        self.display.setAlignment(QtCore.Qt.AlignCenter)
        self.display.setMinimumSize(220, 150)
        self.display.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.meta = QtWidgets.QLabel("실제 VIO 추적 이미지 대기")
        self.meta.setWordWrap(True)
        self.pixmap = None
        layout.addWidget(self.title)
        layout.addWidget(self.display, 1)
        layout.addWidget(self.meta)

    def set_image(self, image):
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.redraw()

    def redraw(self):
        if self.pixmap is not None:
            self.display.setPixmap(self.pixmap.scaled(
                self.display.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.redraw()


class _Card(QtWidgets.QGroupBox):
    def __init__(self, title, fields):
        super().__init__(title)
        layout = QtWidgets.QGridLayout(self)
        layout.setHorizontalSpacing(18)
        self.receipt = QtWidgets.QLabel("미수신")
        self.receipt.setStyleSheet(f"color:{MUTED}; font-size:11px;")
        layout.addWidget(self.receipt, 0, 0, 1, 2)
        self.values = {}
        for row, (key, label) in enumerate(fields, 1):
            caption = QtWidgets.QLabel(label)
            caption.setStyleSheet(f"color:{MUTED};")
            value = QtWidgets.QLabel("—")
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            value.setWordWrap(True)
            value.setStyleSheet(f"color:{TEXT}; font-weight:600;")
            layout.addWidget(caption, row, 0)
            layout.addWidget(value, row, 1)
            self.values[key] = value
        layout.setColumnStretch(1, 1)

    def set_values(self, values, age, limit, received):
        summary, color = _age_description(age, limit)
        set_text(self.receipt, summary if received else "미수신")
        set_style(self.receipt, f"color:{color if received else MUTED}; font-size:11px;")
        stale = received and age is not None and age > limit
        for key, label in self.values.items():
            set_text(label, _text(values.get(key)) if received else "—")
            set_style(label, f"color:{STALE if stale else TEXT}; font-weight:600;")


class LocalizationWindow(QtWidgets.QMainWindow):
    """A passive detail view updated by the owning monitor's visible-window timer."""

    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.setWindowFlag(QtCore.Qt.Window, True)
        self.setWindowTitle("NOMAD | 위치추정 개발")
        self.resize(1280, 830)
        self.setMinimumSize(880, 600)
        self.setStyleSheet("""
            QMainWindow, QWidget { background:#111827; color:#e2e8f0; }
            QGroupBox { border:1px solid #334155; border-radius:5px;
                        margin-top:12px; padding:12px 9px 7px; font-weight:600; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; }
            QPushButton { background:#1e293b; border:1px solid #475569; padding:5px 12px; }
            QPushButton:hover { background:#334155; }
            QCheckBox { spacing:6px; }
        """)
        self._dragging = False
        self._signature = None
        self._last_image_key = None
        self._last_snapshot = {}
        self._heading_note = ""
        self._samplers = {source: HistorySampler(4000) for source in SOURCES}
        self._source_signatures = {}
        self._fit_initialized = False
        central = QtWidgets.QWidget(self)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(14, 12, 14, 10)
        self.summary = QtWidgets.QLabel("위치추정 데이터 대기")
        self.summary.setStyleSheet("font-size:16px; font-weight:600;")
        root.addWidget(self.summary)
        self.connections = QtWidgets.QLabel("Pi5 SSH: — · Jetson SSH: —")
        self.connections.setStyleSheet(f"color:{MUTED}; font-size:11px;")
        root.addWidget(self.connections)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        plot_box = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_box)
        plot_layout.setContentsMargins(0, 0, 6, 0)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("3D 궤적 · 마우스로 회전"))
        controls.addStretch()
        self.auto_fit = QtWidgets.QCheckBox("자동 범위")
        self.auto_fit.setChecked(True)
        controls.addWidget(self.auto_fit)
        fit = QtWidgets.QPushButton("전체 보기")
        fit.clicked.connect(self.fit_view)
        controls.addWidget(fit)
        reset = QtWidgets.QPushButton("시점 초기화")
        reset.clicked.connect(self.reset_view)
        controls.addWidget(reset)
        plot_layout.addLayout(controls)
        self.figure = Figure(facecolor="#111827")
        self.figure.subplots_adjust(left=.01, right=.98, bottom=.03, top=.98)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111, projection="3d", facecolor="#111827")
        self.ax.view_init(elev=24, azim=-60)
        self.ax.set_box_aspect((1, 1, 1))
        for axis, label in zip((self.ax.xaxis, self.ax.yaxis, self.ax.zaxis), ("X (m)", "Y (m)", "Z (m)")):
            axis.set_pane_color((.065, .09, .14, 1.))
            axis.label.set_color(TEXT)
            axis.set_label_text(label)
            axis._axinfo["grid"]["color"] = "#334155"
        self.ax.tick_params(colors=MUTED, labelsize=8)
        self.lines = {}
        self.markers = {}
        for source in SOURCES:
            self.lines[source], = self.ax.plot([], [], [], color=COLORS[source], lw=1.6, label=source.upper())
            self.markers[source], = self.ax.plot([], [], [], color=COLORS[source], marker="o", ms=4)
        legend = self.ax.legend(loc="upper left", facecolor="#1e293b", edgecolor="#334155", fontsize=9)
        for label in legend.get_texts():
            label.set_color(TEXT)
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-1, 1)
        self.canvas.mpl_connect("button_press_event", self._press)
        self.canvas.mpl_connect("button_release_event", self._release)
        self.canvas.mpl_connect("scroll_event", self._scroll)
        plot_layout.addWidget(self.canvas, 1)
        self.pose_info = QtWidgets.QLabel("VIO —   |   FUSION —   |   PUBLIC —")
        self.pose_info.setWordWrap(True)
        self.pose_info.setStyleSheet(f"color:{MUTED}; font-size:11px;")
        plot_layout.addWidget(self.pose_info)
        self.frame_info = QtWidgets.QLabel("좌표계 미수신")
        self.frame_info.setWordWrap(True)
        self.frame_info.setStyleSheet(f"color:{MUTED}; font-size:11px;")
        plot_layout.addWidget(self.frame_info)
        splitter.addWidget(plot_box)
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(4, 0, 0, 0)
        image_type = (components or {}).get("ImagePanel", _ImageFallback)
        self.tracking = image_type("VIO 실제 추적 이미지")
        self.tracking.display.setText("미수신 · 실제 VIO 추적 이미지 대기")
        self.tracking.meta.setWordWrap(True)
        stable_metadata(self.tracking.meta)
        right_layout.addWidget(self.tracking, 5)
        self.features = QtWidgets.QLabel("추적 특징점: —")
        self.features.setStyleSheet("font-size:14px; font-weight:600; color:#38bdf8;")
        right_layout.addWidget(self.features)
        self.magnetometer = MagnetometerPanel(self)
        right_layout.addWidget(self.magnetometer)
        cards = QtWidgets.QHBoxLayout()
        self.gnss = _Card("GNSS", (("fix", "Fix"), ("solution", "측위 상태"), ("satellites", "위성 수"),
                                  ("h_accuracy", "수평 표준편차"), ("v_accuracy", "수직 정확도"),
                                  ("rate", "갱신 주기")))
        self.fusion = _Card("융합", (("mode", "모드"), ("source", "사용 입력"),
                                   ("accepted", "GNSS 수용"), ("optimization", "최적화"),
                                   ("solve_time", "최근 계산 시간")))
        cards.addWidget(self.gnss)
        cards.addWidget(self.fusion)
        right_layout.addLayout(cards, 3)
        self.detail = QtWidgets.QLabel("융합 상태 상세: —")
        self.detail.setWordWrap(True)
        self.detail.setStyleSheet(f"color:{MUTED}; font-size:11px;")
        right_layout.addWidget(self.detail)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 6)
        splitter.setStretchFactor(1, 5)
        splitter.setSizes([650, 530])
        root.addWidget(splitter, 1)
        note = QtWidgets.QLabel("실제 수신 데이터만 표시 · 미수신 항목 — · 이 창을 닫아도 센서·시험은 계속 실행됩니다.")
        note.setStyleSheet(f"color:{MUTED}; font-size:11px;")
        root.addWidget(note)
        self.setCentralWidget(central)
        attach_module_logs(self)

    def append_module_log(self, event):
        return self.log_panel.append_event(event)

    def _press(self, event):
        if event.inaxes is self.ax:
            self._dragging = True

    def _release(self, event):
        self._dragging = False

    def _scroll(self, event):
        if event.inaxes is self.ax:
            self.auto_fit.setChecked(False)

    def reset_view(self):
        self.ax.view_init(elev=24, azim=-60)
        self.canvas.draw_idle()

    def fit_view(self):
        self._fit(self._last_snapshot.get("histories", {}), force=True)
        self.canvas.draw_idle()

    def _fit(self, histories, force=False):
        bounds = []
        for source in SOURCES:
            history = histories.get(source, ())
            bounds.extend(p for p in self._samplers[source].sample(history)
                          if all(_number(p.get(k)) is not None for k in ("x", "y", "z")))
        self._fit_initialized = fit_axes_3d(self.ax, bounds, force=force, initialized=self._fit_initialized)

    def _plot(self, snapshot):
        histories = snapshot.get("histories", {})
        signature, details, frames = [], [], []
        for source in SOURCES:
            history = histories.get(source, ())
            age = _age(snapshot, "pose", source)
            stale = age is not None and age > 1.
            signature.append((history_signature(history), stale))
            freshness = "미수신" if not history else _age_description(age, 1.)[0]
            rate = self._rate(snapshot, source)
            details.append(f"{source.upper()} {len(history):,}개 · {rate} · {freshness}")
            if history:
                frames.append(f"{source.upper()}: {history[-1].get('frame_id') or '미확인'}")
        self.pose_info.setText("\n".join(details))
        frame_ids = {histories[source][-1].get("frame_id") for source in SOURCES if histories.get(source)}
        note = "서로 다른 좌표계 · 정렬 없이 수신값 표시" if len(frame_ids) > 1 else "수신 좌표 그대로 표시"
        self.frame_info.setText(note + "\n" + " · ".join(frames))
        set_style(self.frame_info, f"color:{WARNING if len(frame_ids) > 1 else MUTED}; font-size:11px;")
        signature = tuple(signature)
        if signature == self._signature or self._dragging:
            return
        self._signature = signature
        for source in SOURCES:
            history = histories.get(source, ())
            source_key = (history_signature(history), signature[SOURCES.index(source)][1])
            if self._source_signatures.get(source) == source_key:
                continue
            self._source_signatures[source] = source_key
            points = self._samplers[source].sample(history)
            points = [p for p in points if all(_number(p.get(k)) is not None for k in ("x", "y", "z"))]
            self.lines[source].set_data_3d(*[[float(p[k]) for p in points] for k in ("x", "y", "z")])
            self.markers[source].set_data_3d(*[[float(points[-1][k])] if points else [] for k in ("x", "y", "z")])
            age = _age(snapshot, "pose", source)
            stale = age is not None and age > 1.
            self.lines[source].set_alpha(.4 if stale else 1.)
            self.lines[source].set_linestyle("--" if stale else "-")
        if self.auto_fit.isChecked():
            self._fit(histories)
        # Only the artists/limits change. User-selected camera elevation/azimuth stay intact.
        self.canvas.draw_idle()

    @staticmethod
    def _rate(snapshot, source):
        for board in ("pi", "pi5", "jetson"):
            rates = snapshot.get("rates", {}).get(board, {})
            value = _number(rates.get(source)) if isinstance(rates, Mapping) else None
            if value is not None:
                return f"{value:.1f} Hz"
        return "— Hz"

    @QtCore.pyqtSlot(dict)
    def update_snapshot(self, snapshot):
        update_module_logs(self, snapshot, "localization")
        self._last_snapshot = snapshot
        self._plot(snapshot)
        statuses = snapshot.get("statuses", {})
        state = statuses.get("localization", {})
        status_age = _age(snapshot, "status", "localization")
        if isinstance(state, Mapping) and state:
            self.summary.setText(f"위치추정 · {_text(state.get('state'))} · {_age_description(status_age, 2.)[0]}")
            self.summary.setToolTip(str(state.get("detail", "")))
        else:
            self.summary.setText("위치추정 · 상태 미수신")
        set_style(self.summary, "font-size:16px; font-weight:600; color:" +
                  (STALE if status_age is not None and status_age > 2. else TEXT) + ";")
        ssh = snapshot.get("ssh", {})
        connections = []
        for board, title in (("pi", "Pi5"), ("jetson", "Jetson")):
            value = ssh.get(board, {})
            connections.append(f"{title} SSH: {_text(value.get('phase')) if isinstance(value, Mapping) else _text(value)}")
        self.connections.setText(" · ".join(connections))
        telemetry = snapshot.get("telemetry", {})
        self.magnetometer.set_data(telemetry.get("magnetometer", {}),
                                   _age(snapshot, "telemetry", "magnetometer"))
        gnss = _flatten(telemetry.get("gnss", {}))
        fusion = _flatten(telemetry.get("fusion", {}))
        vio = _flatten(telemetry.get("vio_features", {}))
        gnss_received = bool(gnss) and gnss.get("received") is not False
        fix_received = gnss_received and gnss.get("fix_received") is not False
        quality_received = gnss_received and gnss.get("quality_received") is not False
        gnss_elapsed = _age(snapshot, "telemetry", "gnss")
        gnss_age = _telemetry_age(gnss, gnss_elapsed)
        fix_age = _stream_age(gnss, "fix", gnss_elapsed) if fix_received else None
        quality_age = _stream_age(gnss, "quality", gnss_elapsed) if quality_received else None
        fix = _pick(gnss, "fix_label", "fix_type_label", "fix")
        navsat = _pick(gnss, "fix_status", "navsat_status", "navsatfix_status")
        if navsat is None and isinstance(gnss.get("status"), Mapping):
            navsat = gnss["status"].get("status")
        if fix is None and navsat is not None:
            fix = {-1: "NO_FIX", 0: "FIX", 1: "SBAS_FIX", 2: "GBAS_FIX"}.get(_number(navsat), f"status {navsat}")
        if not fix_received:
            fix = None
        elif gnss.get("fix_valid") is False:
            fix = "위치 미유효" + (f" · {fix}" if fix is not None else "")
        if fix is not None and fix_age is not None and fix_age > 2.5:
            fix = f"{fix} · 오래됨 {fix_age:.1f}초"
        horizontal = _metric(gnss, ("h_acc_m", "horizontal_accuracy_m", "horizontal_std_m", "h_accuracy_m"), " m")
        vertical = _metric(gnss, ("v_acc_m", "vertical_accuracy_m", "vertical_std_m", "v_accuracy_m"), " m")
        # A generic h_acc/v_acc has no guaranteed unit (receiver raw fields can be mm).
        if horizontal == "—" and _pick(gnss, "h_acc", "hAcc") is not None:
            horizontal = _text(_pick(gnss, "h_acc", "hAcc")) + " (단위 미확인)"
        if vertical == "—" and _pick(gnss, "v_acc", "vAcc") is not None:
            vertical = _text(_pick(gnss, "v_acc", "vAcc")) + " (단위 미확인)"
        if not fix_received:
            horizontal = vertical = "—"
        rate = _metric(gnss, ("rate_hz", "hz"), " Hz", 1)
        if _number(gnss.get("fix_hz")) is not None or _number(gnss.get("quality_hz")) is not None:
            rate = ("Fix " + _metric(gnss, ("fix_hz",), " Hz", 1) +
                    " · 품질 " + _metric(gnss, ("quality_hz",), " Hz", 1))
        satellites = _pick(gnss, "satellites", "satellites_used", "num_sv", "numSV")
        if not quality_received or gnss.get("satellites_valid") is False:
            satellites = None
        elif satellites is not None and quality_age is not None and quality_age > 2.5:
            satellites = f"{satellites} · 오래됨 {quality_age:.1f}초"
        solution = None
        if quality_received:
            solution_type = _number(gnss.get("fix_type"))
            quality_flags = _number(gnss.get("quality_valid_fields"))
            if quality_flags is None or int(quality_flags) & 1:
                solution = {0: "UNKNOWN", 1: "NO_FIX", 2: "SINGLE", 3: "DGNSS",
                            4: "RTK_FLOAT", 5: "RTK_FIXED"}.get(solution_type)
            if solution is not None and quality_age is not None and quality_age > 2.5:
                solution += f" · 오래됨 {quality_age:.1f}초"
        self.gnss.set_values({"fix": fix, "solution": solution, "satellites": satellites,
                              "h_accuracy": horizontal, "v_accuracy": vertical,
                              "rate": self._rate(snapshot, "gnss") if rate == "—" else rate},
                             gnss_age, 2.5, gnss_received)
        if gnss_received:
            fix_text = _age_description(fix_age, 2.5)[0] if fix_received else "미수신"
            quality_text = _age_description(quality_age, 2.5)[0] if quality_received else "미수신"
            self.gnss.receipt.setText(f"Fix: {fix_text}\n품질: {quality_text}")
            stale = any(age is not None and age > 2.5 for age in (fix_age, quality_age))
            set_style(self.gnss.receipt, f"color:{STALE if stale else MUTED}; font-size:11px;")
        accepted = _pick(fusion, "gnss_accepted")
        count = _pick(fusion, "gps_accepted", "accepted_gnss_count")
        if accepted is None and count is not None:
            accepted = f"누적 {_text(count)}건"
        elif isinstance(accepted, bool):
            accepted = "수용" if accepted else "미수용"
        elif _number(accepted) is not None:
            accepted = f"누적 {_text(accepted)}건"
        rejected = _pick(fusion, "gnss_rejected", "rejected_gps")
        if accepted is not None and rejected is not None:
            accepted += f" · 거부 {_text(rejected)}건"
        optimization = _pick(fusion, "optimization_state", "optimizer_state", "solve_state")
        if optimization == "not_reported":
            optimization = "진행 상태 미제공"
            if fusion.get("solves") is not None:
                optimization += f" · 누적 {_text(fusion['solves'])}회"
                if fusion.get("discarded_solves") is not None:
                    optimization += f" · 미적용 {_text(fusion['discarded_solves'])}회"
        elif optimization is None and fusion.get("solves") is not None:
            optimization = f"누적 {_text(fusion['solves'])}회"
            if fusion.get("discarded_solves") is not None:
                optimization += f" · 미적용 {_text(fusion['discarded_solves'])}회"
        self.fusion.set_values({"mode": _pick(fusion, "mode", "state"), "source": _pick(fusion, "source", "active_source"),
                                "accepted": accepted, "optimization": optimization,
                                "solve_time": _metric(fusion, ("solve_ms", "optimization_ms"), " ms", 1)},
                               _telemetry_age(fusion, _age(snapshot, "telemetry", "fusion")), 2.,
                               bool(fusion) and fusion.get("received") is not False)
        reasons = [_text(fusion[k]) for k in ("reason", "heading_reason") if fusion.get(k)]
        self.detail.setText("융합 상태 상세: " + (" · ".join(reasons) if reasons else "—"))
        self.detail.setToolTip("\n".join(f"{k}: {v}" for k, v in fusion.items()
                                       if not isinstance(v, (dict, list))))
        count = _pick(vio, "feature_count", "tracked_features", "tracked_feature_count", "num_features", "n_features", "count")
        image = snapshot.get("images", {}).get("vio_tracking")
        meta = snapshot.get("image_meta", {}).get("vio_tracking", {})
        if count is None and isinstance(meta, Mapping):
            count = _pick(meta, "feature_count", "tracked_features")
        feature_age = _telemetry_age(vio, _age(snapshot, "telemetry", "vio_features"))
        if vio.get("received") is False:
            count = None
        freshness = _age_description(feature_age, 2.)[0] if vio else "미수신"
        detail = ""
        if count is not None:
            tracked, added = _pick(vio, "tracked_count"), _pick(vio, "new_count")
            if tracked is not None or added is not None:
                detail = f" · 유지 {_text(tracked)} / 신규 {_text(added)}"
        self.features.setText(f"추적 특징점: {_text(count)}" + (f"개{detail}\n{freshness}" if count is not None else " · 미수신"))
        set_style(self.features, "font-size:14px; font-weight:600; color:" +
                  (STALE if feature_age is not None and feature_age > 2. else "#38bdf8") + ";")
        image_age = _telemetry_age(meta, _age(snapshot, "image", "vio_tracking")) if isinstance(meta, Mapping) else _age(snapshot, "image", "vio_tracking")
        if isinstance(image, QtGui.QImage) and not image.isNull():
            key = image.cacheKey()
            if key != self._last_image_key:
                self.tracking.set_image(image)
                self._last_image_key = key
            freshness, color = _age_description(image_age, 1.5)
            invalid = isinstance(meta, Mapping) and meta.get("decode_error")
            if invalid:
                freshness, color = "새 이미지 해석 실패 · 마지막 정상 이미지 유지 · " + freshness, STALE
            self.tracking.meta.setText(f"실제 VIO 추적 이미지 · {image.width()}×{image.height()} · {freshness}")
            set_style(self.tracking.meta, f"color:{color}; font-size:11px;")
        else:
            self.tracking.meta.setText("미수신 · 원본 VIO 추적 이미지가 전달되면 표시됩니다.")
            set_style(self.tracking.meta, f"color:{MUTED}; font-size:11px;")


if __name__ == "__main__":
    import sys
    application = QtWidgets.QApplication(sys.argv)
    window = LocalizationWindow()
    window.show()
    window.update_snapshot({"now": time.monotonic()})
    if "--smoke" in sys.argv:
        QtCore.QTimer.singleShot(150, application.quit)
    sys.exit(application.exec_())
