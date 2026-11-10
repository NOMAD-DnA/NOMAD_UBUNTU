"""Perception developer window. Reads snapshots; never opens sensors or SSH."""
import math
import time
from PyQt5 import QtGui, QtWidgets
from module_logs import attach_module_logs, update_module_logs
from render_utils import stable_metadata


# RGB equivalents of monitor_probe.cpp's field12 preview palette.
FIELD12 = (
    ("ignore / 미확정", "#646464"), ("사람", "#f04646"), ("차량", "#3c6ef0"),
    ("목재", "#96552d"), ("흙", "#b47846"), ("자갈", "#bebebe"),
    ("포장면", "#5a697d"), ("암석", "#aa5ad2"), ("낙엽", "#f5a523"),
    ("식생", "#32af4b"), ("인공물", "#f5961e"), ("하늘", "#32afff"),
)
STYLE = "QWidget {background:#111827;color:#e2e8f0;} QLabel {font-size:12px;}"


def age(snapshot, key):
    received = snapshot.get("received", {}).get(key)
    now = snapshot.get("now", time.monotonic())
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (received, now)):
        return None
    return max(0., now - received)


def age_text(value):
    return "미수신" if value is None else f"수신 후 {value:.1f}초" + (" · 오래됨" if value > 2. else "")


def hz_text(value):
    return f"{value:.1f}Hz" if isinstance(value, (int, float)) and math.isfinite(value) and value >= 0 else "—"


class PerceptionWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.setWindowTitle("NOMAD | 인지 개발")
        self.resize(1180, 700)
        self.setStyleSheet(STYLE)
        self.panels, self._image_keys, self._image_received, self._image_meta = {}, {}, {}, {}
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        self.status = QtWidgets.QLabel("인지: 미수신")
        self.model = QtWidgets.QLabel("모델 선택 확인: — | 분할 영상: 미수신")
        self.rate = QtWidgets.QLabel("원본 — | 분할 —")
        for label in (self.status, self.model, self.rate):
            label.setWordWrap(True)
            layout.addWidget(label)
        cpu_box = QtWidgets.QGroupBox("경사 계산 CPU")
        cpu_layout = QtWidgets.QGridLayout(cpu_box)
        for col, title in enumerate(("방식", "CPU 사용량", "측정 상태")):
            cpu_layout.addWidget(QtWidgets.QLabel(title), 0, col)
        self.slope_cpu_values, self.slope_cpu_states = {}, {}
        for row, (mode, title) in enumerate((("sequential", "기존 방식 (1스레드)"),
                                              ("parallel", "병렬 방식 (3스레드)")), 1):
            cpu_layout.addWidget(QtWidgets.QLabel(title), row, 0)
            self.slope_cpu_values[mode] = QtWidgets.QLabel("—")
            self.slope_cpu_states[mode] = QtWidgets.QLabel("미확인")
            cpu_layout.addWidget(self.slope_cpu_values[mode], row, 1)
            cpu_layout.addWidget(self.slope_cpu_states[mode], row, 2)
        self.slope_cpu_note = QtWidgets.QLabel("100% = CPU 한 코어 전체. 경사 worker의 모든 스레드를 합산하며 약1초 간격으로 갱신합니다.")
        self.slope_cpu_note.setWordWrap(True)
        cpu_layout.addWidget(self.slope_cpu_note, 3, 0, 1, 3)
        self.slope_cpu_status = QtWidgets.QLabel("CPU 측정 미수신")
        self.slope_cpu_status.setWordWrap(True)
        cpu_layout.addWidget(self.slope_cpu_status, 4, 0, 1, 3)
        layout.addWidget(cpu_box)
        images = QtWidgets.QHBoxLayout()
        panel_type = (components or {}).get("ImagePanel")
        for source, title in (("rgb", "원본 영상 · 현재 흑백 미리보기"), ("semantic", "환경 의미 분할 · field12")):
            panel = panel_type(title) if panel_type else QtWidgets.QLabel(title + "\nImagePanel 구성요소 미연결")
            if panel_type:
                self.panels[source] = panel
                panel.meta.setWordWrap(True)
                stable_metadata(panel.meta)
            images.addWidget(panel, 1)
        layout.addLayout(images, 1)
        legend = QtWidgets.QGridLayout()
        for index, (name, color) in enumerate(FIELD12):
            label = QtWidgets.QLabel(f'<span style="color:{color}">■</span> {index} {name}')
            legend.addWidget(label, index // 4, index % 4)
        layout.addLayout(legend)
        footer = QtWidgets.QLabel("모델 선택 확인과 새 분할 영상의 수신 여부는 별도입니다. 0은 미확정이며 주행 가능 판정이 아닙니다.")
        footer.setWordWrap(True)
        footer.setStyleSheet("color:#94a3b8;")
        layout.addWidget(footer)
        self.setCentralWidget(body)
        attach_module_logs(self)

    def append_module_log(self, event):
        return self.log_panel.append_event(event)

    def update_snapshot(self, snapshot):
        update_module_logs(self, snapshot, "perception")
        self.update_slope_cpu(snapshot)
        status = snapshot.get("statuses", {}).get("perception", {})
        self.status.setText("인지: " + str(status.get("state", "미수신")) + " | " +
                            age_text(age(snapshot, "status:perception")))
        self.status.setToolTip(str(status.get("detail", "")))
        selected = {"b2": "B2", "fcn": "FCN"}.get(snapshot.get("active_model"), "—")
        rates = snapshot.get("rates", {}).get("jetson", {})
        self.rate.setText("원본 " + hz_text(rates.get("rgb")) + " | 분할 " + hz_text(rates.get("semantic")) +
                          " | Hz 보고: " + age_text(age(snapshot, "rates:jetson")))
        for source, panel in self.panels.items():
            image = snapshot.get("images", {}).get(source)
            event = snapshot.get("image_meta", {}).get(source, {})
            if isinstance(image, QtGui.QImage) and not image.isNull() and not event.get("decode_error"):
                key = image.cacheKey()
                if self._image_keys.get(source) != key:
                    panel.set_image(image)
                    self._image_keys[source] = key
                self._image_received[source] = snapshot.get("received", {}).get("image:" + source)
                self._image_meta[source] = event
            shown = self._image_meta.get(source, {})
            dimensions = f"{shown['width']}×{shown['height']}" if shown.get("width") and shown.get("height") else "—"
            prefix = "새 영상 오류 · 마지막 정상 영상 유지 | " if event.get("decode_error") else ""
            panel.meta.setText(prefix + age_text(self._shown_age(snapshot, source)) + " | " + dimensions)
        self.model.setText("모델 선택 확인: " + selected + " | 분할 영상: " +
                           age_text(self._shown_age(snapshot, "semantic")) +
                           (" · 새 영상 오류" if snapshot.get("image_meta", {}).get("semantic", {}).get("decode_error") else ""))

    def update_slope_cpu(self, snapshot):
        event = snapshot.get("telemetry", {}).get("slope_cpu", {})
        received_age = age(snapshot, "telemetry:slope_cpu")
        fresh = received_age is not None and received_age <= 3.
        active = event.get("active_mode") if fresh and event.get("state") == "live" else None
        cpu = event.get("cpu", {})
        history = snapshot.get("slope_cpu_history", {})
        for mode in ("sequential", "parallel"):
            value, label = "—", "미확인"
            if active == mode:
                percent = cpu.get("percent_one_core")
                if cpu.get("state") == "live" and isinstance(percent, (int, float)) and math.isfinite(percent):
                    value, label = f"{percent:.1f}%", "실시간 측정"
                else:
                    label = "실행 중 / 측정 준비"
            elif active is not None or (fresh and event.get("state") == "not_running"):
                label = "미실행"
            elif not fresh:
                label = "연결 상태 미확인"
            previous = history.get(mode, {})
            if active != mode and isinstance(previous.get("percent_one_core"), (int, float)):
                value = f"최근 {previous['percent_one_core']:.1f}%"
                since = max(0., snapshot.get("now", time.monotonic()) - previous.get("received", 0.))
                label += f" / {since:.0f}초 전 측정"
            self.slope_cpu_values[mode].setText(value)
            self.slope_cpu_states[mode].setText(label)
        worker = event.get("worker") or {}
        if not fresh:
            text = "CPU 연결 미수신 또는 오래됨. 이전 값은 실시간 사용량이 아닙니다."
        elif event.get("state") == "not_running":
            text = "경사 worker 미실행. 두 방식을 동시에 실행하지 않습니다."
        elif active is None:
            text = "경사 worker 상태 미확인: " + str(event.get("state", "unknown"))
        else:
            text = f"worker PID {worker.get('pid', '—')} | 계산 스레드 설정 {worker.get('threads', '—')} | 수신 후 {received_age:.1f}초"
        self.slope_cpu_status.setText(text)

    def _shown_age(self, snapshot, source):
        received = self._image_received.get(source)
        return None if received is None else max(0., snapshot.get("now", time.monotonic()) - received)
