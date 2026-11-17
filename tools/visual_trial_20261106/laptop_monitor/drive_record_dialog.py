"""2026-11-15 '조종·녹화' 창 (초기 화면 '조종·녹화' 버튼, 주행 화면 '조종·녹화' 버튼에서 연다).

위: 키보드 조종(누름 유지형, keyboard_drive) · 아래: 센서 녹화(Jetson SSD, Pi5 GPS 포함 선택, sensor_record).
조종 중에는 이 창이 키를 가져간다(W/↑ 전진, S/↓ 후진, A/D·←/→ 조향, Space/Esc STOP). 창이 초점을 잃으면 조종 세션을 닫는다.
녹화는 창을 닫아도 Jetson에서 계속된다(다시 열어 '녹화 정지').
"""
from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

import keyboard_drive
import sensor_record
import vesc_status

FORWARD = {QtCore.Qt.Key_W, QtCore.Qt.Key_Up}
LEFT = {QtCore.Qt.Key_A, QtCore.Qt.Key_Left}
RIGHT = {QtCore.Qt.Key_D, QtCore.Qt.Key_Right}
BACK = {QtCore.Qt.Key_S, QtCore.Qt.Key_Down}
STOP = {QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape}
DRIVE_KEYS = FORWARD | BACK | LEFT | RIGHT | STOP


class _Task(QtCore.QThread):
    result = QtCore.pyqtSignal(object)

    def __init__(self, function, parent=None):
        super().__init__(parent)
        self.function = function

    def run(self):
        try:
            value = self.function()
        except Exception as error:  # never let a worker exception reach Qt
            value = {"ok": False, "reason": "local_control_error", "message": str(error)[:120]}
        self.result.emit(value)


def _number(value, digits=1):
    return f"{value:.{digits}f}" if isinstance(value, (int, float)) else "—"


class DriveRecordDialog(QtWidgets.QDialog):
    def __init__(self, pi4, jetson, pi5, password, parent=None):
        super().__init__(parent)
        self.setWindowTitle("조종·녹화 — 키보드 조종(Pi4) · 센서 녹화(Jetson)")
        self.resize(760, 860)
        self.pi4, self.jetson, self.pi5, self.password = pi4, jetson, pi5, password or None
        self.tunnel = self.client = None
        self.tasks = set()
        self.pressed = set()
        self.recording = False
        self.gui_sensors = False  # 2026-11-16: sensors a GUI recording started are still on
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel(f"Pi4: {pi4 or '—'}   |   Jetson: {jetson or '—'}   |   Pi5: {pi5 or '—'}"))

        drive = QtWidgets.QGroupBox("키보드 조종 (Pi4 제어기 · 누르는 동안만 움직임)")
        grid = QtWidgets.QGridLayout(drive)
        self.connect_button = QtWidgets.QPushButton("Pi4 연결")
        self.connect_button.setToolTip("Pi4 제어기 수동 조종 서비스를 조종 가능 모드로 켜고 연결합니다. 차는 움직이지 않습니다.")
        self.connect_button.clicked.connect(self.connect_pi4)
        self.disconnect_button = QtWidgets.QPushButton("연결 끊기")
        self.disconnect_button.clicked.connect(self.disconnect_pi4)
        self.erpm = QtWidgets.QSpinBox()
        self.erpm.setRange(200, 3000)
        self.erpm.setSingleStep(50)
        self.erpm.setValue(1000)
        self.erpm.setSuffix(" ERPM")
        self.erpm.setToolTip("W/↑를 누르는 동안의 속도. 1000 ERPM ≈ 0.27 m/s(팀원 영상 환산, 추정). 제어기 최대값을 넘지 않습니다.")
        self.reverse = QtWidgets.QSpinBox()
        self.reverse.setRange(200, 1500)
        self.reverse.setSingleStep(50)
        self.reverse.setValue(600)
        self.reverse.setSuffix(" ERPM")
        self.reverse.setToolTip("S/↓를 누르는 동안의 후진 속도. Pi4 제어기의 후진 최대값(manual_reverse_max_erpm)을 넘지 않습니다. "
                                "전진↔후진은 제어기가 먼저 멈춘 뒤 바꿉니다.")
        self.steer = QtWidgets.QDoubleSpinBox()
        self.steer.setRange(0.1, 1.0)
        self.steer.setSingleStep(0.1)
        self.steer.setValue(0.7)
        self.steer.setToolTip("A/D를 누를 때 꺾는 정도(가운데→끝 비율). 떼면 가운데로 돌아갑니다.")
        self.steer_invert = QtWidgets.QCheckBox("조향 반대(뒷바퀴 조향)")
        self.steer_invert.setChecked(True)
        self.steer_invert.setToolTip("켜면 A/←를 눌렀을 때 서보를 반대로 보내 차가 왼쪽으로 돕니다(뒷바퀴 조향). "
                                     "2026-11-15 기본 켬. 팀원 Pi4 웹 GUI의 A=서보 감소와 반대입니다.")
        self.steer_invert.toggled.connect(lambda value: self.client and self.client.set_steer_invert(value))
        self.start_button = QtWidgets.QPushButton("조종 시작")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start_drive)
        self.stop_button = QtWidgets.QPushButton("STOP (Space)")
        self.stop_button.setStyleSheet("QPushButton {background:#991b1b;color:white;font-weight:700;padding:8px 18px;}")
        self.stop_button.clicked.connect(self.stop_drive)
        grid.addWidget(self.connect_button, 0, 0)
        grid.addWidget(self.disconnect_button, 0, 1)
        grid.addWidget(QtWidgets.QLabel("전진 속도"), 0, 2)
        grid.addWidget(self.erpm, 0, 3)
        grid.addWidget(QtWidgets.QLabel("후진 속도"), 0, 4)
        grid.addWidget(self.reverse, 0, 5)
        grid.addWidget(QtWidgets.QLabel("조향 폭"), 0, 6)
        grid.addWidget(self.steer, 0, 7)
        grid.addWidget(self.steer_invert, 1, 0, 1, 2)
        grid.addWidget(self.start_button, 1, 2, 1, 3)
        grid.addWidget(self.stop_button, 1, 5, 1, 3)
        self.keys_label = QtWidgets.QLabel("")
        self.keys_label.setStyleSheet("font-size:18px;font-weight:700;padding:4px;")
        grid.addWidget(self.keys_label, 2, 0, 1, 8)
        self.drive_status = QtWidgets.QLabel("Pi4 연결 전")
        self.drive_status.setWordWrap(True)
        grid.addWidget(self.drive_status, 3, 0, 1, 8)
        self.drive_message = QtWidgets.QLabel("")
        self.drive_message.setWordWrap(True)
        self.drive_message.setStyleSheet("color:#f59e0b;")
        grid.addWidget(self.drive_message, 4, 0, 1, 8)
        note = QtWidgets.QLabel("W/↑ 누르는 동안 전진 · S/↓ 누르는 동안 후진(Pi4가 후진을 허용할 때) · 떼면 속도 0 → 제어기가 브레이크 후 "
                                "핸드브레이크로 정지 유지(경사 정차 보장 아님) · 전진↔후진은 먼저 멈춘 뒤 바뀜 · "
                                "A/D·←/→ 누르는 동안 조향 · Space/Esc 즉시 STOP · 창 초점을 잃으면 조종 해제. "
                                "Pi4 제어기의 수동 조종 모드를 씁니다(팀원 표시는 '바퀴 띄운 시험용', 지상 사용은 2026-11-15 사용자 결정).")
        note.setWordWrap(True)
        note.setObjectName("muted")
        grid.addWidget(note, 5, 0, 1, 8)
        layout.addWidget(drive)
        self._build_vesc_panel(layout)

        record = QtWidgets.QGroupBox("센서 녹화 (Jetson SSD /mnt/ssd/bags · VIO·지도 없이 원본)")
        rgrid = QtWidgets.QGridLayout(record)
        self.scene = QtWidgets.QLineEdit("주행")
        self.scene.setMaxLength(40)
        self.scene.setToolTip("폴더 이름: <날짜_시각>_<장면>. 한글·영문·숫자·_- 만.")
        self.gps = QtWidgets.QCheckBox("Pi5 GPS 포함 (/gps/fix · /gps/quality)")
        self.gps.setChecked(True)
        self.record_start = QtWidgets.QPushButton("녹화 시작")
        self.record_start.setObjectName("primary")
        self.record_start.clicked.connect(self.start_recording)
        self.record_stop = QtWidgets.QPushButton("녹화 정지")
        self.record_stop.clicked.connect(self.stop_recording)
        self.record_refresh = QtWidgets.QPushButton("상태 보기")
        self.record_refresh.clicked.connect(self.refresh_recording)
        rgrid.addWidget(QtWidgets.QLabel("장면 이름"), 0, 0)
        rgrid.addWidget(self.scene, 0, 1)
        rgrid.addWidget(self.gps, 0, 2, 1, 2)
        rgrid.addWidget(self.record_start, 1, 0, 1, 2)
        rgrid.addWidget(self.record_stop, 1, 2)
        rgrid.addWidget(self.record_refresh, 1, 3)
        self.record_status = QtWidgets.QLabel("녹화 상태: 확인 전")
        self.record_status.setWordWrap(True)
        rgrid.addWidget(self.record_status, 2, 0, 1, 4)
        rnote = QtWidgets.QLabel("시작 전 확인: Jetson 시계가 Pi5에 맞춰져 있어야 합니다(차 5 ms 이하). 센서가 꺼져 있으면 녹화가 켜고, "
                                 "정지 때 녹화가 켠 센서·GPS만 끕니다(시험이 켠 것은 그대로). 창을 닫아도 녹화는 계속됩니다.")
        rnote.setWordWrap(True)
        rnote.setObjectName("muted")
        rgrid.addWidget(rnote, 3, 0, 1, 4)
        layout.addWidget(record)
        layout.addStretch(1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_drive)
        self.timer.start(100)
        self.record_timer = QtCore.QTimer(self)
        self.record_timer.timeout.connect(lambda: self.refresh_recording(quiet=True))
        self.record_timer.start(4000)
        self.update_buttons()
        self.refresh_recording(quiet=True)

    # ---- helpers -------------------------------------------------------------
    def run_task(self, function, done):
        task = _Task(function, self)
        self.tasks.add(task)

        def finished(value, task=task):
            self.tasks.discard(task)
            done(value)
        task.result.connect(finished)
        task.start()

    def driving(self):
        return bool(self.client and self.client.snapshot()["active"])

    def update_buttons(self):
        connected = self.client is not None
        self.connect_button.setEnabled(not connected and bool(self.pi4) and "connect" not in self.busy())
        self.disconnect_button.setEnabled(connected)
        self.start_button.setEnabled(connected and not self.driving())
        self.stop_button.setEnabled(connected)
        rec_busy = "record" in self.busy()
        self.record_start.setEnabled(bool(self.jetson) and not self.recording and not rec_busy)
        self.record_stop.setEnabled(bool(self.jetson) and (self.recording or self.gui_sensors) and not rec_busy)

    def busy(self):
        return {getattr(task, "kind", "") for task in self.tasks}

    # ---- drive -----------------------------------------------------------------
    def connect_pi4(self):
        self.drive_status.setText("Pi4 조종 서비스 준비 중… (VESC 설정 확인 포함, 최대 1분 · 차는 움직이지 않음)")
        pi4, password = self.pi4, self.password

        def work():
            ready = keyboard_drive.prepare(pi4, password)
            if not ready.get("ok"):
                return dict(ready, step="prepare")
            tunnel = keyboard_drive.Tunnel(pi4, password)
            if not tunnel.open():
                return dict(ok=False, reason="tunnel_failed", step="tunnel")
            return dict(ok=True, reason=ready.get("reason"), message=ready.get("message", ""), tunnel=tunnel)

        def done(value):
            if not value.get("ok"):
                self.drive_status.setText("Pi4 연결 실패: " + str(value.get("reason")) + " · " + str(value.get("message", "")))
            else:
                self.tunnel = value["tunnel"]
                self.client = keyboard_drive.DriveClient(keyboard_drive.HttpTransport(self.tunnel.port))
                self.client.set_limits(self.erpm.value(), self.steer.value(), self.reverse.value())
                self.client.set_steer_invert(self.steer_invert.isChecked())
                self.client.start_loop()
                if value.get("reason") == "session_active":
                    self.drive_message.setText("다른 조종 화면이 이미 조종 중입니다. 그쪽에서 STOP한 뒤 시작하세요.")
            self.update_buttons()
        self.run_task(work, done)
        self.tasks_kind("connect")
        self.update_buttons()

    def tasks_kind(self, kind):
        for task in self.tasks:
            if not getattr(task, "kind", ""):
                task.kind = kind

    def disconnect_pi4(self):
        if self.client:
            self.client.request_stop()
            QtCore.QTimer.singleShot(400, self._shutdown_client)
        else:
            self._shutdown_client()

    def _shutdown_client(self):
        client, tunnel, self.client, self.tunnel = self.client, self.tunnel, None, None
        if client:
            client.shutdown()
        if tunnel:
            tunnel.close()
        self.pressed.clear()
        self.update_buttons()
        if client is None and tunnel is None:
            return
        # 2026-11-15 사용자 요청: 연결 끊기 = STOP 후 Pi4 수동 조종 서비스까지 끈다.
        self.drive_status.setText("Pi4 연결 끊는 중… 조종 서비스 끄는 중")
        pi4, password = self.pi4, self.password
        self.run_task(lambda: keyboard_drive.stop_service(pi4, password),
                      lambda value: self.drive_status.setText(
                          "Pi4 연결 끊음 · 조종 서비스 꺼짐" if value.get("ok") else
                          "Pi4 연결은 끊었지만 조종 서비스 끄기 실패: " + str(value.get("reason"))))

    def start_drive(self):
        if self.client:
            self.pressed.clear()
            self.client.set_limits(self.erpm.value(), self.steer.value(), self.reverse.value())
            self.client.request_start()
            self.activateWindow()
            self.setFocus()

    def stop_drive(self):
        self.pressed.clear()
        if self.client:
            self.client.request_stop()

    def push_keys(self):
        if self.client:
            self.client.set_limits(self.erpm.value(), self.steer.value(), self.reverse.value())
            self.client.set_keys(bool(self.pressed & FORWARD), bool(self.pressed & LEFT), bool(self.pressed & RIGHT),
                                 bool(self.pressed & BACK))

    # ---- 2026-11-16 모터·서보(VESC) 표시: 받은 /api/status만 보여 준다(새 요청·명령 없음) ----
    def _build_vesc_panel(self, layout):
        box = QtWidgets.QGroupBox("모터·서보 (Pi4 제어기 · VESC 값 · 읽기 전용)")
        grid = QtWidgets.QGridLayout(box)
        self.vesc_hint = QtWidgets.QLabel("Pi4 연결 전")
        self.vesc_hint.setWordWrap(True)
        self.vesc_hint.setObjectName("muted")
        grid.addWidget(self.vesc_hint, 0, 0, 1, 4)
        self.vesc_values = {}
        titles = [title for title, _text, _level in vesc_status.status_rows({})]
        # 2026-11-17: 짧은 값은 두 칸씩, 긴 값(정지 유지·통신)은 아래에 한 줄 전체로.
        normal = [title for title in titles if title not in vesc_status.WIDE_ROWS]
        wide = [title for title in titles if title in vesc_status.WIDE_ROWS]
        for index, title in enumerate(normal + wide):
            label = QtWidgets.QLabel(vesc_status.MISSING)
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            if index < len(normal):
                row, column = 1 + index // 2, (index % 2) * 2
                grid.addWidget(QtWidgets.QLabel(title), row, column)
                grid.addWidget(label, row, column + 1)
            else:
                row = 1 + (len(normal) + 1) // 2 + index - len(normal)
                grid.addWidget(QtWidgets.QLabel(title), row, 0)
                grid.addWidget(label, row, 1, 1, 3)
            self.vesc_values[title] = label
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        layout.addWidget(box)
        self.vesc_diag = QtWidgets.QGroupBox("VESC 진단 (펼치기)")
        self.vesc_diag.setCheckable(True)
        self.vesc_diag.setChecked(False)
        diag_layout = QtWidgets.QVBoxLayout(self.vesc_diag)
        self.vesc_diag_body = QtWidgets.QWidget()
        diag_grid = QtWidgets.QGridLayout(self.vesc_diag_body)
        diag_grid.setContentsMargins(0, 0, 0, 0)
        self.vesc_diag_values = {}
        for row, (title, _text) in enumerate(vesc_status.diagnostic_rows({})):
            diag_grid.addWidget(QtWidgets.QLabel(title), row, 0)
            label = QtWidgets.QLabel(vesc_status.MISSING)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            diag_grid.addWidget(label, row, 1)
            self.vesc_diag_values[title] = label
        diag_grid.setColumnStretch(1, 1)
        diag_layout.addWidget(self.vesc_diag_body)
        self.vesc_diag_body.setVisible(False)
        self.vesc_diag.toggled.connect(self.vesc_diag_body.setVisible)
        layout.addWidget(self.vesc_diag)

    def _update_vesc(self, data, fresh):
        connected = isinstance(data, dict) and bool(data)
        if not connected:
            hint = "Pi4 연결 전" if not self.client else "제어기 상태 수신 대기"
        elif not vesc_status.controller_sends_vesc_values(data):
            hint = "Pi4 제어기가 VESC 추가 값을 아직 보내지 않습니다(2026-11-16 이후 제어기 배포 필요). 있는 값만 표시합니다."
        else:
            hint = "서보는 명령값입니다(실측 조향 없음). 입력 전류는 VESC 추정값, 속도 m/s는 미검증 환산입니다."
        if not fresh and connected:
            hint = "상태 끊김 · 마지막으로 받은 값 · " + hint
        if self.vesc_hint.text() != hint:
            self.vesc_hint.setText(hint)
        for title, text, level in vesc_status.status_rows(data if connected else {}, fresh):
            label = self.vesc_values.get(title)
            if label is None:
                continue
            colors = vesc_status.LEVEL_COLORS_LIGHT  # 2026-11-17: 런처의 밝은 테마
            color = colors["unknown"] if not (connected and fresh) else colors[level]
            if label.text() != text:
                label.setText(text)
            style = f"color:{color};"
            if label.styleSheet() != style:
                label.setStyleSheet(style)
        if self.vesc_diag.isChecked():
            for title, text in vesc_status.diagnostic_rows(data if connected else {}):
                label = self.vesc_diag_values.get(title)
                if label is not None and label.text() != text:
                    label.setText(text)

    def refresh_drive(self):
        if not self.client:
            self.keys_label.setText("")
            self._update_vesc(None, False)
            return
        snap = self.client.snapshot()
        data, keys = snap["status"], snap["keys"]
        motion = "▲ 전진" if keys["forward"] and not keys.get("back") else "▼ 후진" if keys.get("back") and not keys["forward"] else "■ 정지"
        arrows = ("◀ " if keys["left"] else "   ") + motion + (" ▶" if keys["right"] else "")
        self.keys_label.setText(("조종 중 · " if snap["active"] else "대기 · ") + arrows)
        self.keys_label.setStyleSheet("font-size:18px;font-weight:700;padding:4px;color:%s;" %
                                      ("#16a34a" if snap["active"] and keys["forward"] else "#f59e0b" if snap["active"] and keys.get("back")
                                       else "#e2e8f0" if snap["active"] else "#94a3b8"))
        self.drive_status.setText(
            f"제어기 {data.get('state', '—')} · {'최신' if snap['fresh'] else '상태 끊김'} · 명령 {_number(data.get('commanded_motor_erpm'), 0)} ERPM"
            f" · 실측 {_number(data.get('raw_erpm'), 0)} ERPM · 전류 {_number(data.get('current_a'), 1)} A · 전압 {_number(data.get('voltage_v'), 1)} V"
            f" · 정지 출력 {data.get('stop_output', '—')} {_number(data.get('stop_output_a'), 1)} A · 서보 {_number(data.get('commanded_servo_position'), 3)}"
            + f" · 후진 최대 {_number(data.get('manual_reverse_max_erpm'), 0)} ERPM"
            + (f" · {data.get('reason')}" if data.get("reason") else ""))
        self.drive_message.setText(snap["message"])
        self.start_button.setEnabled(not snap["active"])
        self._update_vesc(data, snap["fresh"])

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease) and self.client and self.isActiveWindow():
            key = event.key()
            if key in DRIVE_KEYS and (self.driving() or key in STOP):
                if kind == QtCore.QEvent.KeyPress and not event.isAutoRepeat():
                    if key in STOP:
                        self.stop_drive()
                    else:
                        self.pressed.add(key)
                        self.push_keys()
                elif kind == QtCore.QEvent.KeyRelease and not event.isAutoRepeat():
                    self.pressed.discard(key)
                    self.push_keys()
                return True
        return super().eventFilter(obj, event)

    def changeEvent(self, event):
        if event.type() == QtCore.QEvent.ActivationChange and not self.isActiveWindow() and self.client:
            self.pressed.clear()
            if self.driving():
                self.client.close_session("창 초점을 잃어 조종을 해제했습니다. 다시 '조종 시작'을 누르세요.")
            else:
                self.client.set_keys(False, False, False)
        super().changeEvent(event)

    # ---- recording ----------------------------------------------------------------
    def start_recording(self):
        scene = self.scene.text().strip()
        jetson, pi5, password, gps = self.jetson, self.pi5, self.password, self.gps.isChecked()
        self.record_status.setText("녹화 시작 중… (센서가 꺼져 있으면 켜는 데 30초쯤)")
        self.run_task(lambda: sensor_record.start(jetson, pi5, password, scene, gps=gps), self._record_started)
        self.tasks_kind("record")
        self.update_buttons()

    def _record_started(self, value):
        self.recording = bool(value.get("ok"))
        self.gui_sensors = bool(value.get("sensors_started"))
        text = sensor_record.describe(value)
        if value.get("ok"):
            text += " · " + str(value.get("session", "")).rsplit("/", 1)[-1]
            text += " · 센서 " + ("녹화가 켬" if value.get("sensors_started") else "이미 켜져 있던 것 사용")
        self.record_status.setText(text)
        self.update_buttons()

    def stop_recording(self):
        jetson, pi5, password = self.jetson, self.pi5, self.password
        self.record_status.setText("녹화 정지 중… (마무리·정리 최대 1분)")
        self.run_task(lambda: sensor_record.stop(jetson, pi5, password), self._record_stopped)
        self.tasks_kind("record")
        self.update_buttons()

    def _record_stopped(self, value):
        if value.get("ok") or value.get("reason") == "not_recording":
            self.recording = False
        self.gui_sensors = value.get("sensors") in ("stop_incomplete", "recording_active")
        self.record_status.setText(sensor_record.describe(value))
        self.update_buttons()

    def refresh_recording(self, quiet=False):
        if not self.jetson or "record" in self.busy() or "status" in self.busy():
            return
        jetson, password = self.jetson, self.password

        def done(value):
            if not value.get("ok"):
                if not quiet:
                    self.record_status.setText("녹화 상태 확인 실패: " + sensor_record.describe(value))
                return
            self.recording = bool(value.get("recording"))
            self.gui_sensors = bool(value.get("gui_sensors"))
            if self.recording:
                size = value.get("bytes")
                self.record_status.setText(
                    f"녹화 중 · {value.get('elapsed_s', '?')} s · {round(size / 1e9, 2) if isinstance(size, (int, float)) else '?'} GB"
                    f" · 녹화기 {'정상' if value.get('recorder_alive') else '!! 멈춤'} · GPS {'포함' if value.get('gps') == '1' else '없음'}"
                    f" · 센서 {value.get('sensors') or '없음'} · SSD 남음 {value.get('ssd_free_gb', '?')} GB"
                    f" · {str(value.get('session', '')).rsplit('/', 1)[-1]}")
            elif not quiet or self.record_status.text().startswith(("녹화 상태", "녹화 중")):
                self.record_status.setText(f"녹화 안 함 · 센서 {value.get('sensors') or '꺼짐'} · SSD 남음 {value.get('ssd_free_gb', '?')} GB"
                                           f" · 시계 차 {value.get('chrony_offset_s', '?')} s"
                                           + (" · 녹화가 켠 센서 남음('녹화 정지'로 끔)" if self.gui_sensors else ""))
            self.update_buttons()
        self.run_task(lambda: sensor_record.status(jetson, password), done)
        self.tasks_kind("status")

    def showEvent(self, event):
        QtWidgets.QApplication.instance().installEventFilter(self)
        self.timer.start(100)
        self.record_timer.start(4000)
        super().showEvent(event)

    def closeEvent(self, event):
        # 조종은 끝내고(STOP 후 연결 해제), 녹화는 Jetson에서 계속된다. 창은 숨기기만 해서 진행 중 작업 결과를 받는다.
        QtWidgets.QApplication.instance().removeEventFilter(self)
        self.timer.stop()
        self.record_timer.stop()
        if self.client:
            self.client.request_stop()
            QtCore.QTimer.singleShot(400, self._shutdown_client)
        super().closeEvent(event)


def open_dialog(pi4, jetson, pi5, password, parent=None, existing=None):
    """Show the window; reuse `existing` (same hosts) so a running recording/connection stays visible."""
    if existing is not None and (existing.pi4, existing.jetson, existing.pi5) == (pi4, jetson, pi5):
        dialog = existing
        dialog.password = password or dialog.password
    else:
        dialog = DriveRecordDialog(pi4, jetson, pi5, password, parent)
    dialog.show()
    dialog.raise_()
    dialog.activateWindow()
    return dialog
