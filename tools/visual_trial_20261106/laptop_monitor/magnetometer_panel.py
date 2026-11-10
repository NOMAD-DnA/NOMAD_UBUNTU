"""Independent, uncalibrated magnetic compass for the laptop detail view.

Only the existing collector packet is used. No sensor access, fusion output,
alignment, calibration or magnetic declination is changed by this view.
"""
from __future__ import annotations

import math
import time
from collections.abc import Mapping

from PyQt5 import QtCore, QtGui, QtWidgets


FRESHNESS_S = 1.5
MAX_PAIR_SKEW_S = 0.25
FUTURE_TOLERANCE_S = 0.02
FORWARD_AXIS = (1., 0., 0.)  # ZED ROS IMU +X; not a calibrated vehicle extrinsic.


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def _vector(data, fields):
    values = [_number(data.get(key)) for key in fields]
    return values if all(value is not None for value in values) else None


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _norm(vector):
    return math.sqrt(_dot(vector, vector))


def compass_state(event=None, packet_age_s=None):
    """Calculate magnetic bearing of ZED IMU +X, using acceleration for tilt.

    Packet vectors are microtesla and m/s². At rest the accelerometer points
    up. Projecting magnetic north and forward into that horizontal plane gives
    a compass bearing clockwise from magnetic north. A moving accelerometer
    can bias this approximation even when the numerical checks pass.
    """
    event = event if isinstance(event, Mapping) else {}
    mag = event.get("mag") if isinstance(event.get("mag"), Mapping) else {}
    imu = event.get("imu") if isinstance(event.get("imu"), Mapping) else {}
    field = _vector(mag, ("x_ut", "y_ut", "z_ut"))
    acceleration = _vector(imu, ("ax", "ay", "az"))
    field_norm = _norm(field) if field is not None else None
    state = dict(received=bool(mag) and event.get("received") is not False,
                 calculable=False, reason="자기장 미수신", bearing_deg=None,
                 field_ut=field, field_norm_ut=field_norm, acceleration_mps2=acceleration,
                 acceleration_norm_mps2=_norm(acceleration) if acceleration is not None else None,
                 mag_age_s=_number(mag.get("age_s")), imu_age_s=_number(imu.get("age_s")),
                 frame_id=str(mag.get("frame_id") or "미확인"), warnings=[],
                 calibration="ZED 보정 상태 미확인 · 장착·편각 미보정", reference="자북 · ZED IMU +X 기준")
    if field_norm is not None:
        if field_norm < 10.:
            state["warnings"].append("자기장 세기 작음 (<10 µT)")
        elif field_norm > 100.:
            state["warnings"].append("자기장 세기 큼 (>100 µT)")
    if not state["received"]:
        return state
    if field is None:
        state["reason"] = "자기장 수치 이상"
        return state
    if field_norm < 1e-6:
        state["reason"] = "자기장 벡터 0"
        return state
    if acceleration is None:
        state["reason"] = "기울기 보정용 IMU 대기"
        return state
    elapsed = _number(packet_age_s)
    if elapsed is None or elapsed < 0.:
        state["reason"] = "수신 시각 미확인"
        return state
    for data, title in ((mag, "자기장"), (imu, "IMU")):
        receipt_age = _number(data.get("received_age_s"))
        measured_age = _number(data.get("age_s"))
        if receipt_age is None or receipt_age < 0. or measured_age is None:
            state["reason"] = f"{title} 시각 미확인"
            return state
        if measured_age < -FUTURE_TOLERANCE_S:
            # This is a passive diagnostic calculation, never a fusion input.
            # Keep fresh, mutually paired vectors visible while exposing the
            # clock error rather than implying estimator acceptance.
            warning = "측정시각 미래 · 시간 검사 불통과"
            if warning not in state["warnings"]:
                state["warnings"].append(warning)
        if max(receipt_age, measured_age) + elapsed > FRESHNESS_S:
            state["reason"] = f"{title} 오래됨"
            return state
    mag_stamp, imu_stamp = _number(mag.get("stamp")), _number(imu.get("stamp"))
    if mag_stamp is None or imu_stamp is None or mag_stamp <= 0. or imu_stamp <= 0.:
        state["reason"] = "센서 측정 시각 미확인"
        return state
    if abs(mag_stamp - imu_stamp) > MAX_PAIR_SKEW_S:
        state["reason"] = "자기장·IMU 측정 시각 불일치"
        return state
    if not mag.get("frame_id") or mag.get("frame_id") != imu.get("frame_id"):
        state["reason"] = "자기장·IMU 좌표계 불일치"
        return state
    gravity = state["acceleration_norm_mps2"]
    if not 7. <= gravity <= 12.5:
        state["reason"] = "가속도 크기 이상 (정지 상태에서 확인)"
        return state
    up = [value / gravity for value in acceleration]
    north = [value - _dot(field, up) * axis for value, axis in zip(field, up)]
    horizontal_norm = _norm(north)
    if horizontal_norm < max(1e-6, .02 * field_norm):
        state["reason"] = "수평 자기장 부족"
        return state
    north = [value / horizontal_norm for value in north]
    east = [north[1]*up[2] - north[2]*up[1], north[2]*up[0] - north[0]*up[2],
            north[0]*up[1] - north[1]*up[0]]
    forward = [value - _dot(FORWARD_AXIS, up)*axis
               for value, axis in zip(FORWARD_AXIS, up)]
    forward_norm = _norm(forward)
    if forward_norm < .02:
        state["reason"] = "기준 전방축이 수직"
        return state
    bearing = math.degrees(math.atan2(_dot(forward, east), _dot(forward, north))) % 360.
    state.update(calculable=True, bearing_deg=0. if abs(bearing-360.) < 1e-9 else bearing,
                 reason="계산값 표시 · 보정 상태 미확인")
    return state


class _Compass(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(122, 124)
        self.bearing = None

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        center = QtCore.QPointF(61, 61)
        painter.setPen(QtGui.QPen(QtGui.QColor("#52718d"), 1.))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawEllipse(center, 33, 33)
        painter.setFont(QtGui.QFont("Sans Serif", 9))
        painter.setPen(QtGui.QColor("#cbd5e1"))
        for text, box in zip(("N", "E", "S", "W"),
                             ((49, 5, 24, 20), (99, 51, 23, 20),
                              (49, 99, 24, 20), (0, 51, 23, 20))):
            painter.drawText(QtCore.QRectF(*box), QtCore.Qt.AlignCenter, text)
        if self.bearing is None:
            painter.setPen(QtGui.QColor("#64748b"))
            painter.drawText(QtCore.QRectF(35, 49, 52, 24), QtCore.Qt.AlignCenter, "—")
            return
        angle = math.radians(self.bearing)
        dx, dy = math.sin(angle), -math.cos(angle)
        tip = QtCore.QPointF(center.x()+28*dx, center.y()+28*dy)
        tail = QtCore.QPointF(center.x()-17*dx, center.y()-17*dy)
        color = QtGui.QColor("#fbbf24")
        painter.setPen(QtGui.QPen(color, 2.4, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        painter.drawLine(tail, tip)
        painter.setBrush(color)
        painter.drawPolygon(QtGui.QPolygonF([
            tip, QtCore.QPointF(tip.x()-9*dx-4*dy, tip.y()-9*dy+4*dx),
            QtCore.QPointF(tip.x()-9*dx+4*dy, tip.y()-9*dy-4*dx)]))


class MagnetometerPanel(QtWidgets.QGroupBox):
    """Passive 2 Hz magnetic test panel, independent of fusion diagnostics."""

    def __init__(self, parent=None):
        super().__init__("지자계 시험 · 융합과 독립", parent)
        self._packet = {}
        self._packet_age = None
        self._accepted_at = time.monotonic()
        self._state = compass_state()
        self._rendered_first_packet = False
        row = QtWidgets.QHBoxLayout(self)
        self.compass = _Compass(self)
        row.addWidget(self.compass)
        column = QtWidgets.QVBoxLayout()
        self.bearing = QtWidgets.QLabel("방위 미확정")
        self.bearing.setStyleSheet("font-size:18px; font-weight:600; color:#fbbf24;")
        self.status = QtWidgets.QLabel("자기장 미수신")
        self.raw = QtWidgets.QLabel("|B| — µT · X — / Y — / Z — µT")
        self.raw.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.raw.setStyleSheet("font-size:11px; color:#cbd5e1;")
        self.detail = QtWidgets.QLabel("자북 · ZED IMU +X 기준 · 0.5초 갱신")
        self.detail.setStyleSheet("font-size:10px; color:#94a3b8;")
        for label in (self.bearing, self.status, self.raw, self.detail):
            label.setWordWrap(True)
            column.addWidget(label)
        row.addLayout(column, 1)
        self.setToolTip("독립 지자계 시험값이며 융합 위치·방위에는 적용하지 않습니다.\n"
                        "ZED 자기장 출력의 센서 보정 상태는 미확인입니다.\n"
                        "이 화면에서 추가 장착·편각 보정을 적용하지 않았습니다.\n"
                        "정지 상태에서 시험하세요. 가속·금속·전류에 의한 오차는 계산 가능 여부와 별개입니다.")
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._refresh)

    def set_data(self, event, packet_age_s):
        self._packet = dict(event) if isinstance(event, Mapping) else {}
        self._packet_age = _number(packet_age_s)
        self._accepted_at = time.monotonic()
        # The timer renders at 2 Hz. Refresh the first real packet immediately.
        if (not self._rendered_first_packet and self._packet.get("mag") and
                self._packet.get("received") is not False):
            self._rendered_first_packet = True
            self._refresh()

    def state(self):
        return dict(self._state)

    def _refresh(self):
        age = (self._packet_age + max(0., time.monotonic()-self._accepted_at)
               if self._packet_age is not None else None)
        self._state = state = compass_state(self._packet, age)
        self.compass.bearing = state["bearing_deg"]
        self.compass.update()
        value = state["bearing_deg"]
        self.bearing.setText(f"자북 방위 {value:05.1f}°" if value is not None else "방위 미확정")
        warnings = " · ".join(state["warnings"])
        receipt = "수신 있음" if state["received"] else "미수신"
        self.status.setText(f"{receipt} · {state['reason']}" + (f"\n{warnings}" if warnings else ""))
        self.status.setStyleSheet("font-size:11px; color:" +
                                 ("#fbbf24" if state["calculable"] else "#fb7185") + ";")
        field = state["field_ut"]
        vector = " / ".join(f"{axis} {value:+.2f}" for axis, value in zip("XYZ", field)) if field else "X — / Y — / Z —"
        strength = f"{state['field_norm_ut']:.2f}" if state["field_norm_ut"] is not None else "—"
        self.raw.setText(f"|B| {strength} µT · {vector} µT")
        mag_age = state["mag_age_s"]
        age_text = f"{mag_age:+.3f}초" if mag_age is not None else "—"
        self.detail.setText("자북 · ZED IMU +X 기준 · 0.5초 갱신\n"
                            "ZED 보정 상태 미확인 · 장착·편각 미보정\n"
                            f"{state['frame_id']} · 측정 age {age_text} · 정확도 미검증")

    def showEvent(self, event):
        super().showEvent(event)
        self._refresh()
        self._timer.start()

    def hideEvent(self, event):
        self._timer.stop()
        super().hideEvent(event)
