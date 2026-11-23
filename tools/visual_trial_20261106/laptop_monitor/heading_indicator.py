"""Small laptop-only heading overlay using already received pose/diagnostics.

The ring remains explicitly local until fresh fusion diagnostics identify a
global heading or an explicitly provisional magnetic initial heading.  This
widget never acquires sensor data or controls a board.
"""
from __future__ import annotations

import math

from PyQt5 import QtCore, QtGui, QtWidgets


FRESHNESS_S = 0.5


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _true(value):
    # ROS DiagnosticStatus values are strings; never use bool("false").
    return value is True or (isinstance(value, str) and value.lower() == "true")


def _false(value):
    return value is False or (isinstance(value, str) and value.lower() == "false")


def _age(value):
    return float(value) if _number(value) and value >= 0 else None


class HeadingIndicator(QtWidgets.QWidget):
    """Compact ENU/local compass with no subscriptions, timers or workers."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(104, 122)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self._state = {}
        self.set_data(None, (1.0, 0.0, 0.0), None)

    def set_data(self, pose, forward_axis, pose_age_s, diagnostics=None,
                 diagnostics_age_s=None, source="public"):
        """Refresh from existing viewer data; all ages are seconds at receipt.

        diagnostics.measurement_age_s is added to diagnostics_age_s so an old
        measurement in a newly relayed packet cannot establish global heading.
        Quaternion order follows the viewer pose dictionary: qx,qy,qz,qw.
        """
        source = source if source in ("public", "fusion", "vio") else "unknown"
        pose_age = _age(pose_age_s)
        diag_age = _age(diagnostics_age_s)
        values = diagnostics.get("values", {}) if isinstance(diagnostics, dict) else {}
        if not isinstance(values, dict):
            values = {}
        if isinstance(diagnostics, dict) and "measurement_age_s" in diagnostics:
            measurement_age = _age(diagnostics["measurement_age_s"])
        elif "measurement_age_s" in values:
            measurement_age = _age(values["measurement_age_s"])
        else:
            measurement_age = 0.0
        effective_diag_age = (diag_age + measurement_age
                              if diag_age is not None and measurement_age is not None else None)
        state = {"source": source, "valid": False, "global_valid": False, "provisional": False,
                 "mode": "waiting", "reason": "pose_missing", "pose_age_s": pose_age,
                 "diagnostics_age_s": effective_diag_age,
                 "yaw_deg": None, "bearing_deg": None, "arrow_xy": None,
                 "ring_labels": ["+Y", "+X", "−Y", "−X"],
                 "note": "방위 미확정", "status": "위치 대기"}
        if not isinstance(pose, dict):
            self._commit(state)
            return
        if pose_age is None or pose_age > FRESHNESS_S:
            state.update(mode="stale", reason="pose_stale", status="위치 오래됨")
            self._commit(state)
            return
        if effective_diag_age is not None and effective_diag_age <= FRESHNESS_S and _false(values.get("orientation_valid")):
            state.update(mode="invalid", reason="orientation_invalid", status="자세 무효")
            self._commit(state)
            return
        try:
            quaternion = [pose[k] for k in ("qx", "qy", "qz", "qw")]
            axis = list(forward_axis)
            if len(axis) != 3 or not all(_number(v) for v in quaternion + axis):
                raise ValueError("non-finite attitude/axis")
            norm = math.sqrt(sum(v * v for v in quaternion))
            axis_norm = math.sqrt(sum(v * v for v in axis))
            if abs(norm - 1.0) > 0.03 or axis_norm < 1e-9:
                raise ValueError("invalid attitude/axis")
            x, y, z, w = (v / norm for v in quaternion)
            vx, vy, vz = (v / axis_norm for v in axis)
            fx = (1 - 2*(y*y+z*z))*vx + 2*(x*y-z*w)*vy + 2*(x*z+y*w)*vz
            fy = 2*(x*y+z*w)*vx + (1 - 2*(x*x+z*z))*vy + 2*(y*z-x*w)*vz
            horizontal = math.hypot(fx, fy)
            if horizontal < 1e-3:
                state.update(mode="invalid", reason="forward_vertical", status="전방축 수직")
                self._commit(state)
                return
        except (KeyError, TypeError, ValueError, OverflowError):
            state.update(mode="invalid", reason="attitude_invalid", status="자세 대기")
            self._commit(state)
            return
        yaw = math.degrees(math.atan2(fy, fx))
        if abs(yaw) < 1e-10:
            yaw = 0.0
        global_valid = (source in ("public", "fusion") and
                        effective_diag_age is not None and effective_diag_age <= FRESHNESS_S and
                        _true(values.get("global_heading_valid")) and
                        not _false(values.get("orientation_valid")))
        provisional = (not global_valid and source in ("public", "fusion") and
                       effective_diag_age is not None and effective_diag_age <= FRESHNESS_S and
                       _true(values.get("heading_provisional")) and
                       _true(values.get("orientation_valid")))
        mode = "global" if global_valid else ("provisional" if provisional else "local")
        reason = "global_confirmed" if global_valid else ("heading_provisional" if provisional else "global_unconfirmed")
        state.update(valid=True, global_valid=global_valid, provisional=provisional, mode=mode,
                     reason=reason,
                     yaw_deg=yaw, arrow_xy=[fx / horizontal, fy / horizontal])
        if global_valid or provisional:
            bearing = (90.0 - yaw) % 360.0
            state.update(ring_labels=["N", "E", "S", "W"], note="전역 방위" if global_valid else "임시 방위",
                         bearing_deg=bearing, status=f"방위 {bearing:.1f}°")
        else:
            state["status"] = f"로컬 {yaw:.1f}°"
        self._commit(state)

    def _commit(self, state):
        self._state = state
        self.setToolTip("전방축의 XY 투영 · 로컬 각도는 +X 기준 반시계 방향\n"
                        "전역 방위는 융합기의 유효성 확인 시에만 표시\n"
                        "임시 방위는 미보정 지자계 초기값 기준이며 보정 완료를 뜻하지 않음")
        self.update()

    def state(self):
        # Keep callers from changing the widget state through its JSON snapshot.
        result = dict(self._state)
        result["ring_labels"] = list(result["ring_labels"])
        if result["arrow_xy"] is not None:
            result["arrow_xy"] = list(result["arrow_xy"])
        return result

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QPen(QtGui.QColor(123, 151, 181, 95), 0.8))
        painter.setBrush(QtGui.QColor(14, 23, 38, 220))
        painter.drawRoundedRect(QtCore.QRectF(0.5, 0.5, 103, 121), 7, 7)
        center, radius = QtCore.QPointF(52, 44), 23
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor("#668197"), 0.9))
        painter.drawEllipse(center, radius, radius)
        painter.setFont(QtGui.QFont("Sans Serif", 8))
        painter.setPen(QtGui.QColor("#cbd8e7"))
        for label, rectangle in zip(self._state["ring_labels"],
                                    [(35, 3, 34, 16), (79, 36, 24, 16),
                                     (35, 70, 34, 16), (1, 36, 24, 16)]):
            painter.drawText(QtCore.QRectF(*rectangle), QtCore.Qt.AlignCenter, label)
        arrow = self._state["arrow_xy"]
        if arrow is not None:
            dx, dy = arrow[0], -arrow[1]
            tip = QtCore.QPointF(center.x() + 20*dx, center.y() + 20*dy)
            tail = QtCore.QPointF(center.x() - 12*dx, center.y() - 12*dy)
            color = QtGui.QColor("#51db9b" if self._state["global_valid"] else
                                ("#e4b360" if self._state["provisional"] else "#25cde0"))
            painter.setPen(QtGui.QPen(color, 2, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
            painter.drawLine(tail, tip)
            painter.setBrush(color)
            painter.drawPolygon(QtGui.QPolygonF([
                tip, QtCore.QPointF(tip.x()-8*dx-4*dy, tip.y()-8*dy+4*dx),
                QtCore.QPointF(tip.x()-8*dx+4*dy, tip.y()-8*dy-4*dx)]))
        else:
            painter.setPen(QtGui.QColor("#748599"))
            painter.drawText(QtCore.QRectF(37, 33, 30, 20), QtCore.Qt.AlignCenter, "—")
        painter.setFont(QtGui.QFont("Sans Serif", 7))
        painter.setPen(QtGui.QColor("#85d7b1" if self._state["global_valid"] else "#e4b360"))
        painter.drawText(QtCore.QRectF(2, 87, 100, 15), QtCore.Qt.AlignCenter, self._state["note"])
        painter.setFont(QtGui.QFont("Sans Serif", 8))
        painter.setPen(QtGui.QColor("#e5edf7"))
        painter.drawText(QtCore.QRectF(2, 103, 100, 16), QtCore.Qt.AlignCenter, self._state["status"])
