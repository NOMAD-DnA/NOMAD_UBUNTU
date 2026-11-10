"""Traversability developer window. Displays existing snapshots, without I/O."""
import math
import time
from PyQt5 import QtGui, QtWidgets
from module_logs import attach_module_logs, update_module_logs
from render_utils import history_signature, stable_metadata


def age(snapshot, key):
    values = (snapshot.get("now", time.monotonic()), snapshot.get("received", {}).get(key))
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        return None
    return max(0., values[0] - values[1])


def age_text(value, threshold=2.):
    return "미수신" if value is None else f"수신 후 {value:.1f}초" + (" · 오래됨" if value > threshold else "")


def valid_grid(event):
    """Same physical XY-grid requirements as the injected MapPanel."""
    grid = event.get("grid", {})
    if event.get("georeferenced") is not True or not isinstance(grid, dict):
        return None
    if grid.get("image_y") != "high_to_low" or not isinstance(grid.get("frame_id"), str) or not grid["frame_id"]:
        return None
    origin = grid.get("origin", {})
    if not isinstance(origin, dict):
        return None
    numbers = [grid.get(k) for k in ("resolution", "width", "height")]
    numbers += [origin.get(k) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in numbers):
        return None
    if min(numbers[:3]) <= 0 or abs(origin["qx"]) > .001 or abs(origin["qy"]) > .001:
        return None
    if abs(sum(origin[k] ** 2 for k in ("qx", "qy", "qz", "qw")) - 1.) > .03:
        return None
    return grid


class TraversabilityWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.setWindowTitle("NOMAD | 주행가능성 개발")
        self.resize(1220, 760)
        self.setStyleSheet("QWidget {background:#111827;color:#e2e8f0;} QLabel {font-size:12px;}")
        self.panels, self._image_keys, self._pose_keys, self._map_received = {}, {}, {}, {}
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        self.status = QtWidgets.QLabel("주행가능성: 미수신")
        self.position = QtWidgets.QLabel("공용 위치: 미수신")
        self.heading = QtWidgets.QLabel("화살표 전방축: —")
        for label in (self.status, self.position, self.heading):
            label.setWordWrap(True)
            layout.addWidget(label)
        maps = QtWidgets.QHBoxLayout()
        panel_type = (components or {}).get("MapPanel")
        for source, title in (("costmap", "코스트맵 | XY 상단뷰"), ("slope", "경사도 | XY 상단뷰")):
            panel = panel_type(title, slope=(source == "slope"), heading_note="통합 창의 전방축") if panel_type else QtWidgets.QLabel(title + "\nMapPanel 구성요소 미연결")
            if panel_type:
                self.panels[source] = panel
                panel.meta.setWordWrap(True)
                stable_metadata(panel.meta)
            maps.addWidget(panel, 1)
        layout.addLayout(maps, 1)
        footer = QtWidgets.QLabel("청록색: 동일 좌표계의 공용 위치·이동 경로 | 회색: 값 미확정(미관측과 관측 후 계산 미완료를 포함할 수 있음)")
        footer.setWordWrap(True)
        footer.setStyleSheet("color:#94a3b8;")
        layout.addWidget(footer)
        self.setCentralWidget(body)
        attach_module_logs(self)

    def append_module_log(self, event):
        return self.log_panel.append_event(event)

    def update_snapshot(self, snapshot):
        update_module_logs(self, snapshot, "traversability")
        status = snapshot.get("statuses", {}).get("perception", {})
        self.status.setText("주행가능성: " + str(status.get("state", "미수신")) + " | " + age_text(age(snapshot, "status:perception")))
        self.status.setToolTip(str(status.get("detail", "")))
        pose_age = age(snapshot, "pose:public")
        history = snapshot.get("histories", {}).get("public", ())
        latest = history[-1] if history else {}
        self.position.setText("공용 위치: " + str(latest.get("frame_id", "—")) + " | " + age_text(pose_age, 1.))
        self.heading.setText("화살표 전방축: " + str(snapshot.get("heading_note", "—")))
        for source, panel in self.panels.items():
            image = snapshot.get("images", {}).get(source)
            event = snapshot.get("image_meta", {}).get(source, {})
            grid = valid_grid(event)
            image_age = age(snapshot, "image:" + source)
            if isinstance(image, QtGui.QImage) and not image.isNull() and grid and not event.get("decode_error"):
                key = (image.cacheKey(), event.get("t"), snapshot.get("received", {}).get("image:" + source))
                if self._image_keys.get(source) != key:
                    panel.set_image(image, event)
                    self._image_keys[source] = key
                    self._map_received[source] = snapshot.get("received", {}).get("image:" + source)
            shown_grid = valid_grid(panel.event)
            frame = shown_grid.get("frame_id") if shown_grid else None
            matching = history if frame and latest.get("frame_id") == frame else ()
            pose_key = (history_signature(matching), pose_age is not None and pose_age <= 1., frame)
            if self._pose_keys.get(source) != pose_key:
                panel.set_pose(matching, pose_age)
                self._pose_keys[source] = pose_key
            forward_axis = snapshot.get("forward_axis", (1., 0., 0.))
            if panel.forward_axis != forward_axis:
                panel.forward_axis = forward_axis
                panel.dirty = True
            panel.redraw()
            info = (f"{shown_grid['width']}×{shown_grid['height']} · {shown_grid['resolution']:.2f}m/cell · {frame}"
                    if shown_grid else "격자 좌표 미수신")
            shown_received = self._map_received.get(source)
            shown_age = None if shown_received is None else max(0., snapshot.get("now", time.monotonic()) - shown_received)
            timing = "표시 지도 " + age_text(shown_age)
            if event.get("decode_error"):
                timing += " | 새 영상 오류 · 마지막 정상 영상 유지"
            elif event and not grid:
                timing += " | 새 영상 좌표 미확정 · 마지막 지도 유지 (" + age_text(image_age) + ")"
            position = "위치 좌표계 일치" if matching else "동일 좌표계 위치 미수신"
            panel.meta.setText(timing + " | " + info + " | " + position)
