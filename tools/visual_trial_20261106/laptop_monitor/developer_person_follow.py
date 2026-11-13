"""Person-follow workspace preview. Shared display only; no command/transport client.

Jetson follow integration is deliberately absent. See 사람추종_작업창_연결계획.md.
"""
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from developer_planning import PlanningMap, finite
from developer_perception import age_text
from planning_client import planning_for_display, stage_summary
from render_utils import stable_metadata, set_text


class PersonFollowWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.setWindowTitle("NOMAD | 사람 추종 작업창 · 미리보기")
        self.resize(1480, 940)
        self.setStyleSheet("QWidget {background:#111827;color:#e2e8f0;} "
                          "QPushButton {padding:7px 12px;background:#334155;border-radius:4px;} "
                          "QPushButton:disabled {color:#94a3b8;background:#1e293b;}")
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        self.notice = QtWidgets.QLabel("화면 미리보기 · Jetson 사람 추종 모듈 미연결")
        self.notice.setStyleSheet("color:#fbbf24;font-size:17px;font-weight:600;padding:8px;")
        self.notice.setWordWrap(True)
        layout.addWidget(self.notice)
        self.connection = QtWidgets.QLabel()
        self.connection.setWordWrap(True)
        layout.addWidget(self.connection)
        actions = QtWidgets.QHBoxLayout()
        self.select_button = QtWidgets.QPushButton("사람 선택 · 준비 중")
        self.start_button = QtWidgets.QPushButton("추종 시작 · 준비 중")
        self.stop_button = QtWidgets.QPushButton("추종 종료 · 준비 중")
        for button in (self.select_button, self.start_button, self.stop_button):
            button.setEnabled(False)
            button.setToolTip("Jetson 사람 추종 모듈과의 연결 구현 후 사용할 수 있습니다.")
            actions.addWidget(button)
        actions.addStretch(1)
        self.target = QtWidgets.QLabel("선택한 사람: 없음  |  사람까지 거리: —  |  추종 상태: 미연결")
        self.target.setWordWrap(True)
        actions.addWidget(self.target)
        layout.addLayout(actions)
        self.gpp_status, self.lpp_status = QtWidgets.QLabel(), QtWidgets.QLabel()
        for label, color in ((self.gpp_status, "#c084fc"), (self.lpp_status, "#fbbf24")):
            stable_metadata(label, rows=2)
            label.setStyleSheet("color:" + color)
            layout.addWidget(label)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        left = QtWidgets.QWidget()
        map_layout = QtWidgets.QVBoxLayout(left)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_layout.addWidget(QtWidgets.QLabel("플래닝 지도 · 현재 수신한 경로 / 목표 표시"))
        self.map = PlanningMap(components)
        # Never connect goal_selected: clicking this map cannot select/send a goal.
        self.map.goal_selection_enabled = False
        controls = QtWidgets.QHBoxLayout()
        for title, action in (("+ 확대", lambda: self.map.zoom(.7)),
                              ("− 축소", lambda: self.map.zoom(1/.7)),
                              ("전체보기", self.map.fit_all),
                              ("차량 중심", self.map.center_vehicle),
                              ("20m 보기", self.map.reset_twenty)):
            button = QtWidgets.QPushButton(title)
            button.clicked.connect(action)
            controls.addWidget(button)
        controls.addStretch(1)
        map_layout.addLayout(controls)
        layers = QtWidgets.QHBoxLayout()
        for name, title in (("global", "GPP 경로"), ("local", "LPP 궤적"), ("history", "이동 경로")):
            check = QtWidgets.QCheckBox(title)
            check.setChecked(True)
            check.toggled.connect(lambda checked, key=name: self.map.set_layer_visible(key, checked))
            layers.addWidget(check)
        for title, action in (("차량 윤곽", self.map.set_footprint_visible),
                              ("검사 셀 테두리", self.map.set_cells_visible)):
            check = QtWidgets.QCheckBox(title)
            check.setChecked(True)
            check.toggled.connect(action)
            layers.addWidget(check)
        layers.addStretch(1)
        map_layout.addLayout(layers)
        map_layout.addWidget(self.map, 1)
        self.map_info = QtWidgets.QLabel("지도 미수신")
        stable_metadata(self.map_info, rows=2)
        map_layout.addWidget(self.map_info)
        splitter.addWidget(left)

        right = QtWidgets.QWidget()
        right.setMinimumWidth(340)
        images = QtWidgets.QVBoxLayout(right)
        images.setContentsMargins(5, 0, 0, 0)
        self.panels = {}
        panel_type = (components or {}).get("ImagePanel")
        for source, title in (("rgb", "원본 영상 · 기존 미리보기"),
                              ("semantic", "인지 · 의미분할 영상")):
            panel = panel_type(title) if panel_type else QtWidgets.QLabel(title + " · 영상 구성요소 미연결")
            if panel_type:
                self.panels[source] = panel
                panel.setToolTip("현재는 영상 보기만 가능합니다. 사람 클릭 선택은 Jetson 모듈 연결 후 제공됩니다.")
            images.addWidget(panel, 1)
        image_note = QtWidgets.QLabel("원본·분할 영상은 각각 수신한 시각의 화면입니다.\n"
                                       "현재는 사람 식별·클릭 선택·실제 거리 계산을 하지 않습니다.")
        image_note.setWordWrap(True)
        image_note.setStyleSheet("color:#94a3b8;")
        images.addWidget(image_note)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([860, 570])
        layout.addWidget(splitter, 1)
        footer = QtWidgets.QLabel("기존 메인 수신 공유 · 이 창은 목표·출력 명령을 보내지 않습니다. "
                                    "창을 열거나 닫아도 기존 플래닝 동작은 바뀌지 않습니다.")
        footer.setWordWrap(True)
        footer.setStyleSheet("color:#94a3b8;")
        layout.addWidget(footer)
        self.setCentralWidget(body)
        self._image_keys, self._image_meta, self._image_received = {}, {}, {}
        self._map_key = self._map_received = self._last_now = None

    @staticmethod
    def _age(now, received):
        return max(0., now-received) if finite(now) and finite(received) else None

    def _clear_frames(self):
        # A replay seek must not retain images/maps from the future.
        self._image_keys.clear()
        self._image_meta.clear()
        self._image_received.clear()
        self._map_key = self._map_received = None
        self.map.event, self.map.pixels = {}, None
        self.map.history, self.map.planning, self.map.selected_goal = [], {}, None
        self.map._image_cache_key = None
        self.map._view_initialized = False
        for panel in self.panels.values():
            panel.pixmap = panel._image_key = panel._scaled_key = None
            panel.display.clear()
            panel.display.setText("WAITING — no image received")

    def update_snapshot(self, snapshot):
        """Only display the operator/replay snapshot; never allocate board readers."""
        now = snapshot.get("now", time.monotonic())
        if finite(self._last_now) and finite(now) and now < self._last_now:
            self._clear_frames()
        self._last_now = now
        ui = snapshot.get("planning_ui", {})
        mode = ("기록 재생 · 읽기 전용" if snapshot.get("replay") else
                "관전 · 읽기 전용" if ui.get("spectator") else
                "SSH 비활성 · 읽기 전용" if ui.get("no_ssh", True) else "운용 화면")
        planner = "연결됨" if ui.get("client_active") else "미연결"
        self.connection.setText(f"{mode}  |  플래너: {planner}  |  사람 추종: 미연결")
        # This UI-only release has no capability negotiation or send handlers.
        # Ordinary planner connection must never enable follow actions.
        for button in (self.select_button, self.start_button, self.stop_button):
            button.setEnabled(False)
        received = snapshot.get("received", {})
        metadata = snapshot.get("image_meta", {})
        source_images = snapshot.get("images", {})
        for source, panel in self.panels.items():
            image, event = source_images.get(source), metadata.get(source, {})
            if isinstance(image, QtGui.QImage) and not image.isNull() and not event.get("decode_error"):
                key = (image.cacheKey(), event.get("t"), received.get("image:"+source))
                if key != self._image_keys.get(source):
                    panel.set_image(image)
                    self._image_keys[source] = key
                    self._image_meta[source] = dict(event)
                    self._image_received[source] = received.get("image:"+source)
            shown = self._image_meta.get(source, {})
            stamp = shown.get("t")
            capture = time.strftime("%H:%M:%S", time.localtime(stamp)) if finite(stamp) and 0 < stamp < 4102444800 else "—"
            dimensions = f"{panel.pixmap.width()}×{panel.pixmap.height()}" if panel.pixmap is not None else "—"
            error = "새 영상 오류 · 마지막 정상 화면 | " if event.get("decode_error") or event.get("latest_frame_error") else ""
            set_text(panel.meta, error + age_text(self._age(now, self._image_received.get(source))) +
                     f" | {dimensions} | 촬영 {capture}")
        image, event = source_images.get("costmap"), metadata.get("costmap", {})
        if isinstance(image, QtGui.QImage) and not image.isNull() and not event.get("decode_error"):
            key = (image.cacheKey(), event.get("t"), received.get("image:costmap"))
            if key != self._map_key and self.map.set_image(image, event):
                self._map_key, self._map_received = key, received.get("image:costmap")
        self.map.map_age = self._age(now, self._map_received)
        self.map.pose_age = self._age(now, received.get("pose:public"))
        self.map.history = snapshot.get("histories", {}).get("public", ())
        self.map.forward_axis = snapshot.get("forward_axis", (1., 0., 0.))
        self.map.goal_selection_enabled = False
        self.map.selected_goal = None
        self.map.planning = planning_for_display(snapshot.get("telemetry", {}).get("planning", {}),
                                                self._age(now, received.get("telemetry:planning")))
        gpp, lpp = stage_summary(self.map.planning)
        set_text(self.gpp_status, gpp)
        set_text(self.lpp_status, lpp)
        set_text(self.map_info, "지도 " + age_text(self.map.map_age) + " | 위치 " + age_text(self.map.pose_age) +
                 "\n휠: 확대/축소 · 오른쪽 드래그: 이동 · 지도 목표 선택 비활성")
        if not self.map._view_initialized and self.map.geometry(self.map.event):
            if self.map._latest() is not None:
                self.map.center_vehicle(reset=True)
            else:
                self.map.fit_all()
            self.map._view_initialized = True
        if self.isVisible():
            self.map.redraw()
