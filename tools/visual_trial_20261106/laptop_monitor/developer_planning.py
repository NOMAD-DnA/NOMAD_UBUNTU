"""Planning developer view; consumes only the operator's shared snapshots."""
import math
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import FancyArrowPatch
from matplotlib.transforms import Affine2D
import numpy as np
from module_logs import attach_module_logs, update_module_logs
from render_utils import HistorySampler, history_signature, stable_metadata, set_text


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class PlanningMap(QtWidgets.QWidget):
    goal_selected = QtCore.pyqtSignal(float, float, str)

    def __init__(self, components=None):
        super().__init__()
        components = components or {}
        self.geometry = components['grid_geometry']
        self.rotate_vector = components['rotate_vector']
        self.event, self.pixels = {}, None
        self.history, self.planning, self.selected_goal = [], {}, None
        self.forward_axis = (1., 0., 0.)
        self.pose_age, self.map_age = None, None
        self.goal_selection_enabled = False
        self._view_initialized, self._press, self._pan = False, None, None
        self._artists_ready = False
        self._pixels_dirty = False
        self._geometry_key = self._paths_key = self._goals_key = self._pose_key = None
        self._image_cache_key = None
        self._sampler = HistorySampler(3000)
        self.figure = Figure(facecolor='#111827')
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setMinimumSize(650, 430)
        self.ax = self.figure.add_subplot(111, facecolor='#303946')
        self.figure.subplots_adjust(left=.065, right=.985, bottom=.09, top=.94)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas, 1)
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.canvas.mpl_connect('button_press_event', self._mouse_press)
        self.canvas.mpl_connect('button_release_event', self._mouse_release)
        self.canvas.mpl_connect('motion_notify_event', self._mouse_move)
        self.canvas.mpl_connect('scroll_event', self._wheel)
        self.redraw()

    def set_image(self, image, event):
        geometry = self.geometry(event)
        if geometry is None or geometry[0]['frame_id'] != 'odom':
            return False
        self.event = event
        if self._image_cache_key == image.cacheKey():
            return True
        self._image_cache_key = image.cacheKey()
        rgb = image.convertToFormat(QtGui.QImage.Format_RGB888)
        bits = rgb.bits()
        bits.setsize(rgb.byteCount())
        pixels = np.frombuffer(bits, dtype=np.uint8).reshape(rgb.height(), rgb.bytesPerLine())[:, :rgb.width()*3].reshape(rgb.height(), rgb.width(), 3)
        if self.pixels is None or not np.array_equal(self.pixels, pixels):
            self.pixels = pixels.copy()
            self._pixels_dirty = True
        return True

    def _frame(self):
        geometry = self.geometry(self.event)
        return geometry[0]['frame_id'] if geometry else None

    def _latest(self):
        frame = self._frame()
        return next((p for p in reversed(self.history) if p.get('frame_id') == frame), None)

    def _limits(self):
        return tuple(self.ax.get_xlim()), tuple(self.ax.get_ylim())

    def _set_limits(self, cx, cy, width, height=None):
        width = max(.5, min(100000., width))
        height = width if height is None else max(.5, min(100000., height))
        self.ax.set_xlim(cx-width/2, cx+width/2)
        self.ax.set_ylim(cy-height/2, cy+height/2)
        self.canvas.draw_idle()

    def zoom(self, factor, center=None):
        x, y = self._limits()
        cx, cy = center or ((x[0]+x[1])/2, (y[0]+y[1])/2)
        if not all(finite(v) for v in (factor, cx, cy)) or factor <= 0:
            return
        self._set_limits(cx, cy, (x[1]-x[0])*factor, (y[1]-y[0])*factor)
        self._view_initialized = True

    def center_vehicle(self, reset=False):
        pose = self._latest()
        if pose is None:
            return
        x, y = self._limits()
        self._set_limits(pose['x'], pose['y'], 20. if reset else x[1]-x[0], 20. if reset else y[1]-y[0])
        self._view_initialized = True

    def reset_twenty(self):
        pose = self._latest()
        center = (pose['x'], pose['y']) if pose else (0., 0.)
        self._set_limits(*center, 20.)
        self._view_initialized = True

    def _all_points(self):
        points = []
        geometry = self.geometry(self.event)
        if geometry:
            g, yaw = geometry
            w, h = g['width']*g['resolution'], g['height']*g['resolution']
            transform = Affine2D().rotate(yaw).translate(g['origin']['x'], g['origin']['y'])
            points.extend(transform.transform([[0, 0], [w, 0], [w, h], [0, h]]).tolist())
        frame = self._frame()
        points.extend((p['x'], p['y']) for p in self.history if p.get('frame_id') == frame)
        if self.planning.get('frame') == frame:
            for name in ('path', 'trajectory'):
                points.extend(p[:2] for p in self.planning.get(name, ()))
            if self.planning.get('goal'):
                points.append(self.planning['goal'][:2])
        if self.selected_goal and self.selected_goal[2] == frame:
            points.append(self.selected_goal[:2])
        return [(p[0], p[1]) for p in points if len(p) >= 2 and finite(p[0]) and finite(p[1])]

    def fit_all(self):
        points = self._all_points()
        if not points:
            self.reset_twenty()
            return
        xs, ys = zip(*points)
        span = max(4., max(xs)-min(xs), max(ys)-min(ys))*1.15
        self._set_limits((min(xs)+max(xs))/2, (min(ys)+max(ys))/2, span)
        self._view_initialized = True

    def _mouse_press(self, event):
        if event.inaxes is not self.ax:
            return
        self._press = (event.x, event.y, event.button)
        if event.button in (2, 3):
            self._pan = (event.x, event.y, self._limits())

    def _mouse_move(self, event):
        if self._pan is None or event.x is None or event.y is None:
            return
        px, py, (x, y) = self._pan
        dx = (event.x-px)/max(1, self.ax.bbox.width)*(x[1]-x[0])
        dy = (event.y-py)/max(1, self.ax.bbox.height)*(y[1]-y[0])
        self.ax.set_xlim(x[0]-dx, x[1]-dx)
        self.ax.set_ylim(y[0]-dy, y[1]-dy)
        self._view_initialized = True
        self.canvas.draw_idle()

    def _mouse_release(self, event):
        press, self._press, self._pan = self._press, None, None
        if press is None or press[2] != 1 or event.button != 1 or event.x is None or event.y is None:
            return
        if math.hypot(event.x-press[0], event.y-press[1]) > 5:
            return
        self.select_goal(event)

    def select_goal(self, event):
        frame = self._frame()
        if (not self.goal_selection_enabled or event.inaxes is not self.ax or event.button != 1
                or frame != 'odom' or self.pixels is None or not finite(event.xdata) or not finite(event.ydata)):
            return
        # Clicks already use world/odom axes. Outside the map stays unknown;
        # this view never invents occupancy cells or transforms the goal again.
        self.goal_selected.emit(float(event.xdata), float(event.ydata), frame)

    def _wheel(self, event):
        if event.inaxes is self.ax and finite(event.xdata) and finite(event.ydata):
            self.zoom(.8 if event.button == 'up' else 1.25, (event.xdata, event.ydata))

    def _make_artists(self):
        """One axes/legend/image for the lifetime of this canvas."""
        ax = self.ax
        ax.set_facecolor('#303946')
        ax.set_aspect('equal', adjustable='box')
        ax.tick_params(colors='#cbd5e1', labelsize=9)
        for spine in ax.spines.values():
            spine.set_color('#64748b')
        ax.set_xlabel('X (m)', color='#cbd5e1')
        ax.set_ylabel('Y (m)', color='#cbd5e1')
        ax.grid(color='#94a3b8', alpha=.18, lw=.5)
        self._image_artist = ax.imshow(np.zeros((1, 1, 3), dtype=np.uint8), origin='upper',
                                       extent=(0, 1, 0, 1), interpolation='nearest', zorder=1)
        self._image_artist.set_visible(False)
        self._boundary_line, = ax.plot([], [], color='#94a3b8', lw=.7, zorder=2)
        self._history_line, = ax.plot([], [], color='#22d3ee', lw=1.1, alpha=.7, zorder=3)
        self._global_line, = ax.plot([], [], color='#c084fc', lw=2.2, label='GPP · Hybrid A*', zorder=5)
        self._local_line, = ax.plot([], [], color='#fbbf24', lw=2.2, label='RPP', zorder=5)
        self._accepted_artist = ax.scatter([], [], marker='o', s=90, facecolors='none',
                                           edgecolors='#fbbf24', label='Sent goal', zorder=7)
        self._selected_artist = ax.scatter([], [], marker='*', s=190, color='#fbbf24',
                                           edgecolors='white', label='Selected goal', zorder=8)
        self._vehicle_artist = ax.scatter([], [], s=75, color='#22d3ee', edgecolors='white',
                                          label='Vehicle', zorder=6)
        self._heading_artist = FancyArrowPatch((0, 0), (0, 0), arrowstyle='-|>',
                                               mutation_scale=13, color='#22d3ee', lw=2, zorder=7)
        ax.add_patch(self._heading_artist)
        self._heading_artist.set_visible(False)
        self._waiting_artist = ax.text(.5, .5, 'WAITING FOR MAP COORDINATES',
                                       transform=ax.transAxes, color='#cbd5e1', ha='center')
        self._unknown_artist = ax.text(.015, .015, 'Outside map: UNKNOWN (not free)',
                                       transform=ax.transAxes, color='#cbd5e1', fontsize=9, zorder=10,
                                       bbox={'facecolor':'#111827', 'alpha':.75, 'edgecolor':'none'})
        self._legend = ax.legend(loc='upper right', facecolor='#111827', edgecolor='#475569',
                                 labelcolor='#e2e8f0', fontsize=9)
        ax.set_title('Planning · waiting', color='#e2e8f0', fontsize=12)
        self._artists_ready = True

    @staticmethod
    def _point(artist, point):
        artist.set_offsets(np.asarray([point[:2]], dtype=float) if point is not None else np.empty((0, 2)))

    def redraw(self):
        limits = self._limits()
        changed = False
        if not self._artists_ready:
            self._make_artists()
            changed = True
        ax = self.ax
        geometry = self.geometry(self.event)
        frame = geometry[0]['frame_id'] if geometry and self.pixels is not None else None
        if frame:
            g, yaw = geometry
            w, h = g['width']*g['resolution'], g['height']*g['resolution']
            geometry_key = (frame, w, h, yaw, g['origin']['x'], g['origin']['y'], g['resolution'])
            if geometry_key != self._geometry_key:
                transform = Affine2D().rotate(yaw).translate(g['origin']['x'], g['origin']['y'])
                self._image_artist.set_extent((0, w, 0, h))
                self._image_artist.set_transform(transform+ax.transData)
                corners = transform.transform([[0, 0], [w, 0], [w, h], [0, h], [0, 0]])
                self._boundary_line.set_data(corners[:, 0], corners[:, 1])
                ax.title.set_text(f"Planning · {frame} · {g['resolution']:.2f}m/cell")
                self._geometry_key = geometry_key
                changed = True
            if self._pixels_dirty:
                self._image_artist.set_data(self.pixels)
                self._pixels_dirty = False
                changed = True
            if not self._image_artist.get_visible():
                self._image_artist.set_visible(True)
                self._waiting_artist.set_visible(False)
                changed = True
        elif self._image_artist.get_visible():
            self._image_artist.set_visible(False)
            self._boundary_line.set_data([], [])
            self._waiting_artist.set_visible(True)
            self._geometry_key = None
            changed = True

        matching = frame is not None and self.planning.get('frame') == frame
        global_path = self.planning.get('path', ()) if matching else ()
        local_path = self.planning.get('trajectory', ()) if matching else ()
        paths_key = (frame, tuple(tuple(p[:2]) for p in global_path), tuple(tuple(p[:2]) for p in local_path),
                     self.planning.get('path_valid'), self.planning.get('executable'))
        if paths_key != self._paths_key:
            for line, path in ((self._global_line, global_path), (self._local_line, local_path)):
                line.set_data([p[0] for p in path], [p[1] for p in path])
                valid = self.planning.get('path_valid', True) if line is self._global_line else self.planning.get('executable', True)
                line.set_linestyle('-' if valid else '--')
                line.set_alpha(1. if valid else .45)
            self._legend.get_texts()[0].set_text('GPP · Hybrid A*' if self.planning.get('path_valid', True) else 'GPP · reference / invalid')
            self._legend.get_texts()[1].set_text('LPP · RPP' if self.planning.get('executable', True) else 'LPP · no executable candidate')
            self._paths_key = paths_key
            changed = True
        accepted = self.planning.get('goal') if matching else None
        selected = self.selected_goal if self.selected_goal and self.selected_goal[2] == frame else None
        goals_key = (tuple(accepted[:2]) if accepted else None, tuple(selected[:2]) if selected else None)
        if goals_key != self._goals_key:
            self._point(self._accepted_artist, accepted)
            self._point(self._selected_artist, selected)
            self._goals_key = goals_key
            changed = True
        history = self._sampler.sample(self.history, frame) if frame else []
        pose = history[-1] if history else None
        color = '#22d3ee' if self.pose_age is not None and self.pose_age <= 1 else '#94a3b8'
        pose_key = (frame, len(history), history[0].get('t') if history else None,
                    tuple(pose.get(k) for k in ('t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')) if pose else None,
                    tuple(self.forward_axis), color)
        if pose_key != self._pose_key:
            path = history
            self._history_line.set_data([p['x'] for p in path], [p['y'] for p in path])
            self._point(self._vehicle_artist, (pose['x'], pose['y']) if pose else None)
            self._vehicle_artist.set_facecolor(color)
            visible_heading = False
            if pose:
                front = self.rotate_vector(pose, self.forward_axis)
                norm = math.hypot(front[0], front[1])
                if norm > .05:
                    self._heading_artist.set_positions((pose['x'], pose['y']),
                        (pose['x']+.6*front[0]/norm, pose['y']+.6*front[1]/norm))
                    self._heading_artist.set_color(color)
                    visible_heading = True
            self._heading_artist.set_visible(visible_heading)
            self._pose_key = pose_key
            changed = True
        # Image extent updates must not reset a user-selected viewport.
        if self._limits() != limits:
            ax.set_xlim(*limits[0])
            ax.set_ylim(*limits[1])
        if changed:
            self.canvas.draw_idle()


class PlanningWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.controller = parent
        self.setWindowTitle('NOMAD | 플래닝 개발')
        self.resize(1320, 900)
        self.setStyleSheet('QWidget {background:#111827;color:#e2e8f0;} QPushButton {padding:7px 13px;background:#334155;border-radius:4px;} QLineEdit {padding:6px;background:#1e293b;}')
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('Pi4'))
        self.host = QtWidgets.QLineEdit()
        self.host.setMaximumWidth(240)
        self.host.textEdited.connect(self._set_host)
        row.addWidget(self.host)
        self.connect_button = QtWidgets.QPushButton('플래너 연결')
        self.connect_button.clicked.connect(lambda: self._action('connect_planner'))
        row.addWidget(self.connect_button)
        self.send_button = QtWidgets.QPushButton('선택 목표 보내기')
        self.send_button.clicked.connect(lambda: self._action('send_goal'))
        row.addWidget(self.send_button)
        self.cancel_button = QtWidgets.QPushButton('목표 취소')
        self.cancel_button.clicked.connect(lambda: self._action('cancel_goal'))
        row.addWidget(self.cancel_button)
        self.arm_button = QtWidgets.QPushButton('출력 허용')
        self.arm_button.setToolTip('출력 허용을 한 번 누르면 재계획·목표 변경에도 유지합니다. 출력 정지 또는 실행 세션 종료 시 해제합니다. 입력·경로가 유효하지 않으면 정지하고, 회복되면 재개합니다.')
        self.arm_button.clicked.connect(lambda: self._action('arm_output'))
        row.addWidget(self.arm_button)
        self.disarm_button = QtWidgets.QPushButton('출력 정지')
        self.disarm_button.setStyleSheet('QPushButton {background:#991b1b;color:white;}')
        self.disarm_button.clicked.connect(lambda: self._action('disarm_output'))
        row.addWidget(self.disarm_button)
        row.addStretch(1)
        layout.addLayout(row)
        self.status = QtWidgets.QLabel('플래너: 미수신')
        self.status.setWordWrap(True)
        self.selection = QtWidgets.QLabel('왼쪽 클릭: 목표 선택 · 휠: 확대/축소 · 오른쪽 드래그: 이동')
        layout.addWidget(self.status)
        self.gpp_status = QtWidgets.QLabel('GPP · 상태 미수신')
        self.lpp_status = QtWidgets.QLabel('LPP · 상태 미수신')
        self.gpp_status.setStyleSheet('color:#c084fc;')
        self.lpp_status.setStyleSheet('color:#fbbf24;')
        for label in (self.gpp_status, self.lpp_status):
            stable_metadata(label, rows=2)
            layout.addWidget(label)
        layout.addWidget(self.selection)
        controls = QtWidgets.QHBoxLayout()
        self.map = PlanningMap(components)
        self.map.goal_selected.connect(self._select_goal)
        for title, action in (('+ 확대', lambda: self.map.zoom(.7)), ('− 축소', lambda: self.map.zoom(1/.7)),
                              ('전체보기', self.map.fit_all), ('차량 중심', self.map.center_vehicle), ('20m 보기', self.map.reset_twenty)):
            button = QtWidgets.QPushButton(title)
            button.clicked.connect(action)
            controls.addWidget(button)
        controls.addStretch(1)
        controls.addWidget(QtWidgets.QLabel('지도 갱신 시 확대/이동 유지'))
        layout.addLayout(controls)
        layout.addWidget(self.map, 1)
        self.detail = QtWidgets.QLabel('기존 메인 수신 공유 · 별도 보드 수신 없음 · 출력 상태 미수신')
        self.detail.setWordWrap(True)
        stable_metadata(self.detail, rows=3)
        layout.addWidget(self.detail)
        self.setCentralWidget(body)
        attach_module_logs(self)
        self._image_key = None
        self._shown_received = None
        self._readonly = True

    def append_module_log(self, event):
        return self.log_panel.append_event(event)

    def _set_host(self, value):
        if self.controller is not None and not self._readonly:
            self.controller.planning_host.setText(value)

    def _action(self, name):
        if self.controller is not None and not self._readonly:
            getattr(self.controller, name)()

    def _select_goal(self, x, y, frame):
        if self.controller is not None and not self._readonly and self.map.goal_selection_enabled:
            self.controller.select_goal(x, y, frame)
            self.update_snapshot(self.controller.developer_snapshot())

    def update_snapshot(self, snapshot):
        update_module_logs(self, snapshot, "planning")
        now = snapshot.get('now', time.monotonic())
        ui = snapshot.get('planning_ui', {})
        self._readonly = bool(ui.get('spectator', True) or ui.get('no_ssh', True))
        host = ui.get('host', '')
        if self.host.text() != host:
            self.host.setText(host)
        self.host.setEnabled(not self._readonly and not ui.get('client_active', False))
        self.connect_button.setEnabled(not self._readonly)
        self.connect_button.setText('플래너 종료' if ui.get('client_active') else '플래너 연결')
        self.send_button.setEnabled(not self._readonly and ui.get('send_enabled', False))
        self.cancel_button.setEnabled(not self._readonly and ui.get('cancel_enabled', False))
        self.arm_button.setEnabled(not self._readonly and ui.get('arm_enabled', False))
        self.disarm_button.setEnabled(not self._readonly and ui.get('disarm_enabled', False))
        self.status.setText(('관전 · 읽기 전용 | ' if ui.get('spectator') else '') + str(ui.get('message', '플래너 미수신')))
        event = snapshot.get('image_meta', {}).get('costmap', {})
        image = snapshot.get('images', {}).get('costmap')
        received = snapshot.get('received', {})
        if isinstance(image, QtGui.QImage) and not image.isNull() and not event.get('decode_error'):
            key = (image.cacheKey(), event.get('t'), received.get('image:costmap'))
            if key != self._image_key and self.map.set_image(image, event):
                self._image_key, self._shown_received = key, received.get('image:costmap')
        self.map.map_age = None if self._shown_received is None else max(0., now-self._shown_received)
        # A last valid displayed map still provides real odom coordinates for
        # staging a goal. Freshness gates sending in the main controller; an
        # invalid replacement must neither refresh nor disable this old map.
        usable_map = (self.map.pixels is not None and self.map._frame() == 'odom'
                      and self.map.geometry(self.map.event) is not None)
        self.map.goal_selection_enabled = not self._readonly and usable_map
        history = snapshot.get('histories', {}).get('public', ())
        self.map.history = history if history and history[-1].get('frame_id') == self.map._frame() else []
        pose_received = received.get('pose:public')
        self.map.pose_age = None if pose_received is None else max(0., now-pose_received)
        self.map.forward_axis = snapshot.get('forward_axis', (1., 0., 0.))
        planning_age = received.get('telemetry:planning')
        from planning_client import planning_for_display, stage_summary
        self.map.planning = planning_for_display(snapshot.get('telemetry', {}).get('planning', {}),
            None if planning_age is None else now-planning_age)
        gpp_text, lpp_text = stage_summary(self.map.planning)
        set_text(self.gpp_status, gpp_text)
        set_text(self.lpp_status, lpp_text)
        self.gpp_status.setToolTip(gpp_text)
        self.lpp_status.setToolTip(lpp_text)
        self.map.selected_goal = ui.get('selected_goal')
        goal = self.map.selected_goal
        self.selection.setText((f'선택 목표 ({goal[0]:.2f}, {goal[1]:.2f}) m · {goal[2]} | ' if goal else '') + '왼쪽 클릭: 선택 · 휠: 확대/축소 · 오른쪽 드래그: 이동')
        if not self.map._view_initialized and self.map._latest() is not None:
            self.map.center_vehicle(reset=True)
        elif not self.map._view_initialized and self.map.geometry(self.map.event):
            g, yaw = self.map.geometry(self.map.event)
            center = Affine2D().rotate(yaw).translate(g['origin']['x'], g['origin']['y']).transform(
                [g['width']*g['resolution']/2, g['height']*g['resolution']/2])
            self.map._set_limits(center[0], center[1], 20.)
            self.map._view_initialized = True
        frame = self.map._frame() or '좌표 미수신'
        age = '미수신' if self.map.map_age is None else f'{self.map.map_age:.1f}초 전'
        if ui.get('spectator', True):
            selection_state = '관전 · 읽기 전용 · 목표 선택 불가'
        elif ui.get('no_ssh', True):
            selection_state = 'SSH 비활성 · 읽기 전용 · 목표 선택 불가'
        elif not usable_map:
            selection_state = ('지도 영상 해독 실패' if event.get('decode_error') else 'odom 지도 좌표 미수신') + ' · 목표 선택 불가'
        elif (self.map.map_age is None or self.map.map_age > 2 or event.get('decode_error')
              or self.map.geometry(event) is None or self.map.geometry(event)[0]['frame_id'] != 'odom'):
            selection_state = '지도 갱신 지연 · 목표 선택/전송 가능 · 주행은 플래너에서 판단'
        else:
            selection_state = '목표 선택 가능'
        self.map.setToolTip(selection_state)
        from planning_client import output_summary
        output = output_summary(self.map.planning) if self.map.planning else '출력 상태 미수신/오래됨 · 현재 허용 여부 확인 불가'
        self.detail.setText(f'지도 {frame} · {age} | {selection_state} | 메인 수신 공유 · 회색 여백은 미관측 | {output}')
        if self.isVisible():
            self.map.redraw()
