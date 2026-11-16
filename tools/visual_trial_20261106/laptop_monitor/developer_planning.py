"""Planning developer view; consumes only the operator's shared snapshots."""
import math
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import FancyArrowPatch, Polygon
from matplotlib.collections import PolyCollection
from matplotlib.transforms import Affine2D
import numpy as np
from module_logs import attach_module_logs, update_module_logs
from render_utils import HistorySampler, history_signature, stable_metadata, set_text
from planning_footprint import load_footprint, cell_polygons
import developer_debug as dbg


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
        self.footprint = components['footprint'] if 'footprint' in components else load_footprint()
        self.show_footprint = self.show_footprint_cells = True
        self._footprint_key = None
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

    def closeup_vehicle(self):
        pose = self._latest()
        if pose is not None:
            self._set_limits(pose['x'], pose['y'], 2.)
            self._view_initialized = True

    def set_footprint_visible(self, checked):
        self.show_footprint = checked
        self.redraw()

    def set_cells_visible(self, checked):
        self.show_footprint_cells = checked
        self.redraw()

    def set_layer_visible(self, name, checked):
        """Show/hide an already received overlay; the data itself is unchanged."""
        artist = {'global': '_global_line', 'local': '_local_line', 'history': '_history_line'}.get(name)
        if artist is None or not self._artists_ready:
            return
        getattr(self, artist).set_visible(bool(checked))
        self.canvas.draw_idle()

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
        self._vehicle_artist = ax.scatter([], [], s=26, color='#22d3ee', edgecolors='white',
                                          label='OAK IMU reference', zorder=8)
        self._footprint_artist = Polygon(np.zeros((4, 2)), closed=True, fill=False,
                                         edgecolor='#67e8f9', linewidth=2.3,
                                         label='Current footprint', zorder=7)
        ax.add_patch(self._footprint_artist)
        self._footprint_artist.set_visible(False)
        self._front_line, = ax.plot([], [], color='#ffffff', lw=3.2, zorder=8)
        self._cell_artist = PolyCollection([], facecolors='none', edgecolors='#fb923c',
                                            linewidths=1.1, linestyles='--', zorder=4)
        ax.add_collection(self._cell_artist)
        self._heading_artist = FancyArrowPatch((0, 0), (0, 0), arrowstyle='-|>',
                                               mutation_scale=13, color='#22d3ee', lw=2, zorder=7)
        ax.add_patch(self._heading_artist)
        self._heading_artist.set_visible(False)
        self._waiting_artist = ax.text(.5, .5, 'WAITING FOR MAP COORDINATES',
                                       transform=ax.transAxes, color='#cbd5e1', ha='center')
        self._unknown_artist = ax.text(.015, .015, 'Outside map: UNKNOWN (not free)',
                                       transform=ax.transAxes, color='#cbd5e1', fontsize=9, zorder=10,
                                       bbox={'facecolor':'#111827', 'alpha':.75, 'edgecolor':'none'})
        self._footprint_note = ax.text(.015, .09, '', transform=ax.transAxes,
                                       color='#67e8f9', fontsize=9, va='bottom', zorder=10,
                                       bbox={'facecolor':'#111827', 'alpha':.85, 'edgecolor':'none'})
        self._legend = ax.legend(loc='upper right', facecolor='#111827', edgecolor='#475569',
                                 labelcolor='#e2e8f0', fontsize=9)
        # Legend swatches describe the overlay even before the first pose.
        for handle in self._legend.get_patches():
            handle.set_visible(True)
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
                try:
                    front = self.rotate_vector(pose, self.forward_axis)
                except (KeyError, TypeError, ValueError):
                    front = (math.nan, math.nan, math.nan)
                norm = math.hypot(front[0], front[1])
                if math.isfinite(norm) and norm >= 1e-3:
                    self._heading_artist.set_positions((pose['x'], pose['y']),
                        (pose['x']+.6*front[0]/norm, pose['y']+.6*front[1]/norm))
                    self._heading_artist.set_color(color)
                    visible_heading = True
            self._heading_artist.set_visible(visible_heading)
            self._pose_key = pose_key
            changed = True
        footprint_key = (pose_key, self._geometry_key, self.footprint,
                         self.show_footprint, self.show_footprint_cells)
        if footprint_key != self._footprint_key:
            self._update_footprint(pose, geometry, color)
            self._footprint_key = footprint_key
            changed = True
        # Image extent updates must not reset a user-selected viewport.
        if self._limits() != limits:
            ax.set_xlim(*limits[0])
            ax.set_ylim(*limits[1])
        if changed:
            self.canvas.draw_idle()

    def _update_footprint(self, pose, geometry, color):
        self._footprint_artist.set_visible(False)
        self._front_line.set_data([], [])
        self._cell_artist.set_verts([])
        if self.footprint is None:
            self._footprint_note.set_text('Recorded footprint unavailable · overlay hidden')
            return
        if not self.show_footprint:
            self._footprint_note.set_text('Current footprint: hidden')
            return
        if pose is None or geometry is None:
            self._footprint_note.set_text('Current footprint: waiting for matching pose / map')
            return
        try:
            front = self.rotate_vector(pose, self.forward_axis)
            if (not all(finite(v) for v in (pose['x'], pose['y'], *front))
                    or math.hypot(front[0], front[1]) < 1e-3):
                raise ValueError('Invalid horizontal heading')
            yaw = math.atan2(front[1], front[0])
        except (KeyError, ValueError, TypeError):
            self._footprint_note.set_text('Current footprint: heading unavailable')
            return
        corners = self.footprint.corners(pose['x'], pose['y'], yaw)
        fresh = self.pose_age is not None and self.pose_age <= 1.
        self._footprint_artist.set_xy(corners)
        self._footprint_artist.set_edgecolor('#67e8f9' if fresh else color)
        self._footprint_artist.set_linestyle('-' if fresh else '--')
        self._footprint_artist.set_visible(True)
        self._front_line.set_data([p[0] for p in corners[:2]], [p[1] for p in corners[:2]])
        self._front_line.set_color('#ffffff' if fresh else color)
        cell_note = 'cell outlines off'
        if self.show_footprint_cells:
            grid, origin_yaw = geometry
            cells = self.footprint.cells(pose['x'], pose['y'], yaw, grid, origin_yaw)
            if cells is None:
                cell_note = 'cell outlines unavailable (grid resolution / range)'
            else:
                self._cell_artist.set_verts(cell_polygons(cells, grid, origin_yaw))
                self._cell_artist.set_edgecolor('#fb923c' if fresh else '#94a3b8')
                cell_note = f'{len(cells)} touched cells on displayed grid'
        f = self.footprint
        self._footprint_note.set_text(
            f'CURRENT POSE' + ('' if fresh else ' · STALE / last received') +
            f'\nF {f.front:.2f} / R {f.rear:.2f} / L,R {f.half_width:.2f} m\n' + cell_note)


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
        row.addWidget(QtWidgets.QLabel('odom x/y'))
        self.goal_x_input, self.goal_y_input = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
        for spin in (self.goal_x_input, self.goal_y_input):
            spin.setRange(-1000., 1000.)
            spin.setDecimals(2)
            spin.setSingleStep(.1)
            spin.setSuffix(' m')
            spin.setToolTip('현재 odom 기준 목표 좌표. VIO 재시작·방향 초기화 뒤에는 다시 확인하세요.')
            row.addWidget(spin)
        self.manual_button = QtWidgets.QPushButton('좌표로 목표 지정')
        self.manual_button.setToolTip('입력한 odom x/y를 선택 목표로 지정합니다. 전송은 ‘선택 목표 보내기’로 따로 합니다.')
        self.manual_button.clicked.connect(self._manual_goal)
        row.addWidget(self.manual_button)
        self._shown_goal = None
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
        self.reset_button = QtWidgets.QPushButton('제어기 복구')
        self.reset_button.setToolTip('Pi4 제어기의 치명 잠금(고장 코드·전압·과속·과전류·바퀴 걸림·운영자 STOP)을 풉니다. '
                                     '차가 0.5초 이상 멈춰 있어야 하며, 확인 창을 거칩니다. 구동 명령은 보내지 않습니다.')
        self.reset_button.setStyleSheet('QPushButton {background:#92400e;color:white;}')
        self.reset_button.clicked.connect(lambda: self._action('reset_controller'))
        row.addWidget(self.reset_button)
        row.addStretch(1)
        layout.addLayout(row)
        self.status = QtWidgets.QLabel('플래너: 미수신')
        self.status.setWordWrap(True)
        self.selection = QtWidgets.QLabel('왼쪽 클릭: 목표 선택 · 휠: 확대/축소 · 오른쪽 드래그: 이동')
        layout.addWidget(self.status)
        self.reset_result = QtWidgets.QLabel('')
        self.reset_result.setWordWrap(True)
        self.reset_result.setStyleSheet('color:#fbbf24;')
        self.reset_result.setVisible(False)
        layout.addWidget(self.reset_result)
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
        self.follow_button = QtWidgets.QPushButton('사람 추종 작업창')
        self.follow_button.setToolTip('지도·인지 화면 미리보기. Jetson 사람 추종 모듈은 아직 연결되지 않았습니다.')
        self.follow_button.clicked.connect(self._open_follow_window)
        controls.addWidget(self.follow_button)
        controls.addStretch(1)
        controls.addWidget(QtWidgets.QLabel('지도 갱신 시 확대/이동 유지'))
        layout.addLayout(controls)
        footprint_controls = QtWidgets.QHBoxLayout()
        self.footprint_toggle = QtWidgets.QCheckBox('차량 윤곽')
        self.cells_toggle = QtWidgets.QCheckBox('검사 셀 테두리')
        self.footprint_toggle.setChecked(True)
        self.cells_toggle.setChecked(True)
        self.footprint_toggle.toggled.connect(self.map.set_footprint_visible)
        self.cells_toggle.toggled.connect(self.map.set_cells_visible)
        self.footprint_toggle.toggled.connect(self.cells_toggle.setEnabled)
        footprint_controls.addWidget(self.footprint_toggle)
        footprint_controls.addWidget(self.cells_toggle)
        self.layer_toggles = {}
        for name, title in (('global', 'GPP 경로'), ('local', 'LPP 궤적'), ('history', '이동 경로')):
            check = QtWidgets.QCheckBox(title)
            check.setChecked(True)
            check.setToolTip('이미 받은 겹침 표시만 켜고 끕니다. 플래너에는 아무것도 보내지 않습니다.')
            check.toggled.connect(lambda checked, key=name: self.map.set_layer_visible(key, checked))
            footprint_controls.addWidget(check)
            self.layer_toggles[name] = check
        self.closeup_button = QtWidgets.QPushButton('차량 근접 보기 · 2m')
        self.closeup_button.clicked.connect(self.map.closeup_vehicle)
        footprint_controls.addWidget(self.closeup_button)
        footprint_controls.addStretch(1)
        layout.addLayout(footprint_controls)
        f = self.map.footprint
        self.footprint_info = QtWidgets.QLabel(
            (f'청록 윤곽: {f.reference} 기준 앞 {f.front:.3f} · 뒤 {f.rear:.3f} · 좌우 각 {f.half_width:.3f}m' if f is not None else '기록 당시 윤곽 미저장 · 윤곽/겹침 셀 표시 안 함') +
            ' | 흰 선: 전면 · 주황 점선: 현재 차량 검사 셀\n'
            '전역 지도 위 현재 위치 표시 · LPP 정지는 지역 지도와 예측 궤적 전체로 판단')
        if (components or {}).get('footprint_note'):
            self.footprint_info.setText(self.footprint_info.text() + '\n' + components['footprint_note'])
        self.footprint_info.setWordWrap(True)
        self.footprint_info.setStyleSheet('color:#a5f3fc;font-size:12px;')
        stable_metadata(self.footprint_info, rows=3)
        self.footprint_info.setToolTip(
            '현재 Pi4 설정과 맞춘 표시용 기하입니다. 보드 기하를 변경하면 config/planning_footprint.json도 맞춰야 합니다.\n'
            '방향은 기존 OAK 카메라 전방축을 전체 자세로 회전해 사용합니다. 기준점은 차체 사각형 중심이 아닙니다.\n'
            '주황 셀은 표시 중인 전역 격자에서 보수적인 풋프린트 교차식으로 선택하며 장애물 확정 표시가 아닙니다.\n'
            '색상 영상만 수신하므로 비용 80 이상 여부를 별도 재판정하지 않습니다.\n'
            '지도와 위치 수신 시각은 다를 수 있고, 실패한 미래 궤적은 이 윤곽에 포함되지 않습니다.\n'
            'margin 0.10m는 제동 여유 설정이며 차량 사각형을 추가로 팽창시키지 않습니다.')
        layout.addWidget(self.footprint_info)
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
        self.transitions = dbg.TransitionLog(maxlen=60)
        self._gpp_stamps = []
        self._exec_samples = []
        self.debug = dbg.DebugTools(self, 'planning', 'NOMAD 플래닝 개발', (
            {'key': 'telemetry:planning', 'label': '플래너 상태', 'limit': 3.},
            {'key': 'image:costmap', 'label': '전역 지도 (코스트맵)', 'limit': 2., 'rate': ('jetson', 'costmap')},
            {'key': 'pose:public', 'label': '공용 위치', 'limit': 1., 'rate': ('pi', 'public')},
        ), log_dock=self.log_dock)
        self.debug.clear_hooks.append(self._clear_debug)

    def append_module_log(self, event):
        return self.log_panel.append_event(event)

    def _clear_debug(self):
        self.transitions.clear()
        self._gpp_stamps, self._exec_samples = [], []

    def _observe_planning(self, snapshot):
        """History of received planner fields; runs even while the display is frozen."""
        now = snapshot.get('now')
        if not dbg.finite(now):
            return
        if self._exec_samples and now < self._exec_samples[-1][0] - .5:
            self._clear_debug()
        report = snapshot.get('telemetry', {}).get('planning', {})
        received = snapshot.get('received', {}).get('telemetry:planning')
        if not report or not dbg.finite(received):
            return
        self.transitions.update(now, {
            '주행 상태': report.get('state'), 'GPP': report.get('gpp_state'), 'LPP': report.get('lpp_state'),
            '경로 유효': report.get('path_valid'), '실행 후보': report.get('executable'),
            'armed': report.get('armed'), 'actuator_enabled': report.get('actuator_enabled'),
            '출력 차단 사유': report.get('output_inhibit') or '없음', 'LPP 사유': report.get('lpp_reason'),
            '목표': '—' if not report.get('goal') else f"({report['goal'][0]:.2f}, {report['goal'][1]:.2f})"},
            dbg.wall_clock(report.get('t')))
        stamp = report.get('gpp_stamp_ns')
        if isinstance(stamp, int) and stamp > 0 and (not self._gpp_stamps or self._gpp_stamps[-1][1] != stamp):
            self._gpp_stamps.append((now, stamp))
        if not self._exec_samples or self._exec_samples[-1][2] != received:
            self._exec_samples.append((now, report.get('executable'), received))
        cutoff = now - dbg.WINDOW_S
        self._gpp_stamps = [row for row in self._gpp_stamps if row[0] >= cutoff][-400:]
        self._exec_samples = [row for row in self._exec_samples if row[0] >= cutoff][-2000:]

    def _debug_checks(self, snapshot):
        ui = snapshot.get('planning_ui', {})
        report = snapshot.get('telemetry', {}).get('planning', {})
        received = snapshot.get('received', {})
        now = snapshot.get('now', time.monotonic())
        planning_age = None if received.get('telemetry:planning') is None else max(0., now-received['telemetry:planning'])
        checks = [dbg.freshness('플래너 상태 수신', planning_age, 1., 3.)]
        if snapshot.get('replay'):
            checks.append(('플래너 연결', 'unknown', '기록 재생 · 연결 없음'))
        elif ui.get('spectator'):
            checks.append(('플래너 연결', 'unknown', '관전 · 운용 PC의 연결을 공유'))
        else:
            checks.append(('플래너 연결', 'ok' if ui.get('client_active') else 'warn',
                           '연결됨' if ui.get('client_active') else '이 PC에서 플래너 미연결'))
        if not report:
            checks.append(('주행 상태', 'fail', '플래너 상태 미수신'))
        else:
            state = str(report.get('state', '—'))
            level = ('fail' if any(word in state.upper() for word in ('FAULT', 'ERROR', 'ESTOP', 'STOP'))
                     else 'ok' if state.upper() in ('DRIVING', 'TRACKING', 'MOVING', 'ARRIVED', 'IDLE') else 'warn')
            checks.append(('주행 상태', level, f"{state} · {dbg.plain(report.get('reason'))}"))
            valid = report.get('path_valid')
            checks.append(('GPP 전역 경로', 'ok' if valid is True else 'fail' if valid is False else 'unknown',
                           f"{report.get('gpp_state', '상태 미제공')} · {dbg.plain(report.get('gpp_reason'))}"))
            executable = report.get('executable')
            checks.append(('LPP 실행 후보', 'ok' if executable is True else 'fail' if executable is False else 'unknown',
                           f"{report.get('lpp_state', '상태 미제공')} · {dbg.plain(report.get('lpp_reason'))}"))
            path_stamp, stamp = report.get('gpp_stamp_ns'), report.get('t')
            if isinstance(path_stamp, int) and path_stamp > 0 and dbg.finite(stamp):
                path_age = max(0., stamp - path_stamp/1e9)
                checks.append(('GPP 경로 기준 시각', 'ok' if path_age <= 5. else 'warn',
                               f'플래너 상태 시각보다 {path_age:.1f}초 전 경로 (5초 초과 시 주의)'))
            from planning_client import output_summary
            armed, enabled, inhibit = report.get('armed'), report.get('actuator_enabled'), report.get('output_inhibit')
            level = 'ok' if armed is True and enabled is True and not inhibit else 'warn'
            if report.get('plan_only', True):
                level = 'unknown'
            checks.append(('출력 허용 · 구동', level, output_summary(report)))
            checks.append(('출력 차단 사유', 'warn' if inhibit else 'ok', inhibit or '없음'))
        map_received = self._shown_received
        map_age = None if map_received is None else max(0., now-map_received)
        frame = self.map._frame()
        checks.append(dbg.freshness('전역 지도 (표시 중)', map_age, 2., 5., note=f'좌표계 {frame or "—"}'))
        checks.append(dbg.freshness('공용 위치 수신', None if received.get('pose:public') is None
                                    else max(0., now-received['pose:public']), 1., 3.))
        checks.append(dbg.log_check(self.log_panel))
        return checks

    def _debug_sections(self, snapshot):
        report = snapshot.get('telemetry', {}).get('planning', {})
        now = snapshot.get('now', time.monotonic())
        path, trajectory = report.get('path') or [], report.get('trajectory') or []
        goal = report.get('goal')
        pose = self.map._latest()
        values = []
        def add(name, value):
            values.append([name, dbg.plain(value, 400)])
        for key, name in (('state', '주행 상태'), ('reason', '상태 사유'), ('gpp_state', 'GPP 상태'), ('gpp_reason', 'GPP 사유'),
                          ('lpp_state', 'LPP 상태'), ('lpp_reason', 'LPP 사유'), ('path_valid', '경로 유효'),
                          ('executable', '실행 후보 유효'), ('plan_only', '경로 확인 전용'), ('armed', 'armed'),
                          ('actuator_enabled', 'actuator_enabled'), ('output_inhibit', '출력 차단 사유'),
                          ('output_mode', '출력 방식'), ('feedback_source', '피드백 출처'), ('prototype_output', '임시 환산 출력')):
            if report:
                add(name, report.get(key) if report.get(key) != '' else '없음')
        if report:
            add('목표 (odom)', f'({goal[0]:.2f}, {goal[1]:.2f}) m' if goal else '없음')
            if goal and pose:
                add('차량→목표 직선 거리', f"{math.hypot(goal[0]-pose['x'], goal[1]-pose['y']):.2f} m")
            add('GPP 경로 길이 · 점 수', f'{dbg.path_length(path):.2f} m · {len(path)}점 (표시용 최대 600점)')
            if path and goal:
                add('GPP 경로 끝 ↔ 목표', f'{math.hypot(path[-1][0]-goal[0], path[-1][1]-goal[1]):.2f} m')
            if path and pose:
                add('차량 ↔ GPP 경로 최단 거리', f"{min(math.hypot(p[0]-pose['x'], p[1]-pose['y']) for p in path):.2f} m (경로 점 기준)")
            add('LPP 궤적 길이 · 점 수', f'{dbg.path_length(trajectory):.2f} m · {len(trajectory)}점')
            stamp = report.get('t')
            for key, name in (('gpp_stamp_ns', 'GPP 경로 기준 시각'), ('stamp_ns', '입력 기준 시각'), ('output_stamp_ns', '출력 시각')):
                value = report.get(key)
                if isinstance(value, int) and value > 0 and dbg.finite(stamp):
                    add(name, f'{dbg.wall_clock(value/1e9)} · 상태 시각보다 {stamp-value/1e9:.2f}초 전')
            add('플래너 상태 시각', dbg.wall_clock(stamp))
        samples = self._exec_samples
        blocked = [row for row in samples if row[1] is False]
        stats = [['GPP 재계획 (새 경로 기준 시각)', f'{max(0, len(self._gpp_stamps)-1)}회'],
                 ['LPP 실행 후보 없음 비율', f'{len(blocked)/len(samples)*100:.0f}% ({len(blocked)}/{len(samples)} 수신)' if samples else '—'],
                 ['주행 상태 변화', f"{self.transitions.count_since(now, '주행 상태')}회"],
                 ['LPP 상태 변화', f"{self.transitions.count_since(now, 'LPP')}회"],
                 ['출력 차단 사유 변화', f"{self.transitions.count_since(now, '출력 차단 사유')}회"],
                 ['플래너 상태 수신 (화면 관측)', f'{len(samples)}회 · 화면 갱신보다 빠른 수신은 합쳐 셉니다']]
        return {
            '플래너 주요 값': (['항목', '현재 값'], values, '플래너가 보낸 값 그대로입니다. 거리·길이는 받은 경로 점으로 이 PC가 계산합니다.'),
            '최근 60초 통계': (['항목', '값'], stats, ''),
            '상태 변화 기록 · 최근 60건': (['플래너 시각', '항목', '이전', '현재'], self.transitions.table(),
                                     '받은 상태가 바뀔 때만 한 줄 추가합니다. 위가 최신입니다.'),
            '플래너 프로세스 · 보드 부하 계측': (dbg.PROCESS_HEADERS, dbg.module_processes(snapshot, ('planning_gpp_lpp',)),
                                         dbg.PROCESS_NOTE),
        }

    def update_snapshot(self, snapshot):
        self.debug.observe(snapshot)
        self._observe_planning(snapshot)
        if self.debug.frozen:
            # Command buttons must keep following the live state even while
            # the map and numbers are frozen (e.g. 출력 정지 stays usable).
            self._update_controls(snapshot)
            return
        self._update_view(snapshot)
        self.debug.publish(snapshot, self._debug_checks(snapshot), self._debug_sections(snapshot))

    def _open_follow_window(self):
        # Opening a shared display is also allowed in replay/spectator mode.
        if self.controller is not None:
            self.controller.open_developer_window("person_follow")

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

    def _manual_goal(self):
        # No map requirement: a typed goal may be staged before a map arrives.
        if self.controller is not None and not self._readonly:
            self.controller.manual_goal(self.goal_x_input.value(), self.goal_y_input.value())
            self.update_snapshot(self.controller.developer_snapshot())

    def _update_controls(self, snapshot):
        ui = snapshot.get('planning_ui', {})
        self._readonly = bool(ui.get('spectator', True) or ui.get('no_ssh', True))
        host = ui.get('host', '')
        if self.host.text() != host:
            self.host.setText(host)
        self.host.setEnabled(not self._readonly and not ui.get('client_active', False))
        self.connect_button.setEnabled(not self._readonly)
        self.connect_button.setText('플래너 종료' if ui.get('client_active') else '플래너 연결')
        for widget in (self.goal_x_input, self.goal_y_input, self.manual_button):
            widget.setEnabled(not self._readonly and ui.get('manual_enabled', False))
        goal = ui.get('selected_goal')
        if goal != self._shown_goal:
            self._shown_goal = goal
            if goal and not (self.goal_x_input.hasFocus() or self.goal_y_input.hasFocus()):
                for spin, value in ((self.goal_x_input, goal[0]), (self.goal_y_input, goal[1])):
                    spin.blockSignals(True)
                    spin.setValue(value)
                    spin.blockSignals(False)
        self.send_button.setEnabled(not self._readonly and ui.get('send_enabled', False))
        self.cancel_button.setEnabled(not self._readonly and ui.get('cancel_enabled', False))
        self.arm_button.setEnabled(not self._readonly and ui.get('arm_enabled', False))
        self.disarm_button.setEnabled(not self._readonly and ui.get('disarm_enabled', False))
        self.reset_button.setEnabled(not self._readonly and ui.get('reset_enabled', False))
        self.reset_button.setText('제어기 복구 중…' if ui.get('reset_busy') else '제어기 복구')
        reset_text = str(ui.get('reset_result', '') or '')
        if self.reset_result.text() != reset_text:
            self.reset_result.setText(reset_text)
        self.reset_result.setVisible(bool(reset_text))
        self.status.setText(('관전 · 읽기 전용 | ' if ui.get('spectator') else '') + str(ui.get('message', '플래너 미수신')))

    def _update_view(self, snapshot):
        update_module_logs(self, snapshot, "planning")
        now = snapshot.get('now', time.monotonic())
        ui = snapshot.get('planning_ui', {})
        self._update_controls(snapshot)
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
        pose_age = '미수신' if self.map.pose_age is None else f'{self.map.pose_age:.1f}초 전'
        self.detail.setText(f'전역 지도 {frame} · {age} | 위치 {pose_age} | {selection_state} | 메인 수신 공유 · 회색 여백은 미관측 | {output}')
        if self.isVisible():
            self.map.redraw()
