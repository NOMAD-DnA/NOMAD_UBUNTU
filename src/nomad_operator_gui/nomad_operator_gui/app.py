"""PC-only NOMAD operator window. No vehicle process is started here."""

import argparse
import os
import queue
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QVBoxLayout, QWidget,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

from .grid import GridGeometry
from .remote import Board, RemoteError, start_sequence, stop_sequence


class RosWorker(QThread):
    image_received = pyqtSignal(object)
    grid_received = pyqtSignal(object)
    odom_received = pyqtSignal(object)
    error = pyqtSignal(str)
    goal_sent = pyqtSignal(float, float)
    goal_ready = pyqtSignal(bool)

    def __init__(self, topics):
        super().__init__()
        self.topics = topics
        self.commands = queue.Queue()
        self.running = True
        self.last_primary_image = 0.0

    def run(self):
        node = None
        try:
            rclpy.init(args=None)
            node = rclpy.create_node('nomad_operator_gui')
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            def on_primary_image(msg):
                self.last_primary_image = time.monotonic()
                self.image_received.emit(msg)

            def on_fallback_image(msg):
                if time.monotonic() - self.last_primary_image > 2.0:
                    self.image_received.emit(msg)

            node.create_subscription(Image, self.topics.image, on_primary_image, qos_profile_sensor_data)
            if self.topics.image_fallback and self.topics.image_fallback != self.topics.image:
                node.create_subscription(Image, self.topics.image_fallback, on_fallback_image,
                                         qos_profile_sensor_data)
            node.create_subscription(OccupancyGrid, self.topics.grid, lambda msg: self.grid_received.emit(msg), map_qos)
            node.create_subscription(Odometry, self.topics.odom, lambda msg: self.odom_received.emit(msg), 10)
            publisher = node.create_publisher(PoseStamped, self.topics.goal, 10)
            last_ready = None
            while self.running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                ready = publisher.get_subscription_count() > 0
                if ready != last_ready:
                    self.goal_ready.emit(ready)
                    last_ready = ready
                try:
                    x, y, frame = self.commands.get_nowait()
                except queue.Empty:
                    continue
                if not ready:
                    self.error.emit('목표 수신 노드가 연결되지 않아 발행하지 않았습니다')
                    continue
                msg = PoseStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = frame
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.orientation.w = 1.0
                publisher.publish(msg)
                self.goal_sent.emit(x, y)
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def send_goal(self, x, y, frame):
        self.commands.put((x, y, frame))

    def stop(self):
        self.running = False
        self.wait(2000)


class BoardWorker(QThread):
    progress = pyqtSignal(str)
    completed = pyqtSignal(object)

    def __init__(self, action, boards, domain, owned):
        super().__init__()
        self.action = action
        self.boards = boards
        self.domain = domain
        self.owned = dict(owned)

    def run(self):
        try:
            if self.action == 'start':
                owned = dict(self.owned)
                start_sequence(self.boards, self.domain, self.progress.emit, owned)
                self.completed.emit({'owned': owned, 'error': ''})
            else:
                errors = stop_sequence(self.boards, self.domain, self.owned, self.progress.emit)
                self.completed.emit({'owned': self.owned, 'error': '; '.join(errors)})
        except RemoteError as exc:
            self.completed.emit({'owned': owned if self.action == 'start' else self.owned, 'error': str(exc)})


class MapView(QWidget):
    selected = pyqtSignal(int, int)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(520, 420)
        self.geometry = None
        self.data = None
        self.pixmap = None
        self.selected_cell = None
        self.vehicle = None

    def set_grid(self, msg):
        info = msg.info
        geo = GridGeometry(info.width, info.height, info.resolution,
                           info.origin.position.x, info.origin.position.y)
        quat = info.origin.orientation
        if not geo.valid(len(msg.data)) or abs(quat.x) > 1e-4 or abs(quat.y) > 1e-4 or abs(quat.z) > 1e-4 or abs(abs(quat.w) - 1) > 1e-4:
            return False
        # Index 0 is unknown. Indices 1..101 represent occupancy values 0..100.
        pixels = bytes(0 if value < 0 else min(100, value) + 1 for value in msg.data)
        image = QImage(pixels, geo.width, geo.height, geo.width, QImage.Format_Indexed8)
        colors = [QColor(110, 110, 110).rgb()]
        colors += [QColor(max(30, 245 - 2 * v), max(30, 245 - 2 * v), max(30, 245 - 2 * v)).rgb() for v in range(101)]
        image.setColorTable(colors)
        previous_geometry = self.geometry
        previous_cell = self.selected_cell
        self.pixmap = QPixmap.fromImage(image.mirrored(False, True).copy())
        self.geometry = geo
        self.data = msg.data
        self.selected_cell = None
        if previous_geometry == geo and previous_cell is not None:
            col, row = previous_cell
            if 0 <= col < geo.width and 0 <= row < geo.height:
                value = msg.data[row * geo.width + col]
                if 0 <= value < 80:
                    self.selected_cell = previous_cell
        self.update()
        return True

    def image_rect(self):
        if not self.pixmap:
            return None
        scale = min(self.width() / self.pixmap.width(), self.height() / self.pixmap.height())
        width = self.pixmap.width() * scale
        height = self.pixmap.height() * scale
        return ((self.width() - width) / 2, (self.height() - height) / 2, width, height)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(35, 39, 44))
        rect = self.image_rect()
        if rect is None:
            painter.setPen(Qt.white)
            painter.drawText(self.rect(), Qt.AlignCenter, '그리드맵 수신 대기 중')
            return
        left, top, width, height = rect
        painter.drawPixmap(int(left), int(top), int(width), int(height), self.pixmap)
        if self.vehicle is not None:
            fx, fy = self.geometry.fraction_from_world(*self.vehicle)
            if 0 <= fx <= 1 and 0 <= fy <= 1:
                painter.setPen(QPen(QColor(0, 120, 255), 3))
                painter.setBrush(QColor(0, 120, 255))
                painter.drawEllipse(int(left + fx * width) - 5, int(top + fy * height) - 5, 10, 10)
        if self.selected_cell is not None:
            x, y = self.geometry.center(*self.selected_cell)
            fx, fy = self.geometry.fraction_from_world(x, y)
            painter.setPen(QPen(QColor(255, 210, 0), 3))
            painter.drawEllipse(int(left + fx * width) - 7, int(top + fy * height) - 7, 14, 14)

    def mousePressEvent(self, event):
        rect = self.image_rect()
        if rect is None or event.button() != Qt.LeftButton:
            return
        left, top, width, height = rect
        cell = self.geometry.cell_from_fraction((event.x() - left) / width, (event.y() - top) / height)
        if cell is not None:
            self.selected.emit(*cell)


class OperatorWindow(QMainWindow):
    def __init__(self, topics):
        super().__init__()
        self.setWindowTitle('NOMAD 실차 운영 화면')
        self.resize(1250, 700)
        self.last_image = 0.0
        self.last_grid = 0.0
        self.pending_image = None
        self.map_frame = ''
        self.planner_connected = False
        self.map = MapView()
        self.video = QLabel('전방 영상 수신 대기 중')
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(500, 420)
        self.video.setStyleSheet('background: #23272c; color: white;')
        self.video_pixmap = None
        self.status = QLabel('토픽 수신 대기 중')
        self.selection = QLabel('목적지: 지도에서 선택하세요')
        self.send_button = QPushButton('선택한 목적지 보내기')
        self.send_button.setEnabled(False)
        self.send_button.clicked.connect(self.send_goal)
        self.domain = os.environ.get('ROS_DOMAIN_ID', '42')
        self.owned_runs = {}
        self.board_worker = None
        self.board_status = QLabel('보드: Jetson 연결 준비 전' if not os.environ.get('NOMAD_JETSON_HOST') or not os.environ.get('NOMAD_JETSON_SCRIPT') else '보드: 시작 전')
        self.start_boards_button = QPushButton('세 보드 자율주행 기능 시작')
        self.stop_boards_button = QPushButton('이번 실행 중지')
        self.start_boards_button.clicked.connect(self.start_boards)
        self.stop_boards_button.clicked.connect(self.stop_boards)
        self.stop_boards_button.setEnabled(False)
        left = QVBoxLayout()
        left.addWidget(QLabel('OAK 전방 카메라'))
        left.addWidget(self.video, 1)
        right = QVBoxLayout()
        right.addWidget(QLabel('주행 그리드맵 (파랑: 차량, 노랑: 선택 목적지)'))
        right.addWidget(self.map, 1)
        right.addWidget(self.selection)
        right.addWidget(self.send_button)
        panes = QHBoxLayout()
        panes.addLayout(left, 1)
        panes.addLayout(right, 1)
        root = QVBoxLayout()
        root.addLayout(panes, 1)
        buttons = QHBoxLayout()
        buttons.addWidget(self.start_boards_button)
        buttons.addWidget(self.stop_boards_button)
        buttons.addWidget(self.board_status, 2)
        root.addLayout(buttons)
        root.addWidget(self.status)
        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)
        self.map.selected.connect(self.select_cell)
        self.worker = RosWorker(topics)
        self.worker.image_received.connect(self.on_image)
        self.worker.grid_received.connect(self.on_grid)
        self.worker.odom_received.connect(self.on_odom)
        self.worker.error.connect(lambda message: self.status.setText('ROS 오류: ' + message))
        self.worker.goal_sent.connect(lambda x, y: self.status.setText(f'목적지 토픽 발행: ({x:.2f}, {y:.2f}) m — 주행 시작 확인은 별도 필요'))
        self.worker.goal_ready.connect(self.on_goal_ready)
        self.worker.start()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def boards(self):
        return [
            Board('Pi 5', os.environ.get('NOMAD_PI5_HOST', ''),
                  os.environ.get('NOMAD_PI5_USER', 'user'),
                  os.environ.get('NOMAD_PI5_SCRIPT', '')),
            Board('Jetson', os.environ.get('NOMAD_JETSON_HOST', ''),
                  os.environ.get('NOMAD_JETSON_USER', 'nvidia'),
                  os.environ.get('NOMAD_JETSON_SCRIPT', '')),
            Board('Pi 4', os.environ.get('NOMAD_PI4_HOST', ''),
                  os.environ.get('NOMAD_PI4_USER', 'user'),
                  os.environ.get('NOMAD_PI4_SCRIPT', '')),
        ]

    def run_board_action(self, action):
        if self.board_worker is not None and self.board_worker.isRunning():
            return
        self.board_worker = BoardWorker(action, self.boards(), self.domain, self.owned_runs)
        self.board_worker.progress.connect(self.board_status.setText)
        self.board_worker.completed.connect(self.on_board_complete)
        self.start_boards_button.setEnabled(False)
        self.stop_boards_button.setEnabled(False)
        self.board_worker.start()

    def start_boards(self):
        if not os.environ.get('NOMAD_JETSON_HOST') or not os.environ.get('NOMAD_JETSON_SCRIPT'):
            self.board_status.setText('Jetson 연결 준비 전: 담당자 스크립트 설치 후 사용할 수 있습니다')
            return
        answer = QMessageBox.question(
            self, '실차 구동 확인',
            'Pi 5 → Jetson → Pi 4를 시작합니다. Pi 4 Control과 VESC 구동도 활성화됩니다. '
            '차량 주변과 정지 수단을 확보했습니까?',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer == QMessageBox.Yes:
            self.run_board_action('start')

    def stop_boards(self):
        self.run_board_action('stop')

    def on_board_complete(self, result):
        self.owned_runs = result['owned']
        self.start_boards_button.setEnabled(not bool(self.owned_runs))
        self.stop_boards_button.setEnabled(bool(self.owned_runs))
        if result['error']:
            self.board_status.setText('원격 실행 오류: ' + result['error'])
        elif self.owned_runs:
            self.board_status.setText('Jetson 영상·지도 준비 확인, Pi 4 시작 요청 완료; 주행 상태는 별도 확인')
        else:
            self.board_status.setText('이번 실행 중지 완료')

    def on_image(self, msg):
        self.pending_image = msg
        self.last_image = time.monotonic()

    def on_grid(self, msg):
        if msg.header.frame_id != 'odom':
            self.status.setText(f'지도 좌표계 {msg.header.frame_id!r}: 현재 GUI는 odom 지도만 지원')
            return
        if self.map.set_grid(msg):
            self.map_frame = msg.header.frame_id
            self.last_grid = time.monotonic()
            if self.map.selected_cell is None:
                self.send_button.setEnabled(False)
                self.selection.setText('목적지: 지도에서 선택하세요')
        else:
            self.status.setText('지도 형식 오류: 크기, 해상도 또는 회전된 origin 확인')

    def on_odom(self, msg):
        if msg.header.frame_id == 'odom':
            self.map.vehicle = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.map.update()

    def on_goal_ready(self, ready):
        self.planner_connected = ready
        self.send_button.setEnabled(ready and self.map.selected_cell is not None)

    def select_cell(self, col, row):
        value = self.map.data[row * self.map.geometry.width + col]
        if value < 0 or value >= 80:
            self.selection.setText(f'선택 불가: 미관측 또는 차단 셀 (값 {value})')
            self.map.selected_cell = None
            self.send_button.setEnabled(False)
        else:
            self.map.selected_cell = (col, row)
            x, y = self.map.geometry.center(col, row)
            self.selection.setText(f'목적지 선택: x={x:.2f} m, y={y:.2f} m (비용 {value})')
            self.send_button.setEnabled(self.planner_connected)
        self.map.update()

    def send_goal(self):
        if self.map.selected_cell is None or self.map_frame != 'odom' or not self.planner_connected:
            return
        x, y = self.map.geometry.center(*self.map.selected_cell)
        self.worker.send_goal(x, y, self.map_frame)
        self.send_button.setEnabled(False)
        self.selection.setText(f'발행 요청: x={x:.2f} m, y={y:.2f} m')

    def refresh(self):
        if self.pending_image is not None:
            msg, self.pending_image = self.pending_image, None
            try:
                if msg.encoding in ('rgb8', 'bgr8'):
                    image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, QImage.Format_RGB888)
                    if msg.encoding == 'bgr8':
                        image = image.rgbSwapped()
                elif msg.encoding == 'mono8':
                    image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, QImage.Format_Grayscale8)
                else:
                    self.video.setText('지원하지 않는 영상 형식: ' + msg.encoding)
                    image = None
                if image is not None and not image.isNull():
                    self.video_pixmap = QPixmap.fromImage(image.copy())
            except Exception as exc:
                self.video.setText('영상 변환 오류: ' + str(exc))
        if self.video_pixmap is not None:
            self.video.setPixmap(self.video_pixmap.scaled(self.video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        now = time.monotonic()
        image_state = '수신 중' if now - self.last_image < 3 else '대기/끊김'
        grid_state = '수신 중' if now - self.last_grid < 10 else '대기/오래됨'
        if not self.status.text().startswith(('ROS 오류:', '지도 ', '목적지 토픽 발행:', '지도 형식')):
            planner_state = '연결됨' if self.planner_connected else '연결 대기'
            self.status.setText(f'전방 영상: {image_state}  |  그리드맵: {grid_state}  |  목표 수신 노드: {planner_state}')

    def closeEvent(self, event):
        if self.board_worker is not None and self.board_worker.isRunning():
            self.board_status.setText('보드 작업이 끝난 뒤 창을 닫으세요')
            event.ignore()
            return
        if self.owned_runs:
            self.board_status.setText('이번 실행 중지 버튼으로 보드를 중지한 뒤 창을 닫으세요')
            event.ignore()
            return
        self.timer.stop()
        self.worker.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description='NOMAD PC operator GUI')
    parser.add_argument('--image', default='/oak/rgb/image_raw')
    parser.add_argument('--image-fallback', default='/perception/semantic/input_image')
    parser.add_argument('--grid', default='/nomad/navigation_costmap')
    parser.add_argument('--odom', default='/nomad/localization/odometry')
    parser.add_argument('--goal', default='/goal_pose')
    topics = parser.parse_args()
    app = QApplication(sys.argv[:1])
    window = OperatorWindow(topics)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
