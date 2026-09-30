#!/usr/bin/env python3
"""Keyboard teleop with real key-release steering in a small Qt window."""

import sys

import rclpy
from geometry_msgs.msg import Twist
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget


class TeleopWindow(QWidget):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.left_down = False
        self.right_down = False

        self.setWindowTitle('NOMAD Gazebo 조종')
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMinimumWidth(390)
        self.status = QLabel()
        self.status.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('W/S: 속도 조절  |  A/D: 누르는 동안 조향\n'
                                'Space: 정지  |  Q: 종료'))
        layout.addWidget(self.status)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publish_command)
        self.timer.start(50)
        self.update_status()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == Qt.Key_W:
            self.speed = min(1.0, self.speed + 0.1)
        elif key == Qt.Key_S:
            self.speed = max(-0.6, self.speed - 0.1)
        elif key == Qt.Key_A:
            self.left_down = True
        elif key == Qt.Key_D:
            self.right_down = True
        elif key == Qt.Key_Space:
            self.speed = self.yaw_rate = 0.0
            self.left_down = self.right_down = False
            self.publish_command()
        elif key == Qt.Key_Q:
            self.close()
        else:
            super().keyPressEvent(event)
        self.update_status()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() == Qt.Key_A:
            self.left_down = False
        elif event.key() == Qt.Key_D:
            self.right_down = False
        else:
            super().keyReleaseEvent(event)
        self.update_status()

    def focusOutEvent(self, event):
        # Releasing a key in another window must not leave the car moving.
        self.left_down = self.right_down = False
        self.speed = self.yaw_rate = 0.0
        self.publish_command()
        self.update_status()
        super().focusOutEvent(event)

    def update_status(self):
        direction = '좌회전' if self.left_down and not self.right_down else (
            '우회전' if self.right_down and not self.left_down else '중앙')
        self.status.setText(f'속도: {self.speed:+.1f} m/s  |  조향: {direction}\n'
                            '키 입력은 이 창을 선택한 상태에서만 받습니다.')

    def publish_command(self):
        target = 0.55 * (int(self.left_down) - int(self.right_down))
        # Smoothly return to center instead of snapping the steering target.
        step = 0.08
        self.yaw_rate += max(-step, min(step, target - self.yaw_rate))
        message = Twist()
        message.linear.x = self.speed
        message.angular.z = self.yaw_rate
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = rclpy.create_node('nomad_gazebo_teleop')
    publisher = node.create_publisher(Twist, '/cmd_vel', 5)
    app = QApplication([sys.argv[0]])
    window = TeleopWindow(publisher)
    window.show()
    window.activateWindow()
    window.setFocus()
    try:
        return app.exec_()
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
