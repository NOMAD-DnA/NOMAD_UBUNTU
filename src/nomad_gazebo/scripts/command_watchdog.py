#!/usr/bin/env python3
"""NOMAD simulation command gate: wall-clock expiry even when Gazebo is paused."""
import math
import time


def bounded_command(linear, angular, max_linear=1.0, max_angular=0.8):
    if not all(math.isfinite(value) for value in (linear, angular)):
        return 0.0, 0.0
    return (max(-max_linear, min(max_linear, linear)),
            max(-max_angular, min(max_angular, angular)))


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    class Watchdog(Node):
        def __init__(self):
            super().__init__('nomad_cmd_watchdog')
            self.timeout = float(self.declare_parameter('timeout_s', .35).value)
            if not math.isfinite(self.timeout) or not 0.1 <= self.timeout <= 1.0:
                raise ValueError('Command timeout must be 0.1..1.0 seconds')
            self.command, self.last_received = (0.0, 0.0), -math.inf
            self.publisher = self.create_publisher(Twist, '/nomad/vehicle/cmd_vel', 5)
            self.subscription = self.create_subscription(Twist, '/cmd_vel', self.receive, 5)
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(.05, self.publish, clock=self.wall_clock)
            self.get_logger().info('Simulation command gate: /cmd_vel, 0.35s wall-clock expiry, 1m/s limit')

        def receive(self, message):
            self.command = bounded_command(message.linear.x, message.angular.z)
            self.last_received = time.monotonic()

        def publish(self, force_stop=False):
            linear, angular = (0.0, 0.0) if force_stop or time.monotonic()-self.last_received > self.timeout else self.command
            message = Twist(); message.linear.x = linear; message.angular.z = angular
            self.publisher.publish(message)

    rclpy.init()
    node = Watchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(force_stop=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
