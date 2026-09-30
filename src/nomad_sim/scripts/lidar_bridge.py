#!/usr/bin/env python3
"""Expose a range-limited G2-like scan without changing upstream MVSim."""
from array import array
import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def validate_profile(min_range, max_range, scan_period):
    if not all(math.isfinite(value) for value in (min_range, max_range, scan_period)):
        raise ValueError("LiDAR clipping limits and period must be finite")
    if not 0 <= min_range < max_range or scan_period <= 0:
        raise ValueError("Invalid LiDAR clipping limits or period")


def scan_message(source, min_range, max_range, scan_period):
    """Retain snapshot timestamps/angles; do not invent sequential ray timing."""
    validate_profile(min_range, max_range, scan_period)
    message = copy.deepcopy(source)
    # Compare float32 samples against float32 limits, including the 0.12 m edge.
    message.range_min, message.range_max = array("f", [min_range, max_range])
    def clipped(value):
        # REP-117: -inf = too close, +inf = no return, NaN = invalid/error.
        if not math.isfinite(value):
            return value
        if value <= 0:
            return math.nan
        if value < message.range_min:
            return -math.inf
        if value >= message.range_max:
            return math.inf
        return value

    message.ranges = array("f", (clipped(value) for value in source.ranges))
    # MVSim also uses maxRange for no-hit rays: >= max is intentionally invalid.
    message.scan_time = scan_period
    # All rays were rendered at the same pose/time, unlike a rotating real G2.
    message.time_increment = 0.0
    return message


class LidarBridge(Node):
    def __init__(self):
        super().__init__("lidar_sim_bridge")
        self.min_range = float(self.declare_parameter("min_range_m", 0.12).value)
        self.max_range = float(self.declare_parameter("max_range_m", 12.0).value)
        scan_hz = float(self.declare_parameter("scan_hz", 10.0).value)
        if not math.isfinite(scan_hz) or scan_hz <= 0:
            raise ValueError("LiDAR scan_hz must be finite and positive")
        self.scan_period = 1.0 / scan_hz
        validate_profile(self.min_range, self.max_range, self.scan_period)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(LaserScan, "/scan", qos)
        self.subscription = self.create_subscription(
            LaserScan, "/nomad/raw/scan", self.on_scan, qos)
        self.get_logger().info(
            f"G2-like scan: {self.min_range:g}-{self.max_range:g} m, "
            f"{scan_hz:g} Hz; snapshot timing, not a rolling sweep")

    def on_scan(self, source):
        self.publisher.publish(scan_message(
            source, self.min_range, self.max_range, self.scan_period))


def main():
    rclpy.init()
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
