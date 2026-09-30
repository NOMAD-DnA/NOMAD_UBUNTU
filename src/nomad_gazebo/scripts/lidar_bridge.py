#!/usr/bin/env python3
"""Self-contained G2-like snapshot scan filter for NOMAD Gazebo.

Adapted from NOMAD nomad_sim/scripts/lidar_bridge.py. This module has no runtime
dependency on nomad_sim, MVSim, or MRPT. The filtering helpers use only the Python
standard library; ROS imports are deferred to main().
"""
from array import array
import copy
import math


def validate_limits(min_range, max_range):
    if not all(math.isfinite(value) for value in (min_range, max_range)):
        raise ValueError("LiDAR clipping limits must be finite")
    if not 0 <= min_range < max_range:
        raise ValueError("Invalid LiDAR clipping limits")


def validate_profile(min_range, max_range, scan_period):
    validate_limits(min_range, max_range)
    if not math.isfinite(scan_period) or scan_period <= 0:
        raise ValueError("LiDAR scan period must be finite and positive")


def clip_ranges(values, min_range, max_range):
    """Return float32 ranges with REP-117 invalid/too-close/no-return values."""
    validate_limits(min_range, max_range)
    # Incoming LaserScan samples and published limits use the same float32
    # representation, including the 0.12-m minimum boundary.
    minimum, maximum = array("f", [min_range, max_range])

    def clipped(value):
        if not math.isfinite(value):
            return value
        if value <= 0:
            return math.nan
        if value < minimum:
            return -math.inf
        if value >= maximum:
            return math.inf
        return value

    return array("f", (clipped(value) for value in values))


def scan_message(source, min_range, max_range, scan_period):
    """Keep native headers/angles; a rendered snapshot has no per-ray delay."""
    validate_profile(min_range, max_range, scan_period)
    message = copy.deepcopy(source)
    message.range_min, message.range_max = array("f", [min_range, max_range])
    message.ranges = clip_ranges(source.ranges, message.range_min, message.range_max)
    message.scan_time = scan_period
    message.time_increment = 0.0
    return message


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

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

    rclpy.init()
    node = None
    try:
        node = LidarBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
