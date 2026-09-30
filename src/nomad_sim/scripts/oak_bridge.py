#!/usr/bin/env python3
"""Publish OAK-like interfaces; retain source stamps, fix optical depth frame."""
import copy

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


def mono_image(source):
    """Convert RGB/BGR to mono8 while respecting row padding and the header."""
    result = Image()
    result.header = copy.deepcopy(source.header)
    result.width, result.height = source.width, source.height
    result.encoding, result.is_bigendian = "mono8", 0
    result.step = source.width
    if source.encoding == "mono8":
        pixels = np.ndarray((source.height, source.width), np.uint8,
                            buffer=source.data, strides=(source.step, 1))
        result.data = pixels.copy().tobytes()
        return result
    if source.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"Unsupported camera encoding: {source.encoding}")
    pixels = np.ndarray((source.height, source.width, 3), np.uint8,
                        buffer=source.data, strides=(source.step, 3, 1))
    weights = [0.114, 0.587, 0.299] if source.encoding == "bgr8" else [0.299, 0.587, 0.114]
    gray = np.rint(pixels @ np.asarray(weights, dtype=np.float32)).astype(np.uint8)
    result.data = gray.tobytes()
    return result


def camera_info(source, frame, right_baseline=0.0):
    """MVSim 1.4 emits identity P; rebuild the rectified pinhole projection."""
    message = copy.deepcopy(source)
    message.header.frame_id = frame
    fx, fy, cx, cy = message.k[0], message.k[4], message.k[2], message.k[5]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [fx, 0.0, cx, -fx * right_baseline,
                 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return message


class OakBridge(Node):
    def __init__(self):
        super().__init__("oak_sim_bridge")
        self.baseline = self.declare_parameter("baseline_m", 0.075).value
        xyz = [self.declare_parameter(f"mount_{axis}_m", default).value
               for axis, default in (("x", 0.22), ("y", 0.0), ("z", 0.29))]
        self.subscriptions_keepalive = []
        camera_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.static_tf = StaticTransformBroadcaster(self)
        rgb_tf = TransformStamped()
        rgb_tf.header.stamp = self.get_clock().now().to_msg()
        rgb_tf.header.frame_id = "base_link"
        rgb_tf.child_frame_id = "oak_rgb_optical_frame"
        rgb_tf.transform.translation.x, rgb_tf.transform.translation.y, rgb_tf.transform.translation.z = xyz
        rgb_tf.transform.rotation.x = -0.5
        rgb_tf.transform.rotation.y = 0.5
        rgb_tf.transform.rotation.z = -0.5
        rgb_tf.transform.rotation.w = 0.5
        depth_tf = TransformStamped()
        depth_tf.header = copy.deepcopy(rgb_tf.header)
        depth_tf.header.frame_id = "oak_rgb_optical_frame"
        depth_tf.child_frame_id = "oak_depth_optical_frame"
        depth_tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform([rgb_tf, depth_tf])

        for side in ("rgb", "depth", "left", "right"):
            frame = f"oak_{side}_optical_frame"
            topic = "image_rect" if side in ("left", "right") else "image_raw"
            image_pub = self.create_publisher(Image, f"/oak/{side}/{topic}", camera_qos)
            info_pub = self.create_publisher(CameraInfo, f"/oak/{side}/camera_info", camera_qos)

            def on_image(source, side=side, frame=frame, publisher=image_pub):
                try:
                    message = mono_image(source) if side in ("left", "right") else copy.deepcopy(source)
                    message.header.frame_id = frame
                    publisher.publish(message)
                except ValueError as error:
                    self.get_logger().error(str(error), throttle_duration_sec=5.0)

            def on_info(source, side=side, frame=frame, publisher=info_pub):
                message = camera_info(source, frame, self.baseline if side == "right" else 0.0)
                publisher.publish(message)

            self.subscriptions_keepalive.extend([
                self.create_subscription(Image, f"/nomad/raw/{side}/image_raw", on_image, camera_qos),
                self.create_subscription(CameraInfo, f"/nomad/raw/{side}/camera_info", on_info, camera_qos),
            ])
        self.get_logger().info("OAK-like RGB/depth + mono stereo bridge ready; timestamps unchanged")


def main():
    rclpy.init()
    node = OakBridge()
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
