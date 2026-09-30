#!/usr/bin/env python3
"""Normalize Gazebo images to NOMAD's provisional OAK interfaces.

The conversion helpers deliberately have no ROS imports, so byte order, row
stride, depth validity, and CameraInfo can be tested without a running graph.
This adapter does not model stereo matching and does not publish TF: the robot
description owns the sensor extrinsics and the identity RGB-to-depth transform.
"""
import array
import copy
import math
from pathlib import Path

import numpy as np


SIDES = ("rgb", "depth", "left", "right")
DEFAULT_CAMERA = dict(
    width=640, height=480, fps=30.0,
    rgb_hfov_deg=69.0, rgb_vfov_deg=54.0,
    mono_hfov_deg=73.0, mono_vfov_deg=58.0,
    rgb_fx=None, rgb_fy=None, mono_fx=None, mono_fy=None, cx=None, cy=None,
    baseline_m=0.075, depth_min_m=0.4, depth_max_m=8.0,
)


def optical_frame(side):
    if side not in SIDES:
        raise ValueError(f"Unknown camera side: {side}")
    return f"oak_{side}_optical_frame"


def _pixels(source, dtype, channels=1):
    """View only active pixels; return the original rows for padding retention."""
    width, height, step = int(source.width), int(source.height), int(source.step)
    dtype = np.dtype(dtype)
    if width <= 0 or height <= 0:
        raise ValueError("Image width and height must be positive")
    if int(source.is_bigendian) not in (0, 1):
        raise ValueError("is_bigendian must be 0 or 1")
    active = width * channels * dtype.itemsize
    if step < active:
        raise ValueError(f"Image step {step} is smaller than active row {active}")
    try:
        raw = bytes(source.data)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("Invalid image byte buffer") from error
    if len(raw) != height * step:
        raise ValueError(f"Image buffer has {len(raw)} bytes; expected {height * step}")
    rows = np.frombuffer(raw, np.uint8).reshape(height, step)
    shape = (height, width) if channels == 1 else (height, width, channels)
    strides = ((step, dtype.itemsize) if channels == 1
               else (step, channels * dtype.itemsize, dtype.itemsize))
    pixels = np.ndarray(shape, dtype, buffer=raw, strides=strides)
    return pixels, rows, active


def _image(source, pixels, encoding, rows, source_active):
    """Pack converted pixels and retain each original row's padding bytes."""
    height, width = pixels.shape[:2]
    channels = pixels.shape[2] if pixels.ndim == 3 else 1
    active = width * channels * pixels.dtype.itemsize
    padding = rows.shape[1] - source_active
    packed = np.empty((height, active + padding), np.uint8)
    packed[:, :active] = np.frombuffer(pixels.tobytes(), np.uint8).reshape(height, active)
    if padding:
        packed[:, active:] = rows[:, source_active:]
    # Construct rather than deepcopy a full image payload on every callback.
    result = type(source)()
    result.header = copy.deepcopy(source.header)
    result.height, result.width = height, width
    result.encoding = encoding
    result.is_bigendian = int(source.is_bigendian)
    result.step = active + padding
    # array('B') also avoids per-byte Python validation when ROS field checks
    # are enabled; Jazzy disables those checks by default, so this is not proof
    # that the default runtime's low delivery rate has been resolved.
    # This changes the container only, not the pixel bytes, stride, or metadata.
    result.data = array.array("B", packed.tobytes())
    return result


def _color_pixels(source):
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4,
                "mono8": 1, "8UC1": 1}.get(source.encoding)
    if channels is None:
        raise ValueError(f"Unsupported color encoding: {source.encoding}")
    return _pixels(source, np.uint8, channels)


def color_image(source):
    pixels, rows, active = _color_pixels(source)
    if pixels.ndim == 2:
        bgr = np.repeat(pixels[..., None], 3, axis=2)
    elif source.encoding.startswith("rgb"):
        bgr = pixels[..., :3][..., ::-1]
    else:
        bgr = pixels[..., :3]
    return _image(source, bgr, "bgr8", rows, active)


def mono_image(source):
    pixels, rows, active = _color_pixels(source)
    if pixels.ndim == 2:
        gray = pixels
    else:
        weights = ([0.299, 0.587, 0.114] if source.encoding.startswith("rgb")
                   else [0.114, 0.587, 0.299])
        gray = np.clip(np.rint(pixels[..., :3] @ np.asarray(weights, np.float32)),
                       0, 255).astype(np.uint8)
    return _image(source, gray, "mono8", rows, active)


def depth_image(source, minimum_m=0.4, maximum_m=8.0):
    if not (math.isfinite(minimum_m) and math.isfinite(maximum_m)
            and 0 <= minimum_m < maximum_m):
        raise ValueError("Depth limits must be finite and 0 <= min < max")
    endian = ">" if source.is_bigendian else "<"
    if source.encoding == "32FC1":
        pixels, rows, active = _pixels(source, endian + "f4")
        meters = pixels.astype(np.float64)
        # Inclusive limits are compared in the source float32 domain. The
        # configured 0.4m boundary itself must not fail representation rounding.
        lower, upper = float(np.float32(minimum_m)), float(np.float32(maximum_m))
    elif source.encoding in ("16UC1", "mono16"):
        pixels, rows, active = _pixels(source, endian + "u2")
        meters = pixels.astype(np.float64) / 1000.0
        lower, upper = minimum_m, maximum_m
    else:
        raise ValueError(f"Unsupported depth encoding: {source.encoding}")
    valid = np.isfinite(meters) & (meters > 0) & (meters >= lower) & (meters <= upper)
    rounded_mm = np.rint(np.where(valid, meters, 0.0) * 1000.0)
    valid &= (rounded_mm >= 1) & (rounded_mm <= np.iinfo(np.uint16).max)
    # Replace invalid/overflow values before casting; never let uint16 wrap.
    millimeters = np.where(valid, rounded_mm, 0.0).astype(endian + "u2")
    return _image(source, millimeters, "16UC1", rows, active)


def normalize_image(source, side, minimum_m=0.4, maximum_m=8.0):
    frame = optical_frame(side)
    if side == "depth":
        result = depth_image(source, minimum_m, maximum_m)
    elif side == "rgb":
        result = color_image(source)
    else:
        result = mono_image(source)
    result.header.frame_id = frame
    return result


def camera_info(source, side, baseline_m=0.075):
    frame = optical_frame(side)
    if not math.isfinite(baseline_m) or baseline_m < 0:
        raise ValueError("Stereo baseline must be finite and nonnegative")
    k = list(source.k)
    if len(k) != 9 or not all(math.isfinite(float(value)) for value in k):
        raise ValueError("CameraInfo K must have nine finite elements")
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if fx <= 0 or fy <= 0 or source.width <= 0 or source.height <= 0:
        raise ValueError("CameraInfo needs positive focal lengths and dimensions")
    result = copy.deepcopy(source)
    result.header.frame_id = frame
    result.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    result.p = [fx, 0.0, cx, -fx * baseline_m if side == "right" else 0.0,
                0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return result


def fallback_intrinsics(camera, side, width, height):
    """Provisional pinhole calibration only when native CameraInfo is absent."""
    optical_frame(side)
    if width <= 0 or height <= 0:
        raise ValueError("Image dimensions must be positive")
    cfg = DEFAULT_CAMERA | camera
    prefix = "mono" if side in ("left", "right") else "rgb"
    values = []
    for axis, pixels, dimension in (("x", width, "width"), ("y", height, "height")):
        configured = cfg.get(f"{prefix}_f{axis}")
        if configured is None:
            fov = float(cfg[f"{prefix}_{'h' if axis == 'x' else 'v'}fov_deg"])
            if not math.isfinite(fov) or not 0 < fov < 180:
                raise ValueError("Pinhole field of view must be between 0 and 180 degrees")
            focal = pixels / (2.0 * math.tan(math.radians(fov) / 2.0))
        else:
            focal = float(configured) * pixels / float(cfg[dimension])
        values.append(focal)
    fx, fy = values
    cx = width / 2.0 if cfg.get("cx") is None else float(cfg["cx"]) * width / float(cfg["width"])
    cy = height / 2.0 if cfg.get("cy") is None else float(cfg["cy"]) * height / float(cfg["height"])
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("Invalid provisional camera intrinsics")
    return [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]


def load_camera_config(path):
    if not path:
        return DEFAULT_CAMERA.copy()
    import yaml
    with Path(path).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get("camera"), dict):
        raise ValueError("Sensor configuration must contain a camera mapping")
    return DEFAULT_CAMERA | document["camera"]


def main(args=None):
    # Keep ROS imports out of the pure byte-conversion test path.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    class GazeboSensorAdapter(Node):
        def __init__(self):
            super().__init__("gazebo_sensor_adapter")
            default_config = Path(__file__).resolve().parents[1] / "config" / "sensors.yaml"
            try:
                from ament_index_python.packages import get_package_share_directory
                default_config = Path(get_package_share_directory("nomad_gazebo")) / "config" / "sensors.yaml"
            except (ImportError, LookupError):
                pass
            path = self.declare_parameter("sensors_config", str(default_config) if default_config.is_file() else "").value
            self.camera = load_camera_config(path)
            self.baseline = self.declare_parameter("baseline_m", float(self.camera["baseline_m"])).value
            self.depth_min = self.declare_parameter("depth_min_m", float(self.camera["depth_min_m"])).value
            self.depth_max = self.declare_parameter("depth_max_m", float(self.camera["depth_max_m"])).value
            output_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            self.keepalive = []
            self.native_info_seen = set()
            for side in SIDES:
                image_suffix = "image_rect" if side in ("left", "right") else "image_raw"
                image_pub = self.create_publisher(Image, f"/oak/{side}/{image_suffix}", output_qos)
                info_pub = self.create_publisher(CameraInfo, f"/oak/{side}/camera_info", output_qos)

                def on_image(source, side=side, image_pub=image_pub, info_pub=info_pub):
                    try:
                        result = normalize_image(source, side, self.depth_min, self.depth_max)
                        image_pub.publish(result)
                        if side not in self.native_info_seen:
                            info = CameraInfo()
                            info.header = copy.deepcopy(source.header)
                            info.width, info.height = source.width, source.height
                            info.distortion_model, info.d = "plumb_bob", [0.0] * 5
                            info.k = fallback_intrinsics(self.camera, side, source.width, source.height)
                            info_pub.publish(camera_info(info, side, self.baseline))
                    except (ValueError, TypeError, OverflowError) as error:
                        self.get_logger().error(f"{side} image: {error}", throttle_duration_sec=5.0)

                def on_info(source, side=side, info_pub=info_pub):
                    try:
                        info_pub.publish(camera_info(source, side, self.baseline))
                        self.native_info_seen.add(side)
                    except (ValueError, TypeError, OverflowError) as error:
                        self.get_logger().error(f"{side} CameraInfo: {error}", throttle_duration_sec=5.0)

                raw = f"/nomad/raw/oak/{side}"
                self.keepalive.extend([
                    self.create_subscription(Image, raw + "/image", on_image, qos_profile_sensor_data),
                    self.create_subscription(CameraInfo, raw + "/camera_info", on_info, qos_profile_sensor_data),
                ])
            self.get_logger().info("OAK image adapter ready; native stamps retained, calibration defaults provisional, TF owned by robot description")

    rclpy.init(args=args)
    node = None
    try:
        node = GazeboSensorAdapter()
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
