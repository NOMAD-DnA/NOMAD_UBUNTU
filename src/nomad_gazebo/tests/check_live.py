#!/usr/bin/env python3
"""Read-only Gazebo/NOMAD live checks and one actual RGB sensor PNG.

Example: ROS_DOMAIN_ID=43 python3 check_live.py --output-dir /tmp/nomad-gz-check
No publisher, service client, Gazebo command, teleport, or motion is created.
Header-derived sampling rates and wall-clock delivery rates are reported
separately; camera timestamp alignment is measured, not assumed synchronous.
"""
import argparse
import bisect
import json
import math
import os
from pathlib import Path
import statistics
import struct
import time
import zlib

import numpy as np


SIDES = ("rgb", "depth", "left", "right")
EXPECTED_ENCODINGS = dict(rgb="bgr8", depth="16UC1", left="mono8", right="mono8")


def image_topic(side):
    return f"/oak/{side}/{'image_rect' if side in ('left', 'right') else 'image_raw'}"


def header_stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def timing_summary(samples, count, elapsed):
    unique = sorted(set(samples))
    intervals = [b-a for a, b in zip(unique, unique[1:])]
    period = statistics.median(intervals) if intervals else None
    return dict(count=count, unique_header_stamps=len(unique),
                received_wall_hz=count / elapsed,
                first_header_stamp_s=unique[0] if unique else None,
                last_header_stamp_s=unique[-1] if unique else None,
                header_span_s=unique[-1]-unique[0] if unique else None,
                median_sim_period_s=period,
                estimated_received_sim_hz=1.0/period if period and period > 0 else None,
                out_of_order_stamps=sum(b < a for a, b in zip(samples, samples[1:])))


def nearest_stamp_offsets(first, second):
    second = sorted(set(second))
    offsets = []
    for value in sorted(set(first)):
        index = bisect.bisect_left(second, value)
        candidates = second[max(0, index-1):index+1]
        if candidates:
            offsets.append(min(abs(value-other) for other in candidates) * 1000.0)
    if not offsets:
        return dict(count=0, median_ms=None, p95_ms=None, maximum_ms=None)
    return dict(count=len(offsets), median_ms=statistics.median(offsets),
                p95_ms=float(np.percentile(offsets, 95)), maximum_ms=max(offsets))


def active_pixels(image):
    channels = 3 if image.encoding == "bgr8" else 1
    dtype = np.dtype((">" if image.is_bigendian else "<") + "u2") if image.encoding == "16UC1" else np.dtype("u1")
    if image.encoding not in ("bgr8", "mono8", "16UC1"):
        raise ValueError(f"Unexpected encoding: {image.encoding}")
    if image.width <= 0 or image.height <= 0 or image.is_bigendian not in (0, 1):
        raise ValueError("Invalid dimensions or byte order")
    active = image.width * channels * dtype.itemsize
    if image.step < active or len(image.data) != image.step * image.height:
        raise ValueError("Invalid image stride/buffer size")
    shape = (image.height, image.width, channels) if channels == 3 else (image.height, image.width)
    strides = (image.step, channels*dtype.itemsize, dtype.itemsize) if channels == 3 else (image.step, dtype.itemsize)
    return np.ndarray(shape, dtype, buffer=bytes(image.data), strides=strides)


def save_rgb_png(image, destination):
    """Encode the actual BGR sensor frame as RGB PNG using the standard library."""
    pixels = active_pixels(image)
    if image.encoding != "bgr8":
        raise ValueError("RGB preview requires bgr8")
    rgb = pixels[..., ::-1].copy()
    rows = b"".join(b"\x00" + row.tobytes() for row in rgb)

    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload
                + struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff))

    payload = (b"\x89PNG\r\n\x1a\n"
               + chunk(b"IHDR", struct.pack(">IIBBBBB", image.width, image.height, 8, 2, 0, 0, 0))
               + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))
    destination.write_bytes(payload)


def rotation(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or abs(norm-1.0) > 1e-3:
        raise ValueError("Invalid TF quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_in_base(frame, transforms, visited=None):
    if frame == "base_link":
        return np.eye(3), np.zeros(3)
    visited = set() if visited is None else visited
    if frame in visited or frame not in transforms:
        raise ValueError(f"Missing or cyclic base_link TF for {frame}")
    visited.add(frame)
    tf = transforms[frame]
    parent_rotation, parent_position = pose_in_base(tf.header.frame_id.lstrip("/"), transforms, visited)
    xyz = tf.transform.translation
    position = np.array([xyz.x, xyz.y, xyz.z])
    if not np.all(np.isfinite(position)):
        raise ValueError("Nonfinite TF translation")
    return (parent_rotation @ rotation(tf.transform.rotation),
            parent_position + parent_rotation @ position)


def load_configuration(path):
    import yaml
    with Path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def json_safe(value):
    """Keep a diagnostic report writable even when a malformed topic has NaN."""
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def build_report(latest, counts, times, transforms, elapsed, configuration):
    camera, lidar = configuration["camera"], configuration["lidar"]
    checks, images, warnings = {}, {}, []
    rates = {topic: timing_summary(times[topic], count, elapsed) for topic, count in counts.items()}
    valid_pixels = {}
    identity = np.eye(3).ravel()
    for side in SIDES:
        topic = image_topic(side)
        image = latest.get(topic)
        info = latest.get(f"/oak/{side}/camera_info")
        valid = bool(image is not None and info is not None)
        if valid:
            try:
                pixels = active_pixels(image)
                valid_pixels[side] = pixels
                valid = (image.width == camera["width"] and image.height == camera["height"]
                         and image.encoding == EXPECTED_ENCODINGS[side]
                         and image.header.frame_id == f"oak_{side}_optical_frame"
                         and info.header.frame_id == image.header.frame_id
                         and info.width == image.width and info.height == image.height
                         and len(info.k) == 9 and len(info.p) == 12 and len(info.r) == 9
                         and all(math.isfinite(x) for x in info.k)
                         and all(math.isfinite(x) for x in info.p)
                         and info.k[0] > 0 and info.k[4] > 0
                         and np.allclose(info.r, identity, rtol=0, atol=1e-8)
                         and np.allclose([info.p[0], info.p[5], info.p[2], info.p[6]],
                                         [info.k[0], info.k[4], info.k[2], info.k[5]], rtol=0, atol=1e-6))
                nonblank = bool(np.any(pixels > 0) if side == "depth" else np.ptp(pixels) > 0)
                valid = bool(valid and nonblank)
                images[side] = dict(width=image.width, height=image.height, encoding=image.encoding,
                                    step=image.step, is_bigendian=int(image.is_bigendian),
                                    frame_id=image.header.frame_id, nonblank=nonblank,
                                    pixel_min=int(pixels.min()), pixel_max=int(pixels.max()),
                                    pixel_mean=float(pixels.mean()), fx=info.k[0], fy=info.k[4],
                                    cx=info.k[2], cy=info.k[5], projection_Tx=info.p[3])
                if any(abs(value) > 1e-9 for value in info.d):
                    warnings.append(f"{side}: nonzero native distortion; adapter does not undistort pixels")
            except (ValueError, TypeError, IndexError, BufferError) as error:
                valid = False
                images[side] = {"error": str(error)}
        checks[f"{side}_image_and_calibration"] = bool(valid)

    depth_pixels = valid_pixels.get("depth")
    if depth_pixels is None:
        checks["depth_range_and_invalid_zero"] = False
    else:
        nonzero = depth_pixels[depth_pixels > 0]
        minimum = math.ceil(float(camera["depth_min_m"]) * 1000 - 1e-8)
        maximum = math.floor(float(camera["depth_max_m"]) * 1000 + 1e-8)
        checks["depth_range_and_invalid_zero"] = bool(nonzero.size and np.all((nonzero >= minimum) & (nonzero <= maximum)))
        images.setdefault("depth", {}).update(valid_fraction=float(nonzero.size/depth_pixels.size),
                                              minimum_valid_mm=int(nonzero.min()) if nonzero.size else None,
                                              maximum_valid_mm=int(nonzero.max()) if nonzero.size else None)
    checks["camera_continuous_updates"] = all(
        rates.get(image_topic(side), {}).get("unique_header_stamps", 0) >= 2 for side in SIDES)

    scan = latest.get("/scan")
    scan_summary = {}
    if scan is not None:
        span = scan.angle_max-scan.angle_min
        count = len(scan.ranges)
        checks["scan_500_unique_angles_and_limits"] = bool(
            count == lidar["nrays"] and scan.angle_increment > 0
            and abs(span-(count-1)*scan.angle_increment) < 1e-5
            and abs(span+scan.angle_increment-math.radians(lidar["fov_deg"])) < 1e-5
            and abs(scan.range_min-lidar["min_range_m"]) < 1e-6
            and abs(scan.range_max-lidar["max_range_m"]) < 1e-6
            and scan.header.frame_id == "scan")
        checks["scan_snapshot_timing"] = bool(abs(scan.scan_time-1/lidar["hz"]) < 1e-6 and scan.time_increment == 0.0)
        values = np.asarray(scan.ranges, dtype=float)
        finite = values[np.isfinite(values)]
        checks["scan_clipping_and_returns"] = bool(finite.size and np.all((finite >= scan.range_min) & (finite < scan.range_max)))
        scan_summary = dict(nrays=count, angle_span_rad=span, angle_increment_rad=scan.angle_increment,
                            span_plus_increment_rad=span+scan.angle_increment,
                            range_min_m=scan.range_min, range_max_m=scan.range_max,
                            scan_time_s=scan.scan_time, time_increment_s=scan.time_increment,
                            finite_returns=int(finite.size), no_hit_returns=int(np.isposinf(values).sum()),
                            too_close_returns=int(np.isneginf(values).sum()), invalid_returns=int(np.isnan(values).sum()))
    else:
        for key in ("scan_500_unique_angles_and_limits", "scan_snapshot_timing", "scan_clipping_and_returns"):
            checks[key] = False
    checks["scan_continuous_updates"] = rates.get("/scan", {}).get("unique_header_stamps", 0) >= 2
    checks["clock_advances"] = rates.get("/clock", {}).get("unique_header_stamps", 0) >= 2

    odom, joints = latest.get("/odom"), latest.get("/joint_states")
    joint_names = list(joints.name) if joints is not None else []
    expected_joints = {"front_left_steering_joint", "front_right_steering_joint",
                       "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"}
    checks["odometry_and_joint_state_updates"] = bool(
        odom is not None and joints is not None
        and rates.get("/odom", {}).get("unique_header_stamps", 0) >= 2
        and rates.get("/joint_states", {}).get("unique_header_stamps", 0) >= 2
        and expected_joints.issubset(joint_names)
        and len(joints.position) == len(joint_names)
        and all(math.isfinite(value) for value in joints.position)
        and odom.header.frame_id == "odom" and odom.child_frame_id == "base_link"
        and all(math.isfinite(value) for value in
                (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                 odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w,
                 odom.twist.twist.linear.x, odom.twist.twist.angular.z)))

    tf_summary, baseline = {}, None
    try:
        poses = {f"oak_{side}_optical_frame": pose_in_base(f"oak_{side}_optical_frame", transforms) for side in SIDES}
        poses["scan"] = pose_in_base("scan", transforms)
        expected_optical_rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        rgb_rotation, rgb_position = poses["oak_rgb_optical_frame"]
        depth_rotation, depth_position = poses["oak_depth_optical_frame"]
        checks["optical_and_scan_tf"] = bool(
            all(np.allclose(poses[f"oak_{side}_optical_frame"][0], expected_optical_rotation, atol=1e-6) for side in SIDES)
            and np.allclose(rgb_position, depth_position, atol=1e-6)
            and np.allclose(rgb_rotation, depth_rotation, atol=1e-6))
        left = poses["oak_left_optical_frame"][1]
        right = poses["oak_right_optical_frame"][1]
        baseline = float(np.linalg.norm(left-right))
        checks["stereo_tf_baseline"] = bool(abs(baseline-camera["baseline_m"]) < 1e-6
                                            and left[1] > right[1] and np.allclose(left[[0, 2]], right[[0, 2]], atol=1e-6))
        tf_summary = {frame: dict(parent="base_link", xyz=position.tolist(), rotation_matrix=orient.ravel().tolist())
                      for frame, (orient, position) in poses.items()}
    except (ValueError, RecursionError) as error:
        checks["optical_and_scan_tf"] = checks["stereo_tf_baseline"] = False
        tf_summary = {"error": str(error)}
    right_info, left_info = latest.get("/oak/right/camera_info"), latest.get("/oak/left/camera_info")
    checks["right_rectified_projection_baseline"] = bool(
        right_info is not None and left_info is not None
        and len(right_info.p) == 12 and len(right_info.k) == 9 and len(left_info.p) == 12
        and abs(right_info.p[3]+right_info.k[0]*camera["baseline_m"]) < 1e-6
        and abs(left_info.p[3]) < 1e-9)

    alignment = {"stereo_left_to_right": nearest_stamp_offsets(times.get(image_topic("left"), []), times.get(image_topic("right"), [])),
                 "rgb_to_depth": nearest_stamp_offsets(times.get(image_topic("rgb"), []), times.get(image_topic("depth"), []))}
    for side in SIDES:
        alignment[f"{side}_image_to_info"] = nearest_stamp_offsets(times.get(image_topic(side), []), times.get(f"/oak/{side}/camera_info", []))
        alignment[f"{side}_output_to_native_image"] = nearest_stamp_offsets(times.get(image_topic(side), []), times.get(f"/nomad/raw/oak/{side}/image", []))
        if not counts.get(f"/nomad/raw/oak/{side}/camera_info", 0):
            warnings.append(f"{side}: no native CameraInfo observed; output may be provisional fallback calibration")
    for key in ("stereo_left_to_right", "rgb_to_depth"):
        offsets = alignment[key]
        if offsets["p95_ms"] is None or offsets["p95_ms"] > 1.0:
            warnings.append(
                f"{key}: nearest observed stamp offsets median={offsets['median_ms']} ms, "
                f"p95={offsets['p95_ms']} ms, maximum={offsets['maximum_ms']} ms; "
                "independently missed samples can contribute, so this is not proof of sensor "
                "desynchronization or of strict synchronization")
    for side in SIDES:
        if rates.get(image_topic(side), {}).get("received_wall_hz", 0) < float(camera["fps"]) * 0.5:
            warnings.append(f"{side}: low wall-clock delivery rate; not a failure if valid images continue updating")

    assert len(checks) == 15, f"Expected 15 checks, found {len(checks)}"
    odom_summary = None if odom is None else dict(frame_id=odom.header.frame_id, child_frame_id=odom.child_frame_id,
        xyz=[odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z],
        quaternion=[odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w],
        linear_speed_m_s=odom.twist.twist.linear.x, yaw_rate_rad_s=odom.twist.twist.angular.z)
    return dict(passed=all(checks.values()), checks=checks, warnings=warnings,
                measurements=dict(images=images, scan=scan_summary, topic_rates=rates,
                                  timestamp_alignment_ms=alignment, stereo_baseline_m=baseline,
                                  transforms=tf_summary, odometry=odom_summary,
                                  joints=dict(names=joint_names, positions=list(joints.position) if joints is not None else [])),
                notes=["Read-only: no commands, publishers, service clients, or teleport.",
                       "Sim-header intervals are received sampling intervals, not wall-clock throughput.",
                       "Nearest timestamp offsets are measured associations, not proof of hardware synchronization."])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--sensor-config", type=Path)
    parser.add_argument("--exercise", action="store_true", help="Reserved; motion is deliberately not implemented by this read-only tool")
    args = parser.parse_args(argv)
    if args.exercise:
        parser.error("--exercise is not implemented: use the separately authorized domain-43 motion test")
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error("--duration must be 1..60 wall-clock seconds")

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
    from tf2_msgs.msg import TFMessage
    config = args.sensor_config
    if config is None:
        from ament_index_python.packages import get_package_share_directory
        config = Path(get_package_share_directory("nomad_gazebo")) / "config" / "sensors.yaml"
    configuration = load_configuration(config)
    latest, counts, times, transforms = {}, {}, {}, {}
    rclpy.init()
    node = rclpy.create_node("nomad_gazebo_live_check")
    subscriptions = []

    def subscribe(message_type, topic):
        counts[topic], times[topic] = 0, []

        def callback(message):
            latest[topic] = message
            counts[topic] += 1
            times[topic].append(message.clock.sec+message.clock.nanosec*1e-9 if topic == "/clock" else header_stamp(message))

        subscriptions.append(node.create_subscription(message_type, topic, callback, qos_profile_sensor_data))

    for side in SIDES:
        subscribe(Image, image_topic(side))
        subscribe(CameraInfo, f"/oak/{side}/camera_info")
        subscribe(Image, f"/nomad/raw/oak/{side}/image")
        subscribe(CameraInfo, f"/nomad/raw/oak/{side}/camera_info")
    for message_type, topic in ((LaserScan, "/scan"), (Clock, "/clock"), (Odometry, "/odom"), (JointState, "/joint_states")):
        subscribe(message_type, topic)

    def tf_callback(message, topic):
        counts[topic] += 1
        for tf in message.transforms:
            transforms[tf.child_frame_id.lstrip("/")] = tf
            times[topic].append(header_stamp(tf))

    for topic, qos in (("/tf", qos_profile_sensor_data),
                       ("/tf_static", QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))):
        counts[topic], times[topic] = 0, []
        subscriptions.append(node.create_subscription(TFMessage, topic, lambda message, topic=topic: tf_callback(message, topic), qos))
    try:
        started = time.monotonic()
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
        elapsed = time.monotonic()-started
        report = build_report(latest, counts, times, transforms, elapsed, configuration)
        report.update(ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0"), elapsed_wall_s=elapsed,
                      sensor_config=str(config.resolve()), node_names=sorted(node.get_node_names()))
        output = args.output_dir.resolve()
        output.mkdir(parents=True, exist_ok=True)
        rgb = latest.get(image_topic("rgb"))
        preview = output / "rgb.png"
        if rgb is not None:
            try:
                save_rgb_png(rgb, preview)
                report["rgb_preview_png"] = str(preview)
            except (ValueError, TypeError, BufferError) as error:
                report["warnings"].append(f"RGB preview unavailable: {error}")
        report["report_json"] = str(output / "report.json")
        encoded = json.dumps(json_safe(report), indent=2, allow_nan=False)
        (output / "report.json").write_text(encoded+"\n", encoding="utf-8")
        print(encoded)
        return 0 if report["passed"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
