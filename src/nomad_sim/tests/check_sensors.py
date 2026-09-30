#!/usr/bin/env python3
"""Read-only live checks: dimensions, projection, TF baseline and 2D scan."""
import bisect
import json
import math
from pathlib import Path
import statistics
import time

import yaml

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def main():
    with (Path(__file__).parents[1] / "config" / "sensors.yaml").open() as stream:
        configuration = yaml.safe_load(stream)
    camera, lidar = configuration["camera"], configuration["lidar"]
    rclpy.init()
    node = rclpy.create_node("nomad_sensor_check")
    latest, counts, times, transforms = {}, {}, {}, {}
    subscriptions = []
    camera_qos = rclpy.qos.QoSProfile(depth=5, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
    for side in ("rgb", "depth", "left", "right"):
        suffix = "image_rect" if side in ("left", "right") else "image_raw"
        for message_type, topic in ((Image, f"/oak/{side}/{suffix}"),
                                    (CameraInfo, f"/oak/{side}/camera_info")):
            counts[topic], times[topic] = 0, []

            def callback(message, topic=topic):
                latest[topic] = message
                counts[topic] += 1
                times[topic].append(stamp(message))

            subscriptions.append(node.create_subscription(message_type, topic, callback, camera_qos))

    def scan_callback(message):
        latest["/scan"] = message
        counts["/scan"] = counts.get("/scan", 0) + 1
        times.setdefault("/scan", []).append(stamp(message))

    def tf_callback(message):
        for transform in message.transforms:
            transforms[transform.child_frame_id] = transform

    subscriptions.append(node.create_subscription(LaserScan, "/scan", scan_callback, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage, "/tf", tf_callback, qos_profile_sensor_data))
    static_qos = rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(TFMessage, "/tf_static", tf_callback, static_qos))
    try:
        started = time.monotonic()
        deadline = started + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        elapsed = time.monotonic() - started
        checks, summary = {}, {}
        for side in ("rgb", "depth", "left", "right"):
            suffix = "image_rect" if side in ("left", "right") else "image_raw"
            topic = f"/oak/{side}/{suffix}"
            image = latest.get(topic)
            info = latest.get(f"/oak/{side}/camera_info")
            expected = "16UC1" if side == "depth" else "mono8" if side in ("left", "right") else "bgr8"
            checks[side] = bool(image and info and image.width == camera["width"] and image.height == camera["height"]
                                and image.encoding == expected and image.header.frame_id == f"oak_{side}_optical_frame"
                                and info.r[0] == 1 and info.r[4] == 1 and info.r[8] == 1
                                and abs(info.p[0] - info.k[0]) < 1e-6)
            if image and info:
                samples = sorted(set(times[topic]))
                period = statistics.median([b-a for a, b in zip(samples, samples[1:])]) if len(samples) > 1 else 0
                summary[side] = {"count": counts[topic], "encoding": image.encoding,
                                 "window_received_fps": counts[topic] / elapsed,
                                 "width": image.width, "height": image.height,
                                 "fx": info.k[0], "fy": info.k[4], "right_Tx": info.p[3],
                                 "median_sim_period_s": period,
                                 "approx_received_sim_hz": 1 / period if period else 0}
        scan = latest.get("/scan")
        scan_period = 1.0 / float(lidar["hz"])
        checks["scan"] = bool(scan and len(scan.ranges) == lidar["nrays"]
                              and abs(scan.range_min - lidar["min_range_m"]) < 1e-6
                              and abs(scan.range_max - lidar["max_range_m"]) < 1e-6
                              and abs(scan.angle_max - scan.angle_min - math.radians(lidar["fov_deg"])) < 1e-5)
        checks["scan_timing"] = bool(scan and abs(scan.scan_time - scan_period) < 1e-6
                                     and scan.time_increment == 0.0)
        checks["scan_clipping"] = bool(scan and all(
            not math.isfinite(value) or scan.range_min <= value < scan.range_max for value in scan.ranges))
        checks["scan_has_returns"] = bool(scan and any(math.isfinite(value) for value in scan.ranges))
        if scan:
            scan_times = sorted(set(times["/scan"]))
            scan_median_period = statistics.median(
                [b-a for a, b in zip(scan_times, scan_times[1:])]) if len(scan_times) > 1 else 0
            summary["scan"] = {"count": counts["/scan"], "nrays": len(scan.ranges),
                                "window_received_hz": counts["/scan"] / elapsed,
                                "fov_rad": scan.angle_max - scan.angle_min,
                                "angle_increment_deg": math.degrees(scan.angle_increment),
                                "min_range_m": scan.range_min, "max_range_m": scan.range_max,
                                "scan_time_s": scan.scan_time, "time_increment_s": scan.time_increment,
                                "median_sim_period_s": scan_median_period,
                                "too_close_returns": sum(x == -math.inf for x in scan.ranges),
                                "no_hit_returns": sum(x == math.inf for x in scan.ranges),
                                "invalid_returns": sum(math.isnan(x) for x in scan.ranges),
                                "finite_returns": sum(math.isfinite(x) and scan.range_min <= x < scan.range_max
                                                      for x in scan.ranges)}
            checks["scan_timestamp_period"] = abs(scan_median_period - scan_period) < 0.2 * scan_period
        left = transforms.get("oak_left_optical_frame")
        right = transforms.get("oak_right_optical_frame")
        baseline = abs(left.transform.translation.y - right.transform.translation.y) if left and right else None
        checks["stereo_baseline"] = baseline is not None and abs(baseline - camera["baseline_m"]) < 1e-9
        right_info = latest.get("/oak/right/camera_info")
        checks["right_projection"] = bool(right_info and abs(right_info.p[3] + right_info.k[0]*camera["baseline_m"]) < 1e-6)
        checks["optical_tf"] = all(frame in transforms for frame in ("oak_rgb_optical_frame", "oak_depth_optical_frame", "scan"))
        summary["baseline_m"] = baseline
        left_times = sorted(set(times["/oak/left/image_rect"]))
        right_times = sorted(set(times["/oak/right/image_rect"]))
        offsets = []
        if right_times:
            for sample in left_times:
                position = bisect.bisect_left(right_times, sample)
                candidates = right_times[max(0, position-1):position+1]
                offsets.append(min(abs(value-sample) for value in candidates))
        summary["stereo_nearest_stamp_median_ms"] = 1000*statistics.median(offsets) if offsets else None
        # Timestamp intervals verify sampling; this is NOT delivery throughput.
        checks["camera_timestamp_period"] = all(
            summary.get(side, {}).get("approx_received_sim_hz", 0) >= 20
            for side in ("rgb", "depth", "left", "right"))
        checks["stereo_stamps"] = bool(offsets) and statistics.median(offsets) < 0.001
        topic_names = {name for name, _ in node.get_topic_names_and_types()}
        checks["old_3d_lidar_removed"] = "/lidar1_points" not in topic_names
        report = {"checks": checks, "measurements": summary, "passed": all(checks.values())}
        print(json.dumps(report, indent=2))
        if not report["passed"]:
            raise RuntimeError("Sensor verification failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
