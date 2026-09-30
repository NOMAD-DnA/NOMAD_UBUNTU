#!/usr/bin/env python3
"""Lightweight, read-only ROS delivery / simulation-time benchmark.

Subscribes only to clock, four output images and scan; KEEP_LAST(1), best effort.
Callbacks retain counters, integer timestamps and payload sizes, never images.
No camera calibration, raw images, TF, odometry, PNG, commands or services.
"""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import time


IMAGE_TOPICS = (
    "/oak/rgb/image_raw", "/oak/depth/image_raw",
    "/oak/left/image_rect", "/oak/right/image_rect",
)
TOPICS = ("/clock",) + IMAGE_TOPICS + ("/scan",)


def empty_samples():
    return {topic: dict(stamps_ns=[], count=0, total_payload_bytes=0,
                        minimum_payload_bytes=None, maximum_payload_bytes=0)
            for topic in TOPICS}


def observe(sample, stamp, payload_bytes):
    sample["count"] += 1
    sample["stamps_ns"].append(stamp.sec * 1_000_000_000 + stamp.nanosec)
    sample["total_payload_bytes"] += payload_bytes
    minimum = sample["minimum_payload_bytes"]
    sample["minimum_payload_bytes"] = payload_bytes if minimum is None else min(minimum, payload_bytes)
    sample["maximum_payload_bytes"] = max(sample["maximum_payload_bytes"], payload_bytes)


def summarize(sample, elapsed):
    stamps = sample["stamps_ns"]
    ordered_unique = sorted(set(stamps))
    intervals = [b-a for a, b in zip(ordered_unique, ordered_unique[1:])]
    period_ns = statistics.median(intervals) if intervals else None
    return dict(
        count=sample["count"],
        received_wall_hz=sample["count"] / elapsed,
        unique_header_stamps=len(ordered_unique),
        first_received_stamp_s=stamps[0] * 1e-9 if stamps else None,
        last_received_stamp_s=stamps[-1] * 1e-9 if stamps else None,
        received_stamp_span_s=(stamps[-1]-stamps[0]) * 1e-9 if stamps else None,
        median_received_sim_period_s=period_ns * 1e-9 if period_ns else None,
        estimated_received_sim_hz=1e9 / period_ns if period_ns else None,
        out_of_order_stamps=sum(b < a for a, b in zip(stamps, stamps[1:])),
        minimum_payload_bytes=sample["minimum_payload_bytes"],
        maximum_payload_bytes=sample["maximum_payload_bytes"],
        total_payload_bytes=sample["total_payload_bytes"],
        received_payload_MB_s=sample["total_payload_bytes"] / elapsed / 1e6,
    )


def build_report(samples, elapsed, cpu_seconds, warmup, requested_duration):
    topics = {topic: summarize(sample, elapsed) for topic, sample in samples.items()}
    clock = topics["/clock"]
    sim_seconds = clock["received_stamp_span_s"]
    advancing = bool(sim_seconds is not None and sim_seconds > 0
                     and clock["unique_header_stamps"] >= 2
                     and clock["out_of_order_stamps"] == 0)
    warnings = []
    if not advancing:
        warnings.append("Clock absent, paused or non-monotonic: this run is not an active-simulation comparison.")
    for topic, values in topics.items():
        if values["unique_header_stamps"] < 2:
            warnings.append(f"{topic}: fewer than two distinct stamps received")
        if values["out_of_order_stamps"]:
            warnings.append(f"{topic}: non-monotonic stamps observed")
    return dict(
        read_only=True,
        subscriptions=list(TOPICS),
        subscription_qos="BEST_EFFORT / VOLATILE / KEEP_LAST(1)",
        requested_duration_wall_s=requested_duration,
        discovery_warmup_wall_s=warmup,
        elapsed_wall_s=elapsed,
        clock_delta_s=sim_seconds,
        real_time_factor=sim_seconds / elapsed if advancing else None,
        clock_advancing=advancing,
        checker_process_cpu_s=cpu_seconds,
        checker_process_cpu_percent=100 * cpu_seconds / elapsed,
        topic_rates=topics,
        warnings=warnings,
        notes=[
            "No commands, publishers, service clients, simulation changes, frame copies or PNG encoding.",
            "DDS must still deserialize full image messages; callback retains only count/stamp/size.",
            "Wall Hz is observed delivery, while received simulation Hz can include dropped samples.",
            "Clock delta uses first/last measurement-window samples; discovery warmup is excluded.",
            "This tool's CPU is not the Gazebo server or GUI CPU.",
        ],
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error("--duration must be 1..60 wall-clock seconds")
    if not math.isfinite(args.warmup) or not 0 <= args.warmup <= 30:
        parser.error("--warmup must be 0..30 wall-clock seconds")

    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, LaserScan

    samples = empty_samples()
    rclpy.init()
    node = rclpy.create_node("nomad_gazebo_perf_check")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    subscriptions = []
    try:
        subscriptions.append(node.create_subscription(
            Clock, "/clock", lambda message: observe(samples["/clock"], message.clock, 0), qos))
        for topic in IMAGE_TOPICS:
            subscriptions.append(node.create_subscription(
                Image, topic, lambda message, topic=topic: observe(
                    samples[topic], message.header.stamp, len(message.data)), qos))
        subscriptions.append(node.create_subscription(
            LaserScan, "/scan", lambda message: observe(
                samples["/scan"], message.header.stamp, 4 * len(message.ranges)), qos))

        warmup_start = time.monotonic()
        while time.monotonic()-warmup_start < args.warmup:
            rclpy.spin_once(node, timeout_sec=0.01)
        samples = empty_samples()
        started, cpu_started = time.monotonic(), time.process_time()
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
        elapsed, cpu_seconds = time.monotonic()-started, time.process_time()-cpu_started
        report = build_report(samples, elapsed, cpu_seconds, args.warmup, args.duration)
        report["ros_domain_id"] = os.environ.get("ROS_DOMAIN_ID", "0")
        output = args.output_dir.resolve()
        output.mkdir(parents=True, exist_ok=True)
        destination = output / "report.json"
        report["report_json"] = str(destination)
        encoded = json.dumps(report, indent=2, allow_nan=False)
        destination.write_text(encoded+"\n", encoding="utf-8")
        print(encoded)
        return 0 if report["clock_advancing"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
