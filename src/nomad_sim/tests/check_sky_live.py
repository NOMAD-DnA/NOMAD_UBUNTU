#!/usr/bin/env python3
"""Read-only stationary RGB sky / depth no-echo check; no motion or RPC."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from check_forest_live import _save_rgb_png


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=4.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 30:
        parser.error('--duration must be 1..30 seconds')
    rclpy.init()
    node = rclpy.create_node('nomad_sky_live_check')
    frames = {'rgb': {}, 'depth': {}}
    counts = {'rgb': 0, 'depth': 0, 'scan': 0}
    latest_scan = None
    subscriptions = []

    def image_callback(message, side):
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        frames[side][key] = message
        counts[side] += 1
        while len(frames[side]) > 10:
            del frames[side][next(iter(frames[side]))]

    def scan_callback(message):
        nonlocal latest_scan
        latest_scan = message
        counts['scan'] += 1

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    for side in ('rgb', 'depth'):
        subscriptions.append(node.create_subscription(
            Image, f'/oak/{side}/image_raw',
            lambda message, side=side: image_callback(message, side), qos))
    subscriptions.append(node.create_subscription(
        LaserScan, '/scan', scan_callback, qos_profile_sensor_data))
    report = {'passed': False, 'checks': {}, 'counts': counts,
              'scope': 'stationary sky appearance and no-echo, not driving or servo'}
    try:
        started = time.monotonic()
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=.01)
        report['elapsed_wall_s'] = time.monotonic() - started
        matching = sorted(set(frames['rgb']) & set(frames['depth']))
        if not matching:
            raise RuntimeError('No RGB/depth pair with exactly the same stamp')
        key = matching[-1]
        rgb, depth = frames['rgb'][key], frames['depth'][key]
        if (rgb.encoding != 'bgr8' or depth.encoding != '16UC1'
                or (rgb.width, rgb.height) != (depth.width, depth.height)):
            raise RuntimeError('Unexpected image encoding or dimensions')
        color = np.ndarray((rgb.height, rgb.width, 3), np.uint8,
                           buffer=bytes(rgb.data), strides=(rgb.step, 3, 1))
        dtype = np.dtype('>u2' if depth.is_bigendian else '<u2')
        ranges = np.ndarray((depth.height, depth.width), dtype,
                            buffer=bytes(depth.data), strides=(depth.step, 2))
        # At the unchanged start pose the upper image contains blue-gray sky.
        # Gray clear-color cannot satisfy this, and dark green trees are excluded.
        top = color[:rgb.height // 3].astype(np.int16)
        sky = ((top[..., 0] > top[..., 2] + 8)
               & (top[..., 0] > top[..., 1] + 2))
        sky_pixels = int(sky.sum())
        sky_depth = ranges[:rgb.height // 3][sky]
        invalid_fraction = float(np.mean(sky_depth == 0)) if sky_pixels else 0.0
        scan = latest_scan
        no_echo = sum(value == math.inf for value in scan.ranges) if scan else 0
        report['checks'] = {
            'rgb_depth_updates': counts['rgb'] >= 3 and counts['depth'] >= 3,
            'visible_blue_gray_sky': sky_pixels >= 1000,
            'sky_depth_is_invalid_zero': sky_pixels >= 1000 and invalid_fraction >= .99,
            'laser_keeps_no_echo_returns': bool(scan and counts['scan'] >= 3
                                              and len(scan.ranges) == 500 and no_echo > 0),
        }
        report['measurements'] = {
            'matching_stamp': list(key), 'sky_pixels_in_upper_third': sky_pixels,
            'sky_depth_invalid_zero_fraction': invalid_fraction,
            'sky_depth_nonzero_pixels': int(np.count_nonzero(sky_depth)),
            'laser_no_echo_returns': no_echo,
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        preview = args.output_dir / 'rgb.png'
        _save_rgb_png(rgb, preview)
        report['rgb_preview_png'] = str(preview.resolve())
        report['passed'] = all(report['checks'].values())
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, indent=2, allow_nan=False)
    (args.output_dir / 'sky_live_report.json').write_text(encoded + '\n', encoding='utf-8')
    print(encoded)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
