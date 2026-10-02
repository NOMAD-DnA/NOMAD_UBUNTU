#!/usr/bin/env python3
"""Read live depth/scan/TF; publish only isolated debug topics, never cmd_vel.

Run with this workspace sourced and the simulator already running. No simulator
or planner is started, stopped or reset. Goal input is isolated as well.
"""
import json
import time
import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from nomad_perception.sensor_input import SensorInput
from nomad_perception.terrain import image_array, classify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', help='Optional directory for captured RGB and classification PNGs')
    options = parser.parse_args()
    outputs = ['observed_map', 'costmap', 'local_costmap', 'current_pose', 'goal',
               'sensors_ready', 'mapping_status', 'terrain_costmap', 'terrain_status', 'terrain_labels']
    args = ['--ros-args', '-r', '__node:=terrain_validation', '-p', 'use_sim_time:=true']
    for topic in outputs:
        args += ['-r', f'/nomad/{topic}:=/nomad/terrain_validation/{topic}']
    args += ['-r', '/goal_pose:=/nomad/terrain_validation/unused_goal']
    rclpy.init(args=args)
    node = SensorInput()
    passed = False
    try:
        deadline = time.monotonic()+25
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if node.camera.active() and np.count_nonzero(node.grid.terrain >= 0) >= 10:
                passed = True
                break
        values, counts = np.unique(node.grid.terrain, return_counts=True)
        if options.output_dir and node.camera.mode == 'colour' and node.camera.rgb:
            destination = Path(options.output_dir)
            destination.mkdir(parents=True, exist_ok=True)
            rgb = image_array(node.camera.rgb[-1])
            labels = classify(rgb, node.camera.dirt_hue, node.camera.field_hue,
                              node.camera.min_saturation, node.camera.min_value)
            palette = np.array([[100, 100, 100], [45, 130, 200], [60, 200, 60]], dtype=np.uint8)
            cv2.imwrite(str(destination/'rgb.png'), rgb)
            cv2.imwrite(str(destination/'labels.png'), palette[labels])
        print(json.dumps({'camera': node.camera.status, 'mapping_fault': node.fault,
                          'terrain_cell_counts': dict(zip(values.tolist(), counts.tolist())),
                          'rgb_frames': len(node.camera.rgb), 'depth_frames': len(node.camera.depth),
                          'passed': passed}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
