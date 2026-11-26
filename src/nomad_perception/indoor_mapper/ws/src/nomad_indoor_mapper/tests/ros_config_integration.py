#!/usr/bin/env python3
"""Synthetic inputs in a private ROS domain; starts no real sensors or controllers."""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Point32
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from nav_msgs.msg import Odometry
from nomad_interfaces.msg import ModuleStatus, PlanningCostLayers
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[4]
PKG = Path(__file__).resolve().parents[1]
checks = []


def check(condition, name):
    if not condition:
        raise AssertionError(name)
    checks.append(name)


def ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


class Synthetic(Node):
    def __init__(self):
        super().__init__('indoor_synthetic_verification')
        self.loc = self.create_publisher(ModuleStatus, '/nomad/localization/status', 10)
        self.odom = self.create_publisher(Odometry, '/nomad/localization/odometry', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.cloud = self.create_publisher(PointCloud, '/point_cloud', qos_profile_sensor_data)
        self.bundle = None
        self.state = None
        self.map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PlanningCostLayers, '/nomad/planning_cost_layers', self.on_bundle, self.map_qos)
        self.create_subscription(ModuleStatus, '/nomad/perception/status', self.on_state, 10)
        self.broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'oak_imu_frame'
        tf.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(tf)
        self.phase = 'high'
        self.emit = True
        self.last_scan = 0.0
        self.sent_stamps = set()
        self.incoming_z = 100.0
        self.create_timer(.02, self.inputs)

    def on_bundle(self, m):
        self.bundle = m

    def on_state(self, m):
        self.state = m

    def inputs(self):
        now = self.get_clock().now().to_msg()
        loc = ModuleStatus()
        loc.header.frame_id = 'odom'
        loc.header.stamp = now
        loc.state = ModuleStatus.READY
        loc.valid_for.sec = 1
        self.loc.publish(loc)
        odom = Odometry()
        odom.header = loc.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.z = self.incoming_z
        self.odom.publish(odom)
        if not self.emit or time.monotonic() - self.last_scan < .10:
            return
        self.last_scan = time.monotonic()
        x = 4.0625 if self.phase == 'clear' else 2.0625
        z = .20 if self.phase == 'high' else 0.
        scan = LaserScan()
        scan.header.frame_id = 'g2_laser'
        scan.header.stamp = now
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 2
        scan.range_min = .01
        scan.range_max = 10.
        scan.ranges = [math.hypot(x, .0625)] * 5
        cloud = PointCloud()
        cloud.header = scan.header
        cloud.points = [Point32(x=x, y=.0625, z=z), Point32(x=x, y=.0625, z=z)]
        cloud.channels = [ChannelFloat32(name='stamps', values=[0., .01])]
        self.sent_stamps.add(ns(scan.header))
        self.scan.publish(scan)
        self.cloud.publish(cloud)

    def wait(self, condition, timeout=8.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.03)
            if condition():
                return
        raise AssertionError('Timed out waiting for expected ROS output')

    def cell(self, x=2.0625, y=.0625):
        if self.bundle is None:
            return None
        info = self.bundle.grid.info
        col = math.floor((x-info.origin.position.x)/info.resolution)
        row = math.floor((y-info.origin.position.y)/info.resolution)
        return row*info.width+col if 0 <= col < info.width and 0 <= row < info.height else None

    def blocked(self):
        i = self.cell()
        return i is not None and self.bundle.grid.data[i] == 100 and math.isnan(self.bundle.slope_cost[i]) and self.bundle.terrain_cost[i] == 1.

    def cleared(self):
        i = self.cell()
        return i is not None and self.bundle.grid.data[i] == 0 and self.bundle.slope_cost[i] == 0. and self.bundle.terrain_cost[i] == 0.


def main():
    config = json.loads((PKG/'config/indoor.json').read_text())
    # Isolate mapping rules from the hardware calibration, tested separately in native tests.
    config['floor_z_cm'] = 35.0
    config['vio_fixed_z_m'] = 0.25
    config['T_imu_from_lidar'] = [[int(r == c) for c in range(4)] for r in range(4)]
    config['moving_free_region']['enabled'] = False
    config['initialization_stationary_hold_s'] = 0.
    config['cost_area'] = 'map_window'
    rclpy.init()
    harness = Synthetic()
    process = None
    result = ROOT/'ros_config_integration_result.json'
    try:
        with tempfile.TemporaryDirectory(prefix='indoor_synthetic_') as temporary:
            cfg = Path(temporary)/'indoor.json'
            cfg.write_text(json.dumps(config))
            with (ROOT/'ros_config_integration_mapper.log').open('w') as log:
                binary = ROOT/'ws/install/nomad_indoor_mapper/lib/nomad_indoor_mapper/nomad_indoor_mapper'
                process = subprocess.Popen([str(binary), '--indoor-config', str(cfg)], stdout=log, stderr=subprocess.STDOUT)
                harness.wait(lambda: harness.blocked() and harness.state is not None and harness.state.state == ModuleStatus.READY)
                check(harness.blocked(), 'Unknown slope + obstacle100 publishes final100 without cameras')
                check(harness.state.state == ModuleStatus.READY, 'READY with VIO and LiDAR only')
                check(harness.incoming_z == 100., 'Incoming VIO Z100 did not translate the point cloud')
                bundle = harness.bundle
                check(bundle.grid.header.frame_id == 'odom' and abs(bundle.grid.info.resolution-.125) < 1e-8, 'Existing frame and grid resolution')
                check(len(bundle.slope_cost) == len(bundle.terrain_cost) == len(bundle.grid.data), 'PlanningCostLayers array sizes')
                # deskew times are not rewritten to processing-time now().
                output_ns = ns(bundle.grid.header)
                check(any(abs(output_ns-(s+10_000_000)) <= 2 for s in harness.sent_stamps), 'Published capture uses actual per-return stamp')
                subscriptions = harness.get_subscriber_names_and_types_by_node('nomad_indoor_mapper', '/')
                names = {name for name, _ in subscriptions}
                check({'/scan', '/point_cloud', '/nomad/localization/odometry', '/nomad/localization/status'} <= names, 'Existing LiDAR and VIO inputs retained')
                check(not any(name.startswith('/oak/') or name.startswith('/zed/') or name.startswith('/perception/') for name in names), 'No OAK, ZED, labels or factory inputs')
                expected_outputs = {'/nomad/navigation_costmap','/nomad/navigation_local_costmap','/nomad/camera_observation_support',
                    '/nomad/planning_cost_layers','/nomad/perception/status','/nomad/semantic_validation/cost_layers',
                    '/nomad/semantic_validation/capture_timing','/nomad/semantic_validation/slope_score',
                    '/nomad/semantic_validation/semantic_score','/nomad/semantic_validation/slope_deg','/nomad/semantic_validation/slope_magnitude_deg'}
                publishers = {name for name, _ in harness.get_publisher_names_and_types_by_node('nomad_indoor_mapper','/')}
                check(expected_outputs <= publishers, 'All eleven original output topics retained')
                high_stamp = ns(harness.bundle.grid.header)
                harness.phase = 'floor'
                harness.wait(lambda: ns(harness.bundle.grid.header) > high_stamp+1_200_000_000 and harness.blocked(), 4.)
                check(harness.blocked(), 'Floor endpoint and elapsed1s cannot release occupancy')
                harness.phase = 'clear'
                harness.wait(harness.cleared, 3.)
                check(harness.cleared(), 'New XY crossing ray restores paved0 and slope0 without floor observation')
                harness.phase = 'high'
                harness.incoming_z = -200.
                harness.wait(harness.blocked, 3.)
                check(harness.blocked(), 'New obstacle reblocks after release; changed VIO Z ignored')
                harness.emit = False
                harness.wait(lambda: harness.state.state != ModuleStatus.READY, 4.)
                check(harness.state.state != ModuleStatus.READY, 'Expired LiDAR revokes READY')
        result.write_text(json.dumps({'status':'passed','checks':checks,'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),
            'synthetic_only':True,'hardware_or_gui_started':False}, indent=2)+'\n')
        print(json.dumps({'checks_passed':len(checks), 'result':str(result)}))
    finally:
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
