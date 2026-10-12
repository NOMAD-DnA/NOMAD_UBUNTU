"""Real RGB-D terrain provider for unchanged NOMAD Planning contracts."""

from array import array
import math, time
from collections import deque
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nomad_interfaces.msg import ModuleStatus
from tf2_ros import TransformException
from .live_terrain import LiveTerrain

# Compatibility exports for existing callers; policies live outside ROS IO.
from .navigation_costmap import (
    NavigationLayers,
    navigation_arrays,
    TemporalBlockEvidence,
)


class NavigationTerrain(LiveTerrain):
    def __init__(self):
        super().__init__()
        q = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.navpub = [
            self.create_publisher(OccupancyGrid, t, q)
            for t in ["/nomad/navigation_costmap", "/nomad/navigation_local_costmap"]
        ]
        self.health = self.create_publisher(
            ModuleStatus, "/nomad/perception/status", 10
        )
        self.odom = None
        self.odom_wall = 0.0
        self.map_wall = 0.0
        self.map_stamp = -np.inf
        self.fault = ""
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/nomad/localization/odometry", self.position, 10
        )
        self.latest_costmaps = None
        self.scan_queue = deque(maxlen=6)
        self.scan_stats = {"received": 0, "integrated": 0, "tf_wait": 0, "expired": 0}
        self.health_timer = self.create_timer(
            0.1, self.health_tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def create_layers(self):
        """Configure navigation policies without creating a second terrain grid."""
        for k, v in [
            ("slope_limit_deg", 20.0),
            ("local_max_age", 30.0),
            ("bootstrap_radius", 3.0),
            ("bootstrap_override", True),
            ("obstacle_cost_radius", 1.2),
            ("passable_semantic_ids", [3]),
        ]:
            self.declare_parameter(k, v)
        return NavigationLayers(
            self.get_parameter("resolution").value,
            self.get_parameter("size").value,
            self.get_parameter("max_size").value,
            self.get_parameter("ground_ids").value,
            slope_limit=self.get_parameter("slope_limit_deg").value,
            bootstrap_override=self.get_parameter("bootstrap_override").value,
            obstacle_cost_radius=self.get_parameter("obstacle_cost_radius").value,
            passable_semantic_ids=self.get_parameter("passable_semantic_ids").value,
        )

    def position(self, m):
        if m.header.frame_id != "odom" or m.child_frame_id != "base_link":
            return
        self.odom = m
        self.odom_wall = time.monotonic()
        self.layers.body_z = m.pose.pose.position.z
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.layers.bootstrap_free(
            m.pose.pose.position.x,
            m.pose.pose.position.y,
            stamp,
            self.get_parameter("bootstrap_radius").value,
            yaw=yaw,
        )
        initial = self.layers.bootstrap_origin
        if (
            math.hypot(
                m.pose.pose.position.x - initial[0], m.pose.pose.position.y - initial[1]
            )
            > initial[3]
        ):
            self.layers.bootstrap_active = False

    def scan(self, m):
        self.scan_stats["received"] += 1
        self.scan_queue.append(m)

    def integrate_pending_scans(self):
        if self.odom is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for m in list(self.scan_queue):
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if now - stamp > 0.6:
                self.scan_queue.remove(m)
                self.scan_stats["expired"] += 1
                continue
            try:
                t = self.buffer.lookup_transform(
                    "odom", m.header.frame_id, Time.from_msg(m.header.stamp)
                ).transform
            except TransformException:
                self.scan_stats["tf_wait"] += 1
                continue
            q = t.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            self.layers.views.grid.integrate(
                (t.translation.x, t.translation.y, yaw),
                m.ranges,
                m.angle_min,
                m.angle_increment,
                m.range_min,
                m.range_max,
                stamp,
            )
            self.scan_queue.remove(m)
            self.scan_stats["integrated"] += 1

    def process(self):
        self.integrate_pending_scans()
        # Collision height uses the same capture-time fused base pose as projection.
        if self.pending is not None:
            depth = self.pending[0]["depth"]
            try:
                self.layers.body_z = self.buffer.lookup_transform(
                    "odom", "base_link", Time.from_msg(depth.header.stamp)
                ).transform.translation.z
            except TransformException:
                return
        super().process()

    def publish(self, stamp, arrays):
        super().publish(stamp, arrays)
        self.map_stamp = (stamp.sec * 1000000000 + stamp.nanosec) * 1e-9
        maps = navigation_arrays(
            self.layers,
            self.estimates,
            arrays,
            self.map_stamp,
            self.get_parameter("slope_limit_deg").value,
            self.get_parameter("local_max_age").value,
        )
        g = self.layers.views.grid
        self.latest_costmaps = maps
        for pub, data in zip(self.navpub, maps):
            m = OccupancyGrid()
            m.header.frame_id = "odom"
            m.header.stamp = stamp
            m.info.resolution = float(g.resolution)
            m.info.width = g.width
            m.info.height = g.height
            m.info.origin.position.x = g.origin_x
            m.info.origin.position.y = g.origin_y
            m.info.origin.orientation.w = 1.0
            m.data = array("b", data.tobytes())
            pub.publish(m)
        self.map_wall = time.monotonic()

    def save(self, arrays=None, report=None):
        super().save(arrays, report)
        if self.directory and getattr(self, "latest_costmaps", None) is not None:
            from pathlib import Path

            g = self.layers.views.grid
            np.savez_compressed(
                Path(self.directory) / "costmaps.npz",
                global_costmap=self.latest_costmaps[0],
                local_costmap=self.latest_costmaps[1],
                origin=[g.origin_x, g.origin_y],
                resolution=g.resolution,
                stamp=self.map_stamp,
                bootstrap_origin=np.asarray(
                    getattr(self.layers, "bootstrap_origin", []) or []
                ),
                scan_received=self.scan_stats["received"],
                scan_integrated=self.scan_stats["integrated"],
                scan_expired=self.scan_stats["expired"],
            )

    def health_tick(self):
        now = self.get_clock().now()
        sec = now.nanoseconds * 1e-9
        wall = time.monotonic()
        ready = (
            self.odom is not None
            and 0 <= sec - self.map_stamp < 2
            and wall - self.map_wall < 3
            and wall - self.odom_wall < 1
        )
        if self.odom is not None:
            age = sec - (
                self.odom.header.stamp.sec + self.odom.header.stamp.nanosec * 1e-9
            )
            ready = ready and 0 <= age < 0.6
        if any("Non-monotonic" in e for e in self.errors):
            self.fault = "CLOCK_RESET: restart map"
        m = ModuleStatus()
        m.header.stamp = now.to_msg()
        m.header.frame_id = "odom"
        m.valid_for.nanosec = 600000000
        m.state = (
            ModuleStatus.FAULT
            if self.fault
            else ModuleStatus.READY if ready else ModuleStatus.WAITING
        )
        m.detail = self.fault or (
            "RGBD terrain + fused pose READY"
            if ready
            else "Waiting for fresh fused pose / RGBD map"
        )
        self.health.publish(m)


def main():
    rclpy.init()
    n = NavigationTerrain()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.save()
        n.destroy_node()
        rclpy.shutdown() if rclpy.ok() else None
