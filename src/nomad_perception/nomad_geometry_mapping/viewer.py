"""Publish a saved manual map to RViz without Gazebo or a simulation clock."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from .archive import load_snapshot, resolve_snapshot, slope_rgba, SLOPE_COLOR_MAX_DEG


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, help="A map.npz/checkpoint.npz or its session directory")
    parser.add_argument("--check", action="store_true", help="Validate and print metadata without starting a ROS node")
    args, ros_arguments = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    if args.check:
        if args.snapshot is None:
            parser.error("--check requires --snapshot")
        path = resolve_snapshot(args.snapshot)
        _, metadata = load_snapshot(path)
        print(json.dumps({"snapshot": str(path), **metadata}, ensure_ascii=False, indent=2))
        return 0

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from sensor_msgs.msg import Image
    from std_msgs.msg import ColorRGBA, String
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    class SavedMapViewer(Node):
        def __init__(self):
            super().__init__("nomad_saved_map_viewer")
            self.set_parameters([Parameter("use_sim_time", value=False)])
            self.declare_parameter("session_directory", "")
            self.declare_parameter("publish_slope_colors", True)
            source = args.snapshot or Path(self.get_parameter("session_directory").value)
            if args.snapshot is None and not self.get_parameter("session_directory").value:
                raise ValueError("Pass --snapshot PATH or the session_directory ROS parameter")
            self.snapshot_path = resolve_snapshot(source)
            self.arrays, self.metadata = load_snapshot(self.snapshot_path)
            self.display_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
            # Give the archive's fixed coordinate frame a display-only identity
            # child. RViz can then recognize the root without a simulator TF.
            self.display_reference = StaticTransformBroadcaster(self)
            reference = TransformStamped()
            reference.header.frame_id = self.metadata['frame_id']
            reference.header.stamp = self.display_clock.now().to_msg()
            reference.child_frame_id = 'manual_saved_reference'
            reference.transform.rotation.w = 1.
            self.display_reference.sendTransform(reference)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.cost_pub = self.create_publisher(OccupancyGrid, "/manual_mapping/costmap", qos)
            self.slope_degrees_pub = self.create_publisher(Image, "/manual_mapping/slope_degrees", qos)
            self.path_pub = self.create_publisher(RosPath, "/manual_mapping/trajectory", qos)
            self.vehicle_pub = self.create_publisher(Marker, "/manual_mapping/vehicle", qos)
            self.slope_pub = self.create_publisher(Marker, "/manual_mapping/slope_colors", qos)
            self.metadata_pub = self.create_publisher(String, "/manual_mapping/metadata", qos)
            # All outputs are latched; a saved map is static and need not be
            # rebuilt every second to serve late RViz subscribers.
            self.publish_map()
            self.get_logger().info(f"Saved map: {self.snapshot_path}; fixed frame {self.metadata['frame_id']}")

        def publish_map(self):
            now = self.display_clock.now().to_msg()
            frame = self.metadata["frame_id"]
            cost = self.arrays["cost"]
            message = OccupancyGrid()
            message.header.frame_id, message.header.stamp = frame, now
            message.info.resolution = float(self.metadata["resolution_m"])
            message.info.width, message.info.height = cost.shape[1], cost.shape[0]
            message.info.origin.position.x = float(self.metadata["origin_x"])
            message.info.origin.position.y = float(self.metadata["origin_y"])
            message.info.origin.orientation.w = 1.
            message.data = cost.ravel().tolist()
            self.cost_pub.publish(message)
            slope_image = Image()
            slope_image.header = message.header
            slope_image.height, slope_image.width = cost.shape
            slope_image.encoding, slope_image.is_bigendian = "32FC1", 0
            slope_image.step = cost.shape[1] * 4
            slope_image.data = self.arrays["slope_deg"].astype("<f4", copy=False).tobytes()
            self.slope_degrees_pub.publish(slope_image)
            trajectory = self.arrays["trajectory"]
            path = RosPath()
            path.header = message.header
            for row in trajectory:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, row[1:4])
                quaternion = row[4:8] / np.linalg.norm(row[4:8])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, quaternion)
                path.poses.append(pose)
            self.path_pub.publish(path)
            vehicle = Marker()
            vehicle.header = message.header
            vehicle.ns, vehicle.id, vehicle.type = "manual_vehicle", 0, Marker.ARROW
            vehicle.pose.orientation.w = 1.
            if path.poses:
                vehicle.action, vehicle.pose = Marker.ADD, path.poses[-1].pose
                vehicle.scale.x, vehicle.scale.y, vehicle.scale.z = 1.1, .35, .25
                vehicle.color.r, vehicle.color.g, vehicle.color.b, vehicle.color.a = .95, .15, .1, 1.
            else:
                vehicle.action = Marker.DELETE
            self.vehicle_pub.publish(vehicle)
            if self.get_parameter("publish_slope_colors").value:
                slopes = self.arrays["slope_deg"]
                rows, columns = np.where(np.isfinite(slopes))
                rgba = slope_rgba(slopes[rows, columns])
                marker = Marker()
                marker.header = message.header
                marker.ns, marker.id, marker.type, marker.action = "manual_slopes", 0, Marker.CUBE_LIST, Marker.ADD
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = message.info.resolution
                marker.scale.z = .02
                marker.color.a = 1.
                for row, column, color in zip(rows, columns, rgba):
                    point = Point()
                    point.x = message.info.origin.position.x + (float(column) + .5) * message.info.resolution
                    point.y = message.info.origin.position.y + (float(row) + .5) * message.info.resolution
                    point.z = .04
                    marker.points.append(point)
                    marker.colors.append(ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=1.))
                self.slope_pub.publish(marker)
            self.metadata_pub.publish(String(data=json.dumps({**self.metadata, "viewer_snapshot": str(self.snapshot_path),
                "display_slope_color_range_deg": [0., SLOPE_COLOR_MAX_DEG], "display_time": "wall clock; original time preserved in stamp_s"}, ensure_ascii=False, allow_nan=False)))

    rclpy.init(args=ros_arguments)
    node = None
    try:
        node = SavedMapViewer()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
