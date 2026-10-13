"""Measured semantic RGB-D -> persistent 2.5D diagnostic grid. No cost output."""

from nomad_logging import CsvLog

import json, time
from array import array
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from .semantic_input import (
    ExactCaptureSynchronizer,
    project_semantic_points,
    image_pixels,
    stamp_ns,
)
from .traversability_output import transform_points

NAMES = [
    "Background",
    "Trees",
    "Lush Bushes",
    "Dry Grass",
    "Dry Bushes",
    "Ground Clutter",
    "Logs",
    "Rocks",
    "Landscape",
    "Sky",
]
PALETTE = [
    [60, 60, 60],
    [30, 130, 45],
    [80, 200, 65],
    [200, 180, 75],
    [175, 130, 75],
    [210, 100, 150],
    [140, 80, 35],
    [160, 160, 170],
    [175, 105, 55],
    [90, 180, 255],
]

from .terrain_layers import Layers


class LiveTerrain(Node):
    def __init__(self):
        super().__init__("live_terrain")
        for k, v in [
            ("resolution", 0.2),
            ("size", 80.0),
            ("max_size", 160.0),
            ("rate", 10.0),
            ("stride", 4),
            ("min_confidence", 0.5),
            ("ground_ids", [8]),
            ("directory", ""),
            ("record_results", True),
        ]:
            self.declare_parameter(k, v)
        self.layers = self.create_layers()
        self.sync = ExactCaptureSynchronizer(
            streams=("depth", "labels", "confidence", "depth_info", "labels_info"),
            max_pending=90,
            max_age_wall_sec=3.0,
        )
        self.pending = None
        self.estimates = {}
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        self.count = 0
        self.errors = {}
        self.last_status = time.monotonic()
        self.last_save = 0.0
        self.numeric_log = CsvLog('perception', ['processing_ms','frames','height_cells','slope_cells','class_cells','sync_dropped'],interval=1.)
        self.directory = self.get_parameter("directory").value
        q = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid_pub = self.create_publisher(OccupancyGrid, "/nomad/terrain/grid", q)
        self.pubs = {
            k: self.create_publisher(Image, "/nomad/terrain/" + k, q)
            for k in ["height", "slope", "classes"]
        }
        self.status_pub = self.create_publisher(String, "/nomad/terrain/status", q)
        streams = [
            ("depth", Image, "/oak/depth/image_raw"),
            ("labels", Image, "/nomad/semantic/labels"),
            ("confidence", Image, "/nomad/semantic/confidence"),
            ("depth_info", CameraInfo, "/oak/depth/camera_info"),
            ("labels_info", CameraInfo, "/oak/rgb/camera_info"),
        ]
        self.subs = [
            self.create_subscription(
                t, topic, lambda m, k=k: self.receive(k, m), qos_profile_sensor_data
            )
            for k, t, topic in streams
        ]
        self.timer = self.create_timer(
            1 / self.get_parameter("rate").value, self.process
        )
        self.get_logger().info(
            "Actual NN labels + exact-capture depth -> 20cm height/slope/class grid; no driving commands"
        )

    def create_layers(self):
        """Choose the computation layer once, before subscriptions start."""
        return Layers(
            self.get_parameter("resolution").value,
            self.get_parameter("size").value,
            self.get_parameter("max_size").value,
            self.get_parameter("ground_ids").value,
        )

    def receive(self, k, m):
        pair = self.sync.add(k, m)
        if pair:
            self.pending = (pair, time.monotonic())

    def process(self):
        if self.pending is None:
            return
        pair, arrived = self.pending
        depth = pair["depth"]
        stamp = stamp_ns(depth) * 1e-9
        if time.monotonic() - arrived > 3:
            self.pending = None
            return
        try:
            t = self.buffer.lookup_transform(
                "odom", depth.header.frame_id, Time.from_msg(depth.header.stamp)
            )
        except TransformException:
            return
        self.pending = None
        began = time.perf_counter()
        try:
            xyz, ids = project_semantic_points(
                depth,
                pair["labels"],
                pair["depth_info"],
                pair["labels_info"],
                stride=self.get_parameter("stride").value,
                pixel_center_offset_px=0.5,
            )
            z = image_pixels(depth)[
                :: self.get_parameter("stride").value,
                :: self.get_parameter("stride").value,
            ]
            z = z.astype(float) * (0.001 if depth.encoding == "16UC1" else 1.0)
            valid = np.isfinite(z) & (z >= 0.4) & (z <= 8.0)
            if (
                pair["confidence"].encoding != "32FC1"
                or pair["confidence"].header.frame_id != pair["labels"].header.frame_id
            ):
                raise ValueError("Confidence encoding/frame must match semantic image")
            if np.any(ids >= 10):
                raise ValueError("Expected SegFormer 10-class labels")
            confidence = image_pixels(pair["confidence"])[
                :: self.get_parameter("stride").value,
                :: self.get_parameter("stride").value,
            ][valid]
            keep = (
                (ids > 0)
                & (ids < 9)
                & np.isfinite(confidence)
                & (confidence >= self.get_parameter("min_confidence").value)
            )
            xyz = transform_points(xyz[keep], t.transform)
            ids = ids[keep]
            self.estimates = self.layers.ingest(xyz, ids, stamp)
            self.count += 1
            arrays = self.layers.arrays(self.estimates)
            self.publish(depth.header.stamp, arrays)
            report = {
                "frames": self.count,
                "capture_stamp": stamp,
                "processing_ms": (time.perf_counter() - began) * 1000,
                "height_cells": int(np.isfinite(arrays[0]).sum()),
                "slope_cells": int(np.isfinite(arrays[1]).sum()),
                "class_cells": int((arrays[2] >= 0).sum()),
                "sync_dropped": self.sync.dropped,
                "class_names": NAMES,
                "palette": PALETTE,
                "errors": self.errors,
                "ground_ids": list(self.layers.ground_ids),
                "slope_method": "raw_ground_points_tls",
                "height_refit": False,
                "plane_sources": self.layers.core.quality_metadata()[
                    "current_cell_sources"
                ],
            }
            self.numeric_log.emit(stamp,*[report[key] for key in ('processing_ms','frames','height_cells','slope_cells','class_cells','sync_dropped')])
            self.status_pub.publish(String(data=json.dumps(report)))
            if time.monotonic() - self.last_status > 5:
                self.get_logger().info(
                    str(
                        {
                            k: report[k]
                            for k in [
                                "frames",
                                "processing_ms",
                                "height_cells",
                                "slope_cells",
                                "class_cells",
                            ]
                        }
                    )
                )
                self.last_status = time.monotonic()
            if time.monotonic() - self.last_save > 5:
                self.save(arrays, report)
        except (ValueError, IndexError) as e:
            key = str(e)
            self.errors[key] = self.errors.get(key, 0) + 1
            self.get_logger().warning(key)

    def publish(self, stamp, arrays):
        g = self.layers.views.grid
        grid = OccupancyGrid()
        grid.header.frame_id = "odom"
        grid.header.stamp = stamp
        grid.info.resolution = float(g.resolution)
        grid.info.width = g.width
        grid.info.height = g.height
        grid.info.origin.position.x = g.origin_x
        grid.info.origin.position.y = g.origin_y
        grid.info.origin.orientation.w = 1.0
        # Grid is metadata/observed cells only, never a traversability costmap.
        grid.data = array(
            "b", np.where(arrays[2] >= 0, 0, -1).astype(np.int8).tobytes()
        )
        self.grid_pub.publish(grid)
        for name, data, encoding, dtype in zip(
            ["height", "slope", "classes"],
            arrays,
            ["32FC1", "32FC1", "16SC1"],
            ["<f4", "<f4", "<i2"],
        ):
            m = Image()
            m.header = grid.header
            m.height, m.width = data.shape
            m.encoding = encoding
            m.step = m.width * np.dtype(dtype).itemsize
            m.data = data.astype(dtype).tobytes()
            self.pubs[name].publish(m)

    def save(self, arrays=None, report=None):
        if not self.directory or not self.get_parameter("record_results").value:
            return
        root = Path(self.directory)
        root.mkdir(parents=True, exist_ok=True)
        g = self.layers.views.grid
        arrays = arrays or self.layers.arrays(self.estimates)
        np.savez_compressed(
            root / "map.pending.npz",
            height=arrays[0],
            slope=arrays[1],
            classes=arrays[2],
            origin=[g.origin_x, g.origin_y],
            resolution=g.resolution,
        )
        (root / "map.pending.npz").replace(root / "map.npz")
        if report:
            (root / "status.json").write_text(json.dumps(report, indent=2))
        self.last_save = time.monotonic()


def main():
    rclpy.init()
    n = LiveTerrain()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.save()
        n.destroy_node()
        rclpy.shutdown() if rclpy.ok() else None
