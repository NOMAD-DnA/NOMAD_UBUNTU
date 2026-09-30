"""G2 snapshot filtering tests using fake packets; no ROS or simulator needed."""
from array import array
import ast
import importlib.util
import math
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "lidar_bridge.py"
spec = importlib.util.spec_from_file_location("nomad_gazebo_lidar_bridge", SCRIPT)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def float32(value):
    return array("f", [value])[0]


def adjacent_float32(value, direction):
    # Test the exact positive float32 samples adjacent to each range limit.
    bits = struct.unpack("!I", struct.pack("!f", value))[0]
    return struct.unpack("!f", struct.pack("!I", bits + direction))[0]


def packet(ranges=None):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="scan", stamp=SimpleNamespace(sec=1790663000, nanosec=123456789)),
        ranges=array("f", [1.0] * 500 if ranges is None else ranges),
        intensities=array("f", [2.0] * 500),
        angle_min=-math.pi, angle_max=math.pi - 2 * math.pi / 500,
        angle_increment=2 * math.pi / 500,
        range_min=0.01, range_max=20.0, scan_time=0.0, time_increment=0.0002,
    )


class LidarBridgeTests(unittest.TestCase):
    def test_default_500_ray_packet_limits_and_snapshot_timing(self):
        source = packet()
        result = bridge.scan_message(source, 0.12, 12.0, 0.1)
        self.assertEqual(len(result.ranges), 500)
        self.assertEqual(result.range_min, float32(0.12))
        self.assertEqual(result.range_max, 12.0)
        self.assertEqual(result.ranges, source.ranges)
        self.assertEqual(result.scan_time, 0.1)
        self.assertEqual(result.time_increment, 0.0)
        self.assertEqual(source.scan_time, 0.0)
        self.assertEqual(source.time_increment, 0.0002)

    def test_rep117_boundaries_nan_and_infinities(self):
        minimum, maximum = float32(0.12), float32(12.0)
        values = [0.0, -0.0, -1.0, math.nan, -math.inf, math.inf,
                  adjacent_float32(minimum, -1), minimum,
                  adjacent_float32(minimum, 1), 1.0,
                  adjacent_float32(maximum, -1), maximum,
                  adjacent_float32(maximum, 1)]
        result = bridge.clip_ranges(values, 0.12, 12.0)
        for index in (0, 1, 2, 3):
            self.assertTrue(math.isnan(result[index]))
        for index in (4, 6):
            self.assertEqual(result[index], -math.inf)
        for index in (5, 11, 12):
            self.assertEqual(result[index], math.inf)
        for index in (7, 8, 9, 10):
            self.assertEqual(result[index], values[index])

    def test_native_stamp_frame_angles_and_intensities_preserved(self):
        source = packet()
        source.header.frame_id = "ydlidar_native_frame"
        result = bridge.scan_message(source, 0.12, 12.0, 0.1)
        self.assertEqual(result.header, source.header)
        self.assertIsNot(result.header, source.header)
        self.assertIsNot(result.header.stamp, source.header.stamp)
        for name in ("angle_min", "angle_max", "angle_increment", "intensities"):
            self.assertEqual(getattr(result, name), getattr(source, name))
        self.assertEqual(result.header.stamp.nanosec, 123456789)
        self.assertEqual(result.header.frame_id, "ydlidar_native_frame")

    def test_source_packet_is_not_mutated_and_shape_is_not_invented(self):
        source = packet([0.01, 0.12, 1.0, 12.0, math.nan])
        result = bridge.scan_message(source, 0.12, 12.0, 0.1)
        self.assertEqual(len(result.ranges), len(source.ranges))
        self.assertIsNot(result.ranges, source.ranges)
        self.assertEqual(source.ranges[0], float32(0.01))
        self.assertEqual(source.ranges[3], 12.0)
        self.assertTrue(math.isnan(source.ranges[4]))
        self.assertEqual(source.range_min, 0.01)
        self.assertEqual(source.range_max, 20.0)
        self.assertEqual(result.intensities, source.intensities)

    def test_invalid_range_profiles_and_periods_are_rejected(self):
        invalid = [(12.0, 12.0, 0.1), (13.0, 12.0, 0.1), (-0.1, 12.0, 0.1),
                   (0.12, math.inf, 0.1), (math.nan, 12.0, 0.1),
                   (0.12, 12.0, 0.0), (0.12, 12.0, -0.1),
                   (0.12, 12.0, math.nan), (0.12, 12.0, math.inf)]
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                bridge.scan_message(packet(), *values)
        for minimum, maximum in ((12, 12), (-1, 12), (0.12, math.inf)):
            with self.subTest(minimum=minimum), self.assertRaises(ValueError):
                bridge.clip_ranges([1.0], minimum, maximum)

    def test_empty_packet_and_generator_input(self):
        result = bridge.scan_message(packet([]), 0.12, 12.0, 0.1)
        self.assertEqual(len(result.ranges), 0)
        result = bridge.clip_ranges((x for x in [0.05, 1.0, 20.0]), 0.12, 12.0)
        self.assertEqual(list(result), [-math.inf, 1.0, math.inf])

    def test_ros_imports_are_deferred_and_ros_topic_defaults_unchanged(self):
        tree = ast.parse(SCRIPT.read_text())
        top_level_imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
        for node in top_level_imports:
            modules = [alias.name for alias in node.names] if isinstance(node, ast.Import) else [node.module]
            self.assertFalse(any(name.startswith(("rclpy", "sensor_msgs")) for name in modules))
        strings = {node.value for node in ast.walk(tree) if isinstance(node, ast.Constant)
                   and isinstance(node.value, str)}
        self.assertIn("/nomad/raw/scan", strings)
        self.assertIn("/scan", strings)
        self.assertIn("scan_hz", strings)
        self.assertTrue(any(isinstance(node, ast.Attribute) and node.attr == "RELIABLE"
                            for node in ast.walk(tree)))


if __name__ == "__main__":
    unittest.main()
