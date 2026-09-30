"""Check range clipping, snapshot timing and launch validation without ROS I/O."""
import importlib.util
import math
from pathlib import Path
import unittest

import yaml
from sensor_msgs.msg import LaserScan


def load_module(name, relative_path):
    spec = importlib.util.spec_from_file_location(
        name, Path(__file__).parents[1] / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = load_module("lidar_bridge", "scripts/lidar_bridge.py")
launch = load_module("nomad_elevation_launch", "launch/elevation.launch.py")


class LidarBridgeTests(unittest.TestCase):
    def test_range_clipping_and_source_unchanged(self):
        source = LaserScan(ranges=[0.0, 0.119, 0.12, 1.0, 11.999, 12.0,
                                  12.01, math.inf, -math.inf, math.nan, -1.0])
        result = bridge.scan_message(source, 0.12, 12.0, 0.1)
        valid_indices = {2, 3, 4}
        for index, value in enumerate(result.ranges):
            if index in valid_indices:
                self.assertEqual(value, source.ranges[index])
            elif index in (0, 9, 10):
                self.assertTrue(math.isnan(value))
            elif index in (1, 8):
                self.assertEqual(value, -math.inf)
            else:
                self.assertEqual(value, math.inf)
        self.assertEqual(source.ranges[0], 0.0)
        self.assertEqual(source.ranges[5], 12.0)
        self.assertTrue(math.isnan(source.ranges[9]))
        self.assertAlmostEqual(result.range_min, 0.12)
        self.assertEqual(result.range_max, 12.0)

    def test_header_geometry_and_intensities_preserved(self):
        source = LaserScan(angle_min=-math.pi, angle_max=math.pi,
                           angle_increment=2*math.pi/499,
                           ranges=[1.0, 2.0], intensities=[3.0, 4.0])
        source.header.stamp.sec, source.header.stamp.nanosec = 17, 123456
        source.header.frame_id = "scan"
        result = bridge.scan_message(source, 0.12, 12.0, 0.1)
        self.assertEqual(result.header, source.header)
        self.assertEqual(result.angle_min, source.angle_min)
        self.assertEqual(result.angle_max, source.angle_max)
        self.assertEqual(result.angle_increment, source.angle_increment)
        self.assertEqual(result.intensities, source.intensities)

    def test_snapshot_timing(self):
        source = LaserScan(scan_time=0.0, time_increment=0.0002)
        result = bridge.scan_message(source, 0.12, 12.0, 0.1)
        self.assertEqual(result.scan_time, 0.1)
        self.assertEqual(result.time_increment, 0.0)
        self.assertEqual(source.time_increment, 0.0002)

    def test_invalid_limits_or_period(self):
        for values in ((12.0, 12.0, 0.1), (-0.1, 12.0, 0.1),
                       (0.12, math.inf, 0.1), (math.nan, 12.0, 0.1),
                       (0.12, 12.0, 0.0)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                bridge.scan_message(LaserScan(), *values)

    def configuration(self):
        with (Path(__file__).parents[1] / "config" / "sensors.yaml").open() as stream:
            return yaml.safe_load(stream)

    def test_g2_default_configuration(self):
        config = self.configuration()
        values = launch.sensor_environment(config, "/example/mvsim")
        self.assertEqual(config["lidar"]["min_range_m"], 0.12)
        self.assertEqual(float(values["NOMAD_LIDAR_MAX_RANGE"]), 12.0)
        self.assertEqual(int(values["NOMAD_LIDAR_NRAYS"]), 500)
        self.assertEqual(float(values["NOMAD_LIDAR_PERIOD"]), 0.1)
        self.assertEqual(config["lidar"]["nrays"]*config["lidar"]["hz"], 5000)

    def test_launch_rejects_invalid_minimum(self):
        for minimum in (-0.1, 12.0, math.inf):
            config = self.configuration()
            config["lidar"]["min_range_m"] = minimum
            with self.subTest(minimum=minimum), self.assertRaises(ValueError):
                launch.sensor_environment(config, "/example/mvsim")


if __name__ == "__main__":
    unittest.main()
