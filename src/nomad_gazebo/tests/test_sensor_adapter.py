"""Byte-level checks; no ROS installation or running simulator is required."""
import array
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location(
    "gazebo_sensor_adapter", Path(__file__).parents[1] / "scripts" / "gazebo_sensor_adapter.py")
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def image(width, height, encoding, data, step, bigendian=0):
    return SimpleNamespace(width=width, height=height, encoding=encoding,
                           data=data, step=step, is_bigendian=bigendian,
                           header=SimpleNamespace(frame_id="raw_frame",
                                                  stamp=SimpleNamespace(sec=7, nanosec=123456789)))


def depth(values, bigendian=0, padding=b""):
    values = np.asarray(values, dtype=(">" if bigendian else "<") + "f4")
    height, width = values.shape
    data = b"".join(row.tobytes() + padding for row in values)
    return image(width, height, "32FC1", data, width * 4 + len(padding), bigendian)


def uint16_pixels(source):
    return np.ndarray((source.height, source.width),
                      dtype=(">" if source.is_bigendian else "<") + "u2",
                      buffer=source.data, strides=(source.step, 2)).tolist()


class SensorAdapterTests(unittest.TestCase):
    def test_rgb_bgr_swap_preserves_padding_header_and_source(self):
        source = image(2, 2, "rgb8", bytes([255, 0, 0, 0, 255, 0, 91, 92,
                                           0, 0, 255, 1, 2, 3, 93, 94]), 8, 1)
        result = adapter.normalize_image(source, "rgb")
        self.assertEqual(result.encoding, "bgr8")
        self.assertEqual(result.step, 8)
        self.assertEqual(result.is_bigendian, 1)
        self.assertIsInstance(result.data, array.array)
        self.assertEqual(result.data.typecode, "B")
        self.assertEqual(bytes(result.data), bytes([0, 0, 255, 0, 255, 0, 91, 92,
                                             255, 0, 0, 3, 2, 1, 93, 94]))
        self.assertEqual(result.header.stamp, source.header.stamp)
        self.assertIsNot(result.header, source.header)
        self.assertEqual(result.header.frame_id, "oak_rgb_optical_frame")
        self.assertEqual(source.header.frame_id, "raw_frame")
        self.assertEqual(source.data[:3], bytes([255, 0, 0]))
        self.assertEqual(adapter.color_image(result).data, result.data)

    def test_mono_luminance_and_8bit_input_preserve_row_padding(self):
        source = image(2, 2, "bgr8", bytes([0, 0, 255, 255, 255, 255, 99, 98,
                                           255, 0, 0, 0, 255, 0, 97, 96]), 8)
        result = adapter.normalize_image(source, "left")
        self.assertEqual((result.encoding, result.step), ("mono8", 4))
        self.assertEqual(bytes(result.data), bytes([76, 255, 99, 98, 29, 150, 97, 96]))
        self.assertEqual(result.header.frame_id, "oak_left_optical_frame")
        mono = image(2, 2, "8UC1", bytes([1, 2, 90, 3, 4, 91]), 3)
        normalized = adapter.normalize_image(mono, "right")
        self.assertEqual((normalized.encoding, normalized.step), ("mono8", 3))
        self.assertEqual(bytes(normalized.data), mono.data)

    def test_depth_invalids_inclusive_float32_limits_and_no_wrapping(self):
        lower = np.float32(0.4)
        upper = np.float32(8.0)
        source = depth([[np.nan, np.inf, -np.inf, -1, 0,
                         np.nextafter(lower, -np.inf, dtype=np.float32), lower,
                         np.nextafter(lower, np.inf, dtype=np.float32),
                         1.2346, upper, np.nextafter(upper, np.inf, dtype=np.float32),
                         70.0, 1e30]])
        result = adapter.normalize_image(source, "depth")
        self.assertEqual(result.encoding, "16UC1")
        self.assertEqual(uint16_pixels(result), [[0, 0, 0, 0, 0, 0, 400, 400, 1235, 8000, 0, 0, 0]])
        self.assertEqual(result.header.stamp, source.header.stamp)
        self.assertEqual(result.header.frame_id, "oak_depth_optical_frame")
        wide = adapter.depth_image(depth([[65.535, 65.536, 70.0]]), maximum_m=100.0)
        self.assertEqual(uint16_pixels(wide), [[65535, 0, 0]])

    def test_bigendian_padded_float_depth_and_existing_mm(self):
        source = depth([[0.4, 1.2346], [8, np.nan]], bigendian=1, padding=b"\x99\x88")
        result = adapter.depth_image(source)
        self.assertEqual((result.step, result.is_bigendian), (6, 1))
        self.assertEqual(bytes(result.data), b"\x01\x90\x04\xd3\x99\x88\x1f\x40\x00\x00\x99\x88")
        self.assertEqual(uint16_pixels(result), [[400, 1235], [8000, 0]])
        mm = image(5, 1, "16UC1", np.asarray([399, 400, 8000, 8001, 65535], ">u2").tobytes(), 10, 1)
        self.assertEqual(uint16_pixels(adapter.depth_image(mm)), [[0, 400, 8000, 0, 0]])

    def test_camera_info_preserves_native_intrinsics_and_stamps(self):
        source = SimpleNamespace(
            width=640, height=480,
            header=SimpleNamespace(frame_id="native", stamp=SimpleNamespace(sec=5, nanosec=10)),
            k=[432.46, 0.0, 320.0, 0.0, 432.97, 240.0, 0.0, 0.0, 1.0],
            d=[0.01, -0.02, 0, 0, 0], distortion_model="plumb_bob",
            r=[0.0] * 9, p=[0.0] * 12)
        right = adapter.camera_info(source, "right")
        self.assertEqual(right.k, source.k)
        self.assertEqual(right.d, source.d)
        self.assertEqual(right.header.stamp, source.header.stamp)
        self.assertEqual(right.header.frame_id, "oak_right_optical_frame")
        self.assertAlmostEqual(right.p[3], -32.4345)
        self.assertEqual(right.r, [1, 0, 0, 0, 1, 0, 0, 0, 1])
        for side in ("rgb", "depth", "left"):
            self.assertEqual(adapter.camera_info(source, side).p[3], 0.0)
        self.assertEqual(source.p, [0.0] * 12)
        self.assertEqual(source.header.frame_id, "native")

    def test_provisional_fallback_matches_rgb_depth_and_scales_explicit_K(self):
        rgb = adapter.fallback_intrinsics({}, "rgb", 640, 480)
        self.assertEqual(rgb, adapter.fallback_intrinsics({}, "depth", 640, 480))
        self.assertEqual(rgb[2:3], [320.0])
        self.assertEqual(rgb[5], 240.0)
        mono = adapter.fallback_intrinsics({}, "left", 640, 480)
        self.assertLess(mono[0], rgb[0])
        explicit = dict(rgb_fx=500, rgb_fy=510, cx=319, cy=241, width=640, height=480)
        small = adapter.fallback_intrinsics(explicit, "rgb", 320, 240)
        self.assertEqual(small, [250.0, 0.0, 159.5, 0.0, 255.0, 120.5, 0.0, 0.0, 1.0])

    def test_malformed_images_and_unsupported_encodings_fail_explicitly(self):
        bad = [image(2, 1, "rgb8", b"12345", 5),
               image(2, 1, "rgb8", b"12345", 6),
               image(2, 1, "rgb8", b"1234567", 6),
               image(0, 1, "rgb8", b"", 0),
               image(1, 1, "rgb8", b"123", 3, 2),
               image(1, 1, "8UC3", b"123", 3)]
        for source in bad:
            with self.subTest(source=source), self.assertRaises(ValueError):
                adapter.color_image(source)
        with self.assertRaises(ValueError):
            adapter.depth_image(image(1, 1, "64FC1", b"0" * 8, 8))
        with self.assertRaises(ValueError):
            adapter.normalize_image(image(1, 1, "mono8", b"1", 1), "rear")

    def test_invalid_limits_calibration_and_fallback_are_rejected(self):
        for limits in ((8, 0.4), (-1, 8), (0.4, np.inf)):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                adapter.depth_image(depth([[1.0]]), *limits)
        with self.assertRaises(ValueError):
            adapter.fallback_intrinsics(dict(rgb_hfov_deg=180), "rgb", 640, 480)
        info = SimpleNamespace(width=640, height=480, k=[0.0] * 9)
        with self.assertRaises(ValueError):
            adapter.camera_info(info, "rgb")

    def test_ros_style_array_fast_path_avoids_per_byte_validation(self):
        class CountedImage:
            """Mirror Image.data's array fast path without requiring ROS."""
            def __init__(self):
                self.validated_bytes = 0

            @property
            def data(self):
                return self._data

            @data.setter
            def data(self, value):
                if isinstance(value, array.array):
                    assert value.typecode == "B"
                    self._data = value
                    return
                self.validated_bytes += 2 * len(value)
                self._data = array.array("B", value)

        source = CountedImage()
        original = image(2, 1, "rgb8", b"\x01\x02\x03\x04\x05\x06\x91\x92", 8, 1)
        for key, value in vars(original).items():
            setattr(source, key, array.array("B", value) if key == "data" else value)
        result = adapter.normalize_image(source, "rgb")
        self.assertEqual(result.validated_bytes, 0)
        self.assertEqual(bytes(result.data), b"\x03\x02\x01\x06\x05\x04\x91\x92")
        self.assertEqual((result.step, result.is_bigendian, result.width, result.height), (8, 1, 2, 1))
        self.assertEqual(result.header.stamp, source.header.stamp)
        self.assertEqual(result.header.frame_id, "oak_rgb_optical_frame")
        self.assertEqual(source.header.frame_id, "raw_frame")


if __name__ == "__main__":
    unittest.main()
