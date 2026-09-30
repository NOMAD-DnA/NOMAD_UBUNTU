"""Unit checks for image stride, timestamps and camera projection matrices."""
import importlib.util
from pathlib import Path
import unittest

from sensor_msgs.msg import CameraInfo, Image

spec = importlib.util.spec_from_file_location(
    "oak_bridge", Path(__file__).parents[1] / "scripts" / "oak_bridge.py")
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class BridgeTests(unittest.TestCase):
    def test_bgr_row_padding_and_stamp(self):
        message = Image(height=2, width=2, encoding="bgr8", step=8)
        message.header.stamp.sec, message.header.stamp.nanosec = 5, 12345
        message.header.frame_id = "oak_left_optical_frame"
        message.data = bytes([0, 0, 255, 255, 255, 255, 99, 99,
                              255, 0, 0, 0, 255, 0, 99, 99])
        result = bridge.mono_image(message)
        self.assertEqual(list(result.data), [76, 255, 29, 150])
        self.assertEqual(result.encoding, "mono8")
        self.assertEqual(result.step, 2)
        self.assertEqual(result.header, message.header)

    def test_rgb_weights(self):
        result = bridge.mono_image(Image(height=1, width=1, encoding="rgb8",
                                        step=3, data=bytes([255, 0, 0])))
        self.assertEqual(list(result.data), [76])

    def test_mono_padding(self):
        result = bridge.mono_image(Image(height=2, width=2, encoding="mono8",
                                        step=3, data=bytes([1, 2, 99, 3, 4, 99])))
        self.assertEqual(list(result.data), [1, 2, 3, 4])

    def test_unsupported_encoding(self):
        with self.assertRaises(ValueError):
            bridge.mono_image(Image(encoding="32FC1"))

    def test_right_projection(self):
        source = CameraInfo()
        source.k = [432.46, 0.0, 320.0, 0.0, 432.97, 240.0, 0.0, 0.0, 1.0]
        source.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        result = bridge.camera_info(source, "oak_right_optical_frame", 0.075)
        self.assertAlmostEqual(result.p[3], -32.4345)
        self.assertEqual(result.p[0], source.k[0])
        self.assertEqual(result.p[2], source.k[2])
        self.assertEqual(result.p[5], source.k[4])
        self.assertEqual(result.p[6], source.k[5])
        self.assertEqual(source.p[3], 0.0)


if __name__ == "__main__":
    unittest.main()
