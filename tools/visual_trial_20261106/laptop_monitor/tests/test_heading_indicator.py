"""Heading display contracts; no sensors, board processes or network needed."""
import json
import math
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtGui, QtWidgets
from heading_indicator import HeadingIndicator


def pose(degrees=0):
    angle = math.radians(degrees) / 2
    return dict(qx=0., qy=0., qz=math.sin(angle), qw=math.cos(angle))


class HeadingIndicatorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.widget = HeadingIndicator()
        self.addCleanup(self.widget.close)

    def data(self, p=None, axis=(1., 0., 0.), age=0., values=None, diag_age=0., source="public", **extra):
        diagnostics = dict(values=values or {}, **extra)
        self.widget.set_data(p or pose(), axis, age, diagnostics, diag_age, source)
        return self.widget.state()

    def test_local_cardinals_and_global_bearings(self):
        for yaw, bearing in ((0, 90), (90, 0), (180, 270), (-90, 180)):
            with self.subTest(yaw=yaw):
                local = self.data(pose(yaw))
                self.assertEqual(local["mode"], "local")
                self.assertEqual(local["ring_labels"], ["+Y", "+X", "−Y", "−X"])
                self.assertAlmostEqual(local["yaw_deg"], yaw)
                self.assertIsNone(local["bearing_deg"])
                global_ = self.data(pose(yaw), values={"global_heading_valid": True})
                self.assertEqual(global_["ring_labels"], ["N", "E", "S", "W"])
                self.assertAlmostEqual(global_["bearing_deg"], bearing)

    def test_full_attitude_rotates_calibrated_axis_not_just_imu_yaw(self):
        # IMU +Z points down here; saved camera front -Z is world +Y.
        half = math.sqrt(.5)
        p = dict(qx=half, qy=0., qz=0., qw=half)
        state = self.data(p, axis=(0., 0., -1.))
        self.assertAlmostEqual(state["yaw_deg"], 90)
        self.assertAlmostEqual(state["arrow_xy"][0], 0.)
        self.assertAlmostEqual(state["arrow_xy"][1], 1.)
        state = self.data(pose(90), axis=(0., 1., 0.))
        self.assertAlmostEqual(abs(state["yaw_deg"]), 180)

    def test_diagnostic_flags_require_exact_true(self):
        for value in (False, "false", "False", "yes", "1", 1, [], {}, None):
            with self.subTest(value=value):
                self.assertFalse(self.data(values={"global_heading_valid": value})["global_valid"])
        for value in (True, "true", "TRUE"):
            self.assertTrue(self.data(values={"global_heading_valid": value})["global_valid"])

    def test_stale_relay_diagnostics_do_not_confirm_global_heading(self):
        values = {"global_heading_valid": True}
        for age, measurement in ((.51, 0.), (.3, .3), (None, 0.), (0., float("nan")), (0., -.1)):
            state = self.data(values=values, diag_age=age, measurement_age_s=measurement)
            self.assertFalse(state["global_valid"])
            self.assertEqual(state["mode"], "local")
            json.dumps(state, allow_nan=False)
        self.assertTrue(self.data(values=values, diag_age=.2, measurement_age_s=.3)["global_valid"])

    def test_fusion_or_public_only_never_raw_vio_global(self):
        for source, expected in (("public", True), ("fusion", True), ("vio", False), ("garbage", False)):
            self.assertEqual(self.data(values={"global_heading_valid": True}, source=source)["global_valid"], expected)

    def test_provisional_magnetic_heading_is_explicit_and_never_global_valid(self):
        values = {"heading_provisional": "true", "orientation_valid": "true",
                  "global_heading_valid": "false"}
        for source in ("public", "fusion"):
            state = self.data(pose(90), source=source, values=values)
            self.assertEqual(state["mode"], "provisional")
            self.assertTrue(state["provisional"])
            self.assertFalse(state["global_valid"])
            self.assertEqual(state["note"], "임시 방위")
            self.assertEqual(state["ring_labels"], ["N", "E", "S", "W"])
            self.assertAlmostEqual(state["bearing_deg"], 0.)
        # Calibration/global validity has its own explicit flag and precedence.
        confirmed = self.data(values=dict(values, global_heading_valid="true"))
        self.assertTrue(confirmed["global_valid"])
        self.assertFalse(confirmed["provisional"])
        self.assertEqual(confirmed["mode"], "global")

    def test_provisional_requires_fresh_flags_and_valid_orientation_on_fusion_pose(self):
        values = {"heading_provisional": True, "orientation_valid": True}
        for options in ({"diag_age": .6}, {"measurement_age_s": .6}, {"source": "vio"}):
            state = self.data(values=values, **options)
            self.assertEqual(state["mode"], "local")
            self.assertFalse(state["provisional"])
            self.assertFalse(state["global_valid"])
            self.assertIsNone(state["bearing_deg"])
        for flag in (False, "false", "1", 1, None):
            self.assertFalse(self.data(values=dict(values, heading_provisional=flag))["provisional"])
        for orientation in (None, "yes", 1):
            self.assertFalse(self.data(values=dict(values, orientation_valid=orientation))["provisional"])
        invalid = self.data(values=dict(values, orientation_valid="false"))
        self.assertIsNone(invalid["arrow_xy"])
        self.assertFalse(invalid["provisional"])
        stale = self.data(values=values, age=.6)
        self.assertIsNone(stale["arrow_xy"])
        self.assertFalse(stale["provisional"])

    def test_missing_stale_and_invalid_attitude_clear_arrow(self):
        self.widget.set_data(None, (1., 0., 0.), 0.)
        self.assertEqual(self.widget.state()["reason"], "pose_missing")
        for age in (None, .501, -1., float("nan")):
            state = self.data(age=age, values={"global_heading_valid": True})
            self.assertIsNone(state["arrow_xy"])
            self.assertEqual(state["reason"], "pose_stale")
        for q in (dict(qx=0., qy=0., qz=0., qw=0.),
                  dict(qx=0., qy=0., qz=0., qw=2.),
                  dict(qx=float("nan"), qy=0., qz=0., qw=1.), dict(qw=1.)):
            state = self.data(q)
            self.assertFalse(state["valid"])
            self.assertIsNone(state["arrow_xy"])
            self.assertEqual(state["reason"], "attitude_invalid")

    def test_vertical_and_invalid_axis_do_not_invent_yaw(self):
        vertical = self.data(axis=(0., 0., 1.))
        self.assertEqual(vertical["reason"], "forward_vertical")
        self.assertIsNone(vertical["arrow_xy"])
        for axis in ((0., 0., 0.), (1., 2.), (1., float("inf"), 0.)):
            self.assertFalse(self.data(axis=axis)["valid"])

    def test_explicit_invalid_orientation_is_not_displayed_as_valid(self):
        for flag in (False, "false", "FALSE"):
            state = self.data(values={"global_heading_valid": True, "orientation_valid": flag})
            self.assertFalse(state["global_valid"])
            self.assertIsNone(state["arrow_xy"])
            self.assertEqual(state["reason"], "orientation_invalid")

    def test_rendering_is_local_and_snapshot_has_no_aliases(self):
        with mock.patch("socket.socket", side_effect=AssertionError("network prohibited")), \
                mock.patch("subprocess.Popen", side_effect=AssertionError("process prohibited")), \
                mock.patch("builtins.open", side_effect=AssertionError("file IO prohibited")):
            self.data(pose(30))
            image = QtGui.QImage(self.widget.size(), QtGui.QImage.Format_ARGB32)
            self.widget.render(image)
            self.assertEqual(image.width(), 104)
            self.assertFalse(image.isNull())
            snapshot = self.widget.state()
            snapshot["ring_labels"][0] = "bad"
            snapshot["arrow_xy"][0] = 999
            self.assertEqual(self.widget.state()["ring_labels"][0], "+Y")
            self.assertNotEqual(self.widget.state()["arrow_xy"][0], 999)
            json.dumps(self.widget.state(), allow_nan=False)


if __name__ == "__main__":
    unittest.main()
