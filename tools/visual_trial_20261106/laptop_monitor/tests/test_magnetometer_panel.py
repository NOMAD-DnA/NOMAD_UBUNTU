"""Independent magnetic compass checks using explicitly synthetic fixtures."""
import copy
import math
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtWidgets
from magnetometer_panel import FORWARD_AXIS, compass_state
from developer_localization import LocalizationWindow


def packet(bearing=0., strength=45.):
    angle = math.radians(bearing)
    return dict(received=True,
                mag=dict(stamp=100., age_s=.1, received_age_s=.05, frame_id="zed_imu_link",
                         x_ut=strength*math.cos(angle), y_ut=strength*math.sin(angle), z_ut=-10.),
                imu=dict(stamp=100.02, age_s=.08, received_age_s=.04, frame_id="zed_imu_link",
                         ax=0., ay=0., az=9.81))


class CompassMathTest(unittest.TestCase):
    def test_cardinal_bearings_are_clockwise_from_magnetic_north(self):
        self.assertEqual(FORWARD_AXIS, (1., 0., 0.))
        for bearing in (0., 90., 180., 270., 359.):
            with self.subTest(bearing=bearing):
                state = compass_state(packet(bearing), .1)
                self.assertTrue(state["calculable"])
                self.assertAlmostEqual(state["bearing_deg"], bearing)
                self.assertIn("미보정", state["calibration"])

    def test_tilt_compensation_uses_acceleration_and_removes_vertical_field(self):
        # Physical pitch about Y: rotate world N/+X and U/+Z into sensor frame.
        angle = math.radians(35.)
        event = packet()
        event["imu"].update(ax=-9.81*math.sin(angle), az=9.81*math.cos(angle))
        event["mag"].update(x_ut=45.*math.cos(angle)+10.*math.sin(angle),
                            z_ut=45.*math.sin(angle)-10.*math.cos(angle))
        state = compass_state(event, .1)
        self.assertTrue(state["calculable"])
        self.assertAlmostEqual(state["bearing_deg"], 0.)

    def test_stale_relay_keeps_field_but_hides_bearing(self):
        for section, key, value, elapsed, reason in (
            ("mag", "age_s", 3., 0., "오래됨"),
            ("imu", "received_age_s", 3., 0., "오래됨"),
            ("mag", "age_s", .1, 2., "오래됨"),
            ("imu", "age_s", None, 0., "미확인"),
        ):
            event = packet()
            event[section][key] = value
            with self.subTest(section=section, key=key, value=value):
                state = compass_state(event, elapsed)
                self.assertFalse(state["calculable"])
                self.assertIsNone(state["bearing_deg"])
                self.assertGreater(state["field_norm_ut"], 0.)
                self.assertIn(reason, state["reason"])

    def test_fresh_future_timestamp_warns_but_keeps_independent_test_angle(self):
        event = packet(90.)
        event["mag"]["age_s"] = -.345
        event["imu"]["age_s"] = -.365
        state = compass_state(event, .1)
        self.assertTrue(state["calculable"])
        self.assertAlmostEqual(state["bearing_deg"], 90.)
        self.assertTrue(any("시간 검사 불통과" in warning
                            for warning in state["warnings"]))

    def test_bad_pairs_and_degenerate_geometry_never_emit_an_angle(self):
        examples = []
        event = packet(); event["imu"]["frame_id"] = "other"; examples.append(event)
        event = packet(); event["imu"]["stamp"] += .5; examples.append(event)
        event = packet(); event["mag"]["x_ut"] = float("nan"); examples.append(event)
        event = packet(); event["mag"].update(x_ut=0., y_ut=0., z_ut=0.); examples.append(event)
        event = packet(); event["mag"].update(x_ut=0., y_ut=0., z_ut=40.); examples.append(event)
        event = packet(); event["imu"].update(ax=0., ay=0., az=0.); examples.append(event)
        event = packet(); event["imu"].update(ax=9.81, ay=0., az=0.); examples.append(event)
        for event in examples:
            with self.subTest(event=event):
                self.assertIsNone(compass_state(event, 0.)["bearing_deg"])

    def test_small_field_warns_without_claiming_accuracy(self):
        event = packet(90., 2.)
        event["mag"]["z_ut"] = 0.
        original = copy.deepcopy(event)
        state = compass_state(event, 0.)
        self.assertTrue(state["calculable"])
        self.assertIn("작음", state["warnings"][0])
        self.assertIn("미확인", state["calibration"])
        self.assertEqual(event, original)


class CompassPanelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_panel_render_and_localization_integration_are_passive(self):
        window = LocalizationWindow()
        self.addCleanup(window.close)
        window.show()
        window.update_snapshot(dict(now=10., telemetry={"magnetometer": packet(90.)},
                                    received={"telemetry:magnetometer": 10.}))
        panel = window.magnetometer
        panel._refresh()
        self.app.processEvents()
        self.assertIn("090.0°", panel.bearing.text())
        self.assertIn("µT", panel.raw.text())
        self.assertIn("미확인", panel.detail.text())
        self.assertEqual(panel._timer.interval(), 500)
        self.assertFalse(panel.grab().isNull())
        window.hide()
        self.assertFalse(panel._timer.isActive())


if __name__ == "__main__":
    unittest.main()
