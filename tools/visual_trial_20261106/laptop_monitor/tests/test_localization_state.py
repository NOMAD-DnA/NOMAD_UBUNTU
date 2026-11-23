"""Pure packet contracts; synthetic values, no board or network access."""
from pathlib import Path
import json
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from localization_state import comparison_samples, localization_summary


def snapshot(mode="VIO_ONLY", age=.1, measurement_age=.05, **values):
    fields = dict(mode=mode, source="VIO" if mode == "VIO_ONLY" else "VIO+GNSS",
                  position_valid="true", orientation_valid="true", **values)
    return dict(now=100., received={"telemetry:fusion": 100.-age, "status:localization": 99.9},
                telemetry={"fusion": dict(received=True, measurement_age_s=measurement_age, values=fields)},
                statuses={"localization": dict(state="READY", detail="")})


class SummaryTests(unittest.TestCase):
    def test_real_modes_and_source_labels(self):
        for mode in ("VIO_ONLY", "FUSED", "GNSS_HEADING", "GNSS_ONLY", "NO_POSITION", "WAITING"):
            with self.subTest(mode=mode):
                state = localization_summary(snapshot(mode))
                self.assertEqual(state["mode"], mode)
                self.assertTrue(state["label"])
        state = localization_summary(snapshot("FUSED", solves="0"))
        self.assertIn("별도", state["detail"])
        self.assertIn("VIO·GNSS", state["label"])

    def test_ros_false_orientation_is_not_true(self):
        data = snapshot("GNSS_ONLY")
        data["telemetry"]["fusion"]["values"]["orientation_valid"] = "false"
        state = localization_summary(data)
        self.assertFalse(state["orientation_valid"])
        self.assertEqual(state["state"], "INVALID")
        self.assertIn("방향 미확정", state["orientation_text"])

    def test_old_diagnostics_and_status_are_stale(self):
        data = snapshot(age=3.)
        data["received"]["status:localization"] = 97.
        state = localization_summary(data)
        self.assertEqual(state["state"], "STALE")
        self.assertEqual(state["mode"], "STALE")
        self.assertFalse(state["can_reset_heading"])

    def test_measurement_age_counts_even_for_new_relay_packet(self):
        data = snapshot(age=.1, measurement_age=3., heading_reset_available="true")
        data["statuses"] = {}
        state = localization_summary(data)
        self.assertEqual(state["mode"], "STALE")
        self.assertFalse(state["can_reset_heading"])

    def test_status_fallback_is_display_only(self):
        data = snapshot(age=3.)
        data["statuses"]["localization"]["detail"] = "mode=GNSS_ONLY;position_valid=true;orientation_valid=false"
        state = localization_summary(data)
        self.assertEqual(state["mode"], "GNSS_ONLY")
        self.assertEqual(state["mode_basis"], "status")
        self.assertFalse(state["orientation_valid"])
        self.assertFalse(state["can_reset_heading"])

    def test_empty_and_explicit_unreceived_are_waiting(self):
        self.assertEqual(localization_summary({})["mode"], "WAITING")
        data = snapshot()
        data["telemetry"]["fusion"]["received"] = False
        data["statuses"] = {}
        self.assertEqual(localization_summary(data)["mode"], "WAITING")

    def test_invalid_position_and_public_fault_stay_visible(self):
        data = snapshot()
        data["telemetry"]["fusion"]["values"]["position_valid"] = "false"
        self.assertEqual(localization_summary(data)["state"], "INVALID")
        data = snapshot()
        data["statuses"]["localization"]["state"] = "FAULT"
        self.assertEqual(localization_summary(data)["state"], "INVALID")

    def test_reset_requires_explicit_fresh_vio_only(self):
        self.assertFalse(localization_summary(snapshot())["can_reset_heading"])
        self.assertIn("업데이트", localization_summary(snapshot())["reset_reason"])
        for flag in ("false", False, "yes", 1, "1", None):
            state = localization_summary(snapshot(heading_reset_available=flag))
            self.assertFalse(state["can_reset_heading"])
        for flag in ("true", True):
            self.assertTrue(localization_summary(snapshot(heading_reset_available=flag))["can_reset_heading"])
        self.assertFalse(localization_summary(snapshot("FUSED", heading_reset_available="true"))["can_reset_heading"])
        self.assertFalse(localization_summary(snapshot(age=1.1, heading_reset_available="true"))["can_reset_heading"])

    def test_replay_and_invalid_clock_never_authorize_reset(self):
        data = snapshot(heading_reset_available="true")
        data["replay"] = True
        self.assertFalse(localization_summary(data)["can_reset_heading"])
        for value in (-.1, float("nan"), None):
            data = snapshot(measurement_age=value, heading_reset_available="true")
            self.assertFalse(localization_summary(data)["can_reset_heading"])

    def test_keyvalue_list_and_json_finite(self):
        data = snapshot()
        data["telemetry"]["fusion"]["values"] = [{"key": key, "value": value}
            for key, value in data["telemetry"]["fusion"]["values"].items()]
        state = localization_summary(data)
        self.assertEqual(state["mode"], "VIO_ONLY")
        json.dumps(state, allow_nan=False)


def compare_event(**replace):
    values = dict(viewer_frame="fusion_odom", viewer_epoch="2", viewer_gnss_valid="true",
                  viewer_gnss_t="1794300000.25", viewer_gnss_x="1.5", viewer_gnss_y="-2.0",
                  viewer_gnss_z="0", viewer_vio_valid="true", viewer_vio_t="1794300000.3",
                  viewer_vio_x="1.4", viewer_vio_y="-2.1", viewer_vio_z=".03")
    values.update(replace)
    return dict(values=values)


class ComparisonTests(unittest.TestCase):
    def test_exact_backend_positions_and_common_frame(self):
        samples = comparison_samples(compare_event())
        self.assertEqual([x["source"] for x in samples], ["gnss", "vio_compare"])
        self.assertEqual(samples[0]["x"], 1.5)
        self.assertEqual(samples[0]["y"], -2.)
        self.assertEqual(samples[0]["t"], 1794300000.25)
        self.assertTrue(all(s["frame_id"] == "fusion_odom" and s["viewer_epoch"] == 2 for s in samples))
        self.assertNotIn("qw", samples[0])

    def test_invalid_and_missing_coords_are_not_zero_filled(self):
        for value in (None, "nan", "inf", True, "bad"):
            samples = comparison_samples(compare_event(viewer_gnss_x=value))
            self.assertEqual([x["source"] for x in samples], ["vio_compare"])
        event = compare_event()
        del event["values"]["viewer_gnss_z"]
        self.assertEqual(len(comparison_samples(event)), 1)

    def test_false_flag_and_epoch_validation(self):
        self.assertEqual(len(comparison_samples(compare_event(viewer_gnss_valid="false"))), 1)
        for value in (None, -1, 1.25, "nan", True):
            self.assertEqual(comparison_samples(compare_event(viewer_epoch=value)), [])
        for frame in (None, "", " ", 4):
            self.assertEqual(comparison_samples(compare_event(viewer_frame=frame)), [])
        self.assertEqual(comparison_samples(compare_event(viewer_gnss_t=0, viewer_vio_valid=False)), [])

    def test_no_explicit_viewer_metadata_means_no_comparison(self):
        self.assertEqual(comparison_samples(dict(latitude=37., longitude=127., altitude=30.)), [])
        self.assertEqual(comparison_samples(None), [])


if __name__ == "__main__":
    unittest.main()
