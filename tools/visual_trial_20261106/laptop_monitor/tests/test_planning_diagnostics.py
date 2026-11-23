"""Candidate failure transitions are independent of actuator permission."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning_diagnostics import LppDiagnostics


def packet(**changes):
    return dict(t=1794203130., state="STOP", goal=[1., 2., 0.], path=[[0., 0., 0.]],
                gpp_state="READY", path_valid=True, lpp_state="READY", executable=False,
                lpp_reason="no collision-free RPP motion; recovery disabled", stamp_ns=10,
                **changes)


class LppDiagnosticsTest(unittest.TestCase):
    def test_failure_before_first_motion_and_duplicates_and_recovery(self):
        recorder = LppDiagnostics()
        event = packet()
        first = recorder.update(event)
        self.assertIn(event["lpp_reason"], first["text"])
        self.assertIn("GPP=READY", first["text"])
        for index in range(200):
            self.assertIsNone(recorder.update({**event, "t": event["t"] + index, "stamp_ns": index}))
        self.assertIsNone(recorder.update({**event, "executable": True}))
        self.assertEqual(recorder.update(event)["seq"], 2)

    def test_all_reported_reasons_remain_distinct(self):
        recorder = LppDiagnostics()
        for reason in ("observed-space safety stop", "observation boundary",
                       "TRAJECTORY_FOOTPRINT_OUTSIDE_MAP", "ODOMETRY_STALE"):
            result = recorder.update({**packet(), "lpp_reason": reason})
            self.assertIn(reason, result["text"])

    def test_idle_arrival_disconnect_and_output_inhibit_are_not_candidate_failure(self):
        recorder = LppDiagnostics()
        for event in ({**packet(), "goal": None, "path": []},
                      {**packet(), "lpp_state": "ARRIVED"},
                      {**packet(), "state": "DISCONNECTED"},
                      {**packet(), "executable": True, "armed": False, "output_inhibit": "DISARMED"}):
            self.assertIsNone(recorder.update(event))

    def test_missing_legacy_flag_unknown_reason_and_goal_without_path(self):
        recorder = LppDiagnostics()
        event = packet()
        del event["executable"]
        self.assertIsNone(recorder.update(event))
        event.update(executable=False, path=[], lpp_reason="", reason="ODOMETRY_STALE")
        self.assertIn("ODOMETRY_STALE", recorder.update(event)["text"])
        event.update(lpp_reason="", reason="")
        self.assertIn("상세 사유 미제공", recorder.update(event)["text"])

    def test_new_goal_logs_same_reason_and_reason_stays_one_line(self):
        recorder = LppDiagnostics()
        event = {**packet(), "lpp_reason": "test\nreason"}
        first = recorder.update(event)
        self.assertNotIn("\n", first["text"])
        self.assertIn("test\\nreason", first["text"])
        self.assertIsNotNone(recorder.update({**event, "goal": [2., 3., 0.]}))


if __name__ == "__main__":
    unittest.main()
