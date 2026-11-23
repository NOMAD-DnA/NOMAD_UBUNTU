"""Pi5 forwarder 64-key cut warning; synthetic packets, no board access."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets

from localization_state import (FORWARD_TRUNCATED_TEXT, HEADING_UNKNOWN_TEXT, forward_truncated,
                                localization_summary)
from developer_localization import LocalizationWindow


def snapshot(mode="FUSED", padding=60, **extra):
    values = dict(mode=mode, source="VIO+GNSS", position_valid="true", orientation_valid="true")
    values.update({f"k{i:02d}": "0" for i in range(padding)})
    values.update(extra)
    return dict(now=100., received={"telemetry:fusion": 99.9, "status:localization": 99.9},
                telemetry={"fusion": dict(received=True, measurement_age_s=.05, values=values)},
                statuses={"localization": dict(state="READY", detail="")})


class ForwardTruncatedTests(unittest.TestCase):
    def test_detection(self):
        self.assertTrue(forward_truncated(snapshot()["telemetry"]["fusion"]))
        self.assertFalse(forward_truncated(snapshot(padding=10)["telemetry"]["fusion"]))
        self.assertFalse(forward_truncated(snapshot(viewer_frame="map")["telemetry"]["fusion"]))
        listed = [dict(key=f"k{i}", value="0") for i in range(64)]
        self.assertTrue(forward_truncated(dict(values=listed)))
        self.assertFalse(forward_truncated({}))
        self.assertFalse(forward_truncated(None))

    def test_summary_detail_warns_only_when_cut(self):
        self.assertIn(FORWARD_TRUNCATED_TEXT, localization_summary(snapshot())["detail"])
        self.assertNotIn(FORWARD_TRUNCATED_TEXT, localization_summary(snapshot(padding=10))["detail"])

    def test_heading_reset_unknown_instead_of_waiting(self):
        state = localization_summary(snapshot("VIO_ONLY"))
        self.assertFalse(state["can_reset_heading"])
        self.assertEqual(state["reset_reason"], HEADING_UNKNOWN_TEXT)
        # Short packet without the key keeps the previous wording.
        self.assertNotIn("전달 안 됨", localization_summary(snapshot("VIO_ONLY", padding=10))["reset_reason"])
        # An explicit key still wins even in a full packet.
        state = localization_summary(snapshot("VIO_ONLY", heading_reset_available="true"))
        self.assertTrue(state["can_reset_heading"])


class ChecklistTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def checks(self, snap):
        window = LocalizationWindow()
        self.addCleanup(window.close)
        return {row[0]: row for row in window._debug_checks(snap)}

    def test_checklist_warns_when_cut(self):
        rows = self.checks(snapshot())
        self.assertEqual(rows["GNSS 비교 좌표"][1:], ("warn", FORWARD_TRUNCATED_TEXT))
        self.assertIn("알 수 없음(전달 안 됨)", rows["방향 초기화 가능"][2])
        rows = self.checks(snapshot("VIO_ONLY"))
        self.assertEqual(rows["방향 초기화 가능"][2].count("전달 안 됨"), 1)

    def test_checklist_unchanged_when_not_cut(self):
        rows = self.checks(snapshot(padding=10))
        self.assertNotIn("GNSS 비교 좌표", rows)
        self.assertNotIn("전달 안 됨", rows["방향 초기화 가능"][2])


if __name__ == "__main__":
    unittest.main()
