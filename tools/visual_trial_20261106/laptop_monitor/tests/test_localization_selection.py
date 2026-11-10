"""Offline localization controls. Synthetic points never open a board connection."""
import os
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/nomad_localization_selection/mpl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
from developer_localization import LocalizationWindow
import heading_control
import vio_control


CAPABLE = {"mode": "vio", "label": "VIO 단독", "color": "#38bdf8",
           "detail": "VIO 입력 사용", "vio_text": "VIO 추적 중", "gnss_text": "GNSS 대기",
           "orientation_text": "지자계 유효", "can_reset_heading": True, "reset_reason": ""}


def point(x, y=0., z=0., frame="odom"):
    return {"x": x, "y": y, "z": z, "frame_id": frame, "t": 1.}


def fixture():
    return {"now": 100., "histories": {"gnss": [point(2.)], "vio_compare": [point(1.)],
            "fusion": [point(100., frame="fusion_odom")], "public": [point(1.5)], "vio": [point(1000., frame="vio")]},
            "received": {f"pose:{s}": 100. for s in ("gnss", "vio", "vio_compare", "fusion", "public")},
            "heading_ui": {"enabled": True, "busy": False},
            "vio_ui": {"enabled": True, "busy": False, "fusion_continuity": True}}


class LocalizationSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.summary_patch = patch("developer_localization.localization_summary", return_value=CAPABLE)
        self.summary_mock = self.summary_patch.start()
        self.window = LocalizationWindow()

    def tearDown(self):
        self.window.close()
        self.app.processEvents()
        self.summary_patch.stop()

    def test_default_selection_and_legend_use_only_comparable_received_positions(self):
        self.window.update_snapshot(fixture())
        self.assertTrue(all(self.window.source_checks[s].isChecked() for s in ("gnss", "vio_compare", "public")))
        self.assertFalse(self.window.source_checks["vio"].isChecked())
        self.assertFalse(self.window.source_checks["fusion"].isChecked())
        self.assertEqual(self.window._visible_legend_sources, ("gnss", "vio_compare", "public"))
        self.assertFalse(self.window.markers["vio"].get_visible())
        self.assertEqual(self.window.source_checks["public"].text(), "융합 위치")
        self.assertIn("IMU", self.window.source_checks["fusion"].text())
        self.assertNotIn("서로 다른 좌표계", self.window.frame_info.text())
        self.assertLess(self.window.ax.get_xlim()[1], 10.)

    def test_toggle_hides_line_marker_and_legend_and_changes_fit(self):
        self.window.update_snapshot(fixture())
        self.window.source_checks["public"].setChecked(False)
        self.assertFalse(self.window.lines["public"].get_visible())
        self.assertFalse(self.window.markers["public"].get_visible())
        self.assertNotIn("public", self.window._visible_legend_sources)
        self.window.source_checks["fusion"].setChecked(True)
        self.window.fit_view()
        self.assertTrue(self.window.lines["fusion"].get_visible())
        self.assertGreater(self.window.ax.get_xlim()[1], 100.)
        self.assertIn("서로 다른 좌표계", self.window.frame_info.text())
        self.window.source_checks["fusion"].setChecked(False)
        self.window.fit_view()
        self.assertLess(self.window.ax.get_xlim()[1], 10.)

    def test_missing_or_nonfinite_coordinates_are_never_drawn_at_zero(self):
        snap = fixture()
        snap["histories"] = {"gnss": [{"x": float("nan"), "y": 0., "z": 0.}]}
        self.window.update_snapshot(snap)
        self.assertEqual(self.window._visible_legend_sources, ())
        self.assertIsNone(self.window.ax.get_legend())
        self.assertFalse(any(m.get_visible() for m in self.window.markers.values()))
        self.assertEqual(len(self.window.lines["gnss"].get_data_3d()[0]), 0)
        self.assertIn("좌표계 미수신", self.window.frame_info.text())
        self.assertIn("로컬 좌표 대기", self.window.pose_info.text())

    def test_raw_vio_requires_explicit_selection_and_shows_frame_warning(self):
        self.window.update_snapshot(fixture())
        self.assertNotIn("주의:", self.window.frame_info.text())
        self.window.source_checks["vio"].setChecked(True)
        self.assertTrue(self.window.markers["vio"].get_visible())
        self.assertIn("서로 다른 좌표계", self.window.frame_info.text())
        self.assertIn("고유 좌표계", self.window.frame_info.text())

    def test_plot_updates_preserve_artists_and_user_camera(self):
        snap = fixture()
        self.window.update_snapshot(snap)
        axes, canvas = self.window.ax, self.window.canvas
        line = self.window.lines["public"]
        self.window.auto_fit.setChecked(False)
        self.window.ax.view_init(elev=42., azim=19.)
        self.window.ax.set_xlim(-8., 8.)
        snap["histories"]["public"].append(point(4.))
        self.window.update_snapshot(snap)
        self.assertIs(self.window.ax, axes)
        self.assertIs(self.window.canvas, canvas)
        self.assertIs(self.window.lines["public"], line)
        self.assertEqual(self.window.ax.elev, 42.)
        self.assertEqual(self.window.ax.azim, 19.)
        self.assertEqual(self.window.ax.get_xlim(), (-8., 8.))

    def test_heading_button_emits_only_an_explicit_user_request(self):
        requests = []
        self.window.heading_reset_requested.connect(lambda: requests.append(True))
        self.window.update_snapshot(fixture())
        self.assertIn("VIO 단독", self.window.summary.text())
        self.assertIn("VIO 추적 중", self.window.input_states.text())
        self.assertEqual(requests, [])
        self.assertTrue(self.window.heading_reset.isEnabled())
        self.window.heading_reset.click()
        self.assertEqual(requests, [True])
        self.assertFalse(self.window.heading_reset.isEnabled())
        self.assertIn("원본 VIO", self.window.heading_reset_note.text())

    def test_spectator_replay_disconnected_and_busy_disable_heading_action(self):
        requests = []
        self.window.heading_reset_requested.connect(lambda: requests.append(True))
        cases = (({"replay": True}, {}, "재생"), ({}, {"spectator": True}, "관전"),
                 ({}, {"no_ssh": True}, "연결"), ({}, {"busy": True}, "처리 중"),
                 ({}, {"enabled": False, "reason": "Pi5 대기"}, "Pi5 대기"))
        for top, ui, phrase in cases:
            snap = fixture()
            snap.update(top)
            snap["heading_ui"].update(ui)
            self.window.update_snapshot(snap)
            self.assertFalse(self.window.heading_reset.isEnabled())
            self.assertIn(phrase, self.window.heading_reset_note.text())
            self.window._request_heading_reset()
        self.assertEqual(requests, [])

    def test_ineligible_mode_and_request_result_are_visible(self):
        self.summary_mock.return_value = {**CAPABLE, "label": "VIO·GNSS 융합", "can_reset_heading": False,
                                          "reset_reason": "VIO 단독 모드에서만 초기화할 수 있습니다."}
        snap = fixture()
        snap["heading_ui"]["result"] = {"success": False, "message": "지자계가 오래됐습니다."}
        self.window.update_snapshot(snap)
        self.assertFalse(self.window.heading_reset.isEnabled())
        self.assertIn("VIO·GNSS 융합", self.window.summary.text())
        self.assertIn("지자계가 오래됐습니다.", self.window.heading_reset_note.text())
        self.assertIn("VIO 단독 모드", self.window.heading_reset_note.text())

    def test_vio_restart_emits_request_and_disables_both_actions(self):
        requests = []
        self.window.vio_restart_requested.connect(lambda: requests.append(True))
        self.window.update_snapshot(fixture())
        self.assertEqual(requests, [])
        self.assertTrue(self.window.vio_restart.isEnabled())
        self.window.vio_restart.click()
        self.assertEqual(requests, [True])
        self.assertFalse(self.window.vio_restart.isEnabled())
        self.assertFalse(self.window.heading_reset.isEnabled())
        self.assertIn("마지막 유효 위치·방향", self.window.vio_restart_note.text())
        self.assertIn("원본 VIO 궤적은 초기화", self.window.vio_restart_note.text())

    def test_vio_restart_is_disabled_for_readonly_or_overlapping_requests(self):
        requests = []
        self.window.vio_restart_requested.connect(lambda: requests.append(True))
        cases = (({"replay": True}, {}, {}, "재생"),
                 ({}, {"spectator": True}, {}, "관전"),
                 ({}, {"no_ssh": True}, {}, "연결"),
                 ({}, {}, {"busy": True}, "방향 초기화"),
                 ({}, {"busy": True}, {}, "재실행 요청 처리"),
                 ({}, {"enabled": False, "reason": "VIO 경로 대기"}, {}, "VIO 경로 대기"))
        for top, ui, heading, phrase in cases:
            snap = fixture()
            snap.update(top)
            snap["vio_ui"].update(ui)
            snap["heading_ui"].update(heading)
            self.window.update_snapshot(snap)
            self.assertFalse(self.window.vio_restart.isEnabled())
            self.assertIn(phrase, self.window.vio_restart_note.text())
            self.window._request_vio_restart()
        self.assertEqual(requests, [])

    def test_vio_restart_progress_blocks_heading_and_shows_result(self):
        snap = fixture()
        snap["vio_ui"].update(busy=True, result={"success": True, "message": "VIO 재실행 완료"})
        self.window.update_snapshot(snap)
        self.assertFalse(self.window.heading_reset.isEnabled())
        self.assertFalse(self.window.vio_restart.isEnabled())
        self.assertIn("VIO 재실행 완료", self.window.vio_restart_note.text())
        snap["vio_ui"].update(busy=False, fusion_continuity=False)
        self.window.update_snapshot(snap)
        self.assertTrue(self.window.vio_restart.isEnabled())
        self.assertIn("새 원점", self.window.vio_restart_note.text())

    def test_compare_freshness_uses_own_receipt_not_raw_vio(self):
        snap = fixture()
        snap["received"]["pose:vio"] = 90.
        self.window.update_snapshot(snap)
        self.assertEqual(self.window.lines["vio_compare"].get_linestyle(), "-")
        snap["received"]["pose:vio_compare"] = 90.
        self.window.update_snapshot(snap)
        self.assertEqual(self.window.lines["vio_compare"].get_linestyle(), "--")

    def test_primary_and_internal_selections_use_separate_rows(self):
        check = self.window.source_checks["gnss"]
        grid = check.parentWidget().layout().itemAt(1).layout()
        for source in ("gnss", "vio_compare", "public"):
            self.assertEqual(grid.getItemPosition(grid.indexOf(self.window.source_checks[source]))[0], 0)
        for source in ("fusion", "vio"):
            self.assertEqual(grid.getItemPosition(grid.indexOf(self.window.source_checks[source]))[0], 1)

    def test_actual_heading_control_result_uses_ok_and_korean_reason(self):
        remote = {"ok": True, "stdout": heading_control.RESULT_MARKER +
                  json.dumps({"ok": True, "state": "HEADING_RESET", "reason": "ok"})}
        with patch("heading_control.run_remote", return_value=remote):
            result = heading_control.reset_heading("user@192.0.2.5")
        self.assertTrue(result["ok"])
        snap = fixture()
        snap["heading_ui"]["result"] = result
        self.window.update_snapshot(snap)
        self.assertIn("현재 위치 유지 · 방향 초기화 완료", self.window.heading_reset_note.text())
        self.assertNotIn("실패", self.window.heading_reset_note.text())
        self.summary_mock.return_value = {**CAPABLE, "can_reset_heading": False,
                                          "reset_reason": "WAITING_FOR_CONTINUOUS_VIO_ONLY"}
        snap["heading_ui"]["result"] = None
        self.window.update_snapshot(snap)
        self.assertIn("안정적으로 유지", self.window.heading_reset_note.text())
        self.assertNotIn("WAITING_FOR_CONTINUOUS", self.window.heading_reset_note.text())

    def test_actual_vio_control_results_distinguish_continuity_and_new_origin(self):
        for preserved, reason, phrase in ((True, "fusion_preserved", "마지막 유효 위치·방향 유지"),
                                           (False, "new_vio_origin", "새 원점")):
            remote = {"ok": True, "stdout": vio_control.RESULT_MARKER + json.dumps({
                "ok": True, "state": "VIO_RESTARTED", "reason": reason, "fusion_preserved": preserved})}
            with patch("vio_control.run_remote", return_value=remote):
                result = vio_control.restart_vio("user@192.0.2.5")
            self.assertTrue(result["ok"])
            snap = fixture()
            snap["vio_ui"].update(result=result, fusion_continuity=preserved)
            self.window.update_snapshot(snap)
            self.assertIn("VIO 재실행 완료", self.window.vio_restart_note.text())
            self.assertIn(phrase, self.window.vio_restart_note.text())
            self.assertNotIn("실패", self.window.vio_restart_note.text())
            self.assertNotIn(reason, self.window.vio_restart_note.text())

    def test_actual_control_failures_show_translated_reason(self):
        remote = {"ok": False, "stdout": vio_control.RESULT_MARKER + json.dumps({
            "ok": False, "state": "VIO_RESTART_FAILED", "reason": "vio_stop_incomplete"})}
        with patch("vio_control.run_remote", return_value=remote):
            result = vio_control.restart_vio("user@192.0.2.5")
        snap = fixture()
        snap["vio_ui"]["result"] = result
        self.window.update_snapshot(snap)
        self.assertIn("이전 VIO 종료를 확인하지 못했습니다.", self.window.vio_restart_note.text())
        self.assertNotIn("vio_stop_incomplete", self.window.vio_restart_note.text())
        snap["heading_ui"]["result"] = {"ok": False, "state": "HEADING_RESET_FAILED", "reason": "HOLD_VEHICLE_STILL"}
        self.window.update_snapshot(snap)
        self.assertIn("차량을 멈춘 상태로 유지", self.window.heading_reset_note.text())


if __name__ == "__main__":
    unittest.main()
