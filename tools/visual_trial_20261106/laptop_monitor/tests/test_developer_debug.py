"""Developer-window debugging aids. All packets below are local test fixtures."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_developer_debug_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import developer_debug as dbg
import monitor
from module_logs import LogHistory, ModuleLogPanel


COMPONENTS = {"ImagePanel": monitor.ImagePanel, "MapPanel": monitor.MapPanel,
              "grid_geometry": monitor.grid_geometry, "rotate_vector": monitor.rotate_vector}
GRID = {"resolution": .1, "width": 40, "height": 20, "frame_id": "odom", "image_y": "high_to_low",
        "origin": {"x": -2., "y": -1., "z": 0., "qx": 0., "qy": 0., "qz": 0., "qw": 1.}}


def heat(value):
    """The board probe's heat colour as RGB (it writes BGR (0, G, R))."""
    value = min(1., max(0., value))
    return (int(255 * min(1., 2 * value)), int(255 * min(1., 2 * (1 - value))), 0)


def costmap(values):
    """values: rows (high Y first) of cost 0..100 or None (unknown)."""
    height, width = len(values), len(values[0])
    image = QtGui.QImage(width, height, QtGui.QImage.Format_RGB888)
    for y, row in enumerate(values):
        for x, value in enumerate(row):
            image.setPixelColor(x, y, QtGui.QColor(100, 100, 100) if value is None else QtGui.QColor(*heat(value / 100)))
    return image


def pose(x=0., y=0., t=None):
    return {"t": time.time() if t is None else t, "x": x, "y": y, "z": 0., "qx": 0., "qy": 0., "qz": 0., "qw": 1.,
            "frame_id": "odom", "child_frame_id": "base"}


def planning_report(**changes):
    wall = time.time()
    report = {"kind": "telemetry", "source": "planning", "t": wall, "frame": "odom", "state": "WAIT",
              "reason": "fixture reason", "plan_only": False, "armed": True, "actuator_enabled": False,
              "output_mode": "rpm", "output_inhibit": "LPP_NOT_EXECUTABLE", "feedback_source": "vesc",
              "path": [[0., 0., 0.], [3., 4., 0.]], "trajectory": [[0., 0., 0.], [1., 0., 0.]],
              "gpp_state": "READY", "gpp_reason": "fixture gpp", "lpp_state": "BLOCKED",
              "lpp_reason": "fixture: all candidates hit cost >= 80", "path_valid": True, "executable": False,
              "stamp_ns": int(wall * 1e9), "gpp_stamp_ns": int((wall - 1.) * 1e9), "output_stamp_ns": 0,
              "goal": [3., 4., 0.]}
    report.update(changes)
    return report


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.windows = []
        self.state = tempfile.TemporaryDirectory()
        self.env = patch.dict(os.environ, {"XDG_STATE_HOME": self.state.name})
        self.env.start()

    def tearDown(self):
        for window in self.windows:
            window.close()
        self.app.processEvents()
        self.env.stop()
        self.state.cleanup()

    def make(self, cls, parent=None):
        window = cls(parent=parent, components=COMPONENTS)
        self.windows.append(window)
        return window

    @staticmethod
    def checks(window):
        return {row["item"]: (row["level"], row["detail"]) for row in window.debug.report["checks"]}


class HelperTest(Base):
    def test_stream_stats_counts_dropouts_and_resets_when_replay_moves_back(self):
        stats = dbg.StreamStats([{"key": "image:costmap", "label": "map", "limit": 1., "rate": ("jetson", "costmap")}])
        for now, received in ((10., 9.9), (10.5, 10.4), (11.5, 10.4), (12., 11.9), (13.5, 11.9)):
            stats.observe({"now": now, "received": {"image:costmap": received}, "rates": {"jetson": {"costmap": 2.}}})
        row = stats.rows()[0]
        self.assertEqual(row["dropouts"], 2)
        self.assertAlmostEqual(row["age_stats"][2], 1.6)
        self.assertEqual(row["rate_stats"], (2., 2., 2.))
        self.assertAlmostEqual(row["fresh_ratio"], .6)
        stats.observe({"now": 5., "received": {"image:costmap": 4.9}})
        self.assertEqual(len(stats.samples["image:costmap"]), 1)
        self.assertEqual(stats.rows()[0]["dropouts"], 0)

    def test_heat_inverse_matches_probe_palette_and_gray_is_unknown(self):
        pixels = dbg.image_rgb(costmap([[0, 1, 49, 50, 79, 80, 99, 100, None]]))
        values = dbg.heat_values(pixels)[0]
        self.assertTrue(np.isnan(values[-1]))
        np.testing.assert_allclose(values[:-1] * 100, [0, 1, 49, 50, 79, 80, 99, 100], atol=.6)
        from developer_traversability import COST_BUCKETS
        total, rows = dbg.bucket_counts(dbg.heat_values(pixels), COST_BUCKETS)
        self.assertEqual(total, 9)
        self.assertEqual([count for _, count in rows], [1, 1, 2, 2, 2, 1])

    def test_freshness_and_status_rows_keep_exact_reason(self):
        self.assertEqual(dbg.freshness("x", None, 1., 3.)[1:], ("fail", "미수신"))
        self.assertEqual(dbg.freshness("x", .5, 1., 3.)[1], "ok")
        self.assertEqual(dbg.freshness("x", 2., 1., 3.)[1], "warn")
        self.assertEqual(dbg.freshness("x", 4., 1., 3.)[1], "fail")
        row = dbg.status_check("s", {"state": "WAITING", "detail": "semantic: input late 0.8s"}, .1)
        self.assertEqual(row, ("s", "warn", "WAITING · semantic: input late 0.8s"))
        self.assertEqual(dbg.rate_check("r", 0.)[1], "fail")

    def test_log_panel_counts_problems_and_filters_without_dropping_rows(self):
        panel = ModuleLogPanel()
        panel.append_event({"board": "jetson", "source": "perception", "seq": 1,
                            "text": "[INFO] ok\n[WARN] slow frame\n[ERROR] camera timeout"})
        summary = panel.problem_summary()
        self.assertEqual((summary["errors"], summary["warnings"]), (1, 1))
        self.assertIn("camera timeout", summary["last"])
        panel.problems_only.setChecked(True)
        self.assertNotIn("[INFO] ok", panel.text.toPlainText())
        self.assertIn("camera timeout", panel.text.toPlainText())
        panel.problems_only.setChecked(False)
        self.assertIn("[INFO] ok", panel.text.toPlainText())
        panel.clear()
        self.assertEqual(panel.problem_summary()["errors"], 0)


class ToolbarTest(Base):
    def test_freeze_copy_save_and_clear_are_local_only(self):
        from developer_perception import PerceptionWindow
        window = self.make(PerceptionWindow)
        now = time.monotonic()
        snapshot = {"now": now, "received": {"status:perception": now}, "statuses": {
            "perception": {"state": "READY", "detail": "fixture ready", "t": time.time()}}}
        window.update_snapshot(snapshot)
        self.assertIn("READY", window.status.text())
        window.debug.freeze_button.setChecked(True)
        frozen = {**snapshot, "now": now + 1, "statuses": {"perception": {"state": "FAULT", "detail": "x", "t": 1.}}}
        window.update_snapshot(frozen)
        self.assertIn("READY", window.status.text())          # display frozen
        self.assertEqual(len(window.debug.stats.samples["status:perception"]), 2)  # history continues
        self.assertIn("화면 멈춤", window.debug.strip.text())
        text = window.debug.copy_report()
        self.assertEqual(QtWidgets.QApplication.clipboard().text(), text)
        self.assertIn("인지 모듈 상태", text)
        path = window.debug.save_report()
        self.assertTrue(str(path).startswith(os.path.join(self.state.name, "nomad-trial", "developer_snapshots")))
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self.assertEqual(data["schema"], "nomad_developer_debug_v1")
        self.assertEqual(data["module"], "perception")
        self.assertTrue(data["frozen"])
        window.debug.freeze_button.setChecked(False)          # resume redraws the latest snapshot
        self.assertIn("FAULT", window.status.text())
        window.debug.clear_history()
        self.assertEqual(len(window.debug.stats.samples["status:perception"]), 0)

    def test_log_toggle_hides_only_the_existing_dock(self):
        from developer_traversability import TraversabilityWindow
        window = self.make(TraversabilityWindow)
        window.show()
        window.debug.toggle_log()
        self.assertFalse(window.log_dock.isVisible())
        self.assertEqual(window.debug.log_button.text(), "로그 보기")
        window.debug.toggle_log()
        self.assertTrue(window.log_dock.isVisible())


class WindowContentTest(Base):
    def test_traversability_counts_cells_and_checks_vehicle_inside_map(self):
        from developer_traversability import TraversabilityWindow
        window = self.make(TraversabilityWindow)
        rows = [[None if x < 10 else 100 if x >= 30 else 0 for x in range(40)] for _ in range(20)]
        image = costmap(rows)
        now = time.monotonic()
        event = {"t": time.time(), "width": 40, "height": 20, "georeferenced": True, "grid": GRID}
        snapshot = {"now": now, "received": {"image:costmap": now, "pose:public": now},
                    "images": {"costmap": image}, "image_meta": {"costmap": event},
                    "histories": {"public": [pose(0., 0.)]}, "rates": {"jetson": {"costmap": 2.}}}
        window.update_snapshot(snapshot)
        checks = self.checks(window)
        self.assertEqual(checks["코스트맵 좌표 정보"][0], "ok")
        self.assertEqual(checks["차량이 지도 안에 있음"][0], "warn")   # 1 m from the lower edge
        self.assertEqual(checks["경사도 지도 수신"][0], "fail")
        counts = dict((r[0], r[2]) for r in window.debug.report["sections"]["코스트맵 셀 분포 · 표시 영상 기준"]["rows"])
        self.assertEqual(counts["미확정 (회색)"], "200")
        self.assertEqual(counts["비용 0 · 자유"], "400")
        self.assertEqual(counts["비용 100 · 치명"], "200")
        window.show_pose.setChecked(False)
        window.update_snapshot({**snapshot, "now": now + .2})
        self.assertIn("위치 표시 끔", window.panels["costmap"].meta.text())

    def test_planning_checks_reasons_transitions_and_frozen_buttons_stay_live(self):
        from developer_planning import PlanningWindow
        window = self.make(PlanningWindow)
        now = time.monotonic()
        ui = {"spectator": False, "no_ssh": False, "client_active": True, "disarm_enabled": False, "host": "user@192.0.2.1"}
        snapshot = {"now": now, "received": {"telemetry:planning": now}, "telemetry": {"planning": planning_report()},
                    "planning_ui": ui}
        window.update_snapshot(snapshot)
        checks = self.checks(window)
        self.assertEqual(checks["LPP 실행 후보"], ("fail", "BLOCKED · fixture: all candidates hit cost >= 80"))
        self.assertEqual(checks["출력 차단 사유"], ("warn", "LPP_NOT_EXECUTABLE"))
        values = dict(window.debug.report["sections"]["플래너 주요 값"]["rows"])
        self.assertEqual(values["GPP 경로 길이 · 점 수"], "5.00 m · 2점 (표시용 최대 600점)")
        window.update_snapshot({**snapshot, "now": now + .3, "telemetry": {"planning": planning_report(
            lpp_state="READY", executable=True, lpp_reason="ok", gpp_stamp_ns=int(time.time() * 1e9))},
            "received": {"telemetry:planning": now + .3}})
        changes = window.debug.report["sections"]["상태 변화 기록 · 최근 60건"]["rows"]
        self.assertTrue(any(row[1] == "LPP" and row[2] == "BLOCKED" and row[3] == "READY" for row in changes))
        stats = dict(window.debug.report["sections"]["최근 60초 통계"]["rows"])
        self.assertEqual(stats["GPP 재계획 (새 경로 기준 시각)"], "1회")
        window.debug.freeze_button.setChecked(True)
        self.assertFalse(window.disarm_button.isEnabled())
        window.update_snapshot({**snapshot, "now": now + .6, "planning_ui": {**ui, "disarm_enabled": True}})
        self.assertTrue(window.disarm_button.isEnabled())      # stop button follows live state while frozen
        window.map.set_layer_visible("global", False)
        self.assertFalse(window.map._global_line.get_visible())
        self.assertEqual(window.findChildren(QtCore.QProcess), [])
        self.assertEqual(window.findChildren(QtCore.QThread), [])

    def test_control_reset_button_is_gated_and_reuses_existing_action(self):
        from developer_control import ControlWindow
        parent = QtWidgets.QWidget()
        parent.reset_controller = Mock()
        self.windows.append(parent)
        window = self.make(ControlWindow, parent=parent)
        live = {"now": time.monotonic(), "planning_ui": {"spectator": False, "no_ssh": False, "reset_enabled": True}}
        for blocked in ({**live, "replay": True}, {**live, "planning_ui": {**live["planning_ui"], "spectator": True}},
                        {**live, "planning_ui": {**live["planning_ui"], "reset_enabled": False}}):
            window.update_snapshot(blocked)
            self.assertFalse(window.reset_button.isEnabled())
            window._request_reset()
        parent.reset_controller.assert_not_called()
        window.update_snapshot(live)
        self.assertTrue(window.reset_button.isEnabled())
        window.reset_button.click()
        parent.reset_controller.assert_called_once_with()

    def test_control_trace_statistics_and_reset_required_check(self):
        from developer_control import ControlWindow
        window = self.make(ControlWindow)
        stamp = time.time()
        history = LogHistory()
        for index, (target, measured) in enumerate(((1000., 900.), (1200., 1150.))):
            history.append({"kind": "log", "board": "pi4", "source": "planning", "seq": index + 1, "session": "f",
                            "text": f"[controller] [INFO] [{stamp - 1 + index * .5:.3f}] control_trace requested_erpm=1300 "
                                    f"target_erpm={target} output_erpm={target} measured_erpm={measured} current_a=2.5 "
                                    f"phase={'RUN' if index == 0 else 'FAULT'} source=LPP reset_required={index} start_gate="})
        with patch("module_logs.time.monotonic", return_value=100.):
            window.update_snapshot({"now": time.monotonic(), "wall_now": stamp, "module_logs": {"planning": history}})
        checks = self.checks(window)
        self.assertEqual(checks["재설정 필요 · reset_required"][0], "fail")
        rows = {row[0]: row for row in window.debug.report["sections"]["제어 값 통계 · 최근 60초"]["rows"]}
        self.assertEqual(rows["목표 ERPM"][2], "1000 / 1100 / 1200")
        self.assertEqual(rows["추종 오차 (목표 − 측정)"][2], "50 / 75 / 100")
        changes = window.debug.report["sections"]["제어 상태 변화 기록 · 최근 60건"]["rows"]
        self.assertTrue(any(row[1] == "phase" and row[2] == "RUN" and row[3] == "FAULT" for row in changes))

    def test_localization_replay_keeps_actions_disabled_and_counts_jumps(self):
        from developer_localization import LocalizationWindow
        window = self.make(LocalizationWindow)
        t = time.time()
        history = [pose(0., 0., t), pose(.1, 0., t + .1), pose(5., 0., t + .2)]
        now = time.monotonic()
        snapshot = {"now": now, "replay": True, "received": {"pose:public": now},
                    "histories": {"public": history}, "telemetry": {"fusion": {"mode": "VIO_ONLY", "cov_xx": .01}},
                    "heading_ui": {"enabled": True}, "vio_ui": {"enabled": True}}
        window.update_snapshot(snapshot)
        self.assertFalse(window.heading_reset.isEnabled())
        self.assertFalse(window.vio_restart.isEnabled())
        window.debug.freeze_button.setChecked(True)
        window.update_snapshot({**snapshot, "now": now + .2})
        self.assertFalse(window.vio_restart.isEnabled())
        window.debug.freeze_button.setChecked(False)
        rows = {row[0]: row for row in window.debug.report["sections"]["위치 소스별 통계 · 최근 60초 (보드 시각 기준)"]["rows"]}
        self.assertEqual(rows["융합 위치"][6], "1")
        fields = dict(window.debug.report["sections"]["융합 진단 원본 값 · 받은 필드 전체"]["rows"])
        self.assertEqual(fields["cov_xx"], "0.01")

    def test_resources_filters_and_threshold_checks(self):
        from developer_resources import ResourcesWindow
        window = self.make(ResourcesWindow)
        now = time.monotonic()
        process = lambda pid, name, group: {"pid": pid, "ppid": 1, "start_ticks": 1, "threads": 2, "name": name,
                                            "group": group, "cpu_percent_one_core": 10., "rss_mib": 5.}
        event = {"source": "resources_jetson", "board": "jetson", "kind": "telemetry", "schema": 1, "t": time.time(),
                 "sample_monotonic_s": 1., "session": "s", "boot_id": "b", "model": "fixture", "kernel": "k", "seq": 1,
                 "cpu": {"state": "live", "online_cores": 1, "total_percent": 97., "interval_s": 1.,
                         "cores": [{"id": "cpu0", "percent": 97., "frequency_mhz": None}]},
                 "memory": {"total_mib": 1000., "available_mib": 40., "used_mib": 960., "swap_total_mib": 0., "swap_used_mib": 0.},
                 "gpu": {"load_percent": None, "frequency_mhz": None}, "temperatures": [{"name": "t", "celsius": 50.}],
                 "processes": [process(1, "mapper2", "mapping"), process(2, "Xorg", "other")],
                 "processes_total": 2, "processes_omitted": 0, "proc_read_failures": 0, "scan_ms": 1.}
        snapshot = {"now": now, "received": {"telemetry:resources_jetson": now},
                    "telemetry": {"resources_jetson": event}, "resource_history": {"jetson": [], "pi4": []}}
        window.update_snapshot(snapshot)
        checks = self.checks(window)
        self.assertEqual(checks["젯슨 CPU"][0], "fail")
        self.assertEqual(checks["젯슨 남은 메모리"][0], "fail")
        self.assertEqual(window.table.rowCount(), 2)
        window.nomad_only.setChecked(True)
        self.assertEqual(window.table.rowCount(), 1)
        window.search.setText("없는이름")
        self.assertEqual(window.table.rowCount(), 0)


if __name__ == "__main__":
    unittest.main()
