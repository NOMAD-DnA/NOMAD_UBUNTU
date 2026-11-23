"""Offline UI/receiver contract checks. All packets below are test fixtures."""
import base64
import os
from pathlib import Path
import sys
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_developer_ui_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtCore, QtGui, QtWidgets
import monitor


class DeveloperWindowsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        argv = sys.argv
        try:
            sys.argv = ["monitor", "--no-ssh"]
            self.window = monitor.MonitorWindow(monitor.parse_args())
        finally:
            sys.argv = argv

    def tearDown(self):
        self.window.close()
        self.app.processEvents()

    def test_windows_share_receivers_and_reopen_without_duplication(self):
        for part in ("localization", "perception", "traversability"):
            self.window.open_developer_window(part)
            child = self.window.developer_windows[part]
            self.assertTrue(child.isVisible())
            child.close()
            self.window.open_developer_window(part)
            self.assertIs(child, self.window.developer_windows[part])
        self.assertEqual(self.window.workers, [])
        self.assertIs(self.window.developer_snapshot()["histories"], self.window.histories)

    def test_actual_tracking_packet_routes_without_main_panel(self):
        image = QtGui.QImage(20, 16, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("#405060"))
        buffer = QtCore.QBuffer()
        buffer.open(QtCore.QIODevice.WriteOnly)
        self.assertTrue(image.save(buffer, "PNG"))
        event = {"kind": "image", "source": "vio_tracking", "t": time.time(),
                 "encoding": "png", "width": 20, "height": 16,
                 "data": base64.b64encode(bytes(buffer.data())).decode("ascii")}
        self.window.on_packet("pi", event)
        self.window.refresh()
        self.assertEqual(self.window.latest_qimages["vio_tracking"].width(), 20)
        self.window.on_packet("pi", {**event, "data": "corrupt"})
        self.window.refresh()
        self.assertTrue(self.window.images["vio_tracking"]["decode_error"])
        self.assertEqual(self.window.latest_qimages["vio_tracking"].width(), 20)

    def test_main_maps_only_but_camera_cache_reaches_developer_window(self):
        self.assertEqual(set(self.window.panels), {"costmap", "slope"})
        image = QtGui.QImage(20, 16, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("#405060"))
        buffer = QtCore.QBuffer()
        buffer.open(QtCore.QIODevice.WriteOnly)
        self.assertTrue(image.save(buffer, "PNG"))
        for source in ("rgb", "semantic"):
            self.window.on_packet("jetson", {
                "kind": "image", "source": source, "t": time.time(),
                "encoding": "png", "width": 20, "height": 16,
                "data": base64.b64encode(bytes(buffer.data())).decode("ascii")})
        self.window.refresh()
        self.window.open_developer_window("perception")
        child = self.window.developer_windows["perception"]
        for source in ("rgb", "semantic"):
            self.assertEqual(self.window.latest_qimages[source].width(), 20)
            self.assertEqual(child.panels[source].pixmap.width(), 20)
        self.assertEqual(self.window.workers, [])

    def test_missing_stale_and_rejected_telemetry_preserve_real_values(self):
        self.assertEqual(self.window.telemetry, {})
        packet = {"kind": "telemetry", "source": "gnss", "t": time.time(),
                  "fix_valid": False, "satellites": None, "fix_age_s": 12.0}
        self.window.on_packet("pi", packet)
        self.assertIsNone(self.window.telemetry["gnss"]["satellites"])
        self.assertFalse(self.window.telemetry["gnss"]["fix_valid"])
        self.window.on_packet("pi", {**packet, "h_acc_m": float("nan")})
        self.assertEqual(self.window.invalid_events, 1)
        self.window.received["telemetry:gnss"] -= 10
        self.assertGreater(self.window.age("telemetry", "gnss"), 9)
        for part in ("localization", "perception", "traversability"):
            self.window.open_developer_window(part)
        self.window.refresh()
        self.window.close()
        self.assertTrue(all(not w.isVisible() for w in self.window.developer_windows.values()))

    def test_collector_schema_does_not_make_old_fix_fresh(self):
        self.window.open_developer_window("localization")
        child = self.window.developer_windows["localization"]
        self.window.on_packet("pi", {"kind": "telemetry", "source": "gnss",
                                      "t": time.time(), "received": False})
        self.window.refresh()
        self.assertIn("미수신", child.gnss.receipt.text())
        self.window.on_packet("pi", {"kind": "telemetry", "source": "gnss", "t": time.time(),
                                      "received": True, "fix_received": True, "quality_received": True,
                                      "fix_valid": True, "fix_status": 0, "satellites": 13,
                                      "fix_age_s": 12., "fix_received_age_s": 12.,
                                      "quality_age_s": 0., "quality_received_age_s": 0.,
                                      "fix_hz": 5., "quality_hz": 5., "h_acc_m": 0.4})
        self.window.on_packet("pi", {"kind": "telemetry", "source": "fusion", "t": time.time(),
                                      "received": True, "gnss_accepted": 7, "solves": 20,
                                      "optimization_state": "not_reported", "values": {"source": "VIO"}})
        self.window.refresh()
        self.assertIn("오래됨", child.gnss.values["fix"].text())
        self.assertIn("13", child.gnss.values["satellites"].text())
        self.assertIn("누적 7건", child.fusion.values["accepted"].text())
        self.assertIn("20", child.fusion.values["optimization"].text())
        # A last packet with age zero must grow old when callbacks stop.
        self.window.telemetry["gnss"]["fix_age_s"] = 0.
        self.window.telemetry["gnss"]["fix_received_age_s"] = 0.
        self.window.received["telemetry:gnss"] -= 10.
        self.window.refresh()
        self.assertIn("오래됨", child.gnss.values["fix"].text())


if __name__ == "__main__":
    unittest.main()
