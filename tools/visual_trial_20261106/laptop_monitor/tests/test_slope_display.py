"""Actual-run PNG scale must survive GUI, relay and recordings without recolouring costs."""
import base64
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("NOMAD_VIEWER_STATE_DIR", "/tmp/nomad_slope_display_test")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtCore, QtGui, QtWidgets
import monitor
import slope_k
def grid(t=100.):
    return dict(t=t, georeferenced=True, grid=dict(frame_id="odom", width=20, height=10,
        resolution=.1, image_y="high_to_low", origin=dict(x=-1., y=-.5, z=0., qx=0., qy=0., qz=0., qw=1.)))


class SlopeDisplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def packet(self, maximum=10., k=8., source="slope"):
        image = QtGui.QImage(20, 10, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("yellow"))
        data = QtCore.QByteArray()
        buffer = QtCore.QBuffer(data)
        buffer.open(QtCore.QIODevice.WriteOnly)
        image.save(buffer, "PNG")
        return dict(grid(), kind="image", source=source, encoding="png", width=20, height=10,
                    data=base64.b64encode(bytes(data)).decode(),
                    slope_display=dict(max_deg=maximum, k=k, mapper="terrain", applied=True))

    def test_legend_and_pixels_follow_packet_not_local_settings(self):
        panel = monitor.MapPanel("fixture", slope=True)
        try:
            image = QtGui.QImage(20, 10, QtGui.QImage.Format_RGB888)
            image.fill(QtGui.QColor("yellow"))
            for maximum, k, middle in ((20, 4, 10), (10, 8, 5), (40, 2, 20)):
                panel.set_image(image, self.packet(maximum, k))
                self.assertIn(f"yellow {middle}°", panel.legend.text())
                self.assertIn(f"red ≥{maximum}°", panel.legend.text())
                self.assertEqual(tuple(panel.pixels[0, 0]), (255,255,0))
            panel.clear_data()
            self.assertIn("red ≥30°", panel.legend.text())
        finally:
            panel.close()

    def test_cost_legend_keeps_cost_scale_and_notes_block(self):
        text = slope_k.display_legend(self.packet(source="costmap"), slope=False)
        self.assertIn("yellow 50", text)
        self.assertIn("red 100", text)
        self.assertIn("≥10°", text)

    def test_old_recordings_keep_30_degree_scale_and_bad_scale_is_ignored(self):
        for value in (None, {}, {"max_deg": True}, {"max_deg": float("nan")}, {"max_deg": -10}):
            self.assertEqual(slope_k.display_scale({"slope_display": value}), 30.)

    def test_image_metadata_survives_accept_and_relay_cache(self):
        with patch.object(sys, "argv", ["monitor", "--no-ssh"]):
            window = monitor.MonitorWindow(monitor.parse_args())
        try:
            packet = self.packet()
            window.on_packet("jetson", packet)
            self.assertEqual(window.images["slope"]["slope_display"], packet["slope_display"])
            self.assertEqual(window._image_packet_cache["slope"][1]["slope_display"], packet["slope_display"])
        finally:
            window.close()

    def test_recording_preserves_scale(self):
        from session_recording import SessionRecorder, SessionReader
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"scale.nomad"
            writer = SessionRecorder(path)
            writer.append("jetson", self.packet(), 0.)
            writer.close()
            reader = SessionReader(path)
            try:
                # Reader event API is also exercised by the existing session tests.
                rows = list(reader.iter_until(1.))
                self.assertEqual(rows[0]["event"]["slope_display"]["max_deg"], 10.)
            finally:
                reader.close()

    def test_developer_analysis_uses_same_degree_scale(self):
        from developer_traversability import TraversabilityWindow
        window = TraversabilityWindow()
        try:
            image = QtGui.QImage(20, 10, QtGui.QImage.Format_RGB888)
            image.fill(QtGui.QColor("yellow"))
            result = window._analyse({"images": {"slope": image},
                                      "image_meta": {"slope": self.packet()}}, "slope")
            self.assertEqual(result["slope_max_deg"], 10.)
            self.assertAlmostEqual(result["values"][0,0]*result["slope_max_deg"], 5.)
            self.assertEqual(result["rows"][-1][0], "10° 이상")
        finally:
            window.close()


if __name__ == "__main__":
    unittest.main()
