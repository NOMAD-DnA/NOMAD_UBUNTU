"""Mock-only sensor recording contract (2026-11-15). Never contacts boards."""
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sensor_record as sr

REMOTE = Path(__file__).resolve().parents[1] / "remote"


def packet(**value):
    return dict(ok=True, stdout="noise\n" + sr.MARKER + json.dumps(value) + "\n")


class SensorRecordTests(unittest.TestCase):
    def test_scripts_parse(self):
        for name in ("record_start_jetson.sh", "record_stop_jetson.sh", "record_status_jetson.sh",
                     "record_gnss_start_pi.sh", "record_gnss_stop_pi.sh", "pi4_manual_prepare.sh", "pi4_manual_stop.sh"):
            subprocess.run(["bash", "-n", str(REMOTE / name)], check=True)

    def test_start_with_gps_runs_pi5_then_jetson_with_header(self):
        seen = []

        def fake(host, path, password, timeout_s=18):
            seen.append((host, path.name, Path(path).read_text().splitlines()[:2]))
            if path.name == "record_gnss_start_pi.sh":
                return packet(ok=True, reason="started", started=True)
            return packet(ok=True, reason="recording", session="/mnt/ssd/bags/x_주행", sensors_started=True)
        with patch("sensor_record.run_remote", side_effect=fake):
            value = sr.start("nvidia@192.168.1.152", "user@192.168.3.133", "pw", "주행", gps=True)
        self.assertTrue(value["ok"])
        self.assertEqual([s[1] for s in seen], ["record_gnss_start_pi.sh", "record_start_jetson.sh"])
        self.assertEqual(seen[1][2], ["SCENE=주행", "GPS=1"])

    def test_failed_jetson_start_undoes_gui_started_gnss(self):
        calls = []

        def fake(host, path, password, timeout_s=18):
            calls.append(path.name)
            if path.name == "record_gnss_start_pi.sh":
                return packet(ok=True, reason="started", started=True)
            if path.name == "record_start_jetson.sh":
                return packet(ok=False, reason="clock_not_synced")
            return packet(ok=True, reason="stopped")
        with patch("sensor_record.run_remote", side_effect=fake):
            value = sr.start("nvidia@192.168.1.152", "user@192.168.3.133", None, "a", gps=True)
        self.assertFalse(value["ok"])
        self.assertEqual(calls[-1], "record_gnss_stop_pi.sh")
        self.assertIn("Pi5", sr.describe(value))

    def test_bad_scene_and_accounts_rejected_without_ssh(self):
        with patch("sensor_record.run_remote") as call:
            self.assertEqual(sr.start("nvidia@1.2.3.4", None, None, "a b", gps=False)["reason"], "invalid_scene")
            self.assertEqual(sr.start("user@1.2.3.4", None, None, "a", gps=False)["reason"], "invalid_board_address")
            self.assertEqual(sr.start("nvidia@1.2.3.4", None, None, "x;rm", gps=False)["reason"], "invalid_scene")
        call.assert_not_called()

    def test_stop_reports_gps_count(self):
        def fake(host, path, password, timeout_s=18):
            if path.name == "record_stop_jetson.sh":
                return packet(ok=True, reason="stopped", session="/mnt/ssd/bags/20261115_1300_주행", duration_s=61.2,
                              messages=9000, gps_fix=61, bytes=1.3e9, sensors="stopped")
            return packet(ok=True, reason="stopped")
        with patch("sensor_record.run_remote", side_effect=fake):
            value = sr.stop("nvidia@192.168.1.152", "user@192.168.3.133", None)
        text = sr.describe(value)
        self.assertIn("GPS 위치 61개", text)
        self.assertIn("20261115_1300_주행", text)

    def test_missing_marker_keeps_ssh_reason(self):
        with patch("sensor_record.run_remote", return_value=dict(ok=False, stdout="", reason="ssh_unreachable")):
            self.assertEqual(sr.status("nvidia@192.168.1.152", None)["reason"], "ssh_unreachable")


if __name__ == "__main__":
    unittest.main()
