"""Filesystem-only checks; this module imports no ROS and sends no commands."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("recorder", Path(__file__).with_name("record_control_trials.py"))
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class RecorderTests(unittest.TestCase):
    def test_event_boundaries_rotation_redaction_and_csv(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            log = recorder.TrialLog(base / "logs", base / "workspace", max_bytes=500)
            log.on_status({"state": "IDLE", "reason": "ready", "token": "must-not-persist", "telemetry_fresh": True})
            log.on_status({"state": "RUNNING", "commanded_motor_erpm": 0})
            log.on_status({"state": "RUNNING", "commanded_motor_erpm": 700, "target_motor_erpm": 700})
            log.on_command("speed", 700)
            log.on_status({"state": "RUNNING", "commanded_motor_erpm": 0, "reason": "operator zero"})
            log.on_status({"state": "STOPPED_OUTPUT", "commanded_motor_erpm": 0, "reason": "operator stop"})
            log.close()
            self.assertEqual((log.run, log.motion_segment), (1, 1))
            events = [json.loads(line) for line in (base / "logs/events.jsonl").read_text().splitlines()]
            self.assertEqual(sum(row["event"] == "motor_command_start" for row in events), 1)
            self.assertEqual(sum(row["event"] == "motor_command_end" for row in events), 1)
            all_text = "".join(p.read_text() for p in (base / "logs").glob("*.jsonl"))
            self.assertNotIn("must-not-persist", all_text)
            self.assertGreater(len(list((base / "logs").glob("samples-*.jsonl"))), 1)
            rows = []
            for path in (base / "logs").glob("timeline-*.csv"):
                with path.open(newline="") as stream:
                    rows.extend(csv.DictReader(stream))
            self.assertTrue(any(row["command_erpm"] == "700" for row in rows))

    def test_cached_backup_snapshots_are_explicitly_stale_capable(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            workspace = base / "nomad_cpp_ws"
            backup = base / "vesc_ws/backups/trial"
            backup.mkdir(parents=True)
            (backup / "motor_payload.bin").write_bytes(b"motor-config")
            cache = workspace / "commission_session/firmware_readback.json"
            cache.parent.mkdir(parents=True)
            cache.write_text(json.dumps({"backup": str(backup)}))
            config = workspace / "src/nomad_control_cpp/config/commissioning.yaml"
            config.parent.mkdir(parents=True)
            config.write_text("mode: commission")
            log = recorder.TrialLog(base / "logs", workspace)
            log.snapshot()
            self.assertEqual(log.snapshot_number, 1)
            cache.write_text(json.dumps({"backup": str(backup), "changed": True}))
            log.snapshot()
            self.assertEqual(log.snapshot_number, 2)
            log.close()
            snap = base / "logs/snapshots/0001"
            self.assertEqual((snap / "vesc_cached_backup/motor_payload.bin").read_bytes(), b"motor-config")
            self.assertFalse(json.loads((snap / "manifest.json").read_text())["fresh_serial_read_performed"])


if __name__ == "__main__":
    unittest.main()
