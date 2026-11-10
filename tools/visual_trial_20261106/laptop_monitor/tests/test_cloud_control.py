import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cloud_control import switch_cloud_source, RESULT_MARKER


def result(value, ok=True):
    return {"ok": ok, "stdout": RESULT_MARKER + json.dumps(value) + "\n", "reason": "ok"}


class CloudControlTest(unittest.TestCase):
    def test_confirmation_requires_matching_source_and_successful_transport(self):
        confirmed = {"ok": True, "state": "SOURCE_CONFIRMED", "source": "device", "map_reset": False}
        with patch("cloud_control.run_remote", return_value=result(confirmed)) as remote:
            self.assertTrue(switch_cloud_source("nvidia@192.168.1.152", "device")["ok"])
            self.assertEqual(remote.call_args.args[1].name, "switch_cloud_device.sh")
        for invalid, transport in [(dict(confirmed, source="opencv"), True),
                                   (dict(confirmed, state="STARTED"), True),
                                   (dict(confirmed, map_reset=True), True), (confirmed, False)]:
            with self.subTest(invalid=invalid, transport=transport):
                with patch("cloud_control.run_remote", return_value=result(invalid, transport)):
                    self.assertFalse(switch_cloud_source("nvidia@192.168.1.152", "device")["ok"])

    def test_failed_switch_can_report_verified_restoration(self):
        value = {"ok": False, "state": "UNCONFIRMED", "reason": "new_source_no_fresh_cloud",
                 "source": "opencv", "rollback_confirmed": True, "map_reset": False}
        with patch("cloud_control.run_remote", return_value=result(value, False)):
            actual = switch_cloud_source("nvidia@192.168.1.152", "device")
            self.assertFalse(actual["ok"])
            self.assertTrue(actual["rollback_confirmed"])
            self.assertEqual(actual["source"], "opencv")
        with patch("cloud_control.run_remote", return_value=result(dict(value, rollback_confirmed=False), False)):
            self.assertIsNone(switch_cloud_source("nvidia@192.168.1.152", "device")["source"])

    def test_invalid_address_source_and_unverified_results(self):
        with patch("cloud_control.run_remote") as remote:
            for host, mode in [("user@192.168.1.152", "device"), ("nvidia@bad;host", "device"),
                               ("nvidia@192.168.1.152", "depthai;touch")]:
                self.assertFalse(switch_cloud_source(host, mode)["ok"])
            remote.assert_not_called()
        for stdout in ["", RESULT_MARKER + "{}", RESULT_MARKER + "[]", RESULT_MARKER + "bad",
                       RESULT_MARKER + "{}\n" + RESULT_MARKER + "{}"]:
            with patch("cloud_control.run_remote", return_value={"ok": True, "stdout": stdout}):
                self.assertFalse(switch_cloud_source("nvidia@192.168.1.152", "device")["ok"])

    def test_status_is_explicitly_read_only(self):
        with patch("cloud_control.run_remote", return_value=result({
                "ok": True, "state": "SOURCE_CONFIRMED", "source": "opencv", "map_reset": False})) as remote:
            self.assertEqual(switch_cloud_source("nvidia@192.168.1.152", "status")["source"], "opencv")
            self.assertEqual(remote.call_args.args[1].name, "switch_cloud_status.sh")


class CloudGuiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        import monitor
        self.monitor = monitor
        with patch.object(sys, "argv", ["monitor", "--no-ssh", "--enable-trial-control"]):
            self.window = monitor.MonitorWindow(monitor.parse_args())

    def tearDown(self):
        self.window.cloud_switch_in_progress = False
        self.window.model_switch_in_progress = False
        self.window.close()
        self.app.processEvents()

    def test_choices_and_read_only_mode(self):
        w = self.window
        self.assertEqual([w.cloud_combo.itemData(i) for i in range(2)], ["opencv", "device"])
        self.assertIsNone(w.active_cloud_source)
        self.assertFalse(w.cloud_button.isEnabled())
        with patch.object(self.monitor, "CloudSwitchWorker") as worker:
            w.apply_cloud_source()
            worker.assert_not_called()

    def test_confirmed_switch_and_failed_rollback_states(self):
        w = self.window
        w.on_cloud_switched({"ok": True, "source": "device"})
        self.assertEqual(w.active_cloud_source, "device")
        self.assertEqual(w.cloud_combo.currentData(), "device")
        self.assertFalse(w.state()["oak_cloud_source"]["switching"])
        w.on_cloud_switched({"ok": False, "source": None, "reason": "timeout"})
        self.assertIsNone(w.active_cloud_source)
        w.on_cloud_switched({"ok": False, "source": "opencv", "rollback_confirmed": True})
        self.assertEqual(w.active_cloud_source, "opencv")
        self.assertIn("복원 확인", w.cloud_message.text())

    def test_switch_serializes_model_and_stop_and_close(self):
        w = self.window
        w.args.no_ssh = False
        w.set_cloud_controls_enabled(True)
        with patch.dict(os.environ, {"NOMAD_BOARD_PASSWORD1": "test-only"}), \
                patch.object(self.monitor, "CloudSwitchWorker") as worker:
            w.apply_cloud_source()
            worker.return_value.start.assert_called_once()
            self.assertTrue(w.cloud_switch_in_progress)
            self.assertFalse(w.model_button.isEnabled())
            self.assertFalse(w.stop_button.isEnabled())
            w.apply_model()
            self.assertFalse(w.model_switch_in_progress)
            class Event:
                ignored = False
                def ignore(self): self.ignored = True
            event = Event()
            w.closeEvent(event)
            self.assertTrue(event.ignored)
            w.on_cloud_switched({"ok": True, "source": "opencv"})
            w.on_cloud_switch_finished()
            self.assertTrue(w.cloud_button.isEnabled())
            self.assertTrue(w.model_button.isEnabled())

    def test_missing_result_never_becomes_success(self):
        self.window.on_cloud_switch_finished()
        self.assertIsNone(self.window.active_cloud_source)
        self.assertFalse(self.window.cloud_switch_result["ok"])


if __name__ == "__main__": unittest.main()
