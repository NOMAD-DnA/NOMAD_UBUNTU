"""Mock-only controller reset contract. Never contacts boards or processes."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller_reset import RESULT_MARKER, describe, reset_controller


def packet(ok=True, reason="reset_accepted", message="raw reset", transport_ok=True):
    data = dict(ok=ok, reason=reason, message=message)
    return dict(ok=transport_ok, stdout=RESULT_MARKER + json.dumps(data) + "\n")


class ControllerResetTests(unittest.TestCase):
    def test_accepted_reset_uses_fixed_script_and_hides_password(self):
        with patch("controller_reset.run_remote", return_value=packet()) as call:
            result = reset_controller("user@192.168.3.142", "private")
        self.assertEqual(result, dict(ok=True, reason="reset_accepted", message="raw reset"))
        self.assertEqual(call.call_args.args[1].name, "reset_controller.sh")
        self.assertNotIn("private", json.dumps(result))
        self.assertIn("복구 완료", describe(result))

    def test_refusal_keeps_controller_message(self):
        value = packet(ok=False, reason="reset_refused",
                       message="Reset refused: need non-running state and fresh stationary telemetry")
        with patch("controller_reset.run_remote", return_value=value):
            result = reset_controller("user@lpp.local")
        self.assertFalse(result["ok"])
        self.assertIn("stationary", result["message"])
        self.assertIn("0.5초", describe(result))

    def test_bad_user_or_command_never_reaches_transport(self):
        for host in ("nvidia@jetson", "root@pi4", "user@pi4;reboot", "user@pi4\necho"):
            with patch("controller_reset.run_remote") as call:
                self.assertFalse(reset_controller(host)["ok"])
                call.assert_not_called()

    def test_only_confirmed_success(self):
        for data in (packet(ok="true"), packet(reason="reset_refused"), packet(transport_ok=False)):
            with patch("controller_reset.run_remote", return_value=data):
                self.assertFalse(reset_controller("user@pi4")["ok"])

    def test_missing_ambiguous_and_malformed_results(self):
        for stdout in ("", RESULT_MARKER + "bad", RESULT_MARKER + "[]", packet()["stdout"] * 2):
            with patch("controller_reset.run_remote", return_value=dict(ok=True, stdout=stdout)):
                self.assertFalse(reset_controller("user@pi4")["ok"])
        with patch("controller_reset.run_remote", side_effect=RuntimeError("private password")):
            self.assertEqual(reset_controller("user@pi4")["reason"], "local_control_error")

    def test_remote_script_sends_no_drive_or_arm_command(self):
        script = (Path(__file__).resolve().parents[1] / "remote/reset_controller.sh").read_text(encoding="utf-8")
        self.assertIn("std_srvs/srv/Trigger", script)
        for forbidden in ("commands/motor", "commands/servo", "drive_command", '"arm"', "commissioning_control"):
            self.assertNotIn(forbidden, script)


if __name__ == "__main__":
    unittest.main()
