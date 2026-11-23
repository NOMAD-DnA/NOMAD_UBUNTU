"""Transport mocked throughout. Never contacts a board or calls a service."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from heading_control import RESULT_MARKER, reset_heading


def result(ok=True, state="HEADING_RESET", reason="ok", transport_ok=True):
    return dict(ok=transport_ok, reason="ok", stdout=RESULT_MARKER +
                json.dumps(dict(ok=ok, state=state, reason=reason)) + "\n")


class HeadingResetTests(unittest.TestCase):
    def test_single_fixed_script_and_password_only_to_transport(self):
        with patch("heading_control.run_remote", return_value=result()) as call:
            value = reset_heading("user@192.168.3.156", "test-secret")
        self.assertEqual(value, dict(ok=True, state="HEADING_RESET", reason="ok"))
        args, kwargs = call.call_args
        self.assertEqual(args[0], "user@192.168.3.156")
        self.assertEqual(args[1].name, "reset_heading_from_magnetometer.sh")
        self.assertEqual(args[2], "test-secret")
        self.assertEqual(kwargs, dict(timeout_s=14.))
        self.assertNotIn("test-secret", json.dumps(value))

    def test_wrong_user_host_or_options_never_contact_board(self):
        for host in ("nvidia@192.168.1.152", "user@127.0.0.1;echo bad", "user", "root@pi5.local"):
            with patch("heading_control.run_remote") as call:
                self.assertFalse(reset_heading(host)["ok"])
                call.assert_not_called()
        for timeout in (True, float("nan"), -1, 61):
            with patch("heading_control.run_remote") as call:
                self.assertFalse(reset_heading("user@pi5.local", timeout_s=timeout)["ok"])
                call.assert_not_called()

    def test_success_must_be_actual_boolean_and_transport_confirmed(self):
        for data in (result(ok="true"), result(state="SUCCEEDED"), result(reason="maybe"), result(transport_ok=False)):
            with patch("heading_control.run_remote", return_value=data):
                self.assertFalse(reset_heading("user@pi5.local")["ok"])

    def test_safe_backend_reason_only(self):
        for reason in ("VIO_ONLY_REQUIRED", "HOLD_VEHICLE_STILL", "MAGNETOMETER_NOT_FRESH", "service_timeout", "service_unavailable"):
            with patch("heading_control.run_remote", return_value=result(False, "HEADING_RESET_FAILED", reason)):
                value = reset_heading("user@pi5.local")
                self.assertEqual(value["reason"], reason)
        with patch("heading_control.run_remote", return_value=result(False, "FAILED", "secret remote details")):
            self.assertEqual(reset_heading("user@pi5.local")["reason"], "reset_rejected")

    def test_no_raw_output_or_exception_leak(self):
        for data in (dict(ok=False, reason="ssh_unreachable", stdout="secret"),
                     dict(ok=False, reason="secret", stdout="secret")):
            with patch("heading_control.run_remote", return_value=data):
                value = reset_heading("user@pi5.local")
                self.assertEqual(set(value), {"ok", "state", "reason"})
                self.assertNotIn("secret", json.dumps(value))
        with patch("heading_control.run_remote", side_effect=RuntimeError("password secret")):
            self.assertEqual(reset_heading("user@pi5.local")["reason"], "local_control_error")

    def test_missing_ambiguous_and_invalid_marker(self):
        for stdout in ("", RESULT_MARKER + "bad-json", RESULT_MARKER + "[]",
                       result()["stdout"] * 2):
            with patch("heading_control.run_remote", return_value=dict(ok=True, stdout=stdout)):
                self.assertFalse(reset_heading("user@pi5.local")["ok"])


if __name__ == "__main__":
    unittest.main()
