"""Mock-only restart control contract. Never contacts boards or processes."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vio_control import RESULT_MARKER, restart_vio


def packet(preserved=True, **extra):
    data = dict(ok=True, state="VIO_RESTARTED", reason="fusion_preserved" if preserved else "new_vio_origin",
                fusion_preserved=preserved)
    data.update(extra)
    return dict(ok=True, stdout=RESULT_MARKER + json.dumps(data) + "\n")


class VioRestartTests(unittest.TestCase):
    def test_preserves_live_fusion_or_reports_new_origin(self):
        for preserved in (True, False):
            with patch("vio_control.run_remote", return_value=packet(preserved)) as call:
                result = restart_vio("user@192.168.3.156", "private")
            self.assertTrue(result["ok"])
            self.assertIs(result["fusion_preserved"], preserved)
            self.assertEqual(call.call_args.args[1].name, "restart_vio.sh")
            self.assertNotIn("private", json.dumps(result))

    def test_bad_user_or_command_never_reaches_transport(self):
        for host in ("nvidia@jetson", "root@pi5", "user@pi5;reboot", "user@pi5\necho"):
            with patch("vio_control.run_remote") as call:
                result = restart_vio(host)
                self.assertFalse(result["ok"])
                call.assert_not_called()

    def test_only_confirmed_boolean_success(self):
        for data in (packet(ok="true"), packet(fusion_preserved="true"), packet(reason="new_vio_origin"),
                     dict(ok=False, stdout=packet()["stdout"]), packet(state="STARTED")):
            with patch("vio_control.run_remote", return_value=data):
                self.assertFalse(restart_vio("user@pi5")["ok"])

    def test_bounded_failure_reasons_and_no_raw_output(self):
        for reason in ("epoch_publish_failed", "untracked_vio_running", "vio_start_failed", "ssh_unreachable"):
            value = dict(ok=False, state="VIO_RESTART_FAILED", reason=reason, fusion_preserved=False)
            with patch("vio_control.run_remote", return_value=dict(ok=True, stdout=RESULT_MARKER + json.dumps(value))):
                self.assertEqual(restart_vio("user@pi5"), value)
        with patch("vio_control.run_remote", side_effect=RuntimeError("private password")):
            self.assertEqual(restart_vio("user@pi5")["reason"], "local_control_error")
        with patch("vio_control.run_remote", return_value=dict(ok=False, reason="secret", stdout="password")):
            self.assertNotIn("password", json.dumps(restart_vio("user@pi5")))

    def test_missing_ambiguous_and_malformed_results(self):
        for stdout in ("", RESULT_MARKER + "bad", RESULT_MARKER + "[]", packet()["stdout"] * 2):
            with patch("vio_control.run_remote", return_value=dict(ok=True, stdout=stdout)):
                self.assertFalse(restart_vio("user@pi5")["ok"])


if __name__ == "__main__":
    unittest.main()
