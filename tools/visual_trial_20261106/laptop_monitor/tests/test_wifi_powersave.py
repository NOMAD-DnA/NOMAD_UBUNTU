"""2026-11-16: Wi-Fi power saving is turned off on every board at Start (non-blocking)."""
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import startup

PI5, JETSON, PI4 = "user@192.168.1.5", "nvidia@192.168.1.6", "user@192.168.1.7"


def fake_remote(calls, fail_wifi=()):
    def run(host, script, password, timeout_s=18, sudo=False):
        name = Path(script).name
        calls.append((host, name, sudo))
        role = "pi5" if host == PI5 else "jetson"
        ok = not (name == "wifi_powersave_off.sh" and host in fail_wifi)
        return {"ok": ok, "stdout": f"ROLE={role}\n", "state": "OK" if ok else "FAILED",
                "reason": "" if ok else "exit_1", "returncode": 0 if ok else 1}
    return run


class WifiPowersaveStartTest(unittest.TestCase):
    def start(self, fail_wifi=(), **kwargs):
        calls, events = [], []
        with patch.object(startup, "run_remote", side_effect=fake_remote(calls, fail_wifi)), \
             patch.object(startup, "stop_boards", return_value={"ok": True}), \
             patch.object(startup, "synchronize_clock", return_value={"ok": True}):
            value = startup.start_boards(PI5, JETSON, "test-only", progress=events.append, **kwargs)
        return value, calls, events

    def wifi_calls(self, calls):
        return {(host, sudo) for host, name, sudo in calls if name == "wifi_powersave_off.sh"}

    def test_pi5_and_jetson_with_sudo_before_sensors(self):
        value, calls, _ = self.start()
        self.assertTrue(value["ok"])
        self.assertEqual(self.wifi_calls(calls), {(PI5, True), (JETSON, True)})
        names = [name for _host, name, _sudo in calls]
        self.assertLess(names.index("check_lan_pi.sh"), names.index("wifi_powersave_off.sh"))
        self.assertLess(names.index("wifi_powersave_off.sh"), names.index("start_pi_gnss.sh"))

    def test_pi4_included_when_given(self):
        value, calls, _ = self.start(pi4_host=PI4)
        self.assertTrue(value["ok"])
        self.assertEqual(self.wifi_calls(calls), {(PI5, True), (JETSON, True), (PI4, True)})
        # Pi4 gets nothing else from Start.
        self.assertEqual({name for host, name, _ in calls if host == PI4}, {"wifi_powersave_off.sh"})

    def test_failure_does_not_stop_start(self):
        value, calls, events = self.start(fail_wifi=(PI4,), pi4_host=PI4)
        self.assertTrue(value["ok"])
        self.assertFalse(value["stages"]["wifi_power"]["pi4"]["ok"])
        self.assertTrue(any(e.get("stage") == "wifi_power" and "pi4" in e.get("message", "") and e.get("ok") is False
                            for e in events))
        self.assertIn("start_pi_vio_fusion.sh", [name for _h, name, _s in calls])

    def test_bad_or_duplicate_pi4_is_skipped(self):
        for bad in ("not-a-host", PI5):
            value, calls, _ = self.start(pi4_host=bad)
            self.assertTrue(value["ok"])
            self.assertEqual(self.wifi_calls(calls), {(PI5, True), (JETSON, True)})

    def test_cli_flag(self):
        with patch.object(startup, "start_boards", return_value={"ok": True, "state": "STARTED"}) as start:
            with patch("builtins.print"):
                startup.main(["--pi-host", PI5, "--jetson-host", JETSON, "--pi4-host", PI4])
        self.assertEqual(start.call_args.kwargs["pi4_host"], PI4)

    def test_script_is_runtime_only(self):
        path = ROOT / "remote/wifi_powersave_off.sh"
        text = "\n".join(line.split("#", 1)[0] for line in path.read_text().splitlines())  # code only
        subprocess.run(["bash", "-n", str(path)], check=True)
        self.assertIn("set power_save off", text)
        self.assertIn("power off", text)
        for forbidden in ("nmcli", "/etc/", "connection modify", "reboot"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
