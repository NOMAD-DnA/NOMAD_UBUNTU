"""Mock-only keyboard drive contract (2026-11-15). Never contacts boards; a fake Pi4 server answers."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import keyboard_drive as kd


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 0.001)


class FakePi4:
    """Status like the 1afcbb1 controller in raw Wi-Fi manual mode; records every POST."""

    def __init__(self, clock):
        self.clock, self.posts, self.fail_status, self.reject = clock, [], False, False
        self.rate, self.step = 1.0, 0.0  # server clock speed (chrony slew) and jumps
        self.status = dict(connected=True, telemetry_fresh=True, state="IDLE", control_units="raw", commissioning=True,
                           continuous_manual_supported=True, wifi_manual_profile=True, manual_timeout_s=2,
                           command_max_age_s=1.5, browser_freshness_s=1.5, browser_request_timeout_ms=1500,
                           commissioning_setup_acknowledged=True, driver_ready=True, stationary=True,
                           servo_min=0, servo_center=0.5, servo_max=1, motor_max_erpm=1850, manual_reverse_max_erpm=800,
                           token="tok")

    def __call__(self, method, path, body=None, token="", timeout=0.6):
        self.clock.t += 0.002
        if method == "GET":
            if self.fail_status:
                raise OSError("down")
            return dict(self.status, server_time_s=5000.0 + self.rate * self.clock.t + self.step)
        assert token == "tok"
        self.posts.append((path, dict(body)))
        if self.reject and path == "/api/commissioning" and body.get("action") == 2:
            return dict(accepted=False, reason="stale command")
        return dict(accepted=True, reason="ok")


def run(client, clock, seconds):
    end = clock.t + seconds
    while clock.t < end:
        started = clock.t
        with client.lock:
            stop, start = client.want_stop, client.want_start
            client.want_start = False
        if stop:
            client._stop()
        client._poll()
        if start and not stop:
            client._start()
        client._update()
        clock.sleep(kd.UPDATE_S - (clock.t - started))


def updates(fake):
    return [b for p, b in fake.posts if p == "/api/commissioning" and b.get("action") == 2]


class KeyboardDriveTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.fake = FakePi4(self.clock)
        self.client = kd.DriveClient(self.fake, clock=self.clock, sleep=self.clock.sleep)
        run(self.client, self.clock, 0.3)

    def test_start_sends_zero_speed_center_and_wifi_timing(self):
        self.assertIs(kd.control_timing(self.fake.status), kd.WIFI)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        start = [b for p, b in self.fake.posts if b.get("action") == 1][0]
        self.assertEqual((start["axis"], start["motor_erpm"], start["servo_position"], start["duration_s"]), (3, 0.0, 0.5, 0))
        self.assertTrue(self.client.snapshot()["active"])
        self.assertTrue(all(b["motor_erpm"] == 0.0 for b in updates(self.fake)))

    def test_forward_only_while_held_then_zero_on_release(self):
        self.client.set_limits(1000, 0.6)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(True, True, False)
        run(self.client, self.clock, 0.3)
        held = updates(self.fake)[-1]
        self.assertEqual(held["motor_erpm"], 1000.0)
        self.assertAlmostEqual(held["servo_position"], 0.5 + 0.6 * 0.5)  # rear steering: left key -> servo up (inverted, default)
        self.client.set_keys(False, False, False)
        run(self.client, self.clock, 0.2)
        released = updates(self.fake)[-1]
        self.assertEqual((released["motor_erpm"], released["servo_position"]), (0.0, 0.5))
        self.assertTrue(self.client.snapshot()["active"])  # session stays: controller brakes/holds on 0

    def test_speed_clamped_to_controller_max(self):
        self.client.set_limits(5000, 1.0)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(True, False, True)
        run(self.client, self.clock, 0.2)
        self.assertEqual(updates(self.fake)[-1]["motor_erpm"], 1850)
        self.assertEqual(updates(self.fake)[-1]["servo_position"], 0.0)  # right key, inverted

    def test_heartbeat_about_20hz_with_increasing_sequence_and_server_time(self):
        self.client.request_start()
        run(self.client, self.clock, 1.0)
        ups = updates(self.fake)
        self.assertGreaterEqual(len(ups), 15)
        self.assertEqual([u["sequence"] for u in ups], sorted(set(u["sequence"] for u in ups)))
        self.assertTrue(all(u["server_time_s"] > 5000 for u in ups))

    def test_stale_status_closes_session(self):
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.fake.fail_status = True
        run(self.client, self.clock, 2.0)
        self.assertFalse(self.client.snapshot()["active"])
        self.assertTrue(any(b.get("action") == 3 for p, b in self.fake.posts))

    def test_rejected_update_closes_session(self):
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.fake.reject = True
        run(self.client, self.clock, 0.2)
        self.assertFalse(self.client.snapshot()["active"])
        self.assertIn("stale", self.client.snapshot()["message"])

    def test_stop_closes_and_posts_stop(self):
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(True, False, False)
        self.client.request_stop()
        run(self.client, self.clock, 0.1)
        self.assertIn("/api/stop", [p for p, b in self.fake.posts])
        self.assertFalse(self.client.snapshot()["active"])
        self.assertFalse(self.client.snapshot()["keys"]["forward"])

    def test_start_refused_when_not_acknowledged_or_moving(self):
        self.fake.status["commissioning_setup_acknowledged"] = False
        run(self.client, self.clock, 0.2)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.assertFalse(self.client.snapshot()["active"])
        self.fake.status.update(commissioning_setup_acknowledged=True, stationary=False)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.assertFalse(self.client.snapshot()["active"])
        self.assertIn("멈춘", self.client.snapshot()["message"])

    def test_non_idle_state_resets_first(self):
        self.fake.status["state"] = "STOPPED_OUTPUT"
        run(self.client, self.clock, 0.2)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        paths = [p for p, b in self.fake.posts]
        self.assertLess(paths.index("/api/reset"), paths.index("/api/commissioning"))

    def test_unknown_timing_contract_refuses(self):
        self.assertIsNone(kd.control_timing(dict(self.fake.status, manual_timeout_s=3)))
        self.assertIs(kd.control_timing(dict(manual_timeout_s=.35, command_max_age_s=.3)), kd.STRICT)

    def test_server_clock_inconsistency_invalidates(self):
        clock = kd.ServerClock()
        self.assertTrue(clock.observe(1.0, 1.1, 1000.0, 2.0))
        self.assertIsNotNone(clock.now(1.2, 2.0))
        self.assertFalse(clock.observe(1.3, 1.4, 900.0, 2.0))  # server clock jumped back
        self.assertIsNone(clock.now(1.5, 2.0))

    def test_slewing_pi4_clock_keeps_session(self):
        # 2026-11-15 Pi4 5.6 days behind, chrony slewing ~7.5 % fast: must not drop the session
        self.fake.rate = 1.075
        run(self.client, self.clock, 0.5)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(True, False, False)
        run(self.client, self.clock, 5.0)
        self.assertTrue(self.client.snapshot()["active"])
        self.assertGreaterEqual(len(updates(self.fake)), 80)
        self.assertFalse(any(b.get("action") == 3 for p, b in self.fake.posts))

    def test_estimate_never_ahead_of_server_within_drift(self):
        for rate in (0.92, 1.0, 1.08):
            clock = kd.ServerClock()
            t = 0.0
            for i in range(40):
                sent, received = t, t + 0.015
                stamp = 1000 + rate * (t + 0.007)
                self.assertTrue(clock.observe(sent, received, stamp, 2.0), (rate, i))
                at = received + 0.04
                self.assertLessEqual(clock.now(at, 2.0), 1000 + rate * at + 1e-9)
                self.assertGreater(clock.now(at, 2.0), 1000 + rate * at - 0.25)
                t += 0.1

    def test_clock_step_still_drops_session(self):
        self.client.request_start()
        run(self.client, self.clock, 0.3)
        self.fake.step = 30.0  # makestep / controller restart
        run(self.client, self.clock, 0.3)
        self.assertTrue(any(b.get("action") == 3 for p, b in self.fake.posts))

    def test_stop_service_parses_marker(self):
        out = dict(ok=True, stdout=kd.MARKER + json.dumps(dict(ok=True, reason="stopped")) + "\n")
        with patch("keyboard_drive.run_remote", return_value=out) as call:
            self.assertTrue(kd.stop_service("user@192.168.2.252", None)["ok"])
        self.assertEqual(call.call_args.args[1].name, "pi4_manual_stop.sh")

    def test_reverse_only_while_held_and_clamped(self):
        self.client.set_limits(1000, 0.5, 1200)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(False, False, True, back=True)
        run(self.client, self.clock, 0.2)
        held = updates(self.fake)[-1]
        self.assertEqual(held["motor_erpm"], -800)  # clamped to Pi4 manual_reverse_max_erpm
        self.assertAlmostEqual(held["servo_position"], 0.25)  # right key, inverted
        self.client.set_keys(False, False, False, back=False)
        run(self.client, self.clock, 0.2)
        self.assertEqual(updates(self.fake)[-1]["motor_erpm"], 0.0)

    def test_reverse_disabled_when_pi4_does_not_allow_it(self):
        self.fake.status["manual_reverse_max_erpm"] = 0
        run(self.client, self.clock, 0.2)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(False, False, False, back=True)
        run(self.client, self.clock, 0.2)
        self.assertEqual(updates(self.fake)[-1]["motor_erpm"], 0.0)
        del self.fake.status["manual_reverse_max_erpm"]  # older controller without the field
        run(self.client, self.clock, 0.2)
        self.assertEqual(updates(self.fake)[-1]["motor_erpm"], 0.0)

    def test_steer_invert_off_matches_pi4_page(self):
        self.client.set_steer_invert(False)
        self.client.set_limits(1000, 0.6)
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(False, True, False)
        run(self.client, self.clock, 0.2)
        self.assertAlmostEqual(updates(self.fake)[-1]["servo_position"], 0.5 - 0.6 * 0.5)

    def test_forward_and_back_together_is_zero(self):
        self.client.request_start()
        run(self.client, self.clock, 0.2)
        self.client.set_keys(True, False, False, back=True)
        run(self.client, self.clock, 0.2)
        self.assertEqual(updates(self.fake)[-1]["motor_erpm"], 0.0)

    def test_prepare_parses_marker(self):
        out = dict(ok=True, stdout=kd.MARKER + json.dumps(dict(ok=True, reason="ready", message="m")) + "\n")
        with patch("keyboard_drive.run_remote", return_value=out) as call:
            self.assertEqual(kd.prepare("user@192.168.2.252", "pw")["reason"], "ready")
        self.assertEqual(call.call_args.args[1].name, "pi4_manual_prepare.sh")
        self.assertFalse(kd.prepare("nvidia@192.168.1.152", None)["ok"])


if __name__ == "__main__":
    unittest.main()
