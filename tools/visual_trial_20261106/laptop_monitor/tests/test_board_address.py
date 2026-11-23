"""Mock-only board address recovery contract. Never scans or contacts a network."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import board_address as ba

PI5, JET, PI4 = "user@192.168.3.164", "nvidia@192.168.1.152", "user@lpp.local"
KEY = ("ssh-ed25519", "AAAApi5")


class BoardAddressTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = patch.dict(os.environ, {"XDG_STATE_HOME": self.tmp.name})
        self.env.start()
        ba.register(pi5=PI5, jetson=JET, pi4=PI4)
        self.patches = [patch.object(ba, "_local_networks", return_value=([], set())),
                        patch.object(ba, "_wake_neighbours"),
                        patch.object(ba, "_run", return_value="")]
        for p in self.patches:
            p.start()

    def tearDown(self):
        for p in self.patches:
            p.stop()
        self.env.stop()
        self.tmp.cleanup()

    def test_unregistered_host_is_never_looked_for(self):
        with patch.object(ba, "_neighbour_macs") as macs:
            self.assertIsNone(ba.recover("user@10.0.0.9"))
            macs.assert_not_called()

    def test_moved_pi5_found_by_mac_and_host_key(self):
        with patch.object(ba, "_neighbour_macs", return_value={"192.168.3.163": ba.SEED_MACS["pi5"],
                                                               "192.168.3.170": "aa:bb:cc:dd:ee:ff"}), \
                patch.object(ba, "_ssh_open", return_value=True), \
                patch.object(ba, "_stored_keys", return_value={KEY}), \
                patch.object(ba, "_scanned_keys", return_value={KEY}) as scanned:
            found = ba.recover(PI5)
        self.assertEqual(found, {"host": "user@192.168.3.163", "alias": "192.168.3.164", "via": None, "method": "mac"})
        scanned.assert_called_once_with("192.168.3.163")  # the non-matching MAC was never contacted
        self.assertEqual(ba.route(PI5)["host"], "user@192.168.3.163")
        self.assertEqual(ba.route(PI5)["alias"], "192.168.3.164")

    def test_host_key_mismatch_is_rejected_then_wired_route_used(self):
        with patch.object(ba, "_neighbour_macs", return_value={"192.168.3.163": ba.SEED_MACS["pi5"]}), \
                patch.object(ba, "_ssh_open", return_value=True), \
                patch.object(ba, "_stored_keys", return_value={KEY}), \
                patch.object(ba, "_scanned_keys", return_value={("ssh-ed25519", "OTHER")}):
            found = ba.recover(PI5)
        self.assertEqual(found["method"], "wired")
        self.assertEqual(found["host"], "user@10.77.0.2")
        self.assertEqual(found["via"], JET)
        self.assertEqual(found["alias"], "192.168.3.164")

    def test_jetson_reached_through_pi5_wired(self):
        with patch.object(ba, "_neighbour_macs", return_value={}), \
                patch.object(ba, "_ssh_open", side_effect=lambda a, timeout=1.5: a == "192.168.3.164"):
            found = ba.recover(JET)
        self.assertEqual((found["host"], found["via"], found["method"]), ("nvidia@10.77.0.1", PI5, "wired"))

    def test_stale_detour_dropped_when_configured_address_answers(self):
        data = ba._load()
        data.setdefault("current", {})["user@192.168.3.164"] = {
            "host": "user@10.42.0.15", "alias": "192.168.3.164", "via": None, "method": "mac", "role": "pi5", "t": 1}
        ba._save(data)
        self.assertEqual(ba.route("user@192.168.3.164")["host"], "user@10.42.0.15")
        with patch.object(ba, "_ssh_open", side_effect=lambda a, timeout=1.5: a == "192.168.3.164"), \
                patch.object(ba, "_neighbour_macs") as macs:
            found = ba.recover("user@192.168.3.164")
        macs.assert_not_called()                     # no scan needed
        self.assertEqual(found, {"host": "user@192.168.3.164", "alias": None, "via": None, "method": "configured"})
        self.assertEqual(ba.route("user@192.168.3.164"),
                         {"host": "user@192.168.3.164", "alias": None, "via": None, "method": None})
        # Configured address still down: the detour search runs as before (not short-circuited).
        data = ba._load()
        data.setdefault("current", {})["user@192.168.3.164"] = {
            "host": "user@10.42.0.15", "alias": "192.168.3.164", "via": None, "method": "mac", "role": "pi5", "t": 1}
        ba._save(data)
        with patch.object(ba, "_ssh_open", return_value=False), patch.object(ba, "_neighbour_macs", return_value={}):
            self.assertIsNone(ba.recover("user@192.168.3.164"))
        self.assertEqual(ba.route("user@192.168.3.164")["host"], "user@10.42.0.15")

    def test_pi4_has_no_wired_peer_and_fails_cleanly(self):
        with patch.object(ba, "_neighbour_macs", return_value={}), patch.object(ba, "_ssh_open", return_value=True):
            self.assertIsNone(ba.recover(PI4))

    def test_forbidden_board_is_never_contacted(self):
        with patch.object(ba, "_neighbour_macs", return_value={"192.168.2.158": ba.SEED_MACS["pi5"]}), \
                patch.object(ba, "_scanned_keys") as scanned, \
                patch.object(ba.socket, "create_connection") as connect:
            self.assertFalse(ba._ssh_open("192.168.2.158"))
            self.assertFalse(ba._same_board("192.168.2.158", "192.168.3.164", []))
            ba.recover(PI4)  # unrelated board: still no contact
        connect.assert_not_called()
        scanned.assert_not_called()

    def test_retry_backoff(self):
        with patch.object(ba, "_neighbour_macs", return_value={}) as macs, patch.object(ba, "_ssh_open", return_value=False):
            self.assertIsNone(ba.recover(PI5))
            self.assertIsNone(ba.recover(PI5))
        self.assertEqual(macs.call_count, 1)  # second call inside RETRY_AFTER_S does nothing

    def test_learn_records_mac_of_direct_connection_only(self):
        with patch.object(ba, "_neighbour_macs", return_value={"192.168.3.164": "11:22:33:44:55:66"}):
            ba.learn(PI5)
            ba.learn(PI5, {"host": "user@10.77.0.2", "via": JET})
        self.assertEqual(ba._load()["macs"]["pi5"], "11:22:33:44:55:66")

    def test_route_options(self):
        common = ["-o", "BatchMode=yes"]
        options = ba.ssh_route_options({"host": "user@10.77.0.2", "via": JET, "alias": "192.168.3.164"}, common)
        self.assertEqual(options[0:2], ["-o", "ProxyCommand=ssh -o BatchMode=yes -W %h:%p " + JET])
        self.assertEqual(options[2:], ["-o", "HostKeyAlias=192.168.3.164"])
        self.assertEqual(ba.ssh_route_options({"host": PI5}, common), [])


class CollectorFollowTests(unittest.TestCase):
    def test_collector_reconnects_to_recovered_route(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        import monitor
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {"XDG_STATE_HOME": tmp}):
            ba.register(pi5=PI5)
            worker = monitor.Collector("pi", PI5, "probe")
            targets = []
            def once(target):
                targets.append(target["host"])
                worker.stats["error_code"] = "network_unavailable" if len(targets) == 1 else None
                if len(targets) == 1:
                    return
                worker.stats["json_events"] += 1
            stops = iter([False] * 110 + [True] * 1000)  # covers one 10 s wait (100 checks)
            moved = {"host": "user@192.168.3.163", "alias": "192.168.3.164", "via": None, "method": "mac"}
            def recover(host, known=None):
                with ba._lock:
                    data = ba._load(); data.setdefault("current", {})[host] = moved; ba._save(data)
                return moved
            with patch.object(worker, "run_once", side_effect=once), \
                    patch.object(worker, "isInterruptionRequested", side_effect=lambda: next(stops)), \
                    patch.object(ba, "recover", side_effect=recover) as rec, patch.object(ba, "learn"), \
                    patch.object(monitor.time, "sleep"):
                worker.run()
            self.assertEqual(targets[:2], [PI5, "user@192.168.3.163"])
            rec.assert_called_once()


if __name__ == "__main__":
    unittest.main()
