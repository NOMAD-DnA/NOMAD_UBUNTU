"""Role selection must not let spectator actions launch board commands."""
import os
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
import launcher


class LauncherRelayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="nomad-launcher-relay-")
        self.state_patch = patch.object(launcher, "STATE_ROOT", Path(self.temp.name))
        self.state_patch.start()
        self.window = launcher.Launcher()
        self.window.begin_process = Mock()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.state_patch.stop()
        self.temp.cleanup()

    def test_spectator_never_needs_targets_or_board_password(self):
        self.window.connection_mode.setCurrentIndex(1)
        self.window.relay_host.setText("192.168.1.99:8768")
        self.window.targets = Mock(side_effect=AssertionError("Spectator queried SSH targets"))
        self.window.launch_viewer = Mock()
        self.window.start_trial()
        self.window.stop_trial()
        self.window.discover()
        self.window.begin_process.assert_not_called()
        self.window.connect_viewer()
        self.window.launch_viewer.assert_called_once()
        self.assertIsNone(self.window.launch_viewer.call_args.args[0])
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertFalse(self.window.stop_button.isEnabled())
        self.assertFalse(self.window.password.isEnabled())
        args = self.window.monitor_arguments(None, Path(self.temp.name))
        self.assertIn("--relay-host", args)
        self.assertNotIn("--pi-command", args)
        self.assertNotIn("--enable-trial-control", args)
        with patch.dict(os.environ, {"NOMAD_BOARD_PASSWORD1": "test-only", "NOMAD_BOARD_PASSWORD": "test-only"}):
            env = self.window.child_environment(Path(self.temp.name))
        self.assertNotIn("NOMAD_BOARD_PASSWORD1", env)
        self.assertNotIn("NOMAD_BOARD_PASSWORD", env)

    def test_operator_passes_server_option_and_occupied_port_blocks_start(self):
        args = self.window.monitor_arguments(("user@192.168.1.5", "nvidia@192.168.1.6"), Path(self.temp.name))
        self.assertIn("--relay-listen", args)
        self.assertNotIn("--relay-host", args)
        self.window.hosts["pi"].setCurrentText("user@192.168.1.5")
        self.window.hosts["jetson"].setCurrentText("nvidia@192.168.1.6")
        self.window.password.setText("test-only")
        with socket.socket() as occupied:
            occupied.bind(("0.0.0.0", 0)); occupied.listen()
            self.window.relay_port.setValue(occupied.getsockname()[1])
            self.window.start_trial()
        self.window.begin_process.assert_not_called()
        self.assertIn("이미 사용", self.window.status.text())

    def test_relay_address_validation(self):
        self.assertEqual(launcher.validate_relay_address("192.168.1.2"), "192.168.1.2:8768")
        self.assertEqual(launcher.validate_relay_address("main.local:9000"), "main.local:9000")
        for bad in ("", "0.0.0.0", "999.3.4.5", "pc:0", "pc:65536", "pc:abc", "user@pc", "pc;cmd", "[::1]:8768"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                launcher.validate_relay_address(bad)


if __name__ == "__main__":
    unittest.main()
