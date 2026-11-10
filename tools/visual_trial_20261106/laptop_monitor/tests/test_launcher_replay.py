"""Offline recordings must open without any board or relay entry point."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
import launcher


class LauncherReplayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="nomad-launcher-replay-")
        self.state_patch = patch.object(launcher, "STATE_ROOT", Path(self.temp.name) / "state")
        self.state_patch.start()
        self.window = launcher.Launcher()
        self.recording = Path(self.temp.name) / "시험 기록.nomad"
        self.recording.write_bytes(b"format is validated by monitor, not the launcher")
        self.window.targets = Mock(side_effect=AssertionError("Playback queried board targets"))
        self.window.relay_port_available = Mock(side_effect=AssertionError("Playback checked a relay port"))
        self.window.begin_process = Mock(side_effect=AssertionError("Playback launched a board action"))
        self.window.save_hosts = Mock(side_effect=AssertionError("Playback persisted board targets"))

    def tearDown(self):
        self.window.viewer = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.state_patch.stop()
        self.temp.cleanup()

    def test_open_button_launches_only_replay_without_credentials(self):
        self.window.password.setText("form-only-secret")
        fake_process = Mock()
        fake_process.poll.return_value = None
        inherited = {"NOMAD_BOARD_PASSWORD1": "board-secret", "NOMAD_BOARD_PASSWORD": "old-secret",
                     "NOMAD_SSH_PASSWORD": "ssh-secret", "API_TOKEN": "api-secret",
                     "SSH_AUTH_SOCK": "/tmp/test-agent.sock", "SSH_ASKPASS": "/tmp/test-askpass",
                     "SSH_ASKPASS_REQUIRE": "force"}
        with patch.object(launcher.QtWidgets.QFileDialog, "getOpenFileName",
                          return_value=(str(self.recording), "NOMAD 시험 기록 (*.nomad)")) as chooser, \
                patch.object(launcher.subprocess, "Popen", return_value=fake_process) as popen, \
                patch.object(launcher.socket, "socket", side_effect=AssertionError("Playback opened a network socket")), \
                patch.dict(os.environ, inherited):
            self.window.replay_button.click()
        chooser.assert_called_once()
        self.assertIn("*.nomad", chooser.call_args.args[3])
        popen.assert_called_once()
        args = popen.call_args.args[0]
        self.assertEqual(args, [sys.executable, str(launcher.BASE_DIR / "monitor.py"),
                                "--replay", str(self.recording.resolve())])
        env = popen.call_args.kwargs["env"]
        self.assertTrue(all(key not in env for key in inherited))
        self.assertEqual(env["NOMAD_TRIAL_STATE_DIR"], str(self.window.run_dir))
        self.assertEqual((self.window.run_dir.stat().st_mode & 0o777), 0o700)
        self.assertTrue((self.window.run_dir / "monitor.log").is_file())
        self.assertEqual(self.window.viewer_kind, "replay")
        self.assertFalse(self.window.stop_button.isEnabled())
        self.assertFalse(self.window.replay_button.isEnabled())
        self.window.stop_trial()
        self.window.targets.assert_not_called()
        self.window.begin_process.assert_not_called()
        self.window.save_hosts.assert_not_called()

    def test_replay_works_in_spectator_mode_without_host(self):
        self.window.connection_mode.setCurrentIndex(1)
        self.window.relay_host.clear()
        fake_process = Mock()
        fake_process.poll.return_value = None
        with patch.object(launcher.subprocess, "Popen", return_value=fake_process) as popen:
            self.window.launch_replay(self.recording)
        self.assertEqual(popen.call_args.args[0][-2:], ["--replay", str(self.recording.resolve())])
        self.window.targets.assert_not_called()
        self.window.relay_port_available.assert_not_called()

    def test_cancel_or_missing_file_never_launches_process(self):
        with patch.object(launcher.QtWidgets.QFileDialog, "getOpenFileName", return_value=("", "")), \
                patch.object(launcher.subprocess, "Popen") as popen:
            self.window.open_recording()
            self.window.launch_replay(self.recording.with_name("missing.nomad"))
        popen.assert_not_called()
        self.assertIsNone(self.window.viewer)
        self.assertIn("열지 못했습니다", self.window.status.text())

    def test_viewer_exit_reenables_buttons_and_does_not_stop_boards(self):
        self.window.viewer = Mock(returncode=0)
        self.window.viewer.poll.return_value = 0
        self.window.viewer_kind = "replay"
        self.window.check_viewer()
        self.assertIsNone(self.window.viewer)
        self.assertIsNone(self.window.viewer_kind)
        self.assertTrue(self.window.replay_button.isEnabled())
        self.assertIn("재생 창이 닫혔습니다", self.window.status.text())
        self.window.begin_process.assert_not_called()


if __name__ == "__main__":
    unittest.main()
