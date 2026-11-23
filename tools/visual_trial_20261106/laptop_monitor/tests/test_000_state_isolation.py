"""2026-11-16: keep the test suite away from the operator's real GUI state.

Some tests call startup.start_boards(), which records board roles through
board_address.register() into $XDG_STATE_HOME/nomad-trial/board_routes.json. Without
isolation the fake test addresses (192.168.1.5/6) replaced the real roles on the laptop,
which disabled automatic board re-discovery. unittest discovery imports test modules in
name order before running any test, so this module (sorted first) points XDG_STATE_HOME
at a private temporary folder for the whole run. Tests that set their own XDG_STATE_HOME
keep doing so.
"""
import atexit
import os
from pathlib import Path
import shutil
import tempfile
import unittest

_REAL_STATE = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state")))
_TEST_STATE = tempfile.mkdtemp(prefix="nomad-gui-test-state-")
os.environ["XDG_STATE_HOME"] = _TEST_STATE
atexit.register(shutil.rmtree, _TEST_STATE, True)


class StateIsolationTest(unittest.TestCase):
    def test_state_points_to_private_temp(self):
        import board_address  # noqa: E402  (imported after the environment is set)
        state = Path(os.environ["XDG_STATE_HOME"]).resolve()
        self.assertNotEqual(state, _REAL_STATE.resolve())
        self.assertTrue(str(board_address._path()).startswith(str(state)))


if __name__ == "__main__":
    unittest.main()
