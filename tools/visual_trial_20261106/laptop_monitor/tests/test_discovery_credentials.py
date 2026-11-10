"""Public GUI backup must not require a committed board password."""
import ast
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


class DiscoveryCredentialTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_gui_passes_form_password_only_in_child_environment(self):
        with tempfile.TemporaryDirectory(prefix="nomad-discovery-test-") as directory, \
                patch.object(launcher, "STATE_ROOT", Path(directory)), \
                patch.object(launcher.shutil, "which", return_value="/usr/bin/setsid"):
            window = launcher.Launcher()
            try:
                window.password.setText("offline-fixture")
                window.begin_process = Mock()
                window.discover()
                call = window.begin_process.call_args
                self.assertEqual(call.kwargs["environment"]["NOMAD_BOARD_PASSWORD"], "offline-fixture")
                self.assertNotIn("offline-fixture", str(call.args))
                self.assertNotIn("offline-fixture", window.output)
            finally:
                window.close()

    def test_finder_reads_password_from_environment_without_literal_default(self):
        source = launcher.FINDER.read_text()
        python = source.split("<<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]
        tree = ast.parse(python)
        password_assignments = [node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "password" for target in node.targets)]
        self.assertEqual(len(password_assignments), 1)
        self.assertIsInstance(password_assignments[0], ast.Call)
        self.assertIn('os.environ.get("NOMAD_BOARD_PASSWORD"', ast.unparse(password_assignments[0]).replace("'", '"'))


if __name__ == "__main__":
    unittest.main()
