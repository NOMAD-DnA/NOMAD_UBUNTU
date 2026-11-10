"""Bounded laptop log transport checks; no SSH or board process is executed."""
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from module_log_client import MAX_EVENTS, MAX_LINE, ModuleLogClient


def event(seq=1, **changes):
    return {"kind": "log", "board": "pi", "source": "localization", "t": 1794190000.,
            "session": "fixture", "seq": seq, "text": "[vio] existing stdout", **changes}


class PipeProcess:
    """Fake Popen with real pipe framing but no external executable."""
    def __init__(self, body):
        read_fd, write_fd = os.pipe()
        self.stdout = os.fdopen(read_fd, "rb", buffering=0)
        self.stdin = io.BytesIO()
        self.returncode = None
        self.actions = []
        def write():
            with os.fdopen(write_fd, "wb", buffering=0) as output:
                for offset in range(0, len(body), 173):
                    output.write(body[offset:offset+173])
        self.writer = threading.Thread(target=write, daemon=True)
        self.writer.start()

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.returncode = 0
        return 0

    def terminate(self):
        self.actions.append("terminate")

    def kill(self):
        self.actions.append("kill")


class ModuleLogClientTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.client = ModuleLogClient("pi", "user@192.0.2.1", self.temporary.name)

    def tearDown(self):
        self.client.stop()
        self.temporary.cleanup()

    def test_buffer_is_bounded_before_gui_and_arrival_metadata_is_preserved(self):
        for seq in range(160):
            self.assertTrue(self.client._accept_line(json.dumps(event(seq)), 123. + seq))
        output = self.client.drain()
        self.assertEqual(len(output), MAX_EVENTS)
        self.assertEqual(output[0][1]["seq"], 160-MAX_EVENTS)
        self.assertEqual(output[-1][1]["_local_received_monotonic"], 282.)
        self.assertTrue(all(board == "pi" for board, _ in output))
        self.assertEqual(self.client.dropped_events, 160-MAX_EVENTS)
        self.assertEqual(self.client.drain(), [])
        for bad in (event(board="pi4"), event(source="unrecognized"), event(seq=True),
                    event(text="x"*4097), event(session=""), event(t=float("nan"))):
            self.assertFalse(self.client._accept_line(json.dumps(bad)))
        self.assertEqual(self.client.invalid_events, 6)

    def test_fragmented_and_oversize_lines_cannot_accumulate_unbounded_state(self):
        body = b"x"*(MAX_LINE+2000) + b"\ninvalid-json\n" + json.dumps(event(7)).encode() + b"\n"
        process = PipeProcess(body)
        before = time.monotonic()
        try:
            self.client._read_output(process)
            output = self.client.drain()
            self.assertEqual(len(output), 1)
            self.assertEqual(output[0][1]["seq"], 7)
            self.assertGreaterEqual(output[0][1]["_local_received_monotonic"], before)
            self.assertEqual(self.client.invalid_events, 2)
        finally:
            self.client._end_process(process)
            process.writer.join(timeout=1.)

    def test_readonly_command_temp_askpass_and_stdin_lifetime(self):
        captured = {}
        def fake_popen(args, **kwargs):
            captured["args"], captured["kwargs"] = args, kwargs
            captured["askpass"] = kwargs["env"]["SSH_ASKPASS"]
            captured["script"] = Path(captured["askpass"]).read_text()
            captured["process"] = PipeProcess(json.dumps(event()).encode() + b"\n")
            return captured["process"]
        accept = self.client._accept_line
        def receive(*args):
            self.assertFalse(captured["process"].stdin.closed)
            return accept(*args)
        with patch.dict(os.environ, {"NOMAD_BOARD_PASSWORD1": "fixture-secret"}), \
                patch("module_log_client.subprocess.Popen", side_effect=fake_popen), \
                patch.object(self.client, "_accept_line", side_effect=receive):
            self.client._session()
        self.assertIn("nice -n 15", captured["args"][-1])
        self.assertIn("module_log_tail", captured["args"][-1])
        self.assertIn("--board pi", captured["args"][-1])
        self.assertNotIn("fixture-secret", " ".join(captured["args"]))
        self.assertNotIn("fixture-secret", captured["script"])
        self.assertFalse(Path(captured["askpass"]).exists())
        self.assertEqual(captured["kwargs"]["stderr"], subprocess.DEVNULL)
        self.assertTrue(captured["process"].stdin.closed)
        self.assertEqual(captured["process"].actions, [])
        self.assertEqual(len(self.client.drain()), 1)

    def test_invalid_host_and_board_rejected_before_process_creation(self):
        with self.assertRaises(ValueError):
            ModuleLogClient("pi", "user@host;touch-x", self.temporary.name)
        with self.assertRaises(ValueError):
            ModuleLogClient("unknown", "user@192.0.2.1", self.temporary.name)


if __name__ == "__main__":
    unittest.main()
