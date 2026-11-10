import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))
import clock_sync


class ClockSyncTest(unittest.TestCase):
    def assess(self, offset_ns=0, elapsed_ns=20_000_000, wall_jump_ns=0):
        start = 1_794_293_000_000_000_000
        remote = start + elapsed_ns // 2 + offset_ns
        return clock_sync.assess_clock_sample(
            f'{remote//10**9}.{remote%10**9:09d}', start, 20_000_000_000,
            start + elapsed_ns + wall_jump_ns, 20_000_000_000 + elapsed_ns)

    def test_short_probe_accepts_synchronized_clock(self):
        self.assertTrue(self.assess()['ok'])

    def test_observed_future_and_lag_offsets_are_rejected(self):
        for value in (235_000_000, -235_000_000):
            with self.subTest(value=value):
                self.assertEqual(self.assess(value)['reason'], 'clock_skew_or_probe_delay')

    def test_long_probe_cannot_claim_clock_accuracy(self):
        self.assertFalse(self.assess(elapsed_ns=400_000_000)['ok'])

    def test_operator_clock_step_is_reported(self):
        self.assertEqual(self.assess(wall_jump_ns=-100_000_000)['reason'], 'operator_clock_changed')

    def test_boundary_combines_skew_with_network_uncertainty(self):
        self.assertTrue(self.assess(140_000_000)['ok'])
        self.assertFalse(self.assess(140_000_001)['ok'])

    def test_invalid_sample_is_not_success(self):
        for sample in ('bad', '123.5', '-1.000000000', '123.00000000x'):
            self.assertFalse(clock_sync.assess_clock_sample(sample, 1, 1, 2, 2)['ok'])

    def test_authentication_precedes_clock_reference(self):
        # Run a local peer with deliberately slow authentication and mock time.
        # It cannot execute ssh, sudo or date and never changes any system clock.
        real_popen = subprocess.Popen
        peer = '''import sys,time
sys.stdin.readline()
time.sleep(.20)
print('NOMAD_CLOCK_READY',flush=True)
sys.stdin.readline()
print('NOMAD_CLOCK_SAMPLE_READY',flush=True)
assert sys.stdin.readline().strip()=='NOMAD_CLOCK_PROBE'
n=time.time_ns()
print(f'CLOCK_SAMPLE={n//10**9}.{n%10**9:09d}',flush=True)
'''
        def launch(_args, **kwargs):
            return real_popen([sys.executable, '-c', peer], **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.dict(os.environ, XDG_STATE_HOME=directory):
                with mock.patch.object(clock_sync.subprocess, 'Popen', side_effect=launch):
                    result = clock_sync.synchronize_clock('user@127.0.0.1', '', timeout_s=2)
        self.assertTrue(result['ok'], result)
        self.assertLess(result['probe_round_trip_s'], .2)


if __name__ == '__main__':
    unittest.main()
