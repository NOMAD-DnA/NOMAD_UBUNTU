"""Exercise the real clock shell protocol without SSH, sudo, or clock changes."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

BASE = Path(os.environ.get('NOMAD_GUI_TEST_BASE', Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(BASE))
import clock_sync


PASSWORD = 'synthetic-only-password'

FAKE_SUDO = r'''import os, pathlib, sys
args = sys.argv[1:]
cache = pathlib.Path(os.environ['NOMAD_TEST_SUDO_CACHE'])
trace = pathlib.Path(os.environ['NOMAD_TEST_TRACE'])
def note(value):
    with trace.open('a') as output:
        output.write(value + '\n')
def deny(message='a password is required'):
    print('sudo: ' + message, file=sys.stderr, flush=True)
    sys.exit(1)
if '-v' in args:
    note('auth')
    if '-S' in args:
        supplied = sys.stdin.readline().rstrip('\n')
        if supplied != 'synthetic-only-password' or os.environ.get('NOMAD_TEST_AUTH_FAIL') == '1':
            deny('1 incorrect password attempt')
    elif not cache.exists():
        deny()
    # Model the documented sudo -k rule: validation does not update credentials.
    if '-k' in args:
        cache.unlink(missing_ok=True)
    else:
        cache.touch()
    sys.exit(0)
note('set-attempt')
if '-n' not in args or not cache.exists():
    deny()
# Only our adjacent fake date is executable through this sudo substitute.
if args[:2] != ['-n', 'date']:
    raise SystemExit('unexpected fake sudo command')
date = pathlib.Path(__file__).with_name('date')
os.execv(str(date), [str(date), *args[2:]])
'''

FAKE_DATE = r'''import os, pathlib, sys, time
trace = pathlib.Path(os.environ['NOMAD_TEST_TRACE'])
setting = any(arg.startswith('--set=') for arg in sys.argv[1:])
with trace.open('a') as output:
    output.write(('fake-set' if setting else 'sample') + '\n')
now = time.time_ns()
label = 'CLOCK_SET' if setting else 'CLOCK_SAMPLE'
print(f'{label}={now//10**9}.{now%10**9:09d}', flush=True)
'''


class ClockShellProtocolTest(unittest.TestCase):
    def run_protocol(self, *, auth_fail=False, force_legacy_k=False):
        original_popen = subprocess.Popen
        with tempfile.TemporaryDirectory(prefix='nomad-clock-protocol-') as temporary:
            directory = Path(temporary)
            for name, source in (('sudo', FAKE_SUDO), ('date', FAKE_DATE)):
                executable = directory / name
                executable.write_text('#!' + sys.executable + '\n' + source)
                executable.chmod(0o700)

            def local_peer(args, **kwargs):
                self.assertEqual(args[0], 'ssh')
                shell = shlex.split(args[-1])
                self.assertEqual(shell[:2], ['bash', '-c'])
                script = shell[2]
                model_line = "model=$(tr -d '\\0' < /proc/device-tree/model)"
                self.assertEqual(script.count(model_line), 1)
                script = script.replace(model_line, "model='Raspberry Pi 5 Model B test peer'", 1)
                if force_legacy_k:
                    self.assertIn('sudo -S -p', script)
                    script = script.replace('sudo -S -p', 'sudo -k -S -p', 1)
                env = kwargs['env'].copy()
                # No real external program is reachable from the remote script.
                env.update(PATH=str(directory), NOMAD_TEST_SUDO_CACHE=str(directory / 'sudo-cache'),
                           NOMAD_TEST_TRACE=str(directory / 'trace'),
                           NOMAD_TEST_AUTH_FAIL='1' if auth_fail else '0')
                kwargs['env'] = env
                return original_popen(['/bin/bash', '-c', script], **kwargs)

            with mock.patch.dict(os.environ, XDG_STATE_HOME=str(directory / 'state')):
                with mock.patch.object(clock_sync.subprocess, 'Popen', side_effect=local_peer):
                    result = clock_sync.synchronize_clock('user@127.0.0.1', PASSWORD, timeout_s=3)
            trace = (directory / 'trace').read_text().splitlines()
        self.assertNotIn(PASSWORD, json.dumps(result))
        return result, trace

    def test_real_remote_protocol_authenticates_sets_and_probes(self):
        result, trace = self.run_protocol()
        self.assertTrue(result['ok'], result)
        self.assertEqual(trace, ['auth', 'set-attempt', 'fake-set', 'sample'])
        self.assertLess(result['probe_round_trip_s'], .3)

    def test_legacy_k_reproduces_cache_failure_without_reporting_ssh_failure(self):
        result, trace = self.run_protocol(force_legacy_k=True)
        self.assertFalse(result['ok'], result)
        self.assertEqual(result['reason'], 'clock_privilege_required')
        self.assertEqual(result.get('phase'), 'clock_set')
        self.assertEqual(result.get('returncode'), 1)
        self.assertEqual(trace, ['auth', 'set-attempt'])

    def test_authentication_failure_never_attempts_clock_set(self):
        result, trace = self.run_protocol(auth_fail=True)
        self.assertFalse(result['ok'], result)
        self.assertEqual(result['reason'], 'clock_privilege_authentication_failed')
        self.assertEqual(result.get('phase'), 'ssh_ready')
        self.assertEqual(result.get('returncode'), 1)
        self.assertEqual(trace, ['auth'])


if __name__ == '__main__':
    unittest.main()
