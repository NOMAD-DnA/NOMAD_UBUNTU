"""2026-11-12: Pi5 takes the GUI laptop clock only without internet time; followers step once."""
import importlib.util, json, os, stat, subprocess, sys, tempfile, time, unittest
from pathlib import Path
from unittest import mock

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))
spec = importlib.util.spec_from_file_location('time_agent_l', BASE / 'board_network/time_agent.py')
agent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agent)
import clock_sync, network_control


def tracking(reference='127.127.1.1', stratum=8, leap='Normal'):
    return {'reference': reference, 'stratum': stratum, 'reference_time': 1.0, 'system_offset_s': 0.0,
            'last_offset_s': 0.0, 'rms_offset_s': 0.0, 'leap': leap}


class AgentTests(unittest.TestCase):
    def test_pi5_config_has_fixed_address_fallbacks(self):
        text = agent.chrony_config('pi5', [])
        for address in agent.FALLBACK_NTP:
            self.assertIn('server {} iburst'.format(address), text)
        self.assertIn('pool ntp.ubuntu.com', text)
        self.assertIn('local stratum 8', text)

    def test_internet_synced_only_for_real_upstream(self):
        self.assertFalse(agent.internet_synced('pi5', tracking()))
        self.assertFalse(agent.internet_synced('pi5', tracking(reference='', stratum=0, leap='Not synchronised')))
        self.assertTrue(agent.internet_synced('pi5', tracking(reference='185.125.190.58', stratum=3)))
        self.assertIsNone(agent.internet_synced('pi4', tracking()))

    def _set(self, laptop, received, upstream=False):
        calls = []
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'time.txt'
            path.write_text('{} {}\n'.format(laptop, received))
            def fake_run(args, timeout=30, check=True, env_extra=None):
                calls.append(args)
                if args[:2] == ['chronyc', '-c']:
                    t = tracking('185.125.190.58', 3) if upstream else tracking()
                    return '7F7F0101,{},{},1.0,0,0,0,0,0,0,0,0,0,{}'.format(t['reference'], t['stratum'], t['leap'])
                return ''
            with mock.patch.object(agent.os, 'geteuid', return_value=0), \
                    mock.patch.object(agent, 'board_role', return_value='pi5'), \
                    mock.patch.object(agent, 'trial_running', return_value=[]), \
                    mock.patch.object(agent, 'run', side_effect=fake_run), \
                    mock.patch.object(agent, 'check', return_value={'ok': True, 'board': 'pi5'}), \
                    mock.patch.object(agent.time, 'time', return_value=1000.0):
                value = agent.set_time_from_laptop(path)
        return value, calls

    def test_laptop_time_sets_pi5_clock_with_receipt_offset(self):
        value, calls = self._set(laptop=1794410000.0, received=1794225000.0)  # board 185000 s behind
        self.assertTrue(value['time_set'])
        self.assertIn(['date', '-u', '-s', '@{:.3f}'.format(1000.0 + 185000.0)], calls)

    def test_no_change_within_2s_or_when_internet_synced(self):
        value, calls = self._set(laptop=1794410000.0, received=1794409999.0)
        self.assertFalse(value['time_set'])
        self.assertFalse(any(c[:1] == ['date'] for c in calls))
        value, calls = self._set(laptop=1794410000.0, received=1794225000.0, upstream=True)
        self.assertEqual(value['time_set_reason'], 'reference_internet_synced')
        self.assertFalse(any(c[:1] == ['date'] for c in calls))

    def test_old_laptop_clock_rejected(self):
        with self.assertRaises(agent.TimeError):
            self._set(laptop=1700000000.0, received=1794225000.0)

    def test_follower_step_runs_makestep_and_refuses_during_trial(self):
        calls = []
        with mock.patch.object(agent.os, 'geteuid', return_value=0), \
                mock.patch.object(agent, 'board_role', return_value='jetson'), \
                mock.patch.object(agent, 'trial_running', return_value=[]), \
                mock.patch.object(agent, 'run', side_effect=lambda a, **k: calls.append(a) or ''), \
                mock.patch.object(agent, 'check', return_value={'ok': True}):
            self.assertTrue(agent.step_follower(wait_s=0)['stepped'])
        self.assertIn(['chronyc', 'makestep'], calls)
        with mock.patch.object(agent.os, 'geteuid', return_value=0), \
                mock.patch.object(agent, 'board_role', return_value='pi4'), \
                mock.patch.object(agent, 'trial_running', return_value=['planner_node']), \
                mock.patch.object(agent, 'run') as run:
            with self.assertRaises(agent.TimeError):
                agent.step_follower()
            run.assert_not_called()


class GuiTests(unittest.TestCase):
    def test_repair_decisions(self):
        pi5 = {'ok': True, 'board': 'pi5', 'internet_synced': False, 'board_utc': 1794225000.0}
        with mock.patch.object(network_control, 'time_set_from_laptop',
                               return_value={'ok': True, 'board': 'pi5', 'time_set': True, 'laptop_offset_s': 185000.0,
                                             'message': 'x'}) as setter:
            value = clock_sync.repair_time(pi5, 'user@10.0.0.5', 'pw', laptop_now=1794410000.0)
        setter.assert_called_once()
        self.assertIn('노트북 시각으로 Pi5 보정', value['message'])
        with mock.patch.object(network_control, 'time_set_from_laptop') as setter:
            clock_sync.repair_time(dict(pi5, internet_synced=True), 'user@10.0.0.5', 'pw', laptop_now=1794410000.0)
            clock_sync.repair_time(dict(pi5, board_utc=1794409999.5), 'user@10.0.0.5', 'pw', laptop_now=1794410000.0)
            clock_sync.repair_time(pi5, 'user@10.0.0.5', '', laptop_now=1794410000.0)
        setter.assert_not_called()
        far = {'ok': False, 'board': 'jetson', 'reason': 'offset_over_5ms', 'offset_ms': 185000000.0}
        with mock.patch.object(network_control, 'time_request', return_value={'ok': True, 'stepped': True, 'message': 'm'}) as req:
            value = clock_sync.repair_time(far, 'nvidia@10.0.0.6', 'pw')
        req.assert_called_once_with('jetson', 'nvidia@10.0.0.6', 'pw', 'step', timeout_s=40)
        self.assertTrue(value['ok'])
        with mock.patch.object(network_control, 'time_request') as req:
            clock_sync.repair_time(dict(far, offset_ms=6.0), 'nvidia@10.0.0.6', 'pw')
        req.assert_not_called()

    def test_ready_handshake_sends_laptop_time_after_board_is_ready(self):
        """Runs the real wrapper locally with fake ssh/sudo and a fake agent."""
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            agent_py = d / 'agent.py'
            agent_py.write_text('import json,sys\nfrom pathlib import Path\n'
                                'p=Path(sys.argv[2]);laptop,board=Path(p.parent/"time.txt").read_text().split()\n'
                                'print("NOMAD_TIME_RESULT="+json.dumps({"ok":True,"feature":"%s","board":"pi5",'
                                '"reason":"reference_serving_local_clock","time_set":True,"laptop_offset_s":float(laptop)-float(board)}))\n'
                                % clock_sync.FEATURE)
            sudo = d / 'sudo'
            sudo.write_text('#!/bin/sh\ncat >/dev/null &\nwhile [ "$1" != python3 ]; do shift; done\nexec "$@"\n')
            sudo.chmod(sudo.stat().st_mode | stat.S_IEXEC)
            real_popen = subprocess.Popen

            def fake_popen(args, **kwargs):
                kwargs['env'] = dict(kwargs.get('env', os.environ), PATH=str(d) + os.pathsep + os.environ['PATH'])
                return real_popen(['bash', '-c', args[-1][len('bash -c '):] if False else args[-1]], **kwargs)
            with mock.patch.object(network_control, 'TIME_INSTALLED', str(agent_py)), \
                    mock.patch.object(network_control.subprocess, 'Popen', side_effect=fake_popen), \
                    mock.patch.dict(os.environ, {'XDG_STATE_HOME': str(d)}):
                result = network_control.time_set_from_laptop('user@10.0.0.5', 'pw', timeout_s=15)
        self.assertTrue(result['ok'], result)
        self.assertTrue(result['time_set'])
        self.assertLess(abs(result['laptop_offset_s']), 2.0)  # same machine: laptop vs "board" receipt ~0

    def test_startup_checks_pi5_before_jetson(self):
        text = (BASE / 'startup.py').read_text()
        self.assertLess(text.index('clocks = {"pi5": synchronize_clock'), text.index('clocks["jetson"] = synchronize_clock'))


if __name__ == '__main__':
    unittest.main()
