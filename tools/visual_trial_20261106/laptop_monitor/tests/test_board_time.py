"""Board time: Pi5 reference + chrony followers. No SSH, sudo, apt or clock change."""
import importlib.util, json, os, subprocess, sys, tempfile, unittest
from pathlib import Path
from unittest import mock

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))
spec = importlib.util.spec_from_file_location('time_agent', BASE / 'board_network/time_agent.py')
agent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agent)
import clock_sync, network_control


def tracking(reference='10.77.0.2', reftime=1000.0, system=0.0004, last=-0.0002, leap='Normal'):
    return {'reference': reference, 'stratum': 9, 'reference_time': reftime, 'system_offset_s': system,
            'last_offset_s': last, 'rms_offset_s': 0.0003, 'leap': leap}


class AgentTests(unittest.TestCase):
    def test_parse_chronyc_csv(self):
        line = '0A4D0002,10.77.0.2,9,1762835000.123,0.000412,-0.000201,0.000300,-1.2,0.01,0.2,0.001,0.0005,16.2,Normal'
        value = agent.parse_tracking(line)
        self.assertEqual(value['reference'], '10.77.0.2')
        self.assertAlmostEqual(value['system_offset_s'], 0.000412)
        self.assertEqual(value['leap'], 'Normal')

    def test_client_within_5ms_passes(self):
        self.assertEqual(agent.evaluate('jetson', tracking(), ['10.77.0.2'], 1010.0), (True, 'following_pi5_within_5ms'))

    def test_client_over_5ms_fails(self):
        self.assertEqual(agent.evaluate('pi4', tracking(reference='192.168.3.156', system=0.006), ['192.168.3.156'], 1010.0)[1],
                         'offset_over_5ms')
        self.assertEqual(agent.evaluate('pi4', tracking(reference='192.168.3.156', last=-0.0051), ['192.168.3.156'], 1010.0)[1],
                         'offset_over_5ms')

    def test_client_following_other_source_or_unsynced_fails(self):
        self.assertEqual(agent.evaluate('jetson', tracking(reference='185.125.190.58'), ['10.77.0.2'], 1010.0)[1],
                         'not_following_pi5')
        self.assertEqual(agent.evaluate('jetson', tracking(leap='Not synchronised'), ['10.77.0.2'], 1010.0)[1],
                         'not_following_pi5')

    def test_stale_update_fails(self):
        self.assertEqual(agent.evaluate('jetson', tracking(), ['10.77.0.2'], 1100.0)[1], 'pi5_update_stale')

    def test_pi5_reference_serves_even_without_internet(self):
        self.assertTrue(agent.evaluate('pi5', tracking(reference='127.127.1.1', leap='Not synchronised'), [], 5000.0)[0])

    def test_server_lists_come_from_cable_and_peers(self):
        with tempfile.TemporaryDirectory() as d:
            peers = Path(d) / 'p.json'
            peers.write_text(json.dumps({'pi4': '192.168.3.142', 'pi5': '192.168.3.156', 'jetson': '192.168.1.152',
                                         'networks': [{'cidr': '10.42.0.0/24', 'pi4': '10.42.0.14', 'pi5': '10.42.0.15',
                                                       'jetson': '10.42.0.16'}]}))
            self.assertEqual(agent.pi5_addresses('jetson', peers), ['10.77.0.2', '192.168.3.156', '10.42.0.15'])
            self.assertEqual(agent.pi5_addresses('pi4', peers), ['192.168.3.156', '10.42.0.15'])
            peers.write_text('{"pi5": "$(touch /x)"}')
            self.assertEqual(agent.pi5_addresses('pi4', peers), [])

    def test_configs_step_only_at_start_and_follow_pi5(self):
        jetson = agent.chrony_config('jetson', ['10.77.0.2', '192.168.3.156'])
        self.assertIn('makestep 0.1 3', jetson)
        self.assertIn('server 10.77.0.2 iburst minpoll 1 maxpoll 4 prefer', jetson)
        self.assertNotIn('pool', jetson)
        pi5 = agent.chrony_config('pi5', [])
        self.assertIn('local stratum 8', pi5)
        self.assertIn('allow 10.0.0.0/8', pi5)
        with self.assertRaises(agent.TimeError):
            agent.chrony_config('pi4', [])

    def test_install_refuses_while_trial_running(self):
        with mock.patch.object(agent.os, 'geteuid', return_value=0), \
                mock.patch.object(agent, 'board_role', return_value='jetson'), \
                mock.patch.object(agent, 'trial_running', return_value=['oak_publisher']), \
                mock.patch.object(agent, 'run') as run:
            with self.assertRaises(agent.TimeError) as error:
                agent.install('/tmp/x')
        self.assertIn('trial_running', str(error.exception))
        run.assert_not_called()

    def test_existing_temporary_cable_profile_is_saved_without_reactivation(self):
        calls = []

        def fake(args, timeout=30, check=True, env_extra=None):
            calls.append(args)
            if args[:5] == ['nmcli', '-g', 'ipv4.addresses', 'connection', 'show']:
                return '10.77.0.1/30'
            return ''
        with mock.patch.object(agent, 'run', side_effect=fake), mock.patch.object(agent, 'wired_state', return_value='fixed_up'):
            self.assertEqual(agent.fix_wired('jetson'), 'fixed_up')
        self.assertTrue(any(c[:3] == ['nmcli', 'connection', 'modify'] for c in calls))
        self.assertFalse(any('up' in c for c in calls))

    def test_cable_profile_with_other_address_is_refused(self):
        with mock.patch.object(agent, 'run', return_value='10.77.0.9/30'):
            with self.assertRaises(agent.TimeError):
                agent.fix_wired('pi5')


class Pi5AddressTests(unittest.TestCase):
    """Classroom DHCP gave Pi5 a new address; the GUI tells the follower."""

    def peers(self, d):
        path = Path(d) / 'p.json'
        path.write_text(json.dumps({'pi4': '192.168.2.252', 'pi5': '192.168.3.156', 'jetson': '192.168.1.152',
                                    'networks': [{'cidr': '10.42.0.0/24', 'pi4': '10.42.0.14', 'pi5': '10.42.0.15',
                                                  'jetson': '10.42.0.16'}]}))
        return path

    def test_store_classroom_and_hotspot_addresses(self):
        with tempfile.TemporaryDirectory() as d:
            path = self.peers(d)
            self.assertEqual(agent.store_pi5_address('192.168.3.164', path), '192.168.3.156')
            value = json.loads(path.read_text())
            self.assertEqual(value['pi5'], '192.168.3.164')
            self.assertEqual(value['pi4'], '192.168.2.252')
            self.assertEqual(value['networks'][0]['pi5'], '10.42.0.15')
            agent.store_pi5_address('10.42.0.33', path)
            value = json.loads(path.read_text())
            self.assertEqual((value['pi5'], value['networks'][0]['pi5']), ('192.168.3.164', '10.42.0.33'))
            self.assertEqual(oct(path.stat().st_mode & 0o777), '0o644')

    def fake_board(self, d, sources):
        calls = []
        conf = Path(d) / 'chrony.conf'
        conf.write_text('old\n')

        def fake(args, timeout=30, check=True, env_extra=None):
            calls.append(args)
            if args[:2] == ['ip', '-4']:
                return ('1: lo    inet 127.0.0.1/8 scope host lo\n'
                        '3: wlan0    inet 192.168.2.252/22 brd 192.168.3.255 scope global wlan0\n'
                        '4: docker0    inet 172.17.0.1/16 scope global docker0')
            if args[:4] == ['chronyc', '-c', '-n', 'sources']:
                return '\n'.join('^,?,{},0,4,0,4294967295,0,0,0'.format(s) for s in sources)
            return ''
        patches = [mock.patch.object(agent.os, 'geteuid', return_value=0),
                   mock.patch.object(agent, 'board_role', return_value='pi4'),
                   mock.patch.object(agent, 'trial_running', return_value=[]),
                   mock.patch.object(agent, 'run', side_effect=fake),
                   mock.patch.object(agent.shutil, 'which', return_value='/usr/bin/chronyc'),
                   mock.patch.object(agent, 'CHRONY_CONF', conf),
                   mock.patch.object(agent, 'PEERS', self.peers(d)),
                   mock.patch.object(agent, 'check', return_value={'ok': True, 'reason': 'following_pi5_within_5ms'})]
        return calls, conf, patches

    def test_update_saves_and_swaps_running_sources_without_restart(self):
        with tempfile.TemporaryDirectory() as d:
            calls, conf, patches = self.fake_board(d, ['192.168.3.156', '10.42.0.15'])
            for p in patches:
                p.start()
            try:
                value = agent.update_pi5('192.168.3.164', wait_s=0)
            finally:
                for p in reversed(patches):
                    p.stop()
            self.assertTrue(value['ok'] and value['updated'])
            self.assertIn('server 192.168.3.164 iburst', conf.read_text())
            self.assertNotIn('192.168.3.156', conf.read_text())
            self.assertIn(['chronyc', 'delete', '192.168.3.156'], calls)
            self.assertIn(['chronyc', 'add', 'server', '192.168.3.164', 'iburst', 'minpoll', '1', 'maxpoll', '4'], calls)
            self.assertFalse(any('restart' in c for c in calls))
            self.assertFalse(any(c[:3] == ['chronyc', 'delete', '10.42.0.15'] for c in calls))

    def test_update_refuses_other_network_and_bad_address(self):
        with tempfile.TemporaryDirectory() as d:
            calls, conf, patches = self.fake_board(d, ['192.168.3.156'])
            for p in patches:
                p.start()
            try:
                for address, reason in (('192.168.9.5', 'pi5_address_not_on_my_network'),
                                        ('8.8.8.8', 'invalid_pi5_address'), ('10.77.0.2', 'invalid_pi5_address'),
                                        ('x; rm', 'invalid_pi5_address')):
                    with self.assertRaises(agent.TimeError) as error:
                        agent.update_pi5(address, wait_s=0)
                    self.assertEqual(str(error.exception), reason)
            finally:
                for p in reversed(patches):
                    p.stop()
            self.assertEqual(conf.read_text(), 'old\n')

    def test_learn_pi5_only_when_board_polls_another_address(self):
        stale = {'ok': False, 'board': 'pi4', 'reason': 'not_following_pi5', 'servers': ['192.168.3.156', '10.42.0.15']}
        fixed = {'ok': True, 'board': 'pi4', 'reason': 'following_pi5_within_5ms', 'updated': True,
                 'pi5_address': '192.168.3.164', 'message': 'Pi5 기준 5ms 이내'}
        with mock.patch.object(network_control, 'time_request', return_value=dict(fixed)) as request:
            value = clock_sync._learn_pi5(stale, 'pi4', 'user@192.168.2.252', 'pw', 'user@192.168.3.164')
        request.assert_called_once_with('pi4', 'user@192.168.2.252', 'pw', 'pi5_address', timeout_s=60,
                                        address='192.168.3.164')
        self.assertTrue(value['ok'])
        self.assertIn('192.168.3.164', value['message'])
        with mock.patch.object(network_control, 'time_request') as request:
            same = dict(stale, servers=['192.168.3.164'])
            self.assertIs(clock_sync._learn_pi5(same, 'pi4', 'user@192.168.2.252', 'pw', 'user@192.168.3.164'), same)
        request.assert_not_called()

    def test_old_board_agent_asks_for_reinstall(self):
        stale = {'ok': False, 'board': 'pi4', 'reason': 'not_following_pi5', 'servers': ['192.168.3.156']}
        with mock.patch.object(network_control, 'time_request',
                               return_value={'ok': False, 'reason': 'invalid_request', 'message': 'x'}):
            value = clock_sync._learn_pi5(stale, 'pi4', 'user@192.168.2.252', 'pw', 'user@192.168.3.164')
        self.assertEqual(value['reason'], 'pi5_update_needs_agent')
        self.assertIn('다시 설치', value['message'])

    def test_time_request_sends_address_to_installed_agent(self):
        calls = []

        def fake(host, password, payload, wrapper, timeout_s):
            calls.append((payload, wrapper))
            return {'out': clock_sync.MARKER + json.dumps({'ok': True, 'feature': clock_sync.FEATURE, 'board': 'pi4',
                                                            'reason': 'following_pi5_within_5ms', 'updated': True}),
                    'err': '', 'returncode': 0}
        with mock.patch.object(network_control, '_board_call', side_effect=fake):
            result = network_control.time_request('pi4', 'user@192.168.2.252', 'pw', 'pi5_address',
                                                  address='192.168.3.164')
        self.assertTrue(result['ok'] and result['updated'])
        payload, wrapper = calls[0]
        self.assertEqual(json.loads(payload.split('\n')[1]),
                         {'board': 'pi4', 'action': 'pi5_address', 'address': '192.168.3.164'})
        self.assertIn('/usr/local/lib/nomad-network/time_agent.py', wrapper)
        self.assertEqual(network_control.time_request('pi4', 'user@x', 'pw', 'pi5_address', address='$(x)')['reason'],
                         'invalid_request')
        self.assertEqual(network_control.time_request('pi5', 'user@x', 'pw', 'pi5_address', address='10.0.0.1')['reason'],
                         'invalid_request')

    def test_planner_connect_passes_gui_pi5_address(self):
        self.assertIn('pi5_host=getattr(self.args, "pi_host", None)', (BASE / 'monitor.py').read_text())
        self.assertIn('"--pi5-host"', (BASE / 'planning_client.py').read_text())



class LaptopTests(unittest.TestCase):
    def fake_ssh(self, stdout, returncode=0):
        return subprocess.CompletedProcess([], returncode, stdout, '')

    def test_check_reports_board_decision_without_sudo(self):
        out = clock_sync.MARKER + json.dumps({'ok': True, 'feature': clock_sync.FEATURE, 'board': 'jetson',
                                               'reason': 'following_pi5_within_5ms', 'offset_ms': 0.41,
                                               'reference': '10.77.0.2', 'wired': 'fixed_up'})
        with tempfile.TemporaryDirectory() as d, mock.patch.dict(os.environ, {'XDG_STATE_HOME': d}), \
                mock.patch.object(clock_sync.subprocess, 'run', return_value=self.fake_ssh(out)) as run:
            result = clock_sync.synchronize_clock('nvidia@192.168.1.152', 'Zq7-secret-Xv')
        self.assertTrue(result['ok'])
        self.assertIn('5ms', result['message'])
        command = run.call_args[0][0][-1]
        self.assertNotIn('sudo', command)
        self.assertNotIn('date', command)
        self.assertNotIn('Zq7-secret-Xv', ' '.join(run.call_args[0][0]))

    def test_not_installed_and_failures_block_start(self):
        out = clock_sync.MARKER + json.dumps({'ok': False, 'feature': clock_sync.FEATURE, 'reason': 'time_sync_not_installed'})
        with tempfile.TemporaryDirectory() as d, mock.patch.dict(os.environ, {'XDG_STATE_HOME': d}), \
                mock.patch.object(clock_sync.subprocess, 'run', return_value=self.fake_ssh(out)):
            result = clock_sync.synchronize_clock('user@192.168.3.142', 'pw')
        self.assertFalse(result['ok'])
        self.assertIn('설치', result['message'])
        self.assertFalse(clock_sync.parse('garbage')['ok'])
        self.assertFalse(clock_sync.parse(clock_sync.MARKER + json.dumps({'ok': True, 'feature': 'other'}))['ok'])

    def test_remote_fallback_text_is_valid_marker_json(self):
        completed = subprocess.run(['bash', '-c', clock_sync.REMOTE.replace(clock_sync.CHECK, '/nonexistent/time_agent.py')],
                                   capture_output=True, text=True)
        self.assertEqual(clock_sync.parse(completed.stdout)['reason'], 'time_sync_not_installed')

    def test_time_install_uploads_agent_with_sudo_wrapper(self):
        calls = []

        def fake(host, password, payload, wrapper, timeout_s):
            calls.append((payload, wrapper))
            return {'out': clock_sync.MARKER + json.dumps({'ok': True, 'feature': clock_sync.FEATURE, 'board': 'pi5',
                                                            'reason': 'reference_serving'}), 'err': '', 'returncode': 0}
        with mock.patch.object(network_control, '_board_call', side_effect=fake):
            result = network_control.time_request('pi5', 'user@192.168.3.156', 'pw', 'install')
        self.assertTrue(result['ok'])
        payload, wrapper = calls[0]
        self.assertEqual(payload.split('\n', 2)[0], 'pw')
        self.assertIn("FEATURE = 'nomad-board-time-20261111-1'", payload)
        self.assertIn('time_agent.py', wrapper)
        self.assertNotIn('wifi_agent.py', wrapper)

    def test_start_scripts_refuse_when_board_not_ready(self):
        for name in ('remote/start_jetson.sh', 'remote/start_pi_vio_fusion.sh'):
            text = (BASE / name).read_text()
            self.assertIn('time_agent.py --check', text)
            self.assertIn('TIME_NOT_READY', text)


if __name__ == '__main__':
    unittest.main()
