"""Board laptop Wi-Fi agent with a fake NetworkManager/systemd; no host network mutation."""
import importlib.util, json, tempfile, unittest, uuid
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('wifi_agent', Path(__file__).parents[1] / 'board_network/wifi_agent.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
HOME = 'b4c33174-7544-483d-a131-1dc50da24bca'
OLD = '11111111-2222-3333-4444-555555555555'
LAPTOP = str(uuid.uuid5(m.NAMESPACE, 'su-laptop'))


class FakeNM:
    def __init__(self):
        self.calls = []
        self.active = HOME
        self.state = '100 (connected)'
        self.conns = {HOME: ('DX Campus 5G', 'DX Campus 5G'), OLD: ('nomad-hotspot-8064-wlan0', '8064')}
        self.visible = ['DX Campus 5G']

    def __call__(self, *args, timeout=15):
        a = list(args)
        self.calls.append(a)
        if a == ['-t', '-f', 'DEVICE,TYPE', 'device', 'status']:
            return 'wlan0:wifi\neth0:ethernet'
        if a[:2] == ['-g', 'GENERAL.CON-UUID']:
            return self.active
        if a[:2] == ['-g', 'GENERAL.STATE']:
            return self.state
        if a[:2] == ['-g', 'IP4.ADDRESS']:
            return '10.42.0.14/24' if self.active == LAPTOP else '192.168.3.142/22' if self.active else ''
        if a == ['-t', '-f', 'NAME,UUID,TYPE', 'connection', 'show']:
            return '\n'.join('{}:{}:802-11-wireless'.format(n, u) for u, (n, _) in self.conns.items())
        if a[:2] == ['-g', '802-11-wireless.ssid']:
            return self.conns.get(a[-1], ('', ''))[1]
        if a[:2] == ['-g', 'ipv4.addresses']:
            return '10.42.0.14/24' if a[-1] == LAPTOP else ''
        if a[:2] == ['connection', 'load']:
            self.conns[LAPTOP] = ('nomad-laptop-su-laptop', 'su')
            return ''
        if a[:3] == ['connection', 'delete', 'uuid']:
            self.conns.pop(a[3], None)
            return ''
        if a[:3] == ['connection', 'down', 'uuid']:
            self.active = ''
            self.state = '30 (disconnected)'
            return ''
        if a[:4] == ['--wait', '45', 'connection', 'up']:
            self.active = a[5]
            self.state = '100 (connected)'
            return ''
        if a[:2] == ['-g', 'SSID']:
            return '\n'.join(self.visible)
        raise AssertionError('unexpected nmcli ' + repr(a))


class Runner:
    def __init__(self):
        self.calls = []

    def __call__(self, args, timeout=15):
        self.calls.append(args)
        if args[:2] == ['systemctl', 'is-active']:
            return 'active'
        if args[:2] == ['systemctl', 'list-units']:
            return 'nomad-gui-wifi-guard-0123456789ab.timer loaded active waiting x'
        return ''


class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = Path(self.tmp.name)
        self.nm, self.runner = FakeNM(), Runner()
        self.agent = m.Agent(base=d / 'state', config=d / 'etc/laptop-wifi.json', peers=d / 'etc/network-peers.json',
                             profiles=d / 'profiles', nmcli=self.nm, runner=self.runner)
        (d / 'etc').mkdir()
        (d / 'etc/network-peers.json').write_text(json.dumps({'pi4': '192.168.3.142', 'pi5': '192.168.3.156', 'jetson': '192.168.1.152'}))
        self.addresses = {'pi4': '10.42.0.14', 'pi5': '10.42.0.15', 'jetson': '10.42.0.16'}

    def tearDown(self):
        self.tmp.cleanup()

    def register(self, **kw):
        values = dict(board='pi4', name='su-laptop', ssid='su', password='wifi-secret-1', gateway='10.42.0.1/24',
                      addresses=self.addresses, seconds=None)
        values.update(kw)
        return self.agent.register(**values)

    def test_register_writes_fixed_address_unlimited_retry_lower_priority_profile(self):
        self.register()
        text = (Path(self.tmp.name) / 'profiles/nomad-laptop-su-laptop.nmconnection').read_text()
        self.assertIn('autoconnect-retries=0', text)
        self.assertIn('autoconnect-priority=-10', text)
        self.assertIn('address1=10.42.0.14/24,10.42.0.1', text)
        self.assertIn('psk=wifi-secret-1', text)
        self.assertEqual(oct((Path(self.tmp.name) / 'profiles/nomad-laptop-su-laptop.nmconnection').stat().st_mode & 0o777), '0o600')
        config = json.loads((Path(self.tmp.name) / 'etc/laptop-wifi.json').read_text())
        self.assertEqual(config['home_uuid'], HOME)
        self.assertEqual(config['home_ssid'], 'DX Campus 5G')
        # current connection untouched
        self.assertEqual(self.nm.active, HOME)
        self.assertFalse(any(c[:2] == ['connection', 'up'] or c[:3] == ['--wait', '45', 'connection'] for c in self.nm.calls))

    def test_register_keeps_classroom_peer_and_adds_network(self):
        self.register()
        peers = json.loads((Path(self.tmp.name) / 'etc/network-peers.json').read_text())
        self.assertEqual(peers['pi4'], '192.168.3.142')
        self.assertEqual(peers['networks'], [dict(cidr='10.42.0.0/24', **self.addresses)])
        self.register()
        self.assertEqual(len(json.loads((Path(self.tmp.name) / 'etc/network-peers.json').read_text())['networks']), 1)

    def test_secret_never_in_argv(self):
        self.register()
        self.assertNotIn('wifi-secret-1', json.dumps(self.nm.calls + self.runner.calls))

    def test_invalid_inputs_rejected_before_any_change(self):
        for kw, reason in ((dict(name='bad name'), 'invalid_laptop_name'), (dict(password='short'), 'invalid_wifi_password'),
                           (dict(addresses={'pi4': '10.42.0.1', 'pi5': '10.42.0.15', 'jetson': '10.42.0.16'}), 'invalid_laptop_addresses'),
                           (dict(addresses={'pi4': '10.43.0.14', 'pi5': '10.42.0.15', 'jetson': '10.42.0.16'}), 'invalid_laptop_addresses'),
                           (dict(seconds=5), 'invalid_timeout')):
            with self.assertRaises(m.NetworkError) as error:
                self.register(**kw)
            self.assertEqual(str(error.exception), reason)
        self.assertFalse((Path(self.tmp.name) / 'profiles/nomad-laptop-su-laptop.nmconnection').exists())

    def test_register_away_from_classroom_without_record_fails(self):
        self.nm.active = ''
        with self.assertRaises(m.NetworkError) as error:
            self.register()
        self.assertEqual(str(error.exception), 'home_wifi_unknown')

    def test_indoor_requires_laptop_and_classroom_then_schedules(self):
        with self.assertRaises(m.NetworkError):
            self.agent.schedule('_indoor')  # no home recorded yet
        self.register()
        self.agent.schedule('_indoor')
        run = [c for c in self.runner.calls if c[0] == 'systemd-run'][-1]
        self.assertIn('--on-active=8s', run)
        self.assertEqual(run[-1], '_indoor')
        self.assertEqual(self.agent.snapshot()['pending'], 'indoor')

    def test_indoor_execution_deactivates_classroom_only(self):
        self.register()
        self.agent.run_scheduled('_indoor')
        self.assertIn(['connection', 'down', 'uuid', HOME], self.nm.calls)
        self.assertEqual(self.agent.snapshot()['pending'], '')

    def test_classroom_execution_reconnects_home(self):
        self.register()
        self.nm.active = LAPTOP
        self.agent.run_scheduled('_classroom')
        self.assertEqual(self.nm.active, HOME)

    def test_watchdog_waits_then_restores_only_when_classroom_visible(self):
        self.register(seconds=60)
        self.nm.active, self.nm.state = '', '30 (disconnected)'
        since = self.agent.watchdog_step(None, 1000.0)
        self.assertEqual(since, 1000.0)
        self.assertEqual(self.agent.watchdog_step(since, 1059.0), 1000.0)
        self.assertEqual(self.nm.active, '')
        self.nm.visible = ['su']  # outdoors: classroom not visible
        self.assertEqual(self.agent.watchdog_step(since, 1061.0), 1061.0)
        self.assertEqual(self.nm.active, '')
        self.nm.visible = ['DX Campus 5G']
        self.agent.watchdog_step(1061.0, 1122.0)
        self.assertEqual(self.nm.active, HOME)
        self.assertIsNone(self.agent.watchdog_step(1122.0, 1123.0))

    def test_watchdog_ignores_connected_laptop(self):
        self.register()
        self.nm.active = LAPTOP
        self.assertIsNone(self.agent.watchdog_step(None, 5000.0))

    def test_remove_refuses_active_laptop(self):
        self.register()
        self.nm.active = LAPTOP
        with self.assertRaises(m.NetworkError):
            self.agent.remove('su-laptop')
        self.nm.active = HOME
        self.agent.remove('su-laptop')
        self.assertNotIn(LAPTOP, self.nm.conns)

    def test_cleanup_removes_old_profiles_and_timers_not_active(self):
        removed = self.agent.cleanup()
        self.assertIn('nomad-hotspot-8064-wlan0', removed)
        self.assertNotIn(OLD, self.nm.conns)
        self.assertIn(HOME, self.nm.conns)
        self.assertIn(['systemctl', 'stop', 'nomad-gui-wifi-guard-0123456789ab.timer'], self.runner.calls)

    def test_classroom_peers_require_current_address(self):
        self.register()
        with self.assertRaises(m.NetworkError):
            self.agent.save_classroom_peers('pi4', {'pi4': '192.168.3.99', 'pi5': '192.168.3.156', 'jetson': '192.168.1.152'})
        self.agent.save_classroom_peers('pi4', {'pi4': '192.168.3.142', 'pi5': '192.168.3.150', 'jetson': '192.168.1.152'})
        peers = json.loads((Path(self.tmp.name) / 'etc/network-peers.json').read_text())
        self.assertEqual(peers['pi5'], '192.168.3.150')
        self.assertEqual(len(peers['networks']), 1)

    def test_handle_rejects_wrong_board(self):
        with patch.object(m, 'board_identity', return_value=('pi5', 'a' * 32)):
            with self.assertRaises(m.NetworkError):
                m.handle({'action': 'status', 'board': 'pi4'}, self.agent)
            value = m.handle({'action': 'status', 'board': 'pi5'}, self.agent)
        self.assertEqual(value['mode'], 'classroom')
        self.assertEqual(value['feature'], m.FEATURE)


if __name__ == '__main__':
    unittest.main()
