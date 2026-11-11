"""Laptop-side transport: secrets on stdin only, strict result parsing."""
import json, os, tempfile, unittest
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0, str(Path(__file__).parents[1]))
import network_control as n


def response(board='pi4', **extra):
    value = dict(ok=True, feature=n.FEATURE, board=board, machine_id='a' * 32, mode='classroom', ipv4=['192.168.3.142'],
                 interface='wlan0', active_ssid='DX Campus 5G', active_laptop='', home_ssid='DX Campus 5G',
                 watchdog='active', watchdog_seconds=120, pending='', laptops=[{'name': 'su', 'ssid': 'su', 'address': '10.42.0.14/24'}])
    value.update(extra)
    return value


class FakeSSH:
    calls = []
    reply = None

    def __init__(self, args, **kw):
        self.args, self.kw, self.returncode = args, kw, 0
        FakeSSH.calls.append(self)

    def communicate(self, payload=None, timeout=None):
        self.payload = payload
        return n.MARKER + json.dumps(FakeSSH.reply or response()) + '\n', ''


class Tests(unittest.TestCase):
    def setUp(self):
        FakeSSH.reply = None

    def call(self, *args, **kw):
        with tempfile.TemporaryDirectory() as d, patch.dict(os.environ, {'XDG_STATE_HOME': d}), \
                patch.object(n.subprocess, 'Popen', FakeSSH):
            return n.request(*args, **kw), FakeSSH.calls[-1]

    def test_register_secrets_only_on_stdin(self):
        value, call = self.call('pi4', 'user@192.168.3.142', 'board-secret', 'register', name='su', ssid='a; $(touch /x)',
                                password='wifi-secret', gateway='10.42.0.1/24',
                                addresses={'pi4': '10.42.0.14', 'pi5': '10.42.0.15', 'jetson': '10.42.0.16'})
        self.assertTrue(value['ok'])
        self.assertNotIn('wifi-secret', str(call.args))
        self.assertNotIn('board-secret', str(call.args))
        self.assertIn('wifi-secret', call.payload)
        self.assertIn('ControlPath=none', call.args)
        self.assertFalse(Path(call.kw['env']['SSH_ASKPASS']).exists())
        self.assertNotIn('wifi-secret', json.dumps(value))

    def test_install_uploads_agent_source_after_request(self):
        value, call = self.call('jetson', 'nvidia@192.168.1.152', 'board-secret', 'install')
        FakeSSH.reply = None
        lines = call.payload.split('\n', 2)
        self.assertEqual(lines[0], 'board-secret')
        self.assertEqual(json.loads(lines[1])['action'], 'install')
        self.assertIn('FEATURE = ', lines[2])
        self.assertIn('upload', call.args[-1])
        _, status_call = self.call('pi4', 'user@192.168.3.142', 'board-secret', 'status')
        self.assertIn(n.INSTALLED, status_call.args[-1])
        self.assertEqual(status_call.payload.count('\n'), 2)

    def test_password_only_for_register_and_account_checked(self):
        value, _ = self.call('pi4', 'user@192.168.3.142', 'x', 'status', password='leak')
        self.assertEqual(value['reason'], 'invalid_request')
        value = n.request('jetson', 'user@192.168.1.152', 'x', 'status')
        self.assertEqual(value['reason'], 'invalid_board_address')

    def test_wrong_board_or_feature_reply_rejected(self):
        FakeSSH.reply = response(board='pi5')
        value, _ = self.call('pi4', 'user@192.168.3.142', 'x', 'status')
        self.assertFalse(value['ok'])
        FakeSSH.reply = dict(response(), feature='other')
        value, _ = self.call('pi4', 'user@192.168.3.142', 'x', 'status')
        self.assertFalse(value['ok'])

    def test_board_addresses_follow_gateway_network(self):
        self.assertEqual(n.board_addresses('10.42.0.1/24'), {'pi4': '10.42.0.14', 'pi5': '10.42.0.15', 'jetson': '10.42.0.16'})
        self.assertEqual(n.board_addresses('192.168.12.1/24')['jetson'], '192.168.12.16')
        with self.assertRaises(ValueError):
            n.board_addresses('10.42.0.1/31')

    def test_local_hotspot_listing_uses_shared_default_gateway(self):
        outputs = {
            ('-t', '-f', 'NAME,UUID,TYPE,ACTIVE', 'connection', 'show'):
                'DX Campus 2.4G:aaaaaaaa-0000-0000-0000-000000000001:802-11-wireless:yes\n'
                'Hotspot:aaaaaaaa-0000-0000-0000-000000000002:802-11-wireless:no',
        }

        def fake(*args, timeout=20):
            if args in outputs:
                return outputs[args]
            if args[0] == '-g' and args[-1].endswith('01'):
                return 'infrastructure\nDX Campus 2.4G\nauto\n\n'
            if args[0] == '-g' and args[-1].endswith('02'):
                return 'ap\nsu\nshared\n\n'
            raise AssertionError(args)
        with patch.object(n, '_nmcli', side_effect=fake):
            items = n.local_hotspots()
        self.assertEqual([i['name'] for i in items], ['Hotspot'])
        self.assertEqual(items[0]['gateway'], '10.42.0.1/24')
        self.assertFalse(items[0]['active'])


if __name__ == '__main__':
    unittest.main()
