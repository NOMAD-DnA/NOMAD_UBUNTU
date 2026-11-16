import json, os, sys, tempfile, unittest
from pathlib import Path
from unittest.mock import Mock, patch
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).parents[1]))
from PyQt5 import QtWidgets
import launcher, network_dialog as d

HOTSPOT = {'name': 'Hotspot', 'uuid': 'aaaaaaaa-0000-0000-0000-000000000002', 'ssid': 'su', 'active': False,
           'gateway': '10.42.0.1/24', 'band': 'auto'}


def ok(board, mode='classroom', ipv4=None):
    return {'ok': True, 'board': board, 'mode': mode, 'ipv4': ipv4 or [], 'laptops': [], 'watchdog': 'active',
            'watchdog_seconds': 120, 'active_ssid': 'x', 'pending': ''}


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.patches = [patch.object(d.control, 'local_hotspots', return_value=[dict(HOTSPOT)]),
                        patch.object(d.control, 'local_share_addresses',
                                     return_value={'name': 'smb://lab.local/shared', 'ip': 'smb://10.42.0.1/shared'}),
                        patch.object(d, 'STATE_ROOT', Path(self.tmp.name))]
        for p in self.patches:
            p.start()
        self.w = d.NetworkDialog({'pi4': 'user@192.168.3.142', 'pi5': 'user@192.168.3.156', 'jetson': 'nvidia@192.168.1.152'})
        self.app.processEvents()

    def tearDown(self):
        self.w.done(0)
        self.w.deleteLater()
        self.app.processEvents()
        for p in self.patches:
            p.stop()
        self.tmp.cleanup()

    def test_share_address_shown_and_hidden_without_share(self):
        self.assertIn('smb://lab.local/shared', self.w.share_info.text())
        self.assertIn('smb://10.42.0.1/shared', self.w.share_info.text())
        with patch.object(d.control, 'local_share_addresses', return_value={}):
            self.w.refresh_hotspots()
        self.assertEqual(self.w.share_info.text(), '')

    def test_share_connect_on_server_pc_and_teammate(self):
        self.w.connect_share()
        self.assertIn('이 PC가 공유폴더', self.w.share_status.text())
        import share_mount
        with patch.object(d.control, 'local_share_addresses', return_value={}), \
                patch.object(share_mount, 'connect', return_value={'ok': True, 'reason': 'mounted', 'server': '10.42.0.1',
                                                                   'source': '/run/x/A', 'message': ''}):
            self.w.connect_share()
        self.assertIn('연결됨: smb://10.42.0.1/', self.w.share_status.text())
        with patch.object(d.control, 'local_share_addresses', return_value={}), \
                patch.object(share_mount, 'connect', return_value={'ok': False, 'reason': 'password_needed', 'server': 'a',
                                                                   'source': '', 'message': '비밀번호 필요'}), \
                patch.object(share_mount, 'ask_password_dialog', return_value=None):
            self.w.connect_share()
        self.assertIn('연결 안 됨', self.w.share_status.text())

    def test_passwords_hidden_and_cleared_on_close(self):
        for field in (self.w.ssh_password, self.w.wifi_password, self.w.new_password):
            self.assertEqual(field.echoMode(), QtWidgets.QLineEdit.Password)
        self.w.ssh_password.setText('secret')
        self.w.wifi_password.setText('wifi-secret')
        self.w.done(0)
        self.assertEqual(self.w.ssh_password.text(), '')
        self.assertEqual(self.w.wifi_password.text(), '')

    def test_board_addresses_shown_from_gateway(self):
        self.assertIn('10.42.0.14', self.w.board_address_label.text())
        self.w.gateway.setText('192.168.12.1/24')
        self.assertIn('192.168.12.16', self.w.board_address_label.text())

    def test_fill_from_laptop_hotspot_reads_ssid_password_gateway(self):
        with patch.object(d.control, 'local_hotspot_password', return_value='hotspot-pw'):
            self.w.fill_from_hotspot()
        self.assertEqual(self.w.ssid.text(), 'su')
        self.assertEqual(self.w.wifi_password.text(), 'hotspot-pw')
        self.assertEqual(self.w.gateway.text(), '10.42.0.1/24')

    def test_register_sends_all_three_addresses_and_watchdog(self):
        self.w.ssh_password.setText('pw')
        self.w.ssid.setText('su')
        self.w.wifi_password.setText('hotspot-pw')
        with patch.object(d, 'BoardWorker') as worker:
            self.w.register()
        action, targets, passwords, options = worker.call_args[0][:4]
        self.assertEqual(action, 'register')
        self.assertEqual(set(targets), {'pi4', 'pi5', 'jetson'})
        self.assertEqual(options['jetson']['addresses'], {'pi4': '10.42.0.14', 'pi5': '10.42.0.15', 'jetson': '10.42.0.16'})
        self.assertEqual(options['pi4']['seconds'], 120)
        self.assertNotIn('hotspot-pw', self.w.log.toPlainText())

    def test_indoor_needs_all_boards_and_password(self):
        self.w.selected['pi5'].setChecked(False)
        self.w.ssh_password.setText('pw')
        with patch.object(d, 'BoardWorker') as worker:
            self.w.board_action('indoor')
        worker.assert_called_once()
        self.assertEqual(set(worker.call_args[0][1]), {'pi4', 'pi5', 'jetson'})
        self.w.worker = None
        self.w.ssh_password.setText('')
        with patch.object(d, 'BoardWorker') as worker:
            self.w.board_action('indoor')
        worker.assert_not_called()

    def test_duplicate_addresses_do_not_launch_worker(self):
        self.w.ssh_password.setText('pw')
        self.w.hosts['pi5'].setText('user@192.168.3.142')
        with patch.object(d, 'BoardWorker') as worker:
            self.w.board_action('status')
        worker.assert_not_called()

    def test_indoor_success_turns_on_hotspot_and_applies_laptop_addresses(self):
        applied = []
        self.w.hosts_applied.connect(applied.append)
        with patch.object(d.control, 'local_hotspot_switch', return_value=[]) as switch, \
                patch.object(d.QtCore.QTimer, 'singleShot', side_effect=lambda ms, fn: fn()):
            self.w.on_result({'ok': True, 'phase': 'indoor', 'boards': {}})
        switch.assert_called_once_with(HOTSPOT['uuid'], True)
        self.assertEqual(applied[-1], {'pi4': 'user@10.42.0.14', 'pi5': 'user@10.42.0.15', 'jetson': 'nvidia@10.42.0.16'})

    def test_failed_indoor_does_not_touch_hotspot(self):
        with patch.object(d.control, 'local_hotspot_switch') as switch:
            self.w.on_result({'ok': False, 'phase': 'indoor', 'boards': {}})
        switch.assert_not_called()

    def test_classroom_addresses_need_fresh_classroom_status_and_are_saved(self):
        self.w.ssh_password.setText('pw')
        with patch.object(d, 'BoardWorker') as worker:
            self.w.apply_classroom_addresses()
        worker.assert_not_called()
        self.w.latest = {'pi4': ok('pi4', ipv4=['192.168.3.142']), 'pi5': ok('pi5', ipv4=['192.168.3.156']),
                         'jetson': ok('jetson', ipv4=['192.168.1.152'])}
        with patch.object(d, 'BoardWorker') as worker:
            self.w.apply_classroom_addresses()
        self.assertEqual(worker.call_args[0][3]['pi4']['peers']['jetson'], '192.168.1.152')
        self.w.worker = None
        applied = []
        self.w.hosts_applied.connect(applied.append)
        self.w.on_result({'ok': True, 'phase': 'peers', 'boards': {}})
        saved = json.loads((Path(self.tmp.name) / 'network_sets.json').read_text())
        self.assertEqual(saved['classroom']['pi4'], 'user@192.168.3.142')
        self.w.hosts['pi4'].setText('user@10.42.0.14')
        self.w.load_classroom_addresses()
        self.assertEqual(self.w.hosts['pi4'].text(), 'user@192.168.3.142')

    def test_time_install_and_status_give_follower_the_current_pi5_address(self):
        import clock_sync
        for action in ('time_install', 'time_status'):
            stale = {'ok': False, 'board': 'pi4', 'reason': 'not_following_pi5', 'servers': ['192.168.3.156']}
            pi5 = {'ok': True, 'board': 'pi5', 'reason': 'reference_serving'}
            targets = {'pi4': 'user@192.168.2.252', 'pi5': 'user@192.168.3.164'}
            w = d.BoardWorker(action, targets, {'pi4': 'pw4', 'pi5': 'pw5'}, {})
            learned = {'ok': True, 'board': 'pi4', 'updated': True, 'message': 'Pi5 주소 192.168.3.164 로 갱신'}
            with patch.object(d.control, 'time_request', side_effect=lambda b, *a, **k: dict(pi5 if b == 'pi5' else stale)), \
                    patch.object(clock_sync, '_learn_pi5', return_value=learned) as learn:
                results = []
                w.result.connect(results.append)
                w.run()
            learn.assert_called_once_with(stale, 'pi4', 'user@192.168.2.252', 'pw4', 'user@192.168.3.164')
            self.assertTrue(results[0]['ok'], action)

    def test_worker_requests_each_board_and_clears_secrets(self):
        calls = []
        def request(board, host, password, action, **options):
            calls.append((board, action, password, options))
            return ok(board)
        passwords = {'pi4': 'pw'}
        options = {'pi4': {'seconds': 120}}
        w = d.BoardWorker('watchdog', {'pi4': 'user@192.168.3.142'}, passwords, options)
        with patch.object(d.control, 'request', side_effect=request):
            w.run()
        self.assertEqual(calls[0][:3], ('pi4', 'watchdog', 'pw'))
        self.assertEqual(passwords, {})

    def test_launcher_spectator_and_live_viewer_disable_network_window(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(launcher, 'STATE_ROOT', Path(tmp)):
            w = launcher.Launcher()
            self.assertEqual(w.network_button.text(), 'Wi-Fi / 노트북 핫스팟')
            w.connection_mode.setCurrentIndex(1)
            self.assertFalse(w.network_button.isEnabled())
            with patch.object(d, 'NetworkDialog') as dialog:
                w.open_network()
                dialog.assert_not_called()
            w.connection_mode.setCurrentIndex(0)
            w.viewer = Mock()
            w.viewer.poll.return_value = None
            w.update_buttons()
            self.assertFalse(w.network_button.isEnabled())
            w.viewer = None
            w.close()
            w.deleteLater()

    def test_applied_addresses_persist_without_secrets_and_reach_monitor(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(launcher, 'STATE_ROOT', Path(tmp)):
            w = launcher.Launcher()
            hosts = {'pi4': 'user@10.42.0.14', 'pi5': 'user@10.42.0.15', 'jetson': 'nvidia@10.42.0.16'}
            w.password.setText('secret')
            w.apply_network_hosts(hosts)
            stored = json.loads((Path(tmp) / 'boards.json').read_text())
            self.assertEqual(stored['pi4'], hosts['pi4'])
            self.assertNotIn('secret', str(stored))
            args = w.monitor_arguments((hosts['pi5'], hosts['jetson']), Path(tmp))
            self.assertEqual(args[args.index('--planning-host') + 1], hosts['pi4'])
            w.close()
            w.deleteLater()


if __name__ == '__main__':
    unittest.main()
