import json,os,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import Mock,patch
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).parents[1]))
from PyQt5 import QtWidgets
import launcher,network_dialog as d
class Tests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
 def setUp(self):self.w=d.NetworkDialog()
 def tearDown(self):self.w.done(0);self.w.deleteLater();self.app.processEvents()
 def test_both_passwords_hidden_and_clear_on_close(self):
  self.assertEqual(self.w.ssh_password.echoMode(),QtWidgets.QLineEdit.Password)
  self.assertEqual(self.w.wifi_password.echoMode(),QtWidgets.QLineEdit.Password)
  self.w.ssh_password.setText('secret');self.w.wifi_password.setText('wifi-secret');self.w.done(0)
  self.assertEqual(self.w.ssh_password.text(),'');self.assertEqual(self.w.wifi_password.text(),'')
 def test_empty_or_duplicate_addresses_do_not_launch_worker(self):
  self.w.ssh_password.setText('password');self.w.ssid.setText('phone');self.w.wifi_password.setText('wifi-password')
  self.w.hosts['pi4'].setText('user@192.0.2.1');self.w.hosts['pi5'].setText('user@192.0.2.1')
  with patch.object(d,'NetworkWorker') as worker:self.w.start_action('connect');worker.assert_not_called()
 def test_no_password_in_log_or_failure_display(self):
  self.w.wifi_password.setText('wifi-secret')
  self.w.on_progress('pi4',{'ok':False,'reason':'ssh_unreachable'})
  self.assertNotIn('wifi-secret',self.w.log.toPlainText())
 def test_new_address_confirm_pins_prepared_machine_and_token(self):
  identity={'machine_id':'a'*32,'token':'b'*32};calls=[]
  def request(board,host,password,action,**options):
   calls.append((action,options));return dict(ok=True,**identity,status='awaiting_confirmation')
  w=d.NetworkWorker('confirm',{'pi4':'user@192.0.2.1'},{'pi4':'password'},{},{'pi4':identity})
  with patch.object(d.control,'request',side_effect=request):w.run()
  self.assertEqual(calls[0],('status',{'machine_id':'a'*32}));self.assertEqual(calls[1],('confirm',identity))
 def test_wrong_machine_blocks_confirm(self):
  w=d.NetworkWorker('confirm',{'pi4':'user@192.0.2.1'},{'pi4':'password'},{},{'pi4':{'machine_id':'a'*32,'token':'b'*32}})
  with patch.object(d.control,'request',return_value={'ok':False,'reason':'wrong_machine'}) as request:
   w.run();self.assertEqual(request.call_count,1)
 def test_launcher_spectator_and_live_viewer_disable_switch(self):
  with tempfile.TemporaryDirectory() as tmp,patch.object(launcher,'STATE_ROOT',Path(tmp)):
   w=launcher.Launcher();w.connection_mode.setCurrentIndex(1);self.assertFalse(w.network_button.isEnabled())
   with patch.object(d,'NetworkDialog') as dialog:w.open_network();dialog.assert_not_called()
   w.connection_mode.setCurrentIndex(0);w.viewer=Mock();w.viewer.poll.return_value=None;w.update_buttons()
   self.assertFalse(w.network_button.isEnabled())
   with patch.object(d,'NetworkDialog') as dialog:w.open_network();dialog.assert_not_called()
   w.viewer=None;w.close();w.deleteLater()
 def test_applied_addresses_persist_without_secrets_and_reach_monitor(self):
  with tempfile.TemporaryDirectory() as tmp,patch.object(launcher,'STATE_ROOT',Path(tmp)):
   w=launcher.Launcher();hosts={'pi4':'user@192.168.43.4','pi5':'user@192.168.43.5','jetson':'nvidia@192.168.43.6'}
   w.password.setText('secret');w.apply_network_hosts(hosts)
   stored=json.loads((Path(tmp)/'boards.json').read_text());self.assertEqual(stored['pi4'],hosts['pi4']);self.assertNotIn('secret',str(stored))
   args=w.monitor_arguments((hosts['pi5'],hosts['jetson']),Path(tmp));self.assertEqual(args[args.index('--planning-host')+1],hosts['pi4'])
   w.close();w.deleteLater()
if __name__=='__main__':unittest.main()
