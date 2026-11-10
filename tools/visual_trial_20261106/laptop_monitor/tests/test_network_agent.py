"""All network/systemd operations are fake; no host network mutation."""
import importlib.util,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch,MagicMock
spec=importlib.util.spec_from_file_location('agent',Path(__file__).parents[1]/'board_network/network_agent.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
OLD='b4c33174-7544-483d-a131-1dc50da24bca'
class Fake:
 def __init__(self):
  self.calls=[];self.active=OLD;self.fail_target=False;self.fail_restore=False;self.fail_schedule_at=None;self.schedule_count=0
 def __call__(self,args,timeout=15):
  self.calls.append(args)
  if args[0]=='systemd-run':
   self.schedule_count+=1
   if self.schedule_count==self.fail_schedule_at:raise m.NetworkError('schedule_failed')
   return ''
  if args[0]=='systemctl':return 'active'
  a=args[5:]
  if a==['-t','-f','DEVICE,TYPE','device','status']:return 'wlan0:wifi\neth0:ethernet'
  if a==['radio','wifi']:return 'enabled'
  if a[:2]==['-g','GENERAL.STATE']:return '100 (connected)'
  if a[:2]==['-g','GENERAL.CON-UUID']:return self.active
  if a[:2]==['-g','IP4.ADDRESS']:return '192.168.43.24/24'
  if a[:2]==['-g','802-11-wireless.ssid']:return 'current-ssid'
  if a[:2]==['-g','connection.autoconnect-priority']:return '0'
  if a[:4]==['--wait','45','connection','up']:
   target=a[5]
   if target==OLD and self.fail_restore:raise m.NetworkError('restore_failed')
   if target!=OLD and self.fail_target:raise m.NetworkError('activation_failed')
   self.active=target
  return ''
class AgentTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();p=Path(self.tmp.name)
  self.manager=m.Switch(p/'state',p/'profiles',p/'installed.py',p/'etc/network-peers.json');self.fake=Fake()
  self.patches=[patch.object(m,'command',self.fake),patch.object(m.shutil,'which',return_value='/usr/bin/tool'),patch.object(m.time,'sleep'),patch.object(m,'boot_id',return_value='boot'),patch.object(m,'mac_address',return_value='00:11:22:33:44:55'),patch.object(m,'board_identity',return_value=('pi4','a'*32))]
  for p in self.patches:p.start()
 def tearDown(self):
  for p in reversed(self.patches):p.stop()
  self.tmp.cleanup()
 def prepared(self,ssid='한글 Phone ;1;',password='test-password'):return self.manager.prepare(ssid,password)
 def queued(self):
  p=self.prepared();self.manager.queue(p['token']);return self.manager.read()
 def awaiting(self):
  p=self.queued();self.manager.worker_run(p['token']);return self.manager.read()
 def test_prepare_keeps_network_and_secrets_private(self):
  p=self.prepared();s=self.manager.read();f=Path(s['profile_path'])
  self.assertEqual(self.fake.active,OLD);self.assertEqual(p['status'],'prepared')
  self.assertEqual(f.stat().st_mode&0o777,0o600);self.assertEqual(self.manager.state_path.stat().st_mode&0o777,0o600)
  self.assertIn('autoconnect=false',f.read_text());self.assertNotIn('test-password',json.dumps(self.fake.calls)+json.dumps(s)+json.dumps(p))
  self.assertFalse(any('up' in c or 'down' in c for c in self.fake.calls))
 def test_exact_utf8_ssid_and_special_psk(self):
  ssid=' 한글;12; ';self.prepared(ssid,'  a;b\\cdef  ');text=Path(self.manager.read()['profile_path']).read_text()
  encoded=text.split('ssid=',1)[1].splitlines()[0]
  self.assertEqual(bytes(int(x) for x in encoded.split(';') if x).decode(),ssid)
  self.assertIn('psk=\\s\\sa;b\\\\cdef\\s\\s',text)
 def test_guard_precedes_worker(self):
  self.queued();c=[c for c in self.fake.calls if c[0]=='systemd-run']
  self.assertEqual(c[0][-2],'_rollback');self.assertEqual(c[1][-2],'_worker');self.assertEqual(self.fake.active,OLD)
  self.assertTrue(all('--property=StandardOutput=null' in a for a in c))
 def test_confirm_requires_wifi_reconnect(self):
  p=self.awaiting();self.assertEqual(p['status'],'awaiting_confirmation')
  with self.assertRaisesRegex(m.NetworkError,'reconnect_via_wifi_address'):self.manager.confirm(p['token'],'10.77.0.1')
  self.manager.confirm(p['token'],'192.168.43.24');self.assertEqual(self.manager.read()['status'],'committed')
 def test_failed_activation_restores_previous(self):
  p=self.queued();self.fake.fail_target=True;self.manager.worker_run(p['token'])
  self.assertEqual(self.manager.read()['status'],'rolled_back');self.assertEqual(self.fake.active,OLD)
 def test_missing_confirmation_rolls_back_after_dhcp(self):
  p=self.awaiting();self.manager.rollback(p['token']);self.assertEqual(self.fake.active,OLD);self.assertEqual(self.manager.read()['status'],'rolled_back')
 def test_stale_timer_ignores_committed(self):
  p=self.awaiting();self.manager.confirm(p['token'],'192.168.43.24');active=self.fake.active
  self.manager.rollback(p['token']);self.assertEqual(self.fake.active,active)
 def test_stale_tokens_block_confirm_restore_worker(self):
  self.queued()
  with self.assertRaisesRegex(m.NetworkError,'stale_request'):self.manager.confirm('f'*32,'192.168.43.24')
  with self.assertRaisesRegex(m.NetworkError,'stale_request'):self.manager.queue_restore('f'*32)
  self.manager.rollback('f'*32);self.manager.worker_run('f'*32)
  self.assertEqual(self.manager.read()['status'],'queued');self.assertEqual(self.fake.active,OLD)
 def test_expired_and_rebooted_confirm_rejected(self):
  p=self.awaiting();s=self.manager.read();s['deadline']=0;self.manager.save(s)
  with self.assertRaisesRegex(m.NetworkError,'confirmation_expired'):self.manager.confirm(p['token'],'192.168.43.24')
  s.update(deadline=m.time.monotonic()+300,boot_id='old');self.manager.save(s)
  with self.assertRaisesRegex(m.NetworkError,'confirmation_expired'):self.manager.confirm(p['token'],'192.168.43.24')
 def test_duplicate_prepare_and_switch_rejected(self):
  p=self.queued()
  with self.assertRaisesRegex(m.NetworkError,'switch_pending'):self.prepared()
  with self.assertRaisesRegex(m.NetworkError,'not_prepared'):self.manager.queue(p['token'])
 def test_scheduler_failures_preserve_current_connection(self):
  for n in (1,2):
   self.fake.fail_schedule_at=self.fake.schedule_count+n;p=self.prepared()
   with self.assertRaisesRegex(m.NetworkError,'schedule_failed'):self.manager.queue(p['token'])
   self.assertEqual(self.fake.active,OLD);self.assertEqual(self.manager.read()['status'],'schedule_failed')
   self.manager.worker_run(p['token']);self.assertEqual(self.fake.active,OLD)
 def test_restore_failure_is_not_success(self):
  p=self.awaiting();self.fake.fail_restore=True;self.manager.rollback(p['token']);self.assertEqual(self.manager.read()['status'],'rollback_failed')
 def test_restore_detached_and_cancels_queued_forward(self):
  p=self.queued();self.assertEqual(self.manager.queue_restore(p['token'])['status'],'restore_queued')
  self.manager.worker_run(p['token']);self.assertEqual(self.fake.active,OLD)
  self.manager.rollback(p['token'],force=True);self.assertEqual(self.manager.read()['status'],'rolled_back')
 def test_obsolete_timer_ignores_new_preparation(self):
  p=self.awaiting();self.manager.rollback(p['token']);new=self.prepared('other','different-pass')
  self.manager.rollback(p['token']);self.assertEqual(self.manager.read()['token'],new['token'])
 def test_invalid_credentials_cannot_write_or_run(self):
  for ssid,pw in [('', 'password123'),('a'*33,'password123'),('ssid','short'),('ssid','password\n123'),('ssid','한글password'),('ssid\x00','password123')]:
   with self.assertRaises(m.NetworkError):self.prepared(ssid,pw)
  self.assertEqual(self.fake.calls,[]);self.assertFalse(self.manager.profiles.exists())
 def test_hex_psk_and_utf8_byte_limit(self):
  m.credentials('한'*10,'f'*64)
  with self.assertRaises(m.NetworkError):m.credentials('한'*11,'test-password')
  with self.assertRaises(m.NetworkError):m.credentials('phone','g'*64)
 def test_wrong_board_and_machine_rejected_before_mutation(self):
  for req in ({'action':'prepare','board':'pi5'},{'action':'prepare','board':'pi4','machine_id':'b'*32}):
   with self.assertRaises(m.NetworkError):m.handle(req,self.manager)
  self.assertEqual(self.fake.calls,[])
 def test_peers_require_local_address_and_peer_reachability(self):
  peers={'pi4':'192.168.43.24','pi5':'192.168.43.25','jetson':'192.168.43.26'}
  with patch.object(m.socket,'create_connection',return_value=MagicMock()) as connect:self.manager.save_peers(peers,'pi4');self.assertEqual(connect.call_count,2)
  self.assertEqual(json.loads(self.manager.peers.read_text()),peers);self.assertEqual(self.manager.peers.stat().st_mode&0o777,0o644)
  with patch.object(m.socket,'create_connection',side_effect=OSError()):
   with self.assertRaisesRegex(m.NetworkError,'board_peer_unreachable'):self.manager.save_peers(peers,'pi4')
  peers['pi4']='192.168.43.99'
  with self.assertRaisesRegex(m.NetworkError,'peer_address_not_current'):self.manager.save_peers(peers,'pi4')
 def test_peer_file_readable_under_root_private_umask(self):
  import os
  old=os.umask(0o077)
  try:
   with patch.object(m.socket,'create_connection',return_value=MagicMock()):
    self.manager.save_peers({'pi4':'192.168.43.24','pi5':'192.168.43.25','jetson':'192.168.43.26'},'pi4')
   self.assertEqual(self.manager.peers.stat().st_mode&0o777,0o644)
   self.assertEqual(self.manager.peers.parent.stat().st_mode&0o777,0o755)
  finally:os.umask(old)
 def test_pending_connection_cannot_save_stale_peer_configuration(self):
  self.awaiting()
  with self.assertRaisesRegex(m.NetworkError,'connection_not_confirmed'):
   self.manager.save_peers({'pi4':'192.168.43.24','pi5':'192.168.43.25','jetson':'192.168.43.26'},'pi4')
  self.assertFalse(self.manager.peers.exists())
 def test_changed_previous_connection_rejected(self):
  p=self.prepared();self.fake.active='12345678-1234-1234-1234-123456789012'
  with self.assertRaisesRegex(m.NetworkError,'previous_connection_changed'):self.manager.queue(p['token'])
if __name__=='__main__':unittest.main()
