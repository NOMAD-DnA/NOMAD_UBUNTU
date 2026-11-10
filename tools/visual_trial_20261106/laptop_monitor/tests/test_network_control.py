import json,os,subprocess,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0,str(Path(__file__).parents[1]))
import network_control as n


def response(board='pi4',status='idle',**extra):
 return dict(ok=True,feature=n.FEATURE,board=board,machine_id='a'*32,status=status,ipv4=['192.168.43.24'],active_ssid='office',target_active=False,**extra)
class FakeSSH:
 calls=[]
 def __init__(self,args,**kw):self.args=args;self.kw=kw;self.returncode=0;self.calls.append(self)
 def communicate(self,payload=None,timeout=None):
  self.payload=payload
  return n.MARKER+json.dumps(response())+'\n',''
class Tests(unittest.TestCase):
 def test_secrets_never_enter_argv_or_shared_files(self):
  with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{'XDG_STATE_HOME':d}),patch.object(n.subprocess,'Popen',FakeSSH):
   value=n.request('pi4','user@192.168.43.24','board-secret','prepare',ssid='a; $(touch /tmp/never)',password='wifi-secret')
   self.assertTrue(value['ok']);call=FakeSSH.calls[-1]
   self.assertNotIn('wifi-secret',str(call.args));self.assertNotIn('board-secret',str(call.args))
   self.assertIn('ControlPath=none',call.args);self.assertIn('wifi-secret',call.payload)
   self.assertFalse(Path(call.kw['env']['SSH_ASKPASS']).exists())
   self.assertNotIn('wifi-secret',json.dumps(value))
   for p in Path(d).rglob('*'):
    if p.is_file():self.assertNotIn('wifi-secret',p.read_text())
 def test_result_rejects_ambiguous_incompatible_or_wrong_board(self):
  good=n.MARKER+json.dumps(response())
  for text,rc in [(good+'\n'+good,0),(good,1),(n.MARKER+json.dumps(response(board='pi5')),0),(n.MARKER+'{}',0),('',0)]:
   self.assertFalse(n.parse_result(text,rc,'pi4')['ok'])
 def test_unknown_fields_and_error_text_cannot_leak(self):
  v=response(password='secret',stderr='secret');result=n.parse_result(n.MARKER+json.dumps(v),0,'pi4')
  self.assertNotIn('secret',json.dumps(result))
  v={'ok':False,'feature':n.FEATURE,'reason':'wifi-secret'}
  self.assertNotIn('wifi-secret',json.dumps(n.parse_result(n.MARKER+json.dumps(v),1,'pi4')))
 def test_status_machine_mismatch_is_not_success(self):
  result=n.parse_result(n.MARKER+json.dumps(response()),0,'pi4','b'*32)
  self.assertEqual(result['reason'],'wrong_machine')
 def test_all_prepare_must_pass_before_switch(self):
  calls=[]
  def req(board,host,pw,action,**kw):
   calls.append((board,action));return {'ok':board!='pi5','token':'a'*32,'machine_id':'b'*32,'status':'prepared'}
  with patch.object(n,'request',side_effect=req):
   result=n.connect_many({'pi4':'user@a','pi5':'user@b','jetson':'nvidia@c'},{b:'password' for b in n.BOARDS},'phone','wifi-secret')
  self.assertFalse(result['ok']);self.assertFalse(any(a=='switch' for _,a in calls))
 def test_invalid_prepare_state_cannot_start_any_switch(self):
  with patch.object(n,'request',return_value={'ok':True,'status':'idle'}) as request:
   result=n.connect_many({'pi4':'user@a'},{'pi4':'password'},'phone','wifi-secret')
  self.assertFalse(result['ok']);self.assertEqual(request.call_count,1)
 def test_switch_disconnect_retains_prepared_identity_not_success(self):
  def req(board,host,pw,action,**kw):
   return dict(ok=True,token='a'*32,machine_id='b'*32,status='prepared') if action=='prepare' else n.failed('ssh_timeout_unconfirmed')
  with patch.object(n,'request',side_effect=req):result=n.connect_many({'pi4':'user@a'},{'pi4':'password'},'phone','wifi-secret')
  self.assertFalse(result['ok']);self.assertEqual(result['boards']['pi4']['token'],'a'*32)
 def test_partial_switch_keeps_per_board_results(self):
  def req(board,host,pw,action,**kw):
   return dict(ok=True,token='a'*32,machine_id='b'*32,status='prepared' if action=='prepare' else 'queued') if board!='pi5' or action=='prepare' else n.failed('ssh_unreachable')
  with patch.object(n,'request',side_effect=req):result=n.connect_many({'pi4':'user@a','pi5':'user@b'},{b:'password' for b in n.BOARDS},'phone','wifi-secret')
  self.assertFalse(result['ok']);self.assertTrue(result['boards']['pi4']['ok']);self.assertFalse(result['boards']['pi5']['ok'])
 def test_invalid_host_and_credentials_never_launch_ssh(self):
  with patch.object(n.subprocess,'Popen') as popen:
   for host,pw in [('user@1.2.3.4;id','password'),('nvidia@192.0.2.1','password'),('user@192.0.2.1','bad\npassword')]:
    self.assertFalse(n.request('pi4',host,pw)['ok'])
   self.assertFalse(n.request('pi4','user@192.0.2.1','password','prepare',ssid='name',password='short')['ok'])
   popen.assert_not_called()
 def test_sudo_wrapper_consumes_password_independently_of_sudo_cache(self):
  # A fake NOPASSWD sudo reads NO stdin, just like cached real sudo.
  # It verifies the private JSON remains valid rather than executing a password line.
  with tempfile.TemporaryDirectory() as d:
   stub=Path(d)/'sudo'
   stub.write_text('#!/usr/bin/env python3\nimport sys,json,pathlib\np=pathlib.Path(sys.argv[sys.argv.index("--request-file")+1]); v=json.loads(p.read_text()); p.unlink(); print("OK:"+v["action"])\n')
   stub.chmod(0o700)
   r=subprocess.run(['bash','-c',n.WRAPPER],input='sudo secret; $(exit 99)\n'+json.dumps({'action':'status'})+'\n',text=True,capture_output=True,env=dict(os.environ,PATH=d+':'+os.environ['PATH']))
   self.assertEqual(r.returncode,0);self.assertEqual(r.stdout.strip(),'OK:status')
if __name__=='__main__':unittest.main()
