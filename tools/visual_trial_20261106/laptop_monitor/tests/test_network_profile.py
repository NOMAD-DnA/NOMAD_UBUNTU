import json,os,subprocess,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
BASE=Path(__file__).parents[1]/'remote'
class Tests(unittest.TestCase):
 def render(self,peers=None):
  with tempfile.TemporaryDirectory() as tmp:
   d=Path(tmp);p=d/'peers.json'
   if peers is not None:p.write_text(peers)
   script=(BASE/'planning_output_profile.sh').read_text().replace('/etc/nomad/network-peers.json',str(p))
   sh=d/'profile.sh';sh.write_text(script)
   for name,text in [('ip','echo "2: wlan0 inet 172.20.10.3/28 brd 172.20.10.15 scope global wlan0"'),('getent','exit 2')]:
    f=d/name;f.write_text('#!/bin/sh\n'+text+'\n');f.chmod(0o700)
   env=dict(os.environ,PATH=str(d)+':'+os.environ['PATH']);env.pop('NOMAD_PLANNING_PEER',None)
   out=d/'dds.xml';r=subprocess.run(['bash',str(sh),str(out),'10.77.0.1','10.77.0.2'],env=env,capture_output=True,text=True)
   return r.returncode,out.read_text() if out.exists() else ''
 def test_hotspot_address_replaces_fixed_pi4_peer(self):
  rc,text=self.render(json.dumps({'pi4':'172.20.10.4','pi5':'172.20.10.3','jetson':'172.20.10.2'}))
  self.assertEqual(rc,0);ET.fromstring(text);self.assertIn('<address>172.20.10.4</address>',text);self.assertNotIn('192.168.3.142',text)
 def test_mdns_failure_retains_prior_fallback(self):
  rc,text=self.render();self.assertEqual(rc,0);self.assertIn('192.168.3.142',text)
 def test_bad_saved_address_stops_profile_generation(self):
  rc,text=self.render('{"pi4":"$(touch /tmp/unexpected)"}');self.assertNotEqual(rc,0);self.assertEqual(text,'')
 def test_embedded_profile_functions_match(self):
  def function(name):return (BASE/name).read_text().split('nomad_write_planning_output_profile() (',1)[1].split('\n)\n',1)[0]
  for name in ('start_jetson.sh','start_pi_vio_fusion.sh'):self.assertEqual(function(name),function('planning_output_profile.sh'))
if __name__=='__main__':unittest.main()
