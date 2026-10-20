import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parent))
from optimization import bounded,cleanup_registry
from common import save

def test_timeout_stops_owned_child(tmp_path):
 start=time.monotonic()
 assert bounded([sys.executable,'-c','import time; time.sleep(60)'],tmp_path/'child.log',.1,tmp_path)=='watchdog_timeout'
 assert time.monotonic()-start<5

def test_stop_marker_returns_without_hanging(tmp_path):
 (tmp_path/'STOP_REQUESTED').touch()
 assert bounded([sys.executable,'-c','pass'],tmp_path/'child.log',1,tmp_path) in ('finished','interrupted')

def test_cleanup_isolates_partition_and_handles_dead_leader(tmp_path):
 env=dict(os.environ,GZ_PARTITION='nomad_opt_'+tmp_path.name)
 child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True,env=env)
 other=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True,env=dict(env,GZ_PARTITION='other_user_process'))
 try:
  save(tmp_path/'owned_groups.json',[dict(pid=child.pid),dict(pid=other.pid)])
  cleanup_registry(tmp_path)
  child.wait(timeout=3)
  assert other.poll() is None
 finally:
  for proc in (child,other):
   if proc.poll() is None:proc.terminate()
   proc.wait(timeout=3)


def test_cluster_priority_does_not_trade_exposure_for_shortcut():
 from optimization import select_cluster_priority
 rows=[]
 for c,exposure,unknown,length in [(0,10,0,5),(1,2,30,45),(2,2.02,20,50),(3,1,0,5)]:
  for snapshot in ['a','b']:
   if c==3 and snapshot=='b':continue
   rows.append(dict(condition=c,snapshot=snapshot,result='path',influence_exposure_4m_1m=exposure,unknown_length_m=unknown,length_m=length))
 chosen,summary=select_cluster_priority(rows,['a','b'])
 assert chosen['condition']==2
 assert 3 not in summary['metrics']


def test_balanced_forward_rejects_extreme_detour_and_unknown():
 from optimization import rank_balanced_forward
 rows=[]
 for c,L,U,E,N in [(0,10,5,5,2),(1,13,6,1,1),(2,20,6,.1,.1),(3,12,10,.2,.2)]:
  for snap in ['a','b']:rows.append(dict(condition=c,snapshot=snap,result='path',length_m=L,unknown_length_m=U,influence_exposure_4m_1m=E,near_exposure_2m=N))
 ranked,details=rank_balanced_forward(rows,['a','b'])
 assert ranked[0]['condition']==1
 assert details['rejected_conditions']==[2,3]
