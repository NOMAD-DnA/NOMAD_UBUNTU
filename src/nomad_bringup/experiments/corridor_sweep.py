"""Sequential real trials using optimization.py's recorder/watchdogs/cleanup."""
import argparse, json, math, os, time, subprocess, sys, csv, fcntl
from pathlib import Path
import yaml
from common import WORKSPACE, ROOT, save, read
from optimization import cleanup_registry

# proximity, cluster, goal, narrow, heading, endpoint-clearance weights
CONDITIONS=[
 ('baseline',32,64,.4,.8,.5,.35),('proximity_high',128,64,.4,.8,.5,.35),
 ('cluster_high',32,256,.4,.8,.5,.35),('both_high',128,256,.4,.8,.5,.35),
 ('wide',64,128,.1,3.,1.,2.),('very_wide',128,256,.1,6.,2.,4.),
 ('goal_progress',64,128,1.,3.,.5,2.),('direction_hold',64,128,.1,3.,4.,2.),
 ('low_cost',8,16,.4,.8,.5,.35),('moderate',16,64,.2,2.,1.,1.),
 ('strong_clearance',256,512,.2,4.,1.,3.),('balanced',64,64,.4,2.,2.,2.)]

def main():
 p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--status',action='store_true');p.add_argument('--stop',action='store_true');a=p.parse_args()
 import re
 if not re.fullmatch(r'[\w-]{1,55}',a.name):raise ValueError('Invalid name')
 root=ROOT/a.name
 if a.status:print(json.dumps(read(root/'status.json'),ensure_ascii=False,indent=2));return
 if a.stop:
  (root/'STOP_REQUESTED').touch();return
 root.mkdir(parents=True,exist_ok=True);lock=open(root/'sweep.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 if (root/'STOP_REQUESTED').exists():raise RuntimeError('Stopped campaign; explicit resume needs removing its marker')
 rows=[];process=None;runroot=None
 try:
  for index,(label,prox,cluster,goal,narrow,heading,clearance) in enumerate(CONDITIONS,1):
   if (root/'STOP_REQUESTED').exists():break
   name=f'{a.name}_c{index:02}';runroot=ROOT/name;outcome_file=runroot/'runs/trial_01/outcome.json'
   overrides={'dstar_lite_gpp':{'ros__parameters':{'corridor.goal_weight':goal,'corridor.length_weight':.15,'corridor.narrow_weight':narrow,'corridor.heading_weight':heading,'corridor.visit_weight':.8,'corridor.clearance_weight':clearance,'gpp.proximity_radius':4.}},'ackermann_rollout_lpp':{'ros__parameters':{'recovery.enabled':False,'managed_recovery':True}},'nomad_recovery_supervisor':{'ros__parameters':{'recovery.enabled':False}}}
   config=root/f'c{index:02}.yaml';config.write_text(yaml.safe_dump(overrides))
   command=[sys.executable,str(Path(__file__).with_name('optimization.py')),name,'--stage','validate','--foreground','--trial','trial_01','--goal-mode','observed_corridor','--gpp','hybrid_astar','--lpp','rpp','--start-yaw','-1.5707963267948966','--weights',f'{prox},.5,.5','--unknown-surcharge','9','--cluster-weight',str(cluster),'--cluster-radius','4','--cluster-influence','.75','--cluster-saturation','.4','--unknown-direction-weight','4','--unknown-direction-free-angle','15','--no-motion-seconds','45','--planning-overrides',str(config),'--semantic-model',str(ROOT/'semantic_map04_20261019/model_map04/model.onnx'),'--terrain-config',str(ROOT/'map4_sensor_only_20261019/terrain_config.yaml')]
   condition=dict(index=index,label=label,proximity=prox,cluster=cluster,goal=goal,narrow=narrow,heading=heading,clearance=clearance,command=command,overrides=overrides)
   save(root/f'c{index:02}.json',condition)
   if not outcome_file.exists():
    with (root/f'c{index:02}.log').open('w') as log:
     process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
     started=time.monotonic()
     while process.poll() is None:
      child=read(runroot/'status.json') if (runroot/'status.json').exists() else {}
      save(root/'status.json',dict(state='running',condition=index,total=len(CONDITIONS),completed=len(rows),label=label,successes=sum(x['result']=='success' for x in rows),current=child))
      if (root/'STOP_REQUESTED').exists() or time.monotonic()-started>2160:
       runroot.mkdir(parents=True,exist_ok=True);(runroot/'STOP_REQUESTED').touch()
      time.sleep(2)
     process=None
   if not outcome_file.exists():
    save(root/'status.json',dict(state='blocked',condition=index,completed=len(rows),reason='worker exited without outcome',log=str(root/f'c{index:02}.log')));return
   outcome=read(outcome_file)
   pos=outcome.get('position');row=dict(condition=index,label=label,result=outcome['result'],valid=outcome.get('valid',False),distance_m=outcome.get('distance_xy_m'),goal_distance_m=math.dist(pos[:2],[28,0]) if pos else None,reverse_m=outcome.get('reverse_distance_m'),duration_wall_s=outcome.get('duration_wall_s'),last_status=outcome.get('last_status'),root=str(runroot),weights=condition)
   rows.append(row);save(root/'results.json',rows)
   with (root/'results.csv').open('w') as out:
    keys=['condition','label','result','valid','distance_m','goal_distance_m','reverse_m','duration_wall_s'];writer=csv.DictWriter(out,fieldnames=keys,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
   print(json.dumps(row,ensure_ascii=False),flush=True)
   if not row['valid'] and row['result'] not in ('interrupted',):
    save(root/'status.json',dict(state='blocked',completed=len(rows),condition=index,reason=row['result']));return
  save(root/'status.json',dict(state='stopped' if (root/'STOP_REQUESTED').exists() else 'finished',completed=len(rows),total=len(CONDITIONS),successes=sum(x['result']=='success' for x in rows),results=str(root/'results.json')))
 finally:
  if process is not None and process.poll() is None:
   runroot.mkdir(parents=True,exist_ok=True);(runroot/'STOP_REQUESTED').touch()
   try:process.wait(timeout=90)
   except subprocess.TimeoutExpired:
    process.terminate();cleanup_registry(runroot)

if __name__=='__main__':main()
