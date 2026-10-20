import json,math,time,sys,itertools
from pathlib import Path
import numpy as np
from nomad_path_planning.gpp.multicost import compose
from nomad_path_planning.gpp.base import Request,Grid,BudgetExceeded
from nomad_path_planning.gpp.registry import create
from nomad_path_planning.gpp.proximity import proximity_costs
import cv2
root=Path(sys.argv[1]);file=Path(sys.argv[2]);a=np.load(file)
g={k:a[k].item() for k in ('width','height','resolution','origin_x','origin_y')};g['data']=a['data'].tolist();data=np.asarray(a['data'])
rows=[]
for w in [None]+list(itertools.product([2.,4.,8.],[.5,1.,2.],[.5,1.,2.])):
 d=dict(g)
 if w:d['cell_weights']=compose(d,a['slope'],a['terrain'],obstacle_weight=w[0],slope_weight=w[1],terrain_weight=w[2])
 else:d['proximity']=proximity_costs(d['data'],d['width'],d['height'],d['resolution'])
 req=Request(d,tuple(a['start'])+(0.,),tuple(a['goal']))
 p=create('dstar_lite',seconds=20,expansions=200000)
 t=time.perf_counter();c=time.process_time()
 row=dict(weights=w,snapshot=str(file),budget_wall_s=20,expansion_limit=200000)
 try:
  r=p.plan(req);row.update(result='path' if r.path else 'no_path',path=r.path,expansions=r.expansions)
  if r.path:
   field=Grid(req);dist=cv2.distanceTransform((data.reshape(d['height'],d['width'])!=100).astype('uint8'),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*d['resolution']
   length=unknown=0.;clear=[];slope=[];terrain=[]
   for p1,p2 in zip(r.path,r.path[1:]):
    ds=math.dist(p1[:2],p2[:2]);length+=ds
    x,y=field.cell(*p2[:2]);i=y*d['width']+x
    if data[i]<0:unknown+=ds
    elif data[i]<80:clear.append(float(dist[y,x]))
    if np.isfinite(a['slope'][i]):slope.append(float(a['slope'][i]))
    if np.isfinite(a['terrain'][i]):terrain.append(float(a['terrain'][i]))
   row.update(length_m=length,unknown_m=unknown,known_cell_center_clearance_min_m=min(clear) if clear else None,
              known_cells=int((data>=0).sum()),total_cells=int(data.size),
              near_cost_samples=len(clear),slope_samples=len(slope),terrain_samples=len(terrain))
 except (BudgetExceeded,RuntimeError) as e:row.update(result='budget_exceeded',error=str(e),expansions=p.graph.last_compute_expansions if p.graph else None)
 row.update(wall_s=time.perf_counter()-t,cpu_s=time.process_time()-c);rows.append(row)
 print({k:v for k,v in row.items() if k!='path'},flush=True)
 root.mkdir(parents=True,exist_ok=True);temp=root/'probe.pending.json';temp.write_text(json.dumps(rows,indent=2,allow_nan=False));temp.replace(root/'probe.json')
