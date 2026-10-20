import json,sys,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
rows=json.loads(Path(sys.argv[1]).read_text());a=np.load(sys.argv[2]);dest=Path(sys.argv[3]);dest.mkdir(parents=True,exist_ok=True)
rank=[r for r in rows if r.get('weights') and r.get('result')=='path'];rank.sort(key=lambda r:(-r['known_cell_center_clearance_min_m'],r['unknown_m'],r['length_m']))
best=rank[0];base=rows[0]
d=a['data'].reshape(int(a['height']),int(a['width']));img=np.zeros(d.shape);img[(d>=0)&(d<80)]=1;img[d>=80]=2;img[d==100]=3
r=float(a['resolution']);ox=float(a['origin_x']);oy=float(a['origin_y']);extent=[ox,ox+r*d.shape[1],oy,oy+r*d.shape[0]]
fig,axes=plt.subplots(1,2,figsize=(13,6))
for ax in axes:
 ax.imshow(img,origin='lower',extent=extent,cmap=ListedColormap(['#eeeeee','#dbeedb','#777777','#161616']),vmin=0,vmax=3,interpolation='nearest')
 t=np.linspace(0,2*np.pi,400);ax.plot(28*np.cos(t),20*np.sin(t),'--',color='#999999',label='Design route (reference only)')
 for row,color,label in [(base,'#0064c8','Existing cost'),(best,'#d86600',f"Multi-factor {best['weights']}")]:
  path=np.array(row['path']);ax.plot(path[:,0],path[:,1],color=color,lw=1.8,label=label)
 ax.scatter(*a['start'],color='green',s=35,zorder=5);ax.scatter(*a['goal'],color='red',marker='*',s=100,zorder=5)
 ax.set_aspect('equal');ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.grid(alpha=.15)
axes[0].set(xlim=(-40,40),ylim=(-30,30),title='Frozen observed map: unknown is light grey')
axes[1].set(xlim=(-32,-12),ylim=(-14,6),title='Start-area detail; black = blocked/source cells')
axes[0].legend(loc='lower right',fontsize=8)
fig.suptitle('Map 4 opposite-goal cost sweep (not a live-success claim)');fig.tight_layout();fig.savefig(dest/'cost_paths.png',dpi=160);plt.close(fig)
keys=['weights','result','length_m','unknown_m','known_cell_center_clearance_min_m','wall_s','cpu_s','expansions']
with (dest/'sweep.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:r.get(k) for k in keys} for r in rows)
summary=dict(configurations=len(rank),snapshot=str(sys.argv[2]),observed_cells=int((d>=0).sum()),total_cells=int(d.size),best_by_known_cell_clearance={k:best[k] for k in keys if k in best},baseline={k:base[k] for k in keys if k in base},limitations=['Clearance is observed source-cell centre distance, not body clearance','Single frozen observed map; not a success rate','Unobserved future obstacles are unavailable to all conditions'])
(dest/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
