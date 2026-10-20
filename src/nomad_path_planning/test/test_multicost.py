import math
import numpy as np
import pytest
from nomad_path_planning.gpp.multicost import compose,validate_weights
from nomad_path_planning.gpp.registry import create
from nomad_path_planning.gpp.base import Request,Grid

def grid():return dict(width=5,height=3,resolution=1.,origin_x=0.,origin_y=0.,data=[0]*15)

def test_formula_no_legacy_double_count():
 g=grid();g['data'][0]=70
 c=compose(g,[.25]*15,[.5]*15,obstacle_weight=0,slope_weight=2,terrain_weight=4)
 assert c[0]==pytest.approx(3.5) and c[1]==c[0]

def test_missing_and_blocked_are_not_free():
 g=grid();g['data'][:4]=[-1,100,80,79]
 c=compose(g,[math.nan]*15,[math.nan]*15)
 assert c[0]==7.5 and math.isinf(c[1]) and math.isinf(c[2]) and c[3]>=3
 with pytest.raises(ValueError):validate_weights(g['data'],[1.]*15)
 with pytest.raises(ValueError):compose(g,[2.]*15,[0.]*15)

def test_dstar_updates_feature_only_and_agrees_with_astar():
 g=grid();s=[0.]*15;t=[0.]*15
 p=create('dstar_lite',seconds=2.,expansions=1000);req=Request(g,(.5,1.5,0),(4.5,1.5))
 g['cell_weights']=compose(g,s,t,obstacle_weight=0,terrain_weight=20)
 r=p.plan(req);assert r.path
 t[6:9]=[1.]*3
 g['cell_weights']=compose(g,s,t,obstacle_weight=0,terrain_weight=20)
 changed=p.plan(req);fresh=create('astar',seconds=2.).plan(req)
 assert changed.path and fresh.path
 assert max(abs(q[1]-1.5) for q in changed.path)>=1.
 assert Grid(req).path_cost(changed.path)==pytest.approx(Grid(req).path_cost(fresh.path))

def test_perception_missing_features_remain_nan():
 from nomad_perception.navigation_costmap import NavigationLayers
 from nomad_perception.planning_layers import planning_layers
 m=NavigationLayers(size=4,max_size=4)
 m.bootstrap_free(0,0,1)
 s,t=planning_layers(m,m.arrays({}),20)
 assert np.isnan(s).all() and np.isnan(t).all()
 m.votes[(0,0)]=np.array([0,0,0,0,0,0,0,0,10,0]);s,t=planning_layers(m,m.arrays({}),20)
 x,y=m.views.grid.cell(.1,.1)
 assert np.isnan(s[y,x]) and t[y,x]==pytest.approx(5/79)


def test_cluster_density_adds_observed_sources_not_inflation_or_unknown():
 from nomad_path_planning.gpp.multicost import cluster_density
 g=dict(width=15,height=15,resolution=.2,origin_x=0.,origin_y=0.,data=[0]*225)
 a=np.asarray(g['data']).reshape(15,15);a[7,7]=100
 g['data']=a.ravel().tolist();single=cluster_density(g,1.)
 a[7,8]=99;a[7,9]=-1;g['data']=a.ravel().tolist()
 assert np.allclose(single,cluster_density(g,1.))
 a[7,6]=100;g['data']=a.ravel().tolist();dense=cluster_density(g,1.)
 assert dense[7*15+9]>single[7*15+9]
 old=compose(g,[0.]*225,[0.]*225,cluster_weight=0.)
 new=compose(g,[0.]*225,[0.]*225,cluster_weight=32.,cluster_radius=1.)
 assert new[7*15+9]>old[7*15+9]  # observed neighbours affect unknown too
 assert math.isinf(new[7*15+7]) and math.isinf(new[7*15+8])
 assert np.all(np.asarray(new)>=np.asarray(old))


def test_cluster_cost_changes_path_without_blocking_gap():
 g=dict(width=31,height=21,resolution=.2,origin_x=0.,origin_y=0.,data=[0]*651)
 a=np.asarray(g['data']).reshape(21,31)
 a[7:9,12:19]=100;a[12:14,12:19]=100
 g['data']=a.ravel().tolist();features=[0.]*651
 def plan(weight):
  d=dict(g);d['cell_weights']=compose(d,features,features,obstacle_weight=0.,cluster_weight=weight,cluster_radius=1.)
  req=Request(d,(.5,2.1,0),(5.7,2.1))
  result=create('astar',seconds=3.).plan(req)
  assert result.path
  return result.path
 straight=plan(0.);detour=plan(128.)
 assert max(abs(p[1]-2.1) for p in straight)<.21
 assert max(abs(p[1]-2.1) for p in detour)>1.


def test_cluster_invalid_settings():
 for options in ({'cluster_weight':-1},{'cluster_weight':math.nan},{'cluster_radius':0}):
  with pytest.raises(ValueError):compose(grid(),[0.]*15,[0.]*15,**options)


def test_influence_union_and_saturation():
 from nomad_path_planning.gpp.multicost import cluster_density
 g=dict(width=31,height=31,resolution=.2,origin_x=0.,origin_y=0.,data=[0]*961)
 a=np.asarray(g['data']).reshape(31,31);a[15,15]=100;g['data']=a.ravel().tolist()
 old=cluster_density(g,2.)
 new=cluster_density(g,2.,1.,.4)
 assert new[15*31+15]>old[15*31+15]*10
 # An independent brute-force union verifies overlaps are counted just once.
 a[15,18]=100;g['data']=a.ravel().tolist()
 y,x=np.mgrid[:31,:31];union=((x-15)**2+(y-15)**2<=25)|((x-18)**2+(y-15)**2<=25)
 disk=(x-15)**2+(y-15)**2<=100
 expected=min(1.,float(union[disk].mean())/.4)
 assert cluster_density(g,2.,1.,.4)[15*31+15]==pytest.approx(expected,abs=1e-6)
 assert max(cluster_density(g,2.,1.,.4))<=1.
 a[:]= -1;g['data']=a.ravel().tolist()
 assert not cluster_density(g,2.,1.,.4).any()


def test_forward_hybrid_detour_preserves_rear_axle_curvature():
 from nomad_path_planning.gpp.hybrid_astar import wrap
 from nomad_path_planning.route_guard import route_is_behind
 g=dict(width=50,height=50,resolution=.2,origin_x=-5.,origin_y=-5.,data=[0]*2500)
 request=Request(g,(0.,0.,math.pi/2),(3.,0.))
 result=create('hybrid_astar',seconds=5.,expansions=50000).plan(request)
 assert result.path and result.path[0]==request.start
 assert not route_is_behind(request.start,result.path)
 for a,b in zip(result.path,result.path[1:]):
  ax,ay=a[0]-.36*math.cos(a[2]),a[1]-.36*math.sin(a[2])
  bx,by=b[0]-.36*math.cos(b[2]),b[1]-.36*math.sin(b[2])
  chord=math.hypot(bx-ax,by-ay);dyaw=wrap(b[2]-a[2]);mid=a[2]+dyaw/2
  assert (bx-ax)*math.cos(mid)+(by-ay)*math.sin(mid)>0
  assert abs(2*math.sin(dyaw/2)/chord)<=math.tan(.4)/.72+1e-6
