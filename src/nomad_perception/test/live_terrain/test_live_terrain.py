import numpy as np
from nomad_perception.live_terrain import Layers

def ground_patch(z_slope):
 x,y=np.meshgrid(np.linspace(.01,1.19,40),np.linspace(.01,1.19,40))
 return np.column_stack([x.ravel(),y.ravel(),z_slope*x.ravel()]).astype(np.float32)

def test_measured_ground_and_class_same_cell():
 m=Layers(size=4.,max_size=4.);p=ground_patch(np.tan(np.deg2rad(12)))
 e=m.ingest(p,np.full(len(p),8,dtype=np.uint32),1.)
 h,s,c=m.arrays(e)
 assert np.isfinite(h).sum()>0
 assert np.isfinite(s).sum()>0
 assert np.max(np.abs(s[np.isfinite(s)]-12))<.1
 assert np.all(c[np.isfinite(h)]==8)

def test_nonground_tree_does_not_become_ground():
 m=Layers(size=4.,max_size=4.);p=ground_patch(0)+[0,0,1]
 e=m.ingest(p,np.full(len(p),1,dtype=np.uint32),1.)
 h,s,c=m.arrays(e)
 assert (c==1).any();assert not np.isfinite(h).any();assert not np.isfinite(s).any()

def test_clock_reset_and_unknown():
 m=Layers(size=4.,max_size=4.)
 h,s,c=m.arrays({});assert np.isnan(h).all() and np.isnan(s).all() and (c==-1).all()
 p=ground_patch(0);m.ingest(p,np.full(len(p),8,dtype=np.uint32),2.)
 try:m.ingest(p,np.full(len(p),8,dtype=np.uint32),1.)
 except ValueError:pass
 else:raise AssertionError('clock reset accepted')


def test_live_slope_never_falls_back_to_cell_heights():
 m=Layers(size=4.,max_size=4.)
 def forbidden(*args):raise AssertionError('cell-height slope fallback invoked')
 m.core.refit_heights=forbidden
 # Sparse evidence cannot define a plane, even with neighboring cell summaries.
 p=np.array([[.1,.1,.0],[.3,.1,.02],[.5,.1,.04]],dtype=np.float32)
 e=m.ingest(p,np.full(len(p),8,dtype=np.uint32),1.)
 assert not np.isfinite(m.arrays(e)[1]).any()
 assert not m.core.allow_height_refit
 assert all(r['source']!='height_refit' for r in m.core.quality_records.values())
