import numpy as np
from nomad_perception.navigation_terrain import NavigationLayers,navigation_arrays,TemporalBlockEvidence
from test_live_terrain import ground_patch

def test_observed_ground_cost_and_slope_block():
 m=NavigationLayers(size=6.,max_size=6.);p=ground_patch(np.tan(np.deg2rad(6)))
 e=m.ingest(p,np.full(len(p),8,dtype=np.uint32),1.)
 a=m.arrays(e);g,l=navigation_arrays(m,e,a,1.)
 assert np.any((g>=0)&(g<80));assert not np.any(g==100)
 m=NavigationLayers(size=6.,max_size=6.);p=ground_patch(np.tan(np.deg2rad(25)))
 for stamp in [1.,1.79,2.58]:e=m.ingest(p,np.full(len(p),8,dtype=np.uint32),stamp)
 a=m.arrays(e)
 g,l=navigation_arrays(m,e,a,2.58)
 assert np.any(g==100)

def test_canopy_does_not_block_but_collision_height_does():
 m=NavigationLayers(size=6.,max_size=6.);m.body_z=.3
 p=ground_patch(0);ids=np.full(len(p),1,dtype=np.uint32)
 e=m.ingest(p+[0,0,4],ids,1.);assert not m.solid
 for stamp in [2.,2.3,2.6,2.9,3.2,3.5,3.8]:e=m.ingest(p+[0,0,.7],ids,stamp)
 assert m.solid
 g,l=navigation_arrays(m,e,m.arrays(e),2.);assert (g==100).any()

def test_expired_local_unknown_global_persists():
 m=NavigationLayers(size=6.,max_size=6.);p=ground_patch(0)
 e=m.ingest(p,np.full(len(p),8,dtype=np.uint32),1.);a=m.arrays(e)
 g,l=navigation_arrays(m,e,a,35.,local_age=30.)
 assert ((g>=0)&(g<80)).any();assert not ((l>=0)&(l<80)).any()


def test_ground_level_rock_misclassification_is_not_collision():
 m=NavigationLayers(size=6.,max_size=6.);m.body_z=0.
 p=ground_patch(0);e=m.ingest(p,np.full(len(p),7,dtype=np.uint32),1.)
 assert not m.solid
 g,l=navigation_arrays(m,e,m.arrays(e),1.)
 assert not (g==100).any()


def test_temporal_block_requires_independent_captures_and_clears_on_evidence():
 f=TemporalBlockEvidence();k=(0,0)
 assert not f.update(k,True,1.)
 for _ in range(10):assert not f.update(k,True,1.)
 for t in [1.2,1.4]:assert not f.update(k,True,t)
 assert f.update(k,True,1.6)
 for t in [2.,2.3,2.6,2.9,3.2]:f.update(k,False,t)
 assert k not in f.confirmed


def test_bootstrap_is_once_unknown_only_and_confirmed_blocks_win():
 m=NavigationLayers(size=6.,max_size=6.);g=m.views.grid
 x,y=g.cell(.1,.1);g.log_odds[y,x]=2;g.seen[y,x]=.5
 m.bootstrap_free(.0,.0,1.,1.5)
 assert g.log_odds[y,x]==2 and g.seen[y,x]==.5
 a=m.arrays({});globalmap,localmap=navigation_arrays(m,{},a,1.)
 center=g.cell(-.1,-.1)
 assert globalmap[center[1],center[0]]==5
 origin=m.bootstrap_origin;m.bootstrap_free(2.,2.,2.,1.5);assert m.bootstrap_origin==origin
 m.solid[(0,0)]=1.
 globalmap,localmap=navigation_arrays(m,{},a,1.)
 assert globalmap[y,x]==100 and localmap[y,x]==100


def test_scan_waits_for_capture_tf_instead_of_discarding():
 from collections import deque
 from types import SimpleNamespace as NS
 from sensor_msgs.msg import LaserScan
 from tf2_ros import TransformException
 from nomad_perception.navigation_terrain import NavigationTerrain
 m=LaserScan();m.header.stamp.sec=10;m.header.frame_id='laser';m.ranges=[2.];m.range_min=.1;m.range_max=8.
 class Buffer:
  ready=False
  def lookup_transform(self,*args):
   if not self.ready:raise TransformException('future extrapolation')
   return NS(transform=NS(translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
 calls=[];buffer=Buffer();fake=NS(odom=True,scan_queue=deque([m]),scan_stats={'tf_wait':0,'expired':0,'integrated':0},buffer=buffer,layers=NS(views=NS(grid=NS(integrate=lambda *args:calls.append(args)))),get_clock=lambda:NS(now=lambda:NS(nanoseconds=10200000000)))
 NavigationTerrain.integrate_pending_scans(fake)
 assert len(fake.scan_queue)==1 and not calls
 buffer.ready=True;NavigationTerrain.integrate_pending_scans(fake)
 assert not fake.scan_queue and len(calls)==1 and fake.scan_stats['integrated']==1


def test_bootstrap_survives_unclassified_scan_hits_but_not_confirmed_block():
 m=NavigationLayers(size=6.,max_size=6.);m.bootstrap_free(0.,0.,1.,1.5);g=m.views.grid
 g.log_odds[:]=2.;g.seen[:]=2. # Ambiguous 2D scan hits are not our semantic blockers.
 a=m.arrays({});glob,local=navigation_arrays(m,{},a,40.)
 x,y=g.cell(.1,.1);assert glob[y,x]==5 and local[y,x]==5
 m.solid[(0,0)]=40.;glob,local=navigation_arrays(m,{},a,40.)
 assert glob[y,x]==100 and local[y,x]==100
 m.solid.clear();m.bootstrap_active=False;glob,local=navigation_arrays(m,{},a,40.)
 assert glob[y,x]==5 and local[y,x]==-1


def test_runtime_bootstrap_does_not_open_unobserved_rear():
 m=NavigationLayers(size=6.,max_size=6.);m.bootstrap_free(0.,0.,1.,1.5,yaw=0.)
 a=m.arrays({});glob,local=navigation_arrays(m,{},a,1.)
 g=m.views.grid;front=g.cell(.7,.1);rear=g.cell(-.7,.1)
 assert glob[front[1],front[0]]==5 and local[front[1],front[0]]==5
 assert glob[rear[1],rear[0]]==-1 and local[rear[1],rear[0]]==-1


def test_explicit_initial_area_override_wins_only_inside_while_active():
 m=NavigationLayers(size=10.,max_size=10.,bootstrap_override=True)
 m.bootstrap_free(0.,0.,1.,3.,yaw=0.);g=m.views.grid
 m.solid[(3,0)]=2.;m.solid[(18,0)]=2.
 a=m.arrays({});glob,local=navigation_arrays(m,{},a,2.)
 inside=g.cell(.7,.1);outside=g.cell(3.7,.1);rear=g.cell(-.7,.1)
 for data in [glob,local]:
  assert data[inside[1],inside[0]]==5
  assert data[outside[1],outside[0]]==100
  assert data[rear[1],rear[0]]==-1
 assert m.solid[(3,0)]==2. # Recorded sensor evidence is preserved.
 m.bootstrap_active=False;glob,local=navigation_arrays(m,{},a,3.)
 assert glob[inside[1],inside[0]]==100 and local[inside[1],inside[0]]==100


def test_observed_ray_free_memory_survives_ambiguous_hit_but_expires_locally():
 m=NavigationLayers(size=6.,max_size=6.);g=m.views.grid
 g.log_odds[:]=-3.;g.seen[:]=1.;a=m.arrays({})
 navigation_arrays(m,{},a,1.)
 g.log_odds[:]=2.;g.seen[:]=2.
 glob,local=navigation_arrays(m,{},a,2.)
 x,y=g.cell(.1,.1);assert glob[y,x]==5 and local[y,x]==5
 glob,local=navigation_arrays(m,{},a,32.)
 assert glob[y,x]==5 and local[y,x]==-1
 m.solid[(0,0)]=32.;glob,local=navigation_arrays(m,{},a,32.)
 assert glob[y,x]==100 and local[y,x]==100


def test_scan_free_history_does_not_record_initial_assumption_as_measurement():
 m=NavigationLayers(size=6.,max_size=6.,bootstrap_override=True)
 m.bootstrap_free(0.,0.,1.,3.,yaw=0.);g=m.views.grid
 navigation_arrays(m,{},m.arrays({}),1.)
 assert not np.isfinite(m.scan_free_seen).any()


def test_unsupported_slope_without_valid_ground_height_never_confirms_block():
 from types import SimpleNamespace as NS
 m=NavigationLayers(size=6.,max_size=6.);m.body_z=0.
 e=NS(height_valid=False,height=None,slope_valid=True,slope_deg=42.)
 m.core.snapshot=lambda stamp:{(0,0):e}
 for stamp in [1.,1.3,1.6,1.9]:
  m.ingest(np.array([[.1,.1,.0]],dtype=np.float32),np.array([8],dtype=np.uint32),stamp)
 assert not m.solid


def test_obstacle_cost_stays_high_then_falls_over_120cm_without_opening_unknown():
 m=NavigationLayers(size=6.,max_size=6.,obstacle_cost_radius=1.2);g=m.views.grid
 g.log_odds[:]=-3.;g.seen[:]=1.;m.solid[(0,0)]=1.
 glob,local=navigation_arrays(m,{},m.arrays({}),1.)
 x,y=g.cell(.1,.1)
 for layer in [glob,local]:
  assert list(layer[y,x:x+7])==[100,97,89,75,56,31,5]
 # Unknown neighbors stay unknown; cost ring is not free-space evidence.
 m=NavigationLayers(size=6.,max_size=6.,obstacle_cost_radius=1.2);m.solid[(0,0)]=1.
 glob,local=navigation_arrays(m,{},m.arrays({}),1.)
 x,y=m.views.grid.cell(.1,.1)
 assert glob[y,x]==100 and glob[y,x+1]==-1 and local[y,x+1]==-1


def test_confident_grass_does_not_confirm_foliage_as_steep_obstacle():
 from types import SimpleNamespace as NS
 m=NavigationLayers(size=6.,max_size=6.,passable_semantic_ids=(3,));m.body_z=0.
 e=NS(height_valid=True,height=.2,slope_valid=True,slope_deg=60.)
 m.core.snapshot=lambda stamp:{(0,0):e}
 pts=np.tile([.1,.1,.2],(20,1)).astype(np.float32)
 for stamp in [1.,1.3,1.6,1.9]:m.ingest(pts,np.full(20,3,dtype=np.uint32),stamp)
 assert not m.solid
 # Genuine obstacle and ground classes still follow the original confirmation.
 m=NavigationLayers(size=6.,max_size=6.,passable_semantic_ids=(3,));m.body_z=0.
 m.core.snapshot=lambda stamp:{(0,0):e}
 for stamp in [1.,1.3,1.6,1.9]:m.ingest(pts,np.full(20,8,dtype=np.uint32),stamp)
 assert (0,0) in m.solid


def test_same_capture_roundoff_keeps_observed_ground_local():
 # Live capture used integer nanoseconds, map formerly split sec/nanosec.
 capture=(373*1000000000+561517242)*1e-9
 now=373+561517242*1e-9
 assert now<capture
 m=NavigationLayers(size=6.,max_size=6.);p=ground_patch(0)
 e=m.ingest(p,np.full(len(p),8,dtype=np.uint32),capture)
 glob,local=navigation_arrays(m,e,m.arrays(e),now)
 observed=(glob>=0)&(glob<80)
 assert observed.any();assert np.all(local[observed]>=0)
 # A truly future observation must still not authorize local travel.
 _,future=navigation_arrays(m,e,m.arrays(e),capture-.01)
 assert not ((future>=0)&(future<80)).any()


def test_same_capture_roundoff_keeps_actual_scan_free_memory():
 capture=(373*1000000000+561517242)*1e-9;now=373+561517242*1e-9
 m=NavigationLayers(size=6.,max_size=6.);g=m.views.grid
 g.log_odds[:]=-3.;g.seen[:]=capture
 glob,local=navigation_arrays(m,{},m.arrays({}),now)
 assert (glob==5).all() and (local==5).all()
 _,expired=navigation_arrays(m,{},m.arrays({}),capture+31.,local_age=30.)
 assert (expired==-1).all()


def test_disabled_recording_skips_both_terrain_and_costmap_files(tmp_path):
 from types import SimpleNamespace as NS
 from nomad_perception.navigation_terrain import NavigationTerrain
 node=NavigationTerrain.__new__(NavigationTerrain)
 node.__dict__.update(directory=str(tmp_path/'not_created'),get_parameter=lambda _:NS(value=False),
  latest_costmaps=(np.zeros((1,1)),np.zeros((1,1))))
 # Exercise the actual subclass save path, including the first valid costmap update.
 for _ in range(3):node.save()
 assert not (tmp_path/'not_created').exists()


def test_enabled_recording_preserves_both_outputs(tmp_path):
 from types import SimpleNamespace as NS
 from nomad_perception.navigation_terrain import NavigationTerrain
 node=NavigationTerrain.__new__(NavigationTerrain)
 a=(np.zeros((1,1)),np.zeros((1,1)),np.zeros((1,1)))
 node.__dict__.update(directory=str(tmp_path/'recorded'),get_parameter=lambda _:NS(value=True),
  latest_costmaps=(a[0],a[1]),layers=NS(views=NS(grid=NS(origin_x=0,origin_y=0,resolution=.2)),bootstrap_origin=None),
  map_stamp=1,scan_stats=dict(received=0,integrated=0,expired=0))
 node.save(a,{'frames':1})
 assert sorted(p.name for p in (tmp_path/'recorded').iterdir())==['costmaps.npz','map.npz','status.json']


def test_disabled_bootstrap_never_invents_observation_even_at_cell_center():
 m=NavigationLayers(size=6.,max_size=6.,bootstrap_override=True);g=m.views.grid
 x=g.origin_x+.5*g.resolution;y=g.origin_y+.5*g.resolution
 before=g.seen.copy();odds=g.log_odds.copy()
 m.bootstrap_free(x,y,1.,0.,yaw=0.)
 assert getattr(m,'bootstrap_origin',None) is None
 np.testing.assert_array_equal(g.seen,before)
 np.testing.assert_array_equal(g.log_odds,odds)
 a=m.arrays({});glob,local=navigation_arrays(m,{},a,1.)
 assert np.all(glob==-1) and np.all(local==-1)
 m.solid[(0,0)]=1.
 glob,local=navigation_arrays(m,{},a,1.)
 cx,cy=g.cell(.1,.1)
 assert glob[cy,cx]==100 and local[cy,cx]==100

def test_obstacle_stage_diagnostics_report_projected_tree_evidence():
 m=NavigationLayers(size=6.,max_size=6.);m.body_z=0.
 p=ground_patch(0)+[0,0,.5]
 for t in [1.,1.3,1.6,1.9]:m.ingest(p,np.ones(len(p),dtype=np.uint32),t)
 d=m.obstacle_diagnostics
 assert d['tree_points']==len(p) and d['rock_points']==0
 assert d['collision_height_points']==len(p)
 assert d['candidate_cells']>0 and d['confirmed_cells']>0


def test_confirmation_at_measured_slow_capture_rate():
 f=TemporalBlockEvidence();k=(0,0)
 assert not f.update(k,True,1.)
 assert not f.update(k,True,1.79)
 assert f.update(k,True,2.58)
 # Three nonblocking captures must not weaken the original clearing rule.
 for t in [4.,4.79,5.58]: assert f.update(k,False,t)
 assert not f.update(k,False,6.37)


def test_sparse_or_inconsistent_observations_do_not_confirm():
 f=TemporalBlockEvidence();k=(0,0)
 for t in [1.,2.6,4.2,5.8]: assert not f.update(k,True,t)
 f=TemporalBlockEvidence()
 for t,b in [(1.,True),(1.5,False),(2.,True),(2.5,False),(3.,True)]:
  assert not f.update(k,b,t)


def test_retained_slope_is_not_new_confirmation_evidence():
 from types import SimpleNamespace as NS
 m=NavigationLayers(size=6.,max_size=6.)
 e=NS(height_valid=True,height=.2,slope_valid=True,slope_deg=25.,slope_reused=True)
 m.core.snapshot=lambda stamp:{(0,0):e}
 pts=np.tile([.1,.1,.2],(20,1)).astype(np.float32)
 for t in [1.,1.79,2.58]:m.ingest(pts,np.full(20,8,dtype=np.uint32),t)
 assert not m.solid and not m.block_evidence.history
 e.slope_reused=False
 for t in [3.37,4.16,4.95]:m.ingest(pts,np.full(20,8,dtype=np.uint32),t)
 assert (0,0) in m.solid


def test_non_ground_capture_cannot_recount_cached_steep_plane():
 from types import SimpleNamespace as NS
 m=NavigationLayers(size=6.,max_size=6.)
 e=NS(height_valid=True,height=.2,slope_valid=True,slope_deg=25.,slope_reused=False)
 m.core.snapshot=lambda stamp:{(0,0):e}
 pts=np.tile([.1,.1,.2],(20,1)).astype(np.float32)
 for t in [1.,1.79,2.58]:m.ingest(pts,np.full(20,0,dtype=np.uint32),t)
 assert not m.solid and not m.block_evidence.history
