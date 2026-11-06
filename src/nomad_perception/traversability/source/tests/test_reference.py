"""Execute the C++ core against the preserved -7 implementation, not a mock."""
from pathlib import Path
import json,sys,subprocess,os,math
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
BASE=Path(os.environ.get('NOMAD_REFERENCE_ARTIFACT',json.loads((ROOT/'reference/baseline.json').read_text())['artifact']))
sys.path.insert(0,str(BASE/'opt/nomad/install/lib/python3.8/site-packages'))
from nomad_perception.real import semantic_validation as sem
from nomad_perception.real import ground_slope_live as ground
from nomad_perception.real.depth_filter import frame_voxels,SORSettings
from nomad_perception.real.ground_reference.fusion_core import weighted_tls
from nomad_perception.real.ground_reference._geometry.traversability_core import fit_plane
from nomad_perception.real.map_core import scan_with_times
from nomad_perception.real.planning_costs import build_cost_snapshot
from nomad_perception.real.oak_rgbd_input import metadata_from_factory

CONFIG=json.loads((ROOT/'config/semantic.real.json').read_text())
SLOPE=json.loads((ROOT/'config/slope.json').read_text())
os.environ['MAPPING_SLOPE_PROFILE']='12cm_3s_optimized'
G,Q,_=ground.configuration()
PROBE=Path(os.environ.get('NOMAD_CORE_PROBE',ROOT/'build-core/nomad_core_probe'))

def invoke(op,**values):
 request=dict(config=CONFIG,slope_config=SLOPE,op=op);request.update(values)
 p=subprocess.run([str(PROBE)],input=json.dumps(request,allow_nan=False),text=True,capture_output=True)
 assert p.returncode==0,p.stderr
 return json.loads(p.stdout)

@pytest.mark.parametrize('angle',[-25.,-10.,-8.,0.,10.,19.999,20.,25.])
@pytest.mark.parametrize('noise',[0.,.003,.04])
def test_tls(angle,noise):
 rng=np.random.default_rng(739)
 xyz=np.column_stack((rng.uniform(-.18,.18,(100,2)),np.zeros(100)))
 xyz[:,2]=np.tan(np.deg2rad(angle))*xyz[:,0]+rng.normal(0,noise,len(xyz))
 weights=rng.uniform(.2,1.,len(xyz))
 expected=weighted_tls(xyz,weights,G,Q)
 actual=invoke('tls',points=xyz.tolist(),weights=weights.tolist())
 assert (expected is None)==(actual is None)
 if expected:
  np.testing.assert_allclose(actual['normal'],expected.normal,atol=1e-12,rtol=0)
  np.testing.assert_allclose([actual['slope_deg'],actual['offset_m'],actual['residual_m'],actual['q_space']],
    [expected.slope_deg,expected.offset_m,expected.residual_m,expected.q_space],atol=1e-10,rtol=0)

@pytest.mark.parametrize('n',[0,10,80,81,6000])
def test_sor_voxel(n):
 rng=np.random.default_rng(14)
 camera=rng.normal(0,.4,(n,3));camera[:,2]+=2
 world=camera+np.array([-1.,-.7,.3])
 expected=frame_voxels(camera,world,SORSettings())[1]
 actual=np.asarray(invoke('filter',camera=camera.tolist(),points=world.tolist())).reshape(-1,3)
 np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=0)

def test_projection_resize_boundary_and_rounding():
 c=sem.load_config(ROOT/'config/semantic.real.json',require_confirmations=False)
 c['label_image'].update(width=16,height=8,source_width=32,source_height=16)
 labels=np.full((8,16),4,np.uint8);labels[:,8:]=7;labels[3:5,4:6]=0
 K=np.array([[10,0,16],[0,10,8],[0,0,1.]])
 rng=np.random.default_rng(499);p=rng.normal(0,2.,(1000,3));p[:,2]+=3
 special=np.array([[(-16+v)/10,0,1] for v in [-.5,0.,.5,1.5,2.5,15.5,16.5,31.5,32.5]])
 p=np.concatenate([p,special]);expected=sem.transfer(p,labels,None,K,(np.eye(3),np.zeros(3)),c,image_shape=(16,32))[0]
 actual=invoke('project',points=p.tolist(),label_width=16,label_height=8,labels=labels.ravel().tolist(),K=K.tolist(),camera_pose=np.eye(4).tolist(),image_width=32,image_height=16)
 np.testing.assert_array_equal(actual,expected)

def test_factory_calibration_exact_chain():
 factory=json.loads((ROOT/'tests/fixtures/oak_factory_reference.json').read_text())
 expected=metadata_from_factory(factory,CONFIG)['calibration_provenance']
 actual=invoke('factory',factory=factory)
 np.testing.assert_allclose(actual['imu_b'],expected['T_raw_imu_cameras'][0],atol=1e-12,rtol=0)
 np.testing.assert_allclose(actual['imu_a'],expected['T_raw_imu_cam_a'],atol=1e-12,rtol=0)

def test_semantic_dynamic_priority_and_deduplicated_camera_support():
 c=sem.load_config(ROOT/'config/semantic.real.json',require_confirmations=False)
 grid=sem.SemanticGrid(c);grid.frame='odom';frames=[];expected=[]
 for i,(ids,label,now) in enumerate([([4,4],1_000_000_000,1_000_000_000),([1,4],1_200_000_000,1_200_000_000),([4,4],1_000_000_000,2_500_000_000),([4,4],2_600_000_000,2_600_000_000),([7,-1],2_700_000_000,2_700_000_000)]):
  p=np.array([[-.01,-.01,1.],[-.02,-.02,1.]])
  stamps=np.array([1_000_000_000,1_000_000_001]);ls=np.array([label,label]);stamp=1_000_000_000+i*200_000_000
  grid.add(p,np.array(ids),stamp,i%2,point_stamps=stamps,camera_stamp_ns=label,semantic_stamps=ls)
  frames.append(dict(points=p.tolist(),stamp_ns=stamp,source=i%2,point_stamps=stamps.tolist(),labels=ids,label_stamps=ls.tolist(),camera_stamp=label,now_ns=now))
  expected.append(json.loads(json.dumps(grid.snapshot(now)['cells'])))
 actual=invoke('semantic',frames=frames)
 for got,want in zip(actual,expected):assert got['cells']==want

@pytest.mark.parametrize('kind',['plane','step','revision','turn','capacity'])
def test_ground_history_matches_reference(kind):
 mounts=np.array([np.eye(4),np.eye(4)]);mounts[1,:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
 ref=ground.GroundSlope(mounts,0.,1_000_000_000,semantic_prefiltered=True,frame='odom')
 x,y=np.meshgrid(np.arange(.06,1.51,.03),np.arange(-.6,.61,.03));points=np.c_[x.ravel(),y.ravel(),np.tan(np.deg2rad(10))*x.ravel()]
 frames=[];expected=[]
 for i in range(8):
  stamp=1_000_000_000+i*200_000_000;source=i%2;p=points.copy()
  if kind=='step':p[p[:,0]>.73,2]+=.2
  if kind=='capacity':p=np.repeat(p,10,axis=0)
  if kind=='turn' and i>4:quat=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
  else:quat=[0,0,0,1]
  pose=[stamp,0,0,0,*quat]
  revision=kind=='revision' and i>=4
  if revision:stamp=1_000_000_000+(i-4)*200_000_000;p=p[p[:,1]<0]
  f=dict(stamp_ns=stamp,source=source,points=p.tolist(),pose=pose,revision=revision)
  (ref.revise if revision else ref.ingest)(p,stamp,source,None,pose)
  snap=ref.snapshot();expected.append(ground.sparse_payload(snap));frames.append(f)
 actual=invoke('ground',frames=frames,mounts=mounts.tolist())
 for a,b in zip(actual,expected):
  aa=a['modes']['fused']['cells'];bb=b['cells'];assert len(aa)==len(bb)
  np.testing.assert_array_equal(np.array([[r[0],r[1],r[5],r[7]] for r in aa]),np.array([[r[0],r[1],r[5],r[7]] for r in bb]))
  for x,y in zip(aa,bb):
   for ix in (2,3,4,6,8):
    assert (x[ix] is None)==(y[ix] is None),(kind,x,y)
    if x[ix] is not None:assert abs(x[ix]-y[ix])<=2e-5,(kind,ix,x,y)

@pytest.mark.parametrize('grade',[-11.,-10.,-8.,0.,10.,19.999999,20.,25.,None])
@pytest.mark.parametrize('semantic',[0,20,35,100,None])
def test_final_cost_masks_rounding_and_support(grade,semantic):
 row=[-1,-1,grade,0.,1.,False,1.,1_000_000_000,abs(grade) if grade is not None else None]
 live=dict(modes=dict(fused=dict(resolution_m=.12,columns=['ix','iy','slope_deg','height_m','confidence','slope_reused','last_calculated_stamp_s','last_observed_stamp_ns','slope_magnitude_deg'],cells=[row])),metadata=dict(frame_id='odom',slope_value_semantics='signed_longitudinal_deg'))
 semantics=dict(resolution_m=.12,frame='odom',cells=[dict(ix=-1,iy=-1,cost=semantic,last_semantic_ns=1_000_000_000,camera_observed_ns=1_000_000_000,camera_capture_ns=[400_000_000,600_000_000,1_000_000_000])])
 ref=build_cost_snapshot(live,semantics);actual=invoke('cost',live=live,semantics=semantics)
 for key in ('global_costmap','local_costmap','camera_support','camera_capture_count','camera_capture_span_ns','slope_observed_ns','semantic_observed_ns','camera_observed_ns'):
  np.testing.assert_array_equal(actual[key],ref[key].ravel())
 for key in ('slope_deg','slope_magnitude_deg','slope_cost','terrain_cost'):
  np.testing.assert_allclose([np.nan if v is None else v for v in actual[key]],ref[key].ravel(),rtol=0,atol=1e-7,equal_nan=True)

@pytest.mark.parametrize('seed',range(10))
def test_random_ground_expiry_reuse_sensor_balance(seed):
 rng=np.random.default_rng(seed);mounts=np.array([np.eye(4),np.eye(4)]);mounts[1,:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
 ref=ground.GroundSlope(mounts,0.,1_000_000_000,semantic_prefiltered=True,frame='odom')
 frames=[];expected=[]
 for i in range(15):
  stamp=1_000_000_001+i*400_000_003;source=i%2
  xy=rng.uniform([-.3,-.55],[1.2,.55],(750,2));z=np.tan(np.deg2rad(seed*3-12))*xy[:,0]+rng.normal(0,.002,len(xy))
  if seed%3==0:z[xy[:,0]>.6]+=.2
  points=np.column_stack((xy,z));pose=[stamp,i*.015,0.,0.,0.,0.,0.,1.]
  if i>9 and seed%2==0:points=points[:20]
  ref.ingest(points,stamp,source,None,pose);expected.append(ground.sparse_payload(ref.snapshot()))
  frames.append(dict(stamp_ns=stamp,source=source,points=points.tolist(),pose=pose))
 actual=invoke('ground',frames=frames,mounts=mounts.tolist())
 for a,b in zip(actual,expected):
  aa=a['modes']['fused']['cells'];bb=b['cells'];assert len(aa)==len(bb)
  for x,y in zip(aa,bb):
   assert x[:2]==y[:2] and x[5]==y[5] and x[7]==y[7],(seed,x,y)
   for ix in (2,3,4,6,8):
    assert (x[ix] is None)==(y[ix] is None),(seed,ix,x,y)
    if x[ix] is not None:assert abs(x[ix]-y[ix])<=3e-5,(seed,ix,x,y)

@pytest.mark.parametrize('offset',[0,1,50_000_001,199_999_999,200_000_000])
def test_integer_ns_pose_interpolation(offset):
 from nomad_perception.real.semantic_validation import pose_at
 start=1793870000000000001
 rows=[[start,0,0,0,0,0,0,1],[start+200_000_000,1,2,3,0,0,math.sin(.4),math.cos(.4)]]
 expected=pose_at(rows,start+offset,200_000_000);actual=invoke('pose',poses=rows,stamp_ns=start+offset)
 # pose_at returns rotation and translation in the reference.
 R,t=expected
 np.testing.assert_allclose(actual[1:4],t,rtol=0,atol=1e-14)
 from scipy.spatial.transform import Rotation
 np.testing.assert_allclose(Rotation.from_quat(actual[4:]).as_matrix(),R,rtol=0,atol=1e-14)

def test_local_reobservation_unknown_dynamic_and_original_point_stamps():
 from collections import Counter,deque
 from types import SimpleNamespace
 from nomad_perception.real.local_ground import LocalGround
 c=sem.load_config(ROOT/'config/semantic.real.json',require_confirmations=False)
 c['label_image'].update(width=32,height=16,source_width=64,source_height=32)
 K=np.array([[30,0,32],[0,30,16],[0,0,1.]])
 mounts=np.array([np.eye(4),np.eye(4)]);mounts[1,:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
 bridge=SimpleNamespace(c=c,info=(64,32,K),grid=sem.SemanticGrid(c),counts=Counter(),labels=deque(),settings=SORSettings(),last_stamp=0,view_timing=lambda *a:None)
 bridge.grid.frame='odom'
 bridge.camera_view=lambda rows,mount,lt,labels:(lt,labels,None,None,sem.camera_pose(rows,lt,mount,200_000_000),None)
 ref=LocalGround(bridge)
 rng=np.random.default_rng(818);p=rng.uniform([.2,-.2,1.5],[1.,.2,2.3],(180,3))
 events=[];expected=[]
 for i in range(10):
  now=1_000_000_001+i*500_000_000
  rows=[[t,0,0,0,0,0,0,1] for t in range(1_000_000_001,now+1,100_000_000)]
  label=np.full((16,32),4,np.uint8);label[:,16:]=[7,1,0,4,4][min(i//2,4)]
  bridge.labels.append((now,label));frames=[]
  if i<7:
   source=i%2;capture=now-100_000_000 if i else now
   points=p.copy();stamps=np.full(len(p),capture,np.int64)
   if not source:stamps+=np.arange(len(p),dtype=np.int64)*100
   context=(rows[-1],mounts,0.,1_000_000_001)
   ref.submit(points,capture,source,points,context,point_stamps=stamps if source==0 else None)
   frames.append(dict(stamp_ns=capture,source=source,points=points.tolist(),optical=points.tolist(),point_stamps=stamps.tolist()))
  got=[]
  def capture_output(points,stamp,source,context,timing,point_stamps,revision):
   got.append(dict(stamp_ns=stamp,source=source,points=points.tolist(),point_stamps=point_stamps.tolist()))
  ref.process(rows,np.eye(4),None,output_timed=capture_output)
  expected.append(dict(ground=got,semantic=json.loads(json.dumps(bridge.grid.snapshot(now)))))
  events.append(dict(poses=rows,frames=frames,label=dict(stamp_ns=now,width=32,height=16,ids=label.ravel().tolist())))
 actual=invoke('local',config=dict(CONFIG,label_image=c['label_image']),events=events,mounts=mounts.tolist(),mount=np.eye(4).tolist(),K=K.tolist(),width=64,height=32)
 for a,b in zip(actual,expected):
  assert len(a['ground'])==len(b['ground'])
  for x,y in zip(a['ground'],b['ground']):
   assert x['stamp_ns']==y['stamp_ns'] and x['point_stamps']==y['point_stamps'] and x['source']==y['source']
   np.testing.assert_allclose(np.array(x['points']).reshape(-1,3),np.array(y['points']).reshape(-1,3),atol=1e-12,rtol=0)
  assert a['semantic']['cells']==b['semantic']['cells']

@pytest.mark.parametrize('seed',range(5))
def test_lidar_scan_bin_last_sample_and_ray_time_deskew(seed):
 from nomad_perception.real.map_core import deskew as reference_deskew
 rng=np.random.default_rng(seed+398);n=500
 angles=np.linspace(-2.9,2.9,n);ranges=rng.uniform(.6,3.,n)
 points=np.c_[ranges*np.cos(angles),ranges*np.sin(angles),np.zeros(n)]
 scan=dict(stamp_ns=1793870000000000001,angle_min=-math.pi,angle_increment=2*math.pi/720,range_min=.28,range_max=16.)
 bins=np.ceil((angles-scan['angle_min'])/scan['angle_increment']).astype(int)
 values=[None]*721
 for b,p in zip(bins,points):values[b]=float(np.linalg.norm(p[:2]))
 scan['ranges']=values
 cloud=dict(stamp_ns=scan['stamp_ns'],points=points.tolist(),offsets=np.linspace(0,.04,n).tolist(),received_monotonic=1.)
 mount=np.eye(4);mount[:3,:3]=[[0,-1,0],[0,0,-1],[1,0,0]];mount[:3,3]=[.05,-.1,.02]
 expected=scan_with_times(scan,cloud,mount);actual=invoke('scan',scan=scan,cloud=cloud,calibration=mount.tolist())
 for key in ('points_lidar_m','point_time_offsets_s','bin_indices','input_points','matched_bins','rear_discarded','unmatched_points'):
  np.testing.assert_allclose(actual[key],expected[key],atol=1e-14,rtol=0)
 ns=scan['stamp_ns'];poses=[[ns-50_000_000,0,0,0,0,0,0,1],[ns,0.01,.02,0,0,0,0,1],[ns+100_000_000,.05,.04,.01,0,0,math.sin(.1),math.cos(.1)]]
 pair={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in expected.items()}
 got=invoke('deskew',pair=pair,poses=poses,mount=mount.tolist())
 want=reference_deskew(expected,poses,np.eye(4),mount)
 np.testing.assert_allclose(got['points'],want,atol=2e-14,rtol=0)
 np.testing.assert_array_equal(got['point_stamps'],ns+np.rint(expected['point_time_offsets_s']*1e9).astype(np.int64))

@pytest.mark.parametrize('query',[999_999_999,1_000_000_001,1_500_000_001])
def test_missing_pose_and_excessive_bracket_are_rejected(query):
 request=dict(config=CONFIG,slope_config=SLOPE,op='pose',poses=[[1_000_000_000,0,0,0,0,0,0,1],[1_500_000_000,0,0,0,0,0,0,1]],stamp_ns=query)
 p=subprocess.run([str(PROBE)],input=json.dumps(request),text=True,capture_output=True)
 assert p.returncode==2 and ('pose_outside_history' in p.stderr or 'pose_gap' in p.stderr)

def test_cached_semantic_version_changes_at_original_dynamic_expiry():
 c=sem.load_config(ROOT/'config/semantic.real.json',require_confirmations=False);ref=sem.SemanticGrid(c);ref.frame='odom';events=[];versions=[]
 p=np.array([[.01,.01,1.]])
 for i,(capture,label,now) in enumerate([(1_000_000_000,1,1_000_000_000),(1_000_000_000,-1,2_000_000_000),(1_000_000_000,-1,2_000_000_001),(3_000_000_000,4,3_000_000_000)]):
  f=dict(points=p.tolist(),stamp_ns=capture,source=0,labels=[label],label_stamps=[capture],camera_stamp=capture,now_ns=now)
  ref.add(p,np.array([label]),capture,0,semantic_stamps=np.array([capture]),camera_stamp_ns=capture);versions.append(ref.version(now));events.append(f)
 got=invoke('semantic',frames=events)
 assert [v['version'] for v in got]==versions

@pytest.mark.parametrize('ids',[(5,4),(4,5),(8,5),(2,1),(7,3)])
def test_equal_time_equal_priority_preserves_first_class_insertion(ids):
 c=sem.load_config(ROOT/'config/semantic.real.json',require_confirmations=False);ref=sem.SemanticGrid(c);ref.frame='odom'
 p=np.array([[.01,.01,1.]])
 frames=[];expected=[]
 for i,id in enumerate(ids):
  label=1_200_000_000;capture=1_000_000_000+i*50_000_000
  ref.add(p,np.array([id]),capture,i%2,semantic_stamps=np.array([label]),camera_stamp_ns=label)
  expected.append(json.loads(json.dumps(ref.snapshot(label)['cells'])))
  frames.append(dict(points=p.tolist(),stamp_ns=capture,source=i%2,labels=[id],label_stamps=[label],camera_stamp=label,now_ns=label))
 got=invoke('semantic',frames=frames)
 for a,b in zip(got,expected):assert a['cells']==b
 assert got[-1]['cells'][0]['class_id']==ids[0]

@pytest.mark.parametrize('case',['base','raw_k_diff','nonzero_tx','bad_R','R_diagonal_tolerance','bad_D','D_tolerance_edge','wrong_frame','wrong_model','wrong_size','bad_P_length','within_pixel_tolerance'])
def test_rectified_camera_info_uses_P_and_matches_original_validation(case):
 from collections import Counter
 from types import SimpleNamespace
 import copy
 info=dict(frame=CONFIG['zed_optical_frame'],width=1280,height=720,k=CONFIG['expected_zed_camera_info']['K'][:],r=np.eye(3).ravel().tolist(),d=[0.]*5,distortion_model='plumb_bob')
 P=np.zeros((3,4));P[:3,:3]=np.array(info['k']).reshape(3,3);info['p']=P.ravel().tolist()
 if case=='raw_k_diff':info['k'][0]+=10
 if case=='nonzero_tx':info['p'][3]=1e-4
 if case=='bad_R':info['r'][1]=2e-5
 if case=='R_diagonal_tolerance':info['r'][0]+=1.9e-5
 if case=='bad_D':info['d'][0]=1.1e-8
 if case=='D_tolerance_edge':info['d'][0]=1e-8
 if case=='wrong_frame':info['frame']='right_camera'
 if case=='wrong_model':info['distortion_model']='rational_polynomial'
 if case=='wrong_size':info['width']=640
 if case=='bad_P_length':info['p']=info['p'][:-1]
 if case=='within_pixel_tolerance':info['p'][0]+=5e-5
 c=sem.load_config(ROOT/'config/semantic.real.json',require_confirmations=False)
 b=SimpleNamespace(c=c,optical_frame=c['zed_optical_frame'],info=None,info_signature=None,counts=Counter())
 m=SimpleNamespace(**info,header=SimpleNamespace(frame_id=info['frame']))
 sem.SemanticBridge.camera_info(b,m)
 got=invoke('camera_info',frames=[info])[0]
 assert got['valid']==(b.info is not None),(case,got,b.counts)
 if got['valid']:np.testing.assert_array_equal(got['K'],b.info[2].ravel())

def test_camera_info_freeze_survives_invalid_packet_and_restores_original():
 from collections import Counter
 from types import SimpleNamespace
 import copy
 c=sem.load_config(ROOT/'config/semantic.real.json',require_confirmations=False)
 info=dict(frame=CONFIG['zed_optical_frame'],width=1280,height=720,k=CONFIG['expected_zed_camera_info']['K'][:],r=np.eye(3).ravel().tolist(),d=[0.]*5,distortion_model='plumb_bob')
 P=np.zeros((3,4));P[:3,:3]=np.array(info['k']).reshape(3,3);info['p']=P.ravel().tolist()
 invalid=copy.deepcopy(info);invalid['r'][1]=.01
 changed=copy.deepcopy(info);changed['p'][0]+=1e-5
 frames=[info,invalid,changed,info]
 b=SimpleNamespace(c=c,optical_frame=c['zed_optical_frame'],info=None,info_signature=None,counts=Counter());expected=[]
 for value in frames:
  sem.SemanticBridge.camera_info(b,SimpleNamespace(**value,header=SimpleNamespace(frame_id=value['frame'])))
  expected.append(b.info is not None)
 assert expected==[True,False,False,True]
 assert [v['valid'] for v in invoke('camera_info',frames=frames)]==expected
