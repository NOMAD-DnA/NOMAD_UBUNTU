from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import json
import numpy as np
import pytest
from nomad_perception.traversability_core import CoreConfig,CLASS_COSTS
from nomad_perception.observed_grid import ObservedGrid
from nomad_perception.semantic_input import encode_semantic_cloud,decode_semantic_cloud
from nomad_perception.traversability_output import transform_points
from nomad_semantic_mapping.core import SemanticCore
from nomad_semantic_mapping.archive import write_snapshot,load_snapshot,resolve_snapshot,EXPORT_FILES
from nomad_semantic_mapping.labels import build_displays,SemanticLabels
from nomad_semantic_mapping.mapper import SemanticMapper
from nomad_semantic_mapping.palette import NAMES,PALETTE
from nomad_semantic_mapping.session import archive_status

def xyz(n,x=.05,y=.05):return np.tile([x,y,.3],(n,1)).astype(np.float32)
def feed(ids,config=None):
    c=SemanticCore(config);c.ingest(xyz(len(ids)),np.array(ids),1.);return c,c.snapshot(1.)[(0,0)]

def test_cost_uses_all_eligible_points_instead_of_representative():
    c,e=feed([31]*80+[40]*20)
    assert e.representative_class==31 and e.semantic_raw==36 and e.output_cost==36 and e.q_s==1
    assert e.total_points==100 and e.eligible_points==100

@pytest.mark.parametrize('soil,expected',[(55,40),(56,31),(50,40)])
def test_near_tie_prefers_nonground_within_ten_percentage_points(soil,expected):
    _,e=feed([31]*soil+[40]*(100-soil));assert e.representative_class==expected

@pytest.mark.parametrize('ids,expected',[( [51]*10+[40]*10,40),([28]*10+[40]*10,28),([28]*11+[40]*10,28)])
def test_representative_priority_cost_count_then_id(ids,expected):
    _,e=feed(ids);assert e.representative_class==expected

def test_excluded_classes_affect_representative_but_not_cost_or_q():
    _,e=feed([0]*20+[31]*5)
    assert e.representative_class==0 and e.semantic_raw==20 and e.q_s==.25 and e.output_cost==35

@pytest.mark.parametrize('ids,raw,score',[( [31],20,39),([40],100,100),([51]*5,65,71),([0]*20,None,100)])
def test_sparse_support_and_true_100_are_distinct(ids,raw,score):
    _,e=feed(ids);assert e.semantic_raw==raw and e.output_cost==score
    assert (e.representative_class is not None)==(len(ids)>=20)

def test_last_usable_map_and_original_q_stamps_survive_expiry():
    c,e=feed([31]*20);e2=c.snapshot(12.)[(0,0)]
    assert c.active_point_count==0 and e2.semantic_reused and e2.representative_reused
    assert (e2.semantic_raw,e2.q_s,e2.semantic_stamp_s,e2.representative_class)==(20,1,1,31)
    assert e2.semantic_support_points==20 and e2.representative_stamp_s==1
    c.ingest(xyz(1),np.array([51]),13.)
    e3=c.snapshot(13.)[(0,0)]
    assert e3.semantic_raw==65 and e3.q_s==.05 and e3.representative_class==31 and e3.representative_reused


def test_bounded_retention_is_deterministic_not_pixel_order_cutoff():
    config=CoreConfig(per_cell_max_points=32,global_max_points=48)
    clouds=np.r_[xyz(120),xyz(120,.25)];ids=np.tile(np.arange(60)%5+27,4)
    a=SemanticCore(config);b=SemanticCore(config)
    for c in [a,b]:c.ingest(clouds,ids,1.);c.snapshot(1.)
    assert a.active_point_count==48 and all(len(v.points)<=32 for v in a._cells.values())
    for k in a._cells:np.testing.assert_array_equal(a._cells[k].points,b._cells[k].points)
    assert a.diagnostics['global_cap_evicted_points']>0 and a.diagnostics['per_cell_cap_evicted_points']>0


def test_clock_reset_does_not_mix_drives():
    c,_=feed([31]*20);c.ingest(xyz(20,.25),np.full(20,40),.5);s=c.snapshot(.5)
    assert set(s)=={(1,0)} and s[(1,0)].representative_class==40 and c.diagnostics['clock_backward_resets']==1


def test_no_plane_fit_and_no_geometry_dependency(monkeypatch):
    c,e=feed([40]*20)
    monkeypatch.setattr(c,'_raw_index',lambda:pytest.fail('Semantic snapshot attempted geometry'))
    c.ingest(xyz(20,.25),np.full(20,51),2.)
    assert c.snapshot(2.)[(1,0)].semantic_raw==65 and not c.diagnostics['plane_fit_attempts']


def test_capture_time_optical_transform_and_negative_cell_floor():
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Transform
    header=Header(stamp=Time(sec=7,nanosec=123),frame_id='oak_depth_optical_frame')
    msg=encode_semantic_cloud(np.array([[0.,0.,1.]]*20),np.full(20,31,dtype=np.uint32),header)
    t=Transform();t.translation.x=-.01;t.rotation.w=1.
    calls=[]
    def lookup(target,source,stamp):
        calls.append((target,source,stamp.nanoseconds));return NS(transform=t)
    core=SemanticCore();views=NS(accept_points=lambda p:np.ones(len(p),bool))
    n=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=7_000_000_123)),last_clock=None,fault=None,lock=__import__('threading').Lock(),
         pending=__import__('collections').deque([(msg,__import__('time').monotonic())]),last_input_stamp=-np.inf,
         tf_wait=.5,max_age=.6,maximum_pending=2,minimum_z=.4,maximum_z=8.,frame='odom',buffer=NS(lookup_transform=lookup),
         views=NS(accept_points=views.accept_points,grid=ObservedGrid(.2,2.,0.,4.)),counts=__import__('collections').Counter(),
         core=core,version=0,last_checkpoint=__import__('time').monotonic(),checkpoint_period=5.)
    SemanticMapper.process_pending(n,publish=False)
    assert calls==[('odom','oak_depth_optical_frame',7_000_000_123)]
    assert (-1,0) in n.estimates and n.estimates[(-1,0)].representative_class==31
    assert n.counts['processed_frames']==1


def snapshot_data():
    c,_=feed([31]*80+[40]*20);c.ingest(xyz(20,.25),np.full(20,51),2.);c.ingest(xyz(1,.45),np.array([31]),2.)
    a=c.layers(ObservedGrid(.2,2.,0.,4.),c.snapshot(2.));a['trajectory']=np.array([[2.,.05,.05,0.,0.,0.,0.,1.]],np.float64)
    m={'schema':'nomad-semantic-mapping-v1','frame_id':'odom','origin_x':-1.,'origin_y':-1.,'resolution_m':.2,'version':1,'stamp_s':2.,'map_id':'synthetic_validation'}
    return a,m

def test_atomic_archive_and_separate_cost_class_exports(tmp_path):
    a,m=snapshot_data();checkpoint=write_snapshot(tmp_path,a,m);final=write_snapshot(tmp_path,a,m,final=True)
    b,meta=load_snapshot(final)
    for k in a:np.testing.assert_array_equal(a[k],b[k])
    assert meta['status']=='closed' and resolve_snapshot(tmp_path)==final
    assert all((tmp_path/name).is_file() for name in EXPORT_FILES)
    assert '31' in (tmp_path/'numbered_class_map.svg').read_text()
    assert 'semantic_raw' in (tmp_path/'cells.csv').read_text()
    assert archive_status(tmp_path)['final_save_complete']
    bad={**a,'representative_class':a['representative_class'].astype(np.uint8)}
    before=checkpoint.read_bytes()
    with pytest.raises(ValueError):write_snapshot(tmp_path,bad,m)
    assert checkpoint.read_bytes()==before

def test_display_class_ids_not_cost_values_and_pair_same_version():
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import Image
    msg=OccupancyGrid();msg.header.frame_id='odom';msg.header.stamp.sec=2;msg.info.width=3;msg.info.height=1;msg.info.resolution=.2;msg.data=[36,65,-1]
    cls=np.array([[31,51,-1]],np.int16)
    costs,colors,labels=build_displays(msg,cls)
    assert len(costs.points)==len(colors.points)==2 and [m.text for m in labels.markers[1:]]==['31','51']
    im=Image();im.header=msg.header;im.encoding='16SC1';im.height=1;im.width=3;im.step=6;im.data=cls.astype('<i2').tobytes()
    n=NS(costs={},classes={},ready=None,dirty=False)
    n.pair=lambda key:SemanticLabels.pair(n,key)
    SemanticLabels.on_cost(n,msg);assert n.ready is None
    wrong=Image();wrong.header.frame_id='odom';wrong.header.stamp.sec=1;wrong.encoding=im.encoding;wrong.height=im.height;wrong.width=im.width;wrong.step=im.step;wrong.data=im.data
    SemanticLabels.on_class(n,wrong);assert n.ready is None
    SemanticLabels.on_class(n,im);assert n.ready and n.dirty

def test_original_vocabulary_matches_unchanged_shared_core():
    p=Path(__file__).resolve().parents[2]/'config/semantic_class_costs.json'
    d=json.loads(p.read_text());assert len(NAMES)==len(PALETTE)==64
    for r in d['classes']:
        if r['semantic_cost'] is None:assert np.isnan(CLASS_COSTS[r['id']])
        else:assert CLASS_COSTS[r['id']]==r['semantic_cost']


def test_zero_time_never_uses_latest_tf():
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from collections import Counter,deque
    import threading
    n=NS(counts=Counter(),lock=threading.Lock(),pending=deque(),maximum_pending=2)
    msg=PointCloud2();msg.header=Header(frame_id='oak_depth_optical_frame')
    SemanticMapper.on_points(n,msg)
    assert n.counts['invalid_input']==1 and not n.pending


def test_same_xy_different_heights_keep_class_correspondence():
    c=SemanticCore();points=xyz(40);points[20:,2]=3.
    c.ingest(points,np.r_[np.full(20,31),np.full(20,27)],1.)
    e=c.snapshot(1.)[(0,0)]
    assert e.semantic_raw==60 and e.representative_class==27 and e.total_points==40


def test_retention_expiry_marks_changed_cells_without_new_frame():
    c,_=feed([31]*20)
    assert c.last_snapshot_changed_keys==frozenset({(0,0)})
    c.snapshot(2.);assert not c.last_snapshot_changed_keys
    c.snapshot(12.);assert c.last_snapshot_changed_keys==frozenset({(0,0)})
