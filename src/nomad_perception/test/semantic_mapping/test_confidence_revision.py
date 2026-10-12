"""Observation lifetime, semantic boundaries and bounded final-cost regressions."""
from dataclasses import replace
from types import SimpleNamespace
import math
import numpy as np
import pytest
from nomad_perception.traversability_core import CoreConfig
from nomad_semantic_mapping.core import SemanticCore
from nomad_semantic_mapping.cost_policy import SemanticCostPolicy, NEIGHBORS
from nomad_semantic_mapping.mapper import SemanticMapper
from nomad_perception.observed_grid import ObservedGrid


def observe(core, key, ids, stamp):
    x,y=key
    points=np.tile([.2*(x+.25),.2*(y+.25),.3],(len(ids),1)).astype(np.float32)
    core.ingest(points,np.asarray(ids),stamp)


def ring(core, ids=None, stamp=1., skip=None):
    for key in NEIGHBORS:
        if key!=skip:observe(core,key,ids or [51]*16,stamp)


def test_partial_then_full_expiry_keeps_observation_support_and_cost():
    c=SemanticCore()
    observe(c,(0,0),[51]*16,1.)
    observe(c,(0,0),[51]*7,2.)
    before=c.snapshot(2.)[(0,0)]
    partial=c.snapshot(11.1)[(0,0)]
    empty=c.snapshot(12.1)[(0,0)]
    assert [e.total_points for e in [before,partial,empty]]==[23,7,0]
    for e in [before,partial,empty]:
        assert (e.semantic_raw,e.q_s,e.semantic_support_points,e.output_cost)==(65,1,23,65)
        assert (e.representative_class,e.representative_support_points)==(51,23)
        assert e.semantic_stamp_s==e.representative_stamp_s==2.
        assert e.semantic_support_histogram==((51,23),)
    assert partial.semantic_reused and partial.representative_reused
    assert empty.semantic_reused and empty.representative_reused
    assert partial.class_histogram==((51,7),) and empty.class_histogram==()


def test_new_sparse_observation_updates_instead_of_freezing_old_cost():
    c=SemanticCore();observe(c,(0,0),[31]*20,1.);c.snapshot(1.)
    c.snapshot(12.);observe(c,(0,0),[51]*7,13.)
    e=c.snapshot(13.)[(0,0)]
    assert (e.semantic_raw,e.q_s,e.semantic_support_points,e.output_cost)==(65,.35,7,71)
    assert not e.semantic_reused and e.representative_reused
    assert e.semantic_stamp_s==13. and e.representative_stamp_s==1.


def test_empty_or_excluded_only_new_capture_does_not_recompute_semantics():
    c=SemanticCore();observe(c,(0,0),[51]*16,1.)
    observe(c,(0,0),[51]*7,2.);c.snapshot(2.)
    observe(c,(0,0),[0]*20,11.1)
    e=c.snapshot(11.1)[(0,0)]
    assert (e.total_points,e.eligible_points)==(27,7)
    assert (e.semantic_raw,e.q_s,e.semantic_support_points,e.output_cost)==(65,1,23,65)
    assert e.semantic_reused and e.semantic_stamp_s==2.
    c.ingest(np.empty((0,3)),np.empty(0),12.1)
    assert c.snapshot(12.1)[(0,0)].q_s==1.


def test_global_cap_eviction_is_not_an_observation_in_evicted_cell():
    c=SemanticCore(CoreConfig(global_max_points=23))
    observe(c,(0,0),[51]*16,1.);observe(c,(0,0),[51]*7,2.)
    c.snapshot(2.);observe(c,(10,10),[31]*16,3.)
    e=c.snapshot(3.)[(0,0)]
    assert e.total_points==7 and c.active_point_count==23
    assert (e.semantic_raw,e.q_s,e.semantic_support_points,e.output_cost)==(65,1,23,65)
    assert e.semantic_reused and e.representative_reused


def test_capture_is_committed_before_snapshot_can_expire_all_points():
    c=SemanticCore();observe(c,(0,0),[51]*23,1.)
    e=c.snapshot(12.)[(0,0)]
    assert e.total_points==0 and e.output_cost==65 and e.semantic_support_points==23


@pytest.mark.parametrize('q,score',[(1.,65),(.35,71),(0.,74),(.9,66)])
def test_bounded_penalty_examples(q,score):
    value=SemanticCostPolicy().adjusted(65.,q)
    assert math.floor(value+.5)==score


def test_true_100_and_no_evidence_keep_existing_output_contract():
    c=SemanticCore();observe(c,(0,0),[40],1.);ring(c)
    e=c.snapshot(1.)[(0,0)]
    assert e.output_cost==100 and not e.confidence_neighbor_corrected
    c=SemanticCore();observe(c,(0,0),[0]*20,1.);ring(c)
    e=c.snapshot(1.)[(0,0)]
    assert e.output_cost==100 and e.semantic_raw is None
    a=c.layers(ObservedGrid(.2,2.,0.,4.),c.snapshot(1.))
    assert (a['cost']==-1).any()


def test_all_eight_compatible_neighbors_correct_q_without_changing_raw_cost():
    c=SemanticCore();observe(c,(0,0),[51]*7,1.);ring(c)
    e=c.snapshot(1.)[(0,0)]
    assert e.q_s==.35 and e.effective_q_s==pytest.approx(.8)
    assert e.output_cost==67 and e.semantic_raw==65
    assert e.confidence_neighbor_corrected and e.confidence_neighbor_support_cells==8
    assert e.semantic_support_points==7 and e.representative_class is None


@pytest.mark.parametrize('kind',['missing','weak','different','new_high_cost','high_cost_share'])
def test_neighbor_rejection_preserves_target_cost(kind):
    c=SemanticCore();own=[51]*7;other=[51]*16
    if kind=='high_cost_share':own=[51]*6+[40];other=[51]*18+[40]*2
    observe(c,(0,0),own,1.)
    ring(c,other,skip=(1,1))
    replacement={'weak':[51]*13,'different':[31]*20,'new_high_cost':[51]*19+[40]}
    if kind!='missing':observe(c,(1,1),replacement.get(kind,other),1.)
    e=c.snapshot(1.)[(0,0)]
    assert not e.confidence_neighbor_corrected and e.effective_q_s==e.q_s


def test_neighbor_updates_refresh_an_existing_target_without_target_observation():
    c=SemanticCore();observe(c,(0,0),[51]*7,1.);ring(c,skip=(1,1))
    assert c.snapshot(1.)[(0,0)].output_cost==71
    observe(c,(1,1),[51]*16,2.)
    e=c.snapshot(2.)[(0,0)]
    assert (0,0) in c.last_snapshot_changed_keys
    assert e.output_cost==67 and e.semantic_stamp_s==1. and e.semantic_support_points==7


def test_recent_neighbor_evidence_expires_without_destroying_original_support():
    c=SemanticCore();ring(c,stamp=1.);observe(c,(0,0),[51]*7,2.)
    assert c.snapshot(2.)[(0,0)].confidence_neighbor_corrected
    e=c.snapshot(11.1)[(0,0)]
    assert not e.confidence_neighbor_corrected and e.output_cost==71 and e.q_s==.35
    assert c.snapshot(11.2)[(0,0)].output_cost==71
    assert not c.last_snapshot_changed_keys


def test_no_corrected_confidence_propagation_and_order_independent():
    # Two adjacent weak cells must not become sources for one another.
    cells={(x,y) for x in range(-1,3) for y in range(-1,2)}
    results=[]
    for order in [sorted(cells),sorted(cells,reverse=True)]:
        c=SemanticCore()
        for key in order:observe(c,key,[51]*(7 if key in [(0,0),(1,0)] else 20),1.)
        s=c.snapshot(1.);results.append(s)
        for key in [(0,0),(1,0)]:
            assert s[key].effective_q_s==.35 and not s[key].confidence_neighbor_corrected
    assert results[0]==results[1]


def test_disabling_neighbor_correction_keeps_bounded_penalty():
    c=SemanticCore(cost_policy=SemanticCostPolicy(neighbor_confidence_enabled=False))
    observe(c,(0,0),[51]*7,1.);ring(c)
    assert c.snapshot(1.)[(0,0)].output_cost==71


def test_reset_clears_cached_distributions_and_neighbor_state():
    c=SemanticCore();observe(c,(0,0),[51]*7,1.);ring(c);c.snapshot(1.)
    observe(c,(0,0),[31]*20,.5)
    s=c.snapshot(.5)
    assert set(s)=={(0,0)} and s[(0,0)].output_cost==20
    assert not c._neighbor_deadlines and c._semantic_histograms=={(0,0):((31,20),)}


@pytest.mark.parametrize('kwargs',[
    {'confidence_beta':1.},{'confidence_beta':float('nan')},
    {'confidence_beta':-.1},{'confidence_low_threshold':.8},
    {'neighbor_confidence_enabled':1},{'confidence_histogram_tv_max':2.}])
def test_invalid_policy_rejected(kwargs):
    with pytest.raises(ValueError):SemanticCostPolicy(**kwargs)


def test_new_archive_diagnostics_round_trip_and_old_archives_compatible(tmp_path):
    from nomad_semantic_mapping.archive import write_snapshot,load_snapshot
    c=SemanticCore();observe(c,(0,0),[51]*7,1.);ring(c);s=c.snapshot(1.)
    a=c.layers(ObservedGrid(.2,2.,0.,4.),s);a['trajectory']=np.empty((0,8),np.float64)
    m={'schema':'nomad-semantic-mapping-v1','frame_id':'odom','origin_x':-1.,'origin_y':-1.,
       'resolution_m':.2,'version':1,'stamp_s':1.,'cost_model':c.cost_policy.metadata()}
    p=write_snapshot(tmp_path/'new',a,m,final=True);b,_=load_snapshot(p)
    np.testing.assert_array_equal(a['effective_q_s'],b['effective_q_s'])
    assert 'confidence_neighbor_corrected' in (p.parent/'cells.csv').read_text()
    old={k:v for k,v in a.items() if k not in ['effective_q_s','confidence_neighbor_corrected','confidence_neighbor_support_cells']}
    p=write_snapshot(tmp_path/'old',old,m);load_snapshot(p)
    invalid={**a,'effective_q_s':a['effective_q_s'].copy()}
    invalid['effective_q_s'][a['observed'].astype(bool)]=0.
    with pytest.raises(ValueError):write_snapshot(tmp_path/'invalid',invalid,m)


def test_mapper_archives_both_active_and_original_support_histograms():
    c=SemanticCore();observe(c,(0,0),[51]*16,1.);observe(c,(0,0),[51]*7,2.)
    s=c.snapshot(11.1)
    import threading
    n=SimpleNamespace(lock=threading.Lock(),trajectory=[],estimates=s,
                      layers=c.layers(ObservedGrid(.2,2.,0.,4.),s))
    a=SemanticMapper.arrays(n)
    assert a['sparse_retained_class_counts'][0,51]==7
    assert a['sparse_semantic_support_class_counts'][0,51]==23
