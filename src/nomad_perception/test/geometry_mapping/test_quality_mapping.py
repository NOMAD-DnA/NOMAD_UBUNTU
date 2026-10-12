"""Geometry quality, failure recovery and hazard preservation without ROS."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest

from nomad_perception.traversability_core import CoreConfig, TraversabilityCore
from nomad_perception.quality_mapping_core import (
    QualityMappingCore, PlaneQualityConfig, spread_quality, bin_representatives, contains_target)


def make_core(**kwargs):
    return QualityMappingCore(CoreConfig(geometry_only=True, radius_m=.3), PlaneQualityConfig(**kwargs))


def plane(degrees=4., center=(.1, .1), offset=1., extent=.25):
    axis=np.linspace(-extent, extent, 21)
    xx,yy=np.meshgrid(axis+center[0],axis+center[1])
    zz=offset+np.tan(np.deg2rad(degrees))*(xx-center[0])
    return np.column_stack((xx.ravel(),yy.ravel(),zz.ravel())).astype(np.float32)


def weak_plane(offset=1., degrees=4.):
    x=np.array([.09,.1,.11]);y=np.array([.0999,.1001,.0999])
    z=offset+np.tan(np.deg2rad(degrees))*(x-.1)+np.tan(np.deg2rad(85.))*(y-.1)
    return np.column_stack((x,y,z)).astype(np.float32)


def ingest(core, xyz, stamp):
    core.ingest(xyz,np.full(len(xyz),31,np.uint32),stamp)
    return core.snapshot(stamp)


@pytest.mark.parametrize('degrees',[0.,12.,25.,35.])
def test_distributed_raw_planes_are_accepted_and_real_slopes_stay_blocked(degrees):
    core=make_core();state=ingest(core,plane(degrees),1.)
    target=state[(0,0)]
    assert target.slope_valid
    assert target.slope_deg==pytest.approx(degrees,abs=1e-5)
    assert core.quality_records[(0,0)]['source']=='raw'
    assert (target.final_cost==100)==(degrees>=20.)
    assert (0,0) in core.last_snapshot_new_fit_keys


def test_three_point_steep_artifact_is_not_accepted_as_ground():
    core=make_core();target=ingest(core,weak_plane(),1.)[(0,0)]
    assert not target.slope_valid and target.slope_deg is None
    assert target.final_cost==100
    assert core.quality_records[(0,0)]['source']=='uncertain'
    assert core.quality_records[(0,0)]['raw_slope_deg']>80
    assert (0,0) not in core.last_snapshot_new_fit_keys
    samples=core.diagnostic_arrays()
    assert samples['diagnostic_raw_xyz'].shape==(3,3)
    np.testing.assert_array_equal(samples['diagnostic_raw_capture_stamps_s'],[1,1,1])


def test_many_collinear_points_cannot_pass_by_point_count():
    xyz=np.repeat(weak_plane(),100,axis=0)
    core=make_core();target=ingest(core,xyz,1.)[(0,0)]
    assert not target.slope_valid
    assert core.quality_records[(0,0)]['raw_minor_std_m']<.001


def test_height_refit_recovers_expired_support_without_inflating_raw_confidence():
    core=make_core();ingest(core,plane(4.),1.)
    state=ingest(core,weak_plane(),20.)
    target=state[(0,0)]
    assert target.slope_valid and target.slope_deg==pytest.approx(4.,abs=.5)
    record=core.quality_records[(0,0)]
    assert record['source']=='height_refit' and record['radius_m']==.3
    assert record['raw_q_g']<1e-4 and target.q_g>record['raw_q_g']
    assert target.plane_support>=6 and target.final_cost<100
    assert (0,0) in core.last_snapshot_new_fit_keys
    # Demonstrate the actual failure in the old numerical core on the same input.
    old=TraversabilityCore(core.config);ingest(old,plane(4.),1.)
    old_target=ingest(old,weak_plane(),20.)[(0,0)]
    assert old_target.slope_deg>80 and old_target.final_cost==100


def test_trusted_previous_plane_is_retained_when_fallback_has_too_few_cells():
    core=make_core();first=ingest(core,plane(12.,extent=.08),1.)[(0,0)]
    target=ingest(core,weak_plane(degrees=12.),20.)[(0,0)]
    assert target.slope_deg==first.slope_deg and target.q_g==first.q_g
    assert target.slope_reused and core.quality_records[(0,0)]['source']=='retained'
    assert (0,0) not in core.last_snapshot_new_fit_keys


def test_conflicting_new_height_invalidates_previous_passable_plane():
    core=make_core();ingest(core,plane(12.,extent=.08),1.)
    target=ingest(core,weak_plane(offset=1.2,degrees=12.),20.)[(0,0)]
    assert not target.slope_valid and target.final_cost==100
    assert core.quality_records[(0,0)]['source']=='uncertain'
    target=ingest(core,plane(12.,extent=.08,offset=1.2),20.1)[(0,0)]
    assert target.slope_valid and target.slope_deg==pytest.approx(12.,abs=.01)


def heights(degrees=4.):
    return {(x,y):(1.+np.tan(np.deg2rad(degrees))*x*.2,1.)
            for x in (-1,0,1) for y in (-1,0,1)}


def test_refit_requires_target_observation_and_surrounding_support():
    core=make_core();data=heights();del data[(0,0)]
    assert core.refit_heights((0,0),data)[0] is None
    one_side={(x,y):(1.,1.) for x in (0,1,2) for y in (0,1,2)}
    assert core.refit_heights((0,0),one_side)[0] is None
    assert not contains_target([(0,0),(1,0),(2,0)],(.5,0))


def test_step_and_nonplanar_surface_are_not_smoothed_open():
    core=make_core();data=heights(0.)
    data={k:(z+.3 if k[0]>=0 else z,t) for k,(z,t) in data.items()}
    fitted,*_,reason=core.refit_heights((0,0),data)
    assert fitted is None and reason=='height_discontinuity'
    data=heights(0.);data[(0,0)]=(1.06,1.)
    assert core.refit_heights((0,0),data)[0] is None


def test_valid_height_incline_can_still_be_blocked():
    core=make_core();fitted,*_=core.refit_heights((0,0),heights(25.))
    assert fitted is not None and fitted.slope_deg==pytest.approx(25.)


def test_refit_expands_only_when_small_neighborhood_is_insufficient():
    core=make_core();data={(0,0):(1.,1.),(-3,0):(1.,1.),(3,0):(1.,1.),
                           (0,-3):(1.,1.),(0,3):(1.,1.),(1,1):(1.,1.)}
    # Disconnected points do not fill holes in observed ground.
    assert core.refit_heights((0,0),data)[0] is None
    for x,y in [(-2,0),(-1,0),(1,0),(2,0),(0,-2),(0,-1),(0,1),(0,2)]:
        data[(x,y)]=(1.,1.)
    fitted,xyz,stamps,radius,reason=core.refit_heights((0,0),data)
    assert fitted is not None and radius<=.6 and reason=='accepted'


def test_reset_clears_quality_cache_and_diagnostic_points():
    core=make_core();ingest(core,weak_plane(),1.);core.reset()
    assert not core.quality_records and not core.rejected_support and not core._dependents


def test_diagnostic_storage_is_bounded_and_serializable():
    core=make_core(diagnostic_max_cases=2,diagnostic_max_points=2)
    for key in [(0,0),(1,0),(2,0)]:
        core._save_rejected(key,weak_plane(),np.ones(3),None)
    arrays=core.diagnostic_arrays()
    assert len(arrays['diagnostic_raw_cells'])==2
    assert arrays['diagnostic_raw_xyz'].shape==(4,3)
    assert arrays['diagnostic_raw_offsets'].tolist()==[0,2,4]
    json.dumps(core.quality_metadata())


@pytest.mark.parametrize('kwargs',[{'min_raw_points':0},{'min_xy_bins':2},
    {'min_minor_std_m':np.nan},{'min_xy_variance_ratio':2},
    {'refit_radii_m':[.6,.3]},{'refit_radii_m':[]},{'max_refit_rms_m':0}])
def test_invalid_quality_settings_fail_at_startup(kwargs):
    with pytest.raises(ValueError):PlaneQualityConfig(**kwargs)
