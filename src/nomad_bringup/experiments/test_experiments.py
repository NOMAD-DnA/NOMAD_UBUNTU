import math
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from common import contact_classification, make_world, metric_values, save, read
from report import wilson


def test_normal_wheel_contact_is_not_a_crash():
    assert contact_classification('nomad_vehicle::front_left_wheel::wheel','terrain::link::collision')=='wheel_ground'
    assert contact_classification('terrain::link::collision','nomad_vehicle::rear_left_wheel::wheel')=='wheel_ground'
    assert contact_classification('nomad_vehicle::base_link::body','terrain::link::collision')=='forbidden'
    assert contact_classification('nomad_vehicle::front_left_wheel::wheel','tree_1::link::collision')=='forbidden'
    assert contact_classification('','')=='unresolved'


def test_instrumentation_does_not_change_physical_model(tmp_path):
    source=tmp_path/'source.sdf';target=tmp_path/'instrumented.sdf'
    source.write_text('<sdf version="1.9"><world name="test"><model name="nomad_vehicle"><pose>1 2 3 0 0 0</pose><link name="base_link"><inertial><mass>20</mass></inertial><collision name="body"><geometry><box><size>1 2 3</size></box></geometry></collision></link></model></world></sdf>')
    original=source.read_bytes();topics=make_world(source,target)
    assert source.read_bytes()==original
    root=ET.parse(target).getroot()
    assert root.findtext('.//mass')=='20'
    assert root.findtext('.//pose')=='1 2 3 0 0 0'
    assert root.findtext('.//collision/geometry/box/size')=='1 2 3'
    assert topics==['/nomad/experiment/contacts/base_link']
    assert root.find('.//sensor/contact/collision').text=='body'
    assert root.findtext('.//sensor/contact/topic')==topics[0]


def test_missing_reference_or_telemetry_never_becomes_successful_score():
    assert metric_values(dict(valid=False))['safe_success'] is None
    success=dict(valid=True,result='success',distance_xy_m=20,duration_sim_s=40)
    assert metric_values(success)['spl'] is None
    reference=dict(validated=True,shortest_distance_m=10,minimum_time_s=10)
    values=metric_values(success,reference)
    assert values['spl']==.5 and values['sct']==.25
    assert metric_values(dict(success,result='collision'),reference)['spl']==0
    assert metric_values(success,dict(reference,validated=False))['sct'] is None


def test_uncertainty_does_not_claim_perfect_certainty_with_three_trials():
    lo,hi=wilson(3,3)
    assert 0<lo<.5 and .999<hi<=1
    assert wilson(0,0)==(None,None)


def test_atomic_metadata_roundtrip(tmp_path):
    target=tmp_path/'nested/state.json'
    save(target,dict(state='running'));save(target,dict(state='complete'))
    assert read(target)==dict(state='complete')
    assert not list(target.parent.glob('.writing-*'))


def test_optional_timing_preserves_return_and_exception(tmp_path, monkeypatch):
    import sys
    from types import SimpleNamespace
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'nomad_path_planning'))
    from nomad_path_planning import experiment_timing as timing
    node=SimpleNamespace(get_name=lambda:'test',get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=123)))
    monkeypatch.setenv('NOMAD_EXPERIMENT_TIMING_DIR',str(tmp_path))
    with pytest.raises(ValueError):
        with timing.measured(node,'failed_call'):
            raise ValueError('expected')
    with timing.measured(node,'successful_call'):
        value=42
    assert value==42
    import csv
    with next(tmp_path.glob('*.csv')).open() as stream: rows=list(csv.DictReader(stream))
    assert rows[0]['exception']=='ValueError' and rows[1]['exception']==''
    assert all(int(row['elapsed_wall_ns'])>=0 and row['sim_ns']=='123' for row in rows)
    timing._stream.close();timing._writer=None;timing._stream=None


def test_map4_route_two_ordered_laps():
    from route_mission import loop_waypoints
    from common import WORKSPACE
    goals, count = loop_waypoints(WORKSPACE/'src/nomad_gazebo/maps/04_vio_flat_loop/route.csv')
    assert len(goals) == 2 * count
    assert goals[:count] == goals[count:]
    assert goals[0][1] < -3.  # Forward from the south-facing spawn, not across the loop.
    assert abs(goals[count-1][0] + 28.) < 1e-8
    assert abs(goals[count-1][1]) < 1e-8
    assert all(2. < math.dist(a,b) < 6.5 for a,b in zip([[-28.,0.]]+goals,goals))


def test_every_map_contact_instrumentation_preserves_vehicle(tmp_path):
    from common import WORKSPACE
    from batch_missions import ALL_MAPS
    for mid in ALL_MAPS:
        source=WORKSPACE/'src/nomad_gazebo/maps'/mid/'worlds/forest.sdf'
        original=ET.parse(source).getroot().find('world')
        vehicle=original.find("model[@name='nomad_vehicle']")
        include=next((i for i in original.findall('include') if i.findtext('name')=='nomad_vehicle'),None)
        if vehicle is None:
            vehicle=ET.parse((source.parent/include.findtext('uri')).resolve()).getroot().find('model')
        target=tmp_path/(mid+'.sdf')
        topics=make_world(source,target)
        actual=ET.parse(target).getroot().find("world/model[@name='nomad_vehicle']")
        assert len(topics)==5
        assert [ET.tostring(x) for x in actual.findall('.//inertial')]==[ET.tostring(x) for x in vehicle.findall('.//inertial')]
        assert [ET.tostring(x) for x in actual.findall('link/collision')]==[ET.tostring(x) for x in vehicle.findall('link/collision')]
        assert actual.findtext('pose')==(include.findtext('pose') if include is not None else vehicle.findtext('pose'))


def test_incline_surface_contacts_only_allow_wheels():
    for ground in ('test_flat_ground','ramp_05_deg','ramp_30_deg'):
        assert contact_classification('nomad_vehicle::front_left_wheel::collision',ground+'::link::collision')=='wheel_ground'
        assert contact_classification('nomad_vehicle::base_link::collision',ground+'::link::collision')=='forbidden'
