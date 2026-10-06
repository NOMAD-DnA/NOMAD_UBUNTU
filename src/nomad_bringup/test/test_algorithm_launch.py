"""Verify CLI precedence and forest forwarding without starting processes."""
from pathlib import Path
import importlib.util
import pytest
from launch import LaunchContext
from nomad_bringup import launching

ROOT=Path(__file__).resolve().parents[1]


def context(gpp='',lpp=''):
    ctx=LaunchContext()
    ctx.launch_configurations.update({
        'topics_file':str(ROOT/'config/topics.yaml'),
        'modules_file':str(ROOT/'config/modules.yaml'),
        'vehicle_file':str(ROOT/'config/vehicle.yaml'),
        'params_file':'/tmp/custom-planning.yaml','rviz':'false',
        'cmd_vel_topic':'','gpp':gpp,'lpp':lpp,
        'headless':'true','vegetation':'false','world_file':'/tmp/selected.sdf'})
    return ctx


@pytest.mark.parametrize('gpp,lpp',[
    ('hybrid_astar','rpp'),('state_lattice','rpp'),('dstar_lite','rollout'),
    ('astar','rollout'),('weighted_astar','rollout'),('field_dstar','rpp'),('','')])
def test_selection_overrides_yaml_on_planner_nodes_only(monkeypatch,tmp_path,gpp,lpp):
    monkeypatch.setattr(launching,'open',lambda *a: (tmp_path/'lock').open('a'),raising=False)
    monkeypatch.setattr(launching.fcntl,'flock',lambda *a:None)
    monkeypatch.setattr(launching,'Node',lambda **kw:kw)
    monkeypatch.setattr(launching,'_locks',[])
    try:
        nodes=launching.start(context(gpp,lpp),'planning')
        for node in nodes:
            params=node['parameters']
            selected=node['executable'] in ('dstar_lite_gpp','ackermann_lpp')
            if selected and gpp:
                assert params[-1]==dict(gpp_algorithm=gpp,lpp_algorithm=lpp)
                assert params[-2]=='/tmp/custom-planning.yaml'
            else:
                assert params[-1]=='/tmp/custom-planning.yaml'
    finally:
        for lock in launching._locks: lock.close()


def forest_module():
    spec=importlib.util.spec_from_file_location('test_forest_launch',ROOT/'launch/forest.launch.py')
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def test_forest_preserves_map_and_forwards_algorithms():
    gazebo,autonomy=forest_module().start(context('state_lattice','rpp'))
    assert dict(gazebo.launch_arguments)['world_file']=='/tmp/selected.sdf'
    args=dict(autonomy.launch_arguments)
    assert args['gpp']=='state_lattice' and args['lpp']=='rpp'


@pytest.mark.parametrize('gpp,lpp',[('typo','rpp'),('astar','typo')])
def test_invalid_names_rejected_before_process_creation(gpp,lpp):
    with pytest.raises(ValueError,match='Unknown'):
        forest_module().start(context(gpp,lpp))
    with pytest.raises(ValueError,match='Unknown'):
        launching.start(context(gpp,lpp),'planning')
