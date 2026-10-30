"""Preserved map geometry, measured metrics and isolated experiment artifacts."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nomad_bringup.scenario_lab.core import (
    scenario, rectangles, observe, polygon_clearance, summarize, workspace)
from nomad_bringup.scenario_lab.worlds import build, viewer_config


@pytest.mark.parametrize('number', range(1, 7))
def test_world_preserves_grid_obstacles_and_vehicle_physics(number, tmp_path):
    s = scenario(number)
    reconstructed = np.zeros((s['height'], s['width']), dtype=bool)
    r = s['resolution']
    for x0, y0, x1, y1 in rectangles(s):
        reconstructed[round(y0/r):round(y1/r), round(x0/r):round(x1/r)] = True
    assert np.array_equal(reconstructed, np.array(s['grid']).reshape(reconstructed.shape) >= 80)
    world = build(number, tmp_path)
    tree = ET.parse(world)
    original = ET.parse(workspace()/'src/nomad_gazebo/models/nomad_vehicle/model.sdf')
    actual = tree.find(".//model[@name='nomad_vehicle']")
    assert actual is not None
    assert sum(float(x.text) for x in original.findall('.//inertial/mass')) == pytest.approx(sum(float(x.text) for x in actual.findall('.//inertial/mass')))
    assert actual.find("joint[@name='rear_left_steering_joint']") is not None
    assert actual.find("joint[@name='front_left_steering_joint']") is None
    assert len(actual.findall('joint')) == len(original.findall('.//joint'))
    assert not actual.findall('.//sensor')
    assert len(tree.findall(".//model[@name='scenario_ground']")) == 1
    viewer_config(world, tmp_path/'viewer.config')
    assert 'MinimalScene' in (tmp_path/'viewer.config').read_text()
    if number == 3:
        lines=(tmp_path/'terrain.obj').read_text().splitlines()
        assert sum(x.startswith('vn ') for x in lines)==sum(x.startswith('f ') for x in lines)
        assert all('//' in x for x in lines if x.startswith('f '))


def test_gt_observation_does_not_see_through_wall():
    s = dict(width=8, height=5, resolution=1., grid=[0]*40)
    for y in range(5):s['grid'][y*8+3] = 100
    known = [-1]*40
    observe(s, known, (1.5, 2.5, 0.), radius=10)
    assert known[2*8+3] == 100
    assert known[2*8+4] == -1
    assert known[2*8+1] == 0


def test_clearance_uses_rotated_vehicle_envelope():
    assert polygon_clearance((0, 0, 0), [(2, -2, 3, 2)]) == pytest.approx(1.48)
    assert polygon_clearance((0, 0, math.pi/2), [(2, -2, 3, 2)]) == pytest.approx(1.58)
    assert polygon_clearance((2.1, 0, 0), [(2, -2, 3, 2)]) == 0
    assert polygon_clearance((0, 0, 0), []) is None


def test_stop_outcome_is_not_completion_and_missing_metrics_are_not_zero():
    rows = [dict(time_s=i, clearance_m=v) for i, v in enumerate([.1, .2, .3, .4])]
    d = summarize(rows, 'safe_stop', scenario(5), 2, 1, {'gpp': 3}, 'now')
    assert not d['completed'] and d['expected_result_met']
    assert d['clearance_p05_m'] == pytest.approx(.115)
    d = summarize([], 'startup_error', scenario(1), 0, 0, {}, 'now')
    assert not d['expected_result_met']
    assert d['clearance_p05_m'] is None and d['elapsed_sim_s'] is None


@pytest.mark.parametrize('angle', [0., 5., 10., 20., 30.])
def test_selected_slope_matches_generated_mesh_and_blocking(angle, tmp_path):
    import json
    from nomad_bringup.scenario_lab.core import observed_planning_layers
    build(3, tmp_path, slope_degrees=angle)
    s=json.loads((tmp_path/'scenario.json').read_text())
    vertices=[];inclinations=[]
    for line in (tmp_path/'terrain.obj').read_text().splitlines():
        if line.startswith('v '):vertices.append(np.array(list(map(float,line.split()[1:]))))
        if line.startswith('f '):
            a,b,c=[vertices[int(v.split('/')[0])-1] for v in line.split()[1:]]
            normal=np.cross(b-a,c-a)
            inclinations.append(math.degrees(math.atan2(np.linalg.norm(normal[:2]),abs(normal[2]))))
    assert max(inclinations)==pytest.approx(angle, abs=1e-7)
    assert np.array(s['slope_deg'])==pytest.approx(np.max(np.array(inclinations).reshape(-1,2),axis=1))
    # Terrain blocking is a planning constraint, never a new vertical wall.
    assert rectangles(s)==rectangles(scenario(3))
    costs=np.array(s['planning_grid']);slopes=np.array(s['slope_deg'])
    if angle >= 10:assert np.all(costs[slopes>=10-1e-9]>=80)
    if angle == 0:assert np.all(costs[np.array(s['grid'])<80]==0)
    known=[-1]*len(costs);known[0]=s['grid'][0]
    grid,sc,tc=observed_planning_layers(s,known)
    assert grid[1]==-1 and math.isnan(sc[1]) and math.isnan(tc[1])
    assert sc[0]==s['slope_cost'][0]


def test_adjustable_slope_reaches_existing_gpp_cost_equation():
    import yaml
    sys.path.insert(0,str(workspace()/'src/nomad_path_planning'))
    from nomad_path_planning.gpp.multicost import compose
    from nomad_bringup.scenario_lab.core import trial_parameters, observed_planning_layers, PACKAGE
    base=yaml.safe_load((PACKAGE/'config.yaml').read_text())
    flat=scenario(3,0.);hill=scenario(3,8.)
    parameters=trial_parameters(hill,base)['dstar_lite_gpp']['ros__parameters']
    assert parameters['gpp.cost_mode']=='multi' and parameters['gpp.slope_weight']==5.
    def costs(s):
        data,sc,tc=observed_planning_layers(s,s['grid'])
        grid=dict(width=s['width'],height=s['height'],resolution=s['resolution'],data=data)
        return np.array(compose(grid,sc,tc,slope_weight=parameters['gpp.slope_weight'],
                               obstacle_weight=0.,terrain_weight=0.))
    low,high=costs(flat),costs(hill)
    valid=np.isfinite(high)
    assert np.all(high[valid]>=low[valid])
    assert high[valid].max()==pytest.approx(1.+5.*(.8**2))
    # Existing archived/CLI specifications keep their legacy input mode.
    assert 'terrain' not in scenario(3)
    assert trial_parameters(scenario(3),yaml.safe_load((PACKAGE/'config.yaml').read_text()))['dstar_lite_gpp']['ros__parameters']['gpp.cost_mode']=='legacy'


@pytest.mark.parametrize('number,angle', [(1,5.),(3,-1.),(3,31.),(3,float('nan'))])
def test_invalid_slope_is_rejected(number,angle):
    with pytest.raises(ValueError):scenario(number,angle)


def test_slope_overrides_preserve_remote_shared_rear_steering_geometry():
    from nomad_bringup.scenario_lab.core import resolved_config, trial_parameters
    original=resolved_config()
    modified=trial_parameters(scenario(3,8.),resolved_config())
    for node in original:
        for key in ('drive_layout','steering_axle','wheelbase','reference_offset','track','wheel_radius','joint_limit'):
            assert modified[node]['ros__parameters'][key]==original[node]['ros__parameters'][key]
    assert modified['dstar_lite_gpp']['ros__parameters']['gpp.cost_mode']=='multi'
    assert modified['/**']['ros__parameters']['steering_axle']=='rear'
    assert modified['/**']['ros__parameters']['reference_offset']<0
