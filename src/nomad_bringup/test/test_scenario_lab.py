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
