import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from nomad_perception.ground_perception import GroundEvidence
from nomad_perception.geometry import project_scan
from nomad_perception.observed_grid import ObservedGrid
from nomad_perception.geometric_terrain import surface_costs


def plane(slope=.1, height=0.):
    y, x = np.mgrid[-1:1:.025, .4:4.5:.025]
    return np.column_stack((x.ravel(), y.ravel(), (slope*x+height).ravel()))


def evidence(points):
    g = GroundEvidence()
    g.update(points, np.eye(3), np.zeros(3), 10.)
    return g


def test_connected_gentle_slope_matches_surface_not_objects_above_it():
    g = evidence(plane())
    assert len(g.cells) > 100
    assert g.matches(np.array([[3.1, .1, .31]]), 10.1)[0]
    assert not g.matches(np.array([[3.1, .1, .50]]), 10.1)[0]
    _, costs, _ = surface_costs(plane())
    assert np.any((costs > 0) & (costs < 80))  # ground remains graded, not cost zero


@pytest.mark.parametrize('stamp', [9.99, 10.31])
def test_future_or_stale_depth_cannot_clear_lidar(stamp):
    assert not evidence(plane()).matches(np.array([[3.1, .1, .31]]), stamp)[0]


def test_disconnected_flat_top_and_nearby_raised_platform_are_not_ground():
    p = plane(0.)
    p = p[(p[:, 0] < 1.8) | (p[:, 0] > 2.4)]
    p[p[:, 0] > 2.4, 2] = .3
    g = evidence(p)
    assert g.matches(np.array([[1.1, .1, 0.]]), 10.1)[0]
    assert not g.matches(np.array([[3.1, .1, .3]]), 10.1)[0]
    assert not evidence(plane(0., .2)).cells  # an elevated top cannot seed ground


def test_step_and_missing_depth_break_connectivity():
    p = plane(0.)
    p[p[:, 0] > 2, 2] = .25
    g = evidence(p)
    assert not g.matches(np.array([[3.1, .1, .25]]), 10.1)[0]
    p = plane(0.)
    p = p[(p[:, 0] < 1.8) | (p[:, 0] > 2.4)]
    assert not evidence(p).matches(np.array([[3.1, .1, 0.]]), 10.1)[0]


def test_steep_and_rough_surfaces_remain_obstacles():
    assert not evidence(plane(math.tan(.4))).cells
    p = plane(0.)
    p[:, 2] += .09*np.sin(100*p[:, 0])*np.cos(100*p[:, 1])
    assert not evidence(p).cells


def test_solid_points_veto_ground_fit_and_new_invalid_frame_revokes_evidence():
    p = plane(0.)
    y, z = np.mgrid[-.3:.3:.02, 0:.8:.02]
    wall = np.column_stack((np.full(y.size, 2.1), y.ravel(), z.ravel()))
    g = evidence(np.vstack((p, wall)))
    assert not g.matches(np.array([[2.1, .1, .02]]), 10.1)[0]
    g.update(np.empty((0, 3)), np.eye(3), np.zeros(3), 10.2)
    assert not g.matches(np.array([[1.1, .1, 0.]]), 10.3)[0]


def test_matching_return_clears_accumulated_hit_without_extending_the_ray():
    g = evidence(plane())
    scan = NS(ranges=[3.1, 1.], angle_min=0., angle_increment=math.pi/2,
              range_min=.1, range_max=8.)
    tf = NS(rotation=NS(x=0., y=0., z=0., w=1.), translation=NS(x=0., y=0., z=.31))
    origin, rays = project_scan(scan, tf, 8., lambda points: g.matches(points, 10.1))
    assert not rays[0][2] and rays[1][2]
    grid = ObservedGrid(size=10)
    grid.integrate_rays(origin, [(3.1, 0., True)], 9.)
    for stamp in [10.1, 10.2, 10.3, 10.4]:
        grid.integrate_rays(origin, rays, stamp)
    raw = grid.maps(10.4)[0]
    x, y = grid.cell(3.1, 0.)
    assert raw[y, x] == 0
    x, y = grid.cell(3.5, 0.)
    assert raw[y, x] == -1
    x, y = grid.cell(0., 1.)
    assert raw[y, x] == 100


def test_translation_and_yaw_do_not_change_ground_decision():
    angle = .7
    r = np.array([[math.cos(angle), -math.sin(angle), 0.],
                  [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])
    t = np.array([-28., -17., .2])
    g = GroundEvidence()
    assert g.update(plane() @ r.T+t, r, t, 10.) > 0
    assert g.matches(np.array([[3.1, .1, .31]]) @ r.T+t, 10.1)[0]


def test_tilted_vehicle_can_seed_observed_ground_in_a_depression():
    # Vehicle points down into a smooth depression. Extrapolating the nearly
    # level ground ahead back to the base incorrectly gives a 16 cm mismatch.
    pitch = .22
    r = np.array([[math.cos(pitch), 0., math.sin(pitch)],
                  [0., 1., 0.], [-math.sin(pitch), 0., math.cos(pitch)]])
    g = GroundEvidence()
    assert g.update(plane(.03, -.16), r, np.zeros(3), 10.) > 0
    assert g.matches(np.array([[3.1, .1, -.16+.03*3.1]]), 10.1)[0]


def test_near_low_band_does_not_seed_a_separate_higher_top():
    p = plane(0.)
    # A visible narrow gap separates a nearby flat top from the low floor.
    p = p[(p[:, 1] < -.2) | (p[:, 1] > .2)]
    p[p[:, 1] > .2, 2] = .12
    g = evidence(p)
    assert g.matches(np.array([[1.1, -.5, 0.]]), 10.1)[0]
    assert not g.matches(np.array([[1.1, .5, .12]]), 10.1)[0]


def test_old_ground_hit_clears_even_when_new_ray_stops_short():
    grid = ObservedGrid(size=10)
    grid.integrate_rays((0., 0.), [(3.1, .1, True, .31)], 9.)
    grid.integrate_rays((0., 0.), [(2.1, .1, False, .21)], 9.5)
    x, y = grid.cell(3.1, .1)
    assert grid.maps(10.)[0][y, x] == 100
    assert grid.reclassify_ground(evidence(plane()).matches, 10.) == 1
    assert grid.maps(10.)[0][y, x] == 0
    x, y = grid.cell(3.5, .1)
    assert grid.maps(10.)[0][y, x] == -1
    grid.integrate_rays((0., 0.), [(3.1, .1, True, .6)], 10.1)
    assert grid.reclassify_ground(evidence(plane()).matches, 10.2) == 0


@pytest.mark.parametrize('extra', [(3.11, .1, True, .6), (3.11, .1, True)])
def test_solid_or_unknown_height_in_same_cell_vetoes_clearing(extra):
    grid = ObservedGrid(size=10)
    grid.integrate_rays((0., 0.), [(3.1, .1, True, .31), extra], 9.)
    assert grid.reclassify_ground(evidence(plane()).matches, 10.) == 0


def test_ground_history_survives_padding_and_rejects_older_depth():
    grid = ObservedGrid(size=10)
    grid.integrate_rays((0., 0.), [(3.1, .1, True, .31)], 10.1)
    grid.ensure_bounds(-12., -12.)
    g = evidence(plane())
    assert grid.reclassify_ground(g.matches, 10.) == 0
    assert grid.reclassify_ground(g.matches, 10.5) == 0  # stale depth
    assert grid.reclassify_ground(g.matches, 10.2) == 1
    x, y = grid.cell(3.1, .1)
    assert grid.maps(10.2)[0][y, x] == 0


def test_dense_history_keeps_height_evidence_but_out_of_view_stays_blocked():
    grid = ObservedGrid(size=10)
    rays = [(3.1+i*1e-5, .1, True, .31) for i in range(129)]
    rays.append((3.1, 2.1, True, .31))
    grid.integrate_rays((0., 0.), rays, 9.)
    assert grid.reclassify_ground(evidence(plane()).matches, 10.) == 1
    x, y = grid.cell(3.1, 2.1)
    assert grid.maps(10.)[0][y, x] == 100


def test_dense_history_cannot_average_away_a_solid_return():
    grid = ObservedGrid(size=10)
    rays = [(3.1+i*1e-6, .1, True, .31) for i in range(2000)]
    rays.append((3.1, .1, True, .6))
    grid.integrate_rays((0., 0.), rays, 9.)
    x, y = grid.cell(3.1, .1)
    assert len(grid.hit_points[y, x]) <= 16
    assert grid.reclassify_ground(evidence(plane()).matches, 10.) == 0


def test_scan_projection_preserves_world_height_for_history():
    scan = NS(ranges=[3.1], angle_min=0., angle_increment=.1,
              range_min=.1, range_max=8.)
    tf = NS(rotation=NS(x=0., y=0., z=0., w=1.),
            translation=NS(x=0., y=0., z=.31))
    _, rays = project_scan(scan, tf, 8., include_height=True)
    assert rays == [(3.1, 0., True, .31)]


def test_slope_disabled_accepts_connected_steep_ground_but_not_solid_above():
    angle = .6
    r = np.array([[math.cos(angle), 0., -math.sin(angle)],
                  [0., 1., 0.], [math.sin(angle), 0., math.cos(angle)]])
    g = GroundEvidence(slope_enabled=False)
    assert g.update(plane(math.tan(angle)), r, np.zeros(3), 10.) > 0
    z = 3.1*math.tan(angle)
    assert g.matches(np.array([[3.1, .1, z]]), 10.1)[0]
    assert not g.matches(np.array([[3.1, .1, z+.2]]), 10.1)[0]


def test_ground_memory_survives_camera_edge_but_expires_and_keeps_height():
    g = GroundEvidence(memory_seconds=2.)
    p = plane(0.)
    g.update(p, np.eye(3), np.zeros(3), 10.)
    # New frame sees only nearby ground; previous far ground leaves the image.
    g.update(p[p[:, 0] < 2.], np.eye(3), np.zeros(3), 11.)
    assert g.matches(np.array([[3.1, .1, 0.]]), 11.1)[0]
    assert not g.matches(np.array([[3.1, .1, .2]]), 11.1)[0]
    assert not g.matches(np.array([[3.1, .1, 0.]]), 11.4)[0]  # camera stalled
    g.update(p[p[:, 0] < 2.], np.eye(3), np.zeros(3), 12.1)
    assert not g.matches(np.array([[3.1, .1, 0.]]), 12.2)[0]


def test_previous_ground_reattaches_visible_patch_without_inventing_ground():
    g = GroundEvidence(memory_seconds=2.)
    p = plane(0.)
    g.update(p, np.eye(3), np.zeros(3), 10.)
    far = p[p[:, 0] > 2.5]
    assert g.update(far, np.eye(3), np.zeros(3), 11.) > 0
    assert g.matches(np.array([[3.1, .1, 0.]]), 11.1)[0]
    new = GroundEvidence(memory_seconds=2.)
    assert new.update(far, np.eye(3), np.zeros(3), 11.) == 0
    # A new elevated surface must revoke the old ground in this same cell.
    far[:, 2] = .2
    g.update(far, np.eye(3), np.zeros(3), 11.2)
    assert not np.any(g.matches(np.array([[3.1, .1, 0.], [3.1, .1, .2]]), 11.3))


def test_ground_memory_reset_and_empty_frame_revoke_old_support():
    g = GroundEvidence(memory_seconds=10.)
    p = plane(0.)
    g.update(p, np.eye(3), np.zeros(3), 10.)
    g.update(p[p[:, 0] > 2.5], np.eye(3), np.zeros(3), 5.)
    assert not g.history
    g.update(p, np.eye(3), np.zeros(3), 6.)
    g.update([], np.eye(3), np.zeros(3), 6.1)
    assert not g.history


def test_remembered_ground_clears_old_lidar_but_preserves_rock():
    g = GroundEvidence(memory_seconds=2.)
    p = plane(0.)
    g.update(p, np.eye(3), np.zeros(3), 10.)
    grid = ObservedGrid(size=10)
    grid.integrate_rays((0., 0.), [(3.1, .1, True, 0.), (3.1, .5, True, .3)], 10.5)
    g.update(p[p[:, 0] < 2.], np.eye(3), np.zeros(3), 11.)
    assert grid.reclassify_ground(g.matches, 11.) == 1
    raw = grid.maps(11.)[0]
    x, y = grid.cell(3.1, .1)
    assert raw[y, x] == 0
    x, y = grid.cell(3.1, .5)
    assert raw[y, x] == 100
