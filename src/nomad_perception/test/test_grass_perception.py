from types import SimpleNamespace as NS

import numpy as np

from nomad_perception.grass_perception import GrassEvidence, low_grass
from nomad_perception.geometry import project_scan
from nomad_perception.observed_grid import ObservedGrid


def patch(z=.3, green=True):
    y, x = np.mgrid[-.8:.8:.04, -.8:.8:.04]
    ground = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    y, x = np.mgrid[-.15:.15:.025, -.15:.15:.025]
    plant = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))
    points = np.vstack((ground, plant))
    colours = np.tile([65, 95, 130], (len(points), 1)).astype(np.uint8)
    colours[len(ground):] = [30, 140, 40] if green else [75, 75, 75]
    return points, colours, len(ground)


def test_low_grass_needs_colour_height_and_observed_ground():
    points, colours, n = patch()
    flags = low_grass(points, colours)
    assert not flags[:n].any() and flags[n:].all()
    assert not low_grass(points[n:], colours[n:]).any()  # no visible support
    for z, green in [(1.5, True), (.3, False), (0., True)]:
        p, c, n = patch(z, green)
        assert not low_grass(p, c).any()


def test_steep_ground_does_not_become_passable_from_green_colour():
    points, colours, _ = patch()
    points[:, 2] += points[:, 0]*np.tan(.5)
    assert not low_grass(points, colours).any()


def test_repeated_evidence_expiry_and_solid_veto():
    p = np.array([[1.02, .02, .32], [1.03, .03, .33], [1.04, .04, .34]])
    cache = GrassEvidence()
    assert not cache.update(p, np.ones(3, bool), 1.).any()
    assert not cache.update(p, np.ones(3, bool), 1.).any()  # no repeated-stamp votes
    assert cache.update(p, np.ones(3, bool), 2.).all()
    assert cache.matches(p, 2.1).all()
    assert not cache.matches(p, 1.9).any()  # future evidence forbidden
    assert not cache.matches(p, 5.1).any()
    cache.update(p, np.array([True, False, True]), 2.2)
    assert not cache.matches(p, 2.3).any()  # mixed grass and rock is solid


def test_grass_scan_opens_only_observed_endpoint_and_preserves_other_hits():
    scan = NS(ranges=[1., 2.], angle_min=0., angle_increment=np.pi/2,
              range_min=.1, range_max=10.)
    tf = NS(rotation=NS(x=0., y=0., z=0., w=1.), translation=NS(x=0., y=0., z=.35))
    cache = GrassEvidence()
    p = np.array([[1.001, .001, .35], [1.002, .002, .35], [1.003, .003, .35]])
    for stamp in [1., 2.]:
        cache.update(p, np.ones(3, bool), stamp)
    origin, rays = project_scan(scan, tf, 8., lambda p: cache.matches(p, 2.1))
    assert rays[0][2] is False and rays[1][2] is True
    grid = ObservedGrid(size=10)
    grid.integrate_rays(origin, [(1., 0., True)], 1.)
    for stamp in [2.1, 2.2, 2.3, 2.4]:
        grid.integrate_rays(origin, rays, stamp)
    raw = grid.maps(2.4)[0]
    x, y = grid.cell(1., 0.)
    assert raw[y, x] == 0
    x, y = grid.cell(1.5, 0.)
    assert raw[y, x] == -1  # no artificial ray behind grass
    x, y = grid.cell(0., 2.)
    assert raw[y, x] == 100


def test_same_cell_solid_return_wins_over_passable_grass():
    grid = ObservedGrid(size=10)
    grid.integrate_rays((0., 0.), [(1., 0., False), (1.01, .01, True)], 1.)
    x, y = grid.cell(1., 0.)
    assert grid.maps(1.)[0][y, x] == 100
