import numpy as np

from nomad_perception.ground_perception import GroundEvidence


def cloud():
    y, x = np.mgrid[-1:1:.025, .4:4.5:.025]
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))


def classify(points):
    g = GroundEvidence(slope_enabled=False, cell_local=True)
    g.update(points, np.eye(3), np.zeros(3), 10.)
    return g, *g.object_costs(points)


def test_curved_hill_has_no_shape_hazard():
    p = cloud()
    p[:, 2] = .25*(1-np.cos((p[:, 0]-.4)*1.5))
    g, centres, costs = classify(p)
    assert len(g.cells) > 100
    assert np.count_nonzero(costs == 0) > 100
    assert not np.any(costs == 100)
    point = np.array([[2.1, .1, .25*(1-np.cos((2.1-.4)*1.5))]])
    assert g.matches(point, 10.1)[0]


def test_rock_top_is_obstacle_and_adjacent_ground_is_free():
    p = cloud()
    rock = (p[:, 0] > 2.) & (p[:, 0] < 2.4) & (abs(p[:, 1]) < .2)
    p[rock, 2] = .3
    g, centres, costs = classify(p)
    near_rock = (centres[:, 0] > 2.) & (centres[:, 0] < 2.4) & (abs(centres[:, 1]) < .2)
    assert np.all(costs[near_rock] == 100)
    assert not g.matches(np.array([[2.1, .1, .3]]), 10.1)[0]
    nearby = (centres[:, 0] > 2.) & (centres[:, 0] < 2.4) & (centres[:, 1] > .2) & (centres[:, 1] < .4)
    assert np.all(costs[nearby] == 0)


def test_trunk_remains_obstacle_and_missing_reference_stays_unknown():
    p = cloud()
    y, z = np.mgrid[-.1:.1:.025, 0:1:.025]
    trunk = np.column_stack((np.full(y.size, 2.1), y.ravel(), z.ravel()))
    _, centres, costs = classify(np.vstack((p, trunk)))
    at_trunk = (abs(centres[:, 0]-2.1) < .01) & (abs(centres[:, 1]) < .2)
    assert np.all(costs[at_trunk] == 100)
    g = GroundEvidence(slope_enabled=False, cell_local=True)
    assert np.all(g.object_costs(p)[1] == -1)
    assert not g.matches(np.array([[2.1, .1, 0.]]), 10.)[0]
