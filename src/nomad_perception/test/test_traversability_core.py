"""Algorithm and retention contract tests; no ROS/Gazebo runtime required."""

from dataclasses import replace
import math

import numpy as np
import pytest

from nomad_perception.traversability_core import (
    CLASS_COSTS, EXCLUDED_CLASS_IDS, GROUND_CLASS_IDS, CoreConfig,
    TraversabilityCore, fit_plane, fuse_costs, geometry_cost, _fit_tls_batch,
)


def points_in_cell(count=100, z=.5, slope=0.):
    side = math.ceil(math.sqrt(count))
    xy = np.stack(np.meshgrid(np.linspace(.025, .175, side),
                             np.linspace(.025, .175, side)), axis=-1).reshape(-1, 2)[:count]
    return np.c_[xy, z + math.tan(math.radians(slope)) * xy[:, 0]]


def tiled_ground(size=5, points_per_cell=12, slope_deg=8):
    xyz = []
    for i in range(size):
        for j in range(size):
            points = points_in_cell(points_per_cell)
            points[:, :2] += np.array((i * .2, j * .2))
            points[:, 2] = 1.2 + math.tan(math.radians(slope_deg)) * points[:, 0]
            xyz.extend(points)
    return np.array(xyz)


@pytest.mark.parametrize("algorithm", ["ols", "tls"])
@pytest.mark.parametrize("angle", [0., 5., 15., 35.])
def test_ols_and_tls_recover_exact_plane_at_large_world_translation(algorithm, angle):
    xy = np.stack(np.meshgrid(np.linspace(-.5, .5, 7), np.linspace(-.5, .5, 7)), axis=-1).reshape(-1, 2)
    z = 3.1 + math.tan(math.radians(angle)) * xy[:, 0]
    points = np.c_[xy[:, 0] + 1e6, xy[:, 1] - 2e6, z + 300.]
    fitted = fit_plane(points, CoreConfig(plane_method=algorithm))
    assert fitted is not None
    assert fitted.slope_deg == pytest.approx(angle, abs=1e-7)
    assert fitted.residual_m < 1e-9
    assert fitted.q_space == 1.
    assert np.linalg.norm(fitted.normal) == pytest.approx(1.)
    assert np.asarray(points) @ np.asarray(fitted.normal) + fitted.offset_m == pytest.approx(np.zeros(len(points)), abs=1e-8)


@pytest.mark.parametrize("algorithm", ["ols", "tls"])
def test_plane_identifiability_requires_noncollinear_xy(algorithm):
    cfg = CoreConfig(plane_method=algorithm)
    assert fit_plane(np.zeros((2, 3)), cfg) is None
    assert fit_plane(np.zeros((12, 3)), cfg) is None
    assert fit_plane(np.c_[np.arange(12), 3. * np.arange(12), 2. * np.arange(12)], cfg) is None
    assert fit_plane(np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]]), cfg) is not None


def test_tls_rejects_ambiguous_normal_but_accepts_zero_minimum_eigenvalue():
    sphere_axes = np.r_[np.eye(3), -np.eye(3)]
    assert fit_plane(sphere_axes, CoreConfig(plane_method="tls")) is None
    flat = np.array([[-1, -1, 0], [-1, 1, 0], [1, 1, 0], [1, -1, 0]])
    assert fit_plane(flat, CoreConfig(plane_method="tls")) is not None


def test_geometry_fixed_minimum_half_up_rounding_and_hard_limit():
    assert geometry_cost(0) == 5
    assert geometry_cost(10) == 53
    assert geometry_cost(2.5 * 20 / 95) == 8
    assert geometry_cost(19.999) == 99
    assert geometry_cost(20) == 100
    assert geometry_cost(89) == 100
    with pytest.raises(ValueError):
        geometry_cost(float("nan"))


@pytest.mark.parametrize("fusion", ["weighted", "probabilistic"])
def test_hard_cost_precedence_and_ordinary_confidence_never_promotes_100(fusion):
    cfg = CoreConfig(fusion_method=fusion)
    assert fuse_costs(100, 5, 1, 1, cfg)[0] == 100
    assert fuse_costs(5, 100, 1, 1, cfg)[0] == 100
    assert fuse_costs(None, 5, 0, 1, cfg)[0] == 100
    assert fuse_costs(5, None, 1, 0, cfg)[0] == 100
    assert fuse_costs(99, 99, 0, 0, cfg)[0] == 99
    assert fuse_costs(50, 60, 0, 0, cfg)[1:] == (99., 99.)
    expected = 5 if fusion == "weighted" else 10
    assert fuse_costs(5, 5, 1, 1, cfg)[0] == expected
    assert fuse_costs(5, 5, 0, 0, cfg)[0] >= expected


def test_full_class_cost_contract_only_original_zero_cost_classes_become_five():
    assert set(np.flatnonzero(np.isnan(CLASS_COSTS))) == EXCLUDED_CLASS_IDS
    assert GROUND_CLASS_IDS == {3, 5, 7, 9, 11, 21, 23, 24, 31}
    assert CLASS_COSTS[31] == 20
    assert CLASS_COSTS[5] == 35
    assert CLASS_COSTS[3] == 30
    assert CLASS_COSTS[23] == 5
    assert CLASS_COSTS[50] == 35
    assert CLASS_COSTS[51] == 65
    assert CLASS_COSTS[17] == 95
    assert CLASS_COSTS[28] == 100
    assert np.nanmin(CLASS_COSTS) == 5


def test_semantic_cost_aggregates_all_eligible_points_instead_of_representative():
    core = TraversabilityCore()
    xyz = points_in_cell(100)
    classes = np.r_[np.full(80, 31), np.full(20, 40)]
    xyz[80:, 2] = 50.  # Non-ground height must never contaminate the plane/height.
    core.ingest(xyz, classes, 1.)
    result = core.snapshot(1.)[(0, 0)]
    assert result.semantic_cost == 36
    assert result.representative_class == 31
    assert result.height == pytest.approx(.5)
    assert result.slope_deg == pytest.approx(0.)
    assert result.final_cost < 100


@pytest.mark.parametrize("soil_count,expected", [(55, 40), (56, 31), (50, 40)])
def test_near_tie_non_ground_preference_is_ten_percentage_points(soil_count, expected):
    core = TraversabilityCore()
    classes = np.r_[np.full(soil_count, 31), np.full(100 - soil_count, 40)]
    core.ingest(points_in_cell(), classes, 1.)
    assert core.snapshot(1.)[(0, 0)].representative_class == expected


def test_excluded_class_is_not_preferred_as_an_obstacle_or_eligible_cost():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.r_[np.full(55, 31), np.zeros(45)], 1.)
    result = core.snapshot(1.)[(0, 0)]
    assert result.representative_class == 31
    assert result.semantic_points == 55
    assert result.semantic_cost == 20
    other = TraversabilityCore()
    other.ingest(points_in_cell(), np.resize(np.array([0, 8, 53, 56]), 100), 1.)
    blocked = other.snapshot(1.)[(0, 0)]
    assert blocked.semantic_cost is None
    assert blocked.final_cost == 100
    assert blocked.state == "semantic_only"
    assert blocked.observation_stamp_s == blocked.cost_stamp_s == 1.


def test_representative_height_threshold_is_distinct_from_raw_plane_support():
    xyz = points_in_cell(3, slope=7)
    core = TraversabilityCore()
    core.ingest(xyz, np.full(3, 31), 1.)
    result = core.snapshot(1.)[(0, 0)]
    assert result.state == "observed_insufficient"
    assert result.height is None
    assert not result.semantic_valid
    assert result.semantic_cost == 20
    assert result.q_s == pytest.approx(3 / 20)
    assert result.slope_valid
    assert result.q_count == pytest.approx(3 / 30)
    assert result.final_cost < 100


@pytest.mark.parametrize("height_method", ["mean", "median"])
@pytest.mark.parametrize("plane_method", ["ols", "tls"])
def test_valid_height_cell_neighbourhood_fits_same_physical_plane(height_method, plane_method):
    core = TraversabilityCore(CoreConfig(slope_input="cell_heights", height_method=height_method,
                                        plane_method=plane_method, radius_m=.3))
    xyz = tiled_ground()
    core.ingest(xyz, np.full(len(xyz), 31), 1.)
    result = core.snapshot(1.)[(2, 2)]
    assert result.slope_valid
    assert result.plane_support == 9
    assert result.slope_deg == pytest.approx(8., abs=1e-5)
    assert result.q_count == 1
    assert result.q_space == 1


def test_mean_and_median_are_recorded_and_differ_for_asymmetric_noise():
    xyz = points_in_cell(30)
    xyz[-1, 2] += .3
    results = []
    for method in ["mean", "median"]:
        core = TraversabilityCore(CoreConfig(height_method=method))
        core.ingest(xyz, np.full(30, 31), 1.)
        results.append(core.snapshot(1.)[(0, 0)])
    assert results[0].height == pytest.approx(.51, abs=1e-6)
    assert results[1].height == pytest.approx(.5)
    assert results[0].height_mean == results[1].height_mean
    assert results[0].height_median == results[1].height_median
    # Representative-height choice does not change raw-point slope/cost.
    assert results[0].slope_deg == results[1].slope_deg
    assert results[0].final_cost == results[1].final_cost


def test_space_confidence_uses_covariance_divided_by_n_and_radius_squared():
    points = np.array([[-.1, -.1, 0], [-.1, .1, 0], [.1, -.1, 0], [.1, .1, 0]])
    fit = fit_plane(points, CoreConfig(radius_m=.5))
    assert fit.q_space == pytest.approx(.01 / (.1 * .5 ** 2))


def test_equal_retained_point_weights_across_frames_and_support_based_q():
    core = TraversabilityCore()
    core.ingest(points_in_cell(10), np.full(10, 31), 1.)
    core.snapshot(1.)
    core.ingest(points_in_cell(10), np.full(10, 24), 2.)
    result = core.snapshot(2.)[(0, 0)]
    assert result.semantic_cost == 22.5
    assert result.q_s == 1.
    assert core.class_counts((0, 0)) == {31: 10, 24: 10}


def test_expiry_drops_active_points_but_retains_usable_estimates_and_capture_age():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.full(100, 31), 1.)
    initial = core.snapshot(1.)[(0, 0)]
    result = core.snapshot(11.1)[(0, 0)]
    assert core.active_point_count == 0
    assert result.total_points == result.semantic_points == result.ground_points == 0
    assert result.semantic_support_points == result.height_support_points == 100
    assert result.semantic_cost == initial.semantic_cost
    assert result.slope_deg == initial.slope_deg
    assert result.final_cost == initial.final_cost
    assert result.semantic_reused and result.height_reused and result.slope_reused
    assert result.cost_stamp_s == result.semantic_stamp_s == result.geometry_stamp_s == 1.
    assert result.state == "both_valid"
    assert core.class_counts((0, 0)) == {}
    assert core.diagnostics["time_evicted_points"] == 100
    assert core.snapshot(99.)[(0, 0)].cost_stamp_s == 1.


def test_new_non_ground_observation_does_not_refresh_reused_height_or_slope():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.full(100, 31), 1.)
    core.snapshot(1.)
    core.ingest(points_in_cell(20), np.full(20, 51), 12.)
    result = core.snapshot(12.)[(0, 0)]
    assert result.semantic_cost == 65
    assert result.semantic_stamp_s == result.observation_stamp_s == 12.
    assert result.height_stamp_s == result.slope_stamp_s == result.cost_stamp_s == 1.
    assert result.height_reused and result.slope_reused


def test_fresh_hard_semantic_cost_is_not_expired_by_old_passable_geometry():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.full(100, 23), 1.)
    core.snapshot(1.)
    core.ingest(points_in_cell(30), np.full(30, 40), 40.)
    result = core.snapshot(40.)[(0, 0)]
    assert result.semantic_cost == 100
    assert result.geometry_cost == 5
    assert result.semantic_stamp_s == 40.
    assert result.geometry_stamp_s == 1.
    assert result.final_cost == 100
    assert result.cost_stamp_s == 40.
    # The old passable geometry has expired under the inherited camera TTL;
    # the newly observed blocking semantic evidence still has the full 30s.
    assert 40. - result.geometry_stamp_s > 30.
    assert 40. - result.cost_stamp_s <= 30.
    assert core.snapshot(69.9)[(0, 0)].cost_stamp_s == 40.


def test_fresh_hard_geometry_is_not_expired_by_old_passable_semantic_cost():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.full(100, 23), 1.)
    core.snapshot(1.)
    # Target receives excluded points. Eligible ground for the new slope is in
    # its neighbour, so the target's semantic cost correctly remains from t=1.
    target = points_in_cell(30)
    ground = points_in_cell(30, slope=30.)
    ground[:, :2] += np.array((.2, 0))
    core.ingest(np.r_[target, ground], np.r_[np.zeros(30), np.full(30, 31)], 40.)
    result = core.snapshot(40.)[(0, 0)]
    assert result.semantic_cost == 5
    assert result.geometry_cost == 100
    assert result.semantic_stamp_s == 1.
    assert result.geometry_stamp_s == 40.
    assert result.final_cost == 100
    assert result.cost_stamp_s == 40.
    assert 40. - result.semantic_stamp_s > 30.
    assert 40. - result.cost_stamp_s <= 30.


def test_new_passable_component_does_not_refresh_an_old_hard_constraint():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.full(100, 40), 1.)
    core.snapshot(1.)
    # Semantic history is hard=100. New ground occurs only in a neighbour;
    # observing the target as excluded supplies no new semantic evidence.
    ground = points_in_cell(30) + np.array((.2, 0, 0))
    core.ingest(np.r_[points_in_cell(30), ground], np.r_[np.zeros(30), np.full(30, 31)], 40.)
    result = core.snapshot(40.)[(0, 0)]
    assert result.semantic_cost == 100
    assert result.geometry_cost == 5
    assert result.semantic_stamp_s == 1.
    assert result.geometry_stamp_s == 40.
    assert result.cost_stamp_s == 1.


def test_missing_component_blocking_uses_actual_current_observation_stamp():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.zeros(100), 40.)
    result = core.snapshot(40.)[(0, 0)]
    assert result.semantic_cost is None and result.geometry_cost is None
    assert result.final_cost == 100
    assert result.cost_stamp_s == result.observation_stamp_s == 40.


def test_retained_cell_heights_have_real_old_support_ages():
    core = TraversabilityCore(CoreConfig(slope_input="cell_heights", radius_m=.3))
    xyz = tiled_ground()
    core.ingest(xyz, np.full(len(xyz), 31), 1.)
    core.snapshot(1.)
    result = core.snapshot(12.)[(2, 2)]
    assert core.active_point_count == 0
    assert result.slope_valid
    assert result.support_oldest_stamp_s == result.support_newest_stamp_s == 1.
    assert result.cost_stamp_s == 1.


def test_caps_remove_old_capture_groups_and_sample_within_frame_without_pixel_bias():
    cfg = CoreConfig(per_cell_max_points=25, global_max_points=100)
    frames = []
    for _ in range(2):
        core = TraversabilityCore(cfg)
        core.ingest(points_in_cell(15, z=0), np.full(15, 31), 1.)
        core.snapshot(1.)
        # z encodes original point order and is not used as a semantic input.
        xyz = points_in_cell(100)
        xyz[:, 2] = np.arange(100)
        core.ingest(xyz, np.full(100, 24), 2.)
        result = core.snapshot(2.)[(0, 0)]
        assert result.total_points == 25
        assert core.class_counts((0, 0)) == {24: 25}
        chosen_z = core._cells[(0, 0)].points["xyz"][:, 2].copy()
        assert chosen_z.min() < 20
        assert chosen_z.max() > 80
        frames.append(chosen_z)
    np.testing.assert_array_equal(frames[0], frames[1])


def test_global_cap_and_counts_are_bounded_by_actual_retained_points():
    core = TraversabilityCore(CoreConfig(per_cell_max_points=50, global_max_points=75))
    first = points_in_cell(50)
    second = points_in_cell(50) + np.array((1., 0, 0))
    core.ingest(first, np.full(50, 31), 1.)
    core.snapshot(1.)
    core.ingest(second, np.full(50, 24), 2.)
    result = core.snapshot(2.)
    assert core.active_point_count == 75
    assert sum(c.total_points for c in result.values()) == 75
    assert result[(0, 0)].total_points == 25
    assert result[(5, 0)].total_points == 50
    assert core.retained_payload_bytes == 75 * 24
    assert core.diagnostics["global_cap_evicted_points"] == 25


def test_backward_capture_resets_trial_while_processing_lag_does_not():
    core = TraversabilityCore()
    core.ingest(points_in_cell(), np.full(100, 31), 10.)
    core.snapshot(10.1)
    core.ingest(points_in_cell(), np.full(100, 24), 10.05)
    assert core.snapshot(10.1)[(0, 0)].semantic_cost == 22.5
    core.ingest(points_in_cell(3), np.full(3, 23), 1.)
    result = core.snapshot(1.)[(0, 0)]
    assert result.total_points == 3
    assert result.semantic_cost == 5
    assert core.diagnostics["clock_backward_resets"] == 1


def test_never_observed_is_absent_and_invalid_inputs_do_not_observe_cells():
    core = TraversabilityCore()
    assert core.snapshot(0.) == {}
    points = np.array([[.1, .1, 0], [.1, .1, 0], [.1, .1, 0], [np.nan, 0, 0]])
    core.ingest(points, np.array([-1, 64, 1.5, 31]), 1.)
    assert core.snapshot(1.) == {}
    assert core.diagnostics["invalid_class_points"] == 3
    assert core.diagnostics["invalid_xyz_points"] == 1
    with pytest.raises(ValueError):
        core.ingest(points, np.array([31]), 1.)
    with pytest.raises(ValueError):
        core.ingest(points, np.full(4, 31), float("nan"))


def test_dirty_neighbourhoods_refit_when_neighbour_changes_and_idle_snapshot_is_cached():
    core = TraversabilityCore(CoreConfig(radius_m=.8))
    xyz = tiled_ground(size=3)
    core.ingest(xyz, np.full(len(xyz), 31), 1.)
    core.snapshot(1.)
    fits = core.diagnostics["plane_fit_attempts"]
    core.snapshot(1.1)
    assert core.diagnostics["plane_fit_attempts"] == fits
    core.ingest(points_in_cell(3) + np.array((.6, .2, 0)), np.full(3, 31), 2.)
    core.snapshot(2.)
    assert core.diagnostics["plane_fit_attempts"] > fits + 1


def test_batched_tls_matches_scalar_fits_with_noise_and_underfilled_support():
    rng = np.random.default_rng(55)
    xyz = rng.normal(size=(300, 3)) + np.array((1e6, -2e6, 5.))
    xyz[:, 2] = .3 * (xyz[:, 0] - 1e6) + .2 * (xyz[:, 1] + 2e6) + rng.normal(0, .01, 300)
    indices = [rng.choice(300, count, replace=False) for count in [0, 1, 2, 3, 10, 30, 100]]
    batched = _fit_tls_batch(xyz, indices, CoreConfig())
    for support, result in zip(indices, batched):
        scalar = fit_plane(xyz[support], CoreConfig())
        assert (result is None) == (scalar is None)
        if result is not None:
            assert result.slope_deg == pytest.approx(scalar.slope_deg, abs=1e-9)
            assert result.q_space == pytest.approx(scalar.q_space, abs=1e-9)
            assert result.residual_m == pytest.approx(scalar.residual_m, abs=1e-9)


def test_repeated_sparse_frames_preserve_global_per_cell_time_and_count_bounds():
    rng = np.random.default_rng(56)
    cfg = CoreConfig(per_cell_max_points=40, global_max_points=300, raw_window_s=1.)
    core = TraversabilityCore(cfg)
    for frame in range(35):
        stamp = frame * .1
        xyz = np.c_[rng.uniform(-1, 1, 100), rng.uniform(-1, 1, 100), rng.normal(0, .01, 100)]
        core.ingest(xyz, rng.integers(0, 64, 100), stamp)
        estimates = core.snapshot(stamp)
        assert core.active_point_count <= cfg.global_max_points
        assert core.active_point_count == sum(c.total_points for c in estimates.values())
        assert all(c.total_points <= cfg.per_cell_max_points for c in estimates.values())
        assert all(np.all(c.points["stamp"] >= stamp - cfg.raw_window_s) for c in core._cells.values())
        assert all(np.all(np.diff(c.points["stamp"]) >= 0) for c in core._cells.values())
        assert all(0 <= c.final_cost <= 100 for c in estimates.values())


@pytest.mark.parametrize("kwargs", [
    {"radius_m": 0}, {"resolution_m": float("nan")}, {"height_min_ground_points": 0},
    {"fusion_method": "ransac"}, {"semantic_weight": .4}, {"slope_input": "other"},
    {"near_tie_share_difference": 2}, {"semantic_confidence_alpha": -.1},
])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        CoreConfig(**kwargs)
