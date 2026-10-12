"""Cost conversion tests independent of ROS and simulation processes."""
from dataclasses import FrozenInstanceError
from types import SimpleNamespace

import numpy as np
import pytest

from nomad_geometry_mapping.cost_policy import GeometryCostPolicy


def legacy_costs(slopes, confidence):
    base = np.minimum(99, np.floor(5.0 + 95.0 * slopes / 20.0 + 0.5))
    return np.where(slopes >= 20.0, 100,
                    np.minimum(99, np.floor(np.minimum(99, base * (2.0 - confidence)) + 0.5))).astype(np.int8)


def test_recorded_examples_are_reduced_without_changing_angles():
    slopes = np.array([1.297006, 9.890100], dtype=np.float32)
    confidence = np.array([0.033394, 0.316271])
    original = np.array([22, 88], dtype=np.int8)
    result = GeometryCostPolicy().apply(original, slopes, confidence)
    np.testing.assert_array_equal(result, [1, 28])
    np.testing.assert_array_equal(original, [22, 88])
    np.testing.assert_array_equal(slopes, np.array([1.297006, 9.890100], dtype=np.float32))
    assert result.dtype == original.dtype


def test_unknown_blocked_and_unfit_decisions_are_preserved():
    cost = np.array([-1, -1, 100, 100, 11, 11, 11, 99, 0], dtype=np.int16)
    slopes = np.array([np.nan, 1.0, 0.0, np.nan, np.nan, -0.1, 20.0, 25.0, 0.0])
    result = GeometryCostPolicy().apply(cost, slopes, np.ones(cost.shape))
    np.testing.assert_array_equal(result, [-1, -1, 100, 100, 100, 100, 100, 100, 0])
    assert result.dtype == np.int16


def test_legacy_is_an_exact_independent_copy_even_for_invalid_slopes():
    old = np.array([[-1, 22, 88, 99, 100]], dtype=np.int8)
    result = GeometryCostPolicy(name="legacy").apply(old, np.full(old.shape, np.nan), np.full(old.shape, np.nan))
    np.testing.assert_array_equal(result, old)
    assert not np.shares_memory(result, old)


def test_nan_confidence_is_conservative_and_extremes_are_clipped():
    cost = np.full(6, 53, dtype=np.int8)
    result = GeometryCostPolicy().apply(cost, np.full(6, 10.0), np.array([np.nan, 0.0, -1.0, 1.0, 2.0, np.inf]))
    np.testing.assert_array_equal(result, [31, 31, 31, 25, 25, 25])


def test_monotonicity_and_no_cost_increase_across_the_full_ordinary_range():
    angles, confidence = np.meshgrid(np.linspace(0.0, 20.0, 2001), np.linspace(0.0, 1.0, 101))
    old = legacy_costs(angles, confidence)
    new = GeometryCostPolicy().apply(old, angles, confidence)
    assert np.all(new <= old)
    assert np.all(np.diff(new.astype(np.int16), axis=1) >= 0)
    assert np.all(np.diff(new.astype(np.int16), axis=0) <= 0)
    assert np.all(new[:, -1] == 100)
    assert np.all(new[:, 0] == 0)
    assert np.all(new[:, :-1] <= 99)


@pytest.mark.parametrize("kwargs", [
    {"name": "unknown"}, {"limit_deg": 0}, {"limit_deg": -1},
    {"limit_deg": np.nan}, {"limit_deg": np.inf}, {"power": 0.9},
    {"power": np.nan}, {"power": np.inf}, {"confidence_gain": -0.1},
    {"confidence_gain": 1.1}, {"confidence_gain": np.nan},
])
def test_invalid_policy_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        GeometryCostPolicy(**kwargs)


def test_policy_is_immutable_and_describes_saved_interpretation():
    policy = GeometryCostPolicy()
    with pytest.raises(FrozenInstanceError):
        policy.power = 1.0
    assert policy.metadata() == {
        "name": "angle_power_v1", "limit_deg": 20.0, "power": 2.0,
        "confidence_gain": 0.25, "rounding": "half_up",
        "neighbor_confidence": {
            "enabled": True, "target_below": 0.3, "neighbor_min": 0.7,
            "required_neighbors": 8, "average": "arithmetic_mean",
            "source": "original_confidence_single_pass",
        },
    }


@pytest.mark.parametrize("cost,slopes,q", [
    (np.array([0], dtype=np.uint8), [0], [1]),
    (np.array([0.0]), [0], [1]),
    (np.array([-2], dtype=np.int8), [0], [1]),
    (np.array([101], dtype=np.int16), [0], [1]),
    (np.array([0, 1], dtype=np.int8), [0], [1, 1]),
    (np.array([0], dtype=np.int8), [0], [1, 1]),
])
def test_invalid_cost_arrays_are_rejected(cost, slopes, q):
    with pytest.raises(ValueError):
        GeometryCostPolicy().apply(cost, slopes, q)


def test_dense_and_sparse_costs_follow_the_same_policy():
    class Grid:
        width = 3
        height = 2
        resolution = 0.2

        @staticmethod
        def cell(x, y):
            return int(np.floor(x / 0.2)), int(np.floor(y / 0.2))

    estimates = {
        (0, 0): SimpleNamespace(geometry_cost=11, slope_deg=1.297006, q_g=0.033394),
        (1, 0): SimpleNamespace(geometry_cost=52, slope_deg=9.890100, q_g=0.316271),
        (2, 0): SimpleNamespace(geometry_cost=None, slope_deg=None, q_g=0.0),
        (0, 1): SimpleNamespace(geometry_cost=100, slope_deg=21.0, q_g=1.0),
        (99, 99): SimpleNamespace(geometry_cost=11, slope_deg=1.0, q_g=1.0),
    }
    policy = GeometryCostPolicy()
    dense, slopes = policy.layers(estimates, Grid())
    np.testing.assert_array_equal(dense, [[1, 28, 100], [100, -1, -1]])
    np.testing.assert_allclose(slopes[0, :2], [1.297006, 9.890100])
    assert np.isnan(slopes[0, 2])
    assert np.isnan(slopes[1, 1:]).all()
    sparse = policy.sparse_costs({
        "output_cost": np.array([22, 88, 100, 100], dtype=np.int16),
        "slope_deg": np.array([1.297006, 9.890100, np.nan, 21.0]),
        "q_g": np.array([0.033394, 0.316271, 0.0, 1.0]),
    })
    np.testing.assert_array_equal(sparse, dense[[0, 0, 0, 1], [0, 1, 2, 0]])


def neighborhood(center=(0, 0)):
    cells = np.array([(center[0] + dx, center[1] + dy)
                      for dy in (-1, 0, 1) for dx in (-1, 0, 1)], dtype=np.int64)
    q = np.array([0.7, 0.75, 0.8, 0.85, 0.2, 0.9, 0.95, 1.0, 0.8])
    return cells, np.full(9, 10.0), q


def test_isolated_confidence_uses_exact_eight_neighbor_mean_and_keeps_inputs():
    cells, slopes, q = neighborhood(center=(-4, -7))
    before = q.copy()
    effective, corrected = GeometryCostPolicy().correct_confidence(cells, slopes, q)
    assert effective[4] == pytest.approx(np.delete(q, 4).mean())
    np.testing.assert_array_equal(corrected, np.arange(9) == 4)
    np.testing.assert_array_equal(effective[np.arange(9) != 4], q[np.arange(9) != 4])
    np.testing.assert_array_equal(q, before)
    np.testing.assert_array_equal(slopes, np.full(9, 10.0))


@pytest.mark.parametrize("neighbor", [0, 1, 2, 3, 5, 6, 7, 8])
def test_every_cardinal_and_diagonal_neighbor_must_meet_threshold(neighbor):
    cells, slopes, q = neighborhood()
    q[neighbor] = np.nextafter(0.7, 0.0)
    effective, corrected = GeometryCostPolicy().correct_confidence(cells, slopes, q)
    np.testing.assert_array_equal(effective, q)
    assert not corrected.any()


def test_target_threshold_is_strict_and_neighbor_threshold_is_inclusive():
    cells, slopes, q = neighborhood()
    q[:] = 0.7
    q[4] = 0.3
    effective, corrected = GeometryCostPolicy().correct_confidence(cells, slopes, q)
    assert effective[4] == 0.3 and not corrected.any()
    q[4] = np.nextafter(0.3, 0.0)
    effective, corrected = GeometryCostPolicy().correct_confidence(cells, slopes, q)
    assert effective[4] == pytest.approx(0.7) and corrected[4]


@pytest.mark.parametrize("problem", ["missing", "nan_slope", "nan_q", "inf_q", "out_of_range_q"])
def test_missing_or_invalid_neighbor_cannot_supply_confidence(problem):
    cells, slopes, q = neighborhood()
    if problem == "missing":
        cells, slopes, q = (np.delete(value, 0, axis=0) for value in (cells, slopes, q))
    elif problem == "nan_slope":
        slopes[0] = np.nan
    else:
        q[0] = {"nan_q": np.nan, "inf_q": np.inf, "out_of_range_q": 1.1}[problem]
    effective, corrected = GeometryCostPolicy().correct_confidence(cells, slopes, q)
    np.testing.assert_array_equal(effective, q)
    assert not corrected.any()


def test_second_ring_cannot_replace_a_missing_immediate_neighbor():
    cells, slopes, q = neighborhood()
    cells[0] = [-2, -2]
    effective, corrected = GeometryCostPolicy().correct_confidence(cells, slopes, q)
    np.testing.assert_array_equal(effective, q)
    assert not corrected.any()


def test_adjacent_low_cells_are_not_corrected_and_order_does_not_matter():
    cells, slopes, q = neighborhood()
    q[5] = 0.1
    policy = GeometryCostPolicy()
    for order in (np.arange(9), np.arange(8, -1, -1)):
        effective, corrected = policy.correct_confidence(cells[order], slopes[order], q[order])
        np.testing.assert_array_equal(effective, q[order])
        assert not corrected.any()


@pytest.mark.parametrize("slope", [np.nan, -1.0])
def test_invalid_target_slope_does_not_gain_confidence(slope):
    cells, slopes, q = neighborhood()
    slopes[4] = slope
    effective, corrected = GeometryCostPolicy().correct_confidence(cells, slopes, q)
    assert effective[4] == q[4] and not corrected.any()


@pytest.mark.parametrize("policy", [GeometryCostPolicy(neighbor_confidence_enabled=False),
                                    GeometryCostPolicy(name="legacy")])
def test_correction_can_be_disabled_and_legacy_is_unchanged(policy):
    cells, slopes, q = neighborhood()
    effective, corrected = policy.correct_confidence(cells, slopes, q)
    np.testing.assert_array_equal(effective, q)
    assert not corrected.any()


@pytest.mark.parametrize("kwargs", [
    {"neighbor_confidence_enabled": 1}, {"confidence_low_threshold": -0.1},
    {"confidence_low_threshold": np.nan}, {"confidence_neighbor_min": np.inf},
    {"confidence_low_threshold": 0.7}, {"confidence_neighbor_min": 0.2},
    {"confidence_neighbor_min": 1.1},
])
def test_invalid_confidence_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        GeometryCostPolicy(**kwargs)


def test_corrected_dense_and_sparse_costs_match_and_keep_blocking_slopes():
    class Grid:
        width = height = 3
        resolution = 0.2

        @staticmethod
        def cell(x, y):
            return int(np.floor(x / 0.2)) + 1, int(np.floor(y / 0.2)) + 1

    cells, slopes, q = neighborhood()
    policy = GeometryCostPolicy()
    for target_slope in (10.0, 25.0, np.nan):
        slopes[4] = target_slope
        estimates = {tuple(cell): SimpleNamespace(
            geometry_cost=None if not np.isfinite(slope) else 100 if slope >= 20 else 53,
            slope_deg=None if not np.isfinite(slope) else slope, q_g=float(confidence))
            for cell, slope, confidence in zip(cells, slopes, q)}
        dense, angles = policy.layers(estimates, Grid())
        state = {"cells": cells, "slope_deg": slopes, "q_g": q,
                 "output_cost": np.where(np.isfinite(slopes) & (slopes < 20), 64, 100).astype(np.int16)}
        sparse = policy.sparse_costs(state)
        np.testing.assert_array_equal(dense.ravel(), sparse)
        np.testing.assert_allclose(angles.ravel(), slopes, equal_nan=True)
        if target_slope == 10:
            assert dense[1, 1] == 26
            assert policy.apply(state["output_cost"], slopes, q)[4] == 30
        else:
            assert dense[1, 1] == 100
