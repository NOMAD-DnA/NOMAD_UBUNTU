"""Numerical and persistence contracts for the selected manual mapper input.

These checks do not import ROS or start a simulator. The geometry implementation
is the existing unchanged nomad_perception core, shared with the measured
raw_r0.3_tls candidate.
"""
from dataclasses import asdict, fields
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE))

from nomad_perception.traversability_core import CoreConfig, TraversabilityCore
from nomad_perception.stage1_geometry_layers import geometry_layers, sparse_state
from nomad_perception.traversability_output import CostViews


def configured_core():
    params = yaml.safe_load(
        (PACKAGE / "config" / "geometry_mapping.yaml").read_text()
    )["nomad_manual_mapper"]["ros__parameters"]
    names = {item.name for item in fields(CoreConfig)}
    return TraversabilityCore(CoreConfig(**{k: v for k, v in params.items() if k in names}))


def plane_points(degrees, center=(0.1, 0.1), offset=0.0):
    xy = np.linspace(-0.25, 0.25, 21)
    xx, yy = np.meshgrid(xy + center[0], xy + center[1])
    zz = np.tan(np.deg2rad(degrees)) * (xx - center[0]) + offset
    return np.column_stack((xx.ravel(), yy.ravel(), zz.ravel())).astype(np.float32)


def ingest_plane(core, degrees, stamp=1.0, center=(0.1, 0.1), offset=0.0):
    xyz = plane_points(degrees, center, offset)
    core.ingest(xyz, np.full(len(xyz), 31, np.uint32), stamp)
    return core.snapshot(stamp)


def test_settings_match_selected_formal_candidate_exactly():
    experiment = yaml.safe_load(
        (Path(__file__).parent / "fixtures" / "reference_core.yaml").read_text()
    )
    expected = CoreConfig(**{
        **experiment["core"], "geometry_only": True, "radius_m": 0.3,
        "slope_input": "raw_points", "height_method": "median", "plane_method": "tls",
    })
    assert asdict(configured_core().config) == asdict(expected)
    assert experiment["projection_pixel_center_offset_px"] == 0.5


@pytest.mark.parametrize("degrees", [0.0, 12.0, 25.0])
def test_tls_recovers_known_plane_and_geometric_cost(degrees):
    core = configured_core()
    estimate = ingest_plane(core, degrees)[(0, 0)]
    assert estimate.slope_valid
    assert estimate.slope_deg == pytest.approx(degrees, abs=1e-5)
    assert estimate.semantic_cost is None
    assert estimate.plane_support >= core.config.raw_reference_points
    assert estimate.q_g == pytest.approx(1.0)
    expected_cost = 100 if degrees >= 20.0 else min(99, int(np.floor(5 + 95 * degrees / 20 + 0.5)))
    assert estimate.final_cost == expected_cost


def test_cumulative_map_retains_old_plane_after_raw_point_expiry():
    core = configured_core()
    first = ingest_plane(core, 12.0)[(0, 0)]
    snapshot = ingest_plane(core, 3.0, stamp=20.0, center=(3.1, 0.1), offset=0.6)
    retained = snapshot[(0, 0)]
    assert retained.slope_valid and retained.slope_reused
    assert retained.slope_deg == first.slope_deg
    assert retained.final_cost == first.final_cost
    assert retained.slope_stamp_s == first.slope_stamp_s == 1.0
    assert retained.observation_stamp_s == 1.0
    assert retained.ground_points == 0
    assert snapshot[(15, 0)].slope_stamp_s == 20.0
    assert core.active_point_count > 0
    assert core.diagnostics["time_evicted_points"] > 0


def test_geometry_grid_keeps_unknown_separate_from_failed_ground_fit():
    core = configured_core()
    xyz = np.array([[0.1, 0.1, 0.0], [3.1, 0.1, 0.0]], dtype=np.float32)
    # A single valid ground observation cannot identify a plane. A nonground
    # observation must not manufacture a ground map cell.
    core.ingest(xyz, np.array([31, 40], np.uint32), 1.0)
    snapshot = core.snapshot(1.0)
    assert set(snapshot) == {(0, 0)}
    grid = CostViews(.2, initial_size=8.0, max_size=8.0).grid
    cost, angles = geometry_layers(snapshot, grid)
    cx, cy = grid.cell(.1, .1)
    assert cost[cy, cx] == 100
    assert np.isnan(angles[cy, cx])
    ux, uy = grid.cell(3.1, .1)
    assert cost[uy, ux] == -1
    assert np.isnan(angles[uy, ux])


def test_saved_sparse_state_preserves_retained_support_times_and_cost():
    core = configured_core()
    initial = ingest_plane(core, 12.0)
    snapshot = core.snapshot(20.0)
    state = sparse_state(snapshot, core, 20.0, {(0, 0): 1.0})
    row = int(np.flatnonzero(np.all(state["cells"] == [0, 0], axis=1))[0])
    assert state["valid"][row]
    assert state["reused"][row]
    assert state["slope_deg"][row] == pytest.approx(initial[(0, 0)].slope_deg)
    assert state["output_cost"][row] == initial[(0, 0)].final_cost
    assert state["support_oldest_stamp_s"][row] == 1.0
    assert state["support_newest_stamp_s"][row] == 1.0
    assert state["fit_calculation_stamp_s"][row] == 1.0
