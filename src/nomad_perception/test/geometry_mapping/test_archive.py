"""Verify persisted values, recoverable failure, and readable final exports."""
import csv
import json
from xml.etree import ElementTree

import numpy as np
import pytest

from nomad_geometry_mapping import archive


def packet(empty_path=False):
    arrays = {
        "cost": np.array([[-1, 0, 99], [100, 12, -1]], dtype=np.int8),
        "slope_deg": np.array([[np.nan, 0., 35.], [55., 7.5, np.nan]], dtype=np.float32),
        "height_m": np.array([[np.nan, 1., 2.], [3., 1.5, np.nan]], dtype=np.float32),
        "trajectory": np.empty((0, 8), dtype=np.float64) if empty_path else np.array([
            [1., -2., 4., .2, 0., 0., 0., 1.], [2., -1.8, 4.2, .3, 0., 0., .5, .866025403784]], dtype=np.float64),
        "sparse_cells": np.array([[-10, 20]], dtype=np.int64),
    }
    metadata = {"resolution_m": .2, "origin_x": -2., "origin_y": 4., "frame_id": "odom",
                "stamp_s": 2., "version": 3, "map_id": "07_vio_undulating_terrain",
                "candidate": {"radius_m": .3, "representation": "raw", "fit_method": "tls"}}
    return arrays, metadata


def test_roundtrip_preserves_signed_unknown_nan_and_geometry(tmp_path):
    arrays, metadata = packet()
    path = archive.write_snapshot(tmp_path, arrays, metadata)
    stored, info = archive.load_snapshot(path)
    for name, expected in arrays.items():
        np.testing.assert_array_equal(stored[name], expected)
        assert stored[name].dtype == expected.dtype
    assert info["resolution_m"] == .2
    assert info["origin_x"] == -2. and info["origin_y"] == 4.
    assert info["candidate"]["radius_m"] == .3
    assert info["status"] == "running"
    assert archive.resolve_snapshot(tmp_path) == path


def test_failed_atomic_npz_write_keeps_previous_checkpoint(tmp_path, monkeypatch):
    arrays, metadata = packet()
    path = archive.write_snapshot(tmp_path, arrays, metadata)
    previous = path.read_bytes()
    manifest = (tmp_path / "manifest.json").read_bytes()

    def fail(stream, **values):
        stream.write(b"incomplete")
        raise OSError("simulated storage failure")

    monkeypatch.setattr(archive.np, "savez_compressed", fail)
    with pytest.raises(OSError, match="storage failure"):
        archive.write_snapshot(tmp_path, arrays, {**metadata, "version": 4})
    assert path.read_bytes() == previous
    assert (tmp_path / "manifest.json").read_bytes() == manifest
    assert not list(tmp_path.glob(".*.tmp"))


def test_failed_final_export_does_not_mark_closed_or_replace_checkpoint(tmp_path, monkeypatch):
    arrays, metadata = packet()
    path = archive.write_snapshot(tmp_path, arrays, metadata)
    previous = path.read_bytes()
    manifest = (tmp_path / "manifest.json").read_bytes()

    def fail(*args):
        raise OSError("plot export failed")

    monkeypatch.setattr(archive, "_export", fail)
    with pytest.raises(OSError, match="export failed"):
        archive.write_snapshot(tmp_path, arrays, metadata, final=True)
    assert path.read_bytes() == previous
    assert (tmp_path / "manifest.json").read_bytes() == manifest
    assert not (tmp_path / "map.npz").exists()


def test_empty_path_final_exports_and_manifest(tmp_path):
    arrays, metadata = packet(empty_path=True)
    path = archive.write_snapshot(tmp_path, arrays, metadata, final=True)
    stored, info = archive.load_snapshot(path)
    assert stored["trajectory"].shape == (0, 8)
    assert info["status"] == "closed"
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["snapshot_file"] == "map.npz" and manifest["status"] == "closed"
    for filename in archive.EXPORT_FILES:
        assert (tmp_path / filename).stat().st_size > 0
    with (tmp_path / "cells.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 6
    assert rows[0]["cost"] == "-1" and rows[0]["slope_deg"] == "nan"
    assert float(rows[0]["x_center_m"]) == pytest.approx(-1.9)
    assert float(rows[0]["y_center_m"]) == pytest.approx(4.1)
    with (tmp_path / "trajectory.csv").open() as stream:
        assert list(csv.DictReader(stream)) == []


def test_resolver_recovers_from_broken_manifest_or_final(tmp_path):
    arrays, metadata = packet()
    path = archive.write_snapshot(tmp_path, arrays, metadata)
    (tmp_path / "map.npz").write_bytes(b"incomplete")
    (tmp_path / "manifest.json").write_text('{"snapshot_file": "map.npz"}')
    assert archive.resolve_snapshot(tmp_path) == path
    (tmp_path / "manifest.json").write_text("incomplete")
    assert archive.resolve_snapshot(tmp_path) == path


def test_resolver_recovers_from_truncated_npz_zip(tmp_path):
    arrays, metadata = packet()
    path = archive.write_snapshot(tmp_path, arrays, metadata)
    (tmp_path / "map.npz").write_bytes(path.read_bytes()[:80])
    (tmp_path / "manifest.json").write_text('{"snapshot_file": "map.npz"}')
    assert archive.resolve_snapshot(tmp_path) == path


def test_viewer_check_reads_session_without_ros(tmp_path, capsys):
    from nomad_geometry_mapping.viewer import main
    arrays, metadata = packet()
    archive.write_snapshot(tmp_path, arrays, metadata)
    assert main(["--snapshot", str(tmp_path), "--check"]) == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed["version"] == metadata["version"]
    assert printed["frame_id"] == "odom"


@pytest.mark.parametrize("bad", [np.ones((1, 7), dtype=np.float64),
                                  np.full((1, 8), np.nan, dtype=np.float64)])
def test_incomplete_or_invalid_trajectory_rejected_without_files(tmp_path, bad):
    arrays, metadata = packet()
    arrays["trajectory"] = bad
    with pytest.raises(ValueError, match="trajectory"):
        archive.write_snapshot(tmp_path, arrays, metadata)
    assert list(tmp_path.iterdir()) == []


def test_unsigned_cost_refused(tmp_path):
    arrays, metadata = packet()
    arrays["cost"] = arrays["cost"].astype(np.uint8)
    with pytest.raises(ValueError, match="int8"):
        archive.write_snapshot(tmp_path, arrays, metadata)


def test_numbered_svg_labels_every_observed_cell_with_degree_or_na(tmp_path):
    arrays, metadata = packet()
    arrays["cost"][0, 0] = 12
    arrays["slope_deg"][0, 2] = 13.5
    archive._export_numbered_svg(tmp_path, arrays, metadata)
    tree = ElementTree.parse(tmp_path / "numbered_map.svg")
    labels = [element for element in tree.iter() if element.attrib.get("class") == "slope-label"]
    assert len(labels) == np.count_nonzero(arrays["cost"] >= 0)
    assert "13.5°" in [element.text for element in labels]
    assert "N/A" in [element.text for element in labels]
    assert all(element.text == "N/A" or element.text.endswith("°") for element in labels)
    assert tree.find(".//*[@id='trajectory']") is not None
    texts = [element.text for element in tree.iter() if element.tag.endswith("text")]
    assert "odom X (m)" in texts and "odom Y (m)" in texts


def test_numbered_cost_svg_uses_exact_scores_including_unfit_cells(tmp_path):
    arrays, metadata = packet()
    arrays["slope_deg"][1, 0] = np.nan
    archive.write_snapshot(tmp_path, arrays, metadata, final=True)
    cost_tree = ElementTree.parse(tmp_path / "numbered_cost_map.svg")
    labels = [element for element in cost_tree.iter() if element.attrib.get("class") == "cost-label"]
    expected = {(int(row), int(column)): str(int(arrays["cost"][row, column]))
                for row, column in zip(*np.where(arrays["cost"] >= 0))}
    actual = {(int(element.attrib["data-row"]), int(element.attrib["data-column"])): element.text
              for element in labels}
    assert actual == expected
    assert actual[(1, 0)] == "100"  # A missing slope still has a stored cost.
    assert all(element.text.isdigit() and "°" not in element.text for element in labels)
    assert cost_tree.find(".//*[@id='trajectory']") is not None
    slope_tree = ElementTree.parse(tmp_path / "numbered_map.svg")
    slope_labels = {(int(element.attrib["data-row"]), int(element.attrib["data-column"])): element
                    for element in slope_tree.iter() if element.attrib.get("class") == "slope-label"}
    for element in labels:
        index = (int(element.attrib["data-row"]), int(element.attrib["data-column"]))
        for attribute in ("x", "y", "fill"):
            assert element.attrib[attribute] == slope_labels[index].attrib[attribute]
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert "numbered_cost_map.svg" in manifest["exports"]
    assert "numbered_map.svg" in manifest["exports"]
