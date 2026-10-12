"""Lossless, atomic snapshots and readable exports for a manual mapping session."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Callable
from zipfile import BadZipFile
from xml.sax.saxutils import escape

import numpy as np


SCHEMA_VERSION = 1
METADATA_KEY = "__metadata_json__"
SLOPE_COLOR_MAX_DEG = 45.0
EXPORT_FILES = ("cost_map.png", "slope_map.png", "numbered_map.svg", "numbered_cost_map.svg",
                "cells.csv", "trajectory.csv")


def _json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Not JSON serializable: {type(value).__name__}")


def _json_text(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False,
                      default=_json_default)


def _validate(arrays, metadata):
    """Validate before touching an existing snapshot; never coerce signed cost."""
    if not isinstance(metadata, dict):
        raise ValueError("Snapshot metadata must be a dictionary")
    result = {}
    for name, value in arrays.items():
        if (not isinstance(name, str) or not name or name == METADATA_KEY
                or any(char in name for char in ("/", "\\", "\x00"))):
            raise ValueError("Snapshot array names must be plain, nonreserved names")
        value = np.asarray(value)
        if value.dtype.hasobject:
            raise ValueError("Object and pickle arrays are not supported")
        result[name] = value
    for name in ("cost", "slope_deg", "height_m", "trajectory"):
        if name not in result:
            raise ValueError(f"Snapshot is missing {name}")
    cost = result["cost"]
    if cost.dtype != np.dtype("int8") or cost.ndim != 2 or not all(cost.shape):
        raise ValueError("cost must be a nonempty 2D int8 grid")
    if np.any((cost < -1) | (cost > 100)):
        raise ValueError("cost values must be -1 (unknown) or 0..100")
    for name in ("slope_deg", "height_m"):
        value = result[name]
        if value.dtype != np.dtype("float32") or value.shape != cost.shape:
            raise ValueError(f"{name} must be a float32 grid with the cost shape")
        if np.isinf(value).any():
            raise ValueError(f"{name} may contain NaN for missing values, not infinity")
    trajectory = result["trajectory"]
    if trajectory.size == 0 and trajectory.ndim == 1:
        trajectory = trajectory.reshape(0, 8)
        result["trajectory"] = trajectory
    if (trajectory.dtype != np.dtype("float64") or trajectory.ndim != 2
            or trajectory.shape[1] != 8 or not np.isfinite(trajectory).all()):
        raise ValueError("trajectory must be finite float64 Nx8: stamp, XYZ, quaternion XYZW")
    if len(trajectory) and np.any(np.linalg.norm(trajectory[:, 4:8], axis=1) < 1e-12):
        raise ValueError("Trajectory quaternion must have nonzero norm")
    meta = json.loads(_json_text(metadata))
    for key in ("resolution_m", "origin_x", "origin_y", "stamp_s"):
        if key not in meta or not isinstance(meta[key], (int, float)) or not math.isfinite(meta[key]):
            raise ValueError(f"Metadata {key} must be a finite number")
    if meta["resolution_m"] <= 0:
        raise ValueError("Metadata resolution_m must be positive")
    if not isinstance(meta.get("version"), int) or meta["version"] < 0:
        raise ValueError("Metadata version must be a nonnegative integer")
    meta.setdefault("frame_id", "odom")
    if not isinstance(meta["frame_id"], str) or not meta["frame_id"]:
        raise ValueError("Metadata frame_id must be nonempty")
    if meta.get("schema_version", SCHEMA_VERSION) != SCHEMA_VERSION:
        raise ValueError("Unsupported snapshot schema version")
    meta.update(schema_version=SCHEMA_VERSION, width=cost.shape[1], height=cost.shape[0],
                slope_unit="degree", height_unit="meter", unknown_cost=-1,
                blocked_cost=100, missing_slope="NaN", missing_height="NaN")
    return result, meta


def _fsync_directory(directory: Path):
    descriptor = os.open(directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write(destination: Path, writer: Callable):
    """Rename only a fully flushed file, keeping the preceding version on failure."""
    descriptor, temporary = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".tmp",
                                             dir=destination.parent)
    temporary = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
        _fsync_directory(destination.parent)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, value):
    content = (_json_text(value) + "\n").encode("utf-8")
    _atomic_write(path, lambda stream: stream.write(content))


def slope_rgba(values):
    """Shared 0..45 degree blue/cyan/yellow/red display; NaN is transparent."""
    values = np.asarray(values, dtype=float)
    stops = np.array([[0.08, 0.25, 0.85], [0.0, 0.85, 0.85],
                      [1.0, 0.9, 0.1], [0.85, 0.05, 0.05]])
    normalized = np.clip(np.nan_to_num(values, nan=0.0), 0, SLOPE_COLOR_MAX_DEG)
    rgba = np.ones((*values.shape, 4), dtype=np.float32)
    for axis in range(3):
        rgba[..., axis] = np.interp(normalized, [0., 15., 30., 45.], stops[:, axis])
    rgba[..., 3] = np.isfinite(values)
    return rgba


def _export_tables(directory, arrays, metadata):
    cost, slope, height = (arrays[name] for name in ("cost", "slope_deg", "height_m"))
    resolution = metadata["resolution_m"]
    with (directory / "cells.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["row", "column", "x_center_m", "y_center_m", "cost", "slope_deg", "height_m"])
        for row in range(cost.shape[0]):
            y = metadata["origin_y"] + (row + .5) * resolution
            for column in range(cost.shape[1]):
                writer.writerow([row, column, metadata["origin_x"] + (column + .5) * resolution,
                                 y, int(cost[row, column]), float(slope[row, column]), float(height[row, column])])
        stream.flush()
        os.fsync(stream.fileno())
    with (directory / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"])
        writer.writerows(arrays["trajectory"])
        stream.flush()
        os.fsync(stream.fileno())


def _export_plots(directory, arrays, metadata):
    # A headless canvas keeps shutdown export independent of a display server.
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / f"nomad_manual_mpl_{os.getuid()}"))
    import matplotlib
    matplotlib.use("Agg", force=True)
    from matplotlib import pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Patch

    shape = arrays["cost"].shape
    resolution = metadata["resolution_m"]
    origin_x, origin_y = metadata["origin_x"], metadata["origin_y"]
    extent = [origin_x, origin_x + shape[1] * resolution,
              origin_y, origin_y + shape[0] * resolution]
    cost_cmap = LinearSegmentedColormap.from_list("manual_cost", ["white", "#56c4b8", "#ffe45e", "#e84835", "black"])
    slope_cmap = LinearSegmentedColormap.from_list("manual_slope", slope_rgba(np.array([0., 15., 30., 45.])))
    for cmap in (cost_cmap, slope_cmap):
        cmap.set_bad("#a9afb8")
    trajectory = arrays["trajectory"]
    definitions = [("cost_map.png", "Accumulated slope cost", "Cost (100 = blocked)",
                    np.ma.masked_equal(arrays["cost"], -1), cost_cmap, 100., "unknown cost = -1"),
                   ("slope_map.png", "Accumulated slope angle", "Slope (°)",
                    np.ma.masked_invalid(arrays["slope_deg"]), slope_cmap, SLOPE_COLOR_MAX_DEG, "no slope = NaN")]
    for filename, title, label, values, cmap, maximum, unknown_label in definitions:
        figure, axis = plt.subplots(figsize=(10, 8), constrained_layout=True)
        try:
            image = axis.imshow(values, origin="lower", extent=extent, interpolation="nearest",
                                cmap=cmap, vmin=0., vmax=maximum, aspect="equal")
            bar = figure.colorbar(image, ax=axis, shrink=.85, extend="max" if maximum == SLOPE_COLOR_MAX_DEG else "neither")
            bar.set_label(label)
            handles = [Patch(facecolor="#a9afb8", label=unknown_label)]
            if len(trajectory):
                path, = axis.plot(trajectory[:, 1], trajectory[:, 2], color="#0958d9", linewidth=1.3,
                                  label="Actual vehicle trajectory")
                start = axis.scatter(trajectory[0, 1], trajectory[0, 2], c="#14ad45", s=35, label="Start", zorder=4)
                end = axis.scatter(trajectory[-1, 1], trajectory[-1, 2], c="#e92635", marker="^", s=45,
                                   label="Final vehicle position", zorder=4)
                handles.extend((path, start, end))
            axis.legend(handles=handles, loc="upper right", fontsize=8)
            axis.set_title(f"{title}\n{metadata.get('map_id', 'unknown map')} | version {metadata['version']}")
            axis.set_xlabel(f"{metadata['frame_id']} X (m)")
            axis.set_ylabel(f"{metadata['frame_id']} Y (m)")
            axis.set_aspect("equal")
            figure.savefig(directory / filename, dpi=160)
        finally:
            plt.close(figure)
        with (directory / filename).open("rb") as stream:
            os.fsync(stream.fileno())


def _export_numbered_svg(directory, arrays, metadata, *, cost_scores=False):
    """A complete vector map with a slope or geometry-cost label per observed cell."""
    cost, slope = arrays["cost"], arrays["slope_deg"]
    filename = "numbered_cost_map.svg" if cost_scores else "numbered_map.svg"
    label_class = "cost-label" if cost_scores else "slope-label"
    label_group = "numeric-costs" if cost_scores else "numeric-slopes"
    if cost_scores:
        title = "Accumulated geometry cost with numeric cost scores"
        description = ("Every observed cell is labeled with its stored geometry cost score, "
                       "including confidence adjustment, from 0 to 100. Gray cells are unknown. "
                       "Vector cells and text can be zoomed without raster resizing.")
        heading = "G cost scores (0..100)"
        text_legend = ("Cell text: geometry cost score (0..100), including confidence adjustment. "
                       "Blue = actual trajectory, green = start, red = final position.")
    else:
        title = "Accumulated G cost with numeric slope in degrees"
        description = ("Every observed cell is labeled with its slope and degree unit. "
                       "N/A means no valid fit. Gray cells are unknown. "
                       "Vector cells and text can be zoomed without raster resizing.")
        heading = "G cost and slope (°)"
        text_legend = ("Cell text: slope in °; N/A = no valid fit. "
                       "Blue = actual trajectory, green = start, red = final position.")
    rows_count, columns_count = cost.shape
    resolution = float(metadata["resolution_m"])
    cell = max(20., min(120., 900. / max(rows_count, columns_count)))
    map_width, map_height = columns_count * cell, rows_count * cell
    font = max(14., map_width / 80.)
    left, top = 4. * font, 3. * font
    width, height = max(map_width + 8. * font, 64. * font), map_height + 11. * font
    right, bottom = left + map_width, top + map_height
    observed_rows, observed_columns = np.where(cost >= 0)
    stops = np.array([[1., 1., 1.], [.337, .769, .722], [1., .894, .369],
                      [.91, .282, .208], [0., 0., 0.]])

    def color(value):
        rgb = np.array([np.interp(value, [0, 25, 50, 75, 100], stops[:, axis]) for axis in range(3)])
        return "#" + "".join(f"{round(float(channel) * 255):02x}" for channel in rgb), rgb

    def coordinate(x, y):
        return (left + (float(x) - metadata["origin_x"]) / resolution * cell,
                bottom - (float(y) - metadata["origin_y"]) / resolution * cell)

    palette = [color(value) for value in range(101)]
    text_colors = ["white" if float(rgb @ np.array([.2126, .7152, .0722])) < .4 else "black"
                   for _, rgb in palette]

    with (directory / filename).open("w", encoding="utf-8") as stream:
        stream.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        stream.write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.3f}" height="{height:.3f}" viewBox="0 0 {width:.3f} {height:.3f}">\n')
        stream.write(f'<title>{escape(title)}</title>\n')
        stream.write(f'<desc>{escape(description)}</desc>\n')
        stream.write(f'<rect width="{width:.3f}" height="{height:.3f}" fill="white"/>\n')
        stream.write(f'<g font-family="DejaVu Sans, sans-serif"><text x="{left:.3f}" y="{font * 1.5:.3f}" font-size="{font * 1.2:.3f}">{escape(str(metadata.get("map_id", "unknown map")))}: {heading}, version {metadata["version"]}</text>\n')
        stream.write(f'<rect x="{left:.3f}" y="{top:.3f}" width="{map_width:.3f}" height="{map_height:.3f}" fill="#a9afb8"/>\n')
        stream.write('<g id="observed-cells">\n')
        labels = []
        for row, column in zip(observed_rows, observed_columns):
            x, y = left + int(column) * cell, top + (rows_count - 1 - int(row)) * cell
            value_cost = int(cost[row, column])
            value = float(slope[row, column])
            label = str(value_cost) if cost_scores else (f"{value:.1f}°" if math.isfinite(value) else "N/A")
            fill, _ = palette[value_cost]
            foreground = text_colors[value_cost]
            stream.write(f'<rect x="{x:.3f}" y="{y:.3f}" width="{cell:.3f}" height="{cell:.3f}" fill="{fill}"/>\n')
            labels.append(f'<text class="{label_class}" data-row="{int(row)}" data-column="{int(column)}" x="{x + cell / 2:.3f}" y="{y + cell / 2:.3f}" font-size="{cell * .32:.3f}" text-anchor="middle" dominant-baseline="middle" fill="{foreground}">{label}</text>\n')
        stream.write('</g>\n')
        trajectory = arrays["trajectory"]
        if len(trajectory):
            points = [coordinate(row[1], row[2]) for row in trajectory]
            coordinates = " ".join(f"{x:.3f},{y:.3f}" for x, y in points)
            stream.write(f'<polyline id="trajectory" points="{coordinates}" fill="none" stroke="#0958d9" stroke-width="{max(1., cell * .035):.3f}" stroke-opacity=".8"/>\n')
            for name, (x, y), fill in (("start", points[0], "#14ad45"), ("final-position", points[-1], "#e92635")):
                stream.write(f'<circle id="{name}" cx="{x:.3f}" cy="{y:.3f}" r="{max(2., cell * .1):.3f}" fill="{fill}"/>\n')
        stream.write(f'<g id="{label_group}">\n' + ''.join(labels) + '</g>\n')
        stream.write(f'<rect x="{left:.3f}" y="{top:.3f}" width="{map_width:.3f}" height="{map_height:.3f}" fill="none" stroke="black" stroke-width="{font * .04:.3f}"/>\n')
        for fraction in np.linspace(0., 1., 6):
            x, y = left + fraction * map_width, bottom - fraction * map_height
            x_value = metadata["origin_x"] + fraction * columns_count * resolution
            y_value = metadata["origin_y"] + fraction * rows_count * resolution
            stream.write(f'<text x="{x:.3f}" y="{bottom + font * 1.3:.3f}" font-size="{font:.3f}" text-anchor="middle">{x_value:.2f}</text>\n')
            stream.write(f'<text x="{left - font * .7:.3f}" y="{y:.3f}" font-size="{font:.3f}" text-anchor="end" dominant-baseline="middle">{y_value:.2f}</text>\n')
        frame = escape(str(metadata["frame_id"]))
        stream.write(f'<text x="{left + map_width / 2:.3f}" y="{bottom + font * 2.7:.3f}" font-size="{font:.3f}" text-anchor="middle">{frame} X (m)</text>\n')
        stream.write(f'<text transform="translate({font:.3f},{top + map_height / 2:.3f}) rotate(-90)" font-size="{font:.3f}" text-anchor="middle">{frame} Y (m)</text>\n')
        legend_y = bottom + font * 4.
        stream.write(f'<text x="{left:.3f}" y="{legend_y:.3f}" font-size="{font:.3f}">Color: accumulated G cost; 100 = blocked; gray = unknown</text>\n')
        for index, value in enumerate((0, 25, 50, 75, 100)):
            x = left + index * 8. * font
            fill, _ = color(value)
            stream.write(f'<rect x="{x:.3f}" y="{legend_y + font * .4:.3f}" width="{font * 1.3:.3f}" height="{font * .9:.3f}" fill="{fill}" stroke="black" stroke-width="{font * .03:.3f}"/>\n')
            stream.write(f'<text x="{x + font * 1.8:.3f}" y="{legend_y + font * 1.1:.3f}" font-size="{font:.3f}">{value}</text>\n')
        stream.write(f'<text x="{left:.3f}" y="{legend_y + font * 2.4:.3f}" font-size="{font:.3f}">{escape(text_legend)}</text>\n')
        stream.write('</g></svg>\n')
        stream.flush()
        os.fsync(stream.fileno())


def _export_numbered_cost_svg(directory, arrays, metadata):
    """Export the stored geometry cost, with no angle unit or recalculation."""
    _export_numbered_svg(directory, arrays, metadata, cost_scores=True)


def _export(directory, arrays, metadata):
    _export_tables(directory, arrays, metadata)
    _export_plots(directory, arrays, metadata)
    _export_numbered_svg(directory, arrays, metadata)
    _export_numbered_cost_svg(directory, arrays, metadata)


def write_snapshot(session_dir: Path, arrays: dict[str, np.ndarray], metadata: dict,
                   final: bool = False) -> Path:
    """Commit one snapshot; mark closed only after all final exports are complete.

    A failed final export never replaces checkpoint.npz. SIGKILL/power failure
    can retain the preceding checkpoint instead of the last in-memory update.
    Callers serialize writes for a session and pass a consistent map version.
    """
    arrays, metadata = _validate(arrays, metadata)
    session_dir = Path(session_dir).expanduser().resolve()
    session_dir.mkdir(parents=True, exist_ok=True)
    destination = session_dir / ("map.npz" if final else "checkpoint.npz")
    metadata["status"] = "closed" if final else "running"
    metadata["snapshot_file"] = destination.name
    metadata["written_at"] = datetime.now(timezone.utc).isoformat()
    # Prepare all expensive exports first. No closed snapshot is committed if
    # plotting or CSV generation fails, and an earlier checkpoint stays intact.
    if final:
        with tempfile.TemporaryDirectory(prefix=".export-", dir=session_dir) as temporary:
            temporary = Path(temporary)
            _export(temporary, arrays, metadata)
            for name in EXPORT_FILES:
                os.replace(temporary / name, session_dir / name)
            _fsync_directory(session_dir)
    encoded = np.array(_json_text(metadata))
    _atomic_write(destination, lambda stream: np.savez_compressed(stream, **arrays, **{METADATA_KEY: encoded}))
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    manifest = dict(metadata)
    manifest.update(snapshot_sha256=digest, exports=list(EXPORT_FILES) if final else [],
                    trajectory_rows=len(arrays["trajectory"]), known_cost_cells=int(np.count_nonzero(arrays["cost"] >= 0)),
                    finite_slope_cells=int(np.count_nonzero(np.isfinite(arrays["slope_deg"]))))
    _atomic_json(session_dir / "manifest.json", manifest)
    return destination


def load_snapshot(path: Path) -> tuple[dict[str, np.ndarray], dict]:
    """Load without pickle and validate the complete, embedded coordinate contract."""
    with np.load(Path(path), allow_pickle=False) as stored:
        if METADATA_KEY not in stored.files:
            raise ValueError("Snapshot has no embedded metadata")
        raw_metadata = stored[METADATA_KEY]
        if raw_metadata.ndim != 0 or raw_metadata.dtype.kind not in "US":
            raise ValueError("Invalid embedded snapshot metadata")
        metadata = json.loads(str(raw_metadata.item()))
        arrays = {name: stored[name].copy() for name in stored.files if name != METADATA_KEY}
    return _validate(arrays, metadata)


def resolve_snapshot(path: Path) -> Path:
    """Resolve a session's committed snapshot, falling back to valid local files."""
    path = Path(path).expanduser().resolve()
    if not path.is_dir():
        load_snapshot(path)
        return path
    candidates = []
    try:
        manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
        name = manifest.get("snapshot_file")
        if isinstance(name, str) and Path(name).name == name and name in ("map.npz", "checkpoint.npz"):
            candidate = path / name
            load_snapshot(candidate)
            return candidate
    except (OSError, ValueError, TypeError, AttributeError, EOFError, BadZipFile):
        pass
    for name in ("map.npz", "checkpoint.npz"):
        candidate = path / name
        try:
            _, metadata = load_snapshot(candidate)
            candidates.append((metadata["version"], candidate.stat().st_mtime_ns, candidate))
        except (OSError, ValueError, TypeError, EOFError, BadZipFile):
            continue
    if not candidates:
        raise ValueError(f"No complete, valid map snapshot in {path}")
    return max(candidates, key=lambda entry: entry[:2])[2]
