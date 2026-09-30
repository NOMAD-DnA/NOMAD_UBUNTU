#!/usr/bin/env python3
"""Measure stationary r1 height/pitch at four authored hill arc positions.

This is a terrain/posture check, NOT a motor, climbing, or route-completion test.
Only --exercise permits set_pose writes. ROS domain 43 and an exact forest world
are mandatory. Reuse the forest check's guards, bounded RPCs, stop and recorder.
PNG bilinear heights are independent authoring estimates; MVSim uses triangle
interpolation and four wheel contacts, so tolerances allow a small difference.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import signal

from check_forest_live import (
    PACKAGE, _Live, _angle_difference, _binding_paths, _pose_from_odom, _rpc, _start_pose,
)


SAMPLES = (("ascent", 8.0), ("crest", 13.5), ("descent", 20.5), ("flat", 35.0))


class _HeightMap:
    def __init__(self, filename: Path, layout: dict):
        from PIL import Image
        with Image.open(filename) as image:
            if image.mode != "L":
                raise ValueError("Expected the generator's 8-bit grayscale height PNG")
            self.columns, self.rows = image.size
            self.pixels = list(image.getdata())
        self.pixel_min, self.pixel_max = min(self.pixels), max(self.pixels)
        if self.pixel_min == self.pixel_max:
            raise ValueError("Height PNG is flat: no hill to verify")
        self.bounds = layout["bounds"]
        self.resolution = float(layout["height_resolution_m"])
        self.max_z = float(layout["height_max_m"])
        expected = (round((self.bounds["y_max"]-self.bounds["y_min"])/self.resolution)+1,
                    round((self.bounds["x_max"]-self.bounds["x_min"])/self.resolution)+1)
        if (self.columns, self.rows) != expected:
            raise ValueError("Height PNG dimensions disagree with metadata +X rows / +Y columns")

    def height(self, x: float, y: float) -> float:
        row = (x-self.bounds["x_min"])/self.resolution
        column = (y-self.bounds["y_min"])/self.resolution
        if not 0 <= row <= self.rows-1 or not 0 <= column <= self.columns-1:
            raise ValueError("Sample outside the authored height PNG")
        r0, c0 = math.floor(row), math.floor(column)
        r1, c1 = min(r0+1, self.rows-1), min(c0+1, self.columns-1)
        fr, fc = row-r0, column-c0
        pixel = sum(self.pixels[r*self.columns+c] * weight for r, c, weight in (
            (r0, c0, (1-fr)*(1-fc)), (r0, c1, (1-fr)*fc),
            (r1, c0, fr*(1-fc)), (r1, c1, fr*fc)))
        # Match ElevationMap's actual-image-extrema scaling, not just /255.
        return (pixel-self.pixel_min) * self.max_z/(self.pixel_max-self.pixel_min)


def _arc(path: list) -> list[float]:
    distances = [0.0]
    for a, b in zip(path, path[1:]):
        distances.append(distances[-1] + math.dist(a, b))
    if len(path) < 2 or distances[-1] < SAMPLES[-1][1]+.3:
        raise ValueError("uphill path is too short for the requested four samples")
    return distances


def _point(path: list, arc: list[float], distance: float) -> tuple[float, float]:
    if not 0 <= distance <= arc[-1]:
        raise ValueError("Arc sample outside uphill path")
    i = min(bisect.bisect_right(arc, distance)-1, len(path)-2)
    span = arc[i+1]-arc[i]
    if span <= 0:
        raise ValueError("Degenerate uphill path segment")
    fraction = (distance-arc[i])/span
    return tuple(path[i][axis] + fraction*(path[i+1][axis]-path[i][axis]) for axis in (0, 1))


def _samples(layout: dict, heightmap: _HeightMap) -> list[dict]:
    path = layout["paths"]["uphill"]
    arc = _arc(path)
    samples = []
    for name, distance in SAMPLES:
        x, y = _point(path, arc, distance)
        behind, ahead = _point(path, arc, distance-.3), _point(path, arc, distance+.3)
        yaw = math.atan2(ahead[1]-behind[1], ahead[0]-behind[0])
        slope = math.atan2(heightmap.height(*ahead)-heightmap.height(*behind), math.dist(ahead, behind))
        samples.append(dict(name=name, arc_m=distance, expected_slope_deg=math.degrees(slope),
                            pose=dict(x=x, y=y, z=heightmap.height(x, y), yaw=yaw, pitch=0.0, roll=0.0)))
    return samples


def _confirm_stopped_identity(live: _Live, paths: list[str]) -> dict:
    """Reuse graph, RPC and pose parsing; only the comparison is local here."""
    live.guard_graph(motion=True)
    gt = live.latest["/base_pose_ground_truth"]
    twist = gt.twist.twist
    velocities = (twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)
    if not all(math.isfinite(v) and abs(v) <= .05 for v in velocities):
        raise RuntimeError("Safety gate: r1 must be stopped before hill teleports")
    pose, remote = _pose_from_odom(gt), _rpc("get", paths)["pose"]
    if (not all(math.isfinite(v) for v in remote.values())
            or math.dist([pose[k] for k in ("x", "y", "z")], [remote[k] for k in ("x", "y", "z")]) > .10
            or any(abs(_angle_difference(remote[k], pose[k])) > .10 for k in ("yaw", "pitch", "roll"))):
        raise RuntimeError("Safety gate: ZMQ r1 does not match this ROS world's ground truth")
    return pose


def _measure(live: _Live, sample: dict, paths: list[str]) -> dict:
    answer = live.place(paths, sample["pose"])
    if answer["collision"]:
        raise RuntimeError(f"{sample['name']} sample contacts a non-terrain obstacle")
    count = live.counts["/base_pose_ground_truth"]
    live.wait(lambda: live.counts["/base_pose_ground_truth"] >= count+3,
              4.0, "three settled hill ground-truth observations")
    live.stop()
    pose = _pose_from_odom(live.latest["/base_pose_ground_truth"])
    if not all(math.isfinite(v) for v in pose.values()):
        raise RuntimeError("Non-finite hill ground truth")
    return dict(**sample, measured_pose=pose, measured_pitch_deg=math.degrees(pose["pitch"]),
                measured_roll_deg=math.degrees(pose["roll"]),
                height_error_m=pose["z"]-sample["pose"]["z"])


def _hill_checks(samples: list[dict]) -> dict:
    ascent, crest, descent, flat = samples
    up, down = ascent["measured_pitch_deg"], descent["measured_pitch_deg"]
    return dict(
        # Do not presume ROS/MVSim pitch sign: opposite signs are the invariant.
        ascent_pitch=abs(up) >= 10 and abs(abs(up)-abs(ascent["expected_slope_deg"])) < 7,
        descent_pitch=abs(down) >= 8 and abs(abs(down)-abs(descent["expected_slope_deg"])) < 7,
        opposing_pitch_signs=up*down < 0,
        heights_match_png=all(abs(sample["height_error_m"]) < .20 for sample in samples),
        crest_above_flat=crest["measured_pose"]["z"]-flat["measured_pose"]["z"] > 1.5,
        crest_level=abs(crest["measured_pitch_deg"]) < 5,
        flat_level=abs(flat["measured_pitch_deg"]) < 5 and abs(flat["measured_pose"]["z"]) < .15,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exercise", action="store_true", help="ALLOW stationary r1 pose writes; no driving")
    parser.add_argument("--expected-world", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--layout", type=Path, default=PACKAGE/"assets/forest/layout.json")
    parser.add_argument("--height-map", type=Path)
    parser.add_argument("--mvsim-workspace", type=Path, default=Path("/home/user/nomad_ws"))
    args = parser.parse_args()
    import os
    if os.environ.get("ROS_DOMAIN_ID") != "43":
        parser.error("ROS_DOMAIN_ID must already be exactly 43")
    layout = json.loads(args.layout.read_text(encoding="utf-8"))
    height_file = args.height_map or args.layout.parent/"height.png"
    samples = _samples(layout, _HeightMap(height_file, layout))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = dict(domain=43, exercise=args.exercise, hill_verified=False, passed=False,
                  height_png_sha256=hashlib.sha256(height_file.read_bytes()).hexdigest(),
                  authored_samples=samples, measurements=[], checks={}, snapshots={},
                  restoration={"attempted": False}, scope="stationary terrain posture, not motor/climbing ability")
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node("nomad_hill_live_check", namespace="/", use_global_arguments=False)
    live, armed = _Live(node, rclpy), False
    paths, start_pose = _binding_paths(args.mvsim_workspace), _start_pose(layout)

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")

    previous_sigterm = signal.signal(signal.SIGTERM, interrupted)
    try:
        live.wait(live.exact_node, 8.0, "exactly /mvsim in domain 43")
        live.guard_graph()
        report["world_file"] = live.world_file(args.expected_world)
        report["sensors"] = live.sensors()
        if not report["sensors"]["passed"]:
            raise RuntimeError("Live sensor validation failed before hill sampling")
        report["snapshots"]["initial_rgb"] = live.record(args.output_dir, "initial_rgb")
        if args.exercise:
            report["initial_pose"] = _confirm_stopped_identity(live, paths)
            live.enable_motion()
            armed = True
            for sample in samples:
                live.guard_graph(motion=True)
                report["measurements"].append(_measure(live, sample, paths))
                report["snapshots"][sample["name"]] = live.record(args.output_dir, sample["name"]+"_rgb")
            report["checks"] = _hill_checks(report["measurements"])
            report["hill_verified"] = all(report["checks"].values())
            if not report["hill_verified"]:
                raise RuntimeError("Hill posture verification failed; see signed pitch and height measurements")
        else:
            report["skipped"] = "No --exercise: no pose or cmd_vel writes; hill posture NOT verified"
        report["passed"] = True
    except (Exception, KeyboardInterrupt) as error:
        report.update(passed=False, error=f"{type(error).__name__}: {error}")
    finally:
        if armed:
            report["restoration"]["attempted"] = True
            try:
                live.stop()
                answer = live.place(paths, start_pose)
                live.stop()
                restored = _pose_from_odom(live.latest["/base_pose_ground_truth"])
                ok = math.dist([restored[k] for k in ("x", "y")], [start_pose[k] for k in ("x", "y")]) < .15
                ok = ok and not answer["collision"]
                report["restoration"].update(passed=ok, pose=restored)
                report["snapshots"]["restored_rgb"] = live.record(args.output_dir, "restored_rgb")
                if not ok:
                    report["passed"] = False
            except (Exception, KeyboardInterrupt) as error:
                report["restoration"].update(passed=False, error=f"{type(error).__name__}: {error}")
                report["passed"] = False
            finally:
                try:
                    live.stop()
                except (Exception, KeyboardInterrupt) as error:
                    report["restoration"]["final_stop_error"] = str(error)
                    report["passed"] = False
        signal.signal(signal.SIGTERM, previous_sigterm)
        node.destroy_node()
        rclpy.shutdown()
        destination = args.output_dir/"hill_live_report.json"
        destination.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        print(f"Report: {destination}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
