#!/usr/bin/env python3
"""Bounded live sensor / collision checks for the isolated NOMAD forest world.

Default mode only subscribes, reads world_file, and saves RGB snapshots.
--exercise additionally commands /cmd_vel and teleports ONLY r1 through MVSim's
ZMQ set_pose service. It is NOT a route planner or an autonomous driving demo.

Safety gates: ROS_DOMAIN_ID must already equal 43; exactly /mvsim must own the
ground-truth and cmd_vel endpoints; its world must be forest.world.xml; no other
cmd_vel publisher may be active. ZMQ is NOT isolated by ROS_DOMAIN_ID, so its r1
pose must match this ROS world's ground truth before any ZMQ write is permitted.
Every RPC and phase is time-bounded. Exercise cleanup repeatedly sends zero
velocity and attempts to restore the metadata start pose, including on Ctrl-C.
SIGKILL, server failure, or a failed restoration cannot be made safe by Python:
those failures are explicitly reported, never treated as a passing test.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import zlib


PACKAGE = Path(__file__).resolve().parents[1]
RPC_MARKER = "__NOMAD_RPC__"

# A separate process bounds C++/ZMQ calls even if receiveMessage() blocks.
# Only get_pose(r1) and set_pose(r1) are exposed; no CLI bypass for motion exists.
RPC_PROGRAM = r'''
import json, os, sys
if os.environ.get("ROS_DOMAIN_ID") != "43":
    raise RuntimeError("ZMQ worker refuses any ROS_DOMAIN_ID except 43")
request = json.loads(sys.stdin.read())
sys.path[:0] = request["binding_paths"]
from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvGetPose_pb2, SrvGetPoseAnswer_pb2
from mvsim_msgs import SrvSetPose_pb2, SrvSetPoseAnswer_pb2
client = pymvsim_comms.mvsim.Client()
client.setName("nomad_forest_live_rpc")
client.connect()
if request["action"] == "get":
    req = SrvGetPose_pb2.SrvGetPose(objectId="r1")
    ans = SrvGetPoseAnswer_pb2.SrvGetPoseAnswer()
    ans.ParseFromString(client.callService("get_pose", req.SerializeToString()))
elif request["action"] == "set":
    req = SrvSetPose_pb2.SrvSetPose(objectId="r1")
    for key in ("x", "y", "z", "yaw", "pitch", "roll"):
        setattr(req.pose, key, request["pose"][key])
    ans = SrvSetPoseAnswer_pb2.SrvSetPoseAnswer()
    ans.ParseFromString(client.callService("set_pose", req.SerializeToString()))
else:
    raise RuntimeError("Unknown RPC action")
if not ans.success:
    raise RuntimeError(ans.errorMessage)
result = {"success": True, "collision": bool(ans.objectIsInCollision)}
if request["action"] == "get":
    result["pose"] = {key: getattr(ans.pose, key) for key in ("x", "y", "z", "yaw", "pitch", "roll")}
print("__NOMAD_RPC__" + json.dumps(result), flush=True)
client.shutdown()
'''


def _stamp(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def _yaw(quaternion) -> float:
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y**2 + quaternion.z**2))


def _angle_difference(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _pose_from_odom(message) -> dict:
    position = message.pose.pose.position
    q = message.pose.pose.orientation
    return dict(x=position.x, y=position.y, z=position.z, yaw=_yaw(q),
                pitch=math.asin(max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x)))),
                roll=math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y)))


def _cross(a, b) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _ray_entry(start, direction, polygon) -> float | None:
    """Entry of a forward ray into a convex counterclockwise XY footprint."""
    lower, upper = 0.0, math.inf
    for i, a in enumerate(polygon):
        b = polygon[(i + 1) % len(polygon)]
        edge = (b[0] - a[0], b[1] - a[1])
        value = _cross(edge, (start[0] - a[0], start[1] - a[1]))
        rate = _cross(edge, direction)
        if abs(rate) < 1e-10:
            if value < -1e-9:
                return None
        elif rate > 0:
            lower = max(lower, -value / rate)
        else:
            upper = min(upper, -value / rate)
    return lower if upper >= lower and upper >= 0 else None


def _rectangle(center, yaw_deg, length, width):
    angle = math.radians(yaw_deg)
    c, s = math.cos(angle), math.sin(angle)
    return [(center[0] + c*x - s*y, center[1] + s*x + c*y)
            for x, y in ((-length/2, -width/2), (length/2, -width/2),
                         (length/2, width/2), (-length/2, width/2))]


def _test_cases(layout: dict) -> list[dict]:
    cases = []
    for name in ("rocks", "wall"):
        before = layout["blocked_probes"][name + "_before"]
        after = layout["blocked_probes"][name + "_after"]
        dx, dy = after[0] - before[0], after[1] - before[1]
        length = math.hypot(dx, dy)
        if not math.isfinite(length) or length < 0.5:
            raise ValueError(f"Invalid {name} probe direction")
        direction = (dx / length, dy / length)
        if name == "wall":
            wall = layout["wall"]
            polygons = [_rectangle(wall["center"], wall["yaw_deg"],
                                   wall["length_m"], wall["width_m"])]
        else:
            polygons = [[
                (rock["center"][0] + rock["radius_m"] * math.cos(math.radians(rock["yaw_deg"]) + math.tau*i/8),
                 rock["center"][1] + rock["radius_m"] * math.sin(math.radians(rock["yaw_deg"]) + math.tau*i/8))
                for i in range(8)] for rock in layout["rocks"]]
        entries = [_ray_entry(before, direction, polygon) for polygon in polygons]
        entries = [entry for entry in entries if entry is not None]
        if not entries or not 0.5 < min(entries) < 4.0:
            raise ValueError(f"{name} probe must face a nearby, initially clear obstacle")
        cases.append(dict(name=name, before=before, direction=direction,
                          yaw=math.atan2(dy, dx), obstacle_entry_m=min(entries)))
    return cases


def _start_pose(layout: dict) -> dict:
    start = layout["waypoints"]["start"]
    a, b = layout["paths"]["entrance"][:2]
    return dict(x=start[0], y=start[1], z=0.0,
                yaw=math.atan2(b[1] - a[1], b[0] - a[0]), pitch=0.0, roll=0.0)


def _binding_paths(workspace: Path) -> list[str]:
    candidates = [workspace / "build/mvsim"]
    for pattern in ("install/mvsim/local/lib/python*/dist-packages",
                    "install/mvsim/lib/python*/site-packages",
                    "build/mvsim/venv/lib/python*/site-packages"):
        candidates.extend(sorted(workspace.glob(pattern)))
    return [str(path.resolve()) for path in candidates if path.is_dir()]


def _rpc(action: str, paths: list[str], pose: dict | None = None) -> dict:
    request = dict(action=action, binding_paths=paths)
    if pose is not None:
        if not all(math.isfinite(pose[key]) for key in ("x", "y", "z", "yaw", "pitch", "roll")):
            raise ValueError("Refusing non-finite teleport pose")
        request["pose"] = pose
    try:
        completed = subprocess.run([sys.executable, "-B", "-c", RPC_PROGRAM],
                                   input=json.dumps(request), text=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=4.0)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f"MVSim {action}_pose RPC exceeded 4 seconds") from error
    if completed.returncode:
        raise RuntimeError(f"MVSim {action}_pose unavailable: {completed.stderr[-1500:].strip()}")
    for line in reversed(completed.stdout.splitlines()):
        if line.startswith(RPC_MARKER):
            return json.loads(line[len(RPC_MARKER):])
    raise RuntimeError(f"MVSim {action}_pose returned no verified answer")


def _save_rgb_png(image, destination: Path) -> None:
    """Save the subscribed colour image with stdlib only; honour row padding."""
    if image.encoding not in ("rgb8", "bgr8") or image.step < image.width * 3:
        raise ValueError(f"Cannot record RGB encoding {image.encoding!r}")
    data = bytes(image.data)
    if len(data) < image.step * image.height:
        raise ValueError("Truncated RGB image")
    rows = bytearray()
    for y in range(image.height):
        row = data[y*image.step:y*image.step + image.width*3]
        if image.encoding == "bgr8":
            converted = bytearray(len(row))
            converted[0::3], converted[1::3], converted[2::3] = row[2::3], row[1::3], row[0::3]
            row = converted
        rows.extend(b"\x00")
        rows.extend(row)

    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff)

    png = (b"\x89PNG\r\n\x1a\n" +
           chunk(b"IHDR", struct.pack(">IIBBBBB", image.width, image.height, 8, 2, 0, 0, 0)) +
           chunk(b"IDAT", zlib.compress(rows, 6)) + chunk(b"IEND", b""))
    destination.write_bytes(png)


class _Live:
    def __init__(self, node, rclpy_module):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, LaserScan
        from std_msgs.msg import Bool
        self.node, self.rclpy, self.Twist = node, rclpy_module, Twist
        self.latest, self.counts, self.received, self.stamps = {}, {}, {}, {}
        self.collision_count = 0
        self.publisher = None
        self.subscriptions = []
        for kind, topic in ((Image, "/oak/rgb/image_raw"),
                            (CameraInfo, "/oak/rgb/camera_info"),
                            (Image, "/forest_overview/image_raw"),
                            (LaserScan, "/scan"),
                            (Odometry, "/base_pose_ground_truth"),
                            (Bool, "/collision")):
            def callback(message, topic=topic):
                self.latest[topic] = message
                self.counts[topic] = self.counts.get(topic, 0) + 1
                self.received[topic] = time.monotonic()
                if hasattr(message, "header"):
                    self.stamps.setdefault(topic, []).append(_stamp(message))
                    self.stamps[topic] = self.stamps[topic][-1000:]
                if topic == "/collision" and message.data:
                    self.collision_count += 1
            self.subscriptions.append(node.create_subscription(kind, topic, callback, qos_profile_sensor_data))

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))

    def wait(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.spin(0.03)
        raise RuntimeError(f"Timed out after {timeout:g}s: {description}")

    def exact_node(self) -> bool:
        nodes = self.node.get_node_names_and_namespaces()
        return nodes.count(("mvsim", "/")) == 1 and sum(name == "mvsim" for name, _ in nodes) == 1

    def guard_graph(self, motion: bool = False) -> None:
        if os.environ.get("ROS_DOMAIN_ID") != "43" or not self.exact_node():
            raise RuntimeError("Safety gate: expected exactly /mvsim in ROS_DOMAIN_ID=43")
        if self.node.get_node_names_and_namespaces().count((self.node.get_name(), "/")) != 1:
            raise RuntimeError("Safety gate: another forest-check node uses this node name")
        publishers = self.node.get_publishers_info_by_topic("/base_pose_ground_truth")
        if len(publishers) != 1 or (publishers[0].node_name, publishers[0].node_namespace) != ("mvsim", "/"):
            raise RuntimeError("Safety gate: ground truth must have exactly one /mvsim publisher")
        if motion:
            subscribers = self.node.get_subscriptions_info_by_topic("/cmd_vel")
            if len(subscribers) != 1 or (subscribers[0].node_name, subscribers[0].node_namespace) != ("mvsim", "/"):
                raise RuntimeError("Safety gate: /cmd_vel must target only /mvsim")
            publishers = self.node.get_publishers_info_by_topic("/cmd_vel")
            if any((info.node_name, info.node_namespace) != (self.node.get_name(), "/") for info in publishers):
                raise RuntimeError("Safety gate: another /cmd_vel publisher is active")

    def world_file(self, expected: Path | None) -> str:
        from rclpy.parameter_client import AsyncParameterClient
        client = AsyncParameterClient(self.node, "/mvsim")
        self.wait(client.services_are_ready, 4.0, "/mvsim parameter service")
        future = client.get_parameters(["world_file"])
        self.wait(future.done, 4.0, "/mvsim world_file parameter")
        response = future.result()
        filename = response.values[0].string_value if response and response.values else ""
        if Path(filename).name != "forest.world.xml":
            raise RuntimeError(f"Safety gate: not the forest world: {filename!r}")
        if expected is not None and Path(filename).resolve() != expected.resolve():
            raise RuntimeError(f"Safety gate: world_file does not match --expected-world: {filename}")
        return filename

    def sensors(self) -> dict:
        self.wait(lambda: all(self.counts.get(t, 0) >= 3 for t in
                              ("/oak/rgb/image_raw", "/oak/rgb/camera_info", "/scan", "/base_pose_ground_truth")),
                  12.0, "RGB, CameraInfo, 500-ray scan and ground truth")
        image = self.latest["/oak/rgb/image_raw"]
        info = self.latest["/oak/rgb/camera_info"]
        scan = self.latest["/scan"]
        gt = self.latest["/base_pose_ground_truth"]
        data = bytes(image.data)
        sample = data[::max(1, len(data)//10000)]
        pose = _pose_from_odom(gt)
        q = gt.pose.pose.orientation
        checks = dict(
            camera=image.width == 640 and image.height == 480 and image.encoding in ("bgr8", "rgb8")
                   and image.step >= image.width*3 and len(data) >= image.step*image.height
                   and image.header.frame_id == "oak_rgb_optical_frame",
            camera_nonblank=bool(sample) and max(sample) - min(sample) > 8,
            camera_info=info.width == image.width and info.height == image.height
                        and all(math.isfinite(x) and x > 0 for x in (info.k[0], info.k[4]))
                        and all(abs(info.r[i]-1.0) < 1e-6 for i in (0, 4, 8)),
            camera_updates=len(set(self.stamps["/oak/rgb/image_raw"])) >= 3,
            scan=len(scan.ranges) == 500 and abs(scan.range_min-.12) < 1e-6
                 and abs(scan.range_max-12.0) < 1e-6
                 and abs(scan.angle_max-scan.angle_min-math.tau) < 1e-5,
            scan_timing=abs(scan.scan_time-.1) < 1e-6 and scan.time_increment == 0.0,
            scan_values=all(not math.isfinite(x) or scan.range_min <= x < scan.range_max for x in scan.ranges),
            scan_has_returns=any(math.isfinite(x) and scan.range_min <= x < scan.range_max for x in scan.ranges),
            scan_updates=len(set(self.stamps["/scan"])) >= 3,
            ground_truth=all(math.isfinite(x) for x in pose.values())
                         and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < 1e-3,
        )
        return dict(checks=checks, passed=all(checks.values()),
                    measurements=dict(rgb_size=[image.width, image.height], encoding=image.encoding,
                                      nrays=len(scan.ranges), finite_returns=sum(math.isfinite(x) for x in scan.ranges),
                                      scan_time_s=scan.scan_time, pose=pose, counts=dict(self.counts)))

    def record(self, output: Path, name: str, overview: bool = False) -> str | None:
        topic = "/forest_overview/image_raw" if overview else "/oak/rgb/image_raw"
        image = self.latest.get(topic)
        if image is None:
            return None
        destination = output / (name + ".png")
        _save_rgb_png(image, destination)
        return str(destination)

    def enable_motion(self) -> None:
        self.guard_graph(motion=True)
        self.publisher = self.node.create_publisher(self.Twist, "/cmd_vel", 10)
        self.spin(0.3)
        self.guard_graph(motion=True)

    def command(self, speed: float) -> None:
        if self.publisher is None:
            raise RuntimeError("Motion publisher was not armed")
        command = self.Twist()
        command.linear.x = speed
        self.publisher.publish(command)

    def stop(self) -> None:
        if self.publisher is not None:
            for _ in range(8):
                self.command(0.0)
                self.spin(0.06)

    def place(self, paths: list[str], pose: dict) -> dict:
        self.stop()
        self.guard_graph(motion=True)
        count = self.counts.get("/base_pose_ground_truth", 0)
        answer = _rpc("set", paths, pose)
        self.wait(lambda: self.counts.get("/base_pose_ground_truth", 0) > count
                  and math.hypot(self.latest["/base_pose_ground_truth"].pose.pose.position.x - pose["x"],
                                 self.latest["/base_pose_ground_truth"].pose.pose.position.y - pose["y"]) < .15
                  and abs(_angle_difference(_yaw(self.latest["/base_pose_ground_truth"].pose.pose.orientation), pose["yaw"])) < .1,
                  8.0, "teleported r1 pose reflected in ROS ground truth")
        pose_stamp = _stamp(self.latest["/base_pose_ground_truth"])
        # MVSim 1.4 set_pose answers with the PREVIOUS collision flag and then
        # resets it. Do not mistake that answer for contact at the new pose.
        collision_count = self.counts.get("/collision", 0)
        self.wait(lambda: self.counts.get("/collision", 0) >= collision_count + 2,
                  3.0, "fresh collision state after teleport")
        answer["previous_collision"] = answer["collision"]
        answer["collision"] = bool(self.latest["/collision"].data)
        self.wait(lambda: _stamp(self.latest["/oak/rgb/image_raw"]) >= pose_stamp,
                  4.0, "fresh RGB snapshot after teleport")
        self.spin(0.15)
        return answer

    def drive(self, case: dict, sim_seconds: float, speed: float) -> dict:
        self.guard_graph(motion=True)
        start_stamp = _stamp(self.latest["/base_pose_ground_truth"])
        deadline = time.monotonic() + 20.0
        initial_collisions = self.collision_count
        trace = []
        next_command, next_guard, last_stamp = 0.0, 0.0, None
        timed_out = False
        try:
            while True:
                now = time.monotonic()
                if now >= deadline:
                    timed_out = True
                    break
                if now - self.received.get("/base_pose_ground_truth", 0) > 3.0:
                    raise RuntimeError("Ground truth became stale; stopping")
                if now >= next_guard:
                    self.guard_graph(motion=True)
                    next_guard = now + 1.0
                gt = self.latest["/base_pose_ground_truth"]
                elapsed = _stamp(gt) - start_stamp
                if elapsed < -0.01:
                    raise RuntimeError("Simulation clock moved backwards; stopping")
                if _stamp(gt) != last_stamp:
                    p = gt.pose.pose.position
                    if not all(math.isfinite(value) for value in (elapsed, p.x, p.y, p.z)):
                        raise RuntimeError("Non-finite ground-truth pose; stopping")
                    dx, dy = p.x-case["before"][0], p.y-case["before"][1]
                    along = dx*case["direction"][0] + dy*case["direction"][1]
                    across = -dx*case["direction"][1] + dy*case["direction"][0]
                    trace.append(dict(sim_s=elapsed, x=p.x, y=p.y, z=p.z, along_m=along, lateral_m=across))
                    last_stamp = _stamp(gt)
                if elapsed >= sim_seconds:
                    break
                if now >= next_command:
                    self.command(speed)
                    next_command = now + .05
                self.spin(.02)
        finally:
            self.stop()
        if not trace:
            raise RuntimeError("No ground-truth trace during exercise")
        progress = max(p["along_m"] for p in trace)
        tail = [p["along_m"] for p in trace if p["sim_s"] >= trace[-1]["sim_s"]-1.0]
        tail_motion = max(tail)-min(tail)
        collision_seen = self.collision_count > initial_collisions
        checks = dict(simulation_duration=not timed_out and trace[-1]["sim_s"] >= sim_seconds,
                      actually_moved=progress >= .20,
                      collision_observed=collision_seen,
                      did_not_enter_obstacle=progress < case["obstacle_entry_m"] + .10,
                      settled_at_obstacle=tail_motion < .12,
                      stayed_on_probe_line=max(abs(p["lateral_m"]) for p in trace) < .45)
        return dict(checks=checks, passed=all(checks.values()), speed_command_m_s=speed,
                    desired_sim_duration_s=sim_seconds, actual_sim_duration_s=trace[-1]["sim_s"],
                    timed_out_20s_wall_clock=timed_out, obstacle_entry_m=case["obstacle_entry_m"],
                    max_progress_m=progress, last_second_motion_m=tail_motion,
                    collision_messages=self.collision_count-initial_collisions, trace=trace)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exercise", action="store_true", help="ALLOW isolated r1 motion and teleports")
    parser.add_argument("--layout", type=Path, default=PACKAGE / "assets/forest/layout.json")
    parser.add_argument("--output-dir", type=Path, default=PACKAGE / "assets/forest/runtime")
    parser.add_argument("--expected-world", type=Path, help="optional exact world_file path safety gate")
    parser.add_argument("--mvsim-workspace", type=Path, default=Path("/home/user/nomad_ws"))
    parser.add_argument("--drive-seconds", type=float, default=6.0, help="SIMULATION seconds, restricted to 5..8")
    parser.add_argument("--speed", type=float, default=.55, help="forward command m/s, restricted to 0.4..0.8")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "43":
        parser.error("ROS_DOMAIN_ID must already be exactly 43; this script will not select another domain")
    if not 5 <= args.drive_seconds <= 8 or not .4 <= args.speed <= .8:
        parser.error("--drive-seconds must be 5..8 and --speed must be 0.4..0.8")
    layout = json.loads(args.layout.read_text(encoding="utf-8"))
    cases, start_pose = _test_cases(layout), _start_pose(layout)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node("nomad_forest_live_check", namespace="/", use_global_arguments=False)
    live = _Live(node, rclpy)
    report = dict(domain=43, exercise=args.exercise, checks={}, sensors={}, collision_tests={},
                  snapshots={}, restoration={"attempted": False}, passed=False)
    armed = False
    paths = _binding_paths(args.mvsim_workspace)

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")

    previous_sigterm = signal.signal(signal.SIGTERM, interrupted)
    try:
        live.wait(live.exact_node, 8.0, "exactly /mvsim in domain 43")
        live.guard_graph()
        report["world_file"] = live.world_file(args.expected_world)
        report["checks"]["loaded_forest_world"] = True
        report["sensors"] = live.sensors()
        if not report["sensors"]["passed"]:
            raise RuntimeError("Live camera / scan / ground-truth validation failed")
        report["snapshots"]["initial_rgb"] = live.record(args.output_dir, "initial_rgb")
        # A root-added overview camera is optional; production world needs none.
        if "/forest_overview/image_raw" in live.latest:
            report["snapshots"]["overview"] = live.record(args.output_dir, "overview", overview=True)
        if args.exercise:
            live.guard_graph(motion=True)
            gt = live.latest["/base_pose_ground_truth"]
            velocity = gt.twist.twist
            if math.hypot(velocity.linear.x, velocity.linear.y) > .05 or abs(velocity.angular.z) > .05:
                raise RuntimeError("Safety gate: r1 must be stopped before exercise")
            answer = _rpc("get", paths)
            pose = _pose_from_odom(gt)
            remote = answer["pose"]
            if (not all(math.isfinite(value) for value in remote.values())
                    or math.hypot(remote["x"]-pose["x"], remote["y"]-pose["y"]) > .10
                    or abs(remote["z"]-pose["z"]) > .10
                    or any(abs(_angle_difference(remote[key], pose[key])) > .10
                           for key in ("yaw", "pitch", "roll"))):
                raise RuntimeError("Safety gate: ZMQ r1 is not this ROS world's r1")
            report["checks"]["zmq_matches_ros_r1"] = True
            live.enable_motion()
            armed = True
            for case in cases:
                pose = dict(x=case["before"][0], y=case["before"][1], z=0.0,
                            yaw=case["yaw"], pitch=0.0, roll=0.0)
                answer = live.place(paths, pose)
                if answer["collision"]:
                    raise RuntimeError(f"{case['name']} before-probe is already in collision")
                report["snapshots"][case["name"]+"_before_rgb"] = live.record(args.output_dir, case["name"]+"_before_rgb")
                report["collision_tests"][case["name"]] = live.drive(case, args.drive_seconds, args.speed)
                report["snapshots"][case["name"]+"_stopped_rgb"] = live.record(args.output_dir, case["name"]+"_stopped_rgb")
                if not report["collision_tests"][case["name"]]["passed"]:
                    raise RuntimeError(f"{case['name']} collision verification failed; see measurements")
        report["passed"] = True
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = f"{type(error).__name__}: {error}"
        report["passed"] = False
    finally:
        if armed:
            report["restoration"]["attempted"] = True
            try:
                live.stop()
                answer = live.place(paths, start_pose)
                live.stop()
                gt = live.latest["/base_pose_ground_truth"]
                restored = _pose_from_odom(gt)
                ok = (math.hypot(restored["x"]-start_pose["x"], restored["y"]-start_pose["y"]) < .15
                      and not answer["collision"])
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
        destination = args.output_dir / "forest_live_report.json"
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        print(f"Report: {destination}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
