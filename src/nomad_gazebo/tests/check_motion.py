#!/usr/bin/env python3
"""Bounded, opt-in motion check for the isolated NOMAD Gazebo test server.

Never teleports, pauses, removes, or terminates the simulator. Commands are
published only on /cmd_vel after strict domain, partition, Gazebo and ROS graph
checks. This checks short flat-ground movement and watchdog expiry, NOT hill
climbing, motor torque, collision avoidance, or autonomous navigation.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from typing import Any


PARTITION = "nomad_gazebo_43"
WORLD = "nomad_forest"
NATIVE_ODOM = "/nomad/vehicle/odom"
SPEED = 0.25
DRIVE_SIM_SECONDS = 3.0
DRIVE_WALL_SECONDS = 8.0


def require_environment() -> None:
    if os.environ.get("ROS_DOMAIN_ID") != "43" or os.environ.get("NOMAD_ROS_DOMAIN_ID") != "43":
        raise RuntimeError("ROS_DOMAIN_ID and NOMAD_ROS_DOMAIN_ID must both be exactly 43")
    if os.environ.get("GZ_PARTITION") != PARTITION:
        raise RuntimeError(f"GZ_PARTITION must be exactly {PARTITION}")


def gz_read(arguments: list[str]) -> str:
    result = subprocess.run(["gz", *arguments], capture_output=True, text=True,
                            timeout=5.0, check=False)
    if result.returncode:
        raise RuntimeError(f"Gazebo read failed ({result.returncode}): {result.stderr[-400:]}")
    return result.stdout


def native_gate() -> dict[str, Any]:
    services = gz_read(["service", "-l"]).splitlines()
    if f"/world/{WORLD}/control" not in services or f"/world/{WORLD}/scene/info" not in services:
        raise RuntimeError("Expected NOMAD world control and scene services are absent")
    info = gz_read(["topic", "-i", "-t", NATIVE_ODOM])
    publisher_section = info.split("Subscribers", 1)[0]
    publishers = re.findall(r"^\s+(\S+),\s+gz\.msgs\.Odometry\s*$", publisher_section, re.MULTILINE)
    if len(publishers) != 1:
        raise RuntimeError(f"Expected exactly one native odometry publisher, found {len(publishers)}")
    raw = gz_read(["topic", "-e", "-t", NATIVE_ODOM, "--json-output", "-n", "1"])
    messages = [json.loads(line) for line in raw.splitlines() if line.strip().startswith("{")]
    if len(messages) != 1:
        raise RuntimeError("Did not receive one native JSON odometry observation")
    return {"publishers": publishers, "services_verified": True, "odom": messages[0]}


def xyz(data: dict[str, Any]) -> list[float]:
    return [float(data.get(axis, 0.0)) for axis in "xyz"]


def quaternion(data: dict[str, Any]) -> list[float]:
    return [float(data.get(axis, 1.0 if axis == "w" else 0.0)) for axis in "xyzw"]


def distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def orientation_distance(a: list[float], b: list[float]) -> float:
    return min(distance(a, b), distance(a, [-value for value in b]))


class Live:
    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data

        self.rclpy, self.Twist = rclpy, Twist
        self.name = f"nomad_motion_check_{os.getpid()}"
        self.node = rclpy.create_node(self.name)
        self.odom: collections.deque[dict[str, Any]] = collections.deque(maxlen=250)
        self.commands: collections.deque[dict[str, Any]] = collections.deque(maxlen=100)
        self.publisher = None
        self.subscriptions = [
            self.node.create_subscription(Odometry, "/odom", self.receive_odom, qos_profile_sensor_data),
            self.node.create_subscription(Twist, "/nomad/vehicle/cmd_vel", self.receive_command,
                                          qos_profile_sensor_data),
        ]

    def receive_odom(self, message: Any) -> None:
        pose, twist = message.pose.pose, message.twist.twist
        values = {
            "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            "wall": time.monotonic(), "frame": message.header.frame_id,
            "child": message.child_frame_id,
            "xyz": [pose.position.x, pose.position.y, pose.position.z],
            "quaternion": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            "linear": [twist.linear.x, twist.linear.y, twist.linear.z],
            "angular": [twist.angular.x, twist.angular.y, twist.angular.z],
        }
        if not all(math.isfinite(value) for key in ("xyz", "quaternion", "linear", "angular")
                   for value in values[key]):
            raise RuntimeError("Received non-finite odometry")
        self.odom.append(values)

    def receive_command(self, message: Any) -> None:
        self.commands.append({"wall": time.monotonic(), "linear_x": message.linear.x,
                              "angular_z": message.angular.z})

    def pump(self, seconds: float = 0.02) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=seconds)

    @staticmethod
    def names(entries: list[Any]) -> list[str]:
        return [f"{entry.node_namespace.rstrip('/')}/{entry.node_name}" for entry in entries]

    def graph(self, own_publisher: bool = False) -> dict[str, Any]:
        graph = {
            "odom_publishers": self.names(self.node.get_publishers_info_by_topic("/odom")),
            "cmd_publishers": self.names(self.node.get_publishers_info_by_topic("/cmd_vel")),
            "cmd_subscribers": self.names(self.node.get_subscriptions_info_by_topic("/cmd_vel")),
            "vehicle_cmd_publishers": self.names(
                self.node.get_publishers_info_by_topic("/nomad/vehicle/cmd_vel")),
        }
        expected_publishers = [f"/{self.name}"] if own_publisher else []
        if graph != {"odom_publishers": ["/nomad_gz_bridge"],
                     "cmd_publishers": expected_publishers,
                     "cmd_subscribers": ["/nomad_cmd_watchdog"],
                     "vehicle_cmd_publishers": ["/nomad_cmd_watchdog"]}:
            raise RuntimeError(f"Unsafe or incompletely discovered ROS graph: {graph}")
        if any(name == "mvsim" for name, _namespace in self.node.get_node_names_and_namespaces()):
            raise RuntimeError("Unexpected MVSim node in the test domain")
        return graph

    def wait_initial(self) -> dict[str, Any]:
        deadline = time.monotonic() + 12.0
        last_error = "No odometry"
        while time.monotonic() < deadline:
            self.pump(0.1)
            try:
                graph = self.graph()
                latest = self.latest()
                if latest["frame"] != "odom" or latest["child"] != "base_link":
                    raise RuntimeError("Unexpected odometry frames")
                if distance(latest["linear"], [0, 0, 0]) >= 0.02 or distance(latest["angular"], [0, 0, 0]) >= 0.02:
                    raise RuntimeError("Robot is not stopped before testing")
                if len(self.odom) >= 5:
                    return graph
            except RuntimeError as error:
                last_error = str(error)
        raise RuntimeError(f"Initial safety gate timed out: {last_error}")

    def latest(self) -> dict[str, Any]:
        if not self.odom or time.monotonic() - self.odom[-1]["wall"] > 0.8:
            raise RuntimeError("Odometry absent or stale by more than 0.8 seconds")
        return self.odom[-1]

    def compare_native(self, native: dict[str, Any]) -> dict[str, Any]:
        # The robot is stationary, so independent subscription times should
        # give identical world/odom coordinates, not merely an arbitrary offset.
        deadline = time.monotonic() + 2.0
        stamp = native.get("header", {}).get("stamp", {})
        native_stamp = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9
        while time.monotonic() < deadline:
            self.pump(0.05)
            if self.latest()["stamp"] >= native_stamp:
                break
        closest = min(self.odom, key=lambda value: abs(value["stamp"] - native_stamp))
        pose = native["pose"]
        position_error = distance(closest["xyz"], xyz(pose.get("position", {})))
        rotation_error = orientation_distance(closest["quaternion"], quaternion(pose.get("orientation", {})))
        stamp_error = abs(closest["stamp"] - native_stamp)
        if position_error > 0.001 or rotation_error > 0.0001 or stamp_error > 0.08:
            raise RuntimeError(f"Native/ROS odometry mismatch: xyz={position_error}, q={rotation_error}, t={stamp_error}")
        return {"position_error_m": position_error, "quaternion_error": rotation_error,
                "stamp_error_s": stamp_error, "ros": closest}

    def begin_commands(self) -> None:
        self.publisher = self.node.create_publisher(self.Twist, "/cmd_vel", 5)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.pump(0.05)
            try:
                self.graph(own_publisher=True)
                return
            except RuntimeError:
                pass
        raise RuntimeError("Own publisher discovery or exclusive command gate failed")

    def command(self, speed: float) -> None:
        if self.publisher is None:
            raise RuntimeError("Motion was not opted in")
        message = self.Twist()
        message.linear.x = speed
        self.publisher.publish(message)

    def drive(self) -> dict[str, Any]:
        start = dict(self.latest())
        qx, qy, qz, qw = start["quaternion"]
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        forward = (math.cos(yaw), math.sin(yaw))
        start_wall = time.monotonic()
        next_command = start_wall
        next_gate = start_wall
        positions = []
        while self.latest()["stamp"] - start["stamp"] < DRIVE_SIM_SECONDS:
            now = time.monotonic()
            if now - start_wall > DRIVE_WALL_SECONDS:
                raise RuntimeError("Motion phase exceeded 8 wall-clock seconds")
            if now >= next_gate:
                self.graph(own_publisher=True)
                next_gate = now + 0.25
            if now >= next_command:
                self.command(SPEED)
                next_command = now + 0.05
            self.pump()
            latest = self.latest()
            positions.append(latest["xyz"])
            if distance(latest["xyz"][:2], start["xyz"][:2]) >= 1.0:
                raise RuntimeError("Travel reached the 1 metre safety bound")
        end = dict(self.latest())
        displacement = [end["xyz"][axis] - start["xyz"][axis] for axis in (0, 1)]
        progress = sum(displacement[axis] * forward[axis] for axis in (0, 1))
        lateral = -displacement[0] * forward[1] + displacement[1] * forward[0]
        if not 0.3 <= progress < 1.0 or abs(lateral) > 0.1:
            raise RuntimeError(f"Unexpected short straight motion: forward={progress}, lateral={lateral}")
        return {"start": start, "end": end, "forward_m": progress,
                "lateral_m": lateral, "distance_m": distance(start["xyz"][:2], end["xyz"][:2]),
                "sim_seconds": end["stamp"] - start["stamp"],
                "wall_seconds": time.monotonic() - start_wall}

    def watchdog_expiry(self) -> dict[str, Any]:
        # Deliberately cease /cmd_vel for 0.8 wall seconds, without deleting the
        # publisher or sending a stop. The existing gate must output zero.
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            self.graph(own_publisher=True)
            self.pump(0.02)
        if not self.commands or time.monotonic() - self.commands[-1]["wall"] > 0.2:
            raise RuntimeError("Watchdog output was absent/stale")
        output = self.commands[-1]
        if abs(output["linear_x"]) > 1e-9 or abs(output["angular_z"]) > 1e-9:
            raise RuntimeError(f"Watchdog failed to expire: {output}")
        return {"silence_wall_seconds": 0.8, "output": output, "zero_verified": True}

    def stop_and_verify(self) -> dict[str, Any]:
        # At least ten zeros over at least 0.5 wall seconds, even on exceptions.
        stop_start = time.monotonic()
        count = 0
        while count < 10 or time.monotonic() - stop_start < 0.5:
            self.command(0.0)
            count += 1
            end = time.monotonic() + 0.05
            while time.monotonic() < end:
                self.pump(0.01)
        deadline = time.monotonic() + 4.0
        tail_start = None
        final = None
        while time.monotonic() < deadline:
            self.command(0.0)
            self.pump(0.05)
            final = dict(self.latest())
            if distance(final["linear"], [0, 0, 0]) < 0.05 and distance(final["angular"], [0, 0, 0]) < 0.05:
                if tail_start is None:
                    tail_start = final
                if final["stamp"] - tail_start["stamp"] >= 0.3:
                    drift = distance(final["xyz"][:2], tail_start["xyz"][:2])
                    if drift >= 0.03:
                        raise RuntimeError(f"Robot drifted after stopping: {drift}m")
                    return {"zeros_sent_initial": count, "zero_wall_seconds": time.monotonic() - stop_start,
                            "final": final, "tail_xy_drift_m": drift, "stopped_verified": True}
            else:
                tail_start = None
        raise RuntimeError(f"Robot did not settle after stop: {final}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exercise", action="store_true", help="explicitly authorize /cmd_vel writes after all gates")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    report: dict[str, Any] = {"passed": False, "exercise": args.exercise,
                              "domain": 43, "partition": PARTITION, "world": WORLD,
                              "scope": "short flat-ground movement and watchdog expiry only"}
    live = None
    initialized = False
    def interrupted(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt("Interrupted; stopping the isolated test vehicle")
    signal.signal(signal.SIGTERM, interrupted)
    try:
        require_environment()
        import rclpy

        rclpy.init()
        initialized = True
        live = Live()
        report["ros_graph"] = live.wait_initial()
        report["native"] = native_gate()
        report["native_ros_match"] = live.compare_native(report["native"]["odom"])
        live.graph()
        if args.exercise:
            live.begin_commands()
            report["motion"] = live.drive()
            report["watchdog"] = live.watchdog_expiry()
        report["passed"] = True
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        if live is not None:
            if live.publisher is not None:
                try:
                    report["cleanup"] = live.stop_and_verify()
                    if "motion" in report:
                        start = report["motion"]["start"]["xyz"]
                        end = report["cleanup"]["final"]["xyz"]
                        total_distance = distance(start[:2], end[:2])
                        report["motion"]["total_distance_until_stop_m"] = total_distance
                        if not 0.3 <= total_distance < 1.0:
                            raise RuntimeError(f"Total travel including watchdog coast exceeded bounds: {total_distance}m")
                except (Exception, KeyboardInterrupt) as error:
                    report["passed"] = False
                    report["cleanup_error"] = f"{type(error).__name__}: {error}"
            live.node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        path = args.output_dir / "motion_report.json"
        path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps({"passed": report["passed"], "report": str(path),
                          "motion": report.get("motion"), "watchdog": report.get("watchdog"),
                          "cleanup": report.get("cleanup"), "error": report.get("error"),
                          "cleanup_error": report.get("cleanup_error")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
