#!/usr/bin/env python3
"""Log/STOP integration test; explicit isolated domain, synthetic inputs only."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    domain = int(os.environ.get("ROS_DOMAIN_ID", "-1"))
    if not 100 <= domain <= 230 or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Use an unused ROS domain 100..230 and ROS_LOCALHOST_ONLY=1")

    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nomad_interfaces.msg import ModuleStatus, PlanningCostLayers, DriveCommand

    rclpy.init()
    node = rclpy.create_node("perception_status_diagnostic_test")
    prefix = f"/status_diagnostic_test_{os.getpid()}"
    mappings = {
        "/goal_pose": "goal", "/nomad/localization/odometry": "odom",
        "/nomad/localization/status": "loc", "/nomad/perception/status": "per",
        "/nomad/planning_cost_layers": "layers", "/nomad/navigation_local_costmap": "local",
        "/nomad/camera_observation_support": "support", "/nomad/control/vehicle_state": "vehicle",
        "/nomad/planning/drive_command": "drive", "/nomad/planning/status": "status",
        "/nomad/global_path": "global_path", "/nomad/local_path": "local_path",
        "/nomad/goal": "accepted_goal", "/nomad/planning/arm": "arm",
        "/nomad/planning/cancel": "cancel",
    }
    command = [str(args.planner.resolve()), "--ros-args", "-r", "__node:=status_diagnostic_planner"]
    for old, new in mappings.items():
        command += ["-r", f"{old}:={prefix}/{new}"]
    for param in ["plan_only:=true", "prototype_output:=false", "planning_hz:=0.5",
                  "status_timeout:=0.5", "corridor.observation_guard:=false",
                  "corridor.observation_turn_weight:=0.0"]:
        command += ["-p", param]

    reliable = QoSProfile(depth=5)
    sensor = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {name: node.create_publisher(typ, f"{prefix}/{name}", qos) for name, typ, qos in [
        ("odom", Odometry, sensor), ("loc", ModuleStatus, reliable), ("per", ModuleStatus, reliable),
        ("layers", PlanningCostLayers, latched), ("local", OccupancyGrid, latched),
        ("goal", PoseStamped, reliable)]}
    commands = []
    subscription = node.create_subscription(DriveCommand, f"{prefix}/drive", commands.append, 5)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    log_file = args.log.open("w")
    env = dict(os.environ, RCUTILS_COLORIZED_OUTPUT="0", RCUTILS_LOGGING_USE_STDOUT="0")
    proc = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT, env=env)

    def records():
        rows = []
        for line in args.log.read_text().splitlines():
            if " | perception_status=" in line:
                rows.append(json.loads(line.split(" | perception_status=", 1)[1]))
        return rows

    def publish(mode):
        now = node.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        pubs["odom"].publish(odom)
        loc = ModuleStatus()
        loc.header.stamp = now.to_msg()
        loc.state = ModuleStatus.READY
        loc.valid_for.sec = 2
        pubs["loc"].publish(loc)
        if mode not in ("missing", "receive_timeout"):
            per = ModuleStatus()
            per.header.stamp = now.to_msg()
            per.state = ModuleStatus.READY
            per.valid_for.sec = 2
            per.detail = 'synthetic "perception"\nstatus'
            if mode == "waiting":
                per.state = ModuleStatus.WAITING
            elif mode == "fault":
                per.state = ModuleStatus.FAULT
                per.detail = "가" * 180
            elif mode == "source_timeout":
                per.header.stamp = (now - Duration(seconds=1.0)).to_msg()
            elif mode == "lease_expired":
                per.header.stamp = (now - Duration(seconds=.3)).to_msg()
                per.valid_for.sec = 0
                per.valid_for.nanosec = 100000000
            elif mode == "invalid_lease":
                per.valid_for.sec = 0
            elif mode == "future_stamp":
                per.header.stamp = (now + Duration(seconds=1.0)).to_msg()
            elif mode == "zero_stamp":
                per.header.stamp.sec = per.header.stamp.nanosec = 0
            pubs["per"].publish(per)
        grid = OccupancyGrid()
        grid.header.stamp = now.to_msg()
        grid.header.frame_id = "odom"
        grid.info.resolution = .2
        grid.info.width = grid.info.height = 40
        grid.info.origin.position.x = grid.info.origin.position.y = -4.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 1600
        layers = PlanningCostLayers()
        layers.grid = grid
        layers.slope_cost = layers.terrain_cost = [0.] * 1600
        pubs["layers"].publish(layers)
        pubs["local"].publish(grid)

    def pump(mode, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if proc.poll() is not None:
                raise RuntimeError(f"planner exited: {proc.returncode}")
            publish(mode)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.025)

    def phase(mode, cause, category, seconds=.65):
        before = len(records())
        pump(mode, seconds)
        rows = records()[before:]
        matching = [r for r in rows if r["origin"] == "current_input"
                    and r["failure"]["cause"] == cause and r["failure"]["category"] == category]
        assert matching, (mode, rows)
        print(f"PASS {mode}: {category}/{cause}", flush=True)
        return matching

    try:
        deadline = time.monotonic() + 10
        while pubs["goal"].get_subscription_count() == 0 and time.monotonic() < deadline:
            pump("missing", .1)
        assert pubs["goal"].get_subscription_count() == 1, "planner discovery failed"
        pump("missing", .3)
        assert not records(), "no-goal startup must not produce LPP pauses"
        goal = PoseStamped()
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.header.frame_id = "odom"
        goal.pose.position.x = 2.
        goal.pose.orientation.w = 1.
        pubs["goal"].publish(goal)
        phase("missing", "NO_MESSAGE", "MISSING", 2.3)
        before = len(records())
        pump("healthy", .4)
        assert any(r["origin"] == "worker_result" and r["failure"]["category"] == "MISSING"
                   and r["current"]["category"] == "READY" for r in records()[before:]), "lost worker failure evidence"
        print("PASS worker_result retains failure while current status is READY", flush=True)
        rows = phase("waiting", "STATE_NOT_READY", "STATE_NOT_READY")
        assert rows[0]["failure"]["detail"] == 'synthetic "perception"\nstatus', "detail escaping changed content"
        count = len(records())
        pump("waiting", .8)
        assert len(records()) == count, "same failure must not flood at 20 Hz"
        print("PASS duplicate suppression and JSON detail escaping", flush=True)
        rows = phase("fault", "STATE_NOT_READY", "STATE_NOT_READY")
        assert rows[0]["failure"]["detail"] == "가" * 170 and rows[0]["failure"]["detail_truncated"], "UTF-8 detail truncation"
        phase("source_timeout", "SOURCE_TIMEOUT", "TIMING_INVALID")
        phase("lease_expired", "LEASE_EXPIRED", "TIMING_INVALID")
        phase("invalid_lease", "VALID_FOR_INVALID", "TIMING_INVALID")
        phase("future_stamp", "STAMP_IN_FUTURE", "TIMING_INVALID")
        phase("zero_stamp", "STAMP_INVALID", "TIMING_INVALID")
        pump("healthy", .4)
        phase("receive_timeout", "RECEIVE_TIMEOUT", "TIMING_INVALID", .9)
        pump("healthy", 2.5)
        assert commands and all(c.stop_requested and c.target_speed == 0 and c.target_steering_angle == 0 for c in commands), "plan-only STOP contract failed"
        assert "[LPP_CANDIDATE_RECOVERED]" in args.log.read_text(), "no candidate recovery"
        print(f"PASS ROS diagnostics; {len(commands)} commands all STOP; no arm request or controller/driver launch", flush=True)
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=5)
        log_file.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
