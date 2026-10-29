"""Start existing autonomy only after localization/perception initialization.

Allows manual teleop warm-up without a simultaneously active Control publisher.
"""

import os, time, math
from pathlib import Path
import rclpy
from rclpy.node import Node
from nomad_interfaces.msg import ModuleStatus
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


def localization_ready(latest, wall):
    """Alignment completion is distinct from map readiness and vehicle stopping."""
    if not all(key in latest for key in ("vio", "pose")):
        return False
    status, arrived = latest["vio"]
    pose, pose_arrived = latest["pose"]
    p, q = pose.pose.pose.position, pose.pose.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return (status.state == ModuleStatus.READY and 0 <= wall-arrived < .6
            and 0 <= wall-pose_arrived < .6 and pose.header.frame_id == "odom"
            and pose.child_frame_id == "base_link" and all(math.isfinite(v) for v in values)
            and q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w > 1e-8)


def ready_to_start(latest, wall, motion_key="pose"):
    if not all(k in latest for k in ("vio", "perception", "pose", motion_key)):
        return False
    if not all(
        latest[k][0].state == ModuleStatus.READY and 0 <= wall - latest[k][1] < 0.6
        for k in ("vio", "perception")
    ):
        return False
    pose, arrived = latest["pose"]
    motion, motion_arrived = latest[motion_key]
    v = motion.twist.twist.linear
    w = motion.twist.twist.angular
    values = (v.x, v.y, v.z, w.x, w.y, w.z)
    return (
        0 <= wall - arrived < 0.6
        and 0 <= wall - motion_arrived < 0.6
        and pose.header.frame_id == "odom"
        and pose.child_frame_id == "base_link"
        and all(math.isfinite(x) for x in values)
        and math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) <= 0.03
        and math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z) <= 0.03
    )


def finish_initialization_teleop(node):
    """Stop the startup teleop and wait until its command publisher disappears."""
    def active():
        return any(info.node_name == "nomad_gazebo_teleop" for info in node.get_publishers_info_by_topic("/cmd_vel"))
    if not active():
        return True  # The user may already have closed the initialization window.
    client = node.create_client(Trigger, "/nomad/initialization/finish_teleop")
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().warn("Close the manual teleop window before autonomy starts.")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        if not future.done() or future.exception() is not None or not future.result().success:
            return False
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and active() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return not active()
    finally:
        node.destroy_client(client)


def main():
    rclpy.init()
    n = Node("terrain_navigation_start")
    n.declare_parameter("rviz", True)
    n.declare_parameter("initial_teleop", False)
    for key in ("gpp", "lpp", "params_file", "vehicle_file"):
        n.declare_parameter(key, "")
    n.declare_parameter(
        "modules_file",
        str(
            Path(get_package_share_directory("nomad_perception"))
            / "config/modules_terrain_navigation.yaml"
        ),
    )
    latest = {}
    subs = []
    for key, topic in [
        ("vio", "/nomad/localization/status"),
        ("perception", "/nomad/perception/status"),
    ]:

        def cb(m, key=key):
            latest[key] = (m, time.monotonic())

        subs.append(n.create_subscription(ModuleStatus, topic, cb, 10))
    subs.append(
        n.create_subscription(
            Odometry,
            "/nomad/localization/odometry",
            lambda m: latest.update(pose=(m, time.monotonic())),
            10,
        )
    )
    subs.append(n.create_subscription(
        Odometry, "/odom", lambda m: latest.update(motion=(m, time.monotonic())), 10))
    n.get_logger().info(
        "Initial alignment: drive >2m and stop. Managed teleop stops and closes immediately after alignment; external teleop must be closed. Autonomy starts after both providers READY and stationary for 3s."
    )
    # Require stable readiness, leaving the user time to stop the warm-up teleop.
    stable = None
    ready = False
    teleop_released = not n.get_parameter("initial_teleop").value
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            wall = time.monotonic()
            if not teleop_released:
                if not localization_ready(latest, wall):
                    continue
                n.get_logger().info("Localization aligned: stopping initial teleop and closing its window.")
                teleop_released = finish_initialization_teleop(n)
                stable = None
                if not teleop_released:
                    continue
                n.get_logger().info("Initial teleop closed. Waiting for map readiness and physical stop for 3s.")
                continue
            ready = ready_to_start(latest, wall, motion_key="motion")
            if ready:
                stable = wall if stable is None else stable
                if wall - stable >= 3:
                    break
            else:
                stable = None
        args = [
            "ros2",
            "launch",
            "nomad_bringup",
            "autonomy.launch.py",
            "modules_file:=" + n.get_parameter("modules_file").value,
            "rviz:=" + str(n.get_parameter("rviz").value).lower(),
        ]
        for key in ("gpp", "lpp", "params_file", "vehicle_file"):
            value = n.get_parameter(key).value
            if value:
                args.append(key + ":=" + value)
    except KeyboardInterrupt:
        ready = False
    finally:
        n.destroy_node()
        rclpy.shutdown() if rclpy.ok() else None
    if ready:
        os.execvp("ros2", args)
