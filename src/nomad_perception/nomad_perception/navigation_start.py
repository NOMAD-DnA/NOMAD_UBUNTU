"""Start existing autonomy only after localization/perception initialization.

Allows manual teleop warm-up without a simultaneously active Control publisher.
"""

import os, time, math
from pathlib import Path
import rclpy
from rclpy.node import Node
from nomad_interfaces.msg import ModuleStatus
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory


def ready_to_start(latest, wall):
    if not all(k in latest for k in ("vio", "perception", "pose")):
        return False
    if not all(
        latest[k][0].state == ModuleStatus.READY and 0 <= wall - latest[k][1] < 0.6
        for k in ("vio", "perception")
    ):
        return False
    pose, arrived = latest["pose"]
    v = pose.twist.twist.linear
    w = pose.twist.twist.angular
    values = (v.x, v.y, v.z, w.x, w.y, w.z)
    return (
        0 <= wall - arrived < 0.6
        and pose.header.frame_id == "odom"
        and pose.child_frame_id == "base_link"
        and all(math.isfinite(x) for x in values)
        and math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) <= 0.03
        and math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z) <= 0.03
    )


def main():
    rclpy.init()
    n = Node("terrain_navigation_start")
    n.declare_parameter("rviz", True)
    for key in ("gpp", "lpp", "params_file"):
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
    n.get_logger().info(
        "Manual initialization: drive >2m, stop and exit teleop. Autonomy starts after both providers READY and stationary for 3s."
    )
    # Require stable readiness, leaving the user time to stop the warm-up teleop.
    stable = None
    ready = False
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
            wall = time.monotonic()
            ready = ready_to_start(latest, wall)
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
        for key in ("gpp", "lpp", "params_file"):
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
