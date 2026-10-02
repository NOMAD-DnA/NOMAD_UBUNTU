import threading
import time
from types import SimpleNamespace

import rclpy
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from nomad_perception.sensor_input import SensorInput
from nomad_path_planning.input_bridge import PlanningInputs


def test_slow_depth_processing_does_not_block_odometry_or_health():
    rclpy.init(domain_id=85)
    node = SensorInput()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    entered, release = threading.Event(), threading.Event()
    samples = []
    goals = []
    node.now = lambda: 10.
    node.camera.navigation_active = lambda: True
    node.camera.ground_valid = True
    node.health_pub = SimpleNamespace(publish=lambda msg: samples.append(msg.data))
    inputs = SimpleNamespace(frame='odom',get_clock=node.get_clock,
                             goal_pub=SimpleNamespace(publish=goals.append))
    node.published_bounds = (-40., -40., 40., 40.)

    def slow_depth():
        entered.set()
        release.wait(3.)

    node.camera.tick = slow_depth
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp.sec = 10
    msg.pose.pose.orientation.w = 1.
    # Same callback group as real subscriptions and the health timer.
    node.create_timer(.02, lambda: node.on_odom(msg))
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert entered.wait(2.)
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.pose.position.x = 2.
        PlanningInputs.on_goal(inputs,goal)
        node.on_goal(goal)
        assert len(goals) == 1 and goals[0].pose.position.x == 2.
        assert node.pending_goal is goal
        samples.clear()
        deadline = time.monotonic()+1.
        while len(samples) < 3 and time.monotonic() < deadline:
            time.sleep(.01)
        assert not release.is_set()
        assert len(samples) >= 3 and all(samples)
        assert node.pose is not None
    finally:
        release.set()
        executor.shutdown()
        thread.join(timeout=3.)
        node.destroy_node()
        rclpy.shutdown()
