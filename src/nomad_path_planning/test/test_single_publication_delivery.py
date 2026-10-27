"""Cross-topic delivery order must not require a repeated GPP publication."""
import itertools
from unittest.mock import Mock
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nomad_path_planning.command_selector import PlanningCommandNode


def pose(x, stamp):
    p = PoseStamped()
    p.header.frame_id = 'odom'
    p.header.stamp.sec = stamp
    p.pose.position.x = float(x)
    p.pose.orientation.w = 1.
    return p


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args', '-p', 'navigation.goal_mode:=observed_corridor'])
    n = PlanningCommandNode()
    n.stop = Mock()
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('order', list(itertools.permutations(['goal', 'target', 'path'])))
def test_single_publications_survive_all_delivery_orders(node, order):
    goal, target = pose(20, 10), pose(6, 11)
    route = Path(header=target.header, poses=[pose(2, 11), target])
    calls = {'goal': lambda: node.on_goal(goal),
             'target': lambda: node.on_navigation_target(target),
             'path': lambda: node.on_global_path(route)}
    for kind in order:
        calls[kind]()
    assert node.global_path is route
    assert node.navigation_target is target
    assert node.goal_epoch == 10


def test_new_mission_does_not_reuse_old_target_and_route(node):
    target = pose(6, 11)
    route = Path(header=target.header, poses=[pose(2, 11), target])
    node.on_goal(pose(20, 10))
    node.on_navigation_target(target)
    node.on_global_path(route)
    node.on_goal(pose(21, 20))
    assert node.global_path is None
    assert node.navigation_target is None
    # Delayed old messages remain rejected.
    node.on_navigation_target(target)
    node.on_global_path(route)
    assert node.global_path is None
    assert node.navigation_target is None


def test_empty_invalidation_clears_single_cached_path(node):
    target = pose(6, 11)
    node.on_goal(pose(20, 10))
    node.on_navigation_target(target)
    node.on_global_path(Path(header=target.header, poses=[pose(2, 11), target]))
    node.on_global_path(Path(header=target.header))
    assert node.global_path is None and node.local_path is None
