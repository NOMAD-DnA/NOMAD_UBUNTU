from builtin_interfaces.msg import Time
import math
from types import MethodType, SimpleNamespace as NS

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from nomad_path_planning.local_planner_node import LocalPlannerNode
from nomad_path_planning.rollout import AckermannRollout
from nomad_perception.sensor_input import SensorInput
from nomad_perception.observed_grid import ObservedGrid
from nomad_path_planning.command_selector import PlanningCommandNode


def test_lpp_starts_bounded_reverse_and_blocks_after_three_attempts():
    grid = dict(width=100, height=100, resolution=.1, origin_x=-5., origin_y=-5., data=[0]*10000)
    for y in range(100):
        grid['data'][y*100+55] = 100
    pose = PoseStamped()
    pose.pose.orientation.w = 1.
    published, statuses = [], []
    node = NS(require_sensor_health=False, costmap_msg=True, pose_msg=pose,
              global_path_msg=NS(poses=[1]), forward_origin=None, reverse_origin=None,
              reverse_last=None, reverse_distance=0., recovery_count=0,
              wheelbase=.72, max_steer=.4, reference_offset=.36, forward_speed=.3,
              allow_unknown=False, planner=AckermannRollout(speed=.3),
              make_global_path=lambda: [(0.,0.), (3.,0.)], make_costmap=lambda: grid,
              quaternion_to_yaw=lambda q: 0., publish_failed=lambda failed: None,
              publish_status=statuses.append, publish_local_path=lambda trajectory, steer, speed: published.append(trajectory),
              publish_empty_path=lambda: published.append([]))
    node.reverse_recovery = MethodType(LocalPlannerNode.reverse_recovery, node)
    LocalPlannerNode.try_plan(node)
    assert node.recovery_count == 1
    assert published[-1][-1][0] < -.5
    assert abs(published[-1][-1][0]) <= .63
    assert statuses[-1].startswith('LPP_RECOVERY')
    node.reverse_origin, node.recovery_count = None, 3
    LocalPlannerNode.try_plan(node)
    assert published[-1] == [] and node.recovery_count == 3


def test_controller_stops_when_sim_clock_pauses(monkeypatch):
    commands, statuses = [], []
    monkeypatch.setattr('nomad_path_planning.command_selector.time.monotonic', lambda: 10.)
    from test_history_recovery import controller
    node, commands = controller()
    node.last_sim_time,node.last_clock_change = 10.,9.
    node.report=statuses.append
    PlanningCommandNode.control(node)
    assert statuses[-1] == 'STOP: simulation clock paused'
    assert commands[-1].stop_requested and commands[-1].target_speed == 0.


def test_invalid_or_stale_odometry_cannot_keep_health_pose():
    node = NS(fault=None, frame='odom', base_frame='base_link', fresh=lambda stamp: stamp == 2., max_tilt=.35,
              grid=ObservedGrid(), camera=NS(max_range=6.),
              pose=PoseStamped(), pose_stamp=1., pose_pub=NS(publish=lambda msg: None))
    msg = Odometry()
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    msg.header.stamp.sec = 2
    # Zero quaternion cannot be treated as a usable orientation.
    msg.pose.pose.orientation.w = 0.
    SensorInput.on_odom(node, msg)
    assert node.pose is None
    msg.pose.pose.orientation.w = 1.
    SensorInput.on_odom(node, msg)
    assert node.pose is not None
    msg.header.stamp.sec = 1
    SensorInput.on_odom(node, msg)
    assert node.pose is None


def test_clock_reset_latches_sensor_fault():
    health = []
    node = NS(frame="odom", now=lambda: 1., last_now=10., fault=None, pose=None,
              navigation_lidar_enabled=False, camera=NS(tick=lambda: None, navigation_active=lambda: False),
              process_scan=lambda: None, health_pub=NS(publish=health.append), module_pub=NS(publish=lambda msg: None),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=1))),
              status_pub=NS(publish=lambda msg: None), last_status=None,
              get_logger=lambda: NS(info=lambda msg: None), last_map_wall=math.inf)
    SensorInput.tick(node)
    assert node.fault.startswith('CLOCK_RESET') and not health[-1].data
    node.now = lambda: 20.
    SensorInput.tick(node)
    assert node.fault.startswith('CLOCK_RESET') and not health[-1].data
