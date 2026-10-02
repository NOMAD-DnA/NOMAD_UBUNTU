import math
from types import SimpleNamespace as NS, MethodType

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from nomad_interfaces.msg import PlannedMotion, VehicleState
from builtin_interfaces.msg import Time
import time
from nomad_path_planning.history_recovery import (
    Trail, ProgressWindow, ReverseFollower, EntryGate, allowed_path, distance)
from nomad_path_planning.rollout import AckermannRollout
from nomad_path_planning.command_selector import PlanningCommandNode
from nomad_path_planning.recovery_supervisor import RecoverySupervisor
from nomad_path_planning.dstar_lite import DStarLite
from nomad_path_planning.recovery_gpp import astar_path


def grid():
    return dict(width=200, height=200, resolution=.1, origin_x=-10., origin_y=-10., data=[0]*40000)


@pytest.mark.parametrize('steer', [0., .25, -.35])
def test_closed_loop_multimeter_reverse_preserves_vehicle_heading(steer):
    costmap = grid()
    forward = AckermannRollout(speed=.2, horizon=4., steer_samples=17)
    driven = forward.simulate((0., 0., 0.), steer)
    trail = Trail()
    for pose in driven:
        trail.record(pose, costmap)
    index, reference = trail.select(driven[-1], min_retreat=3., max_retreat=4.5)
    follower = ReverseFollower(reference)
    pose = driven[-1]
    travelled = 0.
    for _ in range(300):
        if follower.advance(pose):
            break
        rollout = follower.plan(pose, costmap)
        assert rollout is not None
        nxt = rollout[1]
        assert ((nxt[0]-pose[0])*math.cos(pose[2])+(nxt[1]-pose[1])*math.sin(pose[2])) < 0
        travelled += distance(pose, nxt)
        pose = nxt
    else:
        pytest.fail('reverse did not reach checkpoint')
    assert travelled > 2.7
    assert distance(pose, trail.points[index].pose) < .19


@pytest.mark.parametrize('value', [-1, 100])
def test_reverse_rechecks_rear_obstacle_and_unknown(value):
    costmap = grid()
    follower = ReverseFollower([(0.,0.,0.), (-1.,0.,0.), (-2.,0.,0.)])
    assert follower.plan((0.,0.,0.), costmap)
    # Full barrier behind car, unavoidable by every steering candidate.
    for y in range(200):
        costmap['data'][y*200+97] = value
    assert follower.plan((0.,0.,0.), costmap) is None


def test_stuck_detection_and_history_discontinuity():
    detector = ProgressWindow()
    for i in range(41):
        stuck = detector.stalled(i*.1, (.03*(i%2),0.,0.))
    assert stuck
    detector.clear()
    assert not any(detector.stalled(i*.1, (i*.03,0.,0.)) for i in range(100))
    trail = Trail()
    trail.record((0.,0.,0.), grid())
    trail.record((3.,0.,0.), grid())
    assert trail.select((3.,0.,0.)) is None


def test_directed_gate_applies_to_both_planners_without_blocking_retreat():
    gate = EntryGate(4.5, 5., 0., 2.)
    assert not allowed_path([(4.,5.), (5.,5.)], [gate])
    assert allowed_path([(5.,5.), (4.,5.)], [gate])
    planner = DStarLite(12,12,[0]*144,(2,5),(8,5))
    planner.edge_filter = lambda a,b: not gate.blocks(a,b)
    assert planner.compute_shortest_path()
    for path in (planner.extract_path(), astar_path(planner)):
        assert path[-1] == (8,5)
        assert allowed_path(path, [gate])
        assert any(abs(p[1]-5)>2 for p in path)


def path(points, sec=10):
    msg = Path()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = sec
    for x in points:
        p = PoseStamped()
        p.pose.position.x = float(x)
        p.pose.orientation.w = 1.
        msg.poses.append(p)
    return msg


def motion(points, sec=10):
    value = path(points,sec)
    speed = (points[1]-points[0])/.1 if len(points)>=2 else 0.
    return PlannedMotion(path=value,active=bool(points),target_speed=float(speed))


def controller():
    commands = []
    pose, goal = path([0.,10.]).poses
    vehicle = VehicleState(ready=True,speed_valid=True,steering_valid=True)
    vehicle.header.frame_id='base_link'
    vehicle.header.stamp.sec=10
    node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000,
                                                 to_msg=lambda:Time(sec=10))),
        recovery_active=False,recovery_path=None,recovery_received_at=0.,goal_epoch=0.,
        planning_frame='odom',base_frame='base_link',local_path=None,local_received_at=0.,
        global_path=None,pose=pose,goal=goal,pose_received_at=10.,
        last_sim_time=None,last_clock_change=0.,sync_ok=True,sync_received_at=10.,
        max_speed=.3,max_steer=.4,health_label='test',path_clear=lambda msg:True,
        vehicle_state=vehicle,vehicle_wall=time.monotonic(),
        local_motion=motion([0.,.03]),recovery_motion=None,
        report=lambda msg:None,cmd_pub=NS(publish=commands.append))
    for name in ('command_message','stop','on_recovery_path','on_local_path'):
        setattr(node,name,MethodType(getattr(PlanningCommandNode,name),node))
    return node,commands


def test_controller_reverse_without_global_path_and_stale_hold():
    node, commands = controller()
    PlanningCommandNode.on_recovery_motion(node, motion([0.,-.02,-.04]))
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed == pytest.approx(-.2)
    node.recovery_received_at = 9.
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed == 0.
    assert node.recovery_active
    PlanningCommandNode.on_recovery_motion(node, motion([0.]))
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed == 0.
    PlanningCommandNode.on_recovery_motion(node, motion([]))
    assert not node.recovery_active and node.local_path is None


def test_controller_accepts_verified_forward_exit_but_rejects_old_goal_recovery():
    node, commands = controller()
    PlanningCommandNode.on_recovery_motion(node, motion([0.,.02]))
    assert node.recovery_active and node.recovery_path is not None
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed == pytest.approx(.2)
    node.recovery_active = False
    node.goal_epoch = 10.1
    PlanningCommandNode.on_recovery_motion(node, motion([0.,-.02]))
    assert not node.recovery_active


def test_controller_updated_map_interrupts_reverse():
    node, commands = controller()
    node.recovery_path = path([0.,-.02])
    node.path_clear = lambda msg: False
    PlanningCommandNode.on_costmap(node, OccupancyGrid())
    assert node.recovery_path is None and commands[-1].target_speed == 0.


def test_escape_probe_rejects_stale_or_wrong_goal_global_path():
    fresh = path([1.,5.], sec=38)
    fresh.header.stamp.nanosec = 500_000_000
    calls = []
    node = NS(global_path=fresh, frame='odom', started=38.50000000000001,
        goal=(5.,0.), pose=(1.,0.,0.), grid=grid(), gates=[],
        follower=NS(reference=[(0.,0.,0.),(4.,0.,0.)]),
        probe=NS(plan=lambda *args: calls.append(args)))
    RecoverySupervisor.find_exit(node, 39.)
    assert len(calls) == 1  # Same tick is accepted despite float roundoff.
    RecoverySupervisor.find_exit(node, 45.)
    node.goal = (6.,0.)
    RecoverySupervisor.find_exit(node, 39.)
    assert len(calls) == 1


def test_supervisor_cannot_recover_without_history_and_holds_stale_inputs():
    statuses = []
    node = NS(progress=ProgressWindow(), trail=Trail(), pose=(0.,0.,0.),
        min_retreat=2., max_retreat=8., escape_clearance=1., attempts=0,
        max_attempts=4, mode='REVERSE', hold=statuses.append)
    RecoverySupervisor.begin(node, 10.)
    assert node.mode == 'STOP'
    node.now = lambda: 10.
    node.previous_time = 9.
    node.mode = 'REVERSE'
    node.grid, node.goal = grid(), (5.,0.)
    node.health = True
    node.health_at, node.pose_at, node.map_at = 10.,10.,9.
    RecoverySupervisor.tick(node)
    assert node.mode == 'REVERSE' and statuses[-1].startswith('RECOVERY_PAUSED')


def test_delayed_first_plan_does_not_latch_recovery_stop_and_later_stall_recovers():
    published = []
    node = NS(previous_time=None, mode='FOLLOW', pose=(0.,0.,0.), grid=grid(),
        goal=(8.,0.), health=True, gates=[], resume_after=.3, trail=Trail(),
        progress=ProgressWindow(), min_retreat=2., max_retreat=8., escape_clearance=1.,
        attempts=0, max_attempts=4, wheelbase=.72, max_steer=.4, offset=.36, speed=.2,
        hold=lambda msg: published.append(msg), report=lambda msg: None,
        reference_pub=NS(publish=lambda msg: None), path_message=lambda poses: poses,
        publish_gates=lambda: None, gate_width=2., gate_ttl=120.)
    node.begin = MethodType(RecoverySupervisor.begin, node)
    def tick(t, x):
        node.now = lambda: t
        node.health_at = node.pose_at = node.map_at = t
        node.pose = (x,0.,0.)
        RecoverySupervisor.tick(node)
    # Real failure: first GPP calculation takes longer than the stuck window.
    for i in range(121):
        tick(i*.1, 0.)
        assert node.mode == 'FOLLOW', 'Initial planning wait must not seize control'
    # Delayed valid planning may now start normal driving, without a new goal.
    for i in range(1, 101):
        tick(12.+i*.1, i*.03)
        assert node.mode == 'FOLLOW'
    assert not published
    # A genuine dead end after enough driven history must still recover.
    for i in range(1, 61):
        tick(22.+i*.1, 3.)
        if node.mode == 'REVERSE':
            break
    assert node.mode == 'REVERSE' and node.attempts == 1


def test_missing_history_in_follow_does_not_take_control():
    published, statuses = [], []
    node = NS(progress=ProgressWindow(), trail=Trail(), pose=(0.,0.,0.),
        min_retreat=2., max_retreat=8., escape_clearance=1., attempts=0,
        max_attempts=4, mode='FOLLOW', hold=published.append, report=statuses.append)
    RecoverySupervisor.begin(node, 10.)
    assert node.mode == 'FOLLOW' and not published
    assert statuses[-1].startswith('RECOVERY_WAIT_HISTORY')
