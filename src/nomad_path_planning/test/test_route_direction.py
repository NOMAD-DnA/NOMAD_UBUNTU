import math
from types import SimpleNamespace as NS, MethodType

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nomad_path_planning.rollout import AckermannRollout
from nomad_path_planning.command_selector import PlanningCommandNode
from test_history_recovery import controller, path, grid, motion


def xy_path(points, sec=10):
    msg = Path()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = sec
    for x,y in points:
        p = PoseStamped()
        p.pose.position.x, p.pose.position.y, p.pose.orientation.w = float(x),float(y),1.
        msg.poses.append(p)
    return msg


def test_lpp_does_not_choose_least_bad_forward_rollout_for_a_rearward_route():
    result = AckermannRollout(speed=.4).plan((0.,0.,0.),[(0.,0.),(-3.,0.)],grid())
    assert not result['success']


def test_route_flip_immediately_clears_cached_forward_and_rejects_late_local():
    node, commands = controller()
    node.local_path = path([0.,.03,.06])
    node.global_path = path([0.,10.])
    # Goal is still in front; the current route first retreats and detours.
    changed = xy_path([(0.,0.),(-2.,0.),(-2.,3.),(10.,3.),(10.,0.)])
    PlanningCommandNode.on_global_path(node,changed)
    assert node.local_path is None and commands[-1].target_speed == 0.
    PlanningCommandNode.on_local_path(node,path([0.,.03,.06]))
    assert node.local_path is None
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed == 0.


@pytest.mark.parametrize('pose,points,expected', [
    ((0.,0.,0.), [(0.,0.),(-3.,0.)], True),
    ((0.,0.,math.pi/2), [(0.,0.),(0.,-3.)], True),
    ((0.,0.,math.pi), [(0.,0.),(-3.,0.)], False),
    ((0.,0.,0.), [(0.,0.),(0.,3.)], False),
    ((0.,0.,0.), [(0.,0.),(2.,0.),(2.,2.),(-3.,2.)], False),
    ((0.,0.,0.), [(-4.,0.),(0.,0.),(0.,0.),(4.,0.)], False),
])
def test_guard_uses_vehicle_frame_and_next_route_leg(pose,points,expected):
    from nomad_path_planning.route_guard import route_is_behind
    assert route_is_behind(pose,points) is expected


def test_pose_heading_change_cannot_keep_cached_forward_command():
    node,commands = controller()
    node.global_path, node.local_path = path([0.,10.]),path([0.,.03])
    node.local_received_at = 10.
    node.pose.pose.orientation.w,node.pose.pose.orientation.z = 0.,1.
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed == 0. and node.local_path is None


def test_rearward_gpp_updates_do_not_interrupt_active_recovery():
    node,commands = controller()
    PlanningCommandNode.on_recovery_motion(node,motion([0.,-.02]))
    n = len(commands)
    PlanningCommandNode.on_global_path(node,xy_path([(0.,0.),(-2.,0.),(-2.,3.),(10.,0.)]))
    assert len(commands) == n
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed < 0.


def test_supervisor_confirms_rearward_route_without_four_second_stall_wait():
    from nomad_path_planning.history_recovery import Trail, ProgressWindow
    from nomad_path_planning.recovery_supervisor import RecoverySupervisor
    trail = Trail()
    costmap = grid()
    for i in range(21):
        trail.record((i*.15,0.,0.),costmap)
    started = []
    node = NS(mode='FOLLOW', previous_time=None,pose=(3.,0.,0.),goal=(5.,0.),
        grid=costmap,trail=trail,health=True,gates=[],resume_after=0.,frame='odom',
        min_retreat=2.,max_retreat=60.,progress=ProgressWindow(),behind_since=None,
        behind_seconds=.3, report=lambda msg:None,begin=started.append)
    rear = [(3.,0.),(1.,0.),(1.,3.),(5.,3.),(5.,0.)]
    def tick(t,points):
        node.now = lambda:t
        node.pose_at = node.health_at = node.map_at = t
        node.global_path = xy_path(points)
        node.global_path.header.stamp.sec = int(t)
        node.global_path.header.stamp.nanosec = round((t-int(t))*1e9)
        RecoverySupervisor.tick(node)
    tick(10.,rear)
    tick(10.2,rear)
    assert not started
    tick(10.25,[(3.,0.),(5.,0.)])  # Transient change cancels pending reversal.
    tick(10.3,rear)
    tick(10.6,rear)
    assert started == [10.6]
