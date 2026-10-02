"""No DDS or vehicle commands: exercise real planners and supervisor callbacks."""
import math
from types import SimpleNamespace as NS, MethodType

import pytest

from nomad_path_planning.escape_probe import EscapeProbe, ExitAdjustment
from nomad_path_planning.history_recovery import (
    Trail, ProgressWindow, ReverseFollower, TrajectoryFollower,
    EntryGate, segment_free, distance)
from nomad_path_planning.recovery_supervisor import RecoverySupervisor
from test_continuous_escape import message


def scene():
    grid = dict(width=160, height=160, resolution=.1,
                origin_x=-8., origin_y=-8., data=[0]*25600)
    # Inflated wall appears after the old straight exit was selected.
    for i in range(len(grid['data'])):
        x, y = -8+(i%160+.5)*.1, -8+(i//160+.5)*.1
        if x >= .4 and -.8 < y < .8:
            grid['data'][i] = 100
    return grid, [(0.,0.), (0.,4.), (4.,4.)], [(-4.,0.,0.), (3.,0.,0.)]


def supervisor():
    grid, route, stem = scene()
    trail = Trail()
    for i in range(31):
        trail.record((-4.5+i*.15, 0., 0.), grid)
    publications, reports = [], []
    node = NS(mode='EXIT', pose=(0.,0.,0.), grid=grid, goal=(4.,4.), trail=trail,
        progress=ProgressWindow(), min_retreat=2., max_retreat=60., attempts=1,
        max_attempts=4, wheelbase=.72, max_steer=.4, offset=.36, speed=.2,
        exit_speed=.3, probe=EscapeProbe(), adjust_count=0, adjust_limit=3,
        adjust_follower=None, resume_follower=None, exit_blocked_since=None,
        exit_blocked_seconds=.3, gate_width=2., gate_ttl=120., gates=[],
        recovery_timeout=360., frame='odom', previous_time=None, health=True,
        started=1., global_path=message(route, 2.), last_probe=-math.inf,
        publish_gates=lambda:None, reference_pub=NS(publish=lambda msg:None),
        path_message=lambda p:p, publish_path=publications.append,
        report=reports.append)
    node.follower = ReverseFollower(list(reversed(stem)))
    node.exit_follower = TrajectoryFollower([(0.,0.,0.),(1.,0.,0.),(2.,0.,0.)], speed=.3)
    node.adjuster = ExitAdjustment(node.probe)
    node.hold = lambda reason:(publications.append([node.pose]), reports.append(reason))
    for name in ('begin', 'find_exit', 'find_adjusted_exit', 'adjustment_route',
                 'retry_exit', 'resume_retreat', 'trim_trail'):
        setattr(node, name, MethodType(getattr(RecoverySupervisor,name), node))
    return node, publications, reports


def tick(node, now):
    node.now = lambda:now
    node.health_at = node.pose_at = node.map_at = now
    RecoverySupervisor.tick(node)


def test_adjustment_requires_both_legs_and_changes_heading():
    grid, route, stem = scene()
    probe = EscapeProbe()
    assert probe.plan((0.,0.,0.), route, grid, stem) is None
    reverse, forward = ExitAdjustment(probe).plan((0.,0.,0.), route, grid, stem)
    assert reverse[-1][0] < -.5 and abs(reverse[-1][2]) > .05
    assert forward[-1][1] > 1.
    for leg in (reverse, forward):
        assert all(segment_free(grid,a,b) for a,b in zip(leg,leg[1:]))


@pytest.mark.parametrize('value', [-1,100])
def test_no_adjustment_through_unknown_or_blocked_rear(value):
    grid, route, stem = scene()
    for i in range(len(grid['data'])):
        if -8+(i%160+.5)*.1 < -.1:
            grid['data'][i] = value
    assert ExitAdjustment(EscapeProbe()).plan((0.,0.,0.),route,grid,stem) is None


def test_closed_loop_exit_blocked_reverse_turn_then_forward_exit():
    node, publications, reports = supervisor()
    phases, commands, reverse_distance = [], [], 0.
    # The stopped GPP route is intentionally older than 3s; only the current
    # collision map can authorize a maneuver, never the old route alone.
    for i in range(250):
        publications.clear()
        tick(node, 10.+i*.1)
        if not phases or phases[-1] != node.mode:
            phases.append(node.mode)
        if node.mode == 'FOLLOW':
            break
        rollout = publications[-1]
        if len(rollout) >= 2:
            old, new = node.pose, rollout[1]
            signed = (new[0]-old[0])*math.cos(old[2])+(new[1]-old[1])*math.sin(old[2])
            commands.append(1 if signed > 0 else -1)
            if signed < 0:
                reverse_distance += distance(old,new)
            assert segment_free(node.grid,old,new)
            node.pose = new
        else:
            commands.append(0)
    assert phases == ['EXIT','ADJUST_STOP','ADJUST_REVERSE','SWITCH','EXIT','FOLLOW'], (phases,reports[-5:])
    assert .5 < reverse_distance < .7
    assert node.adjust_count == 1 and node.attempts == 1
    assert node.goal == (4.,4.) and node.pose[1] > 1.
    # Gear changes must include an explicit stopped phase.
    first_forward = commands.index(1)
    assert commands[first_forward-1] == 0


def test_failed_adjustment_falls_back_to_connected_history():
    node, publications, _ = supervisor()
    node.adjuster.plan = lambda *args:None
    for i in range(4):
        tick(node,10.+i*.1)
    assert node.mode == 'REVERSE' and node.attempts == 2
    assert node.follower.reference[0] == node.pose
    assert node.follower.reference[-1][0] < -4.
    assert len(node.gates) == 1 and len(publications[-1]) == 1
    tick(node,10.4)
    assert len(publications[-1]) >= 2 and publications[-1][1][0] < 0.


def test_new_rear_obstacle_interrupts_adjustment_without_forward_fallback():
    node, publications, _ = supervisor()
    for i in range(10):
        tick(node,10.+i*.1)
        if node.mode == 'ADJUST_REVERSE':
            break
    assert node.mode == 'ADJUST_REVERSE'
    for i in range(len(node.grid['data'])):
        if -8+(i%160+.5)*.1 < -.1:
            node.grid['data'][i] = 100
    tick(node,11.)
    assert node.mode == 'REVERSE' and len(publications[-1]) == 1
    tick(node,11.1)
    assert node.mode == 'REVERSE' and len(publications[-1]) == 1


def test_bad_goal_or_gate_never_authorizes_exit_adjustment():
    node, _, _ = supervisor()
    node.goal = (5.,5.)
    assert node.adjustment_route(10.) is None
    grid, route, stem = scene()
    gate = EntryGate(0., .5, math.pi/2, 4.)
    assert ExitAdjustment(EscapeProbe()).plan((0.,0.,0.),route,grid,stem,[gate]) is None


def test_attempt_limit_and_stale_sensors_keep_priority_stop():
    node, publications, _ = supervisor()
    node.adjust_count = node.adjust_limit
    node.attempts = node.max_attempts
    for i in range(4):
        tick(node,10.+i*.1)
    assert node.mode == 'STOP' and len(publications[-1]) == 1
    node.mode = 'ADJUST_REVERSE'
    node.health = False
    tick(node,10.4)
    assert node.mode == 'ADJUST_REVERSE' and len(publications[-1]) == 1
