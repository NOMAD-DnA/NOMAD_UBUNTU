import math
from types import SimpleNamespace as NS, MethodType

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pytest
from nomad_path_planning.escape_probe import EscapeProbe
from nomad_path_planning.history_recovery import Trail, ProgressWindow, EntryGate, segment_free, distance
from nomad_path_planning.dstar_lite import DStarLite
from nomad_path_planning.recovery_supervisor import RecoverySupervisor


def corridor():
    # Already-inflated planning grid. A 14m dead end with an opening before x=5.
    r, w, h, ox, oy = .2, 110, 80, -3., -6.
    data = [0]*(w*h)
    for cy in range(h):
        for cx in range(w):
            x,y = ox+(cx+.5)*r, oy+(cy+.5)*r
            if (x>=5. and .9<=abs(y)<=2.) or (14.5<=x<=16 and abs(y)<=2.):
                data[cy*w+cx] = 100
    return dict(width=w,height=h,resolution=r,origin_x=ox,origin_y=oy,data=data)


def route(grid, start, goal=(8.,5.), gates=()):
    r, ox, oy = grid['resolution'],grid['origin_x'],grid['origin_y']
    cell = lambda p:(math.floor((p[0]-ox)/r), math.floor((p[1]-oy)/r))
    world = lambda c:(ox+(c[0]+.5)*r, oy+(c[1]+.5)*r)
    gpp = DStarLite(grid['width'],grid['height'],grid['data'],cell(start),cell(goal))
    gpp.edge_filter = lambda a,b: not any(g.blocks(world(a),world(b)) for g in gates)
    assert gpp.compute_shortest_path()
    return [world(c) for c in gpp.extract_path()]


def message(points, now):
    msg = Path()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec = int(now)
    msg.header.stamp.nanosec = round((now-int(now))*1e9)
    for x,y in points:
        p = PoseStamped()
        p.pose.position.x,p.pose.position.y,p.pose.orientation.w = float(x),float(y),1.
        msg.poses.append(p)
    return msg


def test_probe_requires_drivable_departure_not_just_any_free_rollout():
    grid = corridor()
    history = [(i*.15,0.,0.) for i in range(95)]
    gates = [EntryGate(13.,0.,0.)]
    probe = EscapeProbe()
    for x in [13.,10.,7.,5.,4.]:
        assert probe.plan((x,0.,0.),route(grid,(x,0.),gates=gates),grid,history,gates) is None
    exit_path = probe.plan((3.,0.,0.),route(grid,(3.,0.),gates=gates),grid,history,gates)
    assert exit_path is not None and exit_path[-1][1] > 1.
    assert all(segment_free(grid,a,b) for a,b in zip(exit_path,exit_path[1:]))
    # All-known empty road still isn't a new branch if it follows the failed stem.
    grid['data'] = [0]*len(grid['data'])
    assert probe.plan((3.,0.,0.),[(3.,0.),(7.,0.)],grid,history) is None


@pytest.mark.parametrize('value', [-1,100])
def test_exit_probe_rejects_unknown_or_new_obstacle(value):
    grid = corridor()
    history = [(i*.15,0.,0.) for i in range(95)]
    candidate_route = route(grid,(3.,0.))
    assert EscapeProbe().plan((3.,0.,0.),candidate_route,grid,history)
    for i in range(len(grid['data'])):
        y = grid['origin_y']+(i//grid['width']+.5)*grid['resolution']
        if y >= .5:
            grid['data'][i] = value
    assert EscapeProbe().plan((3.,0.,0.),candidate_route,grid,history) is None


def test_continuous_reverse_to_branch_then_verified_forward_exit():
    grid = corridor()
    trail = Trail()
    for i in range(94):
        trail.record((i*.15,0.,0.),grid)
    pose = trail.points[-1].pose
    publications, stops = [], []
    node = NS(mode='FOLLOW',pose=pose,grid=grid,goal=(8.,5.),trail=trail,
        progress=ProgressWindow(), min_retreat=2.,max_retreat=60.,attempts=0,max_attempts=4,
        wheelbase=.72,max_steer=.4,offset=.36,speed=.2,exit_speed=.3,probe=EscapeProbe(),
        gate_width=2.,gate_ttl=120.,gates=[],recovery_timeout=360.,frame='odom',
        previous_time=None,health=True,publish_gates=lambda:None,
        reference_pub=NS(publish=lambda msg:None),path_message=lambda p:p,
        publish_path=publications.append, report=lambda msg:None,
        hold=lambda msg:stops.append(msg))
    node.find_exit = MethodType(RecoverySupervisor.find_exit,node)
    RecoverySupervisor.begin(node,1.)
    assert node.follower.reference[-1][0] < .2  # No 2m / 8m intermediate endpoint.
    phases, switched_x, reverse_distance = [], None, 0.
    for i in range(1,1100):
        now = 1.+i*.1
        node.now = lambda: now
        node.health_at = node.pose_at = node.map_at = now
        if i%10 == 1:
            points = route(grid,node.pose,gates=[g for g,_ in node.gates])
            node.global_path = message(points,now)
        publications.clear()
        RecoverySupervisor.tick(node)
        if not phases or phases[-1] != node.mode:
            phases.append(node.mode)
        if node.mode == 'SWITCH' and switched_x is None:
            switched_x = node.pose[0]
            # Only the initial brake is allowed before a real exit has been found.
            assert all('START' in s or 'SWITCH' in s for s in stops)
        if node.mode == 'FOLLOW':
            break
        if publications and len(publications[-1]) >= 2:
            nxt = publications[-1][1]
            if node.mode == 'REVERSE':
                reverse_distance += distance(node.pose,nxt)
            node.pose = nxt
    assert phases == ['REVERSE','SWITCH','EXIT','FOLLOW'], phases
    assert switched_x < 5. and reverse_distance > 9.
    assert node.pose[1] > 1. and node.goal == (8.,5.) and node.attempts == 1
