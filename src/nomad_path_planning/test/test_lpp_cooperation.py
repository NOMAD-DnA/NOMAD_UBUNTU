from dataclasses import replace
import pytest
from nomad_path_planning.lpp_cooperation import Cooperation, Progress, AVOIDING, TRACKING, BLOCKED


def fixture():
    grid = dict(width=100,height=100,resolution=.1,origin_x=0.,origin_y=0.,data=[0]*10000)
    reference = [(2+i*.1,5.,0.) for i in range(61)]
    start = (2.,4.,0.)  # intentional lateral deviation, not a GPP failure
    path = tuple((2+i*.1,4.,0.) for i in range(9))
    p = Progress(10.,(9,0),AVOIDING,False,40,10.,path,.4,0.,'odom')
    args = dict(reference_id=(9,0),reference=reference,start=start,grid=grid,frame='odom',now=10.,wall=20.)
    return Cooperation(),p,args


def check(c,p,args):
    c.receive(p,args['wall'])
    return c.evaluate(**args)


def test_legacy_no_publisher_unchanged():
    c,p,a=fixture(); assert c.evaluate(**a)=='legacy'
    assert check(c,replace(p,state=TRACKING),a)=='legacy'


def test_blocked_reference_prefix_can_hold_but_no_map_edits():
    c,p,a=fixture(); a['grid']['data'][50*100+40]=100
    before=list(a['grid']['data'])
    assert check(c,p,a)=='hold' and c.active
    assert a['grid']['data']==before


@pytest.mark.parametrize('value',[-1,80,100])
def test_blocked_or_unknown_rejoin_suffix_forces_replan(value):
    c,p,a=fixture();a['grid']['data'][50*100+70]=value
    assert check(c,p,a)=='replan' and c.reason=='rejoin_suffix_blocked_or_unknown'


@pytest.mark.parametrize('value',[-1,80,100])
def test_local_motion_still_checked(value):
    c,p,a=fixture();a['grid']['data'][40*100+25]=value
    assert check(c,p,a)=='replan' and c.reason=='local_motion_invalid'


@pytest.mark.parametrize('change',[
    dict(state=BLOCKED),dict(request_replan=True),dict(state=99),
    dict(rejoin_index=1000),dict(rejoin_index=-1),dict(speed=.6),
    dict(steering=.5),dict(speed=float('nan')),dict(motion_stamp=8.),
    dict(frame='map'),dict(path=((2.,4.,0.),(2.,4.,1.))),
    dict(path=((2.,4.,0.),(2.,4.1,0.))), # lateral slide
    dict(path=((2.,4.,0.),(1.9,4.,0.))), # reverse
    dict(path=((2.,4.,0.),(2.1,4.,float('nan')))),
])
def test_invalid_contract_does_not_grant_hold(change):
    c,p,a=fixture();assert check(c,replace(p,**change),a)=='replan'


def test_status_expiry_replay_and_legacy_failure():
    c,p,a=fixture();assert check(c,p,a)=='hold'
    a.update(now=10.6,wall=20.6);c.receive(p,a['wall'])
    assert c.evaluate(**a)=='replan'
    c,p,a=fixture();assert check(c,p,a)=='hold'
    a.update(wall=21.1);assert c.evaluate(**a)=='replan'
    c,p,a=fixture();a['legacy_failed']=True
    assert check(c,p,a)=='replan'


def test_reference_and_mission_reset_and_completion():
    c,p,a=fixture();assert check(c,replace(p,reference=(8,0)),a)=='legacy'
    assert check(c,replace(p,stamp=10.1),dict(a,now=10.1))=='hold'
    assert c.evaluate(**dict(a,reference_id=(10,0)))=='replan'
    c,p,a=fixture();assert check(c,p,a)=='hold'
    assert check(c,replace(p,stamp=10.1,state=TRACKING),dict(a,now=10.1))=='replan'
    c.reset();assert c.evaluate(**a)=='legacy'


def test_no_progress_cannot_be_extended_by_heartbeats():
    c,p,a=fixture();assert check(c,p,a)=='hold'
    for dt in range(1,5):
        assert check(c,replace(p,stamp=10.+dt,motion_stamp=10.+dt),dict(a,now=10.+dt,wall=20.+dt))=='hold'
    assert check(c,replace(p,stamp=15.,motion_stamp=15.),dict(a,now=15.,wall=25.))=='replan'
    assert c.reason=='avoidance_no_progress'


def test_actual_progress_renews_stall_timer_but_not_episode_budget():
    c,p,a=fixture();c.max_duration=6.
    assert check(c,p,a)=='hold'
    for dt in range(1,6):
        start=(2.+.2*dt,4.,0.); path=tuple((x+.2*dt,y,yaw) for x,y,yaw in p.path)
        assert check(c,replace(p,stamp=10.+dt,motion_stamp=10.+dt,path=path),dict(a,start=start,now=10.+dt,wall=20.+dt))=='hold'
    assert check(c,replace(p,stamp=16.,motion_stamp=16.,path=path),dict(a,start=start,now=16.,wall=26.))=='replan'
    assert c.reason=='avoidance_duration_limit'


def test_clock_reset_and_deviation_bound():
    c,p,a=fixture();assert check(c,p,a)=='hold'
    assert c.evaluate(**dict(a,now=1.))=='replan'
    c,p,a=fixture();a['start']=(2.,1.,0.)
    assert check(c,p,a)=='replan' and c.reason=='deviation_limit'


@pytest.mark.parametrize('steer',[-.4,.4])
def test_existing_vehicle_model_turns_are_accepted_at_steering_limit(steer):
    from nomad_path_planning.lpp.rollout import AckermannRollout
    c,p,a=fixture()
    path=tuple(AckermannRollout(speed=.4,horizon=.8).simulate(a['start'],steer))
    assert check(c,replace(p,path=path,steering=steer),a)=='hold'
