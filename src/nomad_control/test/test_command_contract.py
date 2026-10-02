import math
from types import SimpleNamespace as NS, MethodType

import pytest
from builtin_interfaces.msg import Time
from nomad_interfaces.msg import DriveCommand
from nomad_control.controller import Controller, actuator_twist, centre_steering, command_error


def command(t=10., speed=.2, steer=.3, stop=False):
    msg = DriveCommand(target_speed=speed, target_steering_angle=steer, stop_requested=stop)
    msg.header.frame_id = 'base_link'
    ns = round(t*1e9)
    msg.header.stamp.sec,msg.header.stamp.nanosec = divmod(ns,1_000_000_000)
    msg.valid_for.nanosec = 300_000_000
    return msg


def fixture(monkeypatch):
    clock = NS(ros=10.,wall=100.)
    monkeypatch.setattr('nomad_control.controller.time.monotonic',lambda:clock.wall)
    outputs,states = [],[]
    node = NS(now=lambda:clock.ros, max_speed=.4,max_steer=.4,base_frame='base_link',
        wheelbase=.72,command_timeout=.35,command=None,command_wall=-math.inf,
        latest_command_stamp=-math.inf,speed=0.,steer=.12,speed_stamp=10.,steer_stamp=10.,
        speed_wall=100.,steer_wall=100.,last_sim_time=10.,last_clock_change=100.,clock_fault=False,
        get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=int(clock.ros)))),
        pub=NS(publish=outputs.append),state_pub=NS(publish=states.append),
        status_pub=NS(publish=lambda m:None))
    for name in ('on_command','tick'):
        setattr(node,name,MethodType(getattr(Controller,name),node))
    return clock,node,outputs,states


@pytest.mark.parametrize('speed',[-.2,.2])
@pytest.mark.parametrize('steer',[-.3,0.,.3])
def test_signed_speed_and_steering_curvature(speed,steer):
    output=actuator_twist(speed,steer,.72)
    assert output.linear.x==speed
    assert output.angular.z==pytest.approx(speed/.72*math.tan(steer))
    k=math.tan(steer)/.72
    left=math.atan(.72*k/(1.-.3*k))
    right=math.atan(.72*k/(1.+.3*k))
    assert centre_steering(left,right,.72,.60)==pytest.approx(steer)


def test_stop_overrides_same_tick_and_duplicates_do_not_renew(monkeypatch):
    clock,node,out,_=fixture(monkeypatch)
    node.on_command(command())
    node.tick()
    assert out[-1].linear.x==.2
    clock.wall+=.1
    node.on_command(command())
    assert node.command_wall==100.
    node.on_command(command(stop=True))
    node.tick()
    assert out[-1].linear.x==0.
    node.on_command(command())
    node.tick()
    assert out[-1].linear.x==0.


@pytest.mark.parametrize('kind',['lease','wall','measurement','paused','clock_reset'])
def test_expiry_and_feedback_loss_stop(kind,monkeypatch):
    clock,node,out,_=fixture(monkeypatch)
    node.on_command(command())
    node.tick()
    assert out[-1].linear.x==.2
    if kind=='lease': clock.ros+=.31
    if kind=='wall': clock.wall+=.36
    if kind=='measurement': node.steer_stamp=8.
    if kind=='paused': clock.wall+=.51
    if kind=='clock_reset': clock.ros=1.
    node.tick()
    assert out[-1].linear.x==0.
    if kind=='clock_reset': assert node.clock_fault


def test_actual_feedback_and_brake_before_reverse(monkeypatch):
    clock,node,out,states=fixture(monkeypatch)
    node.speed=.1
    node.on_command(command(speed=-.2,steer=-.3))
    node.tick()
    assert out[-1].linear.x==0.
    assert states[-1].speed==.1 and states[-1].steering_angle==.12
    node.speed=0.
    node.tick()
    assert out[-1].linear.x==-.2 and out[-1].angular.z>0.


@pytest.mark.parametrize('mutation',[
    lambda m:setattr(m,'target_speed',float('nan')),
    lambda m:setattr(m,'target_steering_angle',float('inf')),
    lambda m:setattr(m,'target_speed',.8),
    lambda m:setattr(m.header,'frame_id','odom'),
    lambda m:setattr(m.valid_for,'nanosec',0),
    lambda m:setattr(m.header.stamp,'sec',11),
])
def test_invalid_commands_are_rejected(mutation):
    msg=command()
    mutation(msg)
    assert command_error(msg,10.,.4,.4)


def test_delayed_packet_cannot_replace_newer_stop(monkeypatch):
    clock,node,out,_=fixture(monkeypatch)
    node.on_command(command(stop=True))
    node.on_command(command(t=9.9))
    node.tick()
    assert out[-1].linear.x==0.
