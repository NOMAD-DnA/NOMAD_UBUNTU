#!/usr/bin/env python3
"""Low-speed front-drive/rear-steer geometry. No ROS dependencies.

v is longitudinal speed at the non-steering front axle; equivalent steering
alpha is positive for left yaw during forward motion. Physical rear angles
are negative in that case. Both are radians, never servo-normalized values.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Geometry:
    wheelbase: float = .72
    track: float = .60
    radius: float = .15
    joint_limit: float = .55
    steering_axle: str = "rear"

    def __post_init__(self):
        if self.steering_axle not in ('front','rear'):
            raise ValueError('steering_axle must be front or rear')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.wheelbase, self.track, self.radius, self.joint_limit)):
            raise ValueError('Geometry must be finite and positive')
        if self.joint_limit >= math.pi/2:
            raise ValueError('Joint limit must be below pi/2')

    @property
    def equivalent_limit(self):
        """Limit the inner rear wheel, rather than clipping each wheel later."""
        k = math.tan(self.joint_limit)/(self.wheelbase + self.track*.5*math.tan(self.joint_limit))
        return math.atan(self.wheelbase*k)

    def mix(self, speed, equivalent_angle):
        if not all(math.isfinite(v) for v in (speed, equivalent_angle)):
            raise ValueError('Non-finite command')
        if abs(equivalent_angle) > self.equivalent_limit+1e-9:
            raise ValueError('Equivalent steering exceeds physical inner-wheel limit')
        curvature = math.tan(equivalent_angle)/self.wheelbase
        yaw_rate = speed*curvature
        sign = -1 if self.steering_axle == "rear" else 1
        left = math.atan2(sign*self.wheelbase*curvature, 1-self.track*.5*curvature)
        right = math.atan2(sign*self.wheelbase*curvature, 1+self.track*.5*curvature)
        return ((speed-yaw_rate*self.track*.5)/self.radius,
                (speed+yaw_rate*self.track*.5)/self.radius, left, right)

    def equivalent_from_joints(self, left, right):
        if not all(math.isfinite(v) and abs(v) <= self.joint_limit+1e-6 for v in (left,right)):
            raise ValueError('Invalid measured rear angle')
        return centre_steering(left, right, self.wheelbase, self.track, self.steering_axle)

    @property
    def reference_offset(self):
        return (-1 if self.steering_axle == 'rear' else 1)*self.wheelbase*.5

    def advance(self, pose, speed, equivalent_angle, dt):
        """Integrate front axle exactly and return the same body-centre pose."""
        self.mix(speed,equivalent_angle)
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('Invalid integration interval')
        x,y,yaw=pose
        if not all(map(math.isfinite,pose)):
            raise ValueError('Invalid pose')
        return advance(pose, math.tan(equivalent_angle)/self.wheelbase,
                       speed*dt, self.reference_offset)


def advance(pose, curvature, distance, reference_offset):
    """Exact body pose integration; signed offset = body X minus reference-axle X."""
    if not all(map(math.isfinite, (*pose, curvature, distance, reference_offset))):
        raise ValueError('Non-finite motion')
    x,y,yaw=pose
    x-=reference_offset*math.cos(yaw); y-=reference_offset*math.sin(yaw)
    end=yaw+curvature*distance
    if abs(curvature)<1e-10:
        x+=distance*math.cos(yaw); y+=distance*math.sin(yaw)
    else:
        x+=(math.sin(end)-math.sin(yaw))/curvature
        y+=(math.cos(yaw)-math.cos(end))/curvature
    return (x+reference_offset*math.cos(end), y+reference_offset*math.sin(end),
            math.atan2(math.sin(end),math.cos(end)))


def centre_steering(left, right, wheelbase, track, steering_axle='front'):
    sign=-1 if steering_axle=='rear' else 1
    tl,tr=sign*math.tan(left),sign*math.tan(right)
    curvature=.5*(tl/(wheelbase+.5*track*tl)+tr/(wheelbase-.5*track*tr))
    return math.atan(wheelbase*curvature)


def profile(values):
    """Validate one resolved ROS parameter dictionary, reject contradictory geometry."""
    layout=values.get('drive_layout','front_drive_rear_steer')
    if layout not in ('front_drive_rear_steer','rear_drive_front_steer'):
        raise ValueError('Unknown drive_layout')
    axle='rear' if layout=='front_drive_rear_steer' else 'front'
    g=Geometry(float(values.get('wheelbase',.72)),float(values.get('track',.6)),
               float(values.get('wheel_radius',.15)),float(values.get('joint_limit',.55)),axle)
    if values.get('steering_axle',axle)!=axle:
        raise ValueError('drive_layout and steering_axle disagree')
    if abs(float(values.get('reference_offset',g.reference_offset))-g.reference_offset)>1e-8:
        raise ValueError('reference_offset must match non-steering axle: '+str(g.reference_offset))
    if not 0<float(values.get('max_steer',.4))<=g.equivalent_limit:
        raise ValueError('max_steer exceeds inner-wheel joint limit')
    return g


def load_profile(path):
    from pathlib import Path
    import yaml
    return (yaml.safe_load(Path(path).read_text()) or {})['/**']['ros__parameters']
