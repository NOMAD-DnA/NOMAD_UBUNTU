"""Compatibility import plus ROS-independent command lease gate."""
import math
from nomad_vehicle_model import Geometry


class CommandGate:
    """Independent wall/ROS leases; stale packets cannot renew a command."""
    def __init__(self, geometry=None, max_speed=.5, timeout=.35):
        self.geometry=geometry or Geometry()
        if not all(math.isfinite(v) and v>0 for v in (max_speed,timeout)):
            raise ValueError('Invalid gate limits')
        self.max_speed,self.timeout=max_speed,timeout
        self.latest=-math.inf
        self.command=None
        self.last_now=None
        self.last_progress=-math.inf
        self.clock_fault=False
        self.reason='waiting for command'

    def clear(self,reason):
        self.command=None;self.reason=reason

    def receive(self, speed, angle, stamp, lease, stop, frame, now, wall):
        if not all(map(math.isfinite,(speed,angle,stamp,lease,now,wall))):
            self.clear('non-finite command');return
        if stamp < self.latest:
            return
        if stamp == self.latest:
            if stop:self.clear('stop requested')
            return
        if frame!='base_link' or stamp<=0 or not 0<lease<=.5 or not -.05<=now-stamp<=lease:
            self.clear('invalid frame/stamp/lease');return
        self.latest=stamp
        if stop:
            self.clear('stop requested');return
        if abs(speed)>self.max_speed or abs(angle)>self.geometry.equivalent_limit:
            self.clear('command exceeds limits');return
        self.command=(speed,angle,stamp,lease,wall)
        self.reason='running'

    def sample(self, now, wall):
        if self.last_now is not None and now<self.last_now-.05:
            self.clock_fault=True
        if now!=self.last_now:
            self.last_progress=wall
        self.last_now=now
        if self.clock_fault:
            self.clear('clock reset; restart required')
        if self.command:
            v,a,stamp,lease,received=self.command
            if not -.05<=now-stamp<=lease or wall-received>self.timeout or wall-self.last_progress>.5:
                self.clear('command or clock timeout')
            else:
                return v,a
        return 0.,0.
