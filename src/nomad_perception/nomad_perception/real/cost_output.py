"""Live costs and capture-validated odom output for Planning; no file exports."""
from array import array
import json
from pathlib import Path
import numpy as np
from rclpy.clock import Clock,ClockType
from sensor_msgs.msg import Image
from .planning_costs import CostPolicy,build_cost_snapshot,ros_messages
from .planning_transport import map_qos,set_stamp,PlanningSnapshotPublisher


class CostOutput:
    def __init__(self,node,directory,semantic_provider,*,clock_verified=False,policy=None,readiness=None):
        self.node,self.directory,self.semantic_provider=node,Path(directory),semantic_provider
        self.policy=policy or CostPolicy();self.clock_verified=bool(clock_verified)
        self.state={}
        self.images=[node.create_publisher(Image,'/nomad/semantic_validation/'+name,map_qos())
                     for name in ('slope_score','semantic_score')]
        self.bundle_pub=None;self.planning=None
        try:
            from nomad_interfaces.msg import PlanningCostLayers
            self.bundle_pub=node.create_publisher(PlanningCostLayers,'/nomad/semantic_validation/cost_layers',map_qos())
        except ImportError:
            pass
        self.last_key=None;self.latest=None;self.enabled=True;self.closed=False
        self.canonical_published=False
        self.readiness=readiness
        if readiness is not None:self.planning=PlanningSnapshotPublisher(node,readiness=readiness)
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=node.create_timer(.5,self.tick,clock=self.wall_clock)
        self._status('Waiting for measured slope and semantic snapshots')

    def _status(self,reason,**extra):
        self.state=dict(reason=reason,
            planning_wire_type_available=self.bundle_pub is not None,
            canonical_planning_published=self.canonical_published,clock_confirmation_from_config=self.clock_verified,
            slope_policy='4*theta for 0<=theta<20; 100 for theta>=20; invalid=NaN',
            camera_support_policy=dict(source='distinct matched label capture stamps',
                full_count=3,full_span_s=.5,evidence_window_s=3.,
                hold_s=self.policy.support_hold_s,expire_s=self.policy.support_expire_s),
            feature_scale='score/100, independent slope_cost and terrain_cost',**extra)

    def _image(self,values,snapshot):
        msg=Image();msg.header.frame_id=snapshot['frame'];set_stamp(msg.header,snapshot['capture_ns'])
        msg.height,msg.width=values.shape;msg.encoding='32FC1';msg.step=msg.width*4
        msg.data=array('B',np.asarray(values,dtype='<f4').tobytes())
        return msg

    def export(self,snapshot):
        """Publish separate score diagnostics and keep the latest bundle in memory."""
        for publisher,key in zip(self.images,('slope_score','semantic_score')):
            publisher.publish(self._image(snapshot[key],snapshot))
        if self.bundle_pub is not None:
            _,_,layers,_=ros_messages(snapshot);self.bundle_pub.publish(layers)
        self.latest=snapshot
        self._status('Separate costs exported in '+snapshot['frame'],
            frame=snapshot['frame'],capture_ns=snapshot['capture_ns'],
            slope_known=int(np.isfinite(snapshot['slope_cost']).sum()),
            semantic_known=int(np.isfinite(snapshot['terrain_cost']).sum()),
            blocked_cells=int(np.count_nonzero(snapshot['global_costmap']==100)))

    def publish_prepared(self,snapshot,*,capture_pose_validated=False):
        """For the odom adapter: reuse timing/QoS and all-or-nothing validation."""
        if snapshot['frame']!='odom':
            raise ValueError('Measured odom-aligned input required; world cannot be relabelled')
        if self.planning is None:self.planning=PlanningSnapshotPublisher(self.node)
        messages=ros_messages(snapshot)
        accepted=self.planning.publish(*messages,capture_ns=snapshot['capture_ns'],
            navigation_validated=bool(capture_pose_validated and self.clock_verified and self.enabled and not self.closed))
        self.canonical_published=accepted
        self._status('Validated odom snapshot published' if accepted else self.planning.reason,
                     frame=snapshot['frame'],capture_ns=snapshot['capture_ns'])
        return accepted

    def tick(self):
        if self.closed:return
        path=self.directory/'terrain/live.json'
        if not self.enabled:
            if self.planning is not None:self.planning.invalidate('Inspection paused')
            self._status('Inspection paused');return
        if self.readiness is not None:
            ready,reason=self.readiness()
            if not ready:
                self.planning.invalidate(reason);self.canonical_published=False
                self._status(reason);return
        if not path.exists():return
        try:
            live=json.loads(path.read_text())
            if live.get('error') or not live.get('inspection_enabled',True):
                raise ValueError(live.get('error') or 'Inspection paused')
            now=self.node.get_clock().now().nanoseconds
            semantics=self.semantic_provider(now)
            # Same generation, including semantic expiration, must not restamp data.
            source=(live.get('capture_timing') or {}).get('sequence',live['timing']['updates'])
            semantic_key=tuple((r['ix'],r['iy'],r['cost'],r['last_semantic_ns'],r['camera_observed_ns'],
                                tuple(r.get('camera_capture_ns',[]))) for r in semantics['cells'])
            key=(source,semantic_key)
            if key==self.last_key:return
            snapshot=build_cost_snapshot(live,semantics,self.policy)
            self.export(snapshot);self.last_key=key
            if self.readiness is not None:self.publish_prepared(snapshot,capture_pose_validated=True)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            if self.planning is not None:self.planning.invalidate(str(exc))
            self._status(str(exc))

    def close(self):
        if self.closed:return
        self.tick();self.closed=True;self.enabled=False;self.canonical_published=False
        if self.planning is not None:self.planning.close()
        self.node.destroy_timer(self.timer)
        self._status('Cost output closed')
