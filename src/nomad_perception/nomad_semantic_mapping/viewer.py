"""Read and reopen a semantic archive without Gazebo; --check creates no ROS node."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from .archive import load_snapshot,resolve_snapshot

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--snapshot',type=Path);p.add_argument('--check',action='store_true')
    args,ros=p.parse_known_args(sys.argv[1:] if argv is None else argv)
    if args.check:
        if args.snapshot is None:p.error('--check requires --snapshot')
        path=resolve_snapshot(args.snapshot);_,m=load_snapshot(path);print(json.dumps({'snapshot':str(path),**m},ensure_ascii=False,indent=2));return 0
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock,ClockType
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import TransformStamped,PoseStamped
    from nav_msgs.msg import OccupancyGrid,Path as RosPath
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from tf2_ros import StaticTransformBroadcaster
    class Viewer(Node):
        def __init__(self):
            super().__init__('nomad_semantic_saved_viewer');self.set_parameters([Parameter('use_sim_time',value=False)])
            directory=self.declare_parameter('session_directory','').value
            if args.snapshot is None and not directory:raise ValueError('session_directory is required')
            self.path=resolve_snapshot(args.snapshot or Path(directory));self.a,self.m=load_snapshot(self.path)
            self.clock=Clock(clock_type=ClockType.SYSTEM_TIME);qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.tf=StaticTransformBroadcaster(self);t=TransformStamped();t.header.frame_id='odom';t.header.stamp=self.clock.now().to_msg();t.child_frame_id='semantic_saved_reference';t.transform.rotation.w=1.;self.tf.sendTransform(t)
            self.cost=self.create_publisher(OccupancyGrid,'/semantic_mapping/costmap',qos)
            self.image_pubs={k:self.create_publisher(Image,'/semantic_mapping/'+topic,qos) for k,topic in [('representative_class','representative_class'),('semantic_raw','raw_cost'),('semantic_adjusted','adjusted_cost')]}
            self.path_pub=self.create_publisher(RosPath,'/semantic_mapping/trajectory',qos);self.vehicle_pub=self.create_publisher(Marker,'/semantic_mapping/vehicle',qos);self.meta=self.create_publisher(String,'/semantic_mapping/metadata',qos)
            self.publish()
        def publish(self):
            c=OccupancyGrid();c.header.frame_id='odom';c.header.stamp=self.clock.now().to_msg();c.info.resolution=float(self.m['resolution_m']);c.info.height,c.info.width=self.a['cost'].shape
            c.info.origin.position.x=float(self.m['origin_x']);c.info.origin.position.y=float(self.m['origin_y']);c.info.origin.orientation.w=1.;c.data=self.a['cost'].ravel().tolist();self.cost.publish(c)
            for k,pub in self.image_pubs.items():
                v=self.a[k];dtype='<i2' if k=='representative_class' else '<f4';im=Image();im.header=c.header;im.height,im.width=v.shape;im.encoding='16SC1' if k=='representative_class' else '32FC1';im.step=im.width*np.dtype(dtype).itemsize;im.data=v.astype(dtype,copy=False).tobytes();pub.publish(im)
            path=RosPath();path.header=c.header
            for row in self.a['trajectory']:
                pose=PoseStamped();pose.header=c.header;pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,row[1:4]);q=row[4:]/np.linalg.norm(row[4:]);pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q);path.poses.append(pose)
            self.path_pub.publish(path);v=Marker();v.header=c.header;v.ns='saved_vehicle';v.id=0;v.type=Marker.ARROW;v.action=Marker.ADD if path.poses else Marker.DELETE;v.pose.orientation.w=1.
            if path.poses:v.pose=path.poses[-1].pose;v.scale.x,v.scale.y,v.scale.z=1.1,.3,.3;v.color.r,v.color.g,v.color.b,v.color.a=.1,.9,1.,1.
            self.vehicle_pub.publish(v);self.meta.publish(String(data=json.dumps({**self.m,'viewer_snapshot':str(self.path),'display_time':'wall clock; captured time remains in archive'},ensure_ascii=False,allow_nan=False)))
    rclpy.init(args=ros);node=None
    try:node=Viewer();rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0
