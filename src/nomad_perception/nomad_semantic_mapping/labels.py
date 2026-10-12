"""Latched cost colors, categorical class colors and representative-ID labels."""
import time,math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from .palette import PALETTE,cost_rgb

def identity(msg):
    h=msg.header;return h.stamp.sec,h.stamp.nanosec,h.frame_id

def build_displays(cost,classes,text_height=.06):
    values=np.asarray(cost.data,dtype=np.int8).reshape(cost.info.height,cost.info.width)
    classes=np.asarray(classes,dtype=np.int16)
    if classes.shape!=values.shape:raise ValueError('Cost/class grid shapes differ')
    if np.any((classes<-1)|(classes>=64)):raise ValueError('Invalid original GOOSE class ID')
    rows,cols=np.nonzero(values>=0);res=cost.info.resolution
    def cubes(namespace):
        m=Marker();m.header=cost.header;m.ns=namespace;m.id=0;m.type=Marker.CUBE_LIST;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.scale.x=m.scale.y=float(res*.98);m.scale.z=.004;m.color.a=1.
        return m
    costs=cubes('semantic_cost');categorical=cubes('representative_class')
    labels=MarkerArray();reset=Marker();reset.header=cost.header;reset.action=Marker.DELETEALL;labels.markers.append(reset)
    for i,(y,x) in enumerate(zip(rows,cols)):
        px=float(cost.info.origin.position.x+(x+.5)*res);py=float(cost.info.origin.position.y+(y+.5)*res)
        cls=int(classes[y,x]);rgb=cost_rgb(values[y,x]);crgb=PALETTE[cls] if cls>=0 else [.6,.6,.6]
        costs.points.append(Point(x=px,y=py,z=0.));costs.colors.append(ColorRGBA(r=float(rgb[0]),g=float(rgb[1]),b=float(rgb[2]),a=1.))
        categorical.points.append(Point(x=px,y=py,z=.006));categorical.colors.append(ColorRGBA(r=float(crgb[0]),g=float(crgb[1]),b=float(crgb[2]),a=1.))
        m=Marker();m.header=cost.header;m.ns='representative_class_id';m.id=i;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
        m.pose.position.x=px;m.pose.position.y=py;m.pose.position.z=.06;m.pose.orientation.w=1.
        m.scale.z=float(text_height);shade=1. if values[y,x]>=90 else .03
        m.color.r=m.color.g=m.color.b=shade;m.color.a=1.;m.text=str(cls) if cls>=0 else 'N/A';labels.markers.append(m)
    return costs,categorical,labels

class SemanticLabels(Node):
    def __init__(self):
        super().__init__('nomad_semantic_labels')
        self.height=float(self.declare_parameter('text_height_m',.06).value)
        period=float(self.declare_parameter('refresh_period_s',1.).value)
        if not math.isfinite(period) or period<=0 or not math.isfinite(self.height) or self.height<=0:raise ValueError('Invalid display settings')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costs={};self.classes={};self.ready=None;self.dirty=False
        self.cost_pub=self.create_publisher(Marker,'/semantic_mapping/cost_cells',qos)
        self.class_pub=self.create_publisher(Marker,'/semantic_mapping/class_colors',qos)
        self.label_pub=self.create_publisher(MarkerArray,'/semantic_mapping/class_labels',qos)
        self.create_subscription(OccupancyGrid,'/semantic_mapping/costmap',self.on_cost,qos)
        self.create_subscription(Image,'/semantic_mapping/representative_class',self.on_class,qos)
        self.create_timer(period,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_cost(self,msg):
        key=identity(msg);self.costs[key]=msg;self.pair(key)
    def on_class(self,msg):
        if msg.encoding!='16SC1' or msg.step!=msg.width*2 or len(msg.data)!=msg.height*msg.step:return
        v=np.frombuffer(bytes(msg.data),dtype='>i2' if msg.is_bigendian else '<i2').reshape(msg.height,msg.width).copy()
        if np.any((v<-1)|(v>=64)):return
        key=identity(msg);self.classes[key]=v;self.pair(key)
    def pair(self,key):
        if key in self.costs and key in self.classes:
            cost=self.costs[key];cls=self.classes[key]
            if cls.shape!=(cost.info.height,cost.info.width):return
            if self.ready is None or key>=self.ready[0]:self.ready=(key,cost,cls);self.dirty=True
        for cache in [self.costs,self.classes]:
            for old in sorted(cache)[:-3]:cache.pop(old,None)
    def publish(self):
        if self.dirty and self.ready:
            _,cost,cls=self.ready
            c,k,l=build_displays(cost,cls,self.height);self.cost_pub.publish(c);self.class_pub.publish(k);self.label_pub.publish(l);self.dirty=False

def main(args=None):
    rclpy.init(args=args);node=None
    try:node=SemanticLabels();rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
