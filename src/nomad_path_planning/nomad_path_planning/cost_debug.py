"""Read-only views of the arrays used by GPP, never planning inputs."""
import copy
import json
import math
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

SCALES = dict(total=160., slope=10., proximity=32., cluster=64., observation=80., terrain=1., unknown_direction=8.)

def display_values(values, occupancy, limit):
    """0..99 is display-only scaled cost; 100 blocked, -1 unobserved."""
    a=np.asarray(values,dtype=float);c=np.asarray(occupancy)
    if a.shape!=c.shape or not math.isfinite(limit) or limit<=0:
        raise ValueError('invalid cost display geometry/scale')
    v=np.rint(99*np.clip(np.nan_to_num(a,nan=0.,posinf=limit)/limit,0.,1.)).astype(np.int8)
    v[(c>=80)|np.isposinf(a)]=100
    v[c<0]=-1
    return v.tolist()


def cost_rgb(values, occupancy, limit):
    """Green -> yellow -> red; hard blocked black; unknown grey."""
    a=np.asarray(values,dtype=float);c=np.asarray(occupancy)
    t=np.clip(np.nan_to_num(a,nan=0.,posinf=limit)/limit,0.,1.)
    rgb=np.column_stack([np.minimum(2*t,1)*255, np.minimum(2*(1-t),1)*220, np.full(t.shape,30)]).astype(np.uint8)
    rgb[(c>=80)|np.isposinf(a)]=[0,0,0]
    rgb[c<0]=[125,125,125]
    rgb[np.isnan(a)&(c>=0)&(c<80)]=[200,0,200]
    return rgb


def color_cloud(source, values, occupancy, limit):
    rgb=cost_rgb(values,occupancy,limit)
    yy,xx=np.indices((source.info.height,source.info.width))
    pts=np.empty(len(rgb),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    pts['x']=source.info.origin.position.x+(xx.ravel()+.5)*source.info.resolution
    pts['y']=source.info.origin.position.y+(yy.ravel()+.5)*source.info.resolution
    pts['z']=source.info.origin.position.z-.05
    pts['rgb']=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
    msg=PointCloud2();msg.header=copy.deepcopy(source.header);msg.height=1;msg.width=len(pts)
    msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32 if k!='rgb' else PointField.UINT32,count=1) for i,k in enumerate(['x','y','z','rgb'])]
    msg.is_bigendian=False;msg.point_step=16;msg.row_step=16*len(pts);msg.is_dense=True;msg.data=pts.tobytes()
    return msg


def legend_image(header):
    import cv2
    canvas=np.full((110,640,3),240,dtype=np.uint8)
    bar=cost_rgb(np.linspace(0,1,600),np.zeros(600),1.)
    canvas[30:55,20:620]=bar[None,:,:]
    for text,xy in [('LOW', (20,22)),('HIGH (display max)',(420,22)),('Black: blocked   Grey: unobserved',(20,78)),('Range: see layer name. Enable ONE cost layer.',(20,100))]:
        cv2.putText(canvas,text,xy,cv2.FONT_HERSHEY_SIMPLEX,.48,(15,15,15),1,cv2.LINE_AA)
    msg=Image();msg.header=copy.deepcopy(header);msg.height=110;msg.width=640;msg.encoding='rgb8';msg.step=640*3;msg.data=canvas.tobytes();return msg

class CostDebugPublisher:
    def __init__(self,node):
        self.enabled=node.declare_parameter('gpp.debug_costs.enabled',True).value
        self.hz=float(node.declare_parameter('gpp.debug_costs.max_hz',1.).value)
        self.scales=dict(SCALES,total=float(node.declare_parameter('gpp.debug_costs.display_max',160.).value))
        if not math.isfinite(self.hz) or self.hz<=0 or not math.isfinite(self.scales['total']) or self.scales['total']<=0:
            raise ValueError('invalid debug display rate/scale')
        self.last=-math.inf;self.maps={};self.raw={};self.colors={}
        if not self.enabled:return
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for k in self.scales:
            topic='/nomad/gpp_cost/'+k
            self.maps[k]=node.create_publisher(OccupancyGrid,topic,qos)
            self.raw[k]=node.create_publisher(Image,topic+'_raw',qos)
            self.colors[k]=node.create_publisher(PointCloud2,topic+'_color',qos)
        self.legend=node.create_publisher(Image,'/nomad/gpp_cost/legend',qos)
        self.metadata=node.create_publisher(String,'/nomad/gpp_cost/metadata',qos)

    def publish(self,source,grid,components,start):
        now=time.monotonic()
        if not self.enabled or now-self.last<1./self.hz:return
        if 'cell_weights' not in grid:return  # legacy has no separated multi terms
        arrays=dict(components,total=grid['cell_weights'])
        mask=np.asarray(grid['data'])>=80
        # Occupancy remains the original map so unknown differs from hard blocked.
        occupancy=np.asarray(source.data).copy();occupancy[mask & (occupancy>=0)]=100
        for k,limit in self.scales.items():
            if k not in arrays:continue
            a=np.asarray(arrays[k],dtype=np.float32).copy()
            if k=='total':a[mask]=np.inf
            msg=OccupancyGrid();msg.header=copy.deepcopy(source.header);msg.info=copy.deepcopy(source.info)
            msg.data=display_values(a,occupancy,limit);self.maps[k].publish(msg)
            raw=Image();raw.header=copy.deepcopy(source.header);raw.height=source.info.height;raw.width=source.info.width
            raw.encoding='32FC1';raw.is_bigendian=0;raw.step=raw.width*4;raw.data=a.astype('<f4').tobytes();self.raw[k].publish(raw)
            self.colors[k].publish(color_cloud(source,a,occupancy,limit))
        self.legend.publish(legend_image(source.header))
        meta=String();meta.data=json.dumps(dict(capture_stamp=source.header.stamp.sec+source.header.stamp.nanosec*1e-9,
            start=list(start),display_max=self.scales,available=[k for k in self.scales if k in arrays],
            meaning='Per-metre weighted cell costs, not accumulated path cost. display 0..99 scaled, 100 blocked, -1 unknown; raw images preserve floats.',
            exclusions='Corridor goal/heading/visit scores, motion constraints, gates and near-path observation rejection are not scalar cell costs.',
            retained_path_note='Current evaluated map, not necessarily the map on which a retained path was first selected.'))
        self.metadata.publish(meta);self.last=now
