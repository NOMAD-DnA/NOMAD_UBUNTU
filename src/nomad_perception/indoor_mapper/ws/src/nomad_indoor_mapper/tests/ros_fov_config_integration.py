#!/usr/bin/env python3
"""Synthetic private-domain verification of the installed indoor ray mapper. No real sensors."""
import json,math,os,struct,subprocess,tempfile,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped,Point32
from sensor_msgs.msg import LaserScan,PointCloud,ChannelFloat32
from nav_msgs.msg import Odometry,OccupancyGrid
from nomad_interfaces.msg import ModuleStatus,PlanningCostLayers
from rosgraph_msgs.msg import Clock
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
ROOT=Path(__file__).resolve().parents[4]
PKG=Path(__file__).resolve().parents[1]
HALF=float(os.environ.get("TEST_HALF_FOV", "55.25"))
checks=[]
def check(ok,name):
 if not ok:raise AssertionError(name)
 checks.append(name)
def f32(x):return struct.unpack('<f',struct.pack('<f',x))[0]
def stamp(h):return h.stamp.sec*10**9+h.stamp.nanosec
class Harness(Node):
 def __init__(self,imu_frame):
  super().__init__('indoor_ray160_synthetic_verification')
  self.clock=self.create_publisher(Clock,'/clock',10)
  self.loc=self.create_publisher(ModuleStatus,'/nomad/localization/status',10)
  self.odom=self.create_publisher(Odometry,'/nomad/localization/odometry',10)
  self.scan=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
  self.cloud=self.create_publisher(PointCloud,'/point_cloud',qos_profile_sensor_data)
  q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.bundle=None;self.local=None;self.state=None;self.t=100*10**9;self.phase='far';self.emit=True;self.last_scan=0.
  self.create_subscription(PlanningCostLayers,'/nomad/planning_cost_layers',lambda m:setattr(self,'bundle',m),q)
  self.create_subscription(OccupancyGrid,'/nomad/navigation_local_costmap',lambda m:setattr(self,'local',m),q)
  self.create_subscription(ModuleStatus,'/nomad/perception/status',lambda m:setattr(self,'state',m),10)
  self.tf=StaticTransformBroadcaster(self);m=TransformStamped();m.header.frame_id='base_link';m.child_frame_id=imu_frame;m.transform.rotation.w=1.;self.tf.sendTransform(m)
  self.sent=set();self.create_timer(.02,self.inputs)
 def inputs(self):
  self.t+=20_000_000;clock=Clock();clock.clock.sec=self.t//10**9;clock.clock.nanosec=self.t%10**9;self.clock.publish(clock)
  t=self.t-20_000_000;s=ModuleStatus();s.header.frame_id='odom';s.header.stamp.sec=t//10**9;s.header.stamp.nanosec=t%10**9;s.state=ModuleStatus.READY;s.valid_for.sec=1;self.loc.publish(s)
  odom=Odometry();odom.header=s.header;odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.;odom.pose.pose.position.z=100.;self.odom.publish(odom)
  if not self.emit or time.monotonic()-self.last_scan<.1:return
  self.last_scan=time.monotonic()
  rays={'far':[(2.336088,.0625,0.),(2.336088,.0625,0.)],
        'inside':[(1.5625,.0625,-.8),(1.5625,.0625,-.8)],
        'both':[(2.336088,.0625,0.),(1.5625,.0625,-.8)],
        'behind':[(-1.5625,.0625,-.8),(-1.5625,.0625,-.8)],
        'outside_angle':[(1.9*math.cos(math.radians(95)),1.9*math.sin(math.radians(95)),0.)]*2,
        'hold':[(.5,1.6,-.8),(.5,1.6,-.8)]}[self.phase]
  cloud=PointCloud();cloud.header=s.header;cloud.header.frame_id='g2_laser'
  cloud.points=[Point32(x=f32(x),y=f32(y),z=f32(z)) for x,y,z in rays]
  cloud.channels=[ChannelFloat32(name='stamps',values=[0.,.001])]
  scan=LaserScan();scan.header=cloud.header;scan.angle_min=f32(-math.pi);scan.angle_increment=f32(.001);scan.angle_max=f32(math.pi);scan.range_min=.01;scan.range_max=20.
  ranges=[math.inf]*6287
  for p in cloud.points:
   index=math.ceil((math.atan2(p.y,p.x)-scan.angle_min)/scan.angle_increment);ranges[index]=f32(math.hypot(p.x,p.y))
  scan.ranges=ranges;self.sent.add(t+1_000_000);self.scan.publish(scan);self.cloud.publish(cloud)
 def index(self,x,y):
  if self.bundle is None:return None
  info=self.bundle.grid.info;c=math.floor((x-info.origin.position.x)/info.resolution);r=math.floor((y-info.origin.position.y)/info.resolution)
  return r*info.width+c if 0<=c<info.width and 0<=r<info.height else None
 def value(self,x=1.5625,y=.0625):
  i=self.index(x,y);return None if i is None else self.bundle.grid.data[i]
 def wait(self,condition,timeout=12.):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.02)
   if condition():return
  raise AssertionError('Timed out waiting for phase '+self.phase)
 def phase_wait(self,name,predicate):
  self.phase=name;after=self.t
  self.wait(lambda:self.bundle is not None and stamp(self.bundle.grid.header)>=after and predicate())
def main():
 config=json.loads((ROOT/'ws/active_config/indoor.json').read_text())
 # Synthetic world calibration only. Actual stored hardware calibration is never changed.
 config['T_imu_from_lidar']=[[int(r==c) for c in range(4)] for r in range(4)]
 config['camera_forward_axis_imu']=[1.,0.,0.];config['moving_free_region']['enabled']=False
 config['initialization_stationary_hold_s']=0.;config['cost_area']='map_window'
 rclpy.init();h=Harness(config['imu_frame']);p=None
 try:
  config['lidar_half_fov_deg']=HALF
  with tempfile.TemporaryDirectory(prefix='ray160_test_',dir=str(ROOT/'tmp')) as temp:
   path=Path(temp)/'indoor.json';path.write_text(json.dumps(config))
   with (ROOT/('fov_config_ros_'+str(HALF)+'_mapper.log')).open('w') as log:
    binary=ROOT/'ws/install/nomad_indoor_mapper/lib/nomad_indoor_mapper/nomad_indoor_mapper'
    p=subprocess.Popen([str(binary),'--indoor-config',str(path),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT)
    h.wait(lambda:h.value(1.8125)==0 and h.state is not None and h.state.state==ModuleStatus.READY)
    check(h.value(1.8125)==0,'Ray to ground beyond2m creates previously unobserved paved cells within2m')
    check(h.value(2.3125)==-1,'Measured endpoint outside2m does not create an obstacle')
    check(h.value(2.0625)==-1,'Projected path is clipped to vehicle2m disk')
    check(abs(h.bundle.grid.info.resolution-.125)<1e-8,'12.5cm grid retained')
    names={name for name,_ in h.get_subscriber_names_and_types_by_node('nomad_indoor_mapper','/')}
    check(not any(n.startswith('/oak/') or n.startswith('/zed/') or n.startswith('/perception/') for n in names),'No camera or semantic subscriptions')
    check(any(abs(stamp(h.bundle.grid.header)-s)<=2 for s in h.sent),'Grid capture uses actual per-return acquisition time')
    h.phase_wait('inside',lambda:h.value()==100)
    i=h.index(1.5625,.0625);check(math.isnan(h.bundle.slope_cost[i]) and h.bundle.terrain_cost[i]==1.,'Negative-Z endpoint is obstacle with slopeNaN and final100')
    h.phase_wait('both',lambda:h.value()==100)
    check(h.value()==100,'Same-scan endpoint wins over a crossing ray')
    h.phase_wait('far',lambda:h.value()==0)
    i=h.index(1.5625,.0625);check(h.bundle.slope_cost[i]==0. and h.bundle.terrain_cost[i]==0.,'Reobserved crossing cell restores paved0 and slope0')
    h.phase_wait('behind',lambda:h.value(-1.5625)==-1)
    check(h.value(-1.5625)==-1,'Rear-sector return is ignored')
    x=1.9*math.cos(math.radians(95));y=1.9*math.sin(math.radians(95))
    h.phase_wait('outside_angle',lambda:h.value(x,y)==(100 if HALF>=95 else -1))
    check(h.value(x,y)==(100 if HALF>=95 else -1),'95-degree endpoint follows configured half FOV including greater than90')
    h.phase='hold';h.t+=61*10**9;after=h.t
    h.wait(lambda:h.bundle is not None and h.local is not None and stamp(h.bundle.grid.header)>=after and stamp(h.local.header)>=after)
    i=h.index(1.5625,.0625);check(h.bundle.grid.data[i]==0 and h.local.data[i]==0,'Paved cell retains last state in global and local after61 seconds without reobservation')
    h.phase_wait('inside',lambda:h.value()==100)
    check(h.value()==100,'New endpoint observation can reblock a remembered paved cell')
    h.wait(lambda:h.state is not None and h.state.state==ModuleStatus.READY)
    stopped_at=h.t;h.emit=False;h.wait(lambda:h.t-stopped_at>1_100_000_000 and h.state is not None and h.state.state!=ModuleStatus.READY,5.)
    check(h.state.state!=ModuleStatus.READY,'Sensor freshness still revokes READY independently of per-cell retention')
  result={'status':'passed','checks':checks,'checks_passed':len(checks),'selected_half_fov_deg':HALF,'domain':os.environ.get('ROS_DOMAIN_ID'),'actual_active_vio_fixed_z_m':config['vio_fixed_z_m'],'legacy_floor_z_ignored':config['floor_z_cm'],'synthetic_only':True,'actual_sensor_or_normal_trial_started':False}
  (ROOT/('fov_config_ros_'+str(HALF)+'_result.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
 finally:
  if p is not None:
   p.terminate()
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait(timeout=5)
  h.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
