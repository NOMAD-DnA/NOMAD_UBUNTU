"""Explicit controlled GT input and recorder. No semantic/VIO outputs are spoofed."""
import argparse
import csv
import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as ROSPath
from std_msgs.msg import Bool, String
from nomad_interfaces.msg import VehicleState
from .core import observe, rectangles, polygon_clearance, save, read, summarize


def stamp(t):return t.sec+t.nanosec*1e-9


class Worker(Node):
    def __init__(self,directory):
        super().__init__('nomad_scenario_lab_inputs')
        self.directory=Path(directory);self.manifest=read(self.directory/'manifest.json');self.spec=self.manifest['scenario']
        self.start_wall=time.monotonic();self.started_at=datetime.now(timezone.utc).isoformat();self.start_sim=None
        self.odom=None;self.odom_wall=0;self.state=None;self.state_wall=0;self.status='';self.gpp_status='';self.done=False
        self.goal_sent=False;self.samples=[];self.global_count=0;self.empty_count=0;self.invalid_replans=0;self.valid_checks=0
        self.stop_since=None;self.last_map=-math.inf;self.last_sample=-math.inf;self.last_live=0;self.last_progress=None;self.progress_pose=None
        self.boxes=rectangles(self.spec);self.known=[-1]*len(self.spec['grid']);self.cpu={};self.cpu_previous={}
        for p in self.spec.get('prior_observation',[]):observe(self.spec,self.known,p)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(OccupancyGrid,'/nomad/costmap',latched)
        self.pose_pub=self.create_publisher(PoseStamped,'/nomad/current_pose',10)
        self.health_pub=self.create_publisher(Bool,'/nomad/sensors_ready',10)
        self.goal_pub=self.create_publisher(PoseStamped,'/nomad/goal',latched)
        self.create_subscription(Odometry,'/odom',self.on_odom,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(VehicleState,'/nomad/control/vehicle_state',self.on_state,10)
        self.create_subscription(ROSPath,'/nomad/global_path',self.on_path,latched)
        self.create_subscription(String,'/nomad/corridor_status',self.on_corridor,100)
        self.create_subscription(String,'/nomad/planning_status',lambda m:setattr(self,'status',m.data),10)
        self.create_subscription(String,'/nomad/gpp_status',lambda m:setattr(self,'gpp_status',m.data),10)
        self.stream=(self.directory/'trajectory.csv').open('w',newline='');self.csv=csv.DictWriter(self.stream,fieldnames=['time_s','x','y','z','yaw','speed','clearance_m','gpp_paths','valid_checks']);self.csv.writeheader()
        self.events=(self.directory/'events.jsonl').open('w',buffering=1)
        from rclpy.clock import Clock,ClockType
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def on_odom(self,msg):self.odom=msg;self.odom_wall=time.monotonic()
    def on_state(self,msg):self.state=msg;self.state_wall=time.monotonic()
    def event(self,kind,data):self.events.write(json.dumps(dict(sim_time=self.now(),kind=kind,data=data),ensure_ascii=False)+'\n')
    def on_path(self,msg):
        self.global_count+=bool(msg.poses);self.empty_count+=not bool(msg.poses)
        self.event('global_path',dict(points=len(msg.poses),stamp=stamp(msg.header.stamp)))
    def on_corridor(self,msg):
        try:d=json.loads(msg.data)
        except ValueError:return
        if d.get('state')=='validated_safe_path':self.valid_checks+=1
        if d.get('publication')=='new_path' and d.get('replanned_after_invalidity'):self.invalid_replans+=1
        self.event('corridor',d)

    def cpu_sample(self):
        pids=read(self.directory/'pids.json',{})
        for name in ('gpp','lpp'):
            pid=pids.get(name)
            try:
                fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
                # utime/stime and starttime detect PID reuse.
                value=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK');identity=fields[19]
                prev=self.cpu_previous.get(name)
                if prev and prev[0]==identity:self.cpu[name]=self.cpu.get(name,0)+max(0,value-prev[1])
                else:self.cpu.setdefault(name,0.)
                self.cpu_previous[name]=(identity,value)
            except (OSError,ValueError,IndexError):pass

    def tick(self):
        if self.done:return
        wall=time.monotonic();now=self.now();self.cpu_sample()
        fresh=self.odom is not None and wall-self.odom_wall<.6 and -.05<=now-stamp(self.odom.header.stamp)<=.6
        self.health_pub.publish(Bool(data=bool(fresh and self.goal_sent)))
        if not fresh:
            if wall-self.start_wall>30 and self.start_sim is None:self.finish('startup_timeout')
            elif self.start_sim is not None and wall-self.odom_wall>3:self.finish('odometry_lost')
            return
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pose=(p.x,p.y,yaw);self.pose_pub.publish(PoseStamped(header=self.odom.header,pose=self.odom.pose.pose))
        if self.start_sim is None:
            if math.dist(pose[:2],self.spec['start'][:2])>.5:self.finish('spawn_pose_mismatch');return
            self.start_sim=now;self.last_progress=now;self.progress_pose=pose[:2]
        if now-self.last_map>=.2:
            observe(self.spec,self.known,pose);self.last_map=now
            grid=OccupancyGrid();grid.header.frame_id='odom';grid.header.stamp=self.get_clock().now().to_msg();grid.info.resolution=float(self.spec['resolution']);grid.info.width=self.spec['width'];grid.info.height=self.spec['height'];grid.info.origin.orientation.w=1.;grid.data=self.known;self.map_pub.publish(grid)
        if not self.goal_sent and now-self.start_sim>=2 and self.state and self.state.ready:
            if self.goal_pub.get_subscription_count()>=3:
                goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=self.get_clock().now().to_msg();goal.pose.orientation.w=1.;goal.pose.position.x=float(self.spec['goal'][0]);goal.pose.position.y=float(self.spec['goal'][1]);self.goal_pub.publish(goal);self.goal_sent=True;self.start_sim=now;self.last_progress=now;self.event('mission_goal',self.spec['goal'])
        if not self.goal_sent:
            if wall-self.start_wall>40:self.finish('planning_not_ready')
            return
        speed=self.odom.twist.twist.linear.x
        if now-self.last_sample>=.1-1e-6:
            self.last_sample=now;clearance=polygon_clearance(pose,self.boxes)
            row=dict(time_s=now-self.start_sim,x=p.x,y=p.y,z=p.z,yaw=yaw,speed=speed,clearance_m=clearance,gpp_paths=self.global_count,valid_checks=self.valid_checks);self.samples.append(row);self.csv.writerow(row)
            if clearance is not None and clearance<=0:self.finish('envelope_overlap');return
        if math.dist(pose[:2],self.progress_pose)>.1:self.progress_pose=pose[:2];self.last_progress=now
        ready=self.state and self.state.ready and wall-self.state_wall<1
        stopped=ready and abs(speed)<.025
        self.stop_since=(self.stop_since if self.stop_since is not None else now) if stopped else None
        distance=math.dist(pose[:2],self.spec['goal'])
        if distance<.35 and stopped and self.stop_since is not None and now-self.stop_since>=1:self.finish('arrived');return
        waiting=('GPP_WAIT' in self.gpp_status and ('observed corridor' in self.gpp_status))
        if self.spec['id']>=5 and waiting and self.stop_since is not None and now-self.stop_since>=5:
            if self.spec['id']==6:self.finish('forward_limit');return
            wall_seen=any(v>=80 and 14.1<(i%self.spec['width']+.5)*self.spec['resolution']<15 and abs((i//self.spec['width']+.5)*self.spec['resolution']-9)<1 for i,v in enumerate(self.known))
            if wall_seen and math.dist(pose[:2],self.spec['start'][:2])>1:self.finish('safe_stop');return
        if now-self.last_progress>40:self.finish('stalled');return
        if now-self.start_sim>self.manifest.get('timeout_sim_s',180):self.finish('timeout');return
        if wall-self.last_live>.5:
            self.last_live=wall;self.stream.flush();save(self.directory/'live.json',dict(state='running',sim_s=now-self.start_sim,pose=list(pose),goal_distance_m=distance,status=self.status,gpp_status=self.gpp_status,gpp_paths=self.global_count,valid_checks=self.valid_checks,invalidity_replans=self.invalid_replans,cpu=self.cpu))
        if (self.directory/'STOP').exists():self.finish('cancelled')

    def finish(self,result):
        if self.done:return
        self.done=True;self.health_pub.publish(Bool(data=False));self.stream.flush();self.events.flush()
        outcome=summarize(self.samples,result,self.spec,self.global_count,self.invalid_replans,self.cpu,self.started_at)
        outcome.update(algorithm=self.manifest['algorithm'],duration_wall_s=time.monotonic()-self.start_wall,gpp_empty_publications=self.empty_count,valid_checks=self.valid_checks,last_status=self.status,last_gpp_status=self.gpp_status,cpu_definition='whole GPP/LPP process CPU delta (user+system), core-seconds; wall time distinct')
        save(self.directory/'outcome.json',outcome);save(self.directory/'live.json',dict(state='finished',result=result))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory');a,ros_args=parser.parse_known_args();rclpy.init(args=ros_args);node=Worker(a.directory)
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
        # Allow health=false to reach command selector/control before shutdown.
        for _ in range(6):rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:node.finish('cancelled')
    finally:
        node.stream.close();node.events.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
