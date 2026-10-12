"""One isolated live trial. Observe existing topics; publish only a normal goal."""
import csv
import shutil
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, String
from nomad_interfaces.msg import VehicleState
from common import contact_classification, save
import json


class Observer(Node):
    def __init__(self, directory, topics, contact_topics):
        super().__init__('nomad_experiment_observer')
        self.declare_parameter('use_sim_time', True) if not self.has_parameter('use_sim_time') else None
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.directory = directory
        self.create_timer(2., self.inventory, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        self.sim = None
        self.pose = None
        self.ready = False
        self.ready_wall = 0.
        self.odom_wall = 0.
        self.vehicle = None
        self.vehicle_wall = 0.
        self.phase = 'warmup'
        self.begin_sim = None
        self.begin_wall = None
        self.last_pose = None
        self.last_stamp = None
        self.distance = 0.
        self.distance_3d = 0.
        self.reverse = 0.
        self.collision = False
        self.unresolved_contact = False
        self.wheel_contact_sim = None
        self.max_odom_gap = 0.
        self.clock_regression = False
        self.first_path_latency = None
        self.max_tilt = 0.
        self.tilt_since = None
        self.last_status = {}
        self.odom_count = 0
        self.pose_stamp = None
        self.contacts_count = 0
        self.contact_example = None
        self.contact_topics = contact_topics
        self.events = (directory/'events.jsonl').open('w')
        self.trace_file = (directory/'trajectory.csv').open('w', newline='')
        self.trace = csv.writer(self.trace_file)
        self.trace.writerow(['sim_s','phase','x','y','z','roll','pitch','yaw','speed'])
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(Odometry, topics['sensors.odometry'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, topics['planning.inputs_ready'], self.on_ready, 10)
        self.create_subscription(VehicleState, topics['control.vehicle_state'], self.on_vehicle, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, topics['planning.global_path'], self.on_path, latched)
        for key in ('planning.command_status','planning.gpp_status','planning.lpp_status','planning.recovery_status'):
            self.create_subscription(String, topics[key], lambda m,k=key:self.on_status(k,m), 10)
        for topic in contact_topics:
            self.create_subscription(Contacts, topic, self.on_contact, qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, topics['planning.goal_request'], 10)
        self.goal_accepted = False
        self.goal_xy = None
        self.create_subscription(PoseStamped, topics['planning.goal'], self.on_accepted, latched)

    def send_goal(self, goal, index):
        self.goal_xy = goal
        self.goal_accepted = False
        self.goal_sent_wall = time.monotonic()
        request = PoseStamped()
        request.header.frame_id = 'odom'
        request.header.stamp = self.get_clock().now().to_msg()
        request.pose.position.x = float(goal[0])
        request.pose.position.y = float(goal[1])
        request.pose.orientation.w = 1.
        self.goal_pub.publish(request)
        self.event('goal_sent', goal=goal, waypoint_index=index, start=self.pose)

    def inventory(self):
        records=[]
        for name, types in self.get_topic_names_and_types():
            publishers=self.get_publishers_info_by_topic(name)
            records.append(dict(name=name, types=types,
                publishers=[dict(node=p.node_name, namespace=p.node_namespace,
                    reliability=str(p.qos_profile.reliability),durability=str(p.qos_profile.durability)) for p in publishers]))
        save(self.directory/'topic_inventory.json',records)

    def event(self, kind, **data):
        self.events.write(json.dumps(dict(event=kind,sim_s=self.sim,wall_s=time.time(),phase=self.phase,**data),ensure_ascii=False)+'\n')
        self.events.flush()

    def on_clock(self, msg):
        value = msg.clock.sec+msg.clock.nanosec*1e-9
        if self.sim is not None and value<self.sim-1e-6:
            self.clock_regression=True
        self.sim=value

    def on_ready(self, msg):
        self.ready=msg.data
        self.ready_wall=time.monotonic()

    def on_vehicle(self, msg):
        self.vehicle=msg
        self.vehicle_wall=time.monotonic()

    def on_accepted(self, msg):
        if self.goal_xy and math.hypot(msg.pose.position.x-self.goal_xy[0],msg.pose.position.y-self.goal_xy[1])<1e-5:
            self.goal_accepted=True
            self.event('goal_accepted')

    def on_status(self, key, msg):
        if self.last_status.get(key)!=msg.data:
            self.last_status[key]=msg.data
            self.event('status',topic=key,value=msg.data)

    def on_path(self, msg):
        if self.phase=='trial' and msg.poses and self.first_path_latency is None:
            self.first_path_latency=time.monotonic()-self.begin_wall
            self.event('first_global_path',latency_wall_s=self.first_path_latency)

    def on_odom(self, msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        values=[p.x,p.y,p.z,roll,pitch,yaw,msg.twist.twist.linear.x,stamp]
        if not all(map(math.isfinite,values)):
            self.clock_regression=True
            return
        self.pose=values[:6]
        self.pose_stamp=stamp
        self.odom_wall=time.monotonic()
        self.odom_count+=1
        self.trace.writerow([stamp,self.phase,*values[:7]])
        if self.phase=='trial':
            if self.last_stamp is not None:
                gap=stamp-self.last_stamp
                self.max_odom_gap=max(self.max_odom_gap,gap)
                if gap<0:
                    self.clock_regression=True
                if gap>0 and self.last_pose is not None:
                    dx,dy,dz=(p.x-self.last_pose[0],p.y-self.last_pose[1],p.z-self.last_pose[2])
                    distance=math.hypot(dx,dy)
                    self.distance+=distance
                    self.distance_3d+=math.sqrt(dx*dx+dy*dy+dz*dz)
                    if msg.twist.twist.linear.x<-.02:
                        self.reverse+=distance
            self.last_pose=(p.x,p.y,p.z)
            self.last_stamp=stamp
            tilt=max(abs(roll),abs(pitch))
            self.max_tilt=max(self.max_tilt,tilt)
            if tilt>=math.radians(60):
                if self.tilt_since is None:self.tilt_since=stamp
            else:self.tilt_since=None

    def on_contact(self, msg):
        for contact in msg.contacts:
            self.contacts_count+=1
            first,second=contact.collision1.name,contact.collision2.name
            if self.contact_example is None:
                self.contact_example=(first,second)
                self.event('first_contact',first=first,second=second)
            kind=contact_classification(first,second)
            if kind=='wheel_ground':
                self.wheel_contact_sim=self.sim
            elif kind in ('forbidden','unresolved'):
                if kind=='unresolved':self.unresolved_contact=True
                if self.phase=='trial' and kind=='forbidden' and not self.collision:
                    self.collision=True
                    self.event('collision',first=first,second=second,depths=list(contact.depths))
                elif self.phase=='warmup' and kind=='forbidden':
                    self.event('warmup_contact',first=first,second=second)

    def close(self):
        self.inventory()
        self.trace_file.flush();self.trace_file.close();self.events.close()
        self.destroy_node()


def run_trial(directory, manifest, trial, contact_topics, processes):
    from nomad_bringup.configuration import load_topics
    topics=load_topics(directory.parents[2]/manifest.get('snapshot_dir','snapshot')/'src/nomad_bringup/config/topics.yaml')
    rclpy.init()
    node=Observer(directory,topics,contact_topics)
    initial_wall=time.monotonic()
    start_wall=None
    stable=None
    result='startup_timeout'
    valid=False
    waypoint_index=0
    goals=trial.get('waypoints') or [trial['goal']]
    try:
        node.event('observer_started')
        while time.monotonic()-initial_wall<manifest['limits']['startup_wall_s']:
            if (directory.parents[2]/'STOP_REQUESTED').exists():result='interrupted';break
            rclpy.spin_once(node,timeout_sec=.05)
            if any(p.poll() is not None for p in processes):
                result='process_exit';break
            contact_ready=(node.wheel_contact_sim is not None and node.sim is not None and
                           node.sim-node.wheel_contact_sim<1. and not node.unresolved_contact and
                           all(node.count_publishers(t)>0 for t in contact_topics))
            bag_ready=any('experiment_bag' in s.node_name for s in node.get_subscriptions_info_by_topic(topics['sensors.odometry']))
            if (node.ready and time.monotonic()-node.ready_wall<1 and node.pose and
                time.monotonic()-node.odom_wall<1 and node.vehicle and node.vehicle.ready and
                node.goal_pub.get_subscription_count()>0 and contact_ready and bag_ready):
                node.phase='trial';node.begin_sim=node.sim;node.begin_wall=time.monotonic()
                start_wall=node.begin_wall
                node.last_pose=tuple(node.pose[:3]);node.last_stamp=node.pose_stamp
                goal=trial['goal']
                if trial.get('pilot'):
                    goal=[node.pose[0]+2*math.cos(node.pose[5]),node.pose[1]+2*math.sin(node.pose[5])]
                if trial.get('pilot'):goals=[goal]
                node.send_goal(goals[0], 0)
                break
        if start_wall is not None:
            from motion_watch import MotionWatch
            policy=manifest.get('no_motion',dict(timeout_sim_s=60.,radius_m=.5))
            motion=MotionWatch(node.sim,node.pose,policy['timeout_sim_s'],policy['radius_m'])
            last_print=time.monotonic()
            while True:
                if (directory.parents[2]/'STOP_REQUESTED').exists():result='interrupted';break
                rclpy.spin_once(node,timeout_sec=.05)
                elapsed=node.sim-node.begin_sim
                wall=time.monotonic()-start_wall
                if any(p.poll() is not None for p in processes):result='process_exit';break
                if node.clock_regression or node.max_odom_gap>1.:result='invalid_odometry';break
                if node.unresolved_contact:result='unresolved_contact';break
                if node.collision:result='collision';valid=True;break
                if node.tilt_since is not None and node.sim-node.tilt_since>=.5:
                    result='rollover';valid=True;break
                if node.wheel_contact_sim is None or node.sim-node.wheel_contact_sim>2.:
                    result='contact_observation_lost';break
                if time.monotonic()-node.odom_wall>10.:result='odometry_stalled';break
                if not node.goal_accepted and time.monotonic()-node.goal_sent_wall>30.:result='goal_not_accepted';break
                distance=math.hypot(node.pose[0]-node.goal_xy[0],node.pose[1]-node.goal_xy[1])
                if (node.goal_accepted and waypoint_index < len(goals)-1 and
                    distance < trial.get('waypoint_radius_m', 1.)):
                    node.event('waypoint_reached', waypoint_index=waypoint_index, goal=node.goal_xy)
                    waypoint_index += 1
                    node.send_goal(goals[waypoint_index], waypoint_index)
                    save(directory/'mission_progress.json', dict(waypoints_reached=waypoint_index,
                        total_waypoints=len(goals), completed_laps=(waypoint_index//trial['waypoints_per_lap']) if trial.get('waypoints_per_lap') else None,
                        target=goals[waypoint_index], elapsed_sim_s=elapsed))
                    stable=None
                    continue
                stopped=(node.vehicle and node.vehicle.speed_valid and abs(node.vehicle.speed)<.05 and
                         time.monotonic()-node.vehicle_wall<1.)
                if waypoint_index==len(goals)-1 and node.goal_accepted and distance<.35 and stopped:
                    if stable is None:stable=node.sim
                    if node.sim-stable>=1.:result='success';valid=True;break
                else:stable=None
                if not trial.get('pilot') and motion.update(node.sim,node.pose):
                    node.event('no_progress',stationary_sim_s=node.sim-motion.since,anchor=motion.anchor,policy=policy)
                    result='no_progress';valid=True;break
                if elapsed>=trial['timeout_sim_s']:result='timeout';valid=True;break
                if wall>=manifest['limits']['trial_wall_s']:result='wall_timeout';break
                if time.monotonic()-last_print>30:
                    if shutil.disk_usage(directory).free<1024**3:result='disk_full_risk';break
                    print(f"{trial['id']}: {elapsed:.1f}s simulated, remaining {distance:.1f}m",flush=True)
                    last_print=time.monotonic()
        save(directory/'mission_progress.json', dict(waypoints_reached=waypoint_index+(result=='success'), total_waypoints=len(goals), result=result))
        node.event('trial_finished',result=result,valid=valid)
        return dict(result=result,valid=valid,waypoints_reached=waypoint_index+(result=='success'),total_waypoints=len(goals),goal=node.goal_xy,distance_xy_m=node.distance,
                    distance_3d_m=node.distance_3d,reverse_distance_m=node.reverse,
                    duration_sim_s=(node.sim-node.begin_sim) if node.begin_sim is not None else 0.,
                    duration_wall_s=time.monotonic()-initial_wall,
                    first_path_latency_wall_s=node.first_path_latency,max_tilt_deg=math.degrees(node.max_tilt),
                    contacts_received=node.contacts_count,contact_example=node.contact_example,odom_received=node.odom_count,
                    startup_ready=node.ready,contact_unresolved=node.unresolved_contact,
                    max_odom_gap_s=node.max_odom_gap,last_status=node.last_status)
    finally:
        node.close();rclpy.shutdown()
