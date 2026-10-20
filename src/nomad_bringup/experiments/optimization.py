"""Bounded map-4 optimization using existing launchers, recorder and Observer.

All searches and live workers run in owned process groups. A parent wall-clock
watchdog remains responsive even if a ROS callback or GPP computation hangs.
"""
import argparse
import fcntl
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import shutil
import yaml
from common import WORKSPACE, ROOT, save, read, make_world, sha

MAP='04_vio_flat_loop'
GOAL=[28.,0.]
WORLD=None
GPP='dstar_lite'
LPP='rollout'
UNKNOWN_DIRECTION_WEIGHT=0.
UNKNOWN_DIRECTION_ANGLE=30.
LIMITS=dict(startup_wall_s=240.,trial_wall_s=1800.,trial_sim_s=600.,
            no_motion_wall_s=120.,clock_stall_wall_s=20.,search_wall_s=30.)

def status(root,**kw):
    save(root/'status.json',dict(updated=time.time(),pid=os.getpid(),**kw))

def launch(argv,log,env=None):
    stream=open(log,'w')
    try:return subprocess.Popen(argv,stdout=stream,stderr=subprocess.STDOUT,
                                env=env,start_new_session=True)
    finally:stream.close()

def bounded(argv,log,seconds,root):
    from experiment import stop_owned
    p=launch(argv,log);start=time.monotonic()
    try:
        while p.poll() is None:
            if (root/'STOP_REQUESTED').exists():
                # Give the child time to observe the marker and flush normally.
                try:p.wait(timeout=30)
                except subprocess.TimeoutExpired:pass
                return 'interrupted'
            if time.monotonic()-start>seconds:return 'watchdog_timeout'
            time.sleep(.2)
        return 'finished' if p.returncode==0 else 'worker_error'
    finally:
        # Let the live worker flush its bag and stop its own launch groups.
        if p.poll() is None:
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:p.wait(timeout=60)
            except subprocess.TimeoutExpired:pass
        stop_owned([p])
        cleanup_registry(root)

def cleanup_registry(root):
    registry=root/'owned_groups.json'
    if not registry.exists():return
    groups={entry['pid'] for entry in read(registry)}
    expected='nomad_opt_'+root.name
    owned=[]
    for item in Path('/proc').glob('[0-9]*'):
        try:
            fields=(item/'stat').read_text().rsplit(')',1)[1].split()
            if int(fields[2]) not in groups:continue
            env=dict(v.split(b'=',1) for v in (item/'environ').read_bytes().split(b'\0') if b'=' in v)
            if env.get(b'GZ_PARTITION',b'').decode()!=expected:continue
            owned.append((int(item.name),fields[19]))
        except (OSError,ValueError,IndexError):pass
    for pid,starttime in owned:
        try:os.kill(pid,signal.SIGTERM)
        except ProcessLookupError:pass
    if owned:time.sleep(2)
    for pid,starttime in owned:
        try:
            fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
            if fields[19]==starttime:os.kill(pid,signal.SIGKILL)
        except (ProcessLookupError,FileNotFoundError):pass


def provenance(root,unknown_surcharge=4.5):
    root.mkdir(parents=True,exist_ok=True)
    (ROOT/'COLCON_IGNORE').touch();(root/'COLCON_IGNORE').touch()
    model=os.environ.get('NOMAD_OPT_SEMANTIC_MODEL')
    if model:save(root/'semantic_model.json',dict(path=model,sha256=sha(Path(model)),metadata_sha256=sha(Path(model).with_name('metadata.json'))))
    files=[]
    for package in ('nomad_path_planning','nomad_perception','nomad_bringup','nomad_interfaces'):
        for p in (WORKSPACE/'src'/package).rglob('*'):
            if p.is_file() and p.suffix in ('.py','.yaml','.msg','.md','.xml') and '__pycache__' not in p.parts:
                rel=p.relative_to(WORKSPACE); dest=root/'snapshot'/rel
                dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest);files.append((str(rel),sha(p)))
    for rel in ('src/nomad_gazebo/maps/'+MAP+'/worlds/forest.sdf','.nomad/env.sh'):
        p=WORKSPACE/rel;dest=root/'snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest);files.append((rel,sha(p)))
    save(root/'manifest.json',dict(schema=1,kind='multicost',created=time.time(),
        map=MAP,goal_world=list(GOAL),gpp=GPP,lpp=LPP,unknown_direction_weight=UNKNOWN_DIRECTION_WEIGHT,unknown_direction_free_angle_deg=UNKNOWN_DIRECTION_ANGLE,limits=LIMITS,
        files=dict(files),weights=dict(length=1.,radius=2.,missing=1.,unknown_surcharge=unknown_surcharge),
        formula='sum ds*(1+wO*pO+wS*pS+wT*pT+wC*rho_R); unknown uses missing priors + unknown_surcharge',
        notes=['Simulation evidence only','Reference route is not shortest-path ground truth',
               'Offline frozen-map ranking is not a live success claim',
               'Ground-truth used for evaluation only; goal uses verified shared ENU datum']))
    if WORLD is not None:
        shutil.copy2(WORLD,root/'input_world.sdf')
        save(root/'mission.json',dict(map=MAP,goal=list(GOAL),world_file=str(WORLD),world_sha256=sha(WORLD),frame='odom'))
    with (root/'source.patch').open('w') as f:subprocess.run(['git','diff','--binary','HEAD'],cwd=WORKSPACE,stdout=f,check=True)

def validate_source(root):
    for rel,h in read(root/'manifest.json')['files'].items():
        if sha(WORKSPACE/rel)!=h:raise RuntimeError('Source changed since experiment snapshot: '+rel)


def make_trial_world(original, destination, start_yaw=None):
    """Instrument a copy; optional yaw changes only the vehicle model's pose."""
    import xml.etree.ElementTree as ET
    if start_yaw is not None and not math.isfinite(start_yaw):
        raise ValueError('Start yaw must be finite')
    contacts=make_world(original,destination)
    tree=ET.parse(destination)
    model=tree.find("world/model[@name='nomad_vehicle']")
    if model is None:raise ValueError('No embedded nomad_vehicle model in world')
    pose=model.find('pose')
    if pose is None:pose=ET.SubElement(model,'pose');pose.text='0 0 0 0 0 0'
    if pose.get('relative_to') or pose.get('degrees')=='true' or pose.get('rotation_format','euler_rpy')!='euler_rpy':
        raise ValueError('Unsupported vehicle spawn pose convention')
    values=list(map(float,pose.text.split()))
    if len(values)!=6:raise ValueError('Expected XYZ RPY spawn pose')
    before=list(values)
    if start_yaw is not None:
        values[5]=start_yaw
        pose.text=' '.join(format(v,'.17g') for v in values)
        tree.write(destination,encoding='unicode',xml_declaration=True)
    return contacts,dict(original=before,effective=values,units='XYZ metres, RPY radians')


def live(root,trial,weights,unknown_surcharge=4.5,start_yaw=None,rviz=False,cluster_weight=0.,cluster_radius=4.,gazebo=False,cluster_influence=0.,cluster_saturation=1.):
    """One trial; no fallback to different localization/semantic providers."""
    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist,PoseStamped
    from nav_msgs.msg import Odometry
    from nomad_interfaces.msg import ModuleStatus,PlanningCostLayers
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from observe import Observer
    from experiment import check_domain,stop_owned
    from nomad_bringup.configuration import load_topics
    directory=root/'runs'/trial
    directory.mkdir(parents=True,exist_ok=True)
    if (directory/'outcome.json').exists():raise RuntimeError('Trial already exists; use a new trial ID')
    domain=int(os.environ['ROS_DOMAIN_ID']);check_domain(domain)
    topics=load_topics(WORKSPACE/'src/nomad_bringup/config/topics.yaml')
    original=WORLD or WORKSPACE/'src/nomad_gazebo/maps'/MAP/'worlds/forest.sdf'
    import xml.etree.ElementTree as ET
    gps=yaml.safe_load((WORKSPACE/'src/nomad_gazebo/config/sensors.yaml').read_text())['gps']
    datum=yaml.safe_load((WORKSPACE/'src/nomad_localization/config/fusion.yaml').read_text())['fusion']['ros__parameters']['datum']
    coordinates=ET.parse(original).find('world/spherical_coordinates')
    expected=[gps['latitude_deg'],gps['longitude_deg'],gps['elevation_m']]
    if coordinates is not None:
        if coordinates.findtext('world_frame_orientation','ENU')!='ENU' or float(coordinates.findtext('heading_deg','0'))!=0:
            raise ValueError('World axes do not match ENU')
        expected=[float(coordinates.findtext(k)) for k in ('latitude_deg','longitude_deg','elevation')]
    if not gps['enabled'] or expected!=datum:raise ValueError('World/fusion datum mismatch; mission coordinates not valid')
    contacts,spawn=make_trial_world(original,directory/'world.sdf',start_yaw)
    save(directory/'spawn.json',spawn)
    bridge=[dict(ros_topic_name=t,gz_topic_name=t,ros_type_name='ros_gz_interfaces/msg/Contacts',gz_type_name='gz.msgs.Contacts',direction='GZ_TO_ROS') for t in contacts]
    (directory/'contacts.yaml').write_text(yaml.safe_dump(bridge))
    params=yaml.safe_load((WORKSPACE/'src/nomad_perception/config/terrain_planning.yaml').read_text())
    p=params['dstar_lite_gpp']['ros__parameters']
    mode=os.environ.get('NOMAD_OPT_GOAL_MODE','direct')
    for node_name in ('dstar_lite_gpp','nomad_planning_command','nomad_recovery_supervisor'):
        params.setdefault(node_name,{}).setdefault('ros__parameters',{})['navigation.goal_mode']=mode
    p['gpp_algorithm']=GPP
    p['gpp.unknown_direction_weight']=UNKNOWN_DIRECTION_WEIGHT
    p['gpp.unknown_direction_radius']=6.
    p['gpp.unknown_direction_free_angle_deg']=UNKNOWN_DIRECTION_ANGLE
    p.update({'gpp.cost_mode':'legacy' if weights is None else 'multi','gpp.time_budget':20.,'gpp.max_expansions':200000})
    p.update({'gpp.unknown_surcharge':unknown_surcharge,'unknown_penalty':unknown_surcharge,
              'gpp.cluster_weight':cluster_weight,'gpp.cluster_radius':cluster_radius,'gpp.cluster_influence':cluster_influence,'gpp.cluster_saturation':cluster_saturation})
    if weights:p.update({'gpp.proximity_weight':weights[0],'gpp.slope_weight':weights[1],'gpp.terrain_weight':weights[2]})
    override=os.environ.get('NOMAD_OPT_PLANNING_OVERRIDES')
    if override:
        document=yaml.safe_load(Path(override).read_text())
        allowed={'dstar_lite_gpp','ackermann_rollout_lpp','nomad_recovery_supervisor','nomad_planning_command'}
        if not isinstance(document,dict) or set(document)-allowed:raise ValueError('Invalid planning override nodes')
        for name,content in document.items():
            if not isinstance(content,dict) or set(content)!={'ros__parameters'} or not isinstance(content['ros__parameters'],dict):raise ValueError('Invalid planning override structure')
            params.setdefault(name,{}).setdefault('ros__parameters',{}).update(content['ros__parameters'])
        shutil.copy2(override,directory/'planning_overrides.yaml')
    (directory/'params.yaml').write_text(yaml.safe_dump(params))
    latched=[topics[k] for k in ('perception.camera_observation_support','perception.global_costmap','perception.local_costmap','perception.planning_cost_layers','planning.global_path','planning.goal','planning.navigation_target','planning.recovery_gates')]+['/tf_static']
    (directory/'qos.yaml').write_text(yaml.safe_dump({t:dict(reliability='reliable',durability='transient_local',history='keep_last',depth=1) for t in latched}))
    commands=[['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p',f'config_file:={directory/"contacts.yaml"}'],
        ['ros2','bag','record','--node-name','experiment_bag','--disable-keyboard-controls','--use-sim-time','-s','mcap','-o',str(directory/'bag'),'--qos-profile-overrides-path',str(directory/'qos.yaml'),'--storage-preset-profile','zstd_fast','--max-bag-size','536870912','--all-topics','--include-hidden-topics','--exclude-topic-types','sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage'],
        ['bash',str(WORKSPACE/'scripts/run_terrain_navigation.sh'),f'world_file:={directory/"world.sdf"}',f'world_resource_root:={WORKSPACE / "src/nomad_gazebo/maps" / MAP}',
         'method:=basalt',f'gpp:={GPP}',f'lpp:={LPP}',f'params_file:={directory/"params.yaml"}',
         f'headless:={str(not gazebo).lower()}',f'rviz:={str(rviz).lower()}','viewer:=false','initial_teleop:=false',
         'record_results:=true',f'record_directory:={directory/"terrain"}',f'vio_log_directory:={directory/"vio"}']]
    terrain=os.environ.get('NOMAD_OPT_TERRAIN_CONFIG')
    if terrain:
        shutil.copy2(terrain,directory/'terrain_config.yaml')
        commands[-1].append('terrain_config:='+str(directory/'terrain_config.yaml'))
    model=os.environ.get('NOMAD_OPT_SEMANTIC_MODEL')
    if model:
        commands[-1].append('semantic_model:='+model)
        save(directory/'semantic_model.json',dict(path=model,sha256=sha(Path(model)),metadata=json.loads(Path(model).with_name('metadata.json').read_text())))
    env=dict(os.environ);env['ROS_LOG_DIR']=str(directory/'ros_logs');env['NOMAD_EXPERIMENT_TIMING_DIR']=str(directory/'timing')
    save(directory/'metadata.json',dict(commands=commands,spawn=spawn,gpp=GPP,lpp=LPP,unknown_direction_weight=UNKNOWN_DIRECTION_WEIGHT,unknown_direction_free_angle_deg=UNKNOWN_DIRECTION_ANGLE,weights=weights,unknown_surcharge=unknown_surcharge,cluster_weight=cluster_weight,cluster_radius=cluster_radius,cluster_influence=cluster_influence,cluster_saturation=cluster_saturation,limits=LIMITS,domain=domain,partition=env.get('GZ_PARTITION')))
    processes=[];node=None;manual=None;result='startup_timeout';valid=False
    started=time.monotonic();begin=None;clock_seen=started;last_clock=None;moving=started;anchor=None
    warm_origin=None;released=False;warm_distance=0.;latest={};snap_count=0;last_snapshot=-math.inf
    try:
        for i,cmd in enumerate(commands):
            processes.append(launch(cmd,directory/f'process_{i}.log',env))
            save(root/'owned_groups.json',[dict(pid=p.pid,starttime=Path(f'/proc/{p.pid}/stat').read_text().split()[21]) for p in processes])
        rclpy.init();node=Observer(directory,topics,contacts)
        def receive(k,m):latest[k]=(m,time.monotonic())
        node.create_subscription(Odometry,topics['localization.odometry'],lambda m:receive('fused',m),10)
        node.create_subscription(ModuleStatus,topics['localization.status'],lambda m:receive('localization',m),10)
        node.create_subscription(ModuleStatus,topics['perception.status'],lambda m:receive('perception',m),10)
        def capture(m):
            nonlocal snap_count,last_snapshot
            stamp=m.grid.header.stamp.sec+m.grid.header.stamp.nanosec*1e-9
            if stamp-last_snapshot<5:return
            last_snapshot=stamp;snap_count+=1
            i=m.grid.info;folder=directory/'maps';folder.mkdir(exist_ok=True)
            np.savez_compressed(folder/f'{snap_count:05}.npz',data=np.array(m.grid.data,dtype=np.int8),
                slope=np.array(m.slope_cost,dtype=np.float32),terrain=np.array(m.terrain_cost,dtype=np.float32),
                width=i.width,height=i.height,resolution=i.resolution,origin_x=i.origin.position.x,origin_y=i.origin.position.y,
                stamp=stamp,start=np.array([latest['fused'][0].pose.pose.position.x,latest['fused'][0].pose.pose.position.y] if 'fused' in latest else [np.nan,np.nan]),
                start_yaw=(math.atan2(2*(latest['fused'][0].pose.pose.orientation.w*latest['fused'][0].pose.pose.orientation.z+latest['fused'][0].pose.pose.orientation.x*latest['fused'][0].pose.pose.orientation.y),1-2*(latest['fused'][0].pose.pose.orientation.y**2+latest['fused'][0].pose.pose.orientation.z**2)) if 'fused' in latest else np.nan),
                goal=np.array(getattr(node,'goal_odom',[np.nan,np.nan])),frame=m.grid.header.frame_id)
        node.create_subscription(PlanningCostLayers,topics['perception.planning_cost_layers'],capture,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Equivalent of straight initial teleop only, capped at 3 m. No commands
        # after localization alignment / other control publisher discovery.
        manual=node.create_publisher(Twist,'/cmd_vel',5)
        node.event('initial_alignment',method='bounded straight teleop',max_distance_m=3.,speed_mps=.25)
        stable=None;last_report=0.;last_cmd=0.
        while True:
            rclpy.spin_once(node,timeout_sec=.03);now=time.monotonic()
            if (root/'STOP_REQUESTED').exists():result='interrupted';break
            if any(p.poll() is not None for p in processes):result='process_exit';break
            if node.sim is not None and node.sim!=last_clock:last_clock=node.sim;clock_seen=now
            if now-clock_seen>LIMITS['clock_stall_wall_s']:result='clock_stalled';break
            if shutil.disk_usage(directory).free<8*1024**3:result='low_disk';break
            if node.collision:result='collision';valid=begin is not None;break
            if node.unresolved_contact:result='unresolved_contact';break
            if node.clock_regression:result='clock_reset';break
            if begin is None:
                if now-started>LIMITS['startup_wall_s']:break
                if node.pose and now-node.odom_wall<1 and warm_origin is None:warm_origin=tuple(node.pose[:2])
                if node.pose and warm_origin:warm_distance=math.dist(warm_origin,node.pose[:2])
                aligned=('localization' in latest and latest['localization'][0].state==ModuleStatus.READY and now-latest['localization'][1]<1)
                others=[x.node_name for x in node.get_publishers_info_by_topic('/cmd_vel') if x.node_name!=node.get_name()]
                if not released and (aligned or warm_distance>=3. or others):
                    manual.publish(Twist());node.destroy_publisher(manual);manual=None;released=True
                    node.event('initial_teleop_released',aligned=aligned,distance=warm_distance,other_publishers=others)
                if manual and now-last_cmd>=.05:
                    msg=Twist()
                    if warm_origin and now-node.odom_wall<1 and node.sim is not None and node.sim>2:msg.linear.x=.25
                    manual.publish(msg);last_cmd=now
                contact_ready=(node.wheel_contact_sim is not None and node.sim-node.wheel_contact_sim<1)
                bag_ready=any('experiment_bag' in x.node_name for x in node.get_subscriptions_info_by_topic('/odom'))
                if released and node.ready and now-node.ready_wall<1 and node.vehicle and node.vehicle.ready and contact_ready and bag_ready and 'fused' in latest and node.goal_pub.get_subscription_count()>0:
                    # Simulator ENU datum and fusion datum have been checked equal.
                    # Estimated pose error is NOT a coordinate-frame transform.
                    f=latest['fused'][0];goal=list(GOAL)
                    node.goal_odom=goal
                    save(directory/'goal_transform.json',dict(world_goal=goal,odom_goal=goal,rotation=0.,contract='same verified ENU datum',gt_pose=node.pose,fused_position=[f.pose.pose.position.x,f.pose.pose.position.y]))
                    node.phase='trial';node.begin_sim=node.sim;node.begin_wall=now;node.last_pose=tuple(node.pose[:3]);node.last_stamp=node.pose_stamp
                    node.send_goal(goal,0);begin=now;anchor=tuple(node.pose[:2]);moving=now
            else:
                if now-node.odom_wall>10:result='odometry_stalled';break
                if node.wheel_contact_sim is None or node.sim-node.wheel_contact_sim>2:result='contact_observation_lost';break
                if not node.goal_accepted and now-node.goal_sent_wall>30:result='goal_not_accepted';break
                if math.dist(anchor,node.pose[:2])>=.5:anchor=tuple(node.pose[:2]);moving=now
                if now-moving>LIMITS['no_motion_wall_s']:result='no_progress';valid=True;break
                if node.tilt_since is not None and node.sim-node.tilt_since>=.5:result='rollover';valid=True;break
                dist=math.hypot(node.pose[0]-GOAL[0],node.pose[1]-GOAL[1])
                stopped=node.vehicle and node.vehicle.speed_valid and abs(node.vehicle.speed)<.05 and now-node.vehicle_wall<1
                if dist<.35 and stopped and node.goal_accepted:
                    stable=node.sim if stable is None else stable
                    if node.sim-stable>=1:result='success';valid=True;break
                else:stable=None
                if node.sim-node.begin_sim>LIMITS['trial_sim_s']:result='sim_timeout';valid=True;break
                if now-begin>LIMITS['trial_wall_s']:result='wall_timeout';break
            if now-last_report>2:
                status(root,state='running',trial=trial,phase=node.phase,elapsed_wall_s=now-started,sim_s=node.sim,
                       initialized=released,warmup_distance_m=warm_distance,position=node.pose,
                       provider_status={k:v[0].detail for k,v in latest.items() if k!='fused'},last_status=node.last_status,snapshots=snap_count)
                last_report=now
        outcome=dict(result=result,valid=valid,spawn=spawn,gpp=GPP,lpp=LPP,unknown_direction_weight=UNKNOWN_DIRECTION_WEIGHT,unknown_direction_free_angle_deg=UNKNOWN_DIRECTION_ANGLE,weights=weights,unknown_surcharge=unknown_surcharge,cluster_weight=cluster_weight,cluster_radius=cluster_radius,cluster_influence=cluster_influence,cluster_saturation=cluster_saturation,duration_wall_s=time.monotonic()-started,
            duration_sim_s=node.sim-node.begin_sim if begin else None,distance_xy_m=node.distance,reverse_distance_m=node.reverse,
            contacts_received=node.contacts_count,contact_unresolved=node.unresolved_contact,contact_example=node.contact_example,
            last_status=node.last_status,provider_status={k:v[0].detail for k,v in latest.items() if k!='fused'},
            goal_world=list(GOAL),goal_odom=getattr(node,'goal_odom',None),position=node.pose,snapshots=snap_count)
        save(directory/'outcome.json',outcome)
    except Exception as e:
        save(directory/'outcome.json',dict(result='exception',valid=False,error=repr(e)));raise
    finally:
        # A second Ctrl-C must not interrupt bag flushing/owned-process cleanup.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            if manual is not None:manual.publish(Twist());node.destroy_publisher(manual)
            if node is not None:node.close()
            if rclpy.ok():rclpy.shutdown()
        finally:
            stop_owned(processes)
    return outcome


def search(mapfile,output,weights,unknown_surcharge=4.5,cluster_weight=0.,cluster_radius=4.,cluster_influence=0.,cluster_saturation=1.):
    import numpy as np
    from nomad_path_planning.gpp.base import Request,BudgetExceeded
    from nomad_path_planning.gpp.registry import create
    from nomad_path_planning.gpp.multicost import compose
    from nomad_path_planning.gpp.proximity import proximity_costs
    a=np.load(mapfile);g={k:a[k].item() for k in ('width','height','resolution','origin_x','origin_y')};g['data']=a['data'].tolist()
    if not np.isfinite(a['start']).all() or not np.isfinite(a['goal']).all():
        save(output,dict(result='missing_start_goal'));return
    if weights is None:g['proximity']=proximity_costs(g['data'],g['width'],g['height'],g['resolution'])
    else:g['cell_weights']=compose(g,a['slope'],a['terrain'],obstacle_weight=weights[0],slope_weight=weights[1],terrain_weight=weights[2],unknown_surcharge=unknown_surcharge,cluster_weight=cluster_weight,cluster_radius=cluster_radius,cluster_influence=cluster_influence,cluster_saturation=cluster_saturation)
    if GPP=='hybrid_astar' and 'start_yaw' not in a:raise ValueError('Measured start_yaw required for kinematic comparison')
    start_yaw=float(a['start_yaw']) if 'start_yaw' in a else 0.
    request=Request(g,tuple(a['start'])+(start_yaw,),tuple(a['goal']))
    if UNKNOWN_DIRECTION_WEIGHT:
        from nomad_path_planning.gpp.base import Grid
        from nomad_path_planning.gpp.unknown_direction import unknown_direction_cost
        view=Grid(request,unknown_surcharge)
        extra=unknown_direction_cost(g,request.start,UNKNOWN_DIRECTION_WEIGHT,free_angle=math.radians(UNKNOWN_DIRECTION_ANGLE))
        g['cell_weights']=[v+float(e) for v,e in zip(view.weights,extra)]
    start=time.perf_counter();planner=create(GPP,seconds=20.,expansions=200000,unknown_penalty=unknown_surcharge)
    try:r=planner.plan(request)
    except (BudgetExceeded,RuntimeError,IndexError) as e:
        save(output,dict(result='search_failed',reason=str(e),elapsed_s=time.perf_counter()-start));return
    if not r.path:save(output,dict(result='no_path',detail=r.detail));return
    from nomad_path_planning.route_guard import route_is_behind
    if GPP=='hybrid_astar' and route_is_behind(request.start,r.path):
        save(output,dict(result='rearward_start_rejected'));return
    from nomad_path_planning.gpp.base import Grid
    from nomad_path_planning.gpp.multicost import cluster_density
    influence_density=cluster_density(g,4.,1.,.4)
    influence_exposure=0.
    from nomad_path_planning.gpp.proximity import proximity_costs
    near=proximity_costs(g['data'],g['width'],g['height'],g['resolution'],2.,1.)
    near_exposure=0.
    density=cluster_density(g,4.)  # fixed evaluation radius across candidates
    view=Grid(request);values=[];unknown=0.;length=0.;exposure=0.
    for p,q in zip(r.path,r.path[1:]):
        ds=math.dist(p[:2],q[:2]);length+=ds
        x,y=view.cell(*q[:2]);v=g['data'][y*g['width']+x]
        px,py=view.cell(*p[:2])
        near_exposure+=ds*(near[y*g['width']+x]+near[py*g['width']+px])/2
        influence_exposure+=ds*float(influence_density[y*g['width']+x]+influence_density[py*g['width']+px])/2
        exposure+=ds*float(density[y*g['width']+x]+density[py*g['width']+px])/2
        if v<0:unknown+=ds
    save(output,dict(result='path',gpp=GPP,unknown_direction_weight=UNKNOWN_DIRECTION_WEIGHT,unknown_direction_free_angle_deg=UNKNOWN_DIRECTION_ANGLE,weights=weights,unknown_surcharge=unknown_surcharge,cluster_weight=cluster_weight,cluster_radius=cluster_radius,cluster_influence=cluster_influence,cluster_saturation=cluster_saturation,path=r.path,length_m=length,unknown_length_m=unknown,cluster_exposure_4m=exposure,influence_exposure_4m_1m=influence_exposure,near_exposure_2m=near_exposure,
                     screening_score=length+18.*unknown+128.*exposure,
                     elapsed_s=time.perf_counter()-start,expansions=r.expansions,
                     note='Frozen observed map only; not live success or verified body clearance'))


def select_cluster_priority(rows, snapshots, tolerance=.02):
    """First observed-cluster exposure, then unknown length and length.

    Every snapshot must succeed. Within 2% of lowest mean exposure is a tie;
    this is a fixed engineering preference, not a safety/optimality guarantee.
    """
    grouped={}
    for row in rows:
        if row.get('result')=='path':grouped.setdefault(row['condition'],[]).append(row)
    eligible={c:rs for c,rs in grouped.items() if {r['snapshot'] for r in rs}==set(snapshots)}
    if not eligible:raise ValueError('No candidate solved every snapshot')
    metrics={c:{k:sum(r[k] for r in rs)/len(rs) for k in
                ('influence_exposure_4m_1m','unknown_length_m','length_m')} for c,rs in eligible.items()}
    best=min(v['influence_exposure_4m_1m'] for v in metrics.values())
    tied=[c for c,v in metrics.items() if v['influence_exposure_4m_1m']<=best*(1+tolerance)+1e-9]
    winner=min(tied,key=lambda c:(metrics[c]['unknown_length_m'],metrics[c]['length_m'],c))
    return eligible[winner][0],dict(winner=winner,metrics=metrics,exposure_tolerance=tolerance,
        rule='Lowest mean observed cluster exposure; within 2% compare unknown length then length',
        note='Unknown exposure is not proof of safety; no inference of hidden obstacles')


def rank_balanced_forward(rows, snapshots):
    grouped={}
    for row in rows:
        if row.get('result')=='path':grouped.setdefault(row['condition'],[]).append(row)
    eligible={c:rs for c,rs in grouped.items() if {r['snapshot'] for r in rs}==set(snapshots)}
    if not eligible:raise ValueError('No forward candidate solved all snapshots')
    minimum={snap:min(r['length_m'] for rs in eligible.values() for r in rs if r['snapshot']==snap) for snap in snapshots}
    unknown_min={snap:min(r['unknown_length_m'] for rs in eligible.values() for r in rs if r['snapshot']==snap) for snap in snapshots}
    # Limit extreme detours and additional unknown travel before prioritizing clusters.
    bounded={c:rs for c,rs in eligible.items() if all(r['length_m']<=1.35*minimum[r['snapshot']] and
             r['unknown_length_m']<=unknown_min[r['snapshot']]+.25*minimum[r['snapshot']] for r in rs)}
    if not bounded:raise ValueError('No candidate satisfies detour/unknown screening limits')
    keys=('influence_exposure_4m_1m','near_exposure_2m','unknown_length_m','length_m')
    metrics={c:{k:sum(r[k] for r in rs)/len(rs) for k in keys} for c,rs in bounded.items()}
    # Keep a reproducible shortlist: 5%-near exposure ties favor less proximity,
    # then unknown travel, then length. Exclude rejected conditions explicitly.
    remaining=set(metrics);ranked=[]
    while remaining:
        low=min(metrics[c][keys[0]] for c in remaining)
        tied=[c for c in remaining if metrics[c][keys[0]]<=low*1.05+1e-9]
        winner=min(tied,key=lambda c:(metrics[c][keys[1]],metrics[c][keys[2]],metrics[c][keys[3]],c))
        ranked.append(winner);remaining.remove(winner)
    return [bounded[c][0] for c in ranked],dict(ranked=ranked,metrics=metrics,min_lengths=minimum,
        min_unknown=unknown_min,detour_ratio_limit=1.35,extra_unknown_limit_fraction=.25,
        rejected_conditions=sorted(set(grouped)-set(bounded)),
        rule='All snapshots forward-valid; bounded detour/unknown then cluster exposure, 5% ties use proximity/unknown/length')


def main(argv=None):
    global MAP,GOAL,WORLD,GPP,LPP,UNKNOWN_DIRECTION_WEIGHT,UNKNOWN_DIRECTION_ANGLE
    p=argparse.ArgumentParser(description='Bounded multi-cost experiment')
    p.add_argument('--gpp',choices=['dstar_lite','hybrid_astar'],default='dstar_lite')
    p.add_argument('--lpp',choices=['rollout','rpp'],default='rollout')
    p.add_argument('--unknown-direction-weight',type=float,default=0.)
    p.add_argument('--unknown-direction-free-angle',type=float,default=30.)
    p.add_argument('--broad-sweep',action='store_true')
    p.add_argument('--planning-overrides',default=os.environ.get('NOMAD_OPT_PLANNING_OVERRIDES'))
    p.add_argument('--trial-wall-seconds',type=float,default=float(os.environ.get('NOMAD_OPT_TRIAL_WALL','1800')))
    p.add_argument('--trial-sim-seconds',type=float,default=float(os.environ.get('NOMAD_OPT_TRIAL_SIM','600')))
    p.add_argument('--no-motion-seconds',type=float,default=float(os.environ.get('NOMAD_OPT_NO_MOTION','120')))
    p.add_argument('--goal-mode',choices=['direct','observed_corridor'],default=os.environ.get('NOMAD_OPT_GOAL_MODE','direct'))
    p.add_argument('--terrain-config',help='Per-experiment perception YAML, inherited by bounded workers')
    p.add_argument('--semantic-model',help='ONNX override for this experiment; inherited by bounded workers')
    p.add_argument('--mission',help='JSON containing map, world_file, goal and odom frame')
    p.add_argument('name');p.add_argument('--stage',choices=['baseline','validate','sweep','optimize','status','stop','live','search'],default='baseline')
    p.add_argument('--foreground',action='store_true');p.add_argument('--weights');p.add_argument('--trial',default='baseline_01')
    p.add_argument('--start-yaw',type=float,default=None,help='Optional spawn yaw in radians; position unchanged')
    p.add_argument('--unknown-surcharge',type=float,default=4.5)
    p.add_argument('--gazebo',action='store_true',help='Show Gazebo GUI for this live trial')
    p.add_argument('--rviz',action='store_true',help='Show live trial in RViz')
    p.add_argument('--cluster-influence',type=float,default=0.)
    p.add_argument('--cluster-saturation',type=float,default=1.)
    p.add_argument('--cluster-weight',type=float,default=0.)
    p.add_argument('--cluster-radius',type=float,default=4.)
    p.add_argument('--cluster-sweep',action='store_true')
    p.add_argument('--influence-sweep',action='store_true')
    p.add_argument('--cluster-priority',action='store_true')
    p.add_argument('--mapfiles',nargs='+',help='Multiple measured pre-entry snapshots')
    p.add_argument('--wide-sweep',action='store_true',help='16 clearance/unknown combinations on one recorded input')
    p.add_argument('--mapfile');p.add_argument('--output')
    a=p.parse_args(argv)
    for value,key,env,upper in ((a.trial_wall_seconds,'trial_wall_s','NOMAD_OPT_TRIAL_WALL',7200.),(a.trial_sim_seconds,'trial_sim_s','NOMAD_OPT_TRIAL_SIM',3600.)):
        if not math.isfinite(value) or not 30<=value<=upper:raise ValueError('Invalid trial time budget')
        LIMITS[key]=value;os.environ[env]=str(value)
    if not math.isfinite(a.no_motion_seconds) or a.no_motion_seconds<10:raise ValueError('Invalid no-motion timeout')
    LIMITS['no_motion_wall_s']=a.no_motion_seconds
    os.environ['NOMAD_OPT_NO_MOTION']=str(a.no_motion_seconds)
    if a.planning_overrides:
        overrides=Path(a.planning_overrides).resolve()
        if not overrides.is_file():raise ValueError('Missing planning overrides')
        os.environ['NOMAD_OPT_PLANNING_OVERRIDES']=str(overrides)
    os.environ['NOMAD_OPT_GOAL_MODE']=a.goal_mode
    if a.terrain_config:
        terrain=Path(a.terrain_config).resolve()
        if not terrain.is_file():raise ValueError("Missing terrain config")
        os.environ["NOMAD_OPT_TERRAIN_CONFIG"]=str(terrain)
    if a.semantic_model:
        model=Path(a.semantic_model).resolve()
        if not model.is_file() or not model.with_name("metadata.json").is_file():
            raise ValueError("Semantic ONNX and sibling metadata.json required")
        os.environ["NOMAD_OPT_SEMANTIC_MODEL"]=str(model)
    GPP=a.gpp
    LPP=a.lpp
    UNKNOWN_DIRECTION_WEIGHT=a.unknown_direction_weight
    UNKNOWN_DIRECTION_ANGLE=a.unknown_direction_free_angle
    if not math.isfinite(UNKNOWN_DIRECTION_ANGLE) or not 0<=UNKNOWN_DIRECTION_ANGLE<90:raise ValueError("Invalid direction angle")
    if not math.isfinite(UNKNOWN_DIRECTION_WEIGHT) or UNKNOWN_DIRECTION_WEIGHT<0:raise ValueError("Invalid unknown direction weight")
    if a.broad_sweep and GPP!='hybrid_astar':raise ValueError('Broad forward comparison requires hybrid_astar')
    if a.mission:
        mission=read(Path(a.mission));MAP=mission['map'];GOAL=mission['goal'];WORLD=Path(mission['world_file']).resolve()
        if (mission.get('frame')!='odom' or len(GOAL)!=2 or not all(math.isfinite(float(v)) for v in GOAL)
                or not WORLD.is_file() or '/' in MAP or not (WORKSPACE/'src/nomad_gazebo/maps'/MAP).is_dir()):
            raise ValueError('Invalid mission map, goal, frame or world file')
    import re
    if not re.fullmatch(r'[\w-]{1,80}',a.name) or not re.fullmatch(r'[\w-]{1,80}',a.trial):raise ValueError('Invalid experiment name')
    if a.start_yaw is not None and not math.isfinite(a.start_yaw):raise ValueError('Invalid start yaw')
    if not math.isfinite(a.unknown_surcharge) or a.unknown_surcharge<0:raise ValueError('Invalid unknown surcharge')
    if not math.isfinite(a.cluster_weight) or a.cluster_weight<0 or not math.isfinite(a.cluster_radius) or a.cluster_radius<=0:raise ValueError('Invalid cluster settings')
    if not math.isfinite(a.cluster_influence) or a.cluster_influence<0 or not math.isfinite(a.cluster_saturation) or not 0<a.cluster_saturation<=1:raise ValueError('Invalid influence/saturation')
    root=ROOT/a.name;weights=[float(v) for v in a.weights.split(',')] if a.weights else None
    if weights is not None and (len(weights)!=3 or any(not math.isfinite(v) or v<0 for v in weights)):raise ValueError('Three nonnegative weights required')
    if a.stage=='status':print(json.dumps(read(root/'status.json'),ensure_ascii=False,indent=2));return
    if a.stage=='stop':(root/'STOP_REQUESTED').touch();return
    if a.stage=='search':return search(a.mapfile,a.output,weights,a.unknown_surcharge,a.cluster_weight,a.cluster_radius,a.cluster_influence,a.cluster_saturation)
    if a.stage=='live':return live(root,a.trial,weights,a.unknown_surcharge,a.start_yaw,a.rviz,a.cluster_weight,a.cluster_radius,a.gazebo,a.cluster_influence,a.cluster_saturation)
    root.mkdir(parents=True,exist_ok=True)
    if not a.foreground:
        cmd=[sys.executable,str(Path(__file__)),a.name,'--stage',a.stage,'--foreground']
        if a.weights:cmd+=['--weights',a.weights]
        cmd+=['--trial',a.trial,'--unknown-surcharge',str(a.unknown_surcharge),
              '--cluster-weight',str(a.cluster_weight),'--cluster-radius',str(a.cluster_radius)]
        if a.cluster_sweep:cmd+=['--cluster-sweep']
        if a.influence_sweep:cmd+=['--influence-sweep']
        if a.cluster_priority:cmd+=['--cluster-priority']
        if a.broad_sweep:cmd+=['--broad-sweep']
        if a.mapfiles:cmd+=['--mapfiles']+a.mapfiles
        if a.start_yaw is not None:cmd+=['--start-yaw',str(a.start_yaw)]
        cmd+=['--gpp',GPP,'--lpp',LPP,'--unknown-direction-weight',str(UNKNOWN_DIRECTION_WEIGHT),'--unknown-direction-free-angle',str(UNKNOWN_DIRECTION_ANGLE)]
        cmd+=['--cluster-influence',str(a.cluster_influence),'--cluster-saturation',str(a.cluster_saturation)]
        if a.gazebo:cmd+=['--gazebo']
        if a.rviz:cmd+=['--rviz']
        if a.mission:cmd+=['--mission',a.mission]
        if a.wide_sweep:cmd+=['--wide-sweep']
        if a.mapfile:cmd+=['--mapfile',a.mapfile]
        child=launch(cmd,root/'worker.log');print(f'worker PID {child.pid}; status: ./scripts/run_experiment.sh optimize {a.name} --stage status');return
    lock=open(root/'optimization.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if not (root/'manifest.json').exists():provenance(root,a.unknown_surcharge)
    validate_source(root)
    if (root/'STOP_REQUESTED').exists():raise RuntimeError('Stop requested; remove marker only when explicitly resuming')
    # Private ROS domain and Gazebo partition, passed to all children.
    os.environ.update(ROS_DOMAIN_ID='63',NOMAD_ROS_DOMAIN_ID='63',GZ_PARTITION=f'nomad_opt_{a.name}',NOMAD_GZ_PARTITION=f'nomad_opt_{a.name}')
    if a.stage in ('baseline','validate'):
        if a.stage=='validate' and weights is None:raise ValueError('validate requires --weights wO,wS,wT')
        if (root/'runs'/a.trial/'outcome.json').exists():raise RuntimeError('Trial already recorded; choose a new trial ID')
        status(root,state='starting',trial=a.trial)
        cmd=[sys.executable,str(Path(__file__)),a.name,'--stage','live','--trial',a.trial]
        cmd+=['--unknown-surcharge',str(a.unknown_surcharge),'--cluster-weight',str(a.cluster_weight),'--cluster-radius',str(a.cluster_radius)]
        if weights:cmd+=['--weights',a.weights]
        if a.start_yaw is not None:cmd+=['--start-yaw',str(a.start_yaw)]
        cmd+=['--gpp',GPP,'--lpp',LPP,'--unknown-direction-weight',str(UNKNOWN_DIRECTION_WEIGHT),'--unknown-direction-free-angle',str(UNKNOWN_DIRECTION_ANGLE)]
        cmd+=['--cluster-influence',str(a.cluster_influence),'--cluster-saturation',str(a.cluster_saturation)]
        if a.gazebo:cmd+=['--gazebo']
        if a.rviz:cmd+=['--rviz']
        if a.mission:cmd+=['--mission',a.mission]
        end=bounded(cmd,root/'baseline.log',LIMITS['startup_wall_s']+LIMITS['trial_wall_s']+60,root)
        outcome=read(root/'runs'/a.trial/'outcome.json') if (root/'runs'/a.trial/'outcome.json').exists() else dict(result=end,valid=False)
        status(root,state='baseline_finished' if outcome.get('valid') else 'blocked',trial=a.trial,outcome=outcome,
               next_step='sweep' if outcome.get('valid') else 'resolve infrastructure failure; no automatic retry')
    elif a.stage in ('sweep','optimize'):
        files=[Path(a.mapfile)] if a.mapfile else sorted((root/'runs/baseline_01/maps').glob('*.npz'))
        if not files and not a.mapfiles:raise RuntimeError('No actual feature snapshots available')
        if a.mapfiles:files=[Path(f) for f in a.mapfiles]
        selected=files[-3:];out=root/'sweep';out.mkdir(exist_ok=True)
        configs=[(w,a.unknown_surcharge) for w in [None]+list(itertools.product([2.,4.,8.],[.5,1.,2.],[.5,1.,2.]))]
        if a.wide_sweep or a.stage=='optimize':configs=[((o,.5,.5),u) for o,u in itertools.product([2.,4.,8.,16.],[2.,4.5,9.,18.])]
        configs=[(w,u,a.cluster_weight,a.cluster_radius) for w,u in configs]
        if a.cluster_sweep:
            configs=[((8.,.5,.5),18.,0.,4.)]+[((8.,.5,.5),18.,c,r) for r,c in itertools.product([2.,4.,6.],[8.,32.,128.])]
        configs=[(*v,a.cluster_influence,a.cluster_saturation) for v in configs]
        if a.influence_sweep:
            configs=[((8.,.5,.5),18.,0.,4.,0.,1.),((8.,.5,.5),18.,128.,4.,0.,1.)]+[((8.,.5,.5),18.,c,4.,i,.4) for i,c in itertools.product([.75,1.,1.5],[4.,8.,16.])]
        if a.cluster_priority:
            configs=[((8.,.5,.5),18.,8.,4.,1.5,.4)]+[((8.,.5,.5),u,c,4.,1.5,.4) for c,u in itertools.product([32.,64.,128.],[2.,4.5,9.])]
        if a.broad_sweep:
            configs=[((o,.5,.5),u,c,4.,i,.4) for o,c,u,i in itertools.product([4.,8.],[8.,16.,32.,64.],[2.,4.5,9.],[.75,1.5])]
        save(out/'inputs.json',dict(files={str(f):sha(f) for f in selected},conditions=configs,prior_map_for_live=False))
        rows=[]
        for ci,(w,u,c,rad,influence,saturation) in enumerate(configs):
            for file in selected:
                dest=out/f'c{ci:02}_{file.stem}.json'
                if not dest.exists():
                    status(root,state='sweeping',condition=ci,total_conditions=len(configs),snapshot=file.name)
                    cmd=[sys.executable,str(Path(__file__)),a.name,'--stage','search','--mapfile',str(file),'--output',str(dest)]
                    cmd+=['--unknown-surcharge',str(u),'--cluster-weight',str(c),'--cluster-radius',str(rad)]
                    cmd+=['--gpp',GPP,'--lpp',LPP,'--unknown-direction-weight',str(UNKNOWN_DIRECTION_WEIGHT),'--unknown-direction-free-angle',str(UNKNOWN_DIRECTION_ANGLE)]
                    cmd+=['--cluster-influence',str(influence),'--cluster-saturation',str(saturation)]
                    if w:cmd+=['--weights',','.join(map(str,w))]
                    result=bounded(cmd,dest.with_suffix('.log'),LIMITS['search_wall_s'],root)
                    if result=='interrupted':status(root,state='interrupted');return
                    if not dest.exists():save(dest,dict(result=result,weights=w,unknown_surcharge=u))
                rows.append(dict(condition=ci,snapshot=file.name,**read(dest)))
        save(out/'results.json',rows);status(root,state='sweep_finished',results=str(out/'results.json'),
            next_step='Review unknown coverage and clearance; no automatic claim of optimized weights')
        if a.stage=='optimize':
            candidates=[r for r in rows if r.get('result')=='path']
            if not candidates:
                status(root,state='blocked',reason='No valid frozen-map candidate; no live trial started');return
            # Screening priority uses common physical quantities, not each
            # candidate's differently weighted objective score.
            chosen=min(candidates,key=(lambda r:r['screening_score']) if a.cluster_sweep else (lambda r:(r['unknown_length_m'],r['length_m'])))
            if a.influence_sweep:
                grouped={}
                for row in candidates:grouped.setdefault(row['condition'],[]).append(row)
                eligible={c:rs for c,rs in grouped.items() if len(rs)==len(selected)}
                if not eligible:
                    status(root,state='blocked',reason='No candidate solved all pre-entry snapshots');return
                scores={c:sum(v['length_m']+18*v['unknown_length_m']+16*v['influence_exposure_4m_1m'] for v in rs)/len(rs) for c,rs in eligible.items()}
                winner=min(scores,key=scores.get);chosen=eligible[winner][0]
                save(root/'aggregate_selection.json',dict(scores=scores,winner=winner,rule='Mean L+18U+16 integral normalized influence_4m_1m; all input snapshots required',baseline_conditions=[0,1],not_live_success=True))
            if a.cluster_priority:
                chosen,priority=select_cluster_priority(rows,[f.name for f in selected])
                save(root/'aggregate_selection.json',priority)
            if a.broad_sweep:
                ranked,balance=rank_balanced_forward(rows,[f.name for f in selected])
                chosen=ranked[0]
                save(root/'aggregate_selection.json',balance)
            save(root/'selected_candidate.json',dict(candidate=chosen,selection=('See aggregate_selection.json: forward/cluster/proximity/detour/unknown screening' if a.broad_sweep else 'See aggregate_selection.json: cluster first, unknown and length tie-break' if a.cluster_priority else 'See aggregate_selection.json: all pre-entry snapshots, common influence metric' if a.influence_sweep else 'Common score L + 18 U + 128 integral rho_4m ds; heuristic screening, no optimality proof' if a.cluster_sweep else 'Minimum unknown length, then length; screening only, not optimality proof')))
            trial='selected_01'
            if (root/'runs'/trial/'outcome.json').exists():
                outcome=read(root/'runs'/trial/'outcome.json')
            else:
                status(root,state='validating',trial=trial,weights=chosen['weights'],unknown_surcharge=chosen['unknown_surcharge'])
                cmd=[sys.executable,str(Path(__file__)),a.name,'--stage','live','--trial',trial,
                     '--weights',','.join(map(str,chosen['weights'])),'--unknown-surcharge',str(chosen['unknown_surcharge'])]
                cmd+=['--cluster-weight',str(chosen.get('cluster_weight',0.)),'--cluster-radius',str(chosen.get('cluster_radius',4.))]
                cmd+=['--gpp',GPP,'--lpp',LPP,'--unknown-direction-weight',str(UNKNOWN_DIRECTION_WEIGHT),'--unknown-direction-free-angle',str(UNKNOWN_DIRECTION_ANGLE)]
                cmd+=['--cluster-influence',str(chosen.get('cluster_influence',0.)),'--cluster-saturation',str(chosen.get('cluster_saturation',1.))]
                if a.mission:cmd+=['--mission',a.mission]
                if a.start_yaw is not None:cmd+=['--start-yaw',str(a.start_yaw)]
                if a.gazebo:cmd+=['--gazebo']
                if a.rviz:cmd+=['--rviz']
                result=bounded(cmd,root/'validation.log',LIMITS['startup_wall_s']+LIMITS['trial_wall_s']+60,root)
                outcome=read(root/'runs'/trial/'outcome.json') if (root/'runs'/trial/'outcome.json').exists() else dict(result=result,valid=False)
            status(root,state='optimization_finished',conditions=len(configs),outcome=outcome,
                   next_step='Review actual trial evidence; no production weight change')


if __name__=='__main__':
    main()
