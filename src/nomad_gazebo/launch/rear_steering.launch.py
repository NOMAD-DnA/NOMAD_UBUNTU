"""Isolated, opt-in rear-steering physical simulation. No planning/VIO launch."""
import importlib.util
import json
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    share=Path(get_package_share_directory('nomad_gazebo'))
    lib=Path(get_package_prefix('nomad_gazebo'))/'lib/nomad_gazebo'
    spec=importlib.util.spec_from_file_location('rear_vehicle_generator',lib/'generate_vehicle.py')
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    given=LaunchConfiguration('run_directory').perform(context).strip()
    directory=Path(given).expanduser().resolve() if given else Path(tempfile.mkdtemp(prefix='nomad-rear-steer-'))
    if given:directory.mkdir(parents=True,exist_ok=False)
    from nomad_vehicle_model import load_profile
    values=load_profile(Path(get_package_share_directory('nomad_bringup'))/'config/vehicle.yaml')
    metadata=generator.generate_vehicle(directory,share/'config/sensors.yaml',with_sensors=False,
                                        drive_layout='front_drive_rear_steer',vehicle_config=values)
    model=ET.parse(directory/'models/nomad_vehicle/model.sdf').find('model')
    ET.SubElement(model,'pose').text='0 0 .025 0 0 0'
    root=ET.fromstring('''<sdf version="1.10"><world name="rear_steering_test">
      <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><direction>-.5 .1 -1</direction></light>
      <model name="ground"><static>true</static><link name="ground">
        <collision name="ground"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry></collision>
        <visual name="ground"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry><material><diffuse>.55 .62 .43 1</diffuse></material></visual>
      </link></model></world></sdf>''')
    root.find('world').append(model)
    world=directory/'world.sdf';generator.write_xml(world,root)
    entries=[]
    def bridge(ros,gz,ros_type,gz_type,direction):
        entries.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=ros_type,gz_type_name=gz_type,direction=direction))
    bridge('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS')
    bridge('/odom',metadata['native_topics']['odom'],'nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS')
    bridge('/tf',metadata['native_topics']['tf'],'tf2_msgs/msg/TFMessage','gz.msgs.Pose_V','GZ_TO_ROS')
    bridge('/joint_states',metadata['native_topics']['joint_states'],'sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS')
    for name in ('front_left_velocity','front_right_velocity','rear_left_position','rear_right_position'):
        topic='/nomad/rear_steer/actuators/'+name
        bridge(topic,topic,'std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ')
    config=directory/'bridge.yaml';config.write_text(yaml.safe_dump(entries))
    headless=LaunchConfiguration('headless').perform(context).lower() in ('true','1','yes')
    gz=ExecuteProcess(cmd=['gz','sim','-r',*(['-s'] if headless else []),'-v','2',str(world)],output='screen')
    transport=Node(package='ros_gz_bridge',executable='parameter_bridge',name='rear_steer_bridge',
                   parameters=[{'config_file':str(config),'use_sim_time':True}],output='screen')
    mixer=Node(package='nomad_gazebo',executable='rear_steering_mixer.py',
               parameters=[str(share/'config/rear_steering.yaml'),{'use_sim_time':True,
                   'wheelbase':metadata['wheelbase_m'],'track':metadata['track_m'],
                   'wheel_radius':metadata['wheel_radius_m'],'joint_limit':metadata['steering_limit_rad'],
                   'max_speed':metadata['max_speed_mps']}],output='screen')
    robot=Node(package='robot_state_publisher',executable='robot_state_publisher',
               parameters=[{'use_sim_time':True,'robot_description':(directory/'urdf/nomad_vehicle.urdf').read_text()}])
    handlers=[RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[
                EmitEvent(event=Shutdown(reason='rear-steering simulation component exited'))]))
              for node in (gz,mixer,transport)]
    print(json.dumps(dict(rear_steering_run=str(directory),provisional_geometry=True),ensure_ascii=False))
    return handlers+[gz,transport,robot,mixer]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('headless',default_value='true'),
        DeclareLaunchArgument('run_directory',default_value=''),OpaqueFunction(function=launch_nodes)])
