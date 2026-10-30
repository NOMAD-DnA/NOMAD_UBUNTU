"""Only real perception and optional OAK capture. No Gazebo, VIO or Control."""
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnShutdown,OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    value=lambda n:LaunchConfiguration(n).perform(context)
    from nomad_perception.real.semantic_validation import load_config
    load_config(value('semantic_config'))
    metadata=value('camera_metadata_file');actions=[]
    if value('start_oak_capture').lower()=='true':
        state=tempfile.TemporaryDirectory(prefix='nomad_oak_capture_')
        metadata=str(Path(state.name)/'metadata.json')
        actions.extend([
            Node(package='nomad_perception',executable='real_oak_capture',
                 arguments=['--output',state.name,'--ready-file',str(Path(state.name)/'ready.json'),
                            '--duration','0','--publish-cloud','--usb',value('oak_usb_speed'),
                            '--imu-extrinsics','nominal'],output='screen'),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda context:state.cleanup())]))])
    if not metadata:raise ValueError('Provide camera_metadata_file or set start_oak_capture:=true')
    actions.append(Node(package='nomad_perception',executable='real_perception',output='screen',
        parameters=[{'use_sim_time':False,'camera_metadata_file':metadata,
                     'semantic_config':value('semantic_config'),'pose_config':value('pose_config'),
                     'lidar_calibration':value('lidar_calibration'),'slope_profile':'12cm_3s_optimized',
                     'scan_topic':value('scan_topic'),'lidar_points_topic':value('lidar_points_topic'),
                     'oak_depth_topic':value('oak_depth_topic')}]))
    # Either owned process exiting ends this launch and releases its temporary state.
    exits=[RegisterEventHandler(OnProcessExit(target_action=action,
        on_exit=[EmitEvent(event=Shutdown(reason='Real perception process exited'))]))
        for action in actions if isinstance(action,Node)]
    return exits+actions


def generate_launch_description():
    root=Path(get_package_share_directory('nomad_perception'))/'config/real'
    defaults=dict(semantic_config=str(root/'semantic.real.json'),pose_config=str(root/'pi5_pose.json'),
                  lidar_calibration=str(root/'oak_g2b_selected_20261030.json'),
                  camera_metadata_file='',start_oak_capture='false',oak_usb_speed='HIGH',
                  scan_topic='/scan',lidar_points_topic='/point_cloud',oak_depth_topic='/oak/depth/points')
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],OpaqueFunction(function=start)])
