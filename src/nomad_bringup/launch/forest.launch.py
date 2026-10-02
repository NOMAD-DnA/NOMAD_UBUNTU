"""Whole simulator stack with exactly one localization TF owner."""
from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nomad_bringup.configuration import load_topics,load_modules,remappings


def start(context):
    value = lambda name:LaunchConfiguration(name).perform(context)
    topics,providers = load_topics(value('topics_file')),load_modules(value('modules_file'))
    def include(package,file,args):
        share=Path(get_package_share_directory(package))
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch'/file)),
                                        launch_arguments=args.items())
    args={key:value(key) for key in ('topics_file','modules_file','vehicle_file','params_file','rviz','cmd_vel_topic')}
    return [include('nomad_gazebo','forest.launch.py',{
                'headless':value('headless'),'vegetation':value('vegetation'),
                'publish_ground_truth_tf':'true' if providers['tf_owner']=='gazebo' else 'false',
                'ros_remappings':json.dumps(dict(remappings(topics)))}),
            include('nomad_bringup','autonomy.launch.py',args)]


def generate_launch_description():
    share=Path(get_package_share_directory('nomad_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('topics_file',default_value=str(share/'config/topics.yaml')),
        DeclareLaunchArgument('modules_file',default_value=str(share/'config/modules.yaml')),
        DeclareLaunchArgument('vehicle_file',default_value=str(share/'config/vehicle.yaml')),
        DeclareLaunchArgument('params_file',default_value=''),
        DeclareLaunchArgument('cmd_vel_topic',default_value=''),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('headless',default_value='false'),
        DeclareLaunchArgument('vegetation',default_value='true'),
        OpaqueFunction(function=start)])
