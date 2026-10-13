"""Optional GUI visualization wrapper; module commands remain unchanged."""
import json
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch_ros.actions import Node


def start(context):
    values = context.launch_configurations
    module = values['module']
    if module not in ('localization', 'map', 'planning', 'control'):
        raise ValueError('Unsupported GUI module')
    args = json.loads(values['module_args'])
    if not isinstance(args, list) or not all(isinstance(a, str) for a in args):
        raise ValueError('module_args must be a string list')
    script = Path(os.environ['NOMAD_WS_ROOT']) / 'scripts/run_module.sh'
    actions = [ExecuteProcess(cmd=[str(script), module, *args], output='screen', on_exit=Shutdown(reason='Module stopped'))]
    if values['rviz'].lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', values['rviz_config']], parameters=[{'use_sim_time': True}], output='screen'))
    if module == 'map' and values['map_view'].lower() == 'true':
        actions.append(Node(package='nomad_perception', executable='terrain_viewer', parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('module'), DeclareLaunchArgument('module_args', default_value='[]'),
        DeclareLaunchArgument('rviz', default_value='false'), DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('map_view', default_value='false'), OpaqueFunction(function=start),
    ])
