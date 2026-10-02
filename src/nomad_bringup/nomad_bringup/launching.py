"""Launch any module independently or the complete replaceable stack."""
from pathlib import Path
import fcntl
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from .configuration import load_topics,load_modules,remappings,selected_specs

_locks = []


def start(context,module):
    value = lambda name: LaunchConfiguration(name).perform(context)
    topics,providers = load_topics(value('topics_file')),load_modules(value('modules_file'))
    legacy_command_topic = value("cmd_vel_topic").strip()
    if legacy_command_topic:
        import re
        if not re.fullmatch(r"(?:/[A-Za-z_][A-Za-z_0-9]*)+", legacy_command_topic):
            raise ValueError("cmd_vel_topic must be an absolute ROS topic name")
        topics["control.simulator_command"] = legacy_command_topic
    specs = selected_specs(providers,module)
    domain = os.getenv('ROS_DOMAIN_ID','0')
    # All-stack and single-module launches share locks, so they cannot duplicate nodes.
    roles = {'nomad_perception':'perception','nomad_vio':'vio',
             'nomad_control':'control','nomad_path_planning':'planning'}
    for name in sorted({roles[package] for package,_,_ in specs}):
        lock = open(f'/tmp/nomad_module_{os.getuid()}_{domain}_{name}.lock','a')
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:
            lock.close()
            raise RuntimeError(f'{name} already launched in ROS domain {domain}') from exc
        _locks.append(lock)
    nodes = []
    override = value('params_file').strip()
    for package,executable,role in specs:
        share = Path(get_package_share_directory(package))
        config = 'planning.yaml' if package=='nomad_path_planning' else 'parameters.yaml'
        params = [str(share/'config'/config),value('vehicle_file')]
        if override:
            params.append(override)
        nodes.append(Node(package=package,executable=executable,output='screen',
                          parameters=params,remappings=remappings(topics,role),
                          on_exit=Shutdown(reason=f'{package}/{executable} exited')))
    if module in ('all','planning') and value('rviz').lower()=='true':
        share = Path(get_package_share_directory('nomad_path_planning'))
        nodes.append(Node(package='rviz2',executable='rviz2',
                          arguments=['-d',str(share/'rviz/planning.rviz')],
                          parameters=[value('vehicle_file')],remappings=remappings(topics)))
    return nodes


def description(module='all'):
    share = Path(get_package_share_directory('nomad_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('topics_file',default_value=str(share/'config/topics.yaml')),
        DeclareLaunchArgument('modules_file',default_value=str(share/'config/modules.yaml')),
        DeclareLaunchArgument('vehicle_file',default_value=str(share/'config/vehicle.yaml')),
        DeclareLaunchArgument('params_file',default_value='',description='Optional ROS parameter overrides'),
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic',default_value='',description='Legacy actuator-output override; prefer topics_file'),
        OpaqueFunction(function=lambda context:start(context,module)),
    ])
