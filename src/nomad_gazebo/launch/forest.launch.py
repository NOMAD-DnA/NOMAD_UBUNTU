"""Gazebo Harmonic world, bridges and explicit command watchdog; no MVSim node."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_nodes(context):
    share = Path(get_package_share_directory('nomad_gazebo'))
    custom_world = LaunchConfiguration('world_file').perform(context).strip()
    vegetation = LaunchConfiguration('vegetation').perform(context).lower() in ('true', '1', 'yes')
    world = (Path(custom_world).resolve() if custom_world else
             share/'worlds'/('forest.sdf' if vegetation else 'forest_bare.sdf'))
    configuration_path = Path(LaunchConfiguration('sensor_config').perform(context)).resolve()
    settings = yaml.safe_load(configuration_path.read_text(encoding='utf-8'))
    # SDF sensors are generated from this profile. Regenerate before changing
    # dimensions/FOV/mounts; do not pretend runtime bridge settings change SDF.
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    arguments = ['-r', '-v', '2']
    if headless:
        arguments += ['-s', '--headless-rendering']
    arguments += [str(world)]
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': ' '.join(arguments), 'gz_version': '8', 'on_exit_shutdown': 'true'}.items())
    lidar = settings['lidar']
    return [gz,
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='nomad_gz_bridge',
                 parameters=[{'config_file': str(share/'config/bridge.yaml'), 'use_sim_time': True}], output='screen'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='nomad_robot_state_publisher',
                 parameters=[{'robot_description': (share/'urdf/nomad_vehicle.urdf').read_text(encoding='utf-8'),
                              'use_sim_time': True}], output='screen'),
            Node(package='nomad_gazebo', executable='gazebo_sensor_adapter.py', name='nomad_gz_sensors',
                 parameters=[{'sensors_config': str(configuration_path), 'use_sim_time': True}], output='screen'),
            Node(package='nomad_gazebo', executable='lidar_bridge.py', name='nomad_gz_lidar',
                 parameters=[{'min_range_m': float(lidar['min_range_m']), 'max_range_m': float(lidar['max_range_m']),
                              'scan_hz': float(lidar['hz']), 'use_sim_time': True}], output='screen'),
            Node(package='nomad_gazebo', executable='command_watchdog.py', name='nomad_cmd_watchdog', output='screen')]


def generate_launch_description():
    share = get_package_share_directory('nomad_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('vegetation', default_value='True',
                             description='Sparse forest by default; False disables vegetation.'),
        DeclareLaunchArgument('world_file', default_value='',
                             description='Optional explicit SDF; otherwise select by vegetation.'),
        DeclareLaunchArgument('sensor_config', default_value=os.path.join(share, 'config/sensors.yaml')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share+os.pathsep+os.path.join(share, 'models')),
        OpaqueFunction(function=launch_nodes)])
