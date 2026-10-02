"""Gazebo Harmonic world, bridges and explicit command watchdog; no MVSim node."""
import os
import shlex
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnShutdown
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
        launch_arguments={'gz_args': shlex.join(arguments), 'gz_version': '8', 'on_exit_shutdown': 'true'}.items())
    lidar = settings['lidar']
    remap_dict = yaml.safe_load(LaunchConfiguration('ros_remappings').perform(context))
    if not isinstance(remap_dict, dict) or not all(isinstance(k,str) and isinstance(v,str) for k,v in remap_dict.items()):
        raise ValueError('ros_remappings must be a topic-name mapping')
    remaps = list(remap_dict.items())
    bridge_entries = yaml.safe_load((share/'config/bridge.yaml').read_text())
    tf_enabled = LaunchConfiguration('publish_ground_truth_tf').perform(context).lower() in ('true','1','yes')
    if not tf_enabled:
        bridge_entries = [entry for entry in bridge_entries if entry['ros_topic_name'] != '/tf']
    # Only remove the ground-truth dynamic TF bridge. Sensor static TF remains.
    handle = tempfile.NamedTemporaryFile(mode='w', prefix='nomad-bridge-', suffix='.yaml', delete=False)
    yaml.safe_dump(bridge_entries, handle)
    handle.close()
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
        function=lambda context: Path(handle.name).unlink(missing_ok=True))]))
    map_root = world.parent.parent
    resources = [str(share), str(share/'models'), str(world.parent), str(map_root), str(map_root/'models')]
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resources)),
            SetEnvironmentVariable('SDF_PATH', str(world.parent)+os.pathsep+os.environ.get('SDF_PATH', '')),
            cleanup, gz,
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='nomad_gz_bridge',
                 parameters=[{'config_file': handle.name, 'use_sim_time': True}], output='screen', remappings=remaps),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='nomad_robot_state_publisher',
                 parameters=[{'robot_description': (share/'urdf/nomad_vehicle.urdf').read_text(encoding='utf-8'),
                              'use_sim_time': True}], output='screen', remappings=remaps),
            Node(package='nomad_gazebo', executable='gazebo_sensor_adapter.py', name='nomad_gz_sensors',
                 parameters=[{'sensors_config': str(configuration_path), 'use_sim_time': True}], output='screen', remappings=remaps),
            Node(package='nomad_gazebo', executable='lidar_bridge.py', name='nomad_gz_lidar',
                 parameters=[{'min_range_m': float(lidar['min_range_m']), 'max_range_m': float(lidar['max_range_m']),
                              'scan_hz': float(lidar['hz']), 'use_sim_time': True}], output='screen', remappings=remaps),
            Node(package='nomad_gazebo', executable='command_watchdog.py', name='nomad_cmd_watchdog', output='screen', remappings=remaps)]


def generate_launch_description():
    share = get_package_share_directory('nomad_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('publish_ground_truth_tf', default_value='true'),
        DeclareLaunchArgument('ros_remappings', default_value='{}'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('vegetation', default_value='True',
                             description='Sparse forest by default; False disables vegetation.'),
        DeclareLaunchArgument('world_file', default_value='',
                             description='Optional explicit SDF; otherwise select by vegetation.'),
        DeclareLaunchArgument('sensor_config', default_value=os.path.join(share, 'config/sensors.yaml')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share+os.pathsep+os.path.join(share, 'models')),
        OpaqueFunction(function=launch_nodes)])
