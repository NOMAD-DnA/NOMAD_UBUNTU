"""Gazebo Harmonic world, bridges and explicit command watchdog; no MVSim node."""
import os
import shlex
import tempfile
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def prepare_gps_world(world_path, gps, directory):
    """Add NavSat to a runtime copy, preserving map geometry and vehicle physics."""
    values = {key: float(gps[key]) for key in (
        'hz', 'latitude_deg', 'longitude_deg', 'elevation_m',
        'mount_x_m', 'mount_y_m', 'mount_z_m')}
    if not all(math.isfinite(v) for v in values.values()) or values['hz'] <= 0:
        raise ValueError('GPS values must be finite and hz must be positive')
    if not -90 <= values['latitude_deg'] <= 90 or not -180 <= values['longitude_deg'] <= 180:
        raise ValueError('GPS origin is outside latitude/longitude bounds')
    noise_sigmas = {axis: float(gps.get('position_noise_' + axis + '_sigma_m', 0.0))
                    for axis in ('horizontal', 'vertical')}
    if any(not math.isfinite(v) or v < 0 for v in noise_sigmas.values()):
        raise ValueError('GPS position noise sigma must be finite and nonnegative (meters)')
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    # Resolve local resources before relocating the world and included vehicle.
    def resolve_uris(root, base):
        for uri in root.iter('uri'):
            text = (uri.text or '').strip()
            if text and '://' not in text and not Path(text).is_absolute():
                uri.text = str((base / text).resolve())
    resolve_uris(world, world_path.parent)
    includes = [node for node in world.findall('include')
                if node.findtext('name') == 'nomad_vehicle']
    inline_models = world.findall("model[@name='nomad_vehicle']")
    if len(includes) + len(inline_models) != 1:
        raise ValueError('GPS requires one nomad_vehicle model')
    vehicle_tree = None
    if includes:
        include = includes[0]
        vehicle_uri = include.findtext('uri')
        vehicle_path = (world_path.parent.parent / 'models' / 'nomad_vehicle' / 'model.sdf'
                        if vehicle_uri == 'model://nomad_vehicle' else Path(vehicle_uri))
        vehicle_tree = ET.parse(vehicle_path)
        model = vehicle_tree.getroot().find('model')
        resolve_uris(model, vehicle_path.parent)
    else:
        model = inline_models[0]
    base = model.find("link[@name='base_link']")
    if base is None or model.find(".//sensor[@name='nomad_gps']") is not None:
        raise ValueError('Missing base_link or duplicate nomad_gps sensor')
    sensor = ET.SubElement(base, 'sensor', name='nomad_gps', type='navsat')
    xyz = ' '.join(str(values['mount_' + axis + '_m']) for axis in 'xyz')
    for tag, value in [('pose', xyz + ' 0 0 0'), ('always_on', 'true'),
                       ('update_rate', str(values['hz'])), ('topic', '/nomad/raw/gps/fix'),
                       ('gz_frame_id', 'gps_link')]:
        ET.SubElement(sensor, tag).text = value
    navsat = ET.SubElement(sensor, 'navsat')
    position = ET.SubElement(navsat, 'position_sensing')
    for axis, sigma in noise_sigmas.items():
        direction = ET.SubElement(position, axis)
        noise = ET.SubElement(direction, 'noise', type='gaussian')
        ET.SubElement(noise, 'mean').text = '0'
        ET.SubElement(noise, 'stddev').text = str(sigma)
    coordinates = world.find('spherical_coordinates')
    if coordinates is None:
        coordinates = ET.SubElement(world, 'spherical_coordinates')
        for tag, value in [('surface_model', 'EARTH_WGS84'), ('world_frame_orientation', 'ENU'),
                           ('latitude_deg', values['latitude_deg']),
                           ('longitude_deg', values['longitude_deg']),
                           ('elevation', values['elevation_m']), ('heading_deg', 0)]:
            ET.SubElement(coordinates, tag).text = str(value)
    if world.find("plugin[@name='gz::sim::systems::NavSat']") is None:
        ET.SubElement(world, 'plugin', filename='gz-sim-navsat-system',
                      name='gz::sim::systems::NavSat')
    if vehicle_tree is not None:
        vehicle_target = directory / 'vehicle.sdf'
        vehicle_tree.write(vehicle_target, encoding='utf-8', xml_declaration=True)
        include.find('uri').text = str(vehicle_target)
    target = directory / 'world.sdf'
    tree.write(target, encoding='utf-8', xml_declaration=True)
    return target, xyz


def launch_nodes(context):
    share = Path(get_package_share_directory('nomad_gazebo'))
    custom_world = LaunchConfiguration('world_file').perform(context).strip()
    vegetation = LaunchConfiguration('vegetation').perform(context).lower() in ('true', '1', 'yes')
    world = (Path(custom_world).resolve() if custom_world else
             share/'worlds'/('forest.sdf' if vegetation else 'forest_bare.sdf'))
    configuration_path = Path(LaunchConfiguration('sensor_config').perform(context)).resolve()
    settings = yaml.safe_load(configuration_path.read_text(encoding='utf-8'))
    # Camera/lidar SDF are generated; GPS is composed into a runtime copy below.
    gps_directory = None
    gps_mount = None
    original_world = world
    if settings.get('gps', {}).get('enabled', False):
        gps_directory = tempfile.TemporaryDirectory(prefix='nomad-gps-')
        world, gps_mount = prepare_gps_world(world, settings['gps'], Path(gps_directory.name))
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
    if gps_mount is None:
        bridge_entries = [entry for entry in bridge_entries if entry['ros_topic_name'] != '/gps/fix']
    tf_enabled = LaunchConfiguration('publish_ground_truth_tf').perform(context).lower() in ('true','1','yes')
    if not tf_enabled:
        bridge_entries = [entry for entry in bridge_entries if entry['ros_topic_name'] != '/tf']
    # Only remove the ground-truth dynamic TF bridge. Sensor static TF remains.
    handle = tempfile.NamedTemporaryFile(mode='w', prefix='nomad-bridge-', suffix='.yaml', delete=False)
    yaml.safe_dump(bridge_entries, handle)
    handle.close()
    def cleanup_files(context):
        Path(handle.name).unlink(missing_ok=True)
        if gps_directory is not None:
            gps_directory.cleanup()
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_files)]))
    map_root = original_world.parent.parent
    resources = [str(share), str(share/'models'), str(original_world.parent), str(map_root), str(map_root/'models')]
    robot = ET.fromstring((share/'urdf/nomad_vehicle.urdf').read_text(encoding='utf-8'))
    if gps_mount is not None:
        ET.SubElement(robot, 'link', name='gps_link')
        joint = ET.SubElement(robot, 'joint', name='gps_mount', type='fixed')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link='gps_link')
        ET.SubElement(joint, 'origin', xyz=gps_mount, rpy='0 0 0')
    gps_nodes = []
    if gps_mount is not None and settings['gps'].get('visualization', True):
        datum = ET.parse(world).find('world/spherical_coordinates')
        if datum.findtext('world_frame_orientation', 'ENU') != 'ENU' or float(datum.findtext('heading_deg', '0')) != 0:
            raise ValueError('GPS visualization requires an ENU world with heading_deg=0')
        gps_nodes = [Node(package='nomad_gazebo', executable='gps_visualizer.py', name='nomad_gps_visualizer',
                          parameters=[{'use_sim_time': True, 'fixed_frame': 'odom',
                                       'origin_latitude_deg': float(datum.findtext('latitude_deg')),
                                       'origin_longitude_deg': float(datum.findtext('longitude_deg')),
                                       'origin_elevation_m': float(datum.findtext('elevation'))}],
                          output='screen', remappings=remaps)]
    return gps_nodes + [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resources)),
            SetEnvironmentVariable('SDF_PATH', str(original_world.parent)+os.pathsep+os.environ.get('SDF_PATH', '')),
            cleanup, gz,
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='nomad_gz_bridge',
                 parameters=[{'config_file': handle.name, 'use_sim_time': True}], output='screen', remappings=remaps),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='nomad_robot_state_publisher',
                 parameters=[{'robot_description': ET.tostring(robot, encoding='unicode'),
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
