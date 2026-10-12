"""Simulation input shared by the independent slope and semantic mappers."""
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from .semantic_world import instrument_world


def simulation_actions(values):
    gazebo_share = Path(get_package_share_directory('nomad_gazebo'))
    perception_share = Path(get_package_share_directory('nomad_perception'))
    custom = values('world_file').strip()
    vegetation = values('vegetation').lower() in ('true', '1', 'yes')
    world = (Path(custom).resolve() if custom else gazebo_share / 'worlds' /
             ('forest.sdf' if vegetation else 'forest_bare.sdf'))
    source = values('semantic_source')
    if source not in ('simulation', 'external'):
        raise ValueError('semantic_source must be simulation or external')

    actions = []
    if source == 'simulation':
        directory = tempfile.TemporaryDirectory(prefix='nomad-mapping-world-')
        try:
            world, bridges = instrument_world(
                world, gazebo_share / 'models/nomad_vehicle/model.sdf',
                Path(directory.name))
            bridge_file = Path(directory.name) / 'semantic_bridge.yaml'
            bridge_file.write_text(yaml.safe_dump(bridges), encoding='utf-8')
        except BaseException:
            directory.cleanup()
            raise

        def cleanup(context):
            directory.cleanup()

        actions.extend([
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='nomad_mapping_semantic_bridge', output='screen',
                 parameters=[{'config_file': str(bridge_file), 'use_sim_time': True}]),
            Node(package='nomad_perception', executable='semantic_points', output='screen',
                 parameters=[str(perception_share / 'config/semantic_points.yaml')]),
        ])

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch/forest.launch.py')),
        launch_arguments={
            'headless': values('headless'), 'vegetation': values('vegetation'),
            'world_file': str(world), 'publish_ground_truth_tf': 'true',
        }.items()))
    return actions
