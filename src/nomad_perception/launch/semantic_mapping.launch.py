"""The original simulator and semantic input with a separate semantic mapper."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('nomad_perception'))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'mapping_simulation.launch.py')),
        launch_arguments={
            'world_file': value('world_file'),
            'headless': value('headless'),
            'vegetation': value('vegetation'),
            'semantic_source': 'simulation',
        }.items())
    mapper = Node(
        package='nomad_perception', executable='semantic_mapper',
        name='nomad_semantic_mapper', output='screen',
        parameters=[str(share / 'config' / 'semantic_mapping.yaml'), {
            'use_sim_time': True,
            'session_directory': value('session_directory'),
            'world_file': value('world_file'),
            'map_id': value('map_id'),
        }],
        sigterm_timeout='30', sigkill_timeout='40',
        on_exit=Shutdown(reason='Semantic mapper closed'))
    labels = Node(
        package='nomad_perception', executable='semantic_labels',
        name='nomad_semantic_labels', output='screen',
        parameters=[{'use_sim_time': True}],
        on_exit=Shutdown(reason='Semantic label display closed'))
    actions = [SetEnvironmentVariable('NOMAD_STAGE1_PIXEL_OFFSET', '0.5'), simulation, mapper, labels]
    if value('rviz').lower() in ('true', '1', 'yes'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='nomad_semantic_mapping_rviz',
            output='screen',
            arguments=['-d', str(share / 'rviz' / 'semantic_mapping.rviz')],
            parameters=[{'use_sim_time': True}],
            on_exit=Shutdown(reason='Mapping RViz closed')))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_directory'),
        DeclareLaunchArgument('world_file'),
        DeclareLaunchArgument('map_id'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('vegetation', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=start),
    ])
