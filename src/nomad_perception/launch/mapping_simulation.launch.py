"""Simulation sensors and semantic points for either mapping module."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from nomad_perception.mapping_simulation import simulation_actions


def start(context):
    return simulation_actions(lambda name: LaunchConfiguration(name).perform(context))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('semantic_source', default_value='simulation'),
        DeclareLaunchArgument('world_file', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('vegetation', default_value='true'),
        OpaqueFunction(function=start),
    ])
