"""Use the shared NOMAD sensor launch with the generated forest world."""
import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def ensure_mvsim_port_available(_context):
    # MVSim 1.4's ZMQ server is independent of ROS_DOMAIN_ID and uses this
    # fixed port. A different ROS domain does not isolate two MVSim servers.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.3)
        if probe.connect_ex(('127.0.0.1', 23700)) == 0:
            raise RuntimeError(
                'MVSim port 23700 is already in use. Stop the existing simulator '
                'with Ctrl+C before launching the forest world. No process was stopped.')
    return []


def generate_launch_description():
    share = get_package_share_directory("nomad_sim")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="False"),
        DeclareLaunchArgument("sensor_config", default_value=os.path.join(share, "config", "sensors.yaml")),
        DeclareLaunchArgument("world_file", default_value=os.path.join(share, "worlds", "forest.world.xml")),
        OpaqueFunction(function=ensure_mvsim_port_available),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "elevation.launch.py")),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "sensor_config": LaunchConfiguration("sensor_config"),
                "world_file": LaunchConfiguration("world_file"),
            }.items(),
        ),
    ])
