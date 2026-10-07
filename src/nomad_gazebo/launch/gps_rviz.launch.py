"""Open only RViz; use the GPS visualizer already started by forest.launch.py."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('nomad_gazebo')) / 'config/gps.rviz'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rviz2', executable='rviz2', name='nomad_gps_rviz',
             arguments=['-d', str(config)],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen')])
