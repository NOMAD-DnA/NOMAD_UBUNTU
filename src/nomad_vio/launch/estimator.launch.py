from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=Path(get_package_share_directory('nomad_vio'));lc=LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('method',default_value='basalt',choices=['basalt','openvins']),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('vio_config_directory',default_value=[str(share/'config/vio'),'/',lc('method')]),
        Node(package='nomad_localization',executable='vio_supervisor.py',output='screen',parameters=[{
            'method':lc('method'),'manage_vio':True,
            'use_sim_time':ParameterValue(lc('use_sim_time'),value_type=bool),
            'use_estimator_sim_time':ParameterValue(lc('use_sim_time'),value_type=bool),
            'vio_config_directory':lc('vio_config_directory')}]),
    ])
