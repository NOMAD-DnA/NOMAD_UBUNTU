"""Optional GNSS/VIO provider. GT remains the default."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    backend=Path(get_package_share_directory('nomad_localization'))
    vio=Path(get_package_share_directory('nomad_vio'))
    lc=LaunchConfiguration
    boolean=lambda n:ParameterValue(lc(n),value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(backend/'config/fusion.yaml')),
        DeclareLaunchArgument('adapter_config',default_value=str(vio/'config/fusion_adapter.yaml')),
        DeclareLaunchArgument('method',default_value='basalt',choices=['basalt','openvins']),
        DeclareLaunchArgument('manage_vio',default_value='false'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('publish_tf',default_value='true'),
        DeclareLaunchArgument('auto_restart',default_value='false'),
        DeclareLaunchArgument('stop_before_restart',default_value='true'),
        DeclareLaunchArgument('gps_topic',default_value='/gps/fix'),
        DeclareLaunchArgument('quality_topic',default_value='/gps/quality'),
        DeclareLaunchArgument('vio_log_directory',default_value=str(Path.home()/'.ros/nomad_vio')),
        DeclareLaunchArgument('vio_config_directory',default_value=[str(vio/'config/vio'),'/',lc('method')]),
        Node(package='nomad_localization',executable='fusion_node',name='fusion',output='screen',parameters=[lc('config'),{
            'use_sim_time':boolean('use_sim_time'),'gps_topic':lc('gps_topic'),'quality_topic':lc('quality_topic'),'auto_restart':boolean('auto_restart')}]),
        Node(package='nomad_localization',executable='vio_supervisor.py',name='vio_supervisor',output='screen',parameters=[{
            'method':lc('method'),'manage_vio':boolean('manage_vio'),'stop_before_restart':boolean('stop_before_restart'),
            'use_sim_time':boolean('use_sim_time'),'use_estimator_sim_time':boolean('use_sim_time'),
            'vio_config_directory':lc('vio_config_directory'),'log_directory':lc('vio_log_directory')}]),
        Node(package='nomad_vio',executable='fusion_adapter',name='nomad_fusion_adapter',output='screen',parameters=[lc('adapter_config'),{
            'use_sim_time':boolean('use_sim_time'),'publish_tf':boolean('publish_tf')}]),
    ])
