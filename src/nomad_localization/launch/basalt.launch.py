from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
ROOT=Path(__file__).resolve().parent
def generate_launch_description():
    import os
    config=Path(os.environ['NOMAD_VIO_CONFIG'])
    params={'calibration_file':str(config/'nomad_calib.json'),'vio_config_file':str(config/'nomad_vio_config.json'),'use_sim_time':os.environ.get('VIO_USE_SIM_TIME','true').lower()=='true'}
    return LaunchDescription([ComposableNodeContainer(name='basalt_vio_container',namespace='basalt',package='rclcpp_components',executable='component_container',output='screen',composable_node_descriptions=[
        ComposableNode(package='basalt_ros',plugin='basalt_ros::VIOFrontEndNode',name='vio_frontend',namespace='basalt',parameters=[params],remappings=[('left_image','/oak/left/image_rect'),('right_image','/oak/right/image_rect')]),
        ComposableNode(package='basalt_ros',plugin='basalt_ros::VIOBackEndNode',name='vio_backend',namespace='basalt',parameters=[params],remappings=[('imu','/oak/imu')])])])
