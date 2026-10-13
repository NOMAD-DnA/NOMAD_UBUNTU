"""Opt-in terrain/fusion stack, existing Planning and Control unmodified."""

import os
from pathlib import Path
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = lambda p: Path(get_package_share_directory(p))
    lc = LaunchConfiguration

    def include(p, file, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share(p) / "launch" / file)),
            launch_arguments=args.items(),
        )

    root = Path(os.environ.get("NOMAD_WS_ROOT", str(Path.cwd())))
    directory = (
        root / "sessions/terrain_navigation" / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=""),
            DeclareLaunchArgument("record_directory", default_value=str(directory)),
            DeclareLaunchArgument("vio_log_directory", default_value=str(Path.home()/".ros/nomad_vio")),
            DeclareLaunchArgument("ros_remappings", default_value="{}"),
            DeclareLaunchArgument("vio_topic", default_value="/fusion/vio_input"),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("quality_topic", default_value="/gps/quality"),
            DeclareLaunchArgument("sensor_config", default_value=str(share("nomad_gazebo") / "config/sensors.yaml")),
            DeclareLaunchArgument("robot_description_file", default_value=""),
            DeclareLaunchArgument("world_resource_root", default_value=""),
            DeclareLaunchArgument("adapter_config", default_value=str(share("nomad_vio") / "config/fusion_adapter.yaml")),
            DeclareLaunchArgument("vio_config_directory", default_value=[str(share("nomad_vio") / "config/vio"), "/", lc("method")]),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("initial_teleop", default_value="true"),
            DeclareLaunchArgument(
                "terrain_config",
                default_value=str(
                    share("nomad_perception") / "config/navigation_terrain.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "fusion_config",
                default_value=str(share("nomad_localization") / "config/fusion.yaml"),
            ),
            DeclareLaunchArgument(
                "semantic_model",
                default_value=str(
                    share("nomad_semantic") / "models/nomad_b0/model.onnx"
                ),
            ),
            DeclareLaunchArgument("semantic_backend", default_value="cuda"),
            DeclareLaunchArgument("semantic_rate", default_value="10.0"),
            DeclareLaunchArgument("semantic_view", default_value="false"),
            DeclareLaunchArgument("gpp", default_value=""),
            DeclareLaunchArgument("lpp", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    share("nomad_perception") / "config/terrain_planning.yaml"
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=str(share("nomad_perception") / "rviz/terrain_navigation.rviz")),
            DeclareLaunchArgument("viewer", default_value="true"),
            DeclareLaunchArgument(
                "semantic_script",
                default_value=str(
                    Path(get_package_share_directory("nomad_semantic"))
                    / "scripts/run.sh"
                ),
            ),
            DeclareLaunchArgument(
                "method", default_value="basalt", choices=["basalt", "openvins"]
            ),
            include(
                "nomad_gazebo",
                "forest.launch.py",
                {
                    "world_file": lc("world_file"),
                    "sensor_config": lc("sensor_config"),
                    "ros_remappings": lc("ros_remappings"),
                    "robot_description_file": lc("robot_description_file"),
                    "world_resource_root": lc("world_resource_root"),
                    "headless": lc("headless"),
                    "publish_ground_truth_tf": "false",
                },
            ),
            include(
                "nomad_vio",
                "fusion.launch.py",
                {
                    "method": lc("method"),
                    "vio_log_directory": lc("vio_log_directory"),
                    "manage_vio": "true",
                    "auto_restart": "false",
                    "output_warmup_s": "2.0",
                    "config": lc("fusion_config"),
                    "vio_topic": lc("vio_topic"),
                    "gps_topic": lc("gps_topic"),
                    "quality_topic": lc("quality_topic"),
                    "adapter_config": lc("adapter_config"),
                    "vio_config_directory": lc("vio_config_directory"),
                },
            ),
            ExecuteProcess(
                cmd=[
                    lc("semantic_script"),
                    "-p",
                    ["model:=", lc("semantic_model")],
                    "-p",
                    ["backend:=", lc("semantic_backend")],
                    "-p",
                    ["rate:=", lc("semantic_rate")],
                    "-p",
                    ["view:=", lc("semantic_view")],
                ],
                output="screen",
                on_exit=Shutdown(reason="Segmentation stopped"),
            ),
            Node(
                package="nomad_perception",
                executable="navigation_terrain",
                parameters=[
                    lc("terrain_config"),
                    {"directory": ParameterValue(lc("record_directory"), value_type=str)},
                ],
                output="screen",
                on_exit=Shutdown(reason="Terrain stopped"),
            ),
            Node(
                package="nomad_perception",
                executable="terrain_viewer",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(lc("viewer")),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    lc("rviz_config"),
                ],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(lc("rviz")),
                output="screen",
            ),
            Node(
                package="nomad_gazebo",
                executable="teleop.py",
                parameters=[{"initialization_only": True}],
                condition=IfCondition(lc("initial_teleop")),
                output="screen",
            ),
            Node(
                package="nomad_perception",
                executable="navigation_start",
                parameters=[
                    {
                        "modules_file": str(
                            share("nomad_perception")
                            / "config/modules_terrain_navigation.yaml"
                        ),
                        "rviz": False,
                        "initial_teleop": ParameterValue(lc("initial_teleop"), value_type=bool),
                        "gpp": ParameterValue(lc("gpp"), value_type=str),
                        "lpp": ParameterValue(lc("lpp"), value_type=str),
                        "params_file": ParameterValue(
                            lc("params_file"), value_type=str
                        ),
                    }
                ],
                output="screen",
                on_exit=Shutdown(reason="Autonomy stopped"),
            ),
        ]
    )
