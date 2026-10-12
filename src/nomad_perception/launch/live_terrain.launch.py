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
    p = Path(get_package_share_directory("nomad_perception"))
    gz = Path(get_package_share_directory("nomad_gazebo"))
    root = Path(os.environ.get("NOMAD_WS_ROOT", str(Path.cwd())))
    semantic = Path(get_package_share_directory("nomad_semantic")) / "scripts/run.sh"
    directory = (
        root / "sessions/live_terrain" / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=""),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("viewer", default_value="true"),
            DeclareLaunchArgument("teleop", default_value="true"),
            DeclareLaunchArgument("semantic_script", default_value=str(semantic)),
            DeclareLaunchArgument("resolution", default_value="0.2"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz / "launch/forest.launch.py")),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    LaunchConfiguration("semantic_script"),
                    "-p",
                    "rate:=10.0",
                    "-p",
                    "view:=false",
                ],
                output="screen",
                on_exit=Shutdown(reason="Segmentation stopped"),
            ),
            Node(
                package="nomad_perception",
                executable="live_terrain",
                parameters=[
                    {
                        "use_sim_time": True,
                        "resolution": ParameterValue(
                            LaunchConfiguration("resolution"), value_type=float
                        ),
                        "directory": str(directory),
                    }
                ],
                output="screen",
                on_exit=Shutdown(reason="Mapper stopped"),
            ),
            Node(
                package="nomad_perception",
                executable="terrain_viewer",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("viewer")),
                output="screen",
                on_exit=Shutdown(reason="Map viewer closed"),
            ),
            ExecuteProcess(
                cmd=[str(root / "scripts/run_teleop.sh")],
                condition=IfCondition(LaunchConfiguration("teleop")),
                output="screen",
            ),
        ]
    )
