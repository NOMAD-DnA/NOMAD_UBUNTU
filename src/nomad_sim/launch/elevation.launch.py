"""Launch a separate NOMAD sensor world without modifying upstream MVSim."""
import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def sensor_environment(configuration, mvsim_share):
    camera = configuration["camera"]
    lidar = configuration["lidar"]
    for section, settings in (("camera", camera), ("lidar", lidar)):
        for key, value in settings.items():
            if key == "model_reference" or value is None:
                continue
            if key in ("mono_previews", "raytrace_3d"):
                if not isinstance(value, bool):
                    raise ValueError(f"{section}.{key} must be a YAML boolean")
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{section}.{key} must be a finite number")
    for settings, keys in ((camera, ("width", "height")), (lidar, ("nrays",))):
        for key in keys:
            if int(settings[key]) != settings[key]:
                raise ValueError(f"{key} must be an integer")
    if any(float(value) < 0 for value in (
            camera["depth_noise_sigma_m"], lidar["range_noise_sigma_m"], lidar["angle_noise_sigma_deg"])):
        raise ValueError("Noise standard deviations must be nonnegative")
    width, height = int(camera["width"]), int(camera["height"])
    if width <= 0 or height <= 0 or float(camera["fps"]) <= 0:
        raise ValueError("Camera size and fps must be positive")
    if not 0 < float(camera["depth_min_m"]) < float(camera["depth_max_m"]):
        raise ValueError("Invalid depth clipping interval")
    if float(camera["baseline_m"]) <= 0:
        raise ValueError("Stereo baseline must be positive")
    if float(lidar["hz"]) <= 0 or int(lidar["nrays"]) < 2:
        raise ValueError("LiDAR rate must be positive and nrays >= 2")
    if not 0 < float(lidar["fov_deg"]) <= 360:
        raise ValueError("Invalid LiDAR FOV or range")
    if not 0 <= float(lidar.get("min_range_m", 0.0)) < float(lidar["max_range_m"]):
        raise ValueError("Invalid LiDAR range clipping interval")

    def focal(override, size, fov):
        if override is not None:
            value = float(override)
            if value <= 0:
                raise ValueError("Focal length must be positive")
            return value
        if not 0 < float(fov) < 180:
            raise ValueError("Camera FOV must be between 0 and 180 degrees")
        return size / (2.0 * math.tan(math.radians(float(fov)) / 2.0))

    values = {
        "NOMAD_MVSIM_SHARE": mvsim_share,
        "NOMAD_CAMERA_WIDTH": width,
        "NOMAD_CAMERA_HEIGHT": height,
        "NOMAD_CAMERA_PERIOD": 1.0 / float(camera["fps"]),
        "NOMAD_CAMERA_CX": width / 2 if camera["cx"] is None else camera["cx"],
        "NOMAD_CAMERA_CY": height / 2 if camera["cy"] is None else camera["cy"],
        "NOMAD_RGB_FX": focal(camera["rgb_fx"], width, camera["rgb_hfov_deg"]),
        "NOMAD_RGB_FY": focal(camera["rgb_fy"], height, camera["rgb_vfov_deg"]),
        "NOMAD_MONO_FX": focal(camera["mono_fx"], width, camera["mono_hfov_deg"]),
        "NOMAD_MONO_FY": focal(camera["mono_fy"], height, camera["mono_vfov_deg"]),
        "NOMAD_CAMERA_X": camera["mount_x_m"],
        "NOMAD_CAMERA_Y": camera["mount_y_m"],
        "NOMAD_CAMERA_Z": camera["mount_z_m"],
        "NOMAD_LEFT_Y": float(camera["mount_y_m"]) + float(camera["baseline_m"]) / 2,
        "NOMAD_RIGHT_Y": float(camera["mount_y_m"]) - float(camera["baseline_m"]) / 2,
        "NOMAD_DEPTH_MIN": camera["depth_min_m"],
        "NOMAD_DEPTH_MAX": camera["depth_max_m"],
        "NOMAD_DEPTH_NOISE": camera["depth_noise_sigma_m"],
        "NOMAD_MONO_PREVIEWS": str(bool(camera["mono_previews"])).lower(),
        "NOMAD_LIDAR_PERIOD": 1.0 / float(lidar["hz"]),
        "NOMAD_LIDAR_FOV": lidar["fov_deg"],
        "NOMAD_LIDAR_NRAYS": int(lidar["nrays"]),
        "NOMAD_LIDAR_MAX_RANGE": lidar["max_range_m"],
        "NOMAD_LIDAR_RANGE_NOISE": lidar["range_noise_sigma_m"],
        "NOMAD_LIDAR_ANGLE_NOISE": lidar["angle_noise_sigma_deg"],
        "NOMAD_LIDAR_X": lidar["mount_x_m"],
        "NOMAD_LIDAR_Y": lidar["mount_y_m"],
        "NOMAD_LIDAR_Z": lidar["mount_z_m"],
        "NOMAD_LIDAR_YAW": lidar["yaw_deg"],
        "NOMAD_LIDAR_PITCH": lidar["pitch_deg"],
        "NOMAD_LIDAR_ROLL": lidar["roll_deg"],
        "NOMAD_LIDAR_RAYTRACE_3D": str(bool(lidar["raytrace_3d"])).lower(),
    }
    return {key: str(value) for key, value in values.items()}


def launch_nodes(context):
    share = get_package_share_directory("nomad_sim")
    config_file = LaunchConfiguration("sensor_config").perform(context)
    with Path(config_file).open(encoding="utf-8") as stream:
        configuration = yaml.safe_load(stream)
    camera = configuration["camera"]
    lidar = configuration["lidar"]
    environment = sensor_environment(configuration, get_package_share_directory("mvsim"))
    remappings = [
        ("/scan", "/nomad/raw/scan"),
        ("/oak_rgb/image_raw", "/nomad/raw/rgb/image_raw"),
        ("/oak_rgb/camera_info", "/nomad/raw/rgb/camera_info"),
        ("/oak_depth/image_raw", "/nomad/raw/depth/image_raw"),
        ("/oak_depth/camera_info", "/nomad/raw/depth/camera_info"),
        ("/oak_left_optical_frame/image_raw", "/nomad/raw/left/image_raw"),
        ("/oak_left_optical_frame/camera_info", "/nomad/raw/left/camera_info"),
        ("/oak_right_optical_frame/image_raw", "/nomad/raw/right/image_raw"),
        ("/oak_right_optical_frame/camera_info", "/nomad/raw/right/camera_info"),
    ]
    return [
        Node(
            package="mvsim", executable="mvsim_node", name="mvsim", output="screen",
            additional_env=environment, remappings=remappings,
            parameters=[{
                "world_file": ParameterValue(LaunchConfiguration("world_file"), value_type=str),
                "headless": ParameterValue(LaunchConfiguration("headless"), value_type=bool),
                "do_fake_localization": True,
                "publish_tf_odom2baselink": True,
                "force_publish_vehicle_namespace": False,
                "disable_sim_time_clock": False,
            }],
        ),
        Node(
            package="nomad_sim", executable="oak_bridge.py", name="oak_sim_bridge",
            output="screen", parameters=[{
                "use_sim_time": True,
                "baseline_m": float(camera["baseline_m"]),
                "mount_x_m": float(camera["mount_x_m"]),
                "mount_y_m": float(camera["mount_y_m"]),
                "mount_z_m": float(camera["mount_z_m"]),
            }],
        ),
        Node(
            package="nomad_sim", executable="lidar_bridge.py", name="lidar_sim_bridge",
            output="screen", parameters=[{
                "use_sim_time": True,
                "min_range_m": float(lidar.get("min_range_m", 0.0)),
                "max_range_m": float(lidar["max_range_m"]),
                "scan_hz": float(lidar["hz"]),
            }],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("nomad_sim")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="False"),
        DeclareLaunchArgument("sensor_config", default_value=os.path.join(share, "config", "sensors.yaml")),
        DeclareLaunchArgument("world_file", default_value=os.path.join(share, "worlds", "elevation.world.xml")),
        OpaqueFunction(function=launch_nodes),
    ])
