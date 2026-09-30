#!/usr/bin/env python3
"""Generate one provisional Ackermann geometry for Gazebo and ROS TF.

The box / cylinder geometry, masses, friction and joint effort tags are NOT
measurements of the team's vehicle. AckermannSteering requests wheel velocity;
this is not a validated torque-limited drivetrain or suspension model.

Native camera poses look along +X. ROS optical frames have +Z forward, +X
right, +Y down, and are published by robot_state_publisher, not this script.
Generated files are the only files written. No simulator or ROS node is run.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


MODEL = "nomad_vehicle"
BODY_SIZE = (0.85, 0.50, 0.12)
BODY_CENTER_Z = 0.20
BODY_MASS = 30.0
WHEELBASE = 0.72
TRACK = 0.60
WHEEL_RADIUS = 0.15
WHEEL_WIDTH = 0.08
WHEEL_MASS = 1.0
KNUCKLE_MASS = 0.30
KNUCKLE_RADIUS = 0.025
KNUCKLE_HEIGHT = 0.04
STEERING_LIMIT = 0.55
VISUAL_CAMERA_FAR_M = 40.0
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
TOPICS = {
    "cmd_vel": "/nomad/vehicle/cmd_vel",
    "wheel_odom": "/nomad/vehicle/wheel_odom",
    "wheel_tf": "/nomad/vehicle/wheel_tf",
    "odom": "/nomad/vehicle/odom",
    "tf": "/nomad/vehicle/tf",
    "joint_states": "/nomad/vehicle/joint_states",
    "rgb_image": "/nomad/raw/oak/rgbd/image",
    "depth_image": "/nomad/raw/oak/rgbd/depth_image",
    "rgbd_camera_info": "/nomad/raw/oak/rgbd/camera_info",
    "left_image": "/nomad/raw/oak/left/image_raw",
    "left_camera_info": "/nomad/raw/oak/left/camera_info",
    "right_image": "/nomad/raw/oak/right/image_raw",
    "right_camera_info": "/nomad/raw/oak/right/camera_info",
    "scan": "/nomad/raw/scan",
    "imu": "/nomad/raw/oak/imu",
}


def number(value: float | int) -> str:
    if not math.isfinite(value):
        raise ValueError(f"Non-finite XML value: {value}")
    return format(0.0 if abs(value) < 1e-14 else value, ".15g")


def vector(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(number(value) for value in values)


def element(parent: ET.Element, tag: str, text_value: Any = None, **attrs: str) -> ET.Element:
    child = ET.SubElement(parent, tag, attrs)
    if text_value is not None:
        child.text = number(text_value) if isinstance(text_value, (int, float)) else str(text_value)
    return child


def positive(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return result


def finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def load_sensors(path: Path) -> dict[str, Any]:
    # PyYAML is supplied by ROS / python3-yaml; JSON inputs need only stdlib.
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        import yaml

        data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError("Sensor configuration must be a mapping")
    return data


def sensor_parameters(config: dict[str, Any]) -> dict[str, Any]:
    camera, lidar, imu = config["camera"], config["lidar"], config["imu"]
    width, height = camera["width"], camera["height"]
    nrays = lidar["nrays"]
    for value, label in ((width, "width"), (height, "height"), (nrays, "nrays")):
        if isinstance(value, bool) or not isinstance(value, int) or value < 2:
            raise ValueError(f"{label} must be an integer >= 2")
    fps = positive(camera["fps"], "camera.fps")
    baseline = positive(camera["baseline_m"], "camera.baseline_m")
    if baseline > BODY_SIZE[1]:
        raise ValueError("Stereo baseline exceeds the provisional chassis width")
    intrinsics = {}
    for name in ("rgb", "mono"):
        hfov = positive(camera[f"{name}_hfov_deg"], f"{name}_hfov_deg")
        vfov = positive(camera[f"{name}_vfov_deg"], f"{name}_vfov_deg")
        if hfov >= 180 or vfov >= 180:
            raise ValueError("Pinhole fields of view must be less than 180 degrees")
        fx = camera.get(f"{name}_fx")
        fy = camera.get(f"{name}_fy")
        intrinsics[name] = {
            "fx": positive(fx, f"{name}_fx") if fx is not None else
            width / (2.0 * math.tan(math.radians(hfov) / 2.0)),
            "fy": positive(fy, f"{name}_fy") if fy is not None else
            height / (2.0 * math.tan(math.radians(vfov) / 2.0)),
            "cx": finite(camera["cx"], "cx") if camera.get("cx") is not None else width / 2.0,
            "cy": finite(camera["cy"], "cy") if camera.get("cy") is not None else height / 2.0,
            "horizontal_fov": math.radians(hfov),
        }
    camera_mount = tuple(finite(camera[f"mount_{axis}_m"], f"camera.mount_{axis}_m")
                         for axis in "xyz")
    lidar_mount = tuple(finite(lidar[f"mount_{axis}_m"], f"lidar.mount_{axis}_m")
                        for axis in "xyz")
    if min(camera_mount[2], lidar_mount[2]) <= BODY_CENTER_Z + BODY_SIZE[2] / 2.0:
        raise ValueError("Sensor origins must be above the chassis collision box")
    depth_min = positive(camera["depth_min_m"], "depth_min_m")
    depth_max = positive(camera["depth_max_m"], "depth_max_m")
    minimum = positive(lidar["min_range_m"], "lidar.min_range_m")
    maximum = positive(lidar["max_range_m"], "lidar.max_range_m")
    fov = positive(lidar["fov_deg"], "lidar.fov_deg")
    if depth_max <= depth_min or maximum <= minimum or fov > 360.0:
        raise ValueError("Invalid depth / lidar range or field of view")
    # Exactly 500 distinct directions around a 360-degree ring, not a duplicate
    # ray at both -pi and +pi. Partial-FOV endpoints remain inclusive.
    span = math.radians(fov)
    angle_min = -span / 2.0
    angle_max = angle_min + (span * (nrays - 1) / nrays if fov == 360.0 else span)
    noise = finite(lidar.get("range_noise_sigma_m", 0.0), "range_noise_sigma_m")
    if noise < 0.0:
        raise ValueError("Range noise sigma cannot be negative")
    return {
        "width": width, "height": height, "fps": fps, "baseline_m": baseline,
        "intrinsics": intrinsics, "camera_mount": camera_mount,
        "depth_min_m": depth_min, "depth_max_m": depth_max,
        "lidar_mount": lidar_mount,
        "lidar_rpy": tuple(math.radians(finite(lidar.get(f"{axis}_deg", 0.0), axis))
                           for axis in ("roll", "pitch", "yaw")),
        "lidar_hz": positive(lidar["hz"], "lidar.hz"), "nrays": nrays,
        "min_range_m": minimum, "max_range_m": maximum,
        "fov_deg": fov, "angle_min": angle_min, "angle_max": angle_max,
        "range_noise_sigma_m": noise,
        "imu_hz": positive(imu["hz"], "imu.hz"),
    }


def box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, ...]:
    x, y, z = size
    return (mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0,
            mass * (x * x + y * y) / 12.0)


def cylinder_inertia(mass: float, radius: float, length: float) -> tuple[float, ...]:
    transverse = mass * (3.0 * radius * radius + length * length) / 12.0
    return (transverse, mass * radius * radius / 2.0, transverse)


def physical_model() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    links = [{"name": "base_link", "pose": (0.0, 0.0, 0.0), "mass": BODY_MASS,
              "inertia": box_inertia(BODY_MASS, BODY_SIZE),
              "inertial_xyz": (0.0, 0.0, BODY_CENTER_Z), "geometry": "body"}]
    joints = []
    for end, x in (("front", WHEELBASE / 2.0), ("rear", -WHEELBASE / 2.0)):
        for side, y in (("left", TRACK / 2.0), ("right", -TRACK / 2.0)):
            wheel = f"{end}_{side}_wheel"
            parent = "base_link"
            wheel_origin = (x, y, WHEEL_RADIUS)
            if end == "front":
                parent = f"{end}_{side}_steering_link"
                transverse, axial, _ = cylinder_inertia(
                    KNUCKLE_MASS, KNUCKLE_RADIUS, KNUCKLE_HEIGHT)
                links.append({"name": parent, "pose": wheel_origin,
                              "mass": KNUCKLE_MASS, "inertia": (transverse, transverse, axial),
                              "inertial_xyz": (0.0, 0.0, 0.0), "geometry": "knuckle"})
                joints.append({"name": f"{end}_{side}_steering_joint", "parent": "base_link",
                               "child": parent, "origin": wheel_origin, "axis": (0.0, 0.0, 1.0),
                               "type": "revolute", "lower": -STEERING_LIMIT,
                               "upper": STEERING_LIMIT, "effort": 12.0, "velocity": 2.0})
                wheel_origin = (0.0, 0.0, 0.0)
            links.append({"name": wheel, "pose": (x, y, WHEEL_RADIUS), "mass": WHEEL_MASS,
                          "inertia": cylinder_inertia(WHEEL_MASS, WHEEL_RADIUS, WHEEL_WIDTH),
                          "inertial_xyz": (0.0, 0.0, 0.0), "geometry": "wheel"})
            joints.append({"name": f"{wheel}_joint", "parent": parent, "child": wheel,
                           "origin": wheel_origin, "axis": (0.0, 1.0, 0.0),
                           "type": "continuous", "lower": -1e16, "upper": 1e16,
                           "effort": 20.0, "velocity": 20.0})
    return links, joints


def fixed_frames(p: dict[str, Any]) -> list[dict[str, Any]]:
    x, y, z = p["camera_mount"]
    frames = []
    for name, offset in (("rgb", 0.0), ("depth", 0.0),
                         ("left", p["baseline_m"] / 2.0),
                         ("right", -p["baseline_m"] / 2.0)):
        frames.append({"name": f"oak_{name}_optical_frame", "parent": "base_link",
                       "xyz": (x, y + offset, z), "rpy": OPTICAL_RPY})
    frames.append({"name": "scan", "parent": "base_link", "xyz": p["lidar_mount"],
                   "rpy": p["lidar_rpy"]})
    # Provisional BMI270 position: colocated with the OAK camera mount.
    frames.append({"name": "oak_imu_frame", "parent": "base_link", "xyz": p["camera_mount"],
                   "rpy": (0.0, 0.0, 0.0)})
    return frames


def sdf_inertial(link: ET.Element, info: dict[str, Any]) -> None:
    inertial = element(link, "inertial")
    element(inertial, "pose", vector((*info["inertial_xyz"], 0.0, 0.0, 0.0)))
    element(inertial, "mass", info["mass"])
    inertia = element(inertial, "inertia")
    for name, value in zip(("ixx", "iyy", "izz"), info["inertia"]):
        element(inertia, name, value)
    for name in ("ixy", "ixz", "iyz"):
        element(inertia, name, 0.0)


def sdf_geometry(parent: ET.Element, kind: str) -> None:
    geometry = element(parent, "geometry")
    if kind == "body":
        element(element(geometry, "box"), "size", vector(BODY_SIZE))
    else:
        cylinder = element(geometry, "cylinder")
        element(cylinder, "radius", WHEEL_RADIUS if kind == "wheel" else KNUCKLE_RADIUS)
        element(cylinder, "length", WHEEL_WIDTH if kind == "wheel" else KNUCKLE_HEIGHT)


def camera_sensor(base: ET.Element, p: dict[str, Any], side: str) -> None:
    is_rgbd = side == "rgb"
    sensor = element(base, "sensor", name=f"oak_{side}", type="rgbd_camera" if is_rgbd else "camera")
    mount = list(p["camera_mount"])
    if side in ("left", "right"):
        mount[1] += p["baseline_m"] / 2.0 * (1 if side == "left" else -1)
    element(sensor, "pose", vector((*mount, 0.0, 0.0, 0.0)), relative_to="base_link")
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", p["fps"])
    element(sensor, "visualize", "false")
    element(sensor, "topic", "/nomad/raw/oak/rgbd" if is_rgbd else TOPICS[f"{side}_image"])
    element(sensor, "gz_frame_id", f"oak_{side}_optical_frame")
    camera = element(sensor, "camera")
    element(camera, "optical_frame_id", f"oak_{side}_optical_frame")
    if not is_rgbd:
        element(camera, "camera_info_topic", TOPICS[f"{side}_camera_info"])
    k = p["intrinsics"]["rgb" if is_rgbd else "mono"]
    element(camera, "horizontal_fov", k["horizontal_fov"])
    image = element(camera, "image")
    element(image, "width", p["width"])
    element(image, "height", p["height"])
    # Native color is RGB; the ROS adapter publishes bgr8 / mono8 as appropriate.
    element(image, "format", "R8G8B8" if is_rgbd else "L8")
    clip = element(camera, "clip")
    element(clip, "near", 0.05)
    # RGB must see distant scenery, while the depth product keeps the OAK
    # operating range. gz-sensors8 RGBD applies depth_camera/clip separately
    # to published depth values, within the visual camera's rendering limits.
    element(clip, "far", max(VISUAL_CAMERA_FAR_M, p["depth_max_m"]) if is_rgbd else VISUAL_CAMERA_FAR_M)
    if is_rgbd:
        depth_clip = element(element(camera, "depth_camera"), "clip")
        element(depth_clip, "near", p["depth_min_m"])
        element(depth_clip, "far", p["depth_max_m"])
    lens = element(camera, "lens")
    element(lens, "type", "gnomonical")
    intrinsics = element(lens, "intrinsics")
    for key in ("fx", "fy", "cx", "cy"):
        element(intrinsics, key, k[key])
    element(intrinsics, "s", 0.0)
    # Native CameraInfo is already rectified; the right P translation is -fx*B.
    projection = element(lens, "projection")
    for key in ("fx", "fy", "cx", "cy"):
        element(projection, f"p_{key}", k[key])
    element(projection, "tx", -k["fx"] * p["baseline_m"] if side == "right" else 0.0)
    element(projection, "ty", 0.0)


def lidar_sensor(base: ET.Element, p: dict[str, Any]) -> None:
    sensor = element(base, "sensor", name="ydlidar_g2", type="gpu_lidar")
    element(sensor, "pose", vector((*p["lidar_mount"], *p["lidar_rpy"])), relative_to="base_link")
    element(sensor, "topic", TOPICS["scan"])
    element(sensor, "gz_frame_id", "scan")
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", p["lidar_hz"])
    element(sensor, "visualize", "false")
    lidar = element(sensor, "lidar")
    scan = element(lidar, "scan")
    horizontal = element(scan, "horizontal")
    for key, value in (("samples", p["nrays"]), ("resolution", 1),
                       ("min_angle", p["angle_min"]), ("max_angle", p["angle_max"])):
        element(horizontal, key, value)
    vertical = element(scan, "vertical")
    for key, value in (("samples", 1), ("resolution", 1), ("min_angle", 0), ("max_angle", 0)):
        element(vertical, key, value)
    ranges = element(lidar, "range")
    for key, value in (("min", p["min_range_m"]), ("max", p["max_range_m"]), ("resolution", 0.01)):
        element(ranges, key, value)
    noise = element(lidar, "noise")
    element(noise, "type", "gaussian")
    element(noise, "mean", 0.0)
    element(noise, "stddev", p["range_noise_sigma_m"])


def imu_sensor(base: ET.Element, p: dict[str, Any]) -> None:
    sensor = element(base, "sensor", name="oak_imu", type="imu")
    element(sensor, "pose", vector((*p["camera_mount"], 0.0, 0.0, 0.0)), relative_to="base_link")
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", p["imu_hz"])
    element(sensor, "topic", TOPICS["imu"])
    element(sensor, "gz_frame_id", "oak_imu_frame")
    element(sensor, "imu")


def make_sdf(links: list[dict[str, Any]], joints: list[dict[str, Any]],
             frames: list[dict[str, Any]], p: dict[str, Any], with_sensors: bool) -> ET.Element:
    sdf = ET.Element("sdf", {"version": "1.10"})
    model = element(sdf, "model", name=MODEL, canonical_link="base_link")
    model.append(ET.Comment("PROVISIONAL dimensions/masses, no suspension; NOT calibrated motor output."))
    element(model, "static", "false")
    element(model, "self_collide", "false")
    for info in links:
        link = element(model, "link", name=info["name"])
        element(link, "pose", vector((*info["pose"], 0.0, 0.0, 0.0)), relative_to="__model__")
        sdf_inertial(link, info)
        kind = info["geometry"]
        local_pose = (0.0, 0.0, BODY_CENTER_Z, 0.0, 0.0, 0.0) if kind == "body" else \
            (0.0, 0.0, 0.0, math.pi / 2.0 if kind == "wheel" else 0.0, 0.0, 0.0)
        visual = element(link, "visual", name=f"{info['name']}_visual")
        element(visual, "pose", vector(local_pose))
        sdf_geometry(visual, kind)
        material = element(visual, "material")
        color = "0.15 0.38 0.18 1" if kind == "body" else "0.12 0.12 0.12 1"
        element(material, "ambient", color)
        element(material, "diffuse", color)
        if kind != "knuckle":
            collision = element(link, "collision", name=f"{info['name']}_collision")
            element(collision, "pose", vector(local_pose))
            sdf_geometry(collision, kind)
            if kind == "wheel":
                ode = element(element(element(collision, "surface"), "friction"), "ode")
                element(ode, "mu", 1.0)
                element(ode, "mu2", 1.0)
        if info["name"] == "base_link" and with_sensors:
            for side in ("rgb", "left", "right"):
                camera_sensor(link, p, side)
            lidar_sensor(link, p)
            imu_sensor(link, p)
    for joint in joints:
        node = element(model, "joint", name=joint["name"], type="revolute")
        element(node, "parent", joint["parent"])
        element(node, "child", joint["child"])
        element(node, "pose", "0 0 0 0 0 0", relative_to=joint["child"])
        axis = element(node, "axis")
        element(axis, "xyz", vector(joint["axis"]))
        limit = element(axis, "limit")
        for key in ("lower", "upper", "effort", "velocity"):
            element(limit, key, joint[key])
        element(element(axis, "dynamics"), "damping", 0.02)
    for frame in frames:
        node = element(model, "frame", name=frame["name"], attached_to=frame["parent"])
        element(node, "pose", vector((*frame["xyz"], *frame["rpy"])), relative_to=frame["parent"])
    controller = element(model, "plugin", filename="gz-sim-ackermann-steering-system",
                         name="gz::sim::systems::AckermannSteering")
    for key, value in (("left_joint", "rear_left_wheel_joint"),
                       ("right_joint", "rear_right_wheel_joint"),
                       ("left_steering_joint", "front_left_steering_joint"),
                       ("right_steering_joint", "front_right_steering_joint"),
                       ("wheel_base", WHEELBASE), ("wheel_separation", TRACK),
                       ("kingpin_width", TRACK), ("wheel_radius", WHEEL_RADIUS),
                       ("steering_limit", STEERING_LIMIT), ("min_velocity", -1.0),
                       ("max_velocity", 1.0), ("min_acceleration", -0.6),
                       ("max_acceleration", 0.6), ("topic", TOPICS["cmd_vel"]),
                       ("odom_topic", TOPICS["wheel_odom"]), ("tf_topic", TOPICS["wheel_tf"]),
                       ("frame_id", "odom"), ("child_frame_id", "base_link"),
                       ("odom_publish_frequency", 50.0)):
        element(controller, key, value)
    odometry = element(model, "plugin", filename="gz-sim-odometry-publisher-system",
                       name="gz::sim::systems::OdometryPublisher")
    for key, value in (("dimensions", 3), ("odom_frame", "odom"),
                       ("robot_base_frame", "base_link"), ("odom_topic", TOPICS["odom"]),
                       ("tf_topic", TOPICS["tf"]), ("odom_publish_frequency", 50.0),
                       ("gaussian_noise", 0.0)):
        element(odometry, key, value)
    publisher = element(model, "plugin", filename="gz-sim-joint-state-publisher-system",
                        name="gz::sim::systems::JointStatePublisher")
    element(publisher, "topic", TOPICS["joint_states"])
    element(publisher, "update_rate", 50.0)
    for joint in joints:
        element(publisher, "joint_name", joint["name"])
    return sdf


def make_urdf(links: list[dict[str, Any]], joints: list[dict[str, Any]],
              frames: list[dict[str, Any]]) -> ET.Element:
    robot = ET.Element("robot", {"name": MODEL})
    robot.append(ET.Comment("Generated from the same provisional geometry as models/nomad_vehicle/model.sdf."))
    for info in links:
        link = element(robot, "link", name=info["name"])
        inertial = element(link, "inertial")
        element(inertial, "origin", xyz=vector(info["inertial_xyz"]), rpy="0 0 0")
        element(inertial, "mass", value=number(info["mass"]))
        element(inertial, "inertia", ixx=number(info["inertia"][0]), iyy=number(info["inertia"][1]),
                izz=number(info["inertia"][2]), ixy="0", ixz="0", iyz="0")
        kind = info["geometry"]
        for tag in ("visual", "collision"):
            if tag == "collision" and kind == "knuckle":
                continue
            part = element(link, tag)
            xyz = (0.0, 0.0, BODY_CENTER_Z) if kind == "body" else (0.0, 0.0, 0.0)
            rpy = (math.pi / 2.0, 0.0, 0.0) if kind == "wheel" else (0.0, 0.0, 0.0)
            element(part, "origin", xyz=vector(xyz), rpy=vector(rpy))
            geometry = element(part, "geometry")
            if kind == "body":
                element(geometry, "box", size=vector(BODY_SIZE))
            else:
                element(geometry, "cylinder", radius=number(WHEEL_RADIUS if kind == "wheel" else KNUCKLE_RADIUS),
                        length=number(WHEEL_WIDTH if kind == "wheel" else KNUCKLE_HEIGHT))
            if tag == "visual":
                color = "0.15 0.38 0.18 1" if kind == "body" else "0.12 0.12 0.12 1"
                element(element(part, "material", name=f"{info['name']}_material"), "color", rgba=color)
    for joint in joints:
        node = element(robot, "joint", name=joint["name"], type=joint["type"])
        element(node, "parent", link=joint["parent"])
        element(node, "child", link=joint["child"])
        element(node, "origin", xyz=vector(joint["origin"]), rpy="0 0 0")
        element(node, "axis", xyz=vector(joint["axis"]))
        limits = {"effort": number(joint["effort"]), "velocity": number(joint["velocity"])}
        if joint["type"] == "revolute":
            limits.update(lower=number(joint["lower"]), upper=number(joint["upper"]))
        element(node, "limit", **limits)
        element(node, "dynamics", damping="0.02", friction="0")
    for frame in frames:
        element(robot, "link", name=frame["name"])
        node = element(robot, "joint", name=f"{frame['name']}_joint", type="fixed")
        element(node, "parent", link=frame["parent"])
        element(node, "child", link=frame["name"])
        element(node, "origin", xyz=vector(frame["xyz"]), rpy=vector(frame["rpy"]))
    return robot


def write_xml(path: Path, root: ET.Element) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write("\n")


def generate_vehicle(output_dir: Path, sensor_config: Path | dict[str, Any], *,
                     with_sensors: bool = True) -> dict[str, Any]:
    config = load_sensors(sensor_config) if isinstance(sensor_config, Path) else sensor_config
    p = sensor_parameters(config)
    links, joints = physical_model()
    frames = fixed_frames(p)
    output_dir = Path(output_dir)
    write_xml(output_dir / "models" / MODEL / "model.sdf", make_sdf(links, joints, frames, p, with_sensors))
    write_xml(output_dir / "urdf" / f"{MODEL}.urdf", make_urdf(links, joints, frames))
    model_config = ET.Element("model")
    element(model_config, "name", MODEL)
    element(model_config, "version", "0.1.0")
    element(model_config, "sdf", "model.sdf", version="1.10")
    element(model_config, "description", "Provisional block Ackermann vehicle; uncalibrated dimensions, mass and drivetrain.")
    write_xml(output_dir / "models" / MODEL / "model.config", model_config)
    metadata = {
        "schema_version": 1, "model": MODEL, "provisional": True,
        "notes": ["Not the measured team's vehicle; base_link origin is the level ground plane.",
                  "Rear-wheel velocity drive, no suspension; effort limits are not a validated motor-output cap.",
                  "3D odom/TF is ideal ground truth from Gazebo, NOT a state estimator.",
                  "Planar wheel_odom/wheel_tf are separate and must not be bridged to the same ROS odom/TF.",
                  "Depth is ideal co-located RGBD rendering, NOT calibrated physical stereo depth.",
                  "Single-plane GPU lidar samples the actual 3D scene; angular noise is not modeled."],
        "body_size_m": BODY_SIZE, "body_center_z_m": BODY_CENTER_Z,
        "body_mass_kg": BODY_MASS, "wheel_mass_kg": WHEEL_MASS,
        "knuckle_mass_kg": KNUCKLE_MASS, "total_mass_kg": sum(link["mass"] for link in links),
        "wheelbase_m": WHEELBASE, "track_m": TRACK, "wheel_radius_m": WHEEL_RADIUS,
        "wheel_width_m": WHEEL_WIDTH, "steering_limit_rad": STEERING_LIMIT,
        "max_speed_mps": 1.0, "max_acceleration_mps2": 0.6,
        "driven_joints": ["rear_left_wheel_joint", "rear_right_wheel_joint"],
        "physical_links": [link["name"] for link in links],
        "dynamic_joints": joints, "fixed_frames": frames,
        "sensors_enabled": with_sensors, "sensor_parameters": p,
        "rgb_render_clip_m": [0.05, max(VISUAL_CAMERA_FAR_M, p["depth_max_m"])],
        "mono_render_clip_m": [0.05, VISUAL_CAMERA_FAR_M],
        "native_topics": TOPICS,
        "camera_source": config["camera"], "lidar_source": config["lidar"],
        "imu_source": config["imu"],
    }
    metadata_path = output_dir / "config" / "vehicle.json"
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="nomad_gazebo package root")
    parser.add_argument("--sensor-config", type=Path,
                        default=Path(__file__).resolve().parents[2] / "nomad_sim" / "config" / "sensors.yaml")
    parser.add_argument("--no-sensors", action="store_true", help="omit rendering sensors, retain fixed ROS frames")
    args = parser.parse_args()
    metadata = generate_vehicle(args.output_dir, args.sensor_config, with_sensors=not args.no_sensors)
    print(json.dumps({"model": MODEL, "physical_links": len(metadata["physical_links"]),
                      "dynamic_joints": len(metadata["dynamic_joints"]),
                      "total_mass_kg": metadata["total_mass_kg"], "sensors_enabled": metadata["sensors_enabled"]}))


if __name__ == "__main__":
    main()
