"""Instrument a runtime NOMAD map copy with authored original GOOSE64 labels.

Collisions, existing sensors, dynamics, model/link poses and visual face records
are preserved. Tree OBJ visuals are partitioned by existing materials so bark
and canopy receive separate labels; source files remain read-only. Absolute
asset paths always resolve against the running copied workspace, never a saved
path from a previous development checkout.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from .semantic_obj import split_obj_by_class


def cost(class_id):
    # Authored scene labels use the same agreed costs as the independent core.
    return {0: None, 8: None, 23: 5, 27: 100, 28: 100, 31: 20, 40: 100, 51: 65}.get(class_id, 100)

VERSION = "nomad-goose-authored-v1"
GRASS_MODELS = {"forest_grass", "path_grass", "meadow_grass", "undergrowth"}
TREE_MESHES = {"pine_tree", "broadleaf_tree", "forest_far", "forest_clusters", "broadleaf_clusters"}
DETAIL_MESHES = {"roadside_details"}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical(element: ET.Element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(canonical(child) for child in element))


def resolve_uri(text: str, owner: Path) -> Path:
    text = text.strip()
    if text.startswith("file://"):
        path = Path(text[7:])
    elif "://" in text:
        raise ValueError(f"Unsupported URI in local source map: {text}")
    else:
        path = Path(text)
    if not path.is_absolute():
        path = owner.parent / path
    path = path.resolve()
    if not path.exists():
        raise ValueError(f"Referenced source asset does not exist: {path}")
    return path


def absolute_uris(element: ET.Element, owner: Path, sources: dict[Path, str]):
    for uri in element.iter("uri"):
        path = resolve_uri(uri.text or "", owner)
        uri.text = str(path)
        if path.is_file():
            sources.setdefault(path, digest(path))


def load_world(source: Path, sources: dict[Path, str], vehicle_model_path: Path | None = None) -> ET.ElementTree:
    tree = ET.parse(source)
    sources.setdefault(source, digest(source))
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("Expected an SDF world")
    for include in list(world.findall("include")):
        include_uri = include.findtext("uri", "")
        if include_uri == "model://nomad_vehicle":
            model_path = (vehicle_model_path if vehicle_model_path is not None else
                          source.parent.parent / "models" / "nomad_vehicle" / "model.sdf")
            model_path = Path(model_path).resolve()
            if not model_path.is_file():
                raise ValueError(f"Cannot resolve NOMAD vehicle model: {model_path}")
        else:
            model_path = resolve_uri(include_uri, source)
        if model_path.is_dir():
            model_path /= "model.sdf"
        model = deepcopy(ET.parse(model_path).getroot().find("model"))
        if model is None or model.get("name") != "nomad_vehicle":
            raise ValueError("Only the existing NOMAD vehicle include is supported")
        if any(child.tag not in {"uri", "name", "pose"} for child in include):
            raise ValueError("Unsupported vehicle include override; preserve it explicitly")
        sources.setdefault(model_path, digest(model_path))
        absolute_uris(model, model_path, sources)
        model.set("name", include.findtext("name", model.get("name")))
        old_pose = model.find("pose")
        if old_pose is not None:
            model.remove(old_pose)
        if include.find("pose") is not None:
            model.insert(0, deepcopy(include.find("pose")))
        world.insert(list(world).index(include), model)
        world.remove(include)
    absolute_uris(world, source, sources)
    return tree


def material_ids(path: Path) -> dict[str, int]:
    materials = {line.split()[1] for line in path.read_text(encoding="utf-8").splitlines()
                 if line.startswith("usemtl ")}
    result = {}
    for material in materials:
        if material in {"bark", "fallen_bark"}:
            result[material] = 28
        elif material.startswith(("pine_", "leaf_")):
            result[material] = 27
        elif material.startswith("stone_"):
            result[material] = 40
        else:
            raise ValueError(f"No class rule for material {material} in {path}")
    return result


def label(visual: ET.Element, class_id: int):
    for plugin in list(visual.findall("plugin")):
        if "Label" in plugin.get("name", ""):
            visual.remove(plugin)
    plugin = ET.SubElement(visual, "plugin", filename="gz-sim-label-system", name="gz::sim::systems::Label")
    ET.SubElement(plugin, "label").text = str(class_id)


def add_camera(vehicle: ET.Element, reference_name: str) -> dict:
    match = [(link, sensor) for link in vehicle.findall("link") for sensor in link.findall("sensor")
             if sensor.get("name") == reference_name]
    if len(match) != 1:
        raise ValueError(f"Expected one reference camera {reference_name}")
    link, reference = match[0]
    original = reference.find("camera")
    if original is None:
        raise ValueError("Reference sensor is not a camera")
    sensor_name = "goose_semantic_" + reference_name
    if vehicle.find(f".//sensor[@name='{sensor_name}']") is not None:
        raise ValueError("The source is already a labeled copy; regenerate from the original")
    base_topic = "/nomad/semantic_gt/" + reference_name
    sensor = deepcopy(reference)
    sensor.set("name", sensor_name)
    sensor.set("type", "segmentation")
    topic = sensor.find("topic")
    if topic is None:
        topic = ET.SubElement(sensor, "topic")
    topic.text = base_topic
    camera = sensor.find("camera")
    image_format = camera.find("image/format")
    if image_format is not None:
        image_format.text = "R8G8B8"
    for name in ("depth_camera", "segmentation_type", "save", "camera_info_topic"):
        child = camera.find(name)
        if child is not None:
            camera.remove(child)
    ET.SubElement(camera, "segmentation_type").text = "semantic"
    ET.SubElement(camera, "camera_info_topic").text = base_topic + "/camera_info"
    link.append(sensor)
    return {
        "reference_sensor": reference_name, "frame": reference.findtext("gz_frame_id"),
        "pose": reference.findtext("pose"), "width": int(original.findtext("image/width")),
        "height": int(original.findtext("image/height")),
        "intrinsics": {key: float(original.findtext("lens/intrinsics/" + key))
                       for key in ("fx", "fy", "cx", "cy")},
        "update_rate_hz": float(reference.findtext("update_rate")),
        "gazebo_labels_topic": base_topic + "/labels_map",
        "gazebo_camera_info_topic": base_topic + "/camera_info",
        "background_id": 0, "background_cost": None,
        "ros_topics_are_proposed_not_final_contract": False,
    }


def build_world(source: Path, output: Path, *, camera_name: str,
                sources: dict[Path, str], splits: dict[Path, dict[int, Path]],
                vehicle_model_path: Path | None = None) -> dict:
    if source.resolve() == output.resolve():
        raise ValueError("Never overwrite the original world")
    tree = load_world(source, sources, vehicle_model_path)
    world = tree.getroot().find("world")
    vehicle = next((model for model in world.findall("model") if model.get("name") == "nomad_vehicle"), None)
    if vehicle is None:
        raise ValueError("No NOMAD vehicle found")
    protected_tags = ("collision", "inertial", "sensor", "physics", "joint")
    before = {tag: [canonical(element) for element in world.iter(tag)] for tag in protected_tags}
    poses_before = [(model.get("name"), canonical(model.find("pose")) if model.find("pose") is not None else None,
                     [(link.get("name"), canonical(link.find("pose")) if link.find("pose") is not None else None)
                      for link in model.findall("link")]) for model in world.findall("model")]
    records = []
    for model in world.findall("model"):
        name = model.get("name", "")
        for link in model.findall("link"):
            for visual in list(link.findall("visual")):
                uri = visual.find("geometry/mesh/uri")
                mesh_path = None if uri is None else Path(uri.text)
                split_classes = None
                if name == "nomad_vehicle":
                    class_id = 8
                elif name == "terrain":
                    class_id = 31
                elif name in {"test_flat_ground", "ground_plane"} or name.startswith("ramp_"):
                    class_id = 23
                elif name in GRASS_MODELS:
                    class_id = 51
                elif mesh_path is not None and mesh_path.stem in TREE_MESHES | DETAIL_MESHES:
                    if mesh_path not in splits:
                        asset_id = mesh_path.stem + "_" + hashlib.sha256(str(mesh_path).encode()).hexdigest()[:8]
                        splits[mesh_path] = split_obj_by_class(mesh_path, output.parent / "assets" / asset_id,
                                                              material_ids(mesh_path))
                    split_classes = splits[mesh_path]
                    class_id = None
                elif mesh_path is not None and mesh_path.stem == "rock":
                    class_id = 40
                else:
                    raise ValueError(f"Unmapped visual: {name}/{link.get('name')}/{visual.get('name')}")
                assigned = {class_id: None} if split_classes is None else split_classes
                for assigned_id, split_path in assigned.items():
                    target = visual if split_path is None else deepcopy(visual)
                    if split_path is not None:
                        target.set("name", visual.get("name") + "_goose_" + str(assigned_id))
                        target.find("geometry/mesh/uri").text = str(split_path)
                        link.insert(list(link).index(visual), target)
                    label(target, assigned_id)
                    records.append({"model": name, "link": link.get("name"), "visual": target.get("name"),
                                    "class_id": assigned_id, "semantic_cost": cost(assigned_id),
                                    "source_mesh": None if mesh_path is None else str(mesh_path)})
                if split_classes is not None:
                    link.remove(visual)
    # The material partition changes only visual organization. Vehicle dynamics,
    # collisions and all existing sensors must remain identical.
    for tag in protected_tags:
        if before[tag] != [canonical(element) for element in world.iter(tag)]:
            raise AssertionError(f"Protected source geometry/settings changed: {tag}")
    poses_after = [(model.get("name"), canonical(model.find("pose")) if model.find("pose") is not None else None,
                    [(link.get("name"), canonical(link.find("pose")) if link.find("pose") is not None else None)
                     for link in model.findall("link")]) for model in world.findall("model")]
    if poses_before != poses_after:
        raise AssertionError("Model or link poses changed")
    camera = add_camera(vehicle, camera_name)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return {"map": source.parent.parent.name, "variant": source.stem, "source_world": str(source),
            "world": str(output), "camera": camera, "visuals": records,
            "visual_counts_by_class": dict(sorted(Counter(record["class_id"] for record in records).items())),
            "protected_geometry_settings_unchanged": True}



def instrument_world(input_path, model_path, output_directory, camera_name="oak_rgb"):
    """Return ``(runtime_world_path, additional_bridge_entries)``.

    ``model_path`` resolves the default world's ``model://nomad_vehicle``.
    A fresh output directory must be supplied for each launch; keep it alive
    until Gazebo stops because split visual meshes reside beside the world.
    """
    source = Path(input_path).resolve()
    directory = Path(output_directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / "semantic_world.sdf"
    if target.exists():
        raise FileExistsError(f"Refusing to overwrite an existing runtime world: {target}")
    sources, splits = {}, {}
    record = build_world(source, target, camera_name=camera_name,
                         sources=sources, splits=splits,
                         vehicle_model_path=None if model_path is None else Path(model_path))
    if any(digest(path) != expected for path, expected in sources.items()):
        raise AssertionError("A source asset changed during semantic instrumentation")
    camera = record["camera"]
    bridge = [
        {"ros_topic_name": "/nomad/semantic_gt/" + camera_name + "/labels",
         "gz_topic_name": camera["gazebo_labels_topic"],
         "ros_type_name": "sensor_msgs/msg/Image", "gz_type_name": "gz.msgs.Image",
         "direction": "GZ_TO_ROS", "qos_profile": "SENSOR_DATA"},
        {"ros_topic_name": "/nomad/semantic_gt/" + camera_name + "/camera_info",
         "gz_topic_name": camera["gazebo_camera_info_topic"],
         "ros_type_name": "sensor_msgs/msg/CameraInfo", "gz_type_name": "gz.msgs.CameraInfo",
         "direction": "GZ_TO_ROS", "qos_profile": "SENSOR_DATA"}]
    manifest = {"version": VERSION, "protected_geometry_settings_unchanged": True,
                "source_hashes": {str(path): value for path, value in sources.items()},
                "world": record, "point_input_contract_mutable": True}
    (directory / "semantic_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return target, bridge
