"""Verify the vegetation-free comparison world without ROS or a simulator.

Only vegetation may differ. These checks do not measure frame rate or confirm
vehicle dynamics; they keep the comparison's terrain, physics and sensors equal.
"""
import ast
import copy
import hashlib
import json
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def is_vegetation_model(model):
    name = model.attrib.get("name", "")
    return name in {"forest_far", "forest_grass"} or name.startswith("tree_")


def canonical_tree(node):
    """Ignore indentation, attribute order and optional XML comments only."""
    return (node.tag, tuple(sorted(node.attrib.items())), (node.text or "").strip(),
            tuple(canonical_tree(child) for child in node if isinstance(child.tag, str)))


class BareWorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = PACKAGE
        cls.forest_sdf = ET.parse(cls.package / "worlds" / "forest.sdf").getroot()
        cls.bare_sdf = ET.parse(cls.package / "worlds" / "forest_bare.sdf").getroot()
        cls.forest = cls.forest_sdf.find("world")
        cls.bare = cls.bare_sdf.find("world")
        cls.forest_models = {node.attrib["name"]: node for node in cls.forest.findall("model")}
        cls.bare_models = {node.attrib["name"]: node for node in cls.bare.findall("model")}
        cls.full_export = json.loads((cls.package / "assets" / "export.json").read_text())
        cls.bare_export = json.loads((cls.package / "assets" / "export_bare.json").read_text())

    def test_baseline_keeps_vegetation_bare_removes_visuals_and_collisions(self):
        self.assertIn("forest_far", self.forest_models)
        self.assertIn("forest_grass", self.forest_models)
        self.assertTrue(any(name.startswith("tree_") for name in self.forest_models))
        self.assertFalse(any(is_vegetation_model(node) for node in self.bare.findall(".//model")))
        expected = {"terrain", "dead_end_wall", *(f"rock_{index:04d}" for index in range(4))}
        self.assertEqual(set(self.bare_models), expected)
        for uri in self.bare.findall(".//uri"):
            for asset_name in ("forest_far.obj", "grass.obj", "pine_tree.obj", "grass_clump.obj", "shrub.obj"):
                self.assertNotIn(asset_name, uri.text)

    def test_entire_world_diff_is_only_vegetation_removal(self):
        expected = copy.deepcopy(self.forest_sdf)
        world = expected.find("world")
        for model in list(world.findall("model")):
            if is_vegetation_model(model):
                world.remove(model)
        self.assertEqual(canonical_tree(self.bare_sdf), canonical_tree(expected),
                         "Bare world changed something other than vegetation")

    def test_terrain_obstacles_and_vehicle_spawn_are_identical(self):
        for name, node in self.bare_models.items():
            with self.subTest(model=name):
                self.assertEqual(canonical_tree(node), canonical_tree(self.forest_models[name]))
                self.assertEqual(node.findtext("static"), "true")
                self.assertTrue(node.findall(".//visual"))
                self.assertTrue(node.findall(".//collision"))
        for world in (self.forest, self.bare):
            includes = world.findall("include")
            self.assertEqual(len(includes), 1)
            self.assertEqual(includes[0].findtext("uri"), "model://nomad_vehicle")
        self.assertEqual(canonical_tree(self.forest.find("include")), canonical_tree(self.bare.find("include")))

    def test_bare_export_zeroes_render_and_collision_counts(self):
        self.assertIs(self.bare_export["vegetation_enabled"], False)
        for key in ("rendered_trees", "collision_trees", "visual_only_trees", "visual_only_grass"):
            self.assertEqual(self.bare_export[key], 0, key)
            self.assertGreater(self.full_export[key], 0, key)
        self.assertEqual(self.bare_export["rocks"], 4)
        self.assertTrue(self.bare_export["no_roadside_boundary"])

    def test_asset_identity_and_portable_shared_resource_paths(self):
        for key in ("engine", "world", "height_png_sha256", "mesh_grid", "vertex_count", "triangle_count",
                    "bounds", "source_axes", "texture_axes", "rocks"):
            self.assertEqual(self.bare_export[key], self.full_export[key], key)
        height = (self.package / "assets" / "authoring" / "height.png").read_bytes()
        self.assertEqual(hashlib.sha256(height).hexdigest(), self.bare_export["height_png_sha256"])
        width, rows = struct.unpack(">II", height[16:24])
        self.assertEqual([rows, width], self.bare_export["mesh_grid"])
        self.assertEqual([rows, width], [281, 201])
        for node in self.bare.findall(".//uri"):
            uri = node.text
            if uri == "model://nomad_vehicle":
                path = self.package / "models" / "nomad_vehicle"
            else:
                self.assertNotIn("://", uri)
                self.assertFalse(Path(uri).is_absolute())
                path = self.package / "worlds" / uri
            self.assertTrue(path.exists(), uri)
            self.assertTrue(path.resolve().is_relative_to(self.package.resolve()), uri)

    def test_physics_camera_and_sensor_rates_unchanged(self):
        for world in (self.forest, self.bare):
            self.assertEqual(world.attrib["name"], "nomad_forest")
            self.assertEqual(tuple(map(float, world.findtext("gravity").split())), (0, 0, -9.81))
            self.assertAlmostEqual(float(world.findtext("physics/max_step_size")), 1 / 600)
            self.assertEqual(canonical_tree(world.find("gui")), canonical_tree(self.forest.find("gui")))
            self.assertEqual([canonical_tree(plugin) for plugin in world.findall("plugin")],
                             [canonical_tree(plugin) for plugin in self.forest.findall("plugin")])
        # Both worlds share this same local vehicle. No per-comparison sensor
        # overrides or frequency/geometry changes are permitted in either one.
        vehicle = ET.parse(self.package / "models" / "nomad_vehicle" / "model.sdf").getroot().find("model")
        sensors = {node.attrib["name"]: node for node in vehicle.findall(".//sensor")}
        for name in ("oak_rgb", "oak_left", "oak_right"):
            sensor = sensors[name]
            self.assertEqual(float(sensor.findtext("update_rate")), 30)
            self.assertEqual(int(sensor.findtext("camera/image/width")), 640)
            self.assertEqual(int(sensor.findtext("camera/image/height")), 480)
            self.assertAlmostEqual(float(sensor.findtext("pose").split()[0]), 0.46)
            self.assertEqual(float(sensor.findtext("camera/clip/far")), 40)
        self.assertEqual(float(sensors["oak_rgb"].findtext("camera/depth_camera/clip/far")), 8)
        lidar = sensors["ydlidar_g2"]
        self.assertEqual(float(lidar.findtext("update_rate")), 10)
        self.assertEqual(int(lidar.findtext("lidar/scan/horizontal/samples")), 500)

    def test_launch_defaults_to_forest_and_keeps_explicit_world_override(self):
        source = ast.parse((self.package / "launch" / "forest.launch.py").read_text())
        arguments = {}
        for call in ast.walk(source):
            if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                    and call.func.id == "DeclareLaunchArgument" and call.args
                    and isinstance(call.args[0], ast.Constant)):
                default = next((entry.value for entry in call.keywords
                                if entry.arg == "default_value"), None)
                if isinstance(default, ast.Constant):
                    arguments[call.args[0].value] = default.value
        self.assertEqual(arguments["vegetation"].lower(), "true")
        self.assertEqual(arguments["world_file"], "")
        strings = {node.value for node in ast.walk(source) if isinstance(node, ast.Constant)
                   and isinstance(node.value, str)}
        self.assertIn("forest_bare.sdf", strings)
        self.assertIn("forest.sdf", strings)


if __name__ == "__main__":
    unittest.main()
