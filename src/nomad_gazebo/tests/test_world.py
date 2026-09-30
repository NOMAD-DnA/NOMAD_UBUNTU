"""Read-only checks of the generated Gazebo export; no ROS or simulator run.

Authoring data is only ground truth for these tests, not a navigation prior.
Geometric correctness does not prove vehicle handling or real-time sensor rates.
"""
import ast
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def parse_obj(path):
    vertices, uv, normals, faces, uv_indices, normal_indices, materials = [], [], [], [], [], [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "v":
            vertices.append(tuple(map(float, fields[1:4])))
        elif fields[0] == "vt":
            uv.append(tuple(map(float, fields[1:3])))
        elif fields[0] == "vn":
            normals.append(tuple(map(float, fields[1:4])))
        elif fields[0] == "mtllib":
            materials.extend(fields[1:])
        elif fields[0] == "f":
            tokens = [field.split("/") for field in fields[1:]]
            if len(tokens) != 3:
                raise ValueError("Terrain faces must be triangles")
            faces.append([int(token[0]) - 1 for token in tokens])
            uv_indices.append([int(token[1]) - 1 if len(token) > 1 and token[1] else -1 for token in tokens])
            normal_indices.append([int(token[2]) - 1 if len(token) > 2 and token[2] else -1 for token in tokens])
    return {"vertices": np.asarray(vertices), "uv": np.asarray(uv), "normals": np.asarray(normals),
            "faces": np.asarray(faces), "uv_indices": np.asarray(uv_indices),
            "normal_indices": np.asarray(normal_indices), "materials": materials}


def road_distance(point, paths):
    x, y = point
    squared = math.inf
    for points in paths.values():
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            dx, dy = bx - ax, by - ay
            denominator = dx * dx + dy * dy
            fraction = ((x - ax) * dx + (y - ay) * dy) / denominator if denominator else 0
            fraction = min(1, max(0, fraction))
            squared = min(squared, (x - ax - fraction * dx) ** 2 + (y - ay - fraction * dy) ** 2)
    return math.sqrt(squared)


def pose(node):
    values = tuple(map(float, node.findtext("pose", "0 0 0 0 0 0").split()))
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("Invalid finite SDF pose")
    return values


def obj_vertices(path):
    return np.asarray([tuple(map(float, line.split()[1:4]))
                       for line in path.read_text().splitlines() if line.startswith("v ")])


def inertia_matrix(node, urdf=False):
    data = node.attrib if urdf else {child.tag: child.text for child in node}
    xx, yy, zz, xy, xz, yz = [float(data[key]) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


class GazeboWorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = PACKAGE
        cls.geometry = cls.package / "assets" / "geometry"
        cls.authoring = cls.package / "assets" / "authoring"
        cls.layout = json.loads((cls.authoring / "layout.json").read_text())
        cls.export = json.loads((cls.package / "assets" / "export.json").read_text())
        cls.bounds = cls.layout["bounds"]
        with Image.open(cls.authoring / "height.png") as image:
            cls.height_pixels = np.asarray(image).copy()
            cls.height_mode = image.mode
        cls.heights = cls.height_pixels.astype(float) * cls.layout["height_max_m"] / 255
        cls.terrain = parse_obj(cls.geometry / "terrain.obj")
        cls.sdf = ET.parse(cls.package / "worlds" / "forest.sdf").getroot()
        cls.world = cls.sdf.find("world")
        cls.models = cls.world.findall("model")
        cls.named_models = {model.attrib["name"]: model for model in cls.models}
        cls.vehicle = ET.parse(cls.package / "models" / "nomad_vehicle" / "model.sdf").getroot().find("model")
        cls.urdf = ET.parse(cls.package / "urdf" / "nomad_vehicle.urdf").getroot()
        cls.vehicle_data = json.loads((cls.package / "config" / "vehicle.json").read_text())
        cls.sensors_config = yaml.safe_load((cls.package / "config" / "sensors.yaml").read_text())

    @classmethod
    def z_at(cls, xy):
        resolution = cls.layout["height_resolution_m"]
        row = (xy[0] - cls.bounds["x_min"]) / resolution
        col = (xy[1] - cls.bounds["y_min"]) / resolution
        if not (0 <= row <= cls.heights.shape[0] - 1 and 0 <= col <= cls.heights.shape[1] - 1):
            raise ValueError("Point outside authoring heightmap")
        r0, c0 = math.floor(row), math.floor(col)
        r1, c1 = min(r0 + 1, cls.heights.shape[0] - 1), min(c0 + 1, cls.heights.shape[1] - 1)
        low = np.interp(col, [c0, c1], [cls.heights[r0, c0], cls.heights[r0, c1]])
        high = np.interp(col, [c0, c1], [cls.heights[r1, c0], cls.heights[r1, c1]])
        return float(np.interp(row, [r0, r1], [low, high]))

    @classmethod
    def local_uri(cls, uri, context):
        if uri.startswith(("http://", "https://", "fuel://")):
            raise ValueError("Remote resource is not allowed: " + uri)
        if uri.startswith("model://"):
            return cls.package / "models" / uri.removeprefix("model://")
        if uri.startswith("package://nomad_gazebo/"):
            return cls.package / uri.removeprefix("package://nomad_gazebo/")
        if uri.startswith("file://"):
            return Path(uri.removeprefix("file://"))
        if "://" in uri or "${" in uri:
            raise ValueError("Unknown resource URI: " + uri)
        return context / uri

    def test_authoring_axes_dimensions_and_export_integrity(self):
        self.assertEqual(self.layout["axis_convention"], "rows=world +X; columns=world +Y; endpoints included")
        self.assertEqual(self.height_mode, "L")
        self.assertEqual(self.layout["height_resolution_m"], 0.25)
        self.assertEqual(self.height_pixels.shape, (281, 201))
        self.assertEqual(self.layout["boundaries"], [])
        # The PNG used to export the mesh must be recorded by its actual hash.
        digest = hashlib.sha256((self.authoring / "height.png").read_bytes()).hexdigest()
        self.assertEqual(digest, self.export["height_png_sha256"])
        self.assertEqual(self.export["mesh_grid"], list(self.height_pixels.shape))
        self.assertEqual(self.export["vertex_count"], self.height_pixels.size)
        self.assertEqual(self.export["triangle_count"], 2 * 280 * 200)
        self.assertEqual(self.export["source_axes"], "PNG rows +X / columns +Y")
        self.assertEqual(self.export["bounds"], self.bounds)
        self.assertTrue(self.export["no_roadside_boundary"])

    def test_terrain_vertices_reproduce_the_png_grid(self):
        vertices = self.terrain["vertices"]
        rows, cols = self.height_pixels.shape
        self.assertEqual(vertices.shape, (rows * cols, 3))
        self.assertTrue(np.isfinite(vertices).all())
        grid = vertices.reshape(rows, cols, 3)
        resolution = self.layout["height_resolution_m"]
        x = self.bounds["x_min"] + np.arange(rows) * resolution
        y = self.bounds["y_min"] + np.arange(cols) * resolution
        np.testing.assert_allclose(grid[:, :, 0], np.broadcast_to(x[:, None], (rows, cols)), atol=1e-6)
        np.testing.assert_allclose(grid[:, :, 1], np.broadcast_to(y[None, :], (rows, cols)), atol=1e-6)
        np.testing.assert_allclose(grid[:, :, 2], self.heights, rtol=0, atol=1e-5)

    def test_terrain_faces_and_normals_point_up(self):
        data = self.terrain
        rows, cols = self.height_pixels.shape
        self.assertEqual(data["faces"].shape, (2 * (rows - 1) * (cols - 1), 3))
        self.assertGreaterEqual(int(data["faces"].min()), 0)
        self.assertLess(int(data["faces"].max()), len(data["vertices"]))
        triangles = data["vertices"][data["faces"]]
        crosses = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        self.assertTrue((crosses[:, 2] > 0).all())
        np.testing.assert_allclose(crosses[:, 2], np.full(len(crosses), 0.25 ** 2), atol=1e-6)
        self.assertTrue(np.isfinite(data["normals"]).all())
        self.assertGreater(len(data["normals"]), 0)
        self.assertTrue((data["normals"][:, 2] > 0).all())
        np.testing.assert_allclose(np.linalg.norm(data["normals"], axis=1), 1, atol=1e-4)
        self.assertGreaterEqual(int(data["normal_indices"].min()), 0)
        self.assertLess(int(data["normal_indices"].max()), len(data["normals"]))

    def test_uv_axes_and_north_up_texture(self):
        vertices, uv = self.terrain["vertices"], self.terrain["uv"]
        self.assertEqual(uv.shape, (len(vertices), 2))
        expected = np.column_stack([
            (vertices[:, 0] - self.bounds["x_min"]) / (self.bounds["x_max"] - self.bounds["x_min"]),
            (vertices[:, 1] - self.bounds["y_min"]) / (self.bounds["y_max"] - self.bounds["y_min"]),
        ])
        np.testing.assert_allclose(uv, expected, atol=1e-6)
        np.testing.assert_array_equal(self.terrain["uv_indices"], self.terrain["faces"])
        self.assertIn("terrain.mtl", self.terrain["materials"])
        material = (self.geometry / "terrain.mtl").read_text()
        self.assertIn("map_Kd terrain_texture.png", material)
        with Image.open(self.authoring / "terrain.png") as image:
            source = np.asarray(image)
        with Image.open(self.geometry / "terrain_texture.png") as image:
            exported = np.asarray(image)
        from importlib.util import module_from_spec, spec_from_file_location
        spec = spec_from_file_location('nomad_generate_world', self.package / 'scripts' / 'generate_world.py')
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        np.testing.assert_array_equal(exported, np.flipud(np.transpose(module.warm_forest_floor(source), (1, 0, 2))))

    def test_world_plugins_gravity_physics_and_local_resources(self):
        self.assertEqual(self.sdf.attrib["version"], "1.9")
        self.assertEqual(self.world.attrib["name"], "nomad_forest")
        np.testing.assert_allclose(tuple(map(float, self.world.findtext("gravity").split())), [0, 0, -9.81])
        self.assertAlmostEqual(float(self.world.findtext("physics/max_step_size")), 1 / 600, places=8)
        plugins = {node.attrib["name"].split("::")[-1]: node for node in self.world.findall("plugin")}
        for system in ("Physics", "Sensors", "Imu", "UserCommands", "SceneBroadcaster"):
            self.assertIn(system, plugins)
        self.assertEqual(plugins["Sensors"].findtext("render_engine"), "ogre2")
        for node in self.world.findall(".//uri"):
            path = self.local_uri(node.text, self.package / "worlds").resolve()
            self.assertTrue(path.exists(), node.text)
            self.assertTrue(path.is_relative_to(self.package.resolve()), node.text)
        for model in self.models:
            self.assertNotIn("boundary", model.attrib["name"])
            for node in model.findall(".//pose"):
                values = tuple(map(float, node.text.split()))
                self.assertEqual(len(values), 6)
                self.assertTrue(all(math.isfinite(value) for value in values))

    def test_authoring_branch_and_hill_profile_preserved(self):
        waypoints = self.layout["waypoints"]
        np.testing.assert_allclose(waypoints["rocks"], [3, 1])
        np.testing.assert_allclose(waypoints["hill_join"], [7.945125, 9.527875], atol=1e-8)
        self.assertEqual(self.layout["hill_profile"], {
            "start_distance_m": 2.5, "ascent_length_m": 10.0, "crest_length_m": 2.0,
            "descent_length_m": 12.0, "height_m": 2.4,
        })
        branch = np.asarray(self.layout["paths"]["rock_branch"])
        delta = np.diff(branch, axis=0)
        self.assertTrue((delta >= -0.0001).all())
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(delta, axis=1))]
        index = int(np.argmin(np.linalg.norm(branch - waypoints["rocks"], axis=1)))
        np.testing.assert_allclose(branch[index], waypoints["rocks"], atol=0.0001)
        self.assertTrue(0 < index < len(branch) - 1)
        self.assertGreaterEqual(arc[index] / arc[-1], 0.3)
        self.assertLessEqual(arc[index] / arc[-1], 0.7)
        self.assertGreaterEqual(arc[-1] - arc[index], 5)
        np.testing.assert_allclose(branch[-1], waypoints["hill_join"], rtol=0, atol=0.00006)
        hill = np.asarray(self.layout["paths"]["uphill"])
        self.assertLess(float(np.linalg.norm(hill - waypoints["hill_join"], axis=1).min()), 0.12)
        hill_arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(hill, axis=0), axis=1))]
        samples = np.arange(0, hill_arc[-1] - 1, 0.25)
        def levels(at):
            points = np.column_stack([np.interp(at, hill_arc, hill[:, axis]) for axis in (0, 1)])
            return np.asarray([self.z_at(point) for point in points])
        grade = np.degrees(np.arctan(levels(samples + 1) - levels(samples)))
        self.assertGreaterEqual(float(grade[samples <= 12.5].max()), 16)
        self.assertLessEqual(float(grade[samples <= 12.5].max()), 24)
        self.assertLess(float(grade[(samples >= 14.5) & (samples <= 26.5)].min()), -10)
        self.assertGreater(float(levels(np.arange(12.75, 14.25, 0.25)).min()), 2.3)
        self.assertLess(self.z_at(waypoints["goal"]), 0.1)

    def test_tree_distance_policy_and_visual_only_batches(self):
        threshold = self.layout["tree_collision_distance_m"]
        self.assertEqual(threshold, 5.0)
        expected_near = {}
        far = []
        for index, tree in enumerate(self.layout["trees"]):
            distance = road_distance(tree["center"], self.layout["paths"])
            self.assertAlmostEqual(distance, tree["road_distance_m"], delta=0.00015)
            self.assertIs(tree["collision_enabled"], distance <= threshold)
            if tree["collision_enabled"]:
                expected_near[f"tree_{index:04d}"] = tree
            else:
                far.append(tree)
        self.assertTrue(expected_near)
        self.assertTrue(far)
        actual = {name: model for name, model in self.named_models.items() if name.startswith("tree_")}
        self.assertEqual(set(actual), set(expected_near))
        self.assertEqual(self.export["collision_trees"], len(actual))
        self.assertEqual(self.export["visual_only_trees"], len(far))
        self.assertEqual(self.export["rendered_trees"], len(actual) + len(far))
        for name, tree in expected_near.items():
            model = actual[name]
            np.testing.assert_allclose(pose(model)[:3], [*tree["center"], self.z_at(tree["center"])], atol=1e-5)
            self.assertAlmostEqual(pose(model)[5], math.radians(tree["yaw_deg"]))
            self.assertEqual(model.findtext("static"), "true")
            dimensions = tuple(map(float, model.findtext("link/collision/geometry/box/size").split()))
            np.testing.assert_allclose(dimensions, [0.44, 0.44, 3.35 * tree["scale"]])
        for name, source_name, instances, destination in (
            ("forest_far", "pine_tree", far, "forest_far.obj"),
            ("forest_grass", "grass_clump", self.layout["grass"], "grass.obj"),
        ):
            model = self.named_models[name]
            self.assertEqual(model.findtext("static"), "true")
            self.assertFalse(model.findall(".//collision"))
            np.testing.assert_allclose(pose(model), np.zeros(6))
            self.assertEqual(model.findtext("link/visual/geometry/mesh/uri"),
                             "../assets/geometry/" + destination)
            original = obj_vertices(self.package / "assets" / "models" / (source_name + ".obj"))
            baked = obj_vertices(self.geometry / destination)
            self.assertEqual(baked.shape, (len(original) * len(instances), 3))
            expected = []
            for instance in instances:
                yaw = math.radians(instance["yaw_deg"])
                c, s = math.cos(yaw), math.sin(yaw)
                rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                origin = [*instance["center"], self.z_at(instance["center"])]
                expected.append(original @ rotation.T * instance.get("scale", 1) + origin)
            np.testing.assert_allclose(baked, np.concatenate(expected), rtol=0, atol=1e-7)
        self.assertEqual(self.export["visual_only_grass"], len(self.layout["grass"]))

    def test_finite_rock_wall_geometry_and_vehicle_spawn(self):
        for index, rock in enumerate(self.layout["rocks"]):
            model = self.named_models[f"rock_{index:04d}"]
            np.testing.assert_allclose(pose(model)[:3], [*rock["center"], self.z_at(rock["center"])])
            self.assertAlmostEqual(pose(model)[5], math.radians(rock["yaw_deg"]))
            cylinder = model.find("link/collision/geometry/cylinder")
            self.assertAlmostEqual(float(cylinder.findtext("radius")), 0.65)
            self.assertAlmostEqual(float(cylinder.findtext("length")), 1.1)
            self.assertAlmostEqual(pose(model.find("link/collision"))[2], 0.55)
        wall = self.layout["wall"]
        model = self.named_models["dead_end_wall"]
        np.testing.assert_allclose(pose(model)[:3], [*wall["center"], self.z_at(wall["center"])])
        for kind in ("visual", "collision"):
            node = model.find("link/" + kind)
            np.testing.assert_allclose(tuple(map(float, node.findtext("geometry/box/size").split())),
                                       [0.65, 4.8, 2.2])
            self.assertAlmostEqual(pose(node)[2], 1.1)
        includes = self.world.findall("include")
        self.assertEqual(len(includes), 1)
        self.assertEqual(includes[0].findtext("uri"), "model://nomad_vehicle")
        spawn = pose(includes[0])
        np.testing.assert_allclose(spawn[:3], [*self.layout["waypoints"]["start"], 0.03])
        a, b = self.layout["paths"]["entrance"][:2]
        self.assertAlmostEqual(spawn[5], math.atan2(b[1] - a[1], b[0] - a[0]))

    def test_vehicle_dynamic_joints_frames_and_positive_inertias(self):
        data = self.vehicle_data
        sdf_joints = {node.attrib["name"]: node for node in self.vehicle.findall("joint")}
        urdf_joints = {node.attrib["name"]: node for node in self.urdf.findall("joint")
                       if node.attrib["type"] != "fixed"}
        expected = {joint["name"] for joint in data["dynamic_joints"]}
        self.assertEqual(set(sdf_joints), expected)
        self.assertEqual(set(urdf_joints), expected)
        for joint in data["dynamic_joints"]:
            name = joint["name"]
            sdf, urdf = sdf_joints[name], urdf_joints[name]
            for relation in ("parent", "child"):
                self.assertEqual(sdf.findtext(relation), joint[relation])
                self.assertEqual(urdf.find(relation).attrib["link"], joint[relation])
            np.testing.assert_allclose(tuple(map(float, sdf.findtext("axis/xyz").split())), joint["axis"])
            np.testing.assert_allclose(tuple(map(float, urdf.find("axis").attrib["xyz"].split())), joint["axis"])
            self.assertGreater(float(sdf.findtext("axis/limit/effort")), 0)
            self.assertGreater(float(urdf.find("limit").attrib["effort"]), 0)
        sdf_links = {node.attrib["name"]: node for node in self.vehicle.findall("link")}
        urdf_links = {node.attrib["name"]: node for node in self.urdf.findall("link")}
        self.assertEqual(set(sdf_links), set(data["physical_links"]))
        total_mass = 0
        for name in data["physical_links"]:
            sdf, urdf = sdf_links[name].find("inertial"), urdf_links[name].find("inertial")
            mass = float(sdf.findtext("mass"))
            self.assertGreater(mass, 0)
            self.assertAlmostEqual(mass, float(urdf.find("mass").attrib["value"]))
            total_mass += mass
            matrix = inertia_matrix(sdf.find("inertia"))
            self.assertTrue(np.isfinite(matrix).all())
            self.assertTrue((np.linalg.eigvalsh(matrix) > 0).all())
            np.testing.assert_allclose(matrix, inertia_matrix(urdf.find("inertia"), urdf=True), atol=1e-12)
        self.assertAlmostEqual(total_mass, data["total_mass_kg"])
        fixed = {joint.find("child").attrib["link"]: joint for joint in self.urdf.findall("joint")
                 if joint.attrib["type"] == "fixed"}
        frames = {node.attrib["name"]: node for node in self.vehicle.findall("frame")}
        for item in data["fixed_frames"]:
            frame = frames[item["name"]]
            self.assertEqual(frame.attrib["attached_to"], item["parent"])
            np.testing.assert_allclose(pose(frame), [*item["xyz"], *item["rpy"]], atol=1e-12)
            joint = fixed[item["name"]]
            self.assertEqual(joint.find("parent").attrib["link"], item["parent"])
            for key, expected_xyz in (("xyz", item["xyz"]), ("rpy", item["rpy"])):
                np.testing.assert_allclose(tuple(map(float, joint.find("origin").attrib[key].split())),
                                           expected_xyz, atol=1e-12)

    def test_vehicle_camera_lidar_and_native_topic_contract(self):
        data = self.vehicle_data
        parameters, topics = data["sensor_parameters"], data["native_topics"]
        self.assertEqual(data["camera_source"], self.sensors_config["camera"])
        self.assertEqual(data["lidar_source"], self.sensors_config["lidar"])
        sensors = {node.attrib["name"]: node for node in self.vehicle.findall(".//sensor")}
        self.assertEqual(set(sensors), {"oak_rgb", "oak_left", "oak_right", "ydlidar_g2", "oak_imu"})
        for side in ("rgb", "left", "right"):
            sensor = sensors["oak_" + side]
            self.assertEqual(sensor.attrib["type"], "rgbd_camera" if side == "rgb" else "camera")
            self.assertEqual(float(sensor.findtext("update_rate")), 30)
            self.assertEqual(sensor.findtext("gz_frame_id"), f"oak_{side}_optical_frame")
            self.assertEqual(sensor.findtext("camera/optical_frame_id"), f"oak_{side}_optical_frame")
            self.assertEqual(int(sensor.findtext("camera/image/width")), 640)
            self.assertEqual(int(sensor.findtext("camera/image/height")), 480)
            self.assertGreaterEqual(float(sensor.findtext("camera/clip/far")), 40.0)
            if side == "rgb":
                self.assertAlmostEqual(float(sensor.findtext("camera/depth_camera/clip/near")), 0.4)
                self.assertAlmostEqual(float(sensor.findtext("camera/depth_camera/clip/far")), 8.0)
                self.assertEqual(sensor.findtext("topic") + "/image", topics["rgb_image"])
                self.assertEqual(sensor.findtext("topic") + "/depth_image", topics["depth_image"])
            else:
                self.assertEqual(sensor.findtext("topic"), topics[side + "_image"])
                self.assertEqual(sensor.findtext("camera/camera_info_topic"), topics[side + "_camera_info"])
        baseline = pose(sensors["oak_left"])[1] - pose(sensors["oak_right"])[1]
        self.assertAlmostEqual(baseline, parameters["baseline_m"])
        right = sensors["oak_right"]
        fx = float(right.findtext("camera/lens/intrinsics/fx"))
        self.assertAlmostEqual(float(right.findtext("camera/lens/projection/tx")), -fx * baseline)
        lidar = sensors["ydlidar_g2"]
        self.assertEqual(lidar.findtext("topic"), topics["scan"])
        self.assertEqual(lidar.findtext("gz_frame_id"), "scan")
        self.assertEqual(float(lidar.findtext("update_rate")), 10)
        self.assertEqual(int(lidar.findtext("lidar/scan/horizontal/samples")), 500)
        self.assertEqual(int(lidar.findtext("lidar/scan/vertical/samples")), 1)
        self.assertAlmostEqual(float(lidar.findtext("lidar/range/min")), 0.12)
        self.assertAlmostEqual(float(lidar.findtext("lidar/range/max")), 12)
        imu = sensors["oak_imu"]
        self.assertEqual(imu.attrib["type"], "imu")
        self.assertEqual(imu.findtext("topic"), topics["imu"])
        self.assertEqual(imu.findtext("gz_frame_id"), "oak_imu_frame")
        self.assertAlmostEqual(float(imu.findtext("update_rate")), 200.0)
        low = float(lidar.findtext("lidar/scan/horizontal/min_angle"))
        high = float(lidar.findtext("lidar/scan/horizontal/max_angle"))
        self.assertAlmostEqual(high - low + 2 * math.pi / 500, 2 * math.pi)
        plugins = {node.attrib["name"].split("::")[-1]: node for node in self.vehicle.findall("plugin")}
        ackermann = plugins["AckermannSteering"]
        self.assertEqual(ackermann.findtext("topic"), topics["cmd_vel"])
        self.assertEqual([ackermann.findtext("left_joint"), ackermann.findtext("right_joint")], data["driven_joints"])
        self.assertEqual(ackermann.findtext("odom_topic"), topics["wheel_odom"])
        odom = plugins["OdometryPublisher"]
        self.assertEqual(int(odom.findtext("dimensions")), 3)
        self.assertEqual(odom.findtext("odom_topic"), topics["odom"])
        self.assertEqual(odom.findtext("tf_topic"), topics["tf"])

        bridge = yaml.safe_load((self.package / "config" / "bridge.yaml").read_text())
        by_ros = {item["ros_topic_name"]: item for item in bridge}
        self.assertEqual(len(by_ros), len(bridge), "Duplicate bridge output/input topic")
        self.assertEqual(by_ros[topics["cmd_vel"]]["gz_topic_name"], ackermann.findtext("topic"))
        self.assertEqual(by_ros[topics["cmd_vel"]]["direction"], "ROS_TO_GZ")
        for ros, native in (("/odom", topics["odom"]), ("/tf", topics["tf"]),
                            ("/joint_states", topics["joint_states"]), ("/nomad/raw/scan", topics["scan"])):
            self.assertEqual(by_ros[ros]["gz_topic_name"], native)
            self.assertEqual(by_ros[ros]["direction"], "GZ_TO_ROS")
        # The bridge's omitted profile is reliable KeepLast; "DEFAULT" is not
        # a valid Jazzy profile name and would drop the whole scan bridge entry.
        self.assertNotIn("qos_profile", by_ros["/nomad/raw/scan"])
        self.assertNotIn(topics["wheel_odom"], {item["gz_topic_name"] for item in bridge})
        self.assertNotIn(topics["wheel_tf"], {item["gz_topic_name"] for item in bridge})
        self.assertNotIn("/cmd_vel", by_ros, "Commands must pass through the watchdog first")
        for side in ("rgb", "depth", "left", "right"):
            for suffix, native_key in (("image", side + "_image"),
                                       ("camera_info", "rgbd_camera_info" if side in ("rgb", "depth")
                                        else side + "_camera_info")):
                ros = f"/nomad/raw/oak/{side}/{suffix}"
                self.assertEqual(by_ros[ros]["gz_topic_name"], topics[native_key])
                self.assertEqual(by_ros[ros]["direction"], "GZ_TO_ROS")
                self.assertEqual(by_ros[ros]["qos_profile"], "SENSOR_DATA")

        # Inspect scripts rather than importing ROS or starting a command node.
        watchdog = ast.parse((self.package / "scripts" / "command_watchdog.py").read_text())
        calls = [node for node in ast.walk(watchdog) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Attribute)]
        publishers = [node.args[1].value for node in calls if node.func.attr == "create_publisher"]
        subscribers = [node.args[1].value for node in calls if node.func.attr == "create_subscription"]
        self.assertEqual(publishers, [topics["cmd_vel"]])
        self.assertEqual(subscribers, ["/cmd_vel"])
        timeout = next(node.args[1].value for node in calls if node.func.attr == "declare_parameter"
                       and node.args[0].value == "timeout_s")
        timer_period = next(node.args[0].value for node in calls if node.func.attr == "create_timer")
        self.assertTrue(0.1 <= timeout <= 1)
        self.assertTrue(0 < timer_period <= timeout / 2)
        self.assertTrue(any(node.func.attr == "monotonic" for node in calls))
        self.assertTrue(any(isinstance(node, ast.Attribute) and node.attr == "STEADY_TIME"
                            for node in ast.walk(watchdog)))
        launch = ast.parse((self.package / "launch" / "forest.launch.py").read_text())
        executables = {keyword.value.value for node in ast.walk(launch) if isinstance(node, ast.Call)
                       for keyword in node.keywords if keyword.arg == "executable"
                       and isinstance(keyword.value, ast.Constant)}
        self.assertIn("command_watchdog.py", executables)
        self.assertIn("gazebo_sensor_adapter.py", executables)
        self.assertIn("lidar_bridge.py", executables)
        packages = {keyword.value.value for node in ast.walk(launch) if isinstance(node, ast.Call)
                    for keyword in node.keywords if keyword.arg == "package"
                    and isinstance(keyword.value, ast.Constant)}
        self.assertNotIn("nomad_sim", packages)
        self.assertNotIn("mvsim", packages)
        dependencies = {node.text for node in ET.parse(self.package / "package.xml").getroot()
                        if node.tag.endswith("depend")}
        self.assertNotIn("nomad_sim", dependencies)
        self.assertNotIn("mvsim", dependencies)
        adapter = ast.parse((self.package / "scripts" / "gazebo_sensor_adapter.py").read_text())
        strings = {node.value for node in ast.walk(adapter) if isinstance(node, ast.Constant)
                   and isinstance(node.value, str)}
        for required in ("/nomad/raw/oak/", "/oak/", "/image", "/camera_info", "image_rect", "image_raw"):
            self.assertIn(required, strings)


if __name__ == "__main__":
    unittest.main()
