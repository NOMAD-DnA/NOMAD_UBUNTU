"""Check generated forest artifacts without importing or executing the generator.

These are point-grid topology/asset checks, not an Ackermann turning-radius,
controller, or motor-torque test. Dirt roads are visual markings, NOT a physical
fence or navigation prior: off-road travel and side bypasses must remain possible.
Run after generate_forest_map.py has produced the package assets. No ROS process,
upstream package, or simulator is required.
"""
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

# Frozen signatures after the separately approved road-overlap repair. The
# uphill geometry/height, sensors, sky assets and outer-tree thinning policy
# stay unchanged; only road layout and its resulting texture/vegetation move.
# No sibling baseline directory is required.
REFERENCE_CORE_KEYS = (
    "bounds", "seed", "resolution_m", "height_resolution_m",
    "mask_resolution_m", "texture_resolution_m", "height_max_m",
    "render_texture_resolution_m", "hill_profile", "road_width_m",
    "corridor_width_m", "tree_collision_distance_m", "axis_convention",
    "waypoints", "paths", "boundaries", "wall", "rocks", "blocked_probes", "scenario",
)
REFERENCE_ASSET_SHA256 = {
    "height.png": "def563b89e56569434478987fb0f33bdf9cb4765560b872fb301d4e213fb2205",
    "terrain.png": "ad6bee882ad5aab92ca824216143feb56170f5c133a855a02330f21885dc5ae3",
    "terrain_render.png": "8b85d0fa6e087f51908524c5f57c02d956fe77779163cb477053611fbffa708b",
    "road_mask.png": "e15573ce01e916f364cc63caa1814c1ece778725142c98d0d431532ea597a4da",
    # The internal rock gate and road-near tree footprints follow the new road.
    "collision_mask.png": "04871843a2a79204b81703e1b767e679ec76b216061d6fc445d954e78f8f0c2d",
}
REFERENCE_SENSOR_SHA256 = {
    "config/sensors.yaml": "a7c0ac4ec7c26e1cef36775f95815361b4c6bbc2ceeaf900e01bb6a66d96ef59",
    "sensors/lidar2d.sensor.xml": "2ad35a864298a2f0012f9764c5f48b31da9affc319613f6e5bda5731a5f11562",
    "sensors/oak_mono.sensor.xml": "e2c31ed79f1512c2d4ef368012a5fe74f349d68e7d07ab3950da10167acccfbf",
    "sensors/oak_rgbd.sensor.xml": "e0bf1b654667fc66ac1c342832593fb9d894cf1db42478cf67ca824a865ef909",
}
REFERENCE_SKY_SHA256 = {
    "LICENSE.txt": "86971749a37705385a63e6208dbdab2ffc7055d10ca3e7b670d87cec8ac1e4e1",
    "README.txt": "14d41ffed0113c485b73122956ea5f05506f065da241e2b18c41dd1262c7ab0f",
    "SunSetBack.jpg": "5c682d9d8159da5453fdf642d7a232427293b804653a49540407cff69c5e3737",
    "SunSetDown.jpg": "a5357066fb0bc41963a1075e7639195961f096e991a2f04267bc0613366ef273",
    "SunSetFront.jpg": "cf478c9f6f31bc0a6fcf184801f59da862e06049d3f99b2306b5dc38a55eafb6",
    "SunSetLeft.jpg": "333a15f0c2bd21615707f330d31455643e0940ed11110a3685aeafd04fe9f715",
    "SunSetRight.jpg": "f385307582c5b4e5b01baa2470aef06334b1a09c5d989674cf30f4cb276cac76",
    "SunSetUp.jpg": "070d22a4168821a3b3e7ae103cdc0c7438cbfd681c450b0abd22486b9fcd6f08",
}
REFERENCE_REMOVED_INDICES = (
    2, 4, 6, 8, 10, 22, 34, 44, 52, 58, 66, 74, 84, 90, 98,
    104, 112, 120, 124, 128, 134, 146, 160, 162, 164, 166, 168, 170, 172, 174,
)


def data_digest(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def shape_vertices(node):
    return [tuple(map(float, point.text.split()))
            for point in node.findall("shape/pt")]


def pose_values(node):
    """Return x, y, z, yaw, pitch, roll for either MVSim pose form."""
    text = node.findtext("init_pose3d")
    if text is not None:
        values = tuple(map(float, text.split()))
        if len(values) != 6:
            raise ValueError("init_pose3d must contain x y z yaw pitch roll")
    else:
        text = node.findtext("init_pose")
        if text is None:
            raise ValueError("Missing init_pose or init_pose3d")
        values = tuple(map(float, text.split()))
        if len(values) != 3:
            raise ValueError("init_pose must contain x y yaw")
        values = (values[0], values[1], 0.0, values[2], 0.0, 0.0)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Nonfinite pose")
    return values


def flat_pose(node):
    values = pose_values(node)
    return values[0], values[1], values[3]


def world_vertices(vertices, pose):
    x, y, yaw = pose
    angle = math.radians(yaw)
    cosine, sine = math.cos(angle), math.sin(angle)
    return [(x + cosine * px - sine * py, y + sine * px + cosine * py)
            for px, py in vertices]


def point_in_polygon(point, vertices):
    """Ray crossing, used to exempt only actual wall/rock footprints."""
    x, y = point
    inside = False
    for (ax, ay), (bx, by) in zip(vertices, vertices[1:] + vertices[:1]):
        if (ay > y) != (by > y):
            crossing = ax + (y - ay) * (bx - ax) / (by - ay)
            if x < crossing:
                inside = not inside
    return inside


def is_intangible(block, classes):
    text = block.findtext("intangible")
    if text is None:
        definition = classes.get(block.attrib.get("class"))
        text = definition.findtext("intangible", "false") if definition is not None else "false"
    value = text.strip().lower()
    if value not in ("true", "false"):
        raise ValueError("Unexpected intangible value: " + text)
    return value == "true"


def distance_to_roads(point, paths):
    """Exact distance to the recorded polyline segments, not sampled vertices."""
    x, y = point
    best_squared = math.inf
    for points in paths.values():
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            dx, dy = bx - ax, by - ay
            length_squared = dx * dx + dy * dy
            fraction = ((x - ax) * dx + (y - ay) * dy) / length_squared if length_squared else 0
            fraction = min(1.0, max(0.0, fraction))
            squared = (x - ax - fraction * dx) ** 2 + (y - ay - fraction * dy) ** 2
            best_squared = min(best_squared, squared)
    return math.sqrt(best_squared)


def flood_fill(blocked, start):
    """Four-neighbor fill over ALL physical free space, not just road_mask."""
    reached = np.zeros(blocked.shape, dtype=bool)
    if blocked[start]:
        return reached
    reached[start] = True
    pending = deque([start])
    rows, columns = blocked.shape
    while pending:
        row, column = pending.popleft()
        for nr, nc in ((row - 1, column), (row + 1, column),
                       (row, column - 1), (row, column + 1)):
            if (0 <= nr < rows and 0 <= nc < columns
                    and not blocked[nr, nc] and not reached[nr, nc]):
                reached[nr, nc] = True
                pending.append((nr, nc))
    return reached


class ForestMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = PACKAGE_ROOT
        cls.asset_dir = cls.package / "assets" / "forest"
        cls.metadata = json.loads((cls.asset_dir / "layout.json").read_text())
        cls.bounds = cls.metadata["bounds"]
        cls.images = {}
        cls.modes = {}
        for name in ("height", "terrain", "terrain_render", "road_mask", "collision_mask"):
            with Image.open(cls.asset_dir / (name + ".png")) as image:
                cls.modes[name] = image.mode
                cls.images[name] = np.array(image)
        # block:class is valid MVSim syntax, but not namespace-bound XML.
        text = (cls.package / "worlds" / "forest.world.xml").read_text()
        cls.world = ET.fromstring(text.replace("block:class", "block_class"))
        cls.classes = {node.attrib["name"]: node
                       for node in cls.world.findall("block_class")}
        cls.blocks = {node.attrib["name"]: node
                      for node in cls.world.findall("block")}
        cls.polygons = {}
        for name, block in cls.blocks.items():
            vertices = shape_vertices(block)
            if not vertices and "class" in block.attrib:
                vertices = shape_vertices(cls.classes[block.attrib["class"]])
            geometry = block.find("geometry")
            if geometry is not None:
                if geometry.attrib["type"] != "box":
                    raise ValueError("Unsupported collision geometry")
                hx = float(geometry.attrib["lx"]) / 2
                hy = float(geometry.attrib["ly"]) / 2
                vertices = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
            if not vertices:
                raise ValueError("No collision footprint for " + name)
            cls.polygons[name] = world_vertices(vertices, flat_pose(block))
        # Retain all objects for visual/pose/Z tests, but intangible instances
        # (or class defaults) have no physical footprint in Box2D.
        cls.collision_polygons = {
            name: vertices for name, vertices in cls.polygons.items()
            if not is_intangible(cls.blocks[name], cls.classes)
        }

        # Independently rasterize the real XML footprints, including grass.
        # This guards against a correct-looking PNG whose world is passable.
        rows, columns = cls.dimensions(cls.metadata["mask_resolution_m"])
        image = Image.new("L", (columns, rows), 0)
        draw = ImageDraw.Draw(image)
        resolution = cls.metadata["mask_resolution_m"]
        for vertices in cls.collision_polygons.values():
            draw.polygon([((y - cls.bounds["y_min"]) / resolution,
                           (x - cls.bounds["x_min"]) / resolution)
                          for x, y in vertices], fill=255)
        cls.xml_blocked = np.array(image) > 0

    @classmethod
    def dimensions(cls, resolution):
        return (round((cls.bounds["x_max"] - cls.bounds["x_min"]) / resolution) + 1,
                round((cls.bounds["y_max"] - cls.bounds["y_min"]) / resolution) + 1)

    @classmethod
    def pixel(cls, point, resolution):
        return (round((point[0] - cls.bounds["x_min"]) / resolution),
                round((point[1] - cls.bounds["y_min"]) / resolution))

    @classmethod
    def terrain_height_at_xy(cls, point):
        """Interpolate decoded PNG samples, independently of the generator."""
        heights = cls.images["height"].astype(float) * cls.metadata["height_max_m"] / 255
        resolution = cls.metadata["height_resolution_m"]
        row = (point[0] - cls.bounds["x_min"]) / resolution
        column = (point[1] - cls.bounds["y_min"]) / resolution
        if not (0 <= row <= heights.shape[0] - 1
                and 0 <= column <= heights.shape[1] - 1):
            raise ValueError("Block lies outside the elevation image")
        r0, c0 = math.floor(row), math.floor(column)
        r1, c1 = min(r0 + 1, heights.shape[0] - 1), min(c0 + 1, heights.shape[1] - 1)
        lower_row = np.interp(column, [c0, c1], [heights[r0, c0], heights[r0, c1]])
        upper_row = np.interp(column, [c0, c1], [heights[r1, c0], heights[r1, c1]])
        return float(np.interp(row, [r0, r1], [lower_row, upper_row]))

    @classmethod
    def polygon_occupies_cell(cls, vertices, cell):
        """Identify another object's occupancy with the same raster semantics.

        Tiny grass can cover part of a 10-cm cell without containing its center
        or any of nine center-offset probes, so use the actual polygon raster.
        """
        resolution = cls.metadata["mask_resolution_m"]
        points = [((y - cls.bounds["y_min"]) / resolution,
                   (x - cls.bounds["x_min"]) / resolution) for x, y in vertices]
        columns, rows = zip(*points)
        if not (math.floor(min(rows)) - 1 <= cell[0] <= math.ceil(max(rows)) + 1
                and math.floor(min(columns)) - 1 <= cell[1] <= math.ceil(max(columns)) + 1):
            return False
        image = Image.new("L", (cls.xml_blocked.shape[1], cls.xml_blocked.shape[0]), 0)
        ImageDraw.Draw(image).polygon(points, fill=255)
        return image.getpixel((cell[1], cell[0])) != 0

    def test_metadata_contract_and_branch_endpoints(self):
        metadata = self.metadata
        self.assertEqual(metadata["axis_convention"],
                         "rows=world +X; columns=world +Y; endpoints included")
        self.assertEqual(metadata["resolution_m"], metadata["height_resolution_m"])
        self.assertEqual(metadata["height_resolution_m"], 0.25)
        self.assertEqual(metadata["mask_resolution_m"], 0.10)
        self.assertEqual(metadata["texture_resolution_m"], 0.05)
        self.assertEqual(metadata["render_texture_resolution_m"], metadata["height_resolution_m"])
        self.assertEqual(metadata["hill_profile"], {
            "start_distance_m": 2.5, "ascent_length_m": 10.0,
            "crest_length_m": 2.0, "descent_length_m": 12.0, "height_m": 2.4,
        })
        self.assertEqual(metadata["hill_profile"]["height_m"], metadata["height_max_m"])
        expected_waypoints = {
            "start": [-30.0, -18.0], "fork1": [-20.0, -12.0],
            "fork2": [-3.0, -8.0], "rocks": [3.0, 1.0],
            "wall": [21.0, 10.0], "goal": [23.0, 16.0],
            "hill_join": [7.945125, 9.527875],
        }
        self.assertEqual(set(metadata["waypoints"]), set(expected_waypoints))
        for name, expected in expected_waypoints.items():
            np.testing.assert_allclose(metadata["waypoints"][name], expected, rtol=0, atol=1e-8)
        self.assertEqual(metadata["scenario"],
                         ["start", "fork1", "fork2", "wall", "fork2", "fork1", "goal"])
        endpoints = {
            "entrance": ("start", "fork1"),
            "uphill": ("fork1", "goal"),
            "flat_to_fork": ("fork1", "fork2"),
            "rock_branch": ("fork2", "hill_join"),
            "flat_to_wall": ("fork2", "wall"),
        }
        self.assertEqual(set(metadata["paths"]), set(endpoints))
        for name, (start, end) in endpoints.items():
            path = np.asarray(metadata["paths"][name], dtype=float)
            with self.subTest(path=name):
                self.assertEqual(path.shape[1], 2)
                self.assertTrue(np.isfinite(path).all())
                np.testing.assert_allclose(path[0], metadata["waypoints"][start], rtol=0, atol=0.00006)
                np.testing.assert_allclose(path[-1], metadata["waypoints"][end], rtol=0, atol=0.00006)
                self.assertLess(np.linalg.norm(np.diff(path, axis=0), axis=1).max(), 0.12)
        self.assertEqual(metadata["road_width_m"], 3.0)
        self.assertEqual(metadata["corridor_width_m"], metadata["road_width_m"])
        self.assertEqual(metadata["boundaries"], [])

    def test_outer_thinning_is_partial_and_preserves_every_road_near_tree(self):
        metadata = self.metadata
        self.assertEqual(metadata["original_tree_count"], 175)
        self.assertEqual(metadata["removed_outer_tree_count"], 30)
        self.assertEqual(metadata["tree_outer_edge_band_m"], 5.0)
        self.assertEqual(metadata["tree_outer_thinning_modulo"], 2)
        self.assertEqual(metadata["tree_outer_thinning_remainder"], 0)
        self.assertEqual(metadata["tree_outer_pruning_policy"],
                         "thinning_even_original_index; road-near trees always retained")
        removed = metadata["removed_outer_original_indices"]
        self.assertEqual(tuple(removed), REFERENCE_REMOVED_INDICES)
        self.assertEqual(len(removed), metadata["removed_outer_tree_count"])
        self.assertTrue(all(index % 2 == 0 for index in removed))
        kept_indices = [index for index in range(metadata["original_tree_count"])
                        if index not in set(removed)]
        trees = metadata["trees"]
        self.assertEqual(len(trees), 145)
        self.assertEqual(len(kept_indices), len(trees))
        # This frozen expected subset checks centers/yaws/scales/distances and
        # order independently of the generator and its reported deletion list.
        self.assertEqual(data_digest(trees),
                         "20c2ebba587b2cdc4cce07af410c3514e628114d05ca209f610b4ae707c4066f")
        near = [tree for tree in trees if tree["collision_enabled"]]
        self.assertEqual(len(near), 20)
        self.assertEqual(data_digest(near),
                         "53bebd8b758edaf123e57cdffc798d8f897031f889edf2d478f4bb5b43b5b075")
        far_outer = []
        for original_index, tree in zip(kept_indices, trees):
            x, y = tree["center"]
            edge_distance = min(x-self.bounds["x_min"], self.bounds["x_max"]-x,
                                y-self.bounds["y_min"], self.bounds["y_max"]-y)
            if not tree["collision_enabled"] and edge_distance < 5.0:
                far_outer.append(tree)
                self.assertEqual(original_index % 2, 1,
                                 "An even-index outer distant tree was not thinned")
        self.assertEqual(len(far_outer), 29, "Outer trees must be thinned, not entirely removed")

    def test_approved_road_layout_preserves_hill_sensors_and_sky(self):
        metadata = self.metadata
        self.assertEqual(data_digest({key: metadata[key] for key in REFERENCE_CORE_KEYS}),
                         "2fd2419956c8bd64bb37844594e98d25d15c8bf38653c939785fcc7f312b1ac2")
        self.assertEqual(len(metadata["grass"]), 157)
        self.assertEqual(data_digest(metadata["grass"]),
                         "14282df27c8c2423884a36c4cdee9662f8ae7f089606d091bc35954143e9fa3d")
        for filename, expected in REFERENCE_ASSET_SHA256.items():
            with self.subTest(asset=filename):
                self.assertEqual(hashlib.sha256((self.asset_dir / filename).read_bytes()).hexdigest(),
                                 expected, "Unexpected change outside the approved road-layout contract")
        for filename, expected in REFERENCE_SENSOR_SHA256.items():
            with self.subTest(sensor=filename):
                self.assertEqual(hashlib.sha256((self.package / filename).read_bytes()).hexdigest(),
                                 expected, "This edit must not alter sensor settings")
        for filename, expected in REFERENCE_SKY_SHA256.items():
            with self.subTest(sky=filename):
                path = self.package / "assets" / "skybox" / "SunSet" / filename
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected,
                                 "This road edit must not alter the licensed sky assets")

    def test_image_dimensions_and_axes(self):
        for name, key, mode in (("height", "height_resolution_m", "L"),
                                ("terrain", "texture_resolution_m", "RGB"),
                                ("terrain_render", "render_texture_resolution_m", "RGB"),
                                ("road_mask", "mask_resolution_m", "L"),
                                ("collision_mask", "mask_resolution_m", "L")):
            with self.subTest(image=name):
                self.assertEqual(self.modes[name], mode)
                self.assertEqual(self.images[name].shape[:2],
                                 self.dimensions(self.metadata[key]))
        for name in ("road_mask", "collision_mask"):
            self.assertEqual(set(np.unique(self.images[name])), {0, 255})
        # Unequal X/Y extents make an accidental transpose observable.
        self.assertEqual(self.images["height"].shape, (281, 201))
        self.assertEqual(self.images["terrain_render"].shape[:2], self.images["height"].shape)
        self.assertEqual(self.pixel((-35, -25), 0.25), (0, 0))
        self.assertEqual(self.pixel((35, 25), 0.25), (280, 200))

    def test_elevation_has_steeper_ascent_crest_descent_and_flat_goal(self):
        heights = self.images["height"].astype(float) * self.metadata["height_max_m"] / 255
        resolution = self.metadata["height_resolution_m"]
        self.assertGreaterEqual(float(heights.min()), 0)
        self.assertAlmostEqual(float(heights.max()), 2.4, delta=0.02)
        for name in ("entrance", "flat_to_fork", "rock_branch", "flat_to_wall"):
            values = [heights[self.pixel(point, resolution)]
                      for point in self.metadata["paths"][name]]
            with self.subTest(path=name):
                self.assertLess(max(values), 0.1)
        self.assertAlmostEqual(heights[self.pixel(self.metadata["waypoints"]["goal"], resolution)],
                               0.0, delta=0.05)
        self.assertLess(heights[self.pixel(self.metadata["waypoints"]["fork1"], resolution)], 0.1)
        self.assertTrue(np.any(heights > 1.2))

        path = np.asarray(self.metadata["paths"]["uphill"], dtype=float)
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        distances = np.arange(0.0, arc[-1], 0.25)
        profile = self.metadata["hill_profile"]
        ascent_end = profile["start_distance_m"] + profile["ascent_length_m"]
        crest_end = ascent_end + profile["crest_length_m"]
        descent_end = crest_end + profile["descent_length_m"]

        def sample_levels(along):
            points = np.column_stack([np.interp(along, arc, path[:, axis]) for axis in (0, 1)])
            return np.asarray([self.terrain_height_at_xy(point) for point in points])

        levels = sample_levels(distances)
        self.assertLess(float(levels[distances <= profile["start_distance_m"] - 0.25].max()), 0.1,
                        "Initial approach must stay flat")
        self.assertGreater(float(levels[(distances >= ascent_end + 0.25)
                                       & (distances <= crest_end - 0.25)].min()), 2.3,
                           "Two-meter crest is missing")
        self.assertLess(float(levels[distances >= arc[-1] - 2.0].max()), 0.1,
                        "Final goal approach must return to flat ground")
        self.assertLess(self.terrain_height_at_xy(self.metadata["waypoints"]["hill_join"]), 0.1)
        # Compare 1-m rise/run along the road, smoothing pixel quantization.
        starts = distances[distances + 1.0 <= arc[-1]]
        grades = np.degrees(np.arctan(sample_levels(starts + 1.0) - sample_levels(starts)))
        max_ascent = float(grades[starts <= ascent_end].max())
        self.assertGreaterEqual(max_ascent, 16.0)
        self.assertLessEqual(max_ascent, 24.0)
        self.assertLess(float(grades[(starts >= crest_end) & (starts <= descent_end)].min()), -10.0,
                        "There is no clear descent after the crest")

    def test_rock_gate_is_internal_and_branch_continues_to_main_hill(self):
        path = np.asarray(self.metadata["paths"]["rock_branch"], dtype=float)
        gate = np.asarray(self.metadata["waypoints"]["rocks"], dtype=float)
        index = int(np.argmin(np.linalg.norm(path - gate, axis=1)))
        np.testing.assert_allclose(path[index], gate, atol=0.0001)
        self.assertGreater(index, 0)
        self.assertLess(index, len(path) - 1)
        deltas = np.diff(path, axis=0)
        # Strict NE progress also excludes self-crossings and the old westward
        # loop after the rocks, independently of the road-mask appearance.
        self.assertTrue((deltas[:, 0] > 0).all(), "Rock branch turns back in world X")
        self.assertTrue((deltas[:, 1] > 0).all(), "Rock branch turns back in world Y")
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(deltas, axis=1))]
        self.assertGreaterEqual(float(arc[index] / arc[-1]), 0.30)
        self.assertLessEqual(float(arc[index] / arc[-1]), 0.70)
        remaining_length = float(arc[-1] - arc[index])
        self.assertGreaterEqual(remaining_length, 5.0)
        join = self.metadata["waypoints"]["hill_join"]
        np.testing.assert_allclose(path[-1], join, rtol=0, atol=0.00006)
        main = np.asarray(self.metadata["paths"]["uphill"], dtype=float)
        self.assertLess(float(np.linalg.norm(main - join, axis=1).min()), 0.12)
        resolution = self.metadata["mask_resolution_m"]
        for point in path[index:]:
            self.assertGreater(self.images["road_mask"][self.pixel(point, resolution)], 0,
                               "Dirt marking does not continue behind the rock gate")
        # The blocking row must use the road's tangent AT the internal gate,
        # not the new, different tangent at the final hill-join endpoint.
        tangent = path[index + 1] - path[index - 1]
        tangent /= np.linalg.norm(tangent)
        normal = np.array([-tangent[1], tangent[0]])
        centers = np.asarray([rock["center"] for rock in self.metadata["rocks"]])
        offsets = centers - gate
        self.assertEqual(len(centers), 4)
        np.testing.assert_allclose(offsets @ tangent, np.zeros(4), atol=0.03)
        np.testing.assert_allclose(np.sort(offsets @ normal), [-1.425, -0.475, 0.475, 1.425], atol=0.03)
        for name, distance in (("rocks_before", -1.8), ("rocks_after", 1.0)):
            np.testing.assert_allclose(self.metadata["blocked_probes"][name], gate + distance * tangent,
                                       atol=0.04)

    def test_rock_branch_merges_once_without_revisiting_old_hill_turn(self):
        branch = np.asarray(self.metadata["paths"]["rock_branch"], dtype=float)
        main = np.asarray(self.metadata["paths"]["uphill"], dtype=float)
        distances = np.asarray([distance_to_roads(point, {"uphill": main})
                                for point in branch])
        # The 3-m roads naturally share a merge fan before their tangent join.
        # It must be a single terminal run, not a crossing then separation.
        shared = np.flatnonzero(distances <= self.metadata["road_width_m"])
        self.assertGreater(len(shared), 0)
        np.testing.assert_array_equal(shared, np.arange(shared[0], len(branch)))
        gate = np.asarray(self.metadata["waypoints"]["rocks"], dtype=float)
        gate_index = int(np.argmin(np.linalg.norm(branch - gate, axis=1)))
        self.assertGreater(int(shared[0]), gate_index,
                           "The main hill road overlaps the rock gate approach")
        self.assertLess(float(distances[-1]), 0.001,
                        "Branch endpoint is not on the main-road polyline")
        self.assertGreater(float(np.linalg.norm(branch - [-3.0, 7.0], axis=1).min()),
                           self.metadata["road_width_m"] + 2.0,
                           "Branch still doubles back to the old hill turn")
        closest = int(np.argmin(np.linalg.norm(main - branch[-1], axis=1)))
        self.assertTrue(0 < closest < len(main) - 1)
        branch_tangent = branch[-1] - branch[-2]
        main_tangent = main[closest + 1] - main[closest - 1]
        cosine = float(np.dot(branch_tangent, main_tangent)
                       / (np.linalg.norm(branch_tangent) * np.linalg.norm(main_tangent)))
        self.assertGreater(cosine, 0.995, "Roads meet as a crossing, not a tangent merge")

    def test_tree_and_grass_visual_footprints_do_not_overlap_dirt_roads(self):
        half_width = self.metadata["road_width_m"] / 2.0
        # Bounds of the procedural pine foliage and grass assets; collision
        # footprints are smaller and checked separately against XML/masks.
        for collection, model_radius in (("trees", 0.9), ("grass", 0.3)):
            for index, item in enumerate(self.metadata[collection]):
                with self.subTest(collection=collection, index=index):
                    distance = distance_to_roads(item["center"], self.metadata["paths"])
                    radius = model_radius * item["scale"]
                    self.assertGreater(distance, half_width + radius + 0.05,
                                       "Vegetation visual footprint intersects the dirt road")

    def test_world_uses_local_assets_and_existing_sensor_contract(self):
        self.assertEqual(self.world.tag, "mvsim_world")
        skyboxes = self.world.findall("element[@class='skybox']")
        self.assertEqual(len(skyboxes), 1)
        self.assertEqual(skyboxes[0].findtext("textures"),
                         "../assets/skybox/SunSet/SunSet%s.jpg")
        terrain = self.world.find("element[@class='elevation_map']")
        self.assertIsNotNone(terrain)
        for tag, value in (("resolution", self.metadata["height_resolution_m"]),
                           ("corner_min_x", self.bounds["x_min"]),
                           ("corner_min_y", self.bounds["y_min"]),
                           ("elevation_image_min_z", 0),
                           ("elevation_image_max_z", self.metadata["height_max_m"])):
            self.assertAlmostEqual(float(terrain.findtext(tag)), value)
        for tag, filename in (("elevation_image", "height.png"),
                              ("texture_image", "terrain_render.png")):
            uri = terrain.findtext(tag)
            expected = "../assets/forest/" + filename
            self.assertEqual(uri, expected)
            self.assertTrue((self.package / "worlds" / uri).resolve().is_file(), uri)
        self.assertEqual(self.world.find("include").attrib,
                         {"file": "${UPSTREAM}/definitions/jackal.vehicle.xml",
                          "default_sensors": "false"})
        vehicle = self.world.find("vehicle[@name='r1']")
        self.assertEqual(vehicle.attrib["class"], "jackal")
        np.testing.assert_allclose(flat_pose(vehicle)[:2], self.metadata["waypoints"]["start"])
        includes = vehicle.findall("include")
        self.assertEqual([node.attrib["file"] for node in includes],
                         ["../sensors/oak_rgbd.sensor.xml", "../sensors/oak_mono.sensor.xml",
                          "../sensors/oak_mono.sensor.xml", "../sensors/lidar2d.sensor.xml"])
        for index, side in ((1, "left"), (2, "right")):
            self.assertEqual(includes[index].attrib["frame_name"], f"oak_{side}_optical_frame")
            self.assertEqual(includes[index].attrib["sensor_y"], f"$env{{NOMAD_{side.upper()}_Y}}")
        for node in self.world.findall(".//model_uri"):
            uri = node.text
            self.assertNotIn("://", uri)
            self.assertNotIn("${", uri)
            self.assertTrue(uri.startswith("../assets/models/"), uri)
            path = (self.package / "worlds" / uri).resolve()
            self.assertTrue(path.is_file(), uri)

    def test_xml_collision_shapes_match_metadata(self):
        metadata = self.metadata
        self.assertEqual(metadata["boundaries"], [])
        self.assertNotIn("boundary_shrub", self.classes)
        self.assertFalse(any(name.startswith("boundary_") for name in self.blocks))
        self.assertFalse(any(block.attrib.get("class") == "boundary_shrub"
                             for block in self.blocks.values()))
        self.assertFalse(any("shrub" in node.text
                             for node in self.world.findall(".//model_uri")))
        for collection, prefix, class_name in (("trees", "tree", "pine"),
                                               ("grass", "grass", "grass"),
                                               ("rocks", "rock", "rock")):
            members = [name for name in self.blocks if name.startswith(prefix + "_")]
            self.assertEqual(len(members), len(metadata[collection]))
            self.assertEqual(self.classes[class_name].findtext("static"), "true")
            for index, item in enumerate(metadata[collection]):
                node = self.blocks[f"{prefix}_{index:04d}"]
                self.assertEqual(node.attrib["class"], class_name)
                pose = flat_pose(node)
                np.testing.assert_allclose(pose, [*item["center"], item["yaw_deg"]], atol=0.00006)
        wall = self.blocks["dead_end_wall"]
        self.assertEqual(wall.findtext("static"), "true")
        self.assertIsNone(wall.find("shape_from_visual"))
        self.assertIsNone(wall.find("geometry"))
        wall_shape = np.asarray(shape_vertices(wall))
        self.assertEqual(wall_shape.shape, (4, 2))
        self.assertTrue(np.isfinite(wall_shape).all())
        np.testing.assert_allclose(np.ptp(wall_shape, axis=0),
                                   [metadata["wall"]["length_m"], metadata["wall"]["width_m"]],
                                   atol=0.00011)
        self.assertEqual(float(wall.findtext("zmin")), 0.0)
        self.assertEqual(float(wall.findtext("zmax")), metadata["wall"]["height_m"])
        pose = flat_pose(wall)
        np.testing.assert_allclose(pose,
                                   [*metadata["wall"]["center"], metadata["wall"]["yaw_deg"]],
                                   atol=0.00006)
        # Explicit finite wall prism and local model-based vegetation/rocks are present.
        self.assertEqual(set(self.classes), {"pine", "grass", "rock"})

    def test_tree_collision_threshold_matches_independent_segment_distance(self):
        threshold = self.metadata["tree_collision_distance_m"]
        self.assertEqual(threshold, 5.0)
        trees = self.metadata["trees"]
        self.assertTrue(trees)
        enabled = 0
        for index, tree in enumerate(trees):
            name = f"tree_{index:04d}"
            with self.subTest(tree=name):
                distance = distance_to_roads(tree["center"], self.metadata["paths"])
                self.assertTrue(math.isfinite(distance))
                # Paths and recorded distances are rounded to four decimals.
                self.assertAlmostEqual(tree["road_distance_m"], distance, delta=0.00015)
                expected = distance <= threshold
                self.assertIs(tree["collision_enabled"], expected)
                enabled += int(expected)
                node = self.blocks[name]
                self.assertEqual(node.findtext("intangible"), "false" if expected else "true")
                self.assertEqual(is_intangible(node, self.classes), not expected)
                self.assertEqual(name in self.collision_polygons, expected)
                # Disabled trees still instantiate the same visual class.
                self.assertEqual(node.attrib["class"], "pine")
                self.assertIn(name, self.polygons)
        self.assertGreater(enabled, 0, "No tree retains road-near collision")
        self.assertGreater(len(trees) - enabled, 0, "No distant tree has collision disabled")
        self.assertEqual(enabled, sum(tree["collision_enabled"] for tree in trees))
        self.assertEqual(self.classes["pine"].findtext("visual/model_uri"),
                         "../assets/models/pine_tree.obj")

    def test_near_tree_centers_collide_and_far_tree_centers_do_not(self):
        resolution = self.metadata["mask_resolution_m"]
        blocked_sources = (("collision_mask", self.images["collision_mask"] > 0),
                           ("world_xml", self.xml_blocked))
        checked_clear_far = 0
        for index, tree in enumerate(self.metadata["trees"]):
            name = f"tree_{index:04d}"
            cell = self.pixel(tree["center"], resolution)
            with self.subTest(tree=name):
                if tree["collision_enabled"]:
                    for source, blocked in blocked_sources:
                        self.assertTrue(blocked[cell], (source, name, "near tree must collide"))
                    continue
                # Other physical grass/rocks may occupy a disabled tree's
                # sampled cell; distinguish their exact raster footprint.
                nearby_other = any(
                    self.polygon_occupies_cell(vertices, cell)
                    for other_name, vertices in self.collision_polygons.items()
                    if other_name != name
                )
                if not nearby_other:
                    checked_clear_far += 1
                    for source, blocked in blocked_sources:
                        self.assertFalse(blocked[cell], (source, name, "far tree still collides"))
        far_count = sum(not tree["collision_enabled"] for tree in self.metadata["trees"])
        self.assertGreaterEqual(checked_clear_far, max(1, math.ceil(0.8 * far_count)),
                                "Too many far-tree centers were exempted as overlapping other objects")

    def test_static_blocks_have_explicit_z_matching_decoded_terrain(self):
        self.assertTrue(self.blocks, "Static-object checks must not be vacuous")
        for name, block in self.blocks.items():
            with self.subTest(block=name):
                self.assertIsNotNone(block.find("init_pose3d"))
                self.assertEqual(block.findtext("skip_elevation_adjust"), "true")
                x, y, z, _, pitch, roll = pose_values(block)
                self.assertEqual((pitch, roll), (0.0, 0.0))
                expected_z = self.terrain_height_at_xy((x, y))
                self.assertAlmostEqual(z, expected_z, delta=0.03)

    def test_actual_world_and_mask_allow_open_forest_and_side_bypasses(self):
        resolution = self.metadata["mask_resolution_m"]
        # A known open point outside the visual dirt road, not a planner's route.
        forest_cell = self.pixel((-34.0, -24.0), resolution)
        self.assertEqual(self.images["road_mask"][forest_cell], 0)
        for source, blocked in (("collision_mask", self.images["collision_mask"] > 0),
                                ("world_xml", self.xml_blocked)):
            with self.subTest(source=source):
                reached = flood_fill(blocked, self.pixel(self.metadata["waypoints"]["start"], resolution))
                for name in ("start", "fork1", "fork2", "goal"):
                    self.assertTrue(reached[self.pixel(self.metadata["waypoints"][name], resolution)], name)
                for name in ("wall_before", "rocks_before"):
                    self.assertTrue(reached[self.pixel(self.metadata["blocked_probes"][name], resolution)], name)
                for name in ("wall_after", "rocks_after"):
                    cell = self.pixel(self.metadata["blocked_probes"][name], resolution)
                    self.assertFalse(blocked[cell], name + " should be a probe past the obstacle")
                    self.assertTrue(reached[cell], name + " must be physically accessible around the sides")
                self.assertFalse(blocked[forest_cell], "Open-forest probe is occupied")
                self.assertTrue(reached[forest_cell], "A visual road must not fence off the forest")
                for edge in (reached[0, :], reached[-1, :], reached[:, 0], reached[:, -1]):
                    self.assertTrue(edge.any(), "Removed road boundaries still prevent reaching a map edge")
                # Bypass freedom must not remove the actual straight-ahead obstacle.
                for name in ("wall", "rocks"):
                    cell = self.pixel(self.metadata["waypoints"][name], resolution)
                    self.assertTrue(blocked[cell], name + " direct approach no longer collides")

    def test_centerlines_free_except_actual_rock_and_wall_footprints(self):
        resolution = self.metadata["mask_resolution_m"]
        gates = {name: polygon for name, polygon in self.collision_polygons.items()
                 if name == "dead_end_wall" or name.startswith("rock_")}
        other_obstacles = {name: polygon for name, polygon in self.collision_polygons.items()
                           if name not in gates}
        for name, points in self.metadata["paths"].items():
            for point in points:
                cell = self.pixel(point, resolution)
                self.assertGreater(self.images["road_mask"][cell], 0, (name, point))
                # Use the exact XML polygon raster, not enlarged geometry or
                # offset probes: Pillow can include a boundary cell even when
                # the sampled point and all nine nearby probes lie outside it.
                gate_occupies_cell = any(self.polygon_occupies_cell(polygon, cell)
                                        for polygon in gates.values())
                if gate_occupies_cell:
                    occupants = [obstacle for obstacle, polygon in other_obstacles.items()
                                 if self.polygon_occupies_cell(polygon, cell)]
                    self.assertEqual(occupants, [],
                                     (name, point, "Non-gate obstacle shares a gate cell", occupants))
                else:
                    self.assertFalse(self.images["collision_mask"][cell], (name, point, "mask"))
                    self.assertFalse(self.xml_blocked[cell], (name, point, "XML"))


if __name__ == "__main__":
    unittest.main()
