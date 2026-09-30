#!/usr/bin/env python3
"""Build the reproducible NOMAD fork/backtracking world (not a planner).

All generated assets are local. MVSim elevation/texture arrays use rows=+X,
columns=+Y, unlike a conventional north-up image. No upstream files are edited.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

from forest_models import generate_models


BOUNDS = dict(x_min=-35.0, x_max=35.0, y_min=-25.0, y_max=25.0)
HEIGHT_RESOLUTION = 0.25
MASK_RESOLUTION = 0.10
TEXTURE_RESOLUTION = 0.05
ROAD_WIDTH = 3.0
TREE_COLLISION_DISTANCE_M = 5.0  # tree center to the nearest road centerline
TREE_OUTER_EDGE_BAND_M = 5.0  # distance from the nearest rectangular map edge
TREE_OUTER_THINNING_MODULO = 2
TREE_OUTER_THINNING_REMAINDER = 0  # remove even ORIGINAL generated indices only
HEIGHT_MAX = 2.4
HILL_PROFILE = dict(start_distance_m=2.5, ascent_length_m=10.0,
                    crest_length_m=2.0, descent_length_m=12.0,
                    height_m=HEIGHT_MAX)
SEED = 42
HILL_TURN = (-3.0, 7.0)
WAYPOINTS = dict(start=(-30.0, -18.0), fork1=(-20.0, -12.0),
                 fork2=(-3.0, -8.0), rocks=(3.0, 1.0),
                 # Point at t=0.45 on the unchanged upper uphill Bezier.
                 hill_join=(7.945125, 9.527875),
                 wall=(21.0, 10.0), goal=(23.0, 16.0))


def bezier(control):
    points = np.asarray(control, dtype=float)
    # Initial dense sampling, then approximately uniform arc-length samples.
    t = np.linspace(0, 1, 401)[:, None]
    dense = ((1-t)**3*points[0] + 3*(1-t)**2*t*points[1]
             + 3*(1-t)*t*t*points[2] + t**3*points[3])
    distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    positions = np.linspace(0, distance[-1], math.ceil(distance[-1]/0.10)+1)
    return np.column_stack([np.interp(positions, distance, dense[:, axis]) for axis in (0, 1)])


def route_paths():
    p = WAYPOINTS
    def join(*segments):
        return np.concatenate([bezier(segment)[int(i > 0):] for i, segment in enumerate(segments)])
    return dict(
        entrance=join([p['start'], (-27, -21), (-25, -12), p['fork1']]),
        uphill=join([p['fork1'], (-20, 2), (-10, 11), HILL_TURN],
                    [HILL_TURN, (9, 3), (8, 17), p['goal']]),
        flat_to_fork=join([p['fork1'], (-15, -12), (-10, -17), p['fork2']]),
        # Continuous forward road; the rocks sit near its midpoint. Neither
        # segment turns back toward the fork after passing the rock gate.
        rock_branch=join([p['fork2'], (-1, -5), (1, -2), p['rocks']],
                         [p['rocks'], (5, 4), (6.093375, 7.872625), p['hill_join']]),
        flat_to_wall=join([p['fork2'], (3, -10), (8, -18), (12, -11)],
                         [(12, -11), (24, -4), (14, 2), p['wall']]),
    )


def dimensions(resolution):
    return (round((BOUNDS['x_max']-BOUNDS['x_min'])/resolution)+1,
            round((BOUNDS['y_max']-BOUNDS['y_min'])/resolution)+1)


def pixel(point, resolution):
    # Pillow x=column=world Y, Pillow y=row=world X.
    return ((point[1]-BOUNDS['y_min'])/resolution,
            (point[0]-BOUNDS['x_min'])/resolution)


def road_mask(paths, resolution, width):
    rows, columns = dimensions(resolution)
    image = Image.new('L', (columns, rows))
    draw = ImageDraw.Draw(image)
    thickness = round(width/resolution)
    radius = width/(2*resolution)
    for path in paths.values():
        pts = [pixel(point, resolution) for point in path]
        draw.line(pts, fill=255, width=thickness, joint='curve')
        for px, py in (pts[0], pts[-1]):
            draw.ellipse((px-radius, py-radius, px+radius, py+radius), fill=255)
    return np.array(image)


def nearest(points, samples):
    distances = np.full(points.shape[:-1], np.inf)
    indices = np.zeros(points.shape[:-1], dtype=int)
    for index, sample in enumerate(samples):
        trial = np.sum((points-sample)**2, axis=-1)
        update = trial < distances
        indices[update] = index
        distances[update] = trial[update]
    return np.sqrt(distances), indices


def nearest_polyline(points, path):
    """Distance and continuous arc position of the closest segment projection.

    Elevation must not snap to a nearest sampled vertex: that makes artificial
    stair steps even when the desired longitudinal hill profile is smooth.
    """
    distances2 = np.full(points.shape[:-1], np.inf)
    along = np.zeros(points.shape[:-1])
    arc = 0.0
    for start, end in zip(path[:-1], path[1:]):
        delta = end-start
        length2 = float(np.dot(delta, delta))
        if length2 == 0:
            continue
        length = math.sqrt(length2)
        t = np.clip(np.sum((points-start)*delta, axis=-1)/length2, 0.0, 1.0)
        trial = np.sum((points-(start+t[..., None]*delta))**2, axis=-1)
        update = trial < distances2
        along[update] = arc+t[update]*length
        distances2[update] = trial[update]
        arc += length
    return np.sqrt(distances2), along


def smoothstep(value):
    value = np.clip(value, 0.0, 1.0)
    return value*value*(3.0-2.0*value)


def terrain_height(points, paths):
    hill = paths['uphill']
    distance, arc = nearest_polyline(points, hill)
    p = HILL_PROFILE
    fall_start = p['start_distance_m']+p['ascent_length_m']+p['crest_length_m']
    rise = (p['height_m']*smoothstep((arc-p['start_distance_m'])/p['ascent_length_m'])
            * (1-smoothstep((arc-fall_start)/p['descent_length_m'])))
    shoulder = 1.0-smoothstep((distance-2.3)/2.0)
    elevation = rise*shoulder
    # The alternative branch really stays flat all the way to its wall.
    for name in ('entrance', 'flat_to_fork', 'rock_branch', 'flat_to_wall'):
        flat_distance, _ = nearest(points, paths[name][::3])
        elevation = np.where(flat_distance < 1.9, 0.0, elevation)
    return elevation


def rectangle(center, yaw, length, width):
    local = np.array([[-length/2, -width/2], [length/2, -width/2],
                      [length/2, width/2], [-length/2, width/2]])
    angle = math.radians(yaw)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    return local @ rotation.T + center


def obstacle_definitions(paths):
    direction = paths['flat_to_wall'][-1]-paths['flat_to_wall'][-2]
    direction /= np.linalg.norm(direction)
    wall = dict(center=list(WAYPOINTS['wall']), yaw_deg=math.degrees(math.atan2(direction[1], direction[0])),
                length_m=0.65, width_m=4.80, height_m=2.2)
    # The rocks are an internal gate, not the artificial end of this road.
    rock_path = paths['rock_branch']
    gate_index = int(np.argmin(np.linalg.norm(rock_path-WAYPOINTS['rocks'], axis=1)))
    if not 0 < gate_index < len(rock_path)-1:
        raise ValueError('Rock gate must have a real road on both sides')
    direction_rock = rock_path[gate_index+1]-rock_path[gate_index-1]
    direction_rock /= np.linalg.norm(direction_rock)
    normal = np.array([-direction_rock[1], direction_rock[0]])
    rocks = [dict(center=(np.asarray(WAYPOINTS['rocks'])+normal*(i-1.5)*0.95).tolist(),
                  yaw_deg=17*i, radius_m=0.65, height_m=1.1) for i in range(4)]
    probes = dict(
        wall_before=(np.asarray(wall['center'])-1.8*direction).tolist(),
        wall_after=(np.asarray(wall['center'])+1.0*direction).tolist(),
        rocks_before=(np.asarray(WAYPOINTS['rocks'])-1.8*direction_rock).tolist(),
        rocks_after=(np.asarray(WAYPOINTS['rocks'])+1.0*direction_rock).tolist())
    return wall, rocks, probes


def collision_mask(wall, rocks, trees):
    rows, columns = dimensions(MASK_RESOLUTION)
    image = Image.new('L', (columns, rows))
    draw = ImageDraw.Draw(image)
    for block in [wall]:
        vertices = rectangle(block['center'], block['yaw_deg'], block['length_m'], block['width_m'])
        draw.polygon([pixel(p, MASK_RESOLUTION) for p in vertices], fill=255)
    for rock in rocks:
        angles = np.arange(8)*math.pi/4+math.radians(rock['yaw_deg'])
        vertices = np.column_stack((np.cos(angles), np.sin(angles)))*rock['radius_m']+rock['center']
        draw.polygon([pixel(p, MASK_RESOLUTION) for p in vertices], fill=255)
    for tree in trees:
        if not tree['collision_enabled']:
            continue
        px, py = pixel(tree['center'], MASK_RESOLUTION)
        radius = 0.22/MASK_RESOLUTION
        draw.rectangle((px-radius, py-radius, px+radius, py+radius), fill=255)
    return np.asarray(image)


def vegetation(paths):
    rng = np.random.default_rng(SEED)
    samples = np.concatenate(list(paths.values()))[::2]
    segment_starts = np.concatenate([path[:-1] for path in paths.values()])
    segment_deltas = np.concatenate([np.diff(path, axis=0) for path in paths.values()])
    segment_lengths2 = np.sum(segment_deltas*segment_deltas, axis=1)
    candidates = []
    for x in np.arange(-32.5, 33.0, 3.4):
        for y in np.arange(-22.5, 23.0, 3.4):
            point = np.array([x, y])+rng.uniform(-0.50, 0.50, 2)
            if np.min(np.linalg.norm(samples-point, axis=1)) > 3.9:
                # Create a Box2D body only near roads. Visual thinning happens
                # after ALL tree/grass RNG draws so retained objects never move.
                # Measure line-segment distance, not just sampled vertices.
                t = np.clip(np.sum((point-segment_starts)*segment_deltas, axis=1)
                            / segment_lengths2, 0, 1)
                closest = segment_starts+t[:, None]*segment_deltas
                road_distance = float(np.min(np.linalg.norm(point-closest, axis=1)))
                candidates.append(dict(center=point.tolist(), yaw_deg=float(rng.uniform(-180, 180)),
                                       scale=float(rng.uniform(0.8, 1.25)),
                                       road_distance_m=round(road_distance, 4),
                                       collision_enabled=road_distance <= TREE_COLLISION_DISTANCE_M))
    grass = []
    for _ in range(220):
        point = rng.uniform([-33, -23], [33, 23])
        if np.min(np.linalg.norm(samples-point, axis=1)) > 2.8:
            grass.append(dict(center=point.tolist(), yaw_deg=float(rng.uniform(-180, 180)),
                              scale=float(rng.uniform(0.7, 1.3))))
    kept, removed_indices = [], []
    for original_index, tree in enumerate(candidates):
        x, y = tree['center']
        edge_distance = min(x-BOUNDS['x_min'], BOUNDS['x_max']-x,
                            y-BOUNDS['y_min'], BOUNDS['y_max']-y)
        remove = (not tree['collision_enabled']
                  and edge_distance < TREE_OUTER_EDGE_BAND_M
                  and original_index % TREE_OUTER_THINNING_MODULO
                  == TREE_OUTER_THINNING_REMAINDER)
        if remove:
            removed_indices.append(original_index)
        else:
            kept.append(tree)
    pruning = dict(original_tree_count=len(candidates),
                   removed_outer_tree_count=len(removed_indices),
                   tree_outer_edge_band_m=TREE_OUTER_EDGE_BAND_M,
                   tree_outer_thinning_modulo=TREE_OUTER_THINNING_MODULO,
                   tree_outer_thinning_remainder=TREE_OUTER_THINNING_REMAINDER,
                   tree_outer_pruning_policy='thinning_even_original_index; road-near trees always retained',
                   removed_outer_original_indices=removed_indices)
    return kept, grass, pruning


def texture(paths):
    mask = Image.fromarray(road_mask(paths, TEXTURE_RESOLUTION, ROAD_WIDTH))
    blend = np.asarray(mask.filter(ImageFilter.GaussianBlur(0.14/TEXTURE_RESOLUTION)), dtype=float)/255
    rng = np.random.default_rng(SEED)
    # Fine, deterministic soil/forest-floor variation, not a paved road.
    noise = rng.normal(0, 5.5, blend.shape)
    grain = rng.uniform(-3, 3, blend.shape)
    forest_color = np.array([57, 75, 35], dtype=float)
    dirt_color = np.array([133, 99, 62], dtype=float)
    pixels = ((1-blend[..., None])*forest_color+blend[..., None]*dirt_color
              + noise[..., None] + grain[..., None]*np.array([1.0, 0.8, 0.5]))
    return Image.fromarray(np.clip(pixels, 0, 255).astype(np.uint8))


def sub(parent, name, value=None):
    element = ET.SubElement(parent, name)
    if value is not None:
        element.text = str(value)
    return element


def footprint(parent, vertices):
    shape = sub(parent, 'shape')
    for point in vertices:
        sub(shape, 'pt', f'{point[0]:.4f} {point[1]:.4f}')


def surface_z(point, height_data):
    row = (point[0]-BOUNDS['x_min'])/HEIGHT_RESOLUTION
    column = (point[1]-BOUNDS['y_min'])/HEIGHT_RESOLUTION
    i = min(height_data.shape[0]-2, max(0, math.floor(row)))
    j = min(height_data.shape[1]-2, max(0, math.floor(column)))
    a, b = row-i, column-j
    return float((1-a)*(1-b)*height_data[i, j] + a*(1-b)*height_data[i+1, j]
                 + (1-a)*b*height_data[i, j+1] + a*b*height_data[i+1, j+1])


def write_world(package, paths, wall, rocks, trees, grass, height_data):
    root = ET.Element('mvsim_world', version='1.0')
    root.append(ET.Comment('Generated by generate_forest_map.py; route choice/backtracking is NOT implemented here.'))
    sub(root, 'simul_timestep', '0.0016666666666666668')
    ET.SubElement(root, 'variable', name='UPSTREAM', value='$env{NOMAD_MVSIM_SHARE}')
    gui = sub(root, 'gui')
    for key, value in dict(ortho='false', cam_distance=73, cam_point_to='0 0 0',
                           cam_azimuth=-90, cam_elevation=68, fov_deg=65,
                           show_forces='false', show_sensor_points='false', refresh_fps=20).items():
        sub(gui, key, value)
    sub(sub(root, 'lights'), 'enable_shadows', 'false')
    # SkyBox is a render background, not a physical box surrounding the map.
    # Keep all six licensed textures local so startup does not need a download.
    sky = ET.SubElement(root, 'element', attrib={'class': 'skybox'})
    sub(sky, 'textures', '../assets/skybox/SunSet/SunSet%s.jpg')
    terrain = sub(root, 'element'); terrain.set('class', 'elevation_map')
    # Root block classes do not inherit the include-only CURRENT_FILE variable.
    # MVSim resolves plain relative asset paths against the world file itself.
    asset = '../assets/forest/'
    for key, value in dict(resolution=HEIGHT_RESOLUTION, elevation_image=asset+'height.png',
                           elevation_image_min_z=0, elevation_image_max_z=HEIGHT_MAX,
                           texture_image=asset+'terrain_render.png', corner_min_x=BOUNDS['x_min'],
                           corner_min_y=BOUNDS['y_min'], texture_extension_x=0,
                           texture_extension_y=0, model_split_size=0).items():
        sub(terrain, key, value)
    ET.SubElement(root, 'include', file='${UPSTREAM}/definitions/jackal.vehicle.xml', default_sensors='false')
    vehicle = ET.SubElement(root, 'vehicle', name='r1', attrib={'class': 'jackal'})
    tangent = paths['entrance'][1]-paths['entrance'][0]
    yaw = math.degrees(math.atan2(tangent[1], tangent[0]))
    sub(vehicle, 'init_pose', f'{WAYPOINTS["start"][0]} {WAYPOINTS["start"][1]} {yaw:.3f}')
    ET.SubElement(vehicle, 'include', file='../sensors/oak_rgbd.sensor.xml')
    ET.SubElement(vehicle, 'include', file='../sensors/oak_mono.sensor.xml',
                  frame_name='oak_left_optical_frame', sensor_y='$env{NOMAD_LEFT_Y}')
    ET.SubElement(vehicle, 'include', file='../sensors/oak_mono.sensor.xml',
                  frame_name='oak_right_optical_frame', sensor_y='$env{NOMAD_RIGHT_Y}')
    ET.SubElement(vehicle, 'include', file='../sensors/lidar2d.sensor.xml')
    model_asset = '../assets/models/'
    for name, height in [('pine', 5.7), ('grass', 0.45), ('rock', 1.1)]:
        cls = ET.SubElement(root, 'block:class', name=name)
        sub(cls, 'static', 'true'); sub(cls, 'zmin', 0.0); sub(cls, 'zmax', height)
        visual = sub(cls, 'visual')
        filename = dict(pine='pine_tree', grass='grass_clump', rock='rock')[name]
        sub(visual, 'model_uri', model_asset+filename+'.obj')
        # Explicit footprints: foliage bounding boxes must not occupy the road.
        if name == 'pine':
            footprint(cls, rectangle([0, 0], 0, 0.44, 0.44))
        elif name == 'grass':
            footprint(cls, rectangle([0, 0], 0, 0.12, 0.12))
        elif name == 'rock':
            angles = np.arange(8)*math.pi/4
            footprint(cls, np.column_stack((np.cos(angles), np.sin(angles)))*0.65)
    for name, collection, cls in [('tree', trees, 'pine'), ('grass', grass, 'grass'), ('rock', rocks, 'rock')]:
        for index, block in enumerate(collection):
            node = ET.SubElement(root, 'block', name=f'{name}_{index:04d}', attrib={'class': cls})
            z = surface_z(block['center'], height_data)
            sub(node, 'init_pose3d', f'{block["center"][0]:.4f} {block["center"][1]:.4f} {z:.4f} {block["yaw_deg"]:.4f} 0 0')
            sub(node, 'skip_elevation_adjust', 'true')
            if name == 'tree':
                # In MVSim 1.4 this skips Box2D, but the OBJ stays in the
                # physical render scene (camera/RGBD/raytrace_3d LiDAR).
                sub(node, 'intangible', 'false' if block['collision_enabled'] else 'true')
            if 'scale' in block:
                sub(node, 'visual_scale', f'{block["scale"]:.4f}')
    node = ET.SubElement(root, 'block', name='dead_end_wall')
    sub(node, 'static', 'true'); sub(node, 'color', '#77716A')
    # MVSim 1.4's primitive box + shape_from_visual produced infinite Z
    # bounds on this MRPT build. An explicit finite prism is equivalent and
    # keeps the displayed wall, Box2D footprint and test data consistent.
    sub(node, 'zmin', 0.0); sub(node, 'zmax', wall['height_m'])
    footprint(node, rectangle([0, 0], 0, wall['length_m'], wall['width_m']))
    z = surface_z(wall['center'], height_data)
    sub(node, 'init_pose3d', f'{wall["center"][0]:.4f} {wall["center"][1]:.4f} {z:.4f} {wall["yaw_deg"]:.4f} 0 0')
    sub(node, 'skip_elevation_adjust', 'true')
    ET.indent(root, space='  ')
    destination = package/'worlds'/'forest.world.xml'
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=False)


def overview(image, paths, trees, wall, rocks, destination):
    # This is a north-up plan generated from the exact world data, not a GUI screenshot.
    source = np.asarray(image)
    image = Image.fromarray(np.flipud(np.transpose(source, (1, 0, 2)))).resize((1120, 800))
    draw = ImageDraw.Draw(image)
    def xy(point):
        return ((point[0]-BOUNDS['x_min'])/70*1120, (BOUNDS['y_max']-point[1])/50*800)
    for tree in trees:
        x, y = xy(tree['center']); radius = 14*tree['scale']
        draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill=(31, 65, 29))
        draw.ellipse((x-2, y-2, x+2, y+2), fill=(82, 56, 36))
    for rock in rocks:
        x, y = xy(rock['center'])
        draw.ellipse((x-10, y-10, x+10, y+10), fill=(145, 141, 129), outline=(55, 54, 47), width=2)
    draw.polygon([xy(p) for p in rectangle(wall['center'], wall['yaw_deg'], wall['length_m'], wall['width_m'])], fill=(186, 183, 168))
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 20)
    for name, position in WAYPOINTS.items():
        x, y = xy(position)
        draw.ellipse((x-4, y-4, x+4, y+4), fill=(242, 217, 142))
        text = dict(start='START', fork1='1st fork', fork2='2nd fork', rocks='ROCKS',
                    hill_join='HILL JOIN', wall='WALL', goal='GOAL')[name]
        draw.text((x+9, y-27), text, font=font, fill=(255, 241, 207), stroke_width=2, stroke_fill=(37, 47, 27))
    draw.text(xy((-29, 11)), 'Hill: 0 -> 2.4 -> 0 m', font=font, fill=(255, 241, 207), stroke_width=2, stroke_fill=(37, 47, 27))
    draw.text(xy((8, -21)), 'Flat dead-end route', font=font, fill=(255, 241, 207), stroke_width=2, stroke_fill=(37, 47, 27))
    image.save(destination)


def generate(package):
    paths = route_paths()
    assets = package/'assets'/'forest'
    assets.mkdir(parents=True, exist_ok=True)
    generate_models(package/'assets'/'models')
    rows, columns = dimensions(HEIGHT_RESOLUTION)
    x = BOUNDS['x_min']+np.arange(rows)*HEIGHT_RESOLUTION
    y = BOUNDS['y_min']+np.arange(columns)*HEIGHT_RESOLUTION
    grid = np.stack(np.meshgrid(x, y, indexing='ij'), axis=-1)
    heights = terrain_height(grid, paths)
    height_pixels = np.rint(255*heights/HEIGHT_MAX).astype(np.uint8)
    Image.fromarray(height_pixels).save(assets/'height.png')
    paint = texture(paths); paint.save(assets/'terrain.png')
    # MRPT CMesh color matrices require image/Z dimensions to match. Keep the
    # detailed authoring texture, but use this matching-size rendering asset.
    paint.resize((columns, rows), Image.Resampling.LANCZOS).save(assets/'terrain_render.png')
    # The dirt paint is an authoring mask, not a physical driving corridor.
    # Terrain outside it remains open: a future traversability/planner must
    # decide where to drive instead of relying on invisible roadside walls.
    corridor = road_mask(paths, MASK_RESOLUTION, ROAD_WIDTH)
    wall, rocks, probes = obstacle_definitions(paths)
    trees, grass, tree_pruning = vegetation(paths)
    blocked = collision_mask(wall, rocks, trees)
    Image.fromarray(corridor).save(assets/'road_mask.png')
    Image.fromarray(blocked).save(assets/'collision_mask.png')
    metadata = dict(bounds=BOUNDS, seed=SEED, resolution_m=HEIGHT_RESOLUTION,
                    height_resolution_m=HEIGHT_RESOLUTION, mask_resolution_m=MASK_RESOLUTION,
                    texture_resolution_m=TEXTURE_RESOLUTION, height_max_m=HEIGHT_MAX,
                    render_texture_resolution_m=HEIGHT_RESOLUTION,
                    hill_profile=HILL_PROFILE,
                    road_width_m=ROAD_WIDTH, corridor_width_m=ROAD_WIDTH,
                    tree_collision_distance_m=TREE_COLLISION_DISTANCE_M,
                    tree_collision_policy='Near road centerline: physical; far: rendered without Box2D body.',
                    road_mask_purpose='Terrain authoring only; not a physical driving restriction.',
                    axis_convention='rows=world +X; columns=world +Y; endpoints included',
                    waypoints=WAYPOINTS, paths={k:v.round(4).tolist() for k,v in paths.items()},
                    boundaries=[], trees=trees, grass=grass, wall=wall, rocks=rocks,
                    blocked_probes=probes,
                    scenario=['start', 'fork1', 'fork2', 'wall', 'fork2', 'fork1', 'goal'],
                    limitations=['Layout data is not automatically a navigation prior.',
                                 'No roadside boundary: off-road bypass is physically possible; traversability is not implemented.',
                                 'Backtracking/branch selection and torque limits are not implemented.',
                                 'Vehicle remains upstream Jackal with twist_ideal.'],
                    **tree_pruning)
    (assets/'layout.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='utf-8')
    write_world(package, paths, wall, rocks, trees, grass,
                height_pixels.astype(float)*HEIGHT_MAX/255.0)
    overview(paint, paths, trees, wall, rocks, assets/'forest_overview.png')
    print(json.dumps(dict(package=str(package), boundaries=0, trees=len(trees),
                          collision_trees=sum(tree['collision_enabled'] for tree in trees),
                          grass=len(grass), rocks=len(rocks), height_range_m=[float(heights.min()), float(heights.max())]), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True, help='nomad_sim package root (source or staging directory)')
    args = parser.parse_args()
    generate(args.output_dir.resolve())


if __name__ == '__main__':
    main()
