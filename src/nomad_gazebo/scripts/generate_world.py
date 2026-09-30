#!/usr/bin/env python3
"""Export authored NOMAD XY/Z/soil data to a local Gazebo Harmonic SDF.

This changes the engine, not the route planner. Assets are offline and portable.
Far vegetation is batched into material groups instead of many physics bodies.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image


def sub(parent, tag, value=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if value is not None:
        node.text = str(value)
    return node


def save_xml(root, path):
    ET.indent(root, space='  ')
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def level(point, heights, data):
    row = (point[0]-data['bounds']['x_min'])/data['height_resolution_m']
    column = (point[1]-data['bounds']['y_min'])/data['height_resolution_m']
    i = min(heights.shape[0]-2, max(0, math.floor(row)))
    j = min(heights.shape[1]-2, max(0, math.floor(column)))
    a, b = row-i, column-j
    return float((1-a)*(1-b)*heights[i, j]+a*(1-b)*heights[i+1, j]
                 +(1-a)*b*heights[i, j+1]+a*b*heights[i+1, j+1])


def terrain_mesh(folder, heights, data):
    rows, cols = heights.shape
    resolution, bounds = data['height_resolution_m'], data['bounds']
    dx, dy = np.gradient(heights, resolution)
    normals = np.stack((-dx, -dy, np.ones_like(heights)), axis=-1)
    normals /= np.linalg.norm(normals, axis=-1)[..., None]
    with (folder/'terrain.obj').open('w', encoding='utf-8') as stream:
        stream.write('# NOMAD: rows=+X, columns=+Y, metres; exact authored height grid.\n'
                     'mtllib terrain.mtl\no terrain\ns 1\n')
        for i in range(rows):
            for j in range(cols):
                stream.write(f'v {bounds["x_min"]+i*resolution:.6f} '
                             f'{bounds["y_min"]+j*resolution:.6f} {heights[i,j]:.8f}\n')
        for i in range(rows):
            for j in range(cols):
                stream.write(f'vt {i/(rows-1):.8f} {j/(cols-1):.8f}\n')
        for row in normals:
            for vector in row:
                stream.write('vn '+' '.join(f'{v:.8f}' for v in vector)+'\n')
        stream.write('usemtl dirt_terrain\n')
        for i in range(rows-1):
            for j in range(cols-1):
                a = i*cols+j+1
                for triangle in ((a, a+cols, a+cols+1), (a, a+cols+1, a+1)):
                    stream.write('f '+' '.join(f'{k}/{k}/{k}' for k in triangle)+'\n')
    (folder/'terrain.mtl').write_text(
        'newmtl dirt_terrain\nKa 0.6 0.6 0.6\nKd 1 1 1\nKs 0 0 0\n'
        'd 1\nillum 1\nmap_Kd terrain_texture.png\n', encoding='utf-8')


def warm_forest_floor(source):
    """Paper-inspired dry soil palette; retain the authored road mask exactly."""
    source = np.asarray(source, dtype=np.float32)
    # The source road has R-G ~= +34, forest floor R-G ~= -18.
    road = np.clip((source[..., 0] - source[..., 1] + 18.0) / 52.0, 0.0, 1.0)
    forest = np.array([165.0, 151.0, 112.0])
    dirt = np.array([130.0, 99.0, 67.0])
    variation = (source[..., 0] - (57.0 + 76.0 * road)) * 0.65
    return np.clip((1.0 - road[..., None]) * forest + road[..., None] * dirt +
                   variation[..., None], 0, 255).astype(np.uint8)


def batch_mesh(source, destination, instances, heights, data):
    """Bake transforms; use one OBJ group per material for low draw-call count."""
    vertices, normals, faces = [], [], defaultdict(list)
    material = None
    for line in source.read_text(encoding='utf-8').splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append([float(v) for v in fields[1:4]])
        elif fields[0] == 'vn':
            normals.append([float(v) for v in fields[1:4]])
        elif fields[0] == 'usemtl':
            material = fields[1]
        elif fields[0] == 'f':
            faces[material].append([tuple(int(k) if k else 0 for k in token.split('/'))
                                    for token in fields[1:]])
    v0, n0 = np.asarray(vertices), np.asarray(normals)
    output_vertices, output_normals = [], []
    for instance in instances:
        angle = math.radians(instance['yaw_deg'])
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        origin = [*instance['center'], level(instance['center'], heights, data)]
        output_vertices.extend(v0@rotation.T*instance.get('scale', 1.0)+origin)
        output_normals.extend(n0@rotation.T)
    with destination.open('w', encoding='utf-8') as stream:
        stream.write(f'# Batched NOMAD visual-only instances: {len(instances)}\n'
                     f'mtllib ../models/{source.stem}.mtl\no {destination.stem}\ns off\n')
        for vector in output_vertices:
            stream.write('v '+' '.join(f'{v:.8f}' for v in vector)+'\n')
        for vector in output_normals:
            stream.write('vn '+' '.join(f'{v:.8f}' for v in vector)+'\n')
        for material, polygons in faces.items():
            stream.write(f'usemtl {material}\n')
            for instance_index in range(len(instances)):
                vo, no = instance_index*len(v0), instance_index*len(n0)
                for polygon in polygons:
                    stream.write('f '+' '.join(f'{token[0]+vo}//{token[-1]+no}'
                                               for token in polygon)+'\n')


def mesh(parent, uri, scale=None):
    node = sub(sub(parent, 'geometry'), 'mesh')
    sub(node, 'uri', uri)
    if scale is not None:
        sub(node, 'scale', ' '.join([str(scale)]*3))


def color(parent, value):
    node = sub(parent, 'material')
    sub(node, 'ambient', value)
    sub(node, 'diffuse', value)
    sub(node, 'specular', '0 0 0 1')


def static_model(world, name, x=0, y=0, z=0, yaw=0):
    model = sub(world, 'model', name=name)
    sub(model, 'static', 'true')
    sub(model, 'pose', f'{x} {y} {z} 0 0 {yaw}')
    return sub(model, 'link', name='link')


def box(parent, size, pose=None):
    if pose:
        sub(parent, 'pose', pose)
    sub(sub(sub(parent, 'geometry'), 'box'), 'size', size)


def world_file(package, data, heights, vegetation=True):
    root = ET.Element('sdf', version='1.9')
    world = sub(root, 'world', name='nomad_forest')
    sub(world, 'gravity', '0 0 -9.81')
    physics = sub(world, 'physics', name='nomad_physics', type='ignored')
    sub(physics, 'max_step_size', 1/600)
    sub(physics, 'real_time_factor', 1)
    for name in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors'):
        filename = dict(Physics='physics', UserCommands='user-commands',
                        SceneBroadcaster='scene-broadcaster', Sensors='sensors')[name]
        node = sub(world, 'plugin', filename=f'gz-sim-{filename}-system',
                   name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            sub(node, 'render_engine', 'ogre2')
    sub(world, 'plugin', filename='gz-sim-imu-system', name='gz::sim::systems::Imu')
    scene = sub(world, 'scene')
    sub(scene, 'ambient', '0.6 0.6 0.6 1')
    sub(scene, 'background', '0.62 0.75 0.86 1')
    sub(scene, 'shadows', 'false')
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 20 0 0 0')
    sub(light, 'diffuse', '0.8 0.8 0.8 1')
    sub(light, 'specular', '0.1 0.1 0.1 1')
    sub(light, 'direction', '-0.4 -0.2 -1')
    sub(light, 'cast_shadows', 'false')
    gui = sub(world, 'gui', fullscreen='false')
    view = sub(gui, 'plugin', filename='MinimalScene', name='3D View')
    properties = sub(view, 'gz-gui')
    sub(properties, 'title', '3D View')
    sub(properties, 'property', 'false', type='bool', key='showTitleBar')
    sub(properties, 'property', 'docked', type='string', key='state')
    sub(view, 'engine', 'ogre2')
    sub(view, 'scene', 'scene')
    sub(view, 'ambient_light', '0.6 0.6 0.6')
    sub(view, 'background_color', '0.62 0.75 0.86')
    sub(view, 'camera_pose', '-43 -38 48 0 0.70 0.72')
    for name in ('GzSceneManager', 'InteractiveViewControl', 'CameraTracking',
                 'EntityContextMenuPlugin', 'SelectEntities', 'WorldControl',
                 'WorldStats', 'EntityTree', 'ComponentInspector'):
        node = sub(gui, 'plugin', filename=name, name=name)
        props = sub(node, 'gz-gui')
        if name not in ('EntityTree', 'ComponentInspector'):
            sub(props, 'property', 'floating', type='string', key='state')
            sub(props, 'property', 'false', type='bool', key='showTitleBar')
            sub(props, 'property', 'false', type='bool', key='resizable')
            if name not in ('WorldControl', 'WorldStats'):
                for dimension in ('height', 'width'):
                    sub(props, 'property', '5', type='double', key=dimension)
        if name == 'WorldControl':
            sub(node, 'play_pause', 'true'); sub(node, 'step', 'true')
            sub(node, 'start_paused', 'false')
        if name in ('WorldControl', 'WorldStats'):
            sub(props, 'property', '72' if name == 'WorldControl' else '110', type='double', key='height')
            sub(props, 'property', '180' if name == 'WorldControl' else '290', type='double', key='width')
            sub(props, 'property', '1', type='double', key='z')
            anchor = sub(props, 'anchors', target='3D View')
            side = 'left' if name == 'WorldControl' else 'right'
            sub(anchor, 'line', own=side, target=side)
            sub(anchor, 'line', own='bottom', target='bottom')
        if name == 'WorldStats':
            for tag in ('sim_time', 'real_time', 'real_time_factor', 'iterations'):
                sub(node, tag, 'true')
    link = static_model(world, 'terrain')
    mesh(sub(link, 'visual', name='dirt_terrain'), '../assets/geometry/terrain.obj')
    mesh(sub(link, 'collision', name='terrain_collision'), '../assets/geometry/terrain.obj')
    if vegetation:
        for name, uri in (('forest_far', 'forest_far.obj'), ('forest_grass', 'grass.obj')):
            link = static_model(world, name)
            mesh(sub(link, 'visual', name='visual_only'), '../assets/geometry/'+uri)
        for index, tree in enumerate(data['trees']):
            if not tree['collision_enabled']:
                continue
            x, y = tree['center']
            link = static_model(world, f'tree_{index:04d}', x, y, level((x,y), heights, data),
                                math.radians(tree['yaw_deg']))
            mesh(sub(link, 'visual', name='pine'), '../assets/models/pine_tree.obj', tree['scale'])
            height = 3.35*tree['scale']
            box(sub(link, 'collision', name='trunk'), f'0.44 0.44 {height}', f'0 0 {height/2} 0 0 0')
    for index, rock in enumerate(data['rocks']):
        x, y = rock['center']
        link = static_model(world, f'rock_{index:04d}', x, y, level((x,y), heights, data),
                            math.radians(rock['yaw_deg']))
        mesh(sub(link, 'visual', name='rock'), '../assets/models/rock.obj')
        collision = sub(link, 'collision', name='blocking_rock')
        sub(collision, 'pose', f'0 0 {rock["height_m"]/2} 0 0 0')
        cylinder = sub(sub(collision, 'geometry'), 'cylinder')
        sub(cylinder, 'radius', rock['radius_m']); sub(cylinder, 'length', rock['height_m'])
    wall = data['wall']; x, y = wall['center']
    link = static_model(world, 'dead_end_wall', x, y, level((x,y), heights, data), math.radians(wall['yaw_deg']))
    size = f'{wall["length_m"]} {wall["width_m"]} {wall["height_m"]}'
    pose = f'0 0 {wall["height_m"]/2} 0 0 0'
    visual = sub(link, 'visual', name='wall'); box(visual, size, pose); color(visual, '0.5 0.48 0.43 1')
    box(sub(link, 'collision', name='wall_collision'), size, pose)
    include = sub(world, 'include')
    sub(include, 'uri', 'model://nomad_vehicle'); sub(include, 'name', 'nomad_vehicle')
    start, next_point = data['paths']['entrance'][:2]
    yaw = math.atan2(next_point[1]-start[1], next_point[0]-start[0])
    sub(include, 'pose', f'{start[0]} {start[1]} 0.03 0 0 {yaw}')
    filename = 'forest.sdf' if vegetation else 'forest_bare.sdf'
    save_xml(root, package/'worlds'/filename)


def generate(package, map_package=None):
    author, geometry, models = [package/'assets'/name for name in ('authoring', 'geometry', 'models')]
    for path in (author, geometry, models):
        path.mkdir(parents=True, exist_ok=True)
    if map_package is not None:
        source = map_package/'assets/forest'
        for filename in ('layout.json', 'height.png', 'terrain.png', 'forest_overview.png'):
            shutil.copy2(source/filename, author/filename)
        for filename in ('pine_tree.obj', 'pine_tree.mtl', 'grass_clump.obj', 'grass_clump.mtl', 'rock.obj', 'rock.mtl'):
            shutil.copy2(map_package/'assets/models'/filename, models/filename)
    data = json.loads((author/'layout.json').read_text(encoding='utf-8'))
    with Image.open(author/'height.png') as image:
        heights = np.asarray(image, dtype=float)*data['height_max_m']/255
    with Image.open(author/'terrain.png') as image:
        north_up = np.flipud(np.transpose(warm_forest_floor(image), (1, 0, 2)))
        Image.fromarray(north_up).save(geometry/'terrain_texture.png')
    terrain_mesh(geometry, heights, data)
    far = [tree for tree in data['trees'] if not tree['collision_enabled']]
    batch_mesh(models/'pine_tree.obj', geometry/'forest_far.obj', far, heights, data)
    batch_mesh(models/'grass_clump.obj', geometry/'grass.obj', data['grass'], heights, data)
    world_file(package, data, heights)
    # Keep the authored forest available for an exact, reversible A/B check.
    # Only vegetation models differ; terrain, obstacles, vehicle and sensors do not.
    world_file(package, data, heights, vegetation=False)
    report = dict(engine='Gazebo Harmonic', world='nomad_forest',
                  vegetation_enabled=True,
                  height_png_sha256=hashlib.sha256((author/'height.png').read_bytes()).hexdigest(),
                  mesh_grid=list(heights.shape), vertex_count=int(heights.size),
                  triangle_count=int(2*(heights.shape[0]-1)*(heights.shape[1]-1)),
                  bounds=data['bounds'], source_axes='PNG rows +X / columns +Y',
                  texture_axes='north-up; OBJ u=+X, v=+Y',
                  rendered_trees=len(data['trees']), collision_trees=len(data['trees'])-len(far),
                  visual_only_trees=len(far), visual_only_grass=len(data['grass']),
                  rocks=len(data['rocks']), no_roadside_boundary=True,
                  limitations=['No automatic traversability, branch memory or route planning.',
                               'Vehicle mass and geometry are provisional, not measured hardware.',
                               'Motor effort limits and payload climbing failure are not verified.'])
    (package/'assets/export.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    bare_report = dict(report, vegetation_enabled=False, rendered_trees=0,
                       collision_trees=0, visual_only_trees=0, visual_only_grass=0)
    (package/'assets/export_bare.json').write_text(json.dumps(bare_report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--map-package', type=Path, help='Initial import; otherwise use saved authoring snapshot')
    args = parser.parse_args()
    generate(args.output_dir.resolve(), args.map_package.resolve() if args.map_package else None)
