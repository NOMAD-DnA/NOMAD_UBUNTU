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


def nearest_path_distance(points, data):
    paths = np.concatenate([np.asarray(p) for p in data['paths'].values()])
    result = np.full(len(points), np.inf)
    for start in range(0, len(paths), 64):
        delta = points[:, None, :] - paths[None, start:start+64, :]
        result = np.minimum(result, np.sqrt((delta*delta).sum(axis=-1)).min(axis=1))
    return result


def warm_forest_floor(source, materials, data):
    """Bake CC0 photographed soil and litter into one bounded-size ground atlas."""
    # Route metadata is intentionally not used as a material/road mask.
    rows, cols = 2801, 2001
    rng = np.random.default_rng(20261001)
    def noise(size):
        small = Image.fromarray(rng.integers(0, 256, (size, size), dtype=np.uint8))
        return np.asarray(small.resize((cols, rows), Image.Resampling.BICUBIC),
                          dtype=np.float32)/255.0 - 0.5
    broad, fine = noise(26), noise(150)
    def photographed(name, tile_metres, shift):
        with Image.open(materials/name) as image:
            image = image.convert('RGB')
            # 2.5 cm atlas pixels; native tile data remains available offline.
            tile = np.asarray(image.resize((round(tile_metres/0.025),)*2,
                                           Image.Resampling.LANCZOS))
        iy = (np.arange(rows)+shift) % tile.shape[0]
        ix = (np.arange(cols)+shift*3) % tile.shape[1]
        return tile[iy[:, None], ix[None, :]].astype(np.float32)
    dirt = photographed('brown_mud_leaves_01_diff_1k.jpg', 3.5, 31)
    litter = photographed('leaves_forest_ground_diff_1k.jpg', 4.7, 79)
    # Independent scales, rotation and broad tonal patches break obvious tiling.
    dirt2 = np.rot90(photographed('brown_mud_leaves_01_diff_1k.jpg', 5.1, 113), 2)
    dirt = dirt*0.72 + dirt2*0.28
    litter_weight = np.clip(0.48+broad*0.8+fine*0.3, 0.12, 0.85)
    atlas = litter_weight[..., None]*litter + (1-litter_weight[..., None])*dirt
    atlas *= (0.93+broad*0.24+fine*0.12)[..., None]
    return np.clip(atlas, 0, 255).astype(np.uint8)


def elevated_backdrop(shape, data):
    """High rear plateau with a short left ascent and a long right ascent."""
    resolution, b = data['height_resolution_m'], data['bounds']
    x, y = np.meshgrid(b['x_min']+np.arange(shape[0])*resolution,
                       b['y_min']+np.arange(shape[1])*resolution, indexing='ij')
    def smoothstep(value):
        t = np.clip(value, 0, 1)
        return t*t*(3-2*t)
    right = smoothstep((x+10)/20)
    ramp_start = -7-5*right
    ramp_length = 7.8+14.2*right
    return 4.0*smoothstep((y-ramp_start)/ramp_length)


def fork2_ramp(terrain, data):
    """Ease only the Fork2 connecting corridor; retain Fork1's steep ascent."""
    profile = data.get('fork2_ramp')
    if not profile or not profile.get('enabled', True):
        return terrain
    path = np.asarray(data['paths'][profile['path']], dtype=float)
    segments = np.diff(path, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths <= 0):
        raise ValueError('Fork2 ramp path must have distinct consecutive points')
    arc = np.r_[0., np.cumsum(lengths)]
    resolution, bounds = data['height_resolution_m'], data['bounds']
    x, y = np.meshgrid(bounds['x_min']+np.arange(terrain.shape[0])*resolution,
                       bounds['y_min']+np.arange(terrain.shape[1])*resolution, indexing='ij')
    points = np.column_stack((x.ravel(), y.ravel()))
    distance = np.full(len(points), np.inf)
    nearest_arc = np.zeros(len(points))
    for i, (start, delta, length) in enumerate(zip(path[:-1], segments, lengths)):
        fraction = np.clip(((points-start)@delta)/(length*length), 0., 1.)
        candidate = np.linalg.norm(points-(start+fraction[:, None]*delta), axis=1)
        closer = candidate < distance
        distance[closer] = candidate[closer]
        nearest_arc[closer] = arc[i]+fraction[closer]*length
    half_width = profile['clear_width_m']/2
    feather = profile['blend_width_m']
    if half_width <= 0 or feather <= 0:
        raise ValueError('Fork2 ramp widths must be positive')
    weight = np.clip((half_width+feather-distance)/feather, 0., 1.)
    weight = weight*weight*(3-2*weight)
    fraction = nearest_arc/arc[-1]
    start_height, end_height = [level(point, terrain, data) for point in (path[0], path[-1])]
    floor = start_height+(end_height-start_height)*fraction*fraction*(3-2*fraction)
    weight = weight.reshape(terrain.shape)
    return terrain*(1-weight)+floor.reshape(terrain.shape)*weight


def woodland_relief(heights, data):
    """Continuous rolling woodland ground, including the driving corridors."""
    resolution, b = data['height_resolution_m'], data['bounds']
    x, y = np.meshgrid(b['x_min']+np.arange(heights.shape[0])*resolution,
                       b['y_min']+np.arange(heights.shape[1])*resolution, indexing='ij')
    # Metre-scale slopes rather than sharp steps: provisional car has no suspension.
    ripple = 0.18*np.sin(x*0.72+y*0.24)*np.sin(y*0.34-x*0.15)
    ripple += 0.12*np.sin(x*0.45-y*0.61)
    ripple += 0.025*np.sin(x*1.35+y*0.45)*np.sin(y*1.1)
    # Keep the initial contact patch level; smoothly join the rolling ground.
    sx, sy = data['paths']['entrance'][0]
    blend = np.clip((np.hypot(x-sx, y-sy)-1.2)/2.0, 0, 1)
    blend = blend*blend*(3-2*blend)
    # Original PNG is preserved as an archived reference, not added as a second hill.
    base = elevated_backdrop(heights.shape, data)
    # Right corridor stays low until its final bend; the main ascent is beyond
    # the terminal blockade. Preserve the surrounding plateau and left ascent.
    profile = data.get('route_corridor_profile', dict(right_block_arc_m=39.0, clear_width_m=4.0))
    path = np.asarray(data['paths']['flat_to_wall'])
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    selected = arc <= profile['right_block_arc_m']
    points = np.column_stack((x.ravel(), y.ravel()))
    best_distance = np.full(len(points), np.inf)
    nearest_arc = np.zeros(len(points))
    for start in range(0, int(selected.sum()), 32):
        batch = path[selected][start:start+32]
        delta = points[:, None, :]-batch[None, :, :]
        distance = np.sqrt((delta*delta).sum(axis=-1))
        indices = distance.argmin(axis=1)
        local = distance[np.arange(len(points)), indices]
        closer = local < best_distance
        best_distance[closer] = local[closer]
        nearest_arc[closer] = arc[selected][start+indices[closer]]
    half_width = profile['clear_width_m']/2
    weight = np.clip((half_width+1.2-best_distance)/1.2, 0, 1)
    weight = weight*weight*(3-2*weight)
    low_floor = .65*np.clip(nearest_arc/profile['right_block_arc_m'], 0, 1)
    base = base*(1-weight.reshape(heights.shape))+low_floor.reshape(heights.shape)*weight.reshape(heights.shape)
    return fork2_ramp(base + blend*ripple, data)


def roadside_details(folder, heights, data):
    """Batch irregular shoulder stones and fallen wood; no extra physics bodies."""
    rng = np.random.default_rng(71023)
    vertices, faces = [], defaultdict(list)
    def triangle(a, b, c, material):
        first = len(vertices)+1
        vertices.extend((a, b, c))
        faces[material].append((first, first+1, first+2))
    def ellipsoid(center, radii, material):
        # Asymmetric rings, 32 triangles per stone rather than a primitive sphere.
        rings = []
        for z, radius in ((-0.75, 0.6), (0, 1), (0.65, 0.7)):
            ring = []
            for k in range(8):
                a = 2*math.pi*k/8
                jitter = rng.uniform(0.82, 1.18)
                ring.append(np.asarray(center)+np.asarray(radii)*
                            [radius*math.cos(a)*jitter, radius*math.sin(a)*jitter, z])
            rings.append(ring)
        for lo, hi in zip(rings, rings[1:]):
            for k in range(8):
                n = (k+1)%8
                triangle(lo[k], lo[n], hi[n], material)
                triangle(lo[k], hi[n], hi[k], material)
        top = np.asarray(center)+np.asarray(radii)*[0.1, 0.05, 1]
        for k in range(8):
            triangle(rings[-1][k], rings[-1][(k+1)%8], top, material)
    def branch(start, end, radius):
        axis = np.asarray(end)-start
        axis /= np.linalg.norm(axis)
        u = np.cross(axis, [0, 0, 1]); u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        for k in range(8):
            a, b = k*math.pi/4, (k+1)*math.pi/4
            da, db = radius*(u*math.cos(a)+v*math.sin(a)), radius*(u*math.cos(b)+v*math.sin(b))
            triangle(start+da, start+db, end+db*0.7, 'fallen_bark')
            triangle(start+da, end+db*0.7, end+da*0.7, 'fallen_bark')
    paths = np.concatenate([np.asarray(p) for p in data['paths'].values()])
    candidates = []
    for _ in range(650):
        i = int(rng.integers(1, len(paths)-1))
        tangent = paths[min(i+1, len(paths)-1)]-paths[max(i-1, 0)]
        if np.linalg.norm(tangent) < 0.001:
            continue
        normal = np.array([-tangent[1], tangent[0]])/np.linalg.norm(tangent)
        candidates.append(paths[i]+normal*rng.choice([-1, 1])*rng.uniform(2, 4))
    candidates = np.asarray(candidates)
    distance = nearest_path_distance(candidates, data)
    b = data['bounds']; count = 0
    for p, d in zip(candidates, distance):
        if not (1.9 < d < 4.1 and b['x_min']+1 < p[0] < b['x_max']-1
                and b['y_min']+1 < p[1] < b['y_max']-1):
            continue
        if count >= 260:
            break
        radius = rng.uniform(0.08, 0.26)
        z = level(p, heights, data)
        ellipsoid([p[0], p[1], z+radius*0.25], [radius, radius*rng.uniform(.6, 1.3), radius*.6],
                  'stone_'+str(count%4))
        if count%6 == 0 and d > 2.7:
            angle = rng.uniform(-math.pi, math.pi)
            delta = np.array([math.cos(angle), math.sin(angle)])*rng.uniform(.4, .8)
            p0, p1 = p-delta/2, p+delta/2
            start = np.array([*p0, level(p0, heights, data)+.07])
            end = np.array([*p1, level(p1, heights, data)+.07])
            branch(start, end, .055)
        count += 1
    destination = folder/'roadside_details.obj'
    with destination.open('w', encoding='utf-8') as stream:
        stream.write('mtllib roadside_details.mtl\no roadside_details\ns off\n')
        for vertex in vertices:
            stream.write('v '+' '.join(f'{v:.6f}' for v in vertex)+'\n')
        # Ogre2 needs explicit normals for dependable lighting of OBJ details.
        for i in range(0, len(vertices), 3):
            a, b, c = np.asarray(vertices[i:i+3])
            normal = np.cross(b-a, c-a)
            normal /= max(np.linalg.norm(normal), 1e-12)
            stream.write('vn '+' '.join(f'{v:.6f}' for v in normal)+'\n')
        for material, triangles in faces.items():
            stream.write(f'usemtl {material}\n')
            for face in triangles:
                normal_index = (face[0]-1)//3+1
                stream.write('f '+' '.join(f'{v}//{normal_index}' for v in face)+'\n')
    colours = {'stone_0':(.29,.27,.23), 'stone_1':(.43,.42,.37),
               'stone_2':(.25,.29,.22), 'stone_3':(.52,.48,.39), 'fallen_bark':(.24,.16,.095)}
    (folder/'roadside_details.mtl').write_text('\n'.join(
        f'newmtl {name}\nKa 0.1 0.1 0.1\nKd '+ ' '.join(map(str, values))+'\nKs 0 0 0\nd 1\nillum 1'
        for name, values in colours.items())+'\n', encoding='utf-8')
    return count


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
        origin = [*instance['center'], level(instance['center'], heights, data)+instance.get('z_offset', 0)]
        scale = np.asarray(instance.get('scale_xyz', [instance.get('scale', 1.0)]*3))
        output_vertices.extend((v0*scale)@rotation.T+origin)
        transformed_normals = n0/scale
        transformed_normals /= np.linalg.norm(transformed_normals, axis=1, keepdims=True)
        output_normals.extend(transformed_normals@rotation.T)
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


def pocket_geometry(package, data, heights):
    """One continuous rock cave mesh: rounded walls, domed roof, one open mouth."""
    spec = json.loads((package/'assets/authoring/dead_end_pocket.json').read_text())
    center = np.asarray(spec['center_uv_m'])
    radii = np.asarray(spec['radii_uv_m'])
    angle = math.radians(data['wall']['yaw_deg'])
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    origin = np.asarray(data['wall']['center'])
    approach = (np.asarray(data['paths']['flat_to_wall'])-origin)@rotation
    normalized = (approach-center)/radii
    residual = (normalized*normalized).sum(axis=1)-1
    entry = None
    for i in range(len(approach)-1):
        if residual[i] > 0 >= residual[i+1]:
            t = residual[i]/(residual[i]-residual[i+1])
            entry = approach[i]*(1-t)+approach[i+1]*t
            break
    if entry is None:
        raise ValueError('Right approach does not enter cave footprint')
    entrance_angle = math.atan2(*((entry-center)/radii)[::-1])
    half_gap = spec['entrance_width_m']/(2*float(radii.mean()))
    spec['entrance_angle_rad'] = entrance_angle
    spec['entrance_world_xy'] = (origin+rotation@entry).tolist()
    angles = np.linspace(entrance_angle+half_gap, entrance_angle+2*math.pi-half_gap, 73)
    full_angles = np.linspace(0, 2*math.pi, 97)
    perimeter = [origin+rotation@(center+radii*[math.cos(t), math.sin(t)]) for t in full_angles]
    roof_edge = max(level(p, heights, data) for p in perimeter)+spec['clearance_above_highest_ground_m']
    triangles = []
    rng = np.random.default_rng(61231)
    def triangle(a, b, c):
        triangles.append((np.asarray(a), np.asarray(b), np.asarray(c), int(rng.integers(0, 3))))
    def quad(a, b, c, d):
        triangle(a, b, c); triangle(a, c, d)
    # Variable rock thickness and ledges, without rows of individually stacked stones.
    inner, outer = [], []
    for t in angles:
        radial = np.array([math.cos(t), math.sin(t)])
        point = origin+rotation@(center+radii*radial)
        low = level(point, heights, data)-.35
        rings_in, rings_out = [], []
        for k, height in enumerate((low, low+1.0, roof_edge-0.7, roof_edge)):
            inset = (0, .14, .38, 0)[k]+.10*math.sin(5*t+k)
            uv = center+(radii-inset)*radial
            p = origin+rotation@uv
            rings_in.append([*p, height])
            thickness = spec['rock_thickness_m']+.3*math.sin(3*t+k)**2
            out = origin+rotation@(center+(radii+thickness)*radial)
            rings_out.append([*out, height+(.45 if k==3 else 0)])
        inner.append(rings_in); outer.append(rings_out)
    for i in range(len(angles)-1):
        for k in range(3):
            quad(inner[i][k], inner[i][k+1], inner[i+1][k+1], inner[i+1][k])
            quad(outer[i][k], outer[i+1][k], outer[i+1][k+1], outer[i][k+1])
        quad(inner[i][-1], outer[i][-1], outer[i+1][-1], inner[i+1][-1])
    # Close the exposed rock cut faces either side of the mouth.
    for i in (0, len(angles)-1):
        for k in range(3):
            quad(inner[i][k], outer[i][k], outer[i][k+1], inner[i][k+1])
    # Rounded stone cap, not a flat building roof. Both underside and upper skin
    # are in the collision mesh; entrance remains clear below the cap.
    def roof_point(t, radius, upper):
        radial = np.array([math.cos(t), math.sin(t)])
        p = origin+rotation@(center+radii*radial*radius)
        z = roof_edge+spec['roof_dome_height_m']*math.sqrt(max(0, 1-radius*radius))
        z += .08*math.sin(4*t)*radius*(1-radius)
        return [*p, z+(.45 if upper else 0)]
    for upper in (False, True):
        for r0, r1 in zip(np.linspace(0, 1, 9)[:-1], np.linspace(0, 1, 9)[1:]):
            for t0, t1 in zip(full_angles[:-1], full_angles[1:]):
                points = [roof_point(t0, r0, upper), roof_point(t0, r1, upper),
                          roof_point(t1, r1, upper), roof_point(t1, r0, upper)]
                if not upper:
                    points.reverse()
                if r0 == 0:
                    # Avoid degenerate triangles at the dome centre.
                    if upper: triangle(points[0], points[1], points[2])
                    else: triangle(points[0], points[1], points[2])
                else:
                    quad(*points)
        if upper:
            for t0, t1 in zip(full_angles[:-1], full_angles[1:]):
                quad(roof_point(t0, 1, False), roof_point(t1, 1, False),
                     roof_point(t1, 1, True), roof_point(t0, 1, True))
    destination = package/'assets/geometry/dead_end_pocket.obj'
    with destination.open('w', encoding='utf-8') as stream:
        stream.write('mtllib dead_end_cave.mtl\no natural_rock_cave\ns off\n')
        for a, b, c, _ in triangles:
            for p in (a, b, c):
                stream.write('v '+' '.join(f'{v:.6f}' for v in p)+'\n')
        for a, b, c, _ in triangles:
            normal = np.cross(b-a, c-a)
            normal /= max(np.linalg.norm(normal), 1e-12)
            stream.write('vn '+' '.join(f'{v:.6f}' for v in normal)+'\n')
        for material in range(3):
            stream.write(f'usemtl rock_{material}\n')
            for i, (_, _, _, face_material) in enumerate(triangles):
                if face_material == material:
                    stream.write('f '+' '.join(f'{i*3+k+1}//{i+1}' for k in range(3))+'\n')
    return spec, []

def corridor_geometry(package, data, heights):
    """Scatter irregular woodland patches, not a single line of boundary trees."""
    spec = json.loads((package/'assets/authoring/route_corridors.json').read_text())
    rng = np.random.default_rng(102026)
    sections = {}
    for name in ('flat_to_wall', 'rock_branch'):
        path = np.asarray(data['paths'][name])
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        stop = spec['right_block_arc_m'] if name == 'flat_to_wall' else float(
            arc[np.argmin(np.linalg.norm(path-np.mean([r['center'] for r in data['rocks']], axis=0), axis=1))])
        distances = np.linspace(spec['mouth_open_length_m'], stop,
                                math.ceil((stop-spec['mouth_open_length_m'])/spec['sample_step_m'])+1)
        points = np.column_stack([np.interp(distances, arc, path[:, a]) for a in (0, 1)])
        tangents = np.gradient(points, axis=0)
        tangents /= np.linalg.norm(tangents, axis=1)[:, None]
        normals = np.column_stack((-tangents[:,1], tangents[:,0]))
        offset = spec['clear_width_m']/2+spec['boundary_thickness_m']/2
        sections[name] = dict(block_arc_m=float(stop), block_center_xy=points[-1].tolist(),
                              centerline=points.tolist(), sides=[(points+normals*offset).tolist(),
                              (points-normals*offset).tolist()], end_normal=normals[-1].tolist())
    b = data['bounds']; step = spec['probe_step_m']
    x, y = np.meshgrid(np.arange(b['x_min']+1, b['x_max']-1, step),
                       np.arange(b['y_min']+1, b['y_max']-1, step), indexing='ij')
    candidates = np.column_stack((x.ravel(), y.ravel()))
    # Jitter avoids visible rows, including the sampling grid itself.
    candidates += rng.uniform(-step*.4, step*.4, candidates.shape)
    blocked = np.zeros(len(candidates), dtype=bool)
    for section in sections.values():
        path = np.asarray(section['centerline'])
        minimum = np.full(len(candidates), np.inf)
        nearest = np.zeros(len(candidates), dtype=int)
        for start in range(0, len(path), 32):
            distances = np.linalg.norm(candidates[:,None,:]-path[None,start:start+32,:], axis=-1)
            index = distances.argmin(axis=1)
            values = distances[np.arange(len(candidates)), index]
            closer = values < minimum
            minimum[closer] = values[closer]
            nearest[closer] = start+index[closer]
        low = spec['clear_width_m']/2+.28
        band = (minimum >= low)&(minimum <= low+spec['forest_band_depth_m'])
        # Leave the fork open; fill only the off-route region farther along.
        band &= nearest >= 3
        end = np.asarray(section['block_center_xy'])
        normal = np.asarray(section['end_normal'])
        forward = np.array([normal[1], -normal[0]])
        delta = candidates-end
        ahead = delta@forward
        lateral = np.abs(delta@normal)
        end_patch = (ahead >= -.15)&(ahead <= spec['blocked_patch_depth_m'])&(
            lateral < low+spec['forest_band_depth_m'])
        blocked |= band|end_patch
    # Never obstruct the entrance/left ascent or the connecting route into fork 2.
    protected = dict(paths={k:data['paths'][k] for k in ('entrance','uphill','flat_to_fork')})
    blocked &= nearest_path_distance(candidates, protected) > spec['clear_width_m']/2+.4
    probes = candidates[blocked]
    # Farthest-gap filling keeps the count low without creating straight fences.
    # Different positions, rotations and scales are reproducible, not grid rows.
    instances, remaining = [], np.full(len(probes), np.inf)
    index = int(rng.integers(0, len(probes)))
    while len(instances) == 0 or float(remaining.max()) > spec['max_probe_clearance_m']:
        if instances:
            score = remaining+rng.uniform(-.04, .04, len(remaining))
            index = int(score.argmax())
        p = probes[index]
        scale = float(rng.uniform(1.30, 1.65))
        instances.append(dict(center=p.tolist(), yaw_deg=float(rng.uniform(0,360)), scale=scale))
        remaining = np.minimum(remaining, np.linalg.norm(probes-p, axis=1)-.18*scale)
        if len(instances) > 1200:
            raise ValueError('Unexpected woodland patch density')
    batch_mesh(package/'assets/models/pine_tree.obj',
               package/'assets/geometry/woodland_patches.obj', instances, heights, data)
    spec['sections'] = sections
    spec['trees'] = instances
    spec['region_probe_max_clearance_m'] = float(remaining.max())
    spec['placement'] = 'Irregular broad patches, no aligned tree rows; nominal vehicle width 0.68 m'
    return spec


def increase_forest_density(package, data):
    """Densify the original whole-map forest, not selected blocking regions."""
    spec = json.loads((package/'assets/authoring/forest_density.json').read_text())
    trees = list(data['trees'])
    original_count = len(trees)
    target = math.ceil(original_count*spec['multiplier'])
    rng = np.random.default_rng(spec['seed'])
    starts = np.concatenate([np.asarray(p[:-1]) for p in data['paths'].values()])
    ends = np.concatenate([np.asarray(p[1:]) for p in data['paths'].values()])
    segments = ends-starts
    lengths = np.maximum((segments*segments).sum(axis=1), 1e-12)
    b = data['bounds']
    attempts = 0
    while len(trees) < target:
        attempts += 1
        if attempts > 100000:
            raise ValueError('Cannot reach requested forest density with current spacing')
        center = rng.uniform([b['x_min']+1, b['y_min']+1], [b['x_max']-1, b['y_max']-1])
        if min(np.linalg.norm(center-np.asarray(t['center'])) for t in trees) < spec['min_tree_spacing_m']:
            continue
        projection = np.clip(((center-starts)*segments).sum(axis=1)/lengths, 0, 1)
        distance = float(np.linalg.norm(center-starts-segments*projection[:,None], axis=1).min())
        if distance < spec['min_path_clearance_m']:
            continue
        trees.append(dict(center=center.tolist(), yaw_deg=float(rng.uniform(-180,180)),
                          scale=float(rng.uniform(.85,1.3)), road_distance_m=round(distance,4),
                          collision_enabled=distance <= data['tree_collision_distance_m']))
    data['trees'] = trees
    spec.update(original_trees=original_count, total_trees=target, added_trees=target-original_count)
    return spec


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
    detail_link = static_model(world, 'roadside_details')
    mesh(sub(detail_link, 'visual', name='visual_only'), '../assets/geometry/roadside_details.obj')
    if vegetation:
        for name, uri in (('forest_far', 'forest_far.obj'), ('forest_grass', 'grass.obj'),
                          ('path_grass', 'path_grass.obj'), ('meadow_grass', 'meadow_grass.obj')):
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
    include = sub(world, 'include')
    sub(include, 'uri', 'model://nomad_vehicle'); sub(include, 'name', 'nomad_vehicle')
    start, next_point = data['paths']['entrance'][:2]
    yaw = math.atan2(next_point[1]-start[1], next_point[0]-start[0])
    spawn_profile = package/'assets/authoring/spawn_pose.json'
    if spawn_profile.exists():
        yaw = float(json.loads(spawn_profile.read_text())['yaw_rad'])
        if not math.isfinite(yaw):
            raise ValueError('Spawn yaw must be finite')
    sub(include, 'pose', f'{start[0]} {start[1]} 0.03 0 0 {yaw}')
    filename = 'forest.sdf' if vegetation else 'forest_bare.sdf'
    save_xml(root, package/'worlds'/filename)


def path_grass(package, data):
    """Visual-only tall grass over route footprints, without extra physics bodies."""
    spec = json.loads((package/'assets/authoring/path_grass.json').read_text())
    rng = np.random.default_rng(spec['seed'])
    points = np.asarray(data['paths'][spec['route']])
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    index = min(len(points)-2, max(0, int(np.searchsorted(arc, spec['center_fraction']*arc[-1])-1)))
    tangent = points[index+1]-points[index]
    tangent /= np.linalg.norm(tangent)
    angle = spec.get('yaw_offset_rad', 0.0)
    c, s = math.cos(angle), math.sin(angle)
    tangent = np.asarray([[c, -s], [s, c]])@tangent
    sideways = np.asarray([-tangent[1], tangent[0]])
    origin = np.asarray([np.interp(spec['center_fraction']*arc[-1], arc, points[:, axis])
                         for axis in (0, 1)])
    step = spec['spacing_m']
    depth, width = spec['along_path_depth_m'], spec['cross_path_width_m']
    offsets = np.asarray([(a, b) for a in np.arange(-depth/2+step/2, depth/2, step)
                          for b in np.arange(-width/2+step/2, width/2, step)])
    offsets += rng.uniform(-0.15*step, 0.15*step, offsets.shape)
    candidates = origin+offsets[:, :1]*tangent+offsets[:, 1:]*sideways
    data['path_grass_wall'] = dict(center=origin.tolist(), tangent=tangent.tolist(),
                                   sideways=sideways.tolist(), yaw_offset_rad=angle)
    source_height = max(float(line.split()[3]) for line in
                        (package/'assets/models/grass_clump.obj').read_text().splitlines()
                        if line.startswith('v '))
    instances = []
    for center in candidates:
        height = rng.uniform(*spec['height_range_m'])
        spread = rng.uniform(1.0, 1.35)
        instances.append(dict(center=center.tolist(), yaw_deg=float(rng.uniform(0, 360)),
                              scale_xyz=[spread, spread, height/source_height]))
    data['path_grass'] = instances
    # Separate sparse, lower grass over the whole map, not a carpet along roads.
    bounds = data['bounds']
    background_step = spec['background_spacing_m']
    background = []
    for x in np.arange(bounds['x_min']+1, bounds['x_max']-1, background_step):
        for y in np.arange(bounds['y_min']+1, bounds['y_max']-1, background_step):
            center = np.asarray([x, y])+rng.uniform(-0.35, 0.35, 2)*background_step
            height = rng.uniform(*spec['background_height_range_m'])
            spread = rng.uniform(0.8, 1.2)
            background.append(dict(center=center.tolist(), yaw_deg=float(rng.uniform(0, 360)),
                                   scale_xyz=[spread, spread, height/source_height]))
    # Filter only after all random draws, preserving every other clump exactly.
    for exclusion in spec.get('background_exclusions', []):
        center = np.asarray(exclusion['center'], dtype=float)
        radius = float(exclusion['radius_m'])
        if center.shape != (2,) or not np.all(np.isfinite(center)) or not math.isfinite(radius) or radius <= 0:
            raise ValueError('Invalid background grass exclusion')
        background = [clump for clump in background
                      if np.linalg.norm(np.asarray(clump['center'])-center) > radius]
    data['meadow_grass'] = background
    return dict(spec, clumps=len(instances), background_clumps=len(background), collision_enabled=False)


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
    data['route_corridor_profile'] = json.loads((author/'route_corridors.json').read_text(encoding='utf-8'))
    density_spec = increase_forest_density(package, data)
    grass_spec = path_grass(package, data)
    (author/'forest_layout.json').write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
    with Image.open(author/'height.png') as image:
        heights = woodland_relief(np.asarray(image, dtype=float)*data['height_max_m']/255, data)
    with Image.open(author/'terrain.png') as image:
        north_up = np.flipud(np.transpose(warm_forest_floor(image, package/'assets/materials', data), (1, 0, 2)))
        Image.fromarray(north_up).save(geometry/'terrain_texture.png')
    terrain_mesh(geometry, heights, data)
    detail_count = roadside_details(geometry, heights, data)
    far = [tree for tree in data['trees'] if not tree['collision_enabled']]
    batch_mesh(models/'pine_tree.obj', geometry/'forest_far.obj', far, heights, data)
    batch_mesh(models/'grass_clump.obj', geometry/'grass.obj', data['grass'], heights, data)
    batch_mesh(models/'grass_clump.obj', geometry/'path_grass.obj', data['path_grass'], heights, data)
    batch_mesh(models/'grass_clump.obj', geometry/'meadow_grass.obj', data['meadow_grass'], heights, data)
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
                  visual_only_path_grass=grass_spec,
                  rocks=0, collision_boundary_trees=0, no_roadside_boundary=True,
                  visual_only_shoulder_stones=detail_count,
                  ground_texture='Continuous Poly Haven CC0 mud/litter blend; no road mask, 2801x2001 atlas',
                  ground_relief='Rear plateau 4 m; short steep left ascent, long gentle right ascent; rolling relief <= +/-0.325 m; spawn flat',
                  terrain_profile=dict(rear_plateau_height_m=4.0, left_ramp_length_m=7.8,
                                       right_ramp_length_m=22.0, fork2_ramp=data.get('fork2_ramp'), road_material_mask=False),
                  forest_density=density_spec,
                  limitations=['No automatic traversability, branch memory or route planning.',
                               'Vehicle mass and geometry are provisional, not measured hardware.',
                               'Motor effort limits and payload climbing failure are not verified.'])
    (package/'assets/export.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    bare_report = dict(report, vegetation_enabled=False, rendered_trees=0,
                       collision_trees=0, visual_only_trees=0, visual_only_grass=0)
    bare_report['visual_only_path_grass'] = dict(grass_spec, clumps=0, background_clumps=0)
    (package/'assets/export_bare.json').write_text(json.dumps(bare_report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--map-package', type=Path, help='Initial import; otherwise use saved authoring snapshot')
    args = parser.parse_args()
    generate(args.output_dir.resolve(), args.map_package.resolve() if args.map_package else None)
