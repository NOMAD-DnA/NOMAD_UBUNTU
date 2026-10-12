"""Simulation-only semantic oracle for observed grass mesh surfaces.

Uses the selected world's authored geometry, NOT a sensor-only classifier.
World and planning coordinates must agree (builtin Gazebo ground-truth VIO).
Only measured endpoints within 5 mm of a grass triangle are labelled; neither
whole clump bounding boxes nor space behind an endpoint are made traversable.
"""
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

GRASS_MODELS = {'undergrowth', 'forest_grass', 'path_grass', 'meadow_grass'}


def _pose(element):
    pose = element.find('pose')
    if pose is not None and (pose.get('relative_to') or pose.get('degrees') == 'true'
                             or pose.get('rotation_format', 'euler_rpy') != 'euler_rpy'):
        raise ValueError('Grass oracle requires local Euler-radian SDF poses')
    values = np.fromstring('0 0 0 0 0 0' if pose is None else pose.text, sep=' ')
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError('Invalid grass pose')
    x, y, z, roll, pitch, yaw = values
    cr, cp, cy = np.cos([roll, pitch, yaw]); sr, sp, sy = np.sin([roll, pitch, yaw])
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                      [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def _triangles(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        words = line.split()
        if not words:
            continue
        if words[0] == 'v':
            vertices.append(list(map(float, words[1:4])))
        elif words[0] == 'f':
            indices = [int(word.split('/')[0]) for word in words[1:]]
            indices = [i-1 if i > 0 else len(vertices)+i for i in indices]
            faces.extend((indices[0], indices[i], indices[i+1]) for i in range(1, len(indices)-1))
    return np.asarray(vertices, dtype=float)[np.asarray(faces, dtype=int)]


class SimulatorGrass:
    def __init__(self, triangles, tolerance=.005, bin_size=.4):
        self.tolerance, self.bin_size = tolerance, bin_size
        triangles = np.asarray(triangles, dtype=float).reshape(-1, 3, 3)
        if not np.isfinite(triangles).all():
            raise ValueError('Non-finite grass mesh')
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        triangles = triangles[np.linalg.norm(normals, axis=1) > 1e-12]
        self.triangles = triangles
        self.bins = {}
        lower = np.floor((triangles.min(axis=1)-tolerance)/bin_size).astype(int)
        upper = np.floor((triangles.max(axis=1)+tolerance)/bin_size).astype(int)
        for i, (lo, hi) in enumerate(zip(lower, upper)):
            for key in product(*(range(a, b+1) for a, b in zip(lo, hi))):
                self.bins.setdefault(key, []).append(i)
        self.bins = {key: np.array(ids) for key, ids in self.bins.items()}

    @classmethod
    def from_world(cls, world_path):
        path = Path(world_path)
        world = ET.parse(path).getroot().find('world')
        if world is None:
            raise ValueError('Grass oracle requires an SDF world')
        triangles = []
        for model in world.findall('model'):
            if model.get('name') not in GRASS_MODELS:
                continue
            if model.findtext('static', 'false').lower() not in ('true', '1'):
                raise ValueError('Grass oracle only supports static grass')
            for link in model.findall('link'):
                if link.find('collision') is not None:
                    raise ValueError('Do not mark physically collidable grass as passable')
                for visual in link.findall('visual'):
                    mesh = visual.find('geometry/mesh')
                    if mesh is None:
                        raise ValueError('Grass oracle requires OBJ meshes')
                    uri = mesh.findtext('uri', '')
                    if '://' in uri or Path(uri).suffix.lower() != '.obj':
                        raise ValueError('Grass oracle requires a local OBJ URI')
                    verts = _triangles((path.parent / uri).resolve())
                    scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
                    if len(scale) != 3 or not np.isfinite(scale).all() or (scale <= 0).any():
                        raise ValueError('Invalid grass mesh scale')
                    transform = _pose(model) @ _pose(link) @ _pose(visual)
                    triangles.append((verts*scale) @ transform[:3, :3].T+transform[:3, 3])
        return cls(np.concatenate(triangles) if triangles else np.empty((0, 3, 3)))

    def matches(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        result = np.zeros(len(points), dtype=bool)
        valid = np.flatnonzero(np.isfinite(points).all(axis=1))
        keys = np.floor(points[valid]/self.bin_size).astype(int)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        groups = np.split(np.argsort(inverse), np.cumsum(np.bincount(inverse))[:-1]) if len(valid) else []
        for key, group in zip(unique, groups):
            candidates = self.bins.get(tuple(key))
            if candidates is None:
                continue
            tri = self.triangles[candidates]
            a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
            ab, ac = b-a, c-a
            normals = np.cross(ab, ac)
            normals /= np.linalg.norm(normals, axis=1)[:, None]
            # Bound intermediate matrices even for a large nearby grass wall.
            for start in range(0, len(group), 64):
                ids = valid[group[start:start+64]]
                p = points[ids, None, :]
                signed = np.sum((p-a)*normals, axis=2)
                projected = p-signed[:, :, None]*normals
                inside = np.ones(signed.shape, dtype=bool)
                distance2 = np.full(signed.shape, np.inf)
                for v, w in ((a, b), (b, c), (c, a)):
                    edge = w-v
                    inside &= np.sum(np.cross(edge, projected-v)*normals, axis=2) >= -1e-12
                    factor = np.clip(np.sum((p-v)*edge, axis=2)/np.sum(edge*edge, axis=1), 0, 1)
                    delta = p-(v+factor[:, :, None]*edge)
                    distance2 = np.minimum(distance2, np.sum(delta*delta, axis=2))
                distance2 = np.where(inside, signed*signed, distance2)
                result[ids] = np.any(distance2 <= self.tolerance**2, axis=1)
        return result
