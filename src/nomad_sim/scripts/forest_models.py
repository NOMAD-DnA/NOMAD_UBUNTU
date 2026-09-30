#!/usr/bin/env python3
"""Generate small, texture-free OBJ/MTL forest assets for NOMAD / MVSim.

Every model uses metres, +Z up, and an origin on the ground (z=0).
These meshes are VISUAL assets, not collision geometry. MVSim block polygons
must define collision footprints separately: foliage and grass must not become
solid collision obstacles. A tree's collision footprint normally covers only
its trunk, whereas a rock may need its own simplified footprint.

No Blender, third-party Python packages, or downloaded models are required.
"""

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, NOMAD contributors
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field
from pathlib import Path


Vec3 = tuple[float, float, float]
Face = tuple[tuple[int, int, int], str]


@dataclass
class _Mesh:
    name: str
    materials: dict[str, Vec3]
    vertices: list[Vec3] = field(default_factory=list)
    faces: list[Face] = field(default_factory=list)

    def vertex(self, xyz: Vec3) -> int:
        self.vertices.append(xyz)
        return len(self.vertices)  # OBJ indices start at one.

    def triangle(self, a: int, b: int, c: int, material: str) -> None:
        self.faces.append(((a, b, c), material))


def _ring(mesh: _Mesh, radius: float, z: float, sides: int) -> list[int]:
    return [
        mesh.vertex((radius * math.cos(a), radius * math.sin(a), z))
        for a in (math.tau * i / sides for i in range(sides))
    ]


def _connect_rings(
    mesh: _Mesh, lower: list[int], upper: list[int], materials: tuple[str, ...]
) -> None:
    for i, a in enumerate(lower):
        j = (i + 1) % len(lower)
        material = materials[i % len(materials)]
        mesh.triangle(a, lower[j], upper[j], material)
        mesh.triangle(a, upper[j], upper[i], material)


def _cylinder(mesh: _Mesh, radius: float, height: float, material: str) -> None:
    lower = _ring(mesh, radius, 0.0, 10)
    upper = _ring(mesh, radius, height, 10)
    bottom = mesh.vertex((0.0, 0.0, 0.0))
    top = mesh.vertex((0.0, 0.0, height))
    _connect_rings(mesh, lower, upper, (material,))
    for i in range(len(lower)):
        j = (i + 1) % len(lower)
        mesh.triangle(bottom, lower[j], lower[i], material)
        mesh.triangle(top, upper[i], upper[j], material)


def _cone(
    mesh: _Mesh, radius: float, base_z: float, tip_z: float,
    materials: tuple[str, ...]
) -> None:
    ring = _ring(mesh, radius, base_z, 12)
    bottom = mesh.vertex((0.0, 0.0, base_z))
    tip = mesh.vertex((0.0, 0.0, tip_z))
    for i, a in enumerate(ring):
        b = ring[(i + 1) % len(ring)]
        material = materials[i % len(materials)]
        mesh.triangle(a, b, tip, material)
        mesh.triangle(bottom, b, a, materials[0])


def _pine_tree() -> _Mesh:
    mesh = _Mesh("pine_tree", {
        "bark": (0.27, 0.17, 0.09),
        "pine_dark": (0.10, 0.25, 0.13),
        "pine_mid": (0.13, 0.31, 0.16),
        "pine_light": (0.16, 0.35, 0.18),
    })
    _cylinder(mesh, radius=0.18, height=3.35, material="bark")
    shades = ("pine_mid", "pine_dark", "pine_mid", "pine_light")
    # Closed, overlapping canopy layers are deliberately visual-only.
    _cone(mesh, 0.90, 1.35, 3.25, shades)
    _cone(mesh, 0.73, 2.15, 3.95, shades)
    _cone(mesh, 0.53, 3.05, 4.50, shades)
    return mesh


def _shrub() -> _Mesh:
    mesh = _Mesh("shrub", {
        "twig": (0.30, 0.21, 0.12),
        "leaf_dark": (0.17, 0.29, 0.10),
        "leaf_mid": (0.24, 0.37, 0.13),
        "leaf_light": (0.30, 0.43, 0.17),
    })
    _cylinder(mesh, radius=0.065, height=0.38, material="twig")
    rng = random.Random(3201)
    rings: list[list[int]] = []
    for radius, z in ((0.39, 0.18), (0.65, 0.47), (0.47, 0.78)):
        ring = []
        for i in range(12):
            angle = math.tau * i / 12
            # Cardinal points keep the nominal radius; other points add facets.
            scale = 1.0 if i % 3 == 0 else rng.uniform(0.90, 1.0)
            ring.append(mesh.vertex((
                radius * scale * math.cos(angle),
                radius * scale * math.sin(angle), z,
            )))
        rings.append(ring)
    shades = ("leaf_mid", "leaf_light", "leaf_mid", "leaf_dark")
    for lower, upper in zip(rings, rings[1:]):
        _connect_rings(mesh, lower, upper, shades)
    bottom = mesh.vertex((0.0, 0.0, 0.13))
    tip = mesh.vertex((0.035, -0.025, 1.0))
    for i in range(12):
        j = (i + 1) % 12
        mesh.triangle(bottom, rings[0][j], rings[0][i], "leaf_dark")
        mesh.triangle(rings[-1][i], rings[-1][j], tip, shades[i % 4])
    return mesh


def _grass_clump() -> _Mesh:
    mesh = _Mesh("grass_clump", {
        "grass_dark": (0.26, 0.34, 0.09),
        "grass_mid": (0.35, 0.43, 0.13),
        "grass_light": (0.43, 0.49, 0.17),
    })
    shades = ("grass_mid", "grass_dark", "grass_light")
    # Each blade is a thin, closed pyramid, not an alpha-textured billboard.
    # This remains visible from either side without relying on two-sided flags.
    for i in range(9):
        angle = math.tau * i / 8 if i < 8 else 0.35
        ux, uy = math.cos(angle), math.sin(angle)
        vx, vy = -uy, ux
        root_radius = 0.095 if i < 8 else 0.0
        tip_radius = (0.30 if i % 2 == 0 else 0.245) if i < 8 else 0.045
        height = (0.245 + (i % 3) * 0.032) if i < 8 else 0.35
        cx, cy = root_radius * ux, root_radius * uy
        half_width, half_depth = 0.026, 0.007
        base = [mesh.vertex((
            cx + sx * half_width * vx + sy * half_depth * ux,
            cy + sx * half_width * vy + sy * half_depth * uy, 0.0,
        )) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        # Reverse the winding: the (v,u) basis above has negative determinant.
        base.reverse()
        tip = mesh.vertex((tip_radius * ux, tip_radius * uy, height))
        material = shades[i % len(shades)]
        for j in range(4):
            mesh.triangle(base[j], base[(j + 1) % 4], tip, material)
        mesh.triangle(base[0], base[2], base[1], material)
        mesh.triangle(base[0], base[3], base[2], material)
    return mesh


def _rock() -> _Mesh:
    mesh = _Mesh("rock", {
        "stone_dark": (0.32, 0.30, 0.27),
        "stone_mid": (0.42, 0.40, 0.36),
        "stone_light": (0.49, 0.46, 0.41),
    })
    rng = random.Random(8042)
    rings: list[list[int]] = []
    for radius, z, offset in (
        (0.73, 0.0, (0.0, 0.0)),
        (1.00, 0.35, (0.05, -0.03)),
        (0.70, 0.87, (-0.11, 0.07)),
    ):
        ring = []
        for i in range(12):
            angle = math.tau * i / 12
            r = radius * rng.uniform(0.87, 1.13)
            ring.append(mesh.vertex((
                offset[0] + r * math.cos(angle),
                offset[1] + r * math.sin(angle), z,
            )))
        rings.append(ring)
    shades = ("stone_mid", "stone_dark", "stone_mid", "stone_light")
    for lower, upper in zip(rings, rings[1:]):
        _connect_rings(mesh, lower, upper, shades)
    bottom = mesh.vertex((0.0, 0.0, 0.0))
    tip = mesh.vertex((-0.13, 0.08, 1.10))
    for i in range(12):
        j = (i + 1) % 12
        mesh.triangle(bottom, rings[0][j], rings[0][i], "stone_dark")
        mesh.triangle(rings[-1][i], rings[-1][j], tip, shades[i % 4])
    # Normalize only the visual bounds, preserving a flat base at ground level.
    # The irregular rock has exact nominal dimensions: x=1.1, y=1.0, z=1.1m.
    x_min = min(v[0] for v in mesh.vertices)
    x_max = max(v[0] for v in mesh.vertices)
    y_min = min(v[1] for v in mesh.vertices)
    y_max = max(v[1] for v in mesh.vertices)
    mesh.vertices = [(
        (x - (x_min + x_max) / 2) * 1.10 / (x_max - x_min),
        (y - (y_min + y_max) / 2) * 1.00 / (y_max - y_min), z,
    ) for x, y, z in mesh.vertices]
    return mesh


def _normal(mesh: _Mesh, indices: tuple[int, int, int]) -> Vec3:
    a, b, c = (mesh.vertices[i - 1] for i in indices)
    u = tuple(b[i] - a[i] for i in range(3))
    v = tuple(c[i] - a[i] for i in range(3))
    cross = (
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    )
    length = math.sqrt(sum(value * value for value in cross))
    if length <= 1e-12:
        raise ValueError(f"{mesh.name}: degenerate triangle {indices}")
    return tuple(value / length for value in cross)


def _write_mesh(destination: Path, mesh: _Mesh) -> None:
    if not mesh.vertices or not mesh.faces:
        raise ValueError(f"{mesh.name}: empty mesh")
    if any(not math.isfinite(x) for v in mesh.vertices for x in v):
        raise ValueError(f"{mesh.name}: non-finite vertex")
    for indices, material in mesh.faces:
        if any(i < 1 or i > len(mesh.vertices) for i in indices):
            raise ValueError(f"{mesh.name}: invalid vertex index")
        if material not in mesh.materials:
            raise ValueError(f"{mesh.name}: undefined material {material}")
    normals = [_normal(mesh, indices) for indices, _ in mesh.faces]
    obj = [
        "# Procedural NOMAD visual asset; metres; +Z up; ground origin.",
        "# Collision footprints are separate; foliage is not collision geometry.",
        f"mtllib {mesh.name}.mtl", f"o {mesh.name}", "s off",
    ]
    obj.extend("v {:.8f} {:.8f} {:.8f}".format(*v) for v in mesh.vertices)
    obj.extend("vn {:.8f} {:.8f} {:.8f}".format(*n) for n in normals)
    last_material = None
    for normal_idx, (indices, material) in enumerate(mesh.faces, start=1):
        if material != last_material:
            obj.append(f"usemtl {material}")
            last_material = material
        obj.append("f " + " ".join(f"{i}//{normal_idx}" for i in indices))
    mtl = ["# Opaque diffuse colours only; no textures or transparency."]
    for name, colour in mesh.materials.items():
        mtl.extend((
            f"newmtl {name}",
            "Ka {:.6f} {:.6f} {:.6f}".format(*(0.15 * c for c in colour)),
            "Kd {:.6f} {:.6f} {:.6f}".format(*colour),
            "Ks 0.000000 0.000000 0.000000", "Ns 1.0", "d 1.0", "illum 1", "",
        ))
    (destination / f"{mesh.name}.obj").write_text("\n".join(obj) + "\n", encoding="ascii")
    (destination / f"{mesh.name}.mtl").write_text("\n".join(mtl) + "\n", encoding="ascii")


def generate_models(destination: Path) -> None:
    """Create/replace four deterministic OBJ/MTL pairs in ``destination``.

    pine_tree: 4.5m tall, trunk radius 0.18m, canopy radius 0.9m.
    shrub: 1.0m tall, maximum horizontal radius 0.65m.
    grass_clump: 0.35m tall, maximum horizontal radius 0.3m.
    rock: 1.1m tall, 1.1m x width, 1.0m y width.
    Other files in the destination directory are left untouched.
    """
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    for mesh in (_pine_tree(), _shrub(), _grass_clump(), _rock()):
        _write_mesh(destination, mesh)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="directory for the four OBJ/MTL pairs")
    args = parser.parse_args()
    generate_models(args.output_dir)
    print(f"Generated 4 texture-free OBJ/MTL model pairs in {args.output_dir}")


if __name__ == "__main__":
    main()
