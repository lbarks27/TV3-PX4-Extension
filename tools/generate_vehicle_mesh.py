#!/usr/bin/env python3
"""Generate a simple OBJ mesh from a TV3 vehicle manifest for Hawkeye/PyVista/Rerun."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.manifest_geometry import body_size_m, engines_from_manifest, vec3

DEFAULT_VEHICLE = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"
DEFAULT_OUTPUT = REPO_ROOT / "assets/meshes/tv3_lander_v1.obj"


def box_triangles(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    cx, cy, cz = center
    sx, sy, sz = size
    xmin, xmax = cx - sx / 2, cx + sx / 2
    ymin, ymax = cy - sy / 2, cy + sy / 2
    zmin, zmax = cz - sz / 2, cz + sz / 2
    corners = [
        (xmin, ymin, zmin),
        (xmax, ymin, zmin),
        (xmax, ymax, zmin),
        (xmin, ymax, zmin),
        (xmin, ymin, zmax),
        (xmax, ymin, zmax),
        (xmax, ymax, zmax),
        (xmin, ymax, zmax),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (2, 6, 7),
        (2, 7, 3),
        (0, 3, 7),
        (0, 7, 4),
        (1, 5, 6),
        (1, 6, 2),
    ]
    return corners, faces


def cylinder_triangles(
    center: tuple[float, float, float],
    *,
    radius: float,
    height: float,
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0),
    segments: int = 12,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / norm, ay / norm, az / norm
    if abs(az) < 0.9:
        ref = (0.0, 0.0, 1.0)
    else:
        ref = (0.0, 1.0, 0.0)
    u = (
        ay * ref[2] - az * ref[1],
        az * ref[0] - ax * ref[2],
        ax * ref[1] - ay * ref[0],
    )
    u_len = math.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]) or 1.0
    u = (u[0] / u_len, u[1] / u_len, u[2] / u_len)
    v = (
        ay * u[2] - az * u[1],
        az * u[0] - ax * u[2],
        ax * u[1] - ay * u[0],
    )

    cx, cy, cz = center
    half = height * 0.5
    base = (cx - ax * half, cy - ay * half, cz - az * half)
    top = (cx + ax * half, cy + ay * half, cz + az * half)

    ring_base: list[tuple[float, float, float]] = []
    ring_top: list[tuple[float, float, float]] = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        offset = (
            radius * (cos_a * u[0] + sin_a * v[0]),
            radius * (cos_a * u[1] + sin_a * v[1]),
            radius * (cos_a * u[2] + sin_a * v[2]),
        )
        ring_base.append((base[0] + offset[0], base[1] + offset[1], base[2] + offset[2]))
        ring_top.append((top[0] + offset[0], top[1] + offset[1], top[2] + offset[2]))

    vertices = ring_base + ring_top + [base, top]
    base_center = len(vertices) - 2
    top_center = len(vertices) - 1
    faces: list[tuple[int, int, int]] = []
    for index in range(segments):
        next_index = (index + 1) % segments
        b0, b1 = index, next_index
        t0, t1 = index + segments, next_index + segments
        faces.append((b0, b1, t1))
        faces.append((b0, t1, t0))
        faces.append((base_center, b1, b0))
        faces.append((top_center, t0, t1))
    return vertices, faces


def merge_meshes(
    parts: list[tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]],
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for part_vertices, part_faces in parts:
        offset = len(vertices)
        vertices.extend(part_vertices)
        faces.extend((a + offset, b + offset, c + offset) for a, b, c in part_faces)
    return vertices, faces


def write_obj(
    path: Path,
    vertices: list[tuple[float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    material_name: str = "tv3_body",
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mtl_path = path.with_suffix(".mtl")
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write(f"mtllib {mtl_path.name}\n")
        handle.write(f"o {path.stem}\n")
        handle.write(f"usemtl {material_name}\n")
        for x, y, z in vertices:
            handle.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c in faces:
            handle.write(f"f {a + 1} {b + 1} {c + 1}\n")
    with mtl_path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("newmtl tv3_body\n")
        handle.write("Kd 0.55 0.55 0.58\n")
        handle.write("Ka 0.20 0.20 0.22\n")
        handle.write("Ks 0.35 0.35 0.38\n")
        handle.write("Ns 32.0\n")
        handle.write("newmtl tv3_engine\n")
        handle.write("Kd 0.85 0.45 0.12\n")
        handle.write("Ka 0.30 0.15 0.05\n")
        handle.write("Ks 0.40 0.25 0.10\n")
        handle.write("Ns 48.0\n")


def build_manifest_mesh(manifest: dict) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    body = manifest["vehicle"]
    body_size = body_size_m(manifest, None)
    body_com = (body["body_com_x_m"], 0.0, 0.0)
    parts = [box_triangles(body_com, body_size)]
    for engine in engines_from_manifest(manifest):
        position = vec3(engine["position_m"])
        thrust_axis = vec3(engine.get("thrust_axis", [1.0, 0.0, 0.0]), (1.0, 0.0, 0.0))
        parts.append(
            cylinder_triangles(
                position,
                radius=0.028,
                height=0.11,
                axis=thrust_axis,
            )
        )
    return merge_meshes(parts)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, default=DEFAULT_VEHICLE)
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    vehicle_path = args.vehicle if args.vehicle.is_absolute() else REPO_ROOT / args.vehicle
    output_path = args.output if args.output.is_absolute() else REPO_ROOT / args.output
    manifest = json.loads(vehicle_path.read_text())
    vertices, faces = build_manifest_mesh(manifest)
    write_obj(output_path, vertices, faces)
    print(f"wrote {output_path} ({len(vertices)} vertices, {len(faces)} faces)")


if __name__ == "__main__":
    main()