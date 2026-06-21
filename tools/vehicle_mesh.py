"""Load and transform TV3 vehicle meshes for PyVista and Rerun."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from tools.ulog_replay_common import ned_to_plot_xyz, rotation_matrix_from_quat
from tools.viz_common import resolve_mesh_path


def load_obj_mesh(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    with path.open(encoding="utf-8") as handle:
        for raw in handle:
            line = raw.strip()
            if line.startswith("v "):
                parts = line.split()
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("f "):
                indices = []
                for token in line.split()[1:]:
                    indices.append(int(token.split("/")[0]) - 1)
                if len(indices) >= 3:
                    faces.append((indices[0], indices[1], indices[2]))
    if not vertices or not faces:
        raise ValueError(f"OBJ mesh is empty: {path}")
    return np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int32)


def transform_body_vertices_to_plot(
    vertices: np.ndarray,
    quaternion: Sequence[float],
    position_ned: Sequence[float],
) -> np.ndarray:
    rotation = rotation_matrix_from_quat(quaternion)
    position = np.asarray(position_ned, dtype=np.float64)
    transformed = []
    for vertex in vertices:
        world_ned = rotation @ vertex + position
        transformed.append(ned_to_plot_xyz(float(world_ned[0]), float(world_ned[1]), float(world_ned[2])))
    return np.asarray(transformed, dtype=np.float64)


def load_pyvista_mesh(path: Path):
    from tools.pyvista_viz import import_pyvista

    pv = import_pyvista()
    return pv.read(str(path))


def add_vehicle_mesh_to_scene(scene, path: Path, *, color: str = "#8a8a90", opacity: float = 0.92) -> list:
    from tools.pyvista_viz import hex_to_rgb, import_pyvista

    pv = import_pyvista()
    mesh = pv.read(str(path))
    actor = scene.plotter.add_mesh(mesh, color=hex_to_rgb(color), opacity=opacity, smooth_shading=True)
    scene.actors.append(actor)
    return [actor]


def add_transformed_vehicle_mesh(
    scene,
    path: Path,
    *,
    quaternion: Sequence[float],
    position_ned: Sequence[float],
    color: str = "#8a8a90",
    opacity: float = 0.92,
) -> list:
    from tools.pyvista_viz import hex_to_rgb, import_pyvista

    pv = import_pyvista()
    mesh = pv.read(str(path)).copy()
    mesh.points = transform_body_vertices_to_plot(
        np.asarray(mesh.points, dtype=np.float64),
        quaternion,
        position_ned,
    )
    actor = scene.plotter.add_mesh(mesh, color=hex_to_rgb(color), opacity=opacity, smooth_shading=True)
    scene.actors.append(actor)
    return [actor]


def rerun_mesh_from_path(path: Path | None = None, manifest: dict | None = None):
    mesh_path = resolve_mesh_path(manifest, path)
    vertices, faces = load_obj_mesh(mesh_path)
    import rerun as rr

    return rr.Mesh3D(
        vertex_positions=vertices.astype(np.float32),
        triangle_indices=faces.astype(np.uint32),
        vertex_colors=[[140, 140, 145] for _ in range(len(vertices))],
    )


def resolve_vehicle_mesh(manifest: dict | None = None, override: Path | None = None) -> Path:
    return resolve_mesh_path(manifest, override)