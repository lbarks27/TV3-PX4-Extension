"""Shared PyVista helpers for TV3 interactive 3D visualization and PNG export."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np

MISSING_DEP_MSG = "missing dependency: install pyvista with `python3 -m pip install -r requirements-viz.txt`"

DARK_BG = (0.08, 0.08, 0.08)
DARK_PANEL = (0.12, 0.12, 0.12)
DARK_TEXT = (0.91, 0.91, 0.91)


def import_pyvista():
    try:
        import pyvista as pv
    except ImportError as exc:
        raise SystemExit(MISSING_DEP_MSG) from exc
    pv.set_plot_theme("document")
    return pv


def hex_to_rgb(color: str) -> tuple[float, float, float]:
    color = color.lstrip("#")
    return tuple(int(color[index : index + 2], 16) / 255.0 for index in (0, 2, 4))


ActorList = list[object]


@dataclass
class SceneLayers:
    groups: dict[str, ActorList] = field(default_factory=dict)

    def add(self, group: str, actors: ActorList) -> None:
        if not actors:
            return
        self.groups.setdefault(group, []).extend(actors)

    def labels(self) -> list[str]:
        return list(self.groups.keys())

    def set_visible(self, group: str, visible: bool) -> None:
        for actor in self.groups.get(group, []):
            actor.SetVisibility(visible)


@dataclass
class PyVistaScene:
    plotter: object
    actors: ActorList = field(default_factory=list)

    def add_line(
        self,
        start: Sequence[float],
        end: Sequence[float],
        *,
        color: str = "#ffffff",
        width: float = 2.0,
        opacity: float = 1.0,
    ) -> ActorList:
        pv = import_pyvista()
        mesh = pv.Line(np.asarray(start, dtype=float), np.asarray(end, dtype=float))
        actor = self.plotter.add_mesh(mesh, color=hex_to_rgb(color), line_width=width, opacity=opacity)
        self.actors.append(actor)
        return [actor]

    def add_vector(
        self,
        origin: Sequence[float],
        direction: Sequence[float],
        length: float,
        *,
        color: str = "#ffffff",
        width: float = 2.0,
        opacity: float = 1.0,
        label: str | None = None,
    ) -> ActorList:
        ox, oy, oz = (float(origin[0]), float(origin[1]), float(origin[2]))
        dx, dy, dz = (float(direction[0]), float(direction[1]), float(direction[2]))
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm <= 1e-9:
            return []
        scale = length / norm
        end = (ox + dx * scale, oy + dy * scale, oz + dz * scale)
        actors = self.add_line((ox, oy, oz), end, color=color, width=width, opacity=opacity)
        if label:
            actors.extend(self.add_label(end, label, color=color))
        return actors

    def add_points(
        self,
        points: Sequence[Sequence[float]],
        *,
        color: str = "#ffffff",
        size: float = 12.0,
        labels: Sequence[str] | None = None,
    ) -> ActorList:
        pv = import_pyvista()
        cloud = pv.PolyData(np.asarray(points, dtype=float))
        actor = self.plotter.add_mesh(
            cloud,
            color=hex_to_rgb(color),
            point_size=size,
            render_points_as_spheres=True,
        )
        self.actors.append(actor)
        actors: ActorList = [actor]
        if labels:
            for point, text in zip(points, labels):
                actors.extend(self.add_label(point, text, color=color))
        return actors

    def add_label(self, point: Sequence[float], text: str, *, color: str = "#e8e8e8") -> ActorList:
        actor = self.plotter.add_point_labels(
            [np.asarray(point, dtype=float)],
            [text],
            text_color=hex_to_rgb(color),
            font_size=10,
            shape=None,
        )
        self.actors.append(actor)
        return [actor]

    def add_wireframe_box(
        self,
        center: Sequence[float],
        size: Sequence[float],
        *,
        color: str = "#7f7f7f",
        label: str | None = None,
    ) -> ActorList:
        pv = import_pyvista()
        cx, cy, cz = (float(center[0]), float(center[1]), float(center[2]))
        sx, sy, sz = (float(size[0]), float(size[1]), float(size[2]))
        box = pv.Box(bounds=(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, cz - sz / 2, cz + sz / 2))
        actor = self.plotter.add_mesh(box, style="wireframe", color=hex_to_rgb(color), line_width=1.5)
        self.actors.append(actor)
        actors: ActorList = [actor]
        if label:
            artists = self.add_label((cx, cy, cz), label, color=color)
            actors.extend(artists)
        return actors

    def add_surface(
        self,
        mesh_x: np.ndarray,
        mesh_y: np.ndarray,
        mesh_z: np.ndarray,
        *,
        color: str,
        opacity: float = 0.14,
    ) -> ActorList:
        pv = import_pyvista()
        grid = pv.StructuredGrid(mesh_x, mesh_y, mesh_z)
        actor = self.plotter.add_mesh(grid, color=hex_to_rgb(color), opacity=opacity, smooth_shading=True)
        self.actors.append(actor)
        return [actor]

    def set_camera(self, elev: float, azim: float) -> None:
        self.plotter.camera.elevation = elev
        self.plotter.camera.azimuth = azim

    def set_equal_limits(self, points: Iterable[Sequence[float]], *, pad: float = 0.08) -> tuple[tuple[float, float, float], float]:
        xs = [float(p[0]) for p in points]
        ys = [float(p[1]) for p in points]
        zs = [float(p[2]) for p in points]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        zmin, zmax = min(zs), max(zs)
        span = max(xmax - xmin, ymax - ymin, zmax - zmin, 0.2)
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        cz = 0.5 * (zmin + zmax)
        half = 0.5 * span + pad
        self.plotter.set_focus((cx, cy, cz))
        self.plotter.camera.zoom(1.0)
        self.plotter.reset_camera(
            bounds=(cx - half, cx + half, cy - half, cy + half, cz - half, cz + half),
            render=False,
        )
        return (cx, cy, cz), half


def apply_dark_theme(plotter) -> None:
    plotter.set_background(DARK_BG)
    plotter.show_axes()


def save_plotter(plotter, output, *, window_size: tuple[int, int] = (1600, 1200)) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    plotter.screenshot(str(output), window_size=window_size, return_img=False)


def show_plotter(plotter) -> None:
    plotter.show()