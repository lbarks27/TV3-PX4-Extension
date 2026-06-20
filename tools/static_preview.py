"""PyVista interactive 3D scenes and optional PNG exports for vehicle config and flight paths."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from tools.pyvista_viz import PyVistaScene, SceneLayers, apply_dark_theme, import_pyvista, save_plotter, show_plotter
from tools.ulog_replay_common import body_axes_in_world, ned_to_plot_xyz
from tools.tv3_control_allocator import EngineGeometry, plant_thrust_direction

ENGINE_COLORS = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")
CAMERA_PRESETS = {
    "iso": (24, -58),
    "top": (90, -90),
    "side": (0, -90),
    "front": (0, 180),
    "forward_up": (-90, 0),
    "overview": (25, -55),
    "track": (20, -70),
}


def vec3(values: Sequence[float], default: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not values or len(values) < 3:
        return default
    return float(values[0]), float(values[1]), float(values[2])


def render_trajectory_preview(
    frames: Sequence[Any],
    output: Path | None,
    *,
    axis_length: float,
    camera: str = "overview",
    interactive: bool = False,
    frame_index: int | None = None,
) -> None:
    pv = import_pyvista()
    plotter = pv.Plotter(off_screen=not interactive and output is not None)
    apply_dark_theme(plotter)
    scene = PyVistaScene(plotter)

    path = [ned_to_plot_xyz(*frame.position_ned) for frame in frames]
    for start, end in zip(path, path[1:]):
        scene.add_line(start, end, color="#9aa0a6", width=2.0, opacity=0.7)

    index = len(frames) - 1 if frame_index is None else max(0, min(frame_index, len(frames) - 1))
    frame = frames[index]
    vehicle = ned_to_plot_xyz(*frame.position_ned)
    scene.add_points([vehicle], color="#ff7f0e", size=16.0, labels=["vehicle"])

    forward, right, down = body_axes_in_world(frame.quaternion)
    for direction, color, label in (
        (forward, "#ff7f0e", "F"),
        (right, "#2ca02c", "R"),
        (down, "#d62728", "D"),
    ):
        direction_plot = (float(direction[0]), float(direction[1]), float(-direction[2]))
        scene.add_vector(vehicle, direction_plot, axis_length, color=color, label=label)

    if frame.setpoint_ned is not None:
        scene.add_points([ned_to_plot_xyz(*frame.setpoint_ned)], color="#6a9bd1", size=12.0, labels=["setpoint"])
    if frame.target_ned is not None:
        scene.add_points([ned_to_plot_xyz(*frame.target_ned)], color="#b08cd1", size=14.0, labels=["target"])

    scene.set_equal_limits(path + [vehicle])
    elev, azim = CAMERA_PRESETS.get(camera, CAMERA_PRESETS["overview"])
    scene.set_camera(elev, azim)
    plotter.add_text(f"Trajectory preview  t={frame.time_s:.2f}s  (orbit/zoom; no timeline)", font_size=10)

    if output is not None:
        save_plotter(plotter, output)
        print(f"wrote {output}")
    if interactive:
        show_plotter(plotter)
    plotter.close()


def render_engine_preview(
    frame: Any,
    engines: Sequence[EngineGeometry],
    manifest: dict,
    output: Path | None,
    *,
    axis_length: float,
    camera: str = "forward_up",
    interactive: bool = False,
) -> None:
    from tools.plot_ulog_engines import draw_dynamic_frame  # noqa: WPS433
    from tools.view_vehicle_frame import body_size_m, build_scene_layers, engines_from_manifest  # noqa: WPS433

    pv = import_pyvista()
    plotter = pv.Plotter(off_screen=not interactive and output is not None)
    apply_dark_theme(plotter)
    scene = PyVistaScene(plotter)
    body_size = body_size_m(manifest, None)
    build_scene_layers(scene, manifest, body_size, axis_length, build_stage=3)
    draw_dynamic_frame(scene, frame, engines, manifest, axis_length=axis_length, build_stage=3)

    points = [vec3(engine["position_m"]) for engine in engines_from_manifest(manifest)] + [(0.0, 0.0, 0.0)]
    scene.set_equal_limits(points, pad=0.12)
    elev, azim = CAMERA_PRESETS.get(camera, CAMERA_PRESETS["forward_up"])
    scene.set_camera(elev, azim)
    plotter.add_text(f"Engine preview  t={frame.time_s:.2f}s  (orbit/zoom; no timeline)", font_size=10)

    if output is not None:
        save_plotter(plotter, output)
        print(f"wrote {output}")
    if interactive:
        show_plotter(plotter)
    plotter.close()


def render_vehicle_overview(
    manifest: dict,
    body_size: Sequence[float],
    axis_length: float,
    *,
    build_stage: int,
    output: Path | None,
    interactive: bool = False,
) -> None:
    from tools.view_vehicle_frame import annotate_scene, extent_points, summary_text  # noqa: WPS433

    pv = import_pyvista()
    plotter = pv.Plotter(shape=(2, 2), off_screen=not interactive and output is not None)
    apply_dark_theme(plotter)

    views = (
        (0, 0, "iso", "3D vehicle frame"),
        (0, 1, "top", "Top view"),
        (1, 0, "side", "Side view"),
        (1, 1, "front", "Front view"),
    )
    points = extent_points(manifest, body_size)
    for row, col, preset, title in views:
        plotter.subplot(row, col)
        apply_dark_theme(plotter)
        scene = PyVistaScene(plotter)
        annotate_scene(scene, manifest, body_size, axis_length, build_stage=build_stage)
        scene.set_equal_limits(points)
        elev, azim = CAMERA_PRESETS[preset]
        scene.set_camera(elev, azim)
        plotter.add_text(title, font_size=9)

    plotter.subplot(0, 0)
    plotter.add_text(summary_text(manifest, build_stage=build_stage), font_size=8)

    if output is not None:
        save_plotter(plotter, output, window_size=(1800, 1400))
        print(f"wrote {output}")
    if interactive:
        show_plotter(plotter)
    plotter.close()