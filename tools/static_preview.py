"""PyVista interactive 3D scenes and optional PNG exports for vehicle config and flight paths."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

from tools.manifest_geometry import body_size_m, extent_points, summary_text, vec3
from tools.manifest_io import engines_from_manifest
from tools.pyvista_draw import annotate_scene, build_scene_layers
from tools.pyvista_viz import PyVistaScene, apply_dark_theme, import_pyvista, save_plotter, show_plotter
from tools.scene_builders import body_axis_vectors_plot, trajectory_path_points
from tools.ulog_replay_common import ned_to_plot_xyz
from tools.vehicle_mesh import add_transformed_vehicle_mesh, resolve_vehicle_mesh
from tools.viz_common import CAMERA_PRESETS


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
    manifest: dict | None = None,
    mesh_path: Path | None = None,
) -> None:
    pv = import_pyvista()
    plotter = pv.Plotter(off_screen=not interactive and output is not None)
    apply_dark_theme(plotter)
    scene = PyVistaScene(plotter)

    path = trajectory_path_points(frames)
    for start, end in zip(path, path[1:]):
        scene.add_line(start, end, color="#9aa0a6", width=2.0, opacity=0.7)

    index = len(frames) - 1 if frame_index is None else max(0, min(frame_index, len(frames) - 1))
    frame = frames[index]
    vehicle = ned_to_plot_xyz(*frame.position_ned)

    try:
        mesh_file = resolve_vehicle_mesh(manifest, mesh_path)
        add_transformed_vehicle_mesh(
            scene,
            mesh_file,
            quaternion=frame.quaternion,
            position_ned=frame.position_ned,
        )
    except SystemExit:
        scene.add_points([vehicle], color="#ff7f0e", size=16.0, labels=["vehicle"])

    for direction, color, label in body_axis_vectors_plot(frame.quaternion, axis_length):
        scene.add_vector(vehicle, direction, axis_length, color=color, label=label)

    if frame.setpoint_ned is not None:
        scene.add_points([ned_to_plot_xyz(*frame.setpoint_ned)], color="#6a9bd1", size=12.0, labels=["setpoint"])
    if frame.target_ned is not None:
        scene.add_points([ned_to_plot_xyz(*frame.target_ned)], color="#b08cd1", size=14.0, labels=["target"])

    scene.set_equal_limits(path + [vehicle])
    elev, azim = CAMERA_PRESETS.get(camera, CAMERA_PRESETS["overview"])
    scene.set_camera(elev, azim)
    plotter.add_text(
        f"Trajectory preview  t={frame.time_s:.2f}s  (orbit/zoom; use --rerun to scrub)",
        font_size=10,
    )

    if output is not None:
        save_plotter(plotter, output)
        print(f"wrote {output}")
    if interactive:
        show_plotter(plotter)
    plotter.close()


def render_engine_preview(
    frame: Any,
    engines: Sequence[Any],
    manifest: dict,
    output: Path | None,
    *,
    axis_length: float,
    camera: str = "forward_up",
    interactive: bool = False,
    mesh_path: Path | None = None,
) -> None:
    from tools.plot_ulog_engines import draw_dynamic_frame  # noqa: WPS433

    pv = import_pyvista()
    plotter = pv.Plotter(off_screen=not interactive and output is not None)
    apply_dark_theme(plotter)
    scene = PyVistaScene(plotter)
    body_size = body_size_m(manifest, None)
    build_scene_layers(scene, manifest, body_size, axis_length, build_stage=3)
    try:
        mesh_file = resolve_vehicle_mesh(manifest, mesh_path)
        add_transformed_vehicle_mesh(scene, mesh_file, quaternion=frame.quaternion, position_ned=(0.0, 0.0, 0.0))
    except SystemExit:
        pass
    draw_dynamic_frame(scene, frame, engines, manifest, axis_length=axis_length, build_stage=3)

    points = [vec3(engine["position_m"]) for engine in engines_from_manifest(manifest)] + [(0.0, 0.0, 0.0)]
    scene.set_equal_limits(points, pad=0.12)
    elev, azim = CAMERA_PRESETS.get(camera, CAMERA_PRESETS["forward_up"])
    scene.set_camera(elev, azim)
    plotter.add_text(
        f"Engine preview  t={frame.time_s:.2f}s  (orbit/zoom; use --rerun to scrub)",
        font_size=10,
    )

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
    mesh_path: Path | None = None,
) -> None:
    from tools.vehicle_mesh import add_vehicle_mesh_to_scene, resolve_vehicle_mesh  # noqa: WPS433

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
    mesh_file = None
    try:
        mesh_file = resolve_vehicle_mesh(manifest, mesh_path)
    except SystemExit:
        mesh_file = None

    for row, col, preset, title in views:
        plotter.subplot(row, col)
        apply_dark_theme(plotter)
        scene = PyVistaScene(plotter)
        annotate_scene(scene, manifest, body_size, axis_length, build_stage=build_stage)
        if mesh_file is not None:
            add_vehicle_mesh_to_scene(scene, mesh_file, opacity=0.55)
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