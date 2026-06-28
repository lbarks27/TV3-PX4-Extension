"""PyVista overview renderer for TV3 vehicle manifests."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

from tools.pyvista_viz import PyVistaScene, apply_dark_theme, import_pyvista, save_plotter, show_plotter


CAMERA_PRESETS = {
    "iso": (24, -58),
    "top": (90, -90),
    "side": (0, -90),
    "front": (0, 180),
}
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