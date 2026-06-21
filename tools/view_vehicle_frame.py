#!/usr/bin/env python3
"""PyVista viewer for TV3 vehicle manifests in the vehicle reference frame.

Plots link poses, COMs, engine mount points, gimbal rotation axes, and joint origins
from a checked-in vehicle JSON manifest. Use this to verify Phase 2 geometry before
promoting measured values.

Modes:
  default       interactive single-view 3D frame with per-engine roll/yaw sliders
  --overview    four-panel overview in an interactive PyVista window
  --save PATH   write a four-panel overview PNG (headless)

Reference frame (from manifest physical_model.reference_frame):
  origin at nozzle_exit_center
  +X forward along airframe
  +Y vehicle right
  +Z vehicle down
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from typing import Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.manifest_geometry import body_size_m, extent_points  # noqa: E402
from tools.manifest_io import load_manifest  # noqa: E402
from tools.pyvista_draw import (  # noqa: E402
    EngineActuatorControl,
    actuator_status_lines,
    build_scene_layers,
    clear_artists,
    draw_computed_thrust,
    engine_actuator_limits,
)
from tools.pyvista_viz import PyVistaScene, apply_dark_theme, import_pyvista, show_plotter  # noqa: E402
from tools.static_preview import render_vehicle_overview  # noqa: E402
from tools.tv3_control_allocator import engines_from_vehicle  # noqa: E402
from tools.tv3_engine_frame import MAX_BUILD_STAGE  # noqa: E402
from tools.viz_common import CAMERA_PRESETS, ENGINE_COLORS  # noqa: E402

DEFAULT_VEHICLE = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"

# Backward-compatible re-exports for callers that still import from view_vehicle_frame
from tools.manifest_geometry import motor_label, summary_text, vec3  # noqa: E402, F401
from tools.manifest_io import engines_from_manifest  # noqa: E402, F401
from tools.pyvista_draw import (  # noqa: E402, F401
    annotate_scene,
    draw_coupled_yaw_axis,
    draw_vector,
)


def run_interactive_viewer(
    manifest: dict,
    body_size: Sequence[float],
    axis_length: float,
    *,
    build_stage: int = MAX_BUILD_STAGE,
) -> None:
    pv = import_pyvista()
    working_manifest = copy.deepcopy(manifest)
    engine_models = engines_from_vehicle(working_manifest)
    controls: list[EngineActuatorControl] = []
    for index, engine in enumerate(engine_models):
        engine_id = working_manifest.get("propulsion", {}).get("engines", [{}])[index].get("id", f"engine_{index}")
        controls.append(
            EngineActuatorControl(
                engine=engine,
                engine_id=str(engine_id),
                color=ENGINE_COLORS[index % len(ENGINE_COLORS)],
                roll_deg=0.0,
                yaw_deg=0.0,
            )
        )

    plotter = pv.Plotter()
    apply_dark_theme(plotter)
    scene = PyVistaScene(plotter)
    build_scene_layers(scene, working_manifest, body_size, axis_length, build_stage=build_stage)
    scene.set_equal_limits(extent_points(working_manifest, body_size))
    elev, azim = CAMERA_PRESETS["iso"]
    scene.set_camera(elev, azim)
    status = plotter.add_text("", position="upper_left", font_size=10)
    selected_index = 0

    def refresh() -> None:
        for control in controls:
            clear_artists(control.thrust_artists)
            control.thrust_artists.clear()
            control.thrust_artists.extend(draw_computed_thrust(scene, control, axis_length))
        lines: list[str] = []
        if controls:
            lines.append(
                f"active engine: {controls[selected_index].engine_id}  (press 1-{len(controls)} to focus status)"
            )
        for control in controls:
            lines.extend(actuator_status_lines(working_manifest, control))
            lines.append("")
        status.SetText(0, "\n".join(lines).rstrip())
        plotter.render()

    def select_engine(index: int) -> None:
        nonlocal selected_index
        selected_index = max(0, min(index, len(controls) - 1))
        refresh()

    if build_stage >= 2 and controls:
        slider_y = 0.04
        for index, control in enumerate(controls):
            limits = engine_actuator_limits(control.engine)

            def make_roll_callback(engine_index: int):
                def on_roll(value: float) -> None:
                    controls[engine_index].roll_deg = float(value)
                    if engine_index == selected_index:
                        refresh()

                return on_roll

            def make_yaw_callback(engine_index: int):
                def on_yaw(value: float) -> None:
                    controls[engine_index].yaw_deg = float(value)
                    if engine_index == selected_index:
                        refresh()

                return on_yaw

            plotter.add_slider_widget(
                make_roll_callback(index),
                [limits["roll_min"], limits["roll_max"]],
                value=0.0,
                title=f"{control.engine_id} roll",
                pointa=(0.02, slider_y),
                pointb=(0.32, slider_y),
            )
            if build_stage >= 3:
                plotter.add_slider_widget(
                    make_yaw_callback(index),
                    [limits["yaw_min"], limits["yaw_max"]],
                    value=0.0,
                    title=f"{control.engine_id} yaw",
                    pointa=(0.34, slider_y),
                    pointb=(0.64, slider_y),
                )
            slider_y += 0.08

        def on_key_press(obj, _event):  # noqa: ANN001
            key = plotter.iren.GetKeySym()
            if key in {"1", "2", "3", "4"}:
                select_engine(int(key) - 1)

        plotter.iren.add_observer("KeyPressEvent", on_key_press)

    refresh()
    show_plotter(plotter)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, default=DEFAULT_VEHICLE, help="Vehicle manifest JSON path")
    parser.add_argument("--body-size", nargs=3, type=float, metavar=("SX", "SY", "SZ"), help="Body box size in meters")
    parser.add_argument("--axis-length", type=float, default=0.12, help="Arrow length for frame and thrust axes (m)")
    parser.add_argument("--save", type=Path, help="Write a four-panel overview PNG to this path (headless)")
    parser.add_argument(
        "--overview",
        action="store_true",
        help="Open the four-panel overview in an interactive PyVista window",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Open the single-view 3D frame with actuator sliders (default when neither --save nor --overview)",
    )
    parser.add_argument(
        "--build-stage",
        type=int,
        choices=(1, 2, 3),
        default=MAX_BUILD_STAGE,
        help="Show engine-frame axes incrementally: 1=thrust ref, 2=+secondary, 3=+primary (default)",
    )
    args = parser.parse_args()

    vehicle_path = args.vehicle if args.vehicle.is_absolute() else REPO_ROOT / args.vehicle
    manifest = load_manifest(vehicle_path)
    body_size = body_size_m(manifest, args.body_size)

    if args.save is not None:
        output = args.save if args.save.is_absolute() else REPO_ROOT / args.save
        render_vehicle_overview(
            manifest,
            body_size,
            args.axis_length,
            build_stage=args.build_stage,
            output=output,
            interactive=False,
        )
        return

    if args.overview:
        render_vehicle_overview(
            manifest,
            body_size,
            args.axis_length,
            build_stage=args.build_stage,
            output=None,
            interactive=True,
        )
        return

    run_interactive_viewer(manifest, body_size, args.axis_length, build_stage=args.build_stage)


if __name__ == "__main__":
    main()
