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
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.tv3_control_allocator import (  # noqa: E402
    EngineGeometry,
    coupled_yaw_axis,
    engines_from_vehicle,
    plant_thrust_direction,
)
from tools.tv3_engine_frame import MAX_BUILD_STAGE, build_engine_frame_axes  # noqa: E402
from tools.tv3_motor_catalog import (  # noqa: E402
    load_motor_catalog,
    resolve_motor_id,
    thrust_basis_from_manifest,
)
from tools.pyvista_viz import (  # noqa: E402
    ActorList,
    PyVistaScene,
    SceneLayers,
    apply_dark_theme,
    import_pyvista,
    save_plotter,
    show_plotter,
)
from tools.static_preview import render_vehicle_overview  # noqa: E402

DEFAULT_VEHICLE = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"

# Deprecated Gazebo fallback box for tv3_lander_v1 static structure.
DEFAULT_LANDER_BODY_SIZE_M = (0.65, 0.25, 0.25)


def load_manifest(path: Path) -> dict:
    return json.loads(path.read_text())


def vec3(values: Sequence[float], default: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if not values or len(values) < 3:
        return default
    return float(values[0]), float(values[1]), float(values[2])


def link_pose_xyz(link: dict) -> tuple[float, float, float]:
    pose = link.get("pose_m", [0.0, 0.0, 0.0])
    if len(pose) >= 3:
        return vec3(pose[:3])
    return (0.0, 0.0, 0.0)


def link_world_com(link: dict) -> tuple[float, float, float]:
    origin = link_pose_xyz(link)
    com = vec3(link.get("com_m", [0.0, 0.0, 0.0]))
    return origin[0] + com[0], origin[1] + com[1], origin[2] + com[2]


def engines_from_manifest(manifest: dict) -> list[dict]:
    engines = manifest.get("propulsion", {}).get("engines")
    if engines:
        return engines

    body = manifest["vehicle"]
    return [
        {
            "id": "engine_0",
            "motor_index": manifest.get("motor_selection", {}).get("index", 0),
            "load_cell_channel": manifest.get("hardware", {}).get("load_cell", {}).get("adc_channel", 0),
            "position_m": [body["motor_com_x_m"], 0.0, 0.0],
            "thrust_axis": [1.0, 0.0, 0.0],
            "roll_axis": [0.0, -1.0, 0.0],
            "yaw_axis": [0.0, 0.0, -1.0],
            "gimbal": {
                "roll_max_deg": body["tvc_max_deg"],
                "yaw_max_deg": body["tvc_max_deg"],
            },
        }
    ]


def body_size_m(manifest: dict, override: Sequence[float] | None) -> tuple[float, float, float]:
    if override is not None:
        return vec3(override, DEFAULT_LANDER_BODY_SIZE_M)

    for link in manifest.get("physical_model", {}).get("links", []) or []:
        if link.get("id") == "body" and "size_m" in link:
            return vec3(link["size_m"], DEFAULT_LANDER_BODY_SIZE_M)

    if manifest.get("name") == "tv3_lander_v1":
        return DEFAULT_LANDER_BODY_SIZE_M
    return (0.35, 0.12, 0.12)


def draw_axes(scene: PyVistaScene, origin: tuple[float, float, float], length: float, label_prefix: str = "") -> ActorList:
    ox, oy, oz = origin
    artists: ActorList = []
    specs = (
        ("+X fwd", (length, 0.0, 0.0), "#d62728"),
        ("+Y right", (0.0, length, 0.0), "#2ca02c"),
        ("+Z down", (0.0, 0.0, length), "#1f77b4"),
    )
    for name, direction, color in specs:
        artists.extend(scene.add_vector((ox, oy, oz), direction, length, color=color, label=f"{label_prefix}{name}"))
    return artists


def draw_vector(
    scene: PyVistaScene,
    origin: Sequence[float],
    direction: Sequence[float],
    length: float,
    color: str,
    label: str | None = None,
    *,
    alpha: float = 1.0,
) -> ActorList:
    ox, oy, oz = vec3(origin)
    dx, dy, dz = vec3(direction, (1.0, 0.0, 0.0))
    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    if norm <= 1e-9:
        return []
    return scene.add_vector((ox, oy, oz), (dx, dy, dz), length, color=color, label=label, opacity=alpha)


def draw_box_wireframe(
    scene: PyVistaScene, center: Sequence[float], size: Sequence[float], color: str, label: str | None = None
) -> ActorList:
    return scene.add_wireframe_box(center, size, color=color, label=label)


def draw_marker(scene: PyVistaScene, point: Sequence[float], color: str, label: str, size: float = 36.0) -> ActorList:
    x, y, z = vec3(point)
    point_size = max(8.0, math.sqrt(size) * 2.0)
    return scene.add_points([(x, y, z)], color=color, size=point_size, labels=[label])


def extent_points(manifest: dict, body_size: Sequence[float]) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]

    body = manifest["vehicle"]
    points.append((body["body_com_x_m"], 0.0, 0.0))
    points.append((body["motor_com_x_m"], 0.0, 0.0))

    bx, by, bz = body["body_com_x_m"], 0.0, 0.0
    sx, sy, sz = vec3(body_size)
    points.extend(
        [
            (bx - sx / 2.0, by - sy / 2.0, bz - sz / 2.0),
            (bx + sx / 2.0, by + sy / 2.0, bz + sz / 2.0),
        ]
    )

    for link in manifest.get("physical_model", {}).get("links", []) or []:
        points.append(link_pose_xyz(link))
        points.append(link_world_com(link))

    for engine in engines_from_manifest(manifest):
        points.append(vec3(engine["position_m"]))

    for joint in manifest.get("physical_model", {}).get("joints", []) or []:
        points.append(vec3(joint.get("origin_m", [0.0, 0.0, 0.0])))

    return points


def set_equal_limits(scene: PyVistaScene, points: Iterable[tuple[float, float, float]], pad: float = 0.08) -> tuple[tuple[float, float, float], float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    span = max(xmax - xmin, ymax - ymin, zmax - zmin, 0.2)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)
    half = 0.5 * span + pad
    apply_zoom_limits(scene, (cx, cy, cz), half)
    return (cx, cy, cz), half


def apply_zoom_limits(
    scene: PyVistaScene,
    center: tuple[float, float, float],
    half_span: float,
    *,
    min_half: float = 0.02,
    max_half: float = 8.0,
) -> float:
    cx, cy, cz = center
    half = max(min_half, min(max_half, half_span))
    scene.plotter.reset_camera((cx - half, cx + half, cy - half, cy + half, cz - half, cz + half))
    return half


def zoom_limits(ax, center: tuple[float, float, float], half_span: float, factor: float) -> float:
    """Scale the view radius. factor < 1 zooms in; factor > 1 zooms out."""
    return apply_zoom_limits(ax, center, half_span * factor)


def build_scene_layers(
    scene: PyVistaScene,
    manifest: dict,
    body_size: Sequence[float],
    axis_length: float,
    *,
    build_stage: int = MAX_BUILD_STAGE,
) -> SceneLayers:
    body = manifest["vehicle"]
    layers = SceneLayers()

    layers.add("reference frame", draw_axes(scene, (0.0, 0.0, 0.0), axis_length))
    layers.add("origin", draw_marker(scene, (0.0, 0.0, 0.0), "#111111", "origin", size=24.0))

    body_com = (body["body_com_x_m"], 0.0, 0.0)
    layers.add("body box", draw_box_wireframe(scene, body_com, body_size, "#7f7f7f", "body COM"))

    for link in manifest.get("physical_model", {}).get("links", []) or []:
        link_id = str(link.get("id", "link"))
        origin = link_pose_xyz(link)
        if link_id == "body":
            continue
        layers.add("links", draw_marker(scene, origin, "#9467bd", f"{link_id} origin", size=28.0))
        com = link_world_com(link)
        if com != origin:
            layers.add("links", draw_marker(scene, com, "#8c564b", f"{link_id} COM", size=20.0))

    engine_colors = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")
    for index, engine in enumerate(engines_from_manifest(manifest)):
        color = engine_colors[index % len(engine_colors)]
        engine_id = engine.get("id", f"engine_{index}")
        position = vec3(engine["position_m"])
        channel = engine.get("load_cell_channel", "?")
        layers.add("engines", draw_marker(scene, position, color, f"{engine_id} (LC {channel})", size=48.0))
        frame = build_engine_frame_axes(position, build_stage=build_stage)
        if build_stage >= 1:
            layers.add(
                "thrust ref",
                draw_vector(scene, position, frame.thrust_axis, axis_length * 0.9, color, "thrust ref"),
            )
        if build_stage >= 2:
            layers.add(
                "primary axis",
                draw_vector(scene, position, frame.primary_axis, axis_length * 0.8, "#bcbd22", "primary"),
            )
        if build_stage >= 3:
            layers.add(
                "secondary axis",
                draw_vector(scene, position, frame.secondary_axis, axis_length * 0.8, "#17becf", "secondary"),
            )

    for joint in manifest.get("physical_model", {}).get("joints", []) or []:
        origin = vec3(joint.get("origin_m", [0.0, 0.0, 0.0]))
        joint_id = str(joint.get("id", "joint"))
        layers.add("joints", draw_marker(scene, origin, "#2ca02c", joint_id, size=18.0))
        layers.add("joints", draw_vector(scene, origin, joint.get("axis", [0.0, 0.0, 1.0]), axis_length * 0.5, "#98df8a", None))

    return layers


def annotate_scene(
    scene: PyVistaScene,
    manifest: dict,
    body_size: Sequence[float],
    axis_length: float,
    *,
    build_stage: int = MAX_BUILD_STAGE,
) -> SceneLayers:
    return build_scene_layers(scene, manifest, body_size, axis_length, build_stage=build_stage)


def motor_label(manifest: dict, engine: dict) -> str:
    motor_selection = manifest.get("motor_selection", {})
    catalog_source = motor_selection.get("catalog_source")
    motor_id = resolve_motor_id(engine, motor_selection)
    if catalog_source:
        entry = load_motor_catalog(str(catalog_source)).get(motor_id)
        if entry is not None:
            return entry.label
    return motor_id or "unassigned"


def summary_text(manifest: dict, *, build_stage: int = MAX_BUILD_STAGE) -> str:
    body = manifest["vehicle"]
    engines = engines_from_manifest(manifest)
    frame = manifest.get("physical_model", {}).get("reference_frame", {})
    stage_labels = {
        1: "stage 1: thrust ref only",
        2: "stage 2: + primary axis (mount->origin)",
        3: "stage 3: + secondary axis",
    }
    lines = [
        f"{manifest.get('name', 'vehicle')} — vehicle frame",
        f"axis build: {stage_labels.get(build_stage, f'stage {build_stage}')}",
        f"origin: {frame.get('origin', 'nozzle_exit_center')}",
        f"body mass: {body['body_mass_kg']} kg @ x={body['body_com_x_m']} m",
        f"motor wet mass: {body['motor_loaded_mass_kg']} kg x {len(engines)}",
        f"splay max: {body['tvc_max_deg']} deg, slew: {body['tvc_slew_dps']} deg/s",
        f"allocator ref thrust: {body['ca_reference_thrust_n']} N",
    ]
    motor_selection = manifest.get("motor_selection", {})
    basis = thrust_basis_from_manifest(motor_selection)
    if motor_selection.get("catalog_source"):
        lines.append(f"motor catalog: {motor_selection['catalog_source']} ({basis})")
    for engine in engines:
        pos = vec3(engine["position_m"])
        engine_models = engines_from_vehicle(manifest)
        thrust_n = next(
            (model.thrust_n for model in engine_models if model.position_m == tuple(engine["position_m"])),
            body["ca_reference_thrust_n"],
        )
        lines.append(
            f"  {engine.get('id', 'engine')}: {motor_label(manifest, engine)} "
            f"@ {thrust_n:.1f} N  pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
        )
    return "\n".join(lines)


CAMERA_PRESETS = {
    "iso": (24, -58),
    "top": (90, -90),
    "side": (0, -90),
    "front": (0, 180),
}

ENGINE_COLORS = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")


@dataclass
class EngineActuatorControl:
    engine: EngineGeometry
    engine_id: str
    color: str
    roll_deg: float
    yaw_deg: float
    thrust_artists: ActorList = field(default_factory=list)
    limit_artists: ActorList = field(default_factory=list)


def engine_actuator_limits(engine: EngineGeometry) -> dict[str, float]:
    return {
        "roll_min": engine.roll_min_deg,
        "roll_max": engine.roll_max_deg,
        "yaw_min": engine.yaw_min_deg,
        "yaw_max": engine.yaw_max_deg,
    }


def at_actuator_limit(value: float, low: float, high: float, epsilon: float = 0.25) -> bool:
    return abs(value - low) <= epsilon or abs(value - high) <= epsilon


def actuator_status_lines(manifest: dict, control: EngineActuatorControl) -> list[str]:
    limits = engine_actuator_limits(control.engine)
    direction = plant_thrust_direction(control.engine, control.roll_deg, control.yaw_deg)
    thrust_n = control.engine.thrust_n
    engine_manifest = next(
        (
            engine
            for engine in engines_from_manifest(manifest)
            if str(engine.get("id", "")) == control.engine_id
        ),
        {},
    )
    lines = [
        f"{control.engine_id}  {motor_label(manifest, engine_manifest)}",
        (
            f"  limits  roll [{limits['roll_min']:+.0f}, {limits['roll_max']:+.0f}] deg  "
            f"yaw [{limits['yaw_min']:+.0f}, {limits['yaw_max']:+.0f}] deg"
        ),
        f"  roll {control.roll_deg:+.1f} deg  yaw {control.yaw_deg:+.1f} deg",
        (
            f"  thrust  {thrust_n:.1f} N  "
            f"dir ({direction[0]:+.3f}, {direction[1]:+.3f}, {direction[2]:+.3f})"
        ),
    ]
    if at_actuator_limit(control.roll_deg, limits["roll_min"], limits["roll_max"]):
        lines.append("  ! roll at limit")
    if at_actuator_limit(control.yaw_deg, limits["yaw_min"], limits["yaw_max"]):
        lines.append("  ! yaw at limit")
    return lines


def clear_artists(artists: ActorList) -> None:
    for actor in artists:
        actor.SetVisibility(False)


def draw_actuator_limit_envelope(
    scene: PyVistaScene,
    control: EngineActuatorControl,
    axis_length: float,
    *,
    samples: int = 12,
) -> ActorList:
    import numpy as np

    engine = control.engine
    position = engine.position_m
    limits = engine_actuator_limits(engine)
    roll_vals = np.linspace(limits["roll_min"], limits["roll_max"], samples)
    yaw_vals = np.linspace(limits["yaw_min"], limits["yaw_max"], samples)
    envelope_length = axis_length * 0.9

    mesh_x = np.zeros((samples, samples))
    mesh_y = np.zeros((samples, samples))
    mesh_z = np.zeros((samples, samples))
    for row, roll_deg in enumerate(roll_vals):
        for col, yaw_deg in enumerate(yaw_vals):
            direction = plant_thrust_direction(engine, float(roll_deg), float(yaw_deg))
            mesh_x[row, col] = position[0] - direction[0] * envelope_length
            mesh_y[row, col] = position[1] - direction[1] * envelope_length
            mesh_z[row, col] = position[2] - direction[2] * envelope_length

    artists: ActorList = scene.add_surface(mesh_x, mesh_y, mesh_z, color=control.color, opacity=0.14)
    border_specs = (
        (roll_vals, np.full(samples, limits["yaw_max"])),
        (roll_vals, np.full(samples, limits["yaw_min"])),
        (np.full(samples, limits["roll_max"]), yaw_vals),
        (np.full(samples, limits["roll_min"]), yaw_vals),
    )
    for roll_line, yaw_line in border_specs:
        points = [
            (
                position[0] - plant_thrust_direction(engine, float(r), float(y))[0] * envelope_length,
                position[1] - plant_thrust_direction(engine, float(r), float(y))[1] * envelope_length,
                position[2] - plant_thrust_direction(engine, float(r), float(y))[2] * envelope_length,
            )
            for r, y in zip(roll_line, yaw_line)
        ]
        for start, end in zip(points, points[1:]):
            artists.extend(scene.add_line(start, end, color=control.color, width=2.0, opacity=0.85))
    return artists


def draw_coupled_yaw_axis(scene: PyVistaScene, control: EngineActuatorControl, axis_length: float) -> ActorList:
    yaw_axis = coupled_yaw_axis(control.engine, control.roll_deg)
    return draw_vector(scene, control.engine.position_m, yaw_axis, axis_length * 0.75, "#17becf", "yaw@roll")


def draw_computed_thrust(scene: PyVistaScene, control: EngineActuatorControl, axis_length: float) -> ActorList:
    direction = plant_thrust_direction(control.engine, control.roll_deg, control.yaw_deg)
    thrust_n = control.engine.thrust_n
    arrow_length = axis_length * 1.2
    artists = draw_coupled_yaw_axis(scene, control, axis_length)
    artists.extend(draw_vector(scene, control.engine.position_m, direction, arrow_length, control.color, None))
    ox, oy, oz = control.engine.position_m
    artists.extend(scene.add_label((ox + direction[0] * arrow_length, oy + direction[1] * arrow_length, oz + direction[2] * arrow_length), f"{thrust_n:.0f} N", color=control.color))
    return artists


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

    def refresh() -> None:
        for control in controls:
            clear_artists(control.thrust_artists)
            control.thrust_artists.clear()
            control.thrust_artists.extend(draw_computed_thrust(scene, control, axis_length))
        lines: list[str] = []
        for control in controls:
            lines.extend(actuator_status_lines(working_manifest, control))
            lines.append("")
        status.SetText(0, "\n".join(lines).rstrip())
        plotter.render()

    if build_stage >= 2 and controls:
        active = controls[0]
        limits = engine_actuator_limits(active.engine)

        def on_roll(value: float) -> None:
            active.roll_deg = float(value)
            refresh()

        def on_yaw(value: float) -> None:
            active.yaw_deg = float(value)
            refresh()

        plotter.add_slider_widget(on_roll, [limits["roll_min"], limits["roll_max"]], value=0.0, title=f"{active.engine_id} roll")
        if build_stage >= 3:
            plotter.add_slider_widget(on_yaw, [limits["yaw_min"], limits["yaw_max"]], value=0.0, title=f"{active.engine_id} yaw")

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

