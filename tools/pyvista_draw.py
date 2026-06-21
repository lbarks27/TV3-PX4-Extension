"""PyVista drawing primitives and scene layers for TV3 vehicle manifests."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from tools.manifest_geometry import link_pose_xyz, link_world_com, vec3
from tools.manifest_io import engines_from_manifest
from tools.pyvista_viz import ActorList, PyVistaScene, SceneLayers
from tools.tv3_control_allocator import EngineGeometry, coupled_yaw_axis, plant_thrust_direction
from tools.tv3_engine_frame import MAX_BUILD_STAGE, build_engine_frame_axes
from tools.viz_common import ENGINE_COLORS

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

    for index, engine in enumerate(engines_from_manifest(manifest)):
        color = ENGINE_COLORS[index % len(ENGINE_COLORS)]
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
    from tools.manifest_geometry import motor_label

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
