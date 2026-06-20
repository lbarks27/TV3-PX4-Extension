#!/usr/bin/env python3
"""Matplotlib viewer for TV3 vehicle manifests in the vehicle reference frame.

Plots link poses, COMs, engine mount points, gimbal rotation axes, and joint origins
from a checked-in vehicle JSON manifest. Use this to verify Phase 2 geometry before
promoting measured values.

Modes:
  default       write a 4-panel overview PNG under build/vehicle_frame/
  --interactive dark-mode 3D viewer with per-engine roll/yaw sliders; thrust vectors
                are computed live from gimbal deflection using the SIH plant model
  --show        open the overview figure in a Matplotlib window

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

DARK_BG = "#141414"
DARK_PANEL = "#1f1f1f"
DARK_TEXT = "#e8e8e8"
DARK_MUTED = "#9aa0a6"
DARK_GRID = "#3a3a3a"
DARK_EDGE = "#555555"
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


ArtistList = list[Any]


@dataclass
class SceneLayers:
    groups: dict[str, ArtistList] = field(default_factory=dict)

    def add(self, group: str, artists: ArtistList) -> None:
        if not artists:
            return
        self.groups.setdefault(group, []).extend(artists)

    def labels(self) -> list[str]:
        return list(self.groups.keys())

    def set_visible(self, group: str, visible: bool) -> None:
        for artist in self.groups.get(group, []):
            artist.set_visible(visible)


def draw_axes(ax, origin: tuple[float, float, float], length: float, label_prefix: str = "") -> ArtistList:
    ox, oy, oz = origin
    artists: ArtistList = []
    specs = (
        ("+X fwd", (length, 0.0, 0.0), "#d62728"),
        ("+Y right", (0.0, length, 0.0), "#2ca02c"),
        ("+Z down", (0.0, 0.0, length), "#1f77b4"),
    )
    for name, direction, color in specs:
        artists.append(
            ax.quiver(
                ox,
                oy,
                oz,
                direction[0],
                direction[1],
                direction[2],
                color=color,
                linewidth=1.5,
                arrow_length_ratio=0.15,
            )
        )
        artists.append(
            ax.text(ox + direction[0], oy + direction[1], oz + direction[2], f"{label_prefix}{name}", color=color, fontsize=8)
        )
    return artists


def draw_vector(
    ax,
    origin: Sequence[float],
    direction: Sequence[float],
    length: float,
    color: str,
    label: str | None = None,
    *,
    alpha: float = 1.0,
) -> ArtistList:
    ox, oy, oz = vec3(origin)
    dx, dy, dz = vec3(direction, (1.0, 0.0, 0.0))
    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    if norm <= 1e-9:
        return []
    scale = length / norm
    artists: ArtistList = [
        ax.quiver(
            ox,
            oy,
            oz,
            dx * scale,
            dy * scale,
            dz * scale,
            color=color,
            linewidth=1.2,
            arrow_length_ratio=0.2,
            alpha=alpha,
        )
    ]
    if label:
        artists.append(ax.text(ox + dx * scale, oy + dy * scale, oz + dz * scale, label, color=color, fontsize=7))
    return artists


def draw_box_wireframe(ax, center: Sequence[float], size: Sequence[float], color: str, label: str | None = None) -> ArtistList:
    cx, cy, cz = vec3(center)
    sx, sy, sz = vec3(size)
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    corners = [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]
    edges = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    )
    artists: ArtistList = []
    for start, end in edges:
        xs = [corners[start][0], corners[end][0]]
        ys = [corners[start][1], corners[end][1]]
        zs = [corners[start][2], corners[end][2]]
        artists.extend(ax.plot(xs, ys, zs, color=color, linewidth=1.0, alpha=0.8))
    artists.append(ax.scatter([cx], [cy], [cz], color=color, s=20))
    if label:
        artists.append(ax.text(cx, cy, cz, label, color=color, fontsize=8))
    return artists


def draw_marker(ax, point: Sequence[float], color: str, label: str, size: float = 36.0) -> ArtistList:
    x, y, z = vec3(point)
    return [
        ax.scatter([x], [y], [z], color=color, s=size, depthshade=False),
        ax.text(x, y, z, f"  {label}", color=color, fontsize=8),
    ]


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


def set_equal_limits(ax, points: Iterable[tuple[float, float, float]], pad: float = 0.08) -> tuple[tuple[float, float, float], float]:
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
    apply_zoom_limits(ax, (cx, cy, cz), half)
    return (cx, cy, cz), half


def apply_zoom_limits(
    ax,
    center: tuple[float, float, float],
    half_span: float,
    *,
    min_half: float = 0.02,
    max_half: float = 8.0,
) -> float:
    cx, cy, cz = center
    half = max(min_half, min(max_half, half_span))
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_zlim(cz - half, cz + half)
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect((1, 1, 1))
    return half


def zoom_limits(ax, center: tuple[float, float, float], half_span: float, factor: float) -> float:
    """Scale the view radius. factor < 1 zooms in; factor > 1 zooms out."""
    return apply_zoom_limits(ax, center, half_span * factor)


def build_scene_layers(
    ax,
    manifest: dict,
    body_size: Sequence[float],
    axis_length: float,
    *,
    build_stage: int = MAX_BUILD_STAGE,
) -> SceneLayers:
    body = manifest["vehicle"]
    layers = SceneLayers()

    layers.add("reference frame", draw_axes(ax, (0.0, 0.0, 0.0), axis_length))
    layers.add("origin", draw_marker(ax, (0.0, 0.0, 0.0), "#111111", "origin", size=24.0))

    body_com = (body["body_com_x_m"], 0.0, 0.0)
    layers.add("body box", draw_box_wireframe(ax, body_com, body_size, "#7f7f7f", "body COM"))

    for link in manifest.get("physical_model", {}).get("links", []) or []:
        link_id = str(link.get("id", "link"))
        origin = link_pose_xyz(link)
        if link_id == "body":
            continue
        layers.add("links", draw_marker(ax, origin, "#9467bd", f"{link_id} origin", size=28.0))
        com = link_world_com(link)
        if com != origin:
            layers.add("links", draw_marker(ax, com, "#8c564b", f"{link_id} COM", size=20.0))

    engine_colors = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")
    for index, engine in enumerate(engines_from_manifest(manifest)):
        color = engine_colors[index % len(engine_colors)]
        engine_id = engine.get("id", f"engine_{index}")
        position = vec3(engine["position_m"])
        channel = engine.get("load_cell_channel", "?")
        layers.add("engines", draw_marker(ax, position, color, f"{engine_id} (LC {channel})", size=48.0))
        frame = build_engine_frame_axes(position, build_stage=build_stage)
        if build_stage >= 1:
            layers.add(
                "thrust ref",
                draw_vector(ax, position, frame.thrust_axis, axis_length * 0.9, color, "thrust ref"),
            )
        if build_stage >= 2:
            layers.add(
                "primary axis",
                draw_vector(ax, position, frame.primary_axis, axis_length * 0.8, "#bcbd22", "primary"),
            )
        if build_stage >= 3:
            layers.add(
                "secondary axis",
                draw_vector(ax, position, frame.secondary_axis, axis_length * 0.8, "#17becf", "secondary"),
            )

    for joint in manifest.get("physical_model", {}).get("joints", []) or []:
        origin = vec3(joint.get("origin_m", [0.0, 0.0, 0.0]))
        joint_id = str(joint.get("id", "joint"))
        layers.add("joints", draw_marker(ax, origin, "#2ca02c", joint_id, size=18.0))
        layers.add("joints", draw_vector(ax, origin, joint.get("axis", [0.0, 0.0, 1.0]), axis_length * 0.5, "#98df8a", None))

    return layers


def annotate_scene(
    ax,
    manifest: dict,
    body_size: Sequence[float],
    axis_length: float,
    *,
    build_stage: int = MAX_BUILD_STAGE,
) -> SceneLayers:
    return build_scene_layers(ax, manifest, body_size, axis_length, build_stage=build_stage)


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


def project_points(points: Sequence[tuple[float, float, float]], axes: tuple[int, int]) -> tuple[list[float], list[float]]:
    a, b = axes
    xs = [p[a] for p in points]
    ys = [p[b] for p in points]
    return xs, ys


def draw_projected_scene(ax, manifest: dict, body_size: Sequence[float], axes: tuple[int, int], title: str) -> None:
    body = manifest["vehicle"]
    body_com = (body["body_com_x_m"], 0.0, 0.0)
    bx, by, bz = body_com
    sx, sy, sz = vec3(body_size)
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    box_points = [
        (bx - hx, by - hy, bz - hz),
        (bx + hx, by - hy, bz - hz),
        (bx + hx, by + hy, bz - hz),
        (bx - hx, by + hy, bz - hz),
        (bx - hx, by - hy, bz + hz),
        (bx + hx, by - hy, bz + hz),
        (bx + hx, by + hy, bz + hz),
        (bx - hx, by + hy, bz + hz),
    ]
    edges = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7))
    for start, end in edges:
        xs = [box_points[start][axes[0]], box_points[end][axes[0]]]
        ys = [box_points[start][axes[1]], box_points[end][axes[1]]]
        ax.plot(xs, ys, color="#7f7f7f", linewidth=1.0)

    ax.scatter([0.0], [0.0], color="#111111", s=18)
    ax.scatter([body_com[axes[0]]], [body_com[axes[1]]], color="#7f7f7f", s=24)

    for index, engine in enumerate(engines_from_manifest(manifest)):
        pos = vec3(engine["position_m"])
        ax.scatter([pos[axes[0]]], [pos[axes[1]]], s=48)
        ax.text(pos[axes[0]], pos[axes[1]], f" e{index}", fontsize=8)

    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")


def build_figure(manifest: dict, body_size: Sequence[float], axis_length: float, *, build_stage: int = MAX_BUILD_STAGE):
    try:
        import matplotlib

        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "missing dependency: install matplotlib with `python3 -m pip install -r requirements-viz.txt`"
        ) from exc

    points = extent_points(manifest, body_size)
    fig = plt.figure(figsize=(12, 9))
    fig.suptitle(summary_text(manifest, build_stage=build_stage), fontsize=10, x=0.02, ha="left", family="monospace")

    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    annotate_scene(ax3d, manifest, body_size, axis_length, build_stage=build_stage)
    ax3d.set_title("3D vehicle frame")
    ax3d.set_xlabel("X forward (m)")
    ax3d.set_ylabel("Y right (m)")
    ax3d.set_zlabel("Z down (m)")
    set_equal_limits(ax3d, points)

    ax_xy = fig.add_subplot(2, 2, 2)
    draw_projected_scene(ax_xy, manifest, body_size, (0, 1), "Top view (X-Y, looking down -Z)")
    ax_xy.set_xlabel("X forward (m)")
    ax_xy.set_ylabel("Y right (m)")

    ax_xz = fig.add_subplot(2, 2, 3)
    draw_projected_scene(ax_xz, manifest, body_size, (0, 2), "Side view (X-Z, looking from -Y)")
    ax_xz.set_xlabel("X forward (m)")
    ax_xz.set_ylabel("Z down (m)")

    ax_yz = fig.add_subplot(2, 2, 4)
    draw_projected_scene(ax_yz, manifest, body_size, (1, 2), "Front view (Y-Z, looking from -X)")
    ax_yz.set_xlabel("Y right (m)")
    ax_yz.set_ylabel("Z down (m)")

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    return fig


CAMERA_PRESETS = {
    "iso": (24, -58),
    "top": (90, -90),
    "side": (0, -90),
    "front": (0, 180),
}

ENGINE_COLORS = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")


def apply_dark_theme(fig, axes: Iterable[Any], text_artists: Iterable[Any] = ()) -> None:
    fig.patch.set_facecolor(DARK_BG)
    for axis in axes:
        if hasattr(axis, "set_facecolor"):
            axis.set_facecolor(DARK_PANEL)
        axis.tick_params(colors=DARK_TEXT)
        for attr in ("xaxis", "yaxis", "zaxis"):
            if not hasattr(axis, attr):
                continue
            axis_obj = getattr(axis, attr)
            axis_obj.label.set_color(DARK_TEXT)
            if hasattr(axis_obj, "pane"):
                axis_obj.pane.set_facecolor(DARK_PANEL)
                axis_obj.pane.set_edgecolor(DARK_EDGE)
            if hasattr(axis_obj, "_axinfo"):
                axis_obj._axinfo["grid"]["color"] = DARK_GRID
        if hasattr(axis, "title"):
            axis.title.set_color(DARK_TEXT)
        if hasattr(axis, "grid"):
            axis.grid(True, color=DARK_GRID, alpha=0.45)
    for artist in text_artists:
        if hasattr(artist, "set_color"):
            artist.set_color(DARK_TEXT)


@dataclass
class EngineActuatorControl:
    engine: EngineGeometry
    engine_id: str
    color: str
    roll_deg: float
    yaw_deg: float
    thrust_artists: ArtistList = field(default_factory=list)
    limit_artists: ArtistList = field(default_factory=list)


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


def clear_artists(artists: ArtistList) -> None:
    for artist in artists:
        artist.remove()
    artists.clear()


def draw_actuator_limit_envelope(
    ax,
    control: EngineActuatorControl,
    axis_length: float,
    *,
    samples: int = 12,
) -> ArtistList:
    """Reachable -thrust directions over roll/yaw limits (opposite of thrust vector, coupled kinematics)."""
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

    artists: ArtistList = [
        ax.plot_surface(
            mesh_x,
            mesh_y,
            mesh_z,
            color=control.color,
            alpha=0.14,
            linewidth=0,
            antialiased=True,
            shade=False,
        )
    ]

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
        xs, ys, zs = zip(*points)
        artists.extend(ax.plot(xs, ys, zs, color=control.color, linewidth=1.4, alpha=0.85))

    return artists


def draw_coupled_yaw_axis(ax, control: EngineActuatorControl, axis_length: float) -> ArtistList:
    """Show yaw hinge after roll coupling (yaw axis rotates with roll, roll does not follow yaw)."""
    yaw_axis = coupled_yaw_axis(control.engine, control.roll_deg)
    artists = draw_vector(ax, control.engine.position_m, yaw_axis, axis_length * 0.75, "#17becf", "yaw@roll")
    for artist in artists:
        if hasattr(artist, "set_linestyle"):
            artist.set_linestyle("--")
    return artists


def draw_computed_thrust(ax, control: EngineActuatorControl, axis_length: float) -> ArtistList:
    direction = plant_thrust_direction(control.engine, control.roll_deg, control.yaw_deg)
    thrust_n = control.engine.thrust_n
    arrow_length = axis_length * 1.2
    artists = draw_coupled_yaw_axis(ax, control, axis_length)
    artists.extend(draw_vector(ax, control.engine.position_m, direction, arrow_length, control.color, None))
    ox, oy, oz = control.engine.position_m
    label = f"{thrust_n:.0f} N"
    artists.append(
        ax.text(
            ox + direction[0] * arrow_length,
            oy + direction[1] * arrow_length,
            oz + direction[2] * arrow_length,
            f"  {label}",
            color=control.color,
            fontsize=8,
        )
    )
    return artists


def apply_motor_selection(manifest: dict, motor_id: str) -> None:
    motor_selection = manifest.setdefault("motor_selection", {})
    motor_selection["default_motor_id"] = motor_id
    for engine in engines_from_manifest(manifest):
        engine["motor_id"] = motor_id


def rebuild_controls(manifest: dict, controls: list[EngineActuatorControl]) -> None:
    engine_models = engines_from_vehicle(manifest)
    for index, control in enumerate(controls):
        control.engine = engine_models[index]


def run_interactive_viewer(
    manifest: dict,
    body_size: Sequence[float],
    axis_length: float,
    *,
    build_stage: int = MAX_BUILD_STAGE,
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import CheckButtons, RadioButtons, Slider
    except ImportError as exc:
        raise SystemExit(
            "missing dependency: install matplotlib with `python3 -m pip install -r requirements-viz.txt`"
        ) from exc

    working_manifest = copy.deepcopy(manifest)
    motor_selection = working_manifest.get("motor_selection", {})
    catalog_source = motor_selection.get("catalog_source")
    catalog = load_motor_catalog(str(catalog_source)) if catalog_source else {}
    motor_options = sorted(catalog.values(), key=lambda entry: entry.motor_index) if catalog else []

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

    points = extent_points(working_manifest, body_size)
    fig = plt.figure(figsize=(13.5, 9.0))
    if getattr(fig.canvas.manager, "set_window_title", None):
        fig.canvas.manager.set_window_title(f"TV3 actuator viewer — {working_manifest.get('name', 'vehicle')}")
    summary_artist = fig.text(
        0.22,
        0.97,
        summary_text(working_manifest, build_stage=build_stage),
        fontsize=9,
        family="monospace",
        va="top",
    )

    ax = fig.add_axes((0.22, 0.08, 0.76, 0.84), projection="3d")
    static_layers = build_scene_layers(ax, working_manifest, body_size, axis_length, build_stage=build_stage)
    ax.set_title(f"Interactive actuator viewer — build stage {build_stage}")
    ax.set_xlabel("X forward (m)")
    ax.set_ylabel("Y right (m)")
    ax.set_zlabel("Z down (m)")
    view_center, default_half = set_equal_limits(ax, points)
    view_half = default_half
    ax.view_init(elev=CAMERA_PRESETS["iso"][0], azim=CAMERA_PRESETS["iso"][1])

    def set_view_zoom(factor: float) -> None:
        nonlocal view_half
        view_half = zoom_limits(ax, view_center, view_half, factor)
        fig.canvas.draw_idle()

    def reset_view_zoom() -> None:
        nonlocal view_half
        view_half = apply_zoom_limits(ax, view_center, default_half)
        fig.canvas.draw_idle()

    checkbox_ax = fig.add_axes((0.02, 0.74, 0.17, 0.20))
    checkbox_ax.set_title("Layers", fontsize=9, color=DARK_TEXT)
    static_labels = static_layers.labels()
    dynamic_labels = ["computed thrust", "actuator limits"] if build_stage >= 3 else []
    all_labels = static_labels + dynamic_labels
    checked = [True] * len(static_labels) + ([True, False] if build_stage >= 3 else [])
    check = CheckButtons(checkbox_ax, all_labels, checked, label_props={"color": [DARK_TEXT]})
    dynamic_visibility = {name: True for name in dynamic_labels}

    motor_ax = None
    motor_buttons: Any | None = None
    if motor_options:
        motor_ax = fig.add_axes((0.02, 0.56, 0.17, 0.12))
        motor_ax.set_title("Motor", fontsize=9, color=DARK_TEXT)
        selected_motor_id = resolve_motor_id(
            engines_from_manifest(working_manifest)[0],
            motor_selection,
        )
        motor_labels = [entry.label for entry in motor_options]
        motor_id_by_label = {entry.label: entry.motor_id for entry in motor_options}
        selected_label = next(
            (entry.label for entry in motor_options if entry.motor_id == selected_motor_id),
            motor_labels[0],
        )
        motor_buttons = RadioButtons(
            motor_ax,
            motor_labels,
            active=motor_labels.index(selected_label),
            label_props={"color": [DARK_TEXT] * len(motor_labels)},
        )

    limits_ax = fig.add_axes((0.02, 0.34, 0.17, 0.20))
    limits_ax.axis("off")
    limits_ax.set_title("Actuator limits & state", fontsize=9, color=DARK_TEXT)
    limits_text = limits_ax.text(0.0, 1.0, "", fontsize=7.5, va="top", family="monospace", color=DARK_TEXT)

    slider_specs: list[tuple[EngineActuatorControl, str, str, float, float, float]] = []
    if build_stage >= 3:
        for control in controls:
            limits = engine_actuator_limits(control.engine)
            slider_specs.extend(
                [
                    (control, "roll", f"{control.engine_id} roll", limits["roll_min"], limits["roll_max"], control.roll_deg),
                    (control, "yaw", f"{control.engine_id} yaw", limits["yaw_min"], limits["yaw_max"], control.yaw_deg),
                ]
            )
    elif build_stage == 2:
        for control in controls:
            limits = engine_actuator_limits(control.engine)
            slider_specs.append(
                (control, "roll", f"{control.engine_id} roll", limits["roll_min"], limits["roll_max"], control.roll_deg)
            )

    slider_height = 0.02
    slider_gap = 0.01
    top_y = 0.30
    sliders: list[Any] = []
    slider_axes: list[Any] = []
    for index, (control, field_name, label, vmin, vmax, initial) in enumerate(slider_specs):
        slider_ax = fig.add_axes((0.02, top_y - index * (slider_height + slider_gap), 0.17, slider_height))
        slider_ax.set_facecolor(DARK_PANEL)
        slider = Slider(slider_ax, label, vmin, vmax, valinit=initial, valfmt="%.1f°", color=control.color)
        slider.label.set_color(DARK_TEXT)
        slider.valtext.set_color(DARK_TEXT)
        sliders.append(slider)
        slider_axes.append(slider_ax)

        def on_slider_change(value, control=control, field_name=field_name) -> None:
            setattr(control, f"{field_name}_deg", float(value))
            refresh_dynamic_artists()

        slider.on_changed(on_slider_change)

    help_ax = fig.add_axes((0.02, 0.02, 0.17, 0.04))
    help_ax.axis("off")
    help_text = help_ax.text(
        0.0,
        1.0,
        "Drag rotate | scroll zoom | r reset",
        fontsize=7,
        va="top",
        family="monospace",
        color=DARK_MUTED,
    )

    theme_axes: list[Any] = [ax, checkbox_ax, limits_ax, help_ax]
    if motor_ax is not None:
        theme_axes.append(motor_ax)
    apply_dark_theme(
        fig,
        theme_axes,
        [summary_artist, limits_text, help_text],
    )
    for spine in checkbox_ax.spines.values():
        spine.set_color(DARK_EDGE)

    def refresh_dynamic_artists() -> None:
        for control in controls:
            clear_artists(control.thrust_artists)
            clear_artists(control.limit_artists)
            if dynamic_visibility.get("computed thrust"):
                control.thrust_artists.extend(draw_computed_thrust(ax, control, axis_length))
            if dynamic_visibility.get("actuator limits"):
                control.limit_artists.extend(draw_actuator_limit_envelope(ax, control, axis_length))

        lines: list[str] = []
        for control in controls:
            lines.extend(actuator_status_lines(working_manifest, control))
            lines.append("")
        limits_text.set_text("\n".join(lines).rstrip())
        summary_artist.set_text(summary_text(working_manifest, build_stage=build_stage))
        fig.canvas.draw_idle()

    def on_motor_selected(label: str) -> None:
        motor_id = motor_id_by_label[label]
        apply_motor_selection(working_manifest, motor_id)
        rebuild_controls(working_manifest, controls)
        refresh_dynamic_artists()

    def toggle_layer(clicked_label: str) -> None:
        if clicked_label in static_layers.groups:
            for artist in static_layers.groups[clicked_label]:
                artist.set_visible(not artist.get_visible())
        elif clicked_label in dynamic_visibility:
            dynamic_visibility[clicked_label] = not dynamic_visibility[clicked_label]
            refresh_dynamic_artists()
            return
        fig.canvas.draw_idle()

    check.on_clicked(toggle_layer)
    if motor_buttons is not None:
        motor_buttons.on_clicked(on_motor_selected)

    def on_scroll(event) -> None:
        if event.inaxes != ax:
            return
        step = getattr(event, "step", 0)
        if step > 0 or event.button == "up":
            set_view_zoom(0.9)
        elif step < 0 or event.button == "down":
            set_view_zoom(1.1)

    def on_key(event) -> None:
        if event.key == "r":
            reset_view_zoom()

    fig.canvas.mpl_connect("scroll_event", on_scroll)
    fig.canvas.mpl_connect("key_press_event", on_key)
    refresh_dynamic_artists()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, default=DEFAULT_VEHICLE, help="Vehicle manifest JSON path")
    parser.add_argument("--body-size", nargs=3, type=float, metavar=("SX", "SY", "SZ"), help="Body box size in meters")
    parser.add_argument("--axis-length", type=float, default=0.12, help="Arrow length for frame and thrust axes (m)")
    parser.add_argument("--save", type=Path, help="Write PNG to this path")
    parser.add_argument("--show", action="store_true", help="Open the static overview figure interactively")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Open interactive 3D viewer with actuator sliders and computed thrust vectors",
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

    if args.interactive:
        run_interactive_viewer(manifest, body_size, args.axis_length, build_stage=args.build_stage)
        return

    fig = build_figure(manifest, body_size, args.axis_length, build_stage=args.build_stage)

    if args.save:
        save_path = args.save if args.save.is_absolute() else REPO_ROOT / args.save
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
        print(f"wrote {save_path}")

    if args.show:
        import matplotlib.pyplot as plt

        plt.show()
    elif not args.save:
        default_out = REPO_ROOT / "build" / "vehicle_frame" / f"{manifest.get('name', vehicle_path.stem)}.png"
        default_out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(default_out, dpi=160, bbox_inches="tight")
        print(f"wrote {default_out}")
        print("use --interactive for the 3D viewer or --show to inspect the overview figure")


if __name__ == "__main__":
    main()