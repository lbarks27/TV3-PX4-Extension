"""Shared scene construction helpers for PyVista and Rerun replay."""

from __future__ import annotations

from typing import Any, Sequence

from tools.tv3_control_allocator import EngineGeometry, plant_thrust_direction
from tools.ulog_replay_common import body_axes_in_world, ned_to_plot_xyz, world_axes_in_body
from tools.viz_common import ENGINE_COLORS


def color_to_rgba(color: str, alpha: float = 1.0) -> list[int]:
    color = color.lstrip("#")
    rgb = [int(color[index : index + 2], 16) for index in (0, 2, 4)]
    return rgb + [int(max(0.0, min(1.0, alpha)) * 255)]


def trajectory_path_points(frames: Sequence[Any]) -> list[tuple[float, float, float]]:
    return [ned_to_plot_xyz(*frame.position_ned) for frame in frames]


def body_axis_vectors_plot(quaternion: Sequence[float], axis_length: float) -> list[tuple[tuple[float, float, float], str, str]]:
    forward, right, down = body_axes_in_world(quaternion)
    return [
        ((float(forward[0]), float(forward[1]), float(-forward[2])), "#ff7f0e", "F"),
        ((float(right[0]), float(right[1]), float(-right[2])), "#2ca02c", "R"),
        ((float(down[0]), float(down[1]), float(-down[2])), "#d62728", "D"),
    ]


def engine_thrust_arrows(
    engines: Sequence[EngineGeometry],
    frame: Any,
    *,
    axis_length: float,
    ref_thrust_n: float,
    build_stage: int = 3,
) -> tuple[list[list[float]], list[list[float]], list[list[int]]]:
    origins: list[list[float]] = []
    vectors: list[list[float]] = []
    colors: list[list[int]] = []
    for index, engine in enumerate(engines):
        roll_deg = frame.engine_roll_deg[index] if index < len(frame.engine_roll_deg) else 0.0
        yaw_deg = frame.engine_yaw_deg[index] if index < len(frame.engine_yaw_deg) else 0.0
        thrust_n = frame.engine_thrust_n[index] if index < len(frame.engine_thrust_n) else 0.0
        if build_stage < 1 or thrust_n <= 1e-3:
            continue
        direction = plant_thrust_direction(engine, roll_deg, yaw_deg)
        scale = thrust_n / max(ref_thrust_n, 1.0)
        length = axis_length * 1.2 * min(max(scale, 0.05), 2.5)
        origin = [float(engine.position_m[0]), float(engine.position_m[1]), float(engine.position_m[2])]
        vector = [direction[0] * length, direction[1] * length, direction[2] * length]
        origins.append(origin)
        vectors.append(vector)
        colors.append(color_to_rgba(ENGINE_COLORS[index % len(ENGINE_COLORS)]))
    return origins, vectors, colors


def world_triad_arrows_body_frame(
    quaternion: Sequence[float],
    axis_length: float,
) -> tuple[list[list[float]], list[list[float]], list[list[int]], list[str]]:
    north, east, down = world_axes_in_body(quaternion)
    triad_len = axis_length * 0.85
    origins = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    vectors = [
        [float(north[0]) * triad_len, float(north[1]) * triad_len, float(north[2]) * triad_len],
        [float(east[0]) * triad_len, float(east[1]) * triad_len, float(east[2]) * triad_len],
        [float(down[0]) * triad_len, float(down[1]) * triad_len, float(down[2]) * triad_len],
    ]
    colors = [[106, 155, 209], [106, 184, 106], [176, 140, 209]]
    labels = ["world N", "world E", "world D"]
    return origins, vectors, colors, labels