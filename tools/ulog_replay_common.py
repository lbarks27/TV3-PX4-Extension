"""Shared ULog replay utilities for TV3 log replay and static previews."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.plot_ulog import find_latest_ulog, import_ulog  # noqa: E402
DEFAULT_VEHICLE = REPO_ROOT / "config/vehicles/tv3_lander_v1.json"

TOPIC_ALIASES = {
    "tv3_engine_state": ("tv3_engine_state", "rocket_engine_state"),
    "tv3_engine_command": ("tv3_engine_command", "rocket_engine_command"),
    "vehicle_attitude": ("vehicle_attitude_groundtruth", "vehicle_attitude"),
    "vehicle_local_position": ("vehicle_local_position_groundtruth", "vehicle_local_position"),
    "tv3_status": ("tv3_status",),
    "tv3_guidance_status": ("tv3_guidance_status",),
    "tv3_thrust": ("tv3_thrust",),
    "trajectory_setpoint": ("trajectory_setpoint",),
    "control_allocator_status": ("control_allocator_status",),
    "vehicle_torque_setpoint": ("vehicle_torque_setpoint",),
    "vehicle_thrust_setpoint": ("vehicle_thrust_setpoint",),
}

TV3_MODE_LABELS = {
    0: "DISARMED_SAFE",
    1: "ARMED_STANDBY",
    2: "READY",
    3: "IGNITION_PENDING",
    4: "BOOST",
    5: "COAST",
    6: "ABORT",
}

GUIDANCE_PHASE_LABELS = {
    0: "STANDBY",
    1: "LAUNCH_ASCENT",
    2: "APOGEE_TRACK",
    3: "WAYPOINT_TRACK",
    4: "LANDING_APPROACH",
    5: "COMPLETE",
    6: "ABORT",
}

CONTROL_UNREACHABLE_LABELS = {
    0: "OK",
    1: "THRUST_ENVELOPE",
    2: "TORQUE_ENVELOPE",
    3: "NO_ACTIVE_ENGINES",
}

GUIDANCE_UNREACHABLE_LABELS = {
    0: "OK",
    1: "IMPULSE",
    2: "THRUST_MARGIN",
    3: "LANDING_RESERVE",
    4: "ABORT_CORRIDOR",
    5: "CONTROL",
}

ArtistList = list[Any]
FrameCallback = Callable[[int], None]
ScrollCallback = Callable[[Any], None]
KeyCallback = Callable[[Any], None]


def load_manifest(path: Path) -> dict:
    return json.loads(path.read_text())


def resolve_manifest(ulog_path: Path, vehicle_path: Path | None) -> dict:
    if vehicle_path is not None:
        return load_manifest(vehicle_path)
    for candidate in (ulog_path.parent / "vehicle.json", DEFAULT_VEHICLE):
        if candidate.exists():
            return load_manifest(candidate)
    raise SystemExit(f"vehicle manifest not found near {ulog_path} and no --vehicle provided")


def topic_dataset(ulog, logical_name: str):
    for alias in TOPIC_ALIASES.get(logical_name, (logical_name,)):
        for dataset in ulog.data_list:
            if dataset.name == alias:
                return dataset
    return None


def topic_times_us(dataset) -> np.ndarray:
    data = dataset.data
    if "timestamp" in data:
        return np.asarray(data["timestamp"], dtype=np.float64)
    if "timestamp_sample" in data:
        return np.asarray(data["timestamp_sample"], dtype=np.float64)
    raise KeyError("dataset has no timestamp field")


def topic_field(dataset, *names: str) -> np.ndarray | None:
    data = dataset.data
    for name in names:
        if name in data:
            return np.asarray(data[name], dtype=np.float64)
    return None


def interpolate_series(times_us: np.ndarray, values: np.ndarray, query_us: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return np.zeros_like(query_us, dtype=np.float64)
    if values.ndim == 1:
        return np.interp(query_us, times_us, values)
    return np.vstack([np.interp(query_us, times_us, row) for row in values])


def build_query_times(datasets: Sequence, *, fps: float, stride: int) -> tuple[float, np.ndarray]:
    """Return (start_us, query_us) spanning all provided datasets."""
    starts = []
    ends = []
    for dataset in datasets:
        if dataset is None:
            continue
        times = topic_times_us(dataset)
        starts.append(float(times[0]))
        ends.append(float(times[-1]))
    if not starts:
        raise SystemExit("no datasets available to build replay timeline")
    start_us = min(starts)
    end_us = max(ends)
    duration_s = max((end_us - start_us) * 1e-6, 1e-3)
    frame_count = max(int(duration_s * fps), 1)
    query_us = np.linspace(start_us, end_us, frame_count)[:: max(stride, 1)]
    return start_us, query_us


def rotation_matrix_from_quat(quat: Sequence[float]) -> np.ndarray:
    w, x, y, z = [float(value) for value in quat]
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def euler_angles_deg(quat: Sequence[float]) -> tuple[float, float, float]:
    rotation = rotation_matrix_from_quat(quat)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -rotation[2, 0]))))
    roll = math.degrees(math.atan2(rotation[2, 1], rotation[2, 2]))
    yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    return roll, pitch, yaw


def body_axes_in_world(quat: Sequence[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Body forward/right/down unit vectors expressed in NED world frame."""
    rotation = rotation_matrix_from_quat(quat)
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    right = rotation @ np.array([0.0, 1.0, 0.0])
    down = rotation @ np.array([0.0, 0.0, 1.0])
    return forward, right, down


def world_axes_in_body(quat: Sequence[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """NED world basis expressed in the body frame (inverse attitude rotation)."""
    rotation = rotation_matrix_from_quat(quat)
    return rotation.T @ np.array([1.0, 0.0, 0.0]), rotation.T @ np.array([0.0, 1.0, 0.0]), rotation.T @ np.array(
        [0.0, 0.0, 1.0]
    )


def ned_to_plot_xyz(north_m: float, east_m: float, down_m: float) -> tuple[float, float, float]:
    """Map NED to plot axes with altitude increasing upward."""
    return north_m, east_m, -down_m


def altitude_from_ned(down_m: float) -> float:
    return -down_m


def frame_index_at_time(frames: Sequence[Any], time_s: float) -> int:
    return min(range(len(frames)), key=lambda index: abs(frames[index].time_s - time_s))


def scalar_series_or_zeros(dataset, field_name: str, times_us: np.ndarray) -> np.ndarray:
    values = topic_field(dataset, field_name) if dataset is not None else None
    if values is None:
        return np.zeros_like(times_us)
    return values


__all__ = [
    "ArtistList",
    "CONTROL_UNREACHABLE_LABELS",
    "DEFAULT_VEHICLE",
    "GUIDANCE_PHASE_LABELS",
    "GUIDANCE_UNREACHABLE_LABELS",
    "REPO_ROOT",
    "TOPIC_ALIASES",
    "TV3_MODE_LABELS",
    "altitude_from_ned",
    "body_axes_in_world",
    "build_query_times",
    "euler_angles_deg",
    "find_latest_ulog",
    "frame_index_at_time",
    "import_ulog",
    "interpolate_series",
    "load_manifest",
    "ned_to_plot_xyz",
    "resolve_manifest",
    "rotation_matrix_from_quat",
    "scalar_series_or_zeros",
    "topic_dataset",
    "topic_field",
    "topic_times_us",
    "world_axes_in_body",
]