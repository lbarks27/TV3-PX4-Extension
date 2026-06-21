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


from tools.manifest_io import load_manifest


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


def topic_sample_rate_hz(dataset) -> float | None:
    if dataset is None:
        return None
    times = topic_times_us(dataset)
    if len(times) < 2:
        return None
    deltas_us = np.diff(times)
    deltas_us = deltas_us[deltas_us > 0]
    if deltas_us.size == 0:
        return None
    return 1e6 / float(np.median(deltas_us))


def fastest_dataset(datasets: Sequence) -> object | None:
    best = None
    best_hz = -1.0
    for dataset in datasets:
        rate_hz = topic_sample_rate_hz(dataset)
        if rate_hz is None or rate_hz <= best_hz:
            continue
        best_hz = rate_hz
        best = dataset
    return best


def infer_replay_fps(datasets: Sequence, *, cap_hz: float = 50.0) -> float:
    """Pick a replay sample rate from the fastest topic in the provided datasets."""
    rate_hz = topic_sample_rate_hz(fastest_dataset(datasets))
    if rate_hz is None:
        return 20.0
    return min(rate_hz, cap_hz)


def resolve_replay_fps(fps: float, datasets: Sequence) -> float:
    """`fps <= 0` selects the native fastest-topic rate (typically 50 Hz for TV3 SIH logs)."""
    if fps > 0:
        return fps
    return infer_replay_fps(datasets)


def build_query_times(datasets: Sequence, *, fps: float, stride: int) -> tuple[float, np.ndarray]:
    """Return (start_us, query_us) spanning all provided datasets.

    When `fps <= 0`, reuse the fastest topic's native timestamps instead of
    downsampling to a fixed 10 Hz grid.
    """
    active = [dataset for dataset in datasets if dataset is not None]
    if not active:
        raise SystemExit("no datasets available to build replay timeline")

    stride = max(stride, 1)
    if fps <= 0:
        master = fastest_dataset(active)
        if master is not None:
            times = topic_times_us(master)
            start_us = float(times[0])
            query_us = times[::stride]
            return start_us, query_us

    starts = []
    ends = []
    for dataset in active:
        times = topic_times_us(dataset)
        starts.append(float(times[0]))
        ends.append(float(times[-1]))
    start_us = min(starts)
    end_us = max(ends)
    duration_s = max((end_us - start_us) * 1e-6, 1e-3)
    effective_fps = resolve_replay_fps(fps, active)
    frame_count = max(int(duration_s * effective_fps), 1)
    query_us = np.linspace(start_us, end_us, frame_count)[::stride]
    return start_us, query_us


def replay_frame_stats(start_us: float, query_us: np.ndarray, *, fps: float, datasets: Sequence) -> dict[str, float | int | bool]:
    span_s = max((float(query_us[-1]) - float(query_us[0])) * 1e-6, 0.0) if query_us.size else 0.0
    if query_us.size > 1 and span_s > 0:
        effective_hz = (query_us.size - 1) / span_s
    else:
        effective_hz = resolve_replay_fps(fps, datasets)
    return {
        "frames": int(query_us.size),
        "span_s": span_s,
        "effective_hz": float(effective_hz),
        "native_timestamps": fps <= 0,
        "requested_fps": float(fps),
    }


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


def format_replay_sampling(frames: Sequence[Any], *, fps: float) -> str:
    if not frames:
        return "replay: no frames"
    span_s = float(frames[-1].time_s - frames[0].time_s)
    if len(frames) > 1 and span_s > 0:
        effective_hz = (len(frames) - 1) / span_s
    else:
        effective_hz = 0.0
    mode = "native ULog timestamps" if fps <= 0 else f"resampled at {fps:g} Hz"
    return f"replay: {len(frames)} frames over {span_s:.1f} s ({effective_hz:.1f} Hz effective, {mode})"


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
    "fastest_dataset",
    "format_replay_sampling",
    "frame_index_at_time",
    "import_ulog",
    "infer_replay_fps",
    "interpolate_series",
    "load_manifest",
    "ned_to_plot_xyz",
    "resolve_manifest",
    "resolve_replay_fps",
    "replay_frame_stats",
    "rotation_matrix_from_quat",
    "topic_sample_rate_hz",
    "scalar_series_or_zeros",
    "topic_dataset",
    "topic_field",
    "topic_times_us",
    "world_axes_in_body",
]