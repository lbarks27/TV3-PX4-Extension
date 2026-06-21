#!/usr/bin/env python3
"""Replay TV3 engine mounts and thrust vectors: PyVista interactive 3D (default) or Rerun (--rerun)."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.pyvista_viz import PyVistaScene  # noqa: E402
from tools.rerun_replay import replay_engines  # noqa: E402
from tools.static_preview import render_engine_preview  # noqa: E402
from tools.ulog_replay_common import (  # noqa: E402
    build_query_times,
    euler_angles_deg,
    find_latest_ulog,
    format_replay_sampling,
    frame_index_at_time,
    import_ulog,
    interpolate_series,
    load_manifest,
    resolve_manifest,
    rotation_matrix_from_quat,
    topic_dataset,
    topic_field,
    topic_times_us,
    world_axes_in_body,
)
from tools.tv3_control_allocator import EngineGeometry, engines_from_vehicle, plant_thrust_direction  # noqa: E402
from tools.tv3_engine_frame import MAX_BUILD_STAGE  # noqa: E402
from tools.manifest_geometry import motor_label, vec3
from tools.manifest_io import engines_from_manifest
from tools.pyvista_draw import draw_coupled_yaw_axis, draw_vector
from tools.viz_common import CAMERA_PRESETS  # noqa: E402


@dataclass(frozen=True)
class ReplayFrame:
    time_s: float
    position_ned: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    engine_thrust_n: tuple[float, ...]
    engine_roll_deg: tuple[float, ...]
    engine_yaw_deg: tuple[float, ...]
    engine_splay_deg: tuple[float, ...]
    ignition_mask: int
    active_mask: int


def build_replay_frames(ulog, manifest: dict, *, fps: float, stride: int) -> list[ReplayFrame]:
    attitude = topic_dataset(ulog, "vehicle_attitude")
    position = topic_dataset(ulog, "vehicle_local_position")
    engine_state = topic_dataset(ulog, "tv3_engine_state")
    engine_command = topic_dataset(ulog, "tv3_engine_command")
    if attitude is None or position is None or engine_state is None:
        missing = [
            name
            for name, dataset in (
                ("vehicle_attitude", attitude),
                ("vehicle_local_position", position),
                ("tv3_engine_state", engine_state),
            )
            if dataset is None
        ]
        raise SystemExit(f"ULog missing required topics: {', '.join(missing)}")

    engines = engines_from_vehicle(manifest)
    engine_count = len(engines)
    att_times = topic_times_us(attitude)
    start_us, query_us = build_query_times(
        (attitude, position, engine_state, engine_command),
        fps=fps,
        stride=stride,
    )

    pos_times = topic_times_us(position)
    state_times = topic_times_us(engine_state)
    quat = np.vstack(
        [
            topic_field(attitude, f"q[{index}]")
            for index in range(4)
            if topic_field(attitude, f"q[{index}]") is not None
        ]
    )
    pos_xyz = np.vstack(
        [
            topic_field(position, "x"),
            topic_field(position, "y"),
            topic_field(position, "z"),
        ]
    )

    thrust_rows = []
    for index in range(engine_count):
        values = topic_field(engine_state, f"measured_thrust_n[{index}]", f"filtered_thrust_n[{index}]")
        if values is None:
            values = np.zeros_like(state_times)
        thrust_rows.append(values)
    thrust = np.vstack(thrust_rows)

    ignition_mask = topic_field(engine_state, "ignition_mask")
    active_mask = topic_field(engine_state, "active_mask")
    if ignition_mask is None:
        ignition_mask = np.zeros_like(state_times)
    if active_mask is None:
        active_mask = np.zeros_like(state_times)

    roll_rows = []
    yaw_rows = []
    splay_rows = []
    cmd_times = topic_times_us(engine_command) if engine_command is not None else state_times
    for index in range(engine_count):
        if engine_command is not None:
            roll = topic_field(engine_command, f"commanded_pitch_deg[{index}]")
            yaw = topic_field(engine_command, f"commanded_yaw_deg[{index}]")
            splay = topic_field(engine_command, f"commanded_splay_deg[{index}]")
        else:
            roll = yaw = splay = None
        roll_rows.append(roll if roll is not None else np.zeros_like(cmd_times))
        yaw_rows.append(yaw if yaw is not None else np.zeros_like(cmd_times))
        splay_rows.append(splay if splay is not None else np.zeros_like(cmd_times))

    quat_i = interpolate_series(att_times, quat, query_us)
    pos_i = interpolate_series(pos_times, pos_xyz, query_us)
    thrust_i = interpolate_series(state_times, thrust, query_us)
    ignition_i = interpolate_series(state_times, ignition_mask, query_us)
    active_i = interpolate_series(state_times, active_mask, query_us)
    roll_i = interpolate_series(cmd_times, np.vstack(roll_rows), query_us)
    yaw_i = interpolate_series(cmd_times, np.vstack(yaw_rows), query_us)
    splay_i = interpolate_series(cmd_times, np.vstack(splay_rows), query_us)

    frames: list[ReplayFrame] = []
    for index, time_us in enumerate(query_us):
        time_s = (time_us - start_us) * 1e-6
        quat_tuple = tuple(float(quat_i[component, index]) for component in range(4))
        thrust_tuple = tuple(float(max(thrust_i[engine, index], 0.0)) for engine in range(engine_count))
        frames.append(
            ReplayFrame(
                time_s=time_s,
                position_ned=(
                    float(pos_i[0, index]),
                    float(pos_i[1, index]),
                    float(pos_i[2, index]),
                ),
                quaternion=quat_tuple,
                engine_thrust_n=thrust_tuple,
                engine_roll_deg=tuple(float(roll_i[engine, index]) for engine in range(engine_count)),
                engine_yaw_deg=tuple(float(yaw_i[engine, index]) for engine in range(engine_count)),
                engine_splay_deg=tuple(float(splay_i[engine, index]) for engine in range(engine_count)),
                ignition_mask=int(round(ignition_i[index])),
                active_mask=int(round(active_i[index])),
            )
        )
    return frames


def replay_summary_text(
    manifest: dict,
    frame: ReplayFrame,
    log_name: str,
    *,
    build_stage: int = MAX_BUILD_STAGE,
) -> str:
    body = manifest["vehicle"]
    engines = engines_from_manifest(manifest)
    roll_deg, pitch_deg, yaw_deg = euler_angles_deg(frame.quaternion)
    altitude_m = -frame.position_ned[2]
    north_m, east_m, down_m = frame.position_ned
    total_thrust = sum(frame.engine_thrust_n)
    lines = [
        f"{manifest.get('name', 'vehicle')} — ULog engine replay",
        f"log: {log_name}",
        f"t={frame.time_s:6.2f} s  alt={altitude_m:6.2f} m  total thrust={total_thrust:6.1f} N",
        (
            f"NED pos (m): N={north_m:+.2f}  E={east_m:+.2f}  D={down_m:+.2f}  "
            f"att (deg): roll={roll_deg:+.1f}  pitch={pitch_deg:+.1f}  yaw={yaw_deg:+.1f}"
        ),
        f"ignition=0b{frame.ignition_mask:03b}  active=0b{frame.active_mask:03b}  "
        f"allocator ref={body['ca_reference_thrust_n']} N",
    ]
    for index, engine in enumerate(engines):
        thrust_n = frame.engine_thrust_n[index] if index < len(frame.engine_thrust_n) else 0.0
        roll = frame.engine_roll_deg[index] if index < len(frame.engine_roll_deg) else 0.0
        yaw = frame.engine_yaw_deg[index] if index < len(frame.engine_yaw_deg) else 0.0
        splay = frame.engine_splay_deg[index] if index < len(frame.engine_splay_deg) else 0.0
        engine_id = engine.get("id", f"engine_{index}")
        lines.append(
            f"  {engine_id}: {motor_label(manifest, engine)}  "
            f"{thrust_n:5.1f} N  roll={roll:+.1f}°  yaw={yaw:+.1f}°  splay={splay:+.1f}°"
        )
    if build_stage < MAX_BUILD_STAGE:
        lines.append(f"(scene build stage {build_stage}; thrust arrows use logged gimbal)")
    return "\n".join(lines)


VEHICLE_ORIGIN = (0.0, 0.0, 0.0)
DEFAULT_VIEW_ZOOM = 0.82


def origin_view_half_span(
    manifest: dict,
    axis_length: float,
    *,
    view_zoom: float = DEFAULT_VIEW_ZOOM,
    pad: float = 0.025,
    min_half: float = 0.11,
    max_half: float = 0.22,
) -> float:
    """Half-width of a cubic view volume centered on the vehicle origin (nozzle exit)."""
    points: list[tuple[float, float, float]] = [VEHICLE_ORIGIN]
    axis_reach = axis_length * 0.95
    triad_reach = axis_length * 0.85

    for engine in engines_from_manifest(manifest):
        points.append(vec3(engine["position_m"]))

    points.extend(
        [
            (axis_reach, 0.0, 0.0),
            (0.0, axis_reach, 0.0),
            (0.0, 0.0, axis_reach),
            (triad_reach, 0.0, 0.0),
            (0.0, triad_reach, 0.0),
            (0.0, 0.0, triad_reach),
        ]
    )

    max_abs = max(max(abs(component) for component in point) for point in points)
    return max(min_half, min(max_half, max_abs * view_zoom + pad))


def draw_replay_engine_thrust(
    scene: PyVistaScene,
    engine: EngineGeometry,
    *,
    roll_deg: float,
    yaw_deg: float,
    thrust_n: float,
    color: str,
    axis_length: float,
    ref_thrust_n: float,
    show_gimbal_axes: bool,
) -> list:
    from tools.pyvista_draw import EngineActuatorControl

    if thrust_n <= 1e-3:
        return []
    direction = plant_thrust_direction(engine, roll_deg, yaw_deg)
    scale = thrust_n / max(ref_thrust_n, 1.0)
    arrow_length = axis_length * 1.2 * min(max(scale, 0.05), 2.5)
    artists: list = []
    if show_gimbal_axes and (abs(roll_deg) > 0.05 or abs(yaw_deg) > 0.05):
        control = EngineActuatorControl(
            engine=engine,
            engine_id="engine",
            color=color,
            roll_deg=roll_deg,
            yaw_deg=yaw_deg,
        )
        artists.extend(draw_coupled_yaw_axis(scene, control, axis_length))
    artists.extend(draw_vector(scene, engine.position_m, direction, arrow_length, color, None))
    return artists


def draw_dynamic_frame(
    scene: PyVistaScene,
    frame: ReplayFrame,
    engines: Sequence[EngineGeometry],
    manifest: dict,
    *,
    axis_length: float,
    build_stage: int,
) -> None:

    ref_thrust_n = float(manifest["vehicle"]["ca_reference_thrust_n"])
    engine_manifests = engines_from_manifest(manifest)
    engine_colors = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")

    north, east, down = world_axes_in_body(frame.quaternion)
    length = axis_length * 0.85
    for direction, color, label in (
        (north, "#6a9bd1", "world N"),
        (east, "#6ab86a", "world E"),
        (down, "#b08cd1", "world D"),
    ):
        draw_vector(scene, (0.0, 0.0, 0.0), (float(direction[0]), float(direction[1]), float(direction[2])), length, color, label, alpha=0.75)

    for index, engine in enumerate(engines):
        roll_deg = frame.engine_roll_deg[index] if index < len(frame.engine_roll_deg) else 0.0
        yaw_deg = frame.engine_yaw_deg[index] if index < len(frame.engine_yaw_deg) else 0.0
        thrust_n = frame.engine_thrust_n[index] if index < len(frame.engine_thrust_n) else 0.0
        color = engine_colors[index % len(engine_colors)]
        draw_replay_engine_thrust(
            scene,
            engine,
            roll_deg=roll_deg,
            yaw_deg=yaw_deg,
            thrust_n=thrust_n,
            color=color,
            axis_length=axis_length,
            ref_thrust_n=ref_thrust_n,
            show_gimbal_axes=build_stage >= 3,
        )


DEFAULT_CAMERA = "forward_up"
REPLAY_CAMERA_PRESETS = CAMERA_PRESETS


def default_output_path(log_path: Path, suffix: str) -> Path:
    return log_path.with_name(f"{log_path.stem}.tv3_engines{suffix}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", nargs="?", type=Path, help="PX4 .ulg path (default: latest archived SITL log)")
    parser.add_argument("--latest", action="store_true", help="Use the newest archived SITL log")
    parser.add_argument("--vehicle", type=Path, help="Vehicle manifest JSON (default: vehicle.json beside log)")
    parser.add_argument("-o", "--output", type=Path, help="Export .rrd (Rerun) or .png (PyVista snapshot)")
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Use Rerun timed playback instead of the default PyVista interactive 3D view",
    )
    parser.add_argument("--camera", choices=tuple(REPLAY_CAMERA_PRESETS), default=DEFAULT_CAMERA, help="PyVista camera preset")
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Replay sample rate when building frames (0 = native fastest ULog topic, typically 50 Hz)",
    )
    parser.add_argument("--stride", type=int, default=1, help="Use every Nth replay frame")
    parser.add_argument("--axis-length", type=float, default=0.12, help="Arrow length for frame and thrust axes (m)")
    parser.add_argument("--build-stage", type=int, choices=(1, 2, 3), default=MAX_BUILD_STAGE)
    parser.add_argument(
        "--time",
        type=float,
        help="Snapshot time in seconds from log start (PyVista view/PNG; default: last frame)",
    )
    return parser


def run_engines_replay(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    log_path = find_latest_ulog() if args.latest or args.ulog is None else args.ulog
    if not log_path.exists():
        raise SystemExit(f"ULog not found: {log_path}")

    manifest = resolve_manifest(log_path, args.vehicle)
    engines = engines_from_vehicle(manifest)
    if not engines:
        raise SystemExit("vehicle manifest has no engines to plot")

    ulog = import_ulog()(str(log_path))
    frames = build_replay_frames(ulog, manifest, fps=args.fps, stride=args.stride)
    if not frames:
        raise SystemExit("no replay frames could be built from the ULog")

    index = frame_index_at_time(frames, args.time) if args.time is not None else len(frames) - 1
    output_kind = args.output.suffix.lower() if args.output is not None else None

    if output_kind == ".png":
        out = args.output or default_output_path(log_path, f"_t{args.time:g}.png" if args.time else ".png")
        render_engine_preview(
            frames[index],
            engines,
            manifest,
            out,
            axis_length=args.axis_length,
            camera=args.camera,
            interactive=False,
        )
        return 0

    if args.rerun or output_kind == ".rrd":
        print(format_replay_sampling(frames, fps=args.fps))

    if not args.rerun and output_kind != ".rrd":
        render_engine_preview(
            frames[index],
            engines,
            manifest,
            None,
            axis_length=args.axis_length,
            camera=args.camera,
            interactive=True,
        )
        return 0

    recording_path = args.output if output_kind == ".rrd" else None
    replay_engines(
        frames,
        engines,
        manifest,
        log_path.name,
        axis_length=args.axis_length,
        build_stage=args.build_stage,
        recording_path=recording_path,
        spawn=recording_path is None,
    )
    if recording_path is not None:
        print(f"wrote {recording_path}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    import warnings

    warnings.warn("plot_ulog_engines is deprecated; use tv3_replay --scene engines", DeprecationWarning, stacklevel=2)
    return run_engines_replay(argv)


if __name__ == "__main__":
    raise SystemExit(main())