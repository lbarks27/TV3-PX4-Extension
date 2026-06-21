#!/usr/bin/env python3
"""Unified TV3 ULog replay CLI: trajectory, engines, guidance, and unified Rerun timelines."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.ulog_replay_common import (  # noqa: E402
    CONTROL_UNREACHABLE_LABELS,
    GUIDANCE_PHASE_LABELS,
    GUIDANCE_UNREACHABLE_LABELS,
    TV3_MODE_LABELS,
    altitude_from_ned,
    build_query_times,
    euler_angles_deg,
    format_replay_sampling,
    find_latest_ulog,
    frame_index_at_time,
    import_ulog,
    interpolate_series,
    scalar_series_or_zeros,
    topic_dataset,
    topic_field,
    topic_times_us,
)


@dataclass(frozen=True)
class TrajectoryFrame:
    time_s: float
    position_ned: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    setpoint_ned: tuple[float, float, float] | None
    target_ned: tuple[float, float, float] | None
    phase: int
    target_distance_m: float
    cross_track_error_m: float


@dataclass(frozen=True)
class GuidanceFrame:
    time_s: float
    mode: int
    fault_reason: int
    burn_fraction: float
    rail_distance_m: float
    phase: int
    required_thrust_n: float
    available_thrust_n: float
    thrust_margin_n: float
    remaining_delta_v_m_s: float
    measured_thrust_n: float
    filtered_thrust_n: float
    expected_thrust_n: float
    unalloc_torque_norm: float
    control_unreachable: int
    guidance_unreachable: int
    est_torque_pitch_nm: float
    est_torque_yaw_nm: float


def build_trajectory_frames(ulog, *, fps: float, stride: int) -> list[TrajectoryFrame]:
    attitude = topic_dataset(ulog, "vehicle_attitude")
    position = topic_dataset(ulog, "vehicle_local_position")
    guidance = topic_dataset(ulog, "tv3_guidance_status")
    setpoint = topic_dataset(ulog, "trajectory_setpoint")
    if attitude is None or position is None:
        missing = [
            name
            for name, dataset in (
                ("vehicle_attitude", attitude),
                ("vehicle_local_position", position),
            )
            if dataset is None
        ]
        raise SystemExit(f"ULog missing required topics: {', '.join(missing)}")

    start_us, query_us = build_query_times((attitude, position, guidance, setpoint), fps=fps, stride=stride)
    att_times = topic_times_us(attitude)
    pos_times = topic_times_us(position)

    quat = np.vstack([topic_field(attitude, f"q[{index}]") for index in range(4)])
    pos_xyz = np.vstack(
        [
            topic_field(position, "x"),
            topic_field(position, "y"),
            topic_field(position, "z"),
        ]
    )

    quat_i = interpolate_series(att_times, quat, query_us)
    pos_i = interpolate_series(pos_times, pos_xyz, query_us)

    if setpoint is not None:
        sp_times = topic_times_us(setpoint)
        sp_xyz = np.vstack(
            [
                topic_field(setpoint, "position[0]"),
                topic_field(setpoint, "position[1]"),
                topic_field(setpoint, "position[2]"),
            ]
        )
        sp_i = interpolate_series(sp_times, sp_xyz, query_us)
    else:
        sp_i = None

    if guidance is not None:
        g_times = topic_times_us(guidance)
        target_n = scalar_series_or_zeros(guidance, "target_n", g_times)
        target_e = scalar_series_or_zeros(guidance, "target_e", g_times)
        target_d = scalar_series_or_zeros(guidance, "target_d", g_times)
        phase = scalar_series_or_zeros(guidance, "phase", g_times)
        target_distance = scalar_series_or_zeros(guidance, "target_distance_m", g_times)
        cross_track = scalar_series_or_zeros(guidance, "cross_track_error_m", g_times)
        target_i = interpolate_series(g_times, np.vstack([target_n, target_e, target_d]), query_us)
        phase_i = interpolate_series(g_times, phase, query_us)
        target_distance_i = interpolate_series(g_times, target_distance, query_us)
        cross_track_i = interpolate_series(g_times, cross_track, query_us)
    else:
        target_i = phase_i = target_distance_i = cross_track_i = None

    frames: list[TrajectoryFrame] = []
    for index, time_us in enumerate(query_us):
        time_s = (time_us - start_us) * 1e-6
        setpoint_ned = None
        if sp_i is not None:
            setpoint_ned = (float(sp_i[0, index]), float(sp_i[1, index]), float(sp_i[2, index]))
        target_ned = None
        if target_i is not None:
            target_ned = (float(target_i[0, index]), float(target_i[1, index]), float(target_i[2, index]))
        frames.append(
            TrajectoryFrame(
                time_s=time_s,
                position_ned=(float(pos_i[0, index]), float(pos_i[1, index]), float(pos_i[2, index])),
                quaternion=tuple(float(quat_i[component, index]) for component in range(4)),
                setpoint_ned=setpoint_ned,
                target_ned=target_ned,
                phase=int(round(phase_i[index])) if phase_i is not None else -1,
                target_distance_m=float(target_distance_i[index]) if target_distance_i is not None else float("nan"),
                cross_track_error_m=float(cross_track_i[index]) if cross_track_i is not None else float("nan"),
            )
        )
    return frames


def build_guidance_frames(ulog, *, fps: float, stride: int) -> list[GuidanceFrame]:
    status = topic_dataset(ulog, "tv3_status")
    guidance = topic_dataset(ulog, "tv3_guidance_status")
    thrust = topic_dataset(ulog, "tv3_thrust")
    allocator = topic_dataset(ulog, "control_allocator_status")
    if status is None and guidance is None:
        raise SystemExit("ULog missing required topics: tv3_status and/or tv3_guidance_status")

    start_us, query_us = build_query_times((status, guidance, thrust, allocator), fps=fps, stride=stride)

    def interp_scalar(dataset, field_name: str) -> np.ndarray:
        if dataset is None:
            return np.zeros_like(query_us)
        times = topic_times_us(dataset)
        values = scalar_series_or_zeros(dataset, field_name, times)
        return interpolate_series(times, values, query_us)

    unalloc_x = interp_scalar(allocator, "unallocated_torque[0]")
    unalloc_y = interp_scalar(allocator, "unallocated_torque[1]")
    unalloc_z = interp_scalar(allocator, "unallocated_torque[2]")
    unalloc_norm = np.sqrt(unalloc_x**2 + unalloc_y**2 + unalloc_z**2)

    frames: list[GuidanceFrame] = []
    for index, time_us in enumerate(query_us):
        time_s = (time_us - start_us) * 1e-6
        frames.append(
            GuidanceFrame(
                time_s=time_s,
                mode=int(round(interp_scalar(status, "mode")[index])),
                fault_reason=int(round(interp_scalar(status, "fault_reason")[index])),
                burn_fraction=float(interp_scalar(status, "burn_fraction")[index]),
                rail_distance_m=float(interp_scalar(status, "rail_distance_m")[index]),
                phase=int(round(interp_scalar(guidance, "phase")[index])),
                required_thrust_n=float(interp_scalar(guidance, "required_thrust_n")[index]),
                available_thrust_n=float(interp_scalar(guidance, "available_thrust_n")[index]),
                thrust_margin_n=float(interp_scalar(guidance, "thrust_margin_n")[index]),
                remaining_delta_v_m_s=float(interp_scalar(guidance, "remaining_delta_v_m_s")[index]),
                measured_thrust_n=float(interp_scalar(thrust, "measured_thrust_n")[index]),
                filtered_thrust_n=float(interp_scalar(thrust, "filtered_thrust_n")[index]),
                expected_thrust_n=float(interp_scalar(thrust, "expected_thrust_n")[index]),
                unalloc_torque_norm=float(unalloc_norm[index]),
                control_unreachable=int(round(interp_scalar(guidance, "control_unreachable_reason")[index])),
                guidance_unreachable=int(round(interp_scalar(guidance, "guidance_unreachable_reason")[index])),
                est_torque_pitch_nm=float(interp_scalar(guidance, "estimated_torque_pitch_nm")[index]),
                est_torque_yaw_nm=float(interp_scalar(guidance, "estimated_torque_yaw_nm")[index]),
            )
        )
    return frames


def trajectory_summary_text(frame: TrajectoryFrame, log_name: str) -> str:
    north_m, east_m, down_m = frame.position_ned
    roll_deg, pitch_deg, yaw_deg = euler_angles_deg(frame.quaternion)
    phase_label = GUIDANCE_PHASE_LABELS.get(frame.phase, f"phase_{frame.phase}")
    lines = [
        "TV3 ULog trajectory replay",
        f"log: {log_name}",
        (
            f"t={frame.time_s:6.2f} s  alt={altitude_from_ned(down_m):6.2f} m  "
            f"phase={phase_label}"
        ),
        (
            f"NED pos (m): N={north_m:+.2f}  E={east_m:+.2f}  D={down_m:+.2f}  "
            f"att (deg): roll={roll_deg:+.1f}  pitch={pitch_deg:+.1f}  yaw={yaw_deg:+.1f}"
        ),
    ]
    if not math.isnan(frame.target_distance_m):
        lines.append(
            f"target dist={frame.target_distance_m:6.2f} m  "
            f"cross-track={frame.cross_track_error_m:+.2f} m"
        )
    return "\n".join(lines)


def guidance_summary_text(frame: GuidanceFrame, log_name: str) -> str:
    mode_label = TV3_MODE_LABELS.get(frame.mode, f"mode_{frame.mode}")
    phase_label = GUIDANCE_PHASE_LABELS.get(frame.phase, f"phase_{frame.phase}")
    control_label = CONTROL_UNREACHABLE_LABELS.get(frame.control_unreachable, str(frame.control_unreachable))
    guidance_label = GUIDANCE_UNREACHABLE_LABELS.get(frame.guidance_unreachable, str(frame.guidance_unreachable))
    return "\n".join(
        [
            "TV3 ULog guidance replay",
            f"log: {log_name}",
            (
                f"t={frame.time_s:6.2f} s  mode={mode_label}  phase={phase_label}  "
                f"fault=0x{frame.fault_reason:x}"
            ),
            (
                f"thrust req={frame.required_thrust_n:6.1f} N  avail={frame.available_thrust_n:6.1f} N  "
                f"margin={frame.thrust_margin_n:+.1f} N  measured={frame.measured_thrust_n:6.1f} N"
            ),
            (
                f"remaining dV={frame.remaining_delta_v_m_s:+.2f} m/s  "
                f"unalloc torque={frame.unalloc_torque_norm:.3f} Nm  "
                f"ctrl={control_label}  guid={guidance_label}"
            ),
        ]
    )


from tools.static_preview import render_trajectory_preview  # noqa: E402
from tools.ulog_replay_common import resolve_manifest  # noqa: E402
from tools.viz_common import CAMERA_PRESETS as TRAJECTORY_CAMERA_PRESETS  # noqa: E402

DEFAULT_TRAJECTORY_CAMERA = "overview"


def default_output_path(log_path: Path, scene: str, suffix: str) -> Path:
    return log_path.with_name(f"{log_path.stem}.tv3_{scene}{suffix}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", nargs="?", type=Path, help="PX4 .ulg path (default: latest archived SITL log)")
    parser.add_argument("--latest", action="store_true", help="Use the newest archived SITL log")
    parser.add_argument(
        "--scene",
        choices=("trajectory", "guidance", "engines", "all"),
        default="trajectory",
        help="Replay scene to render; all combines trajectory+engines+guidance in Rerun (default: %(default)s)",
    )
    parser.add_argument("--vehicle", type=Path, help="Vehicle manifest JSON (engines scene only)")
    parser.add_argument("-o", "--output", type=Path, help="Export .rrd (Rerun) or .png (PyVista snapshot)")
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Use Rerun timed playback instead of the default PyVista interactive 3D view",
    )
    parser.add_argument(
        "--camera",
        choices=tuple(TRAJECTORY_CAMERA_PRESETS),
        default=DEFAULT_TRAJECTORY_CAMERA,
        help="PyVista camera preset (interactive view or PNG export)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Replay sample rate when building frames (0 = native fastest ULog topic, typically 50 Hz)",
    )
    parser.add_argument("--stride", type=int, default=1, help="Use every Nth replay frame")
    parser.add_argument("--axis-length", type=float, default=0.8, help="Body triad arrow length (m)")
    parser.add_argument(
        "--time",
        type=float,
        help="Snapshot time in seconds from log start (PyVista view/PNG; default: last frame)",
    )
    return parser


def _output_kind(output: Path | None) -> str | None:
    if output is None:
        return None
    suffix = output.suffix.lower()
    if suffix == ".rrd":
        return "rrd"
    if suffix == ".png":
        return "png"
    return "other"


def _use_rerun(args, *, scene: str) -> bool:
    kind = _output_kind(args.output)
    if scene in {"guidance", "all"}:
        return kind != "png"
    return args.rerun or kind == "rrd"


def _use_pyvista_interactive(args, *, scene: str) -> bool:
    if scene in {"guidance", "all"}:
        return False
    return not _use_rerun(args, scene=scene) and _output_kind(args.output) != "png"


def _recording_path(output: Path | None, log_path: Path, scene: str) -> Path | None:
    if output is None:
        return None
    if output.suffix.lower() == ".rrd":
        return output
    if output.suffix.lower() == ".png":
        return None
    return default_output_path(log_path, scene, ".rrd")


def _argv_tokens(argv: Sequence[str] | None) -> list[str]:
    if argv is not None:
        return list(argv)
    return sys.argv[1:]


def _strip_scene_flag(tokens: Sequence[str]) -> list[str]:
    engine_argv: list[str] = []
    skip_next = False
    for token in tokens:
        if skip_next:
            skip_next = False
            continue
        if token == "--scene":
            skip_next = True
            continue
        engine_argv.append(token)
    return engine_argv


def main(argv: Sequence[str] | None = None) -> int:
    raw_argv = _argv_tokens(argv)
    args = build_parser().parse_args(raw_argv)

    if args.scene == "engines":
        from tools.plot_ulog_engines import run_engines_replay

        return run_engines_replay(_strip_scene_flag(raw_argv))

    log_path = find_latest_ulog() if args.latest or args.ulog is None else args.ulog
    if not log_path.exists():
        raise SystemExit(f"ULog not found: {log_path}")

    ULog = import_ulog()
    ulog = ULog(str(log_path))
    recording_path = _recording_path(args.output, log_path, args.scene)

    if args.scene == "all":
        from tools.plot_ulog_engines import build_replay_frames
        from tools.rerun_replay import replay_unified
        from tools.tv3_control_allocator import engines_from_vehicle

        if _output_kind(args.output) == "png":
            raise SystemExit("unified replay uses Rerun (.rrd) — omit -o or use a .rrd path")
        manifest = resolve_manifest(log_path, args.vehicle)
        engines = engines_from_vehicle(manifest)
        trajectory_frames = build_trajectory_frames(ulog, fps=args.fps, stride=args.stride)
        guidance_frames = build_guidance_frames(ulog, fps=args.fps, stride=args.stride)
        engine_frames = build_replay_frames(ulog, manifest, fps=args.fps, stride=args.stride)
        if not trajectory_frames and not engine_frames and not guidance_frames:
            raise SystemExit("no frames could be built from the ULog for unified replay")
        print(format_replay_sampling(trajectory_frames or engine_frames or guidance_frames, fps=args.fps))
        unified_recording = _recording_path(args.output, log_path, "unified")
        replay_unified(
            trajectory_frames,
            guidance_frames,
            engine_frames,
            engines,
            manifest,
            log_path.name,
            axis_length=args.axis_length,
            build_stage=3,
            recording_path=unified_recording,
            spawn=unified_recording is None,
        )
        if unified_recording:
            print(f"wrote {unified_recording}")
        return 0

    if args.scene == "trajectory":
        from tools.rerun_replay import replay_trajectory

        manifest = None
        try:
            manifest = resolve_manifest(log_path, args.vehicle)
        except SystemExit:
            manifest = None
        frames = build_trajectory_frames(ulog, fps=args.fps, stride=args.stride)
        if not frames:
            raise SystemExit("no trajectory frames could be built from the ULog")
        index = frame_index_at_time(frames, args.time) if args.time is not None else len(frames) - 1
        if _use_rerun(args, scene="trajectory") or _output_kind(args.output) == "rrd":
            print(format_replay_sampling(frames, fps=args.fps))
        if _output_kind(args.output) == "png":
            out = args.output or default_output_path(log_path, "trajectory", f"_t{args.time:g}.png" if args.time else ".png")
            render_trajectory_preview(
                frames,
                out,
                axis_length=args.axis_length,
                camera=args.camera,
                interactive=False,
                frame_index=index,
                manifest=manifest,
            )
            return 0
        if _use_pyvista_interactive(args, scene="trajectory"):
            render_trajectory_preview(
                frames,
                None,
                axis_length=args.axis_length,
                camera=args.camera,
                interactive=True,
                frame_index=index,
                manifest=manifest,
            )
            return 0
        replay_trajectory(
            frames,
            log_path.name,
            axis_length=args.axis_length,
            recording_path=recording_path,
            spawn=recording_path is None,
            manifest=manifest,
        )
        if recording_path:
            print(f"wrote {recording_path}")
        return 0

    from tools.rerun_replay import replay_guidance

    frames = build_guidance_frames(ulog, fps=args.fps, stride=args.stride)
    if not frames:
        raise SystemExit("no guidance frames could be built from the ULog")
    if _output_kind(args.output) == "png":
        raise SystemExit("guidance replay uses Rerun (.rrd) — omit -o or use a .rrd path")
    print(format_replay_sampling(frames, fps=args.fps))
    trajectory_frames = None
    try:
        trajectory_frames = build_trajectory_frames(ulog, fps=args.fps, stride=args.stride)
    except SystemExit:
        trajectory_frames = None
    replay_guidance(
        frames,
        log_path.name,
        recording_path=recording_path,
        spawn=recording_path is None,
        trajectory_frames=trajectory_frames,
    )
    if recording_path:
        print(f"wrote {recording_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())