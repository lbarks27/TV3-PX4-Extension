"""Rerun-based timed playback for archived TV3 ULogs."""

from __future__ import annotations

import math
import shutil
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from tools.scene_builders import (
    color_to_rgba,
    engine_thrust_arrows,
    trajectory_path_points,
    world_triad_arrows_body_frame,
)
from tools.tv3_control_allocator import EngineGeometry, plant_thrust_direction
from tools.ulog_replay_common import (
    CONTROL_UNREACHABLE_LABELS,
    GUIDANCE_PHASE_LABELS,
    GUIDANCE_UNREACHABLE_LABELS,
    TV3_MODE_LABELS,
    altitude_from_ned,
    frame_index_at_time,
    ned_to_plot_xyz,
)
from tools.vehicle_mesh import rerun_mesh_from_path, resolve_vehicle_mesh
from tools.viz_common import ENGINE_COLORS

MISSING_DEP_MSG = (
    "missing dependency: run ./scripts/setup_viz_env.sh (it installs the pinned rerun-sdk + matching viewer). "
    "Do not pip install rerun-sdk globally or into other venvs; Rerun .rrd files are not readable across SDK versions."
)

# Rerun also creates an automatic wall-clock `log_time` timeline (~milliseconds for export).
# Use a dedicated name so replay spans match archived ULog sim duration (seconds).
SIM_TIME_TIMELINE = "sim_time"


def import_rerun():
    try:
        import rerun as rr
    except ImportError as exc:
        raise SystemExit(MISSING_DEP_MSG) from exc
    return rr


def px4_quat_to_xyzw(quat: Sequence[float]) -> tuple[float, float, float, float]:
    w, x, y, z = [float(value) for value in quat]
    return x, y, z, w


def rerun_executable() -> str | None:
    return shutil.which("rerun")


def set_log_time(time_s: float) -> None:
    rr = import_rerun()
    timestamp = float(time_s)
    if hasattr(rr, "set_time"):
        rr.set_time(SIM_TIME_TIMELINE, duration=timestamp)
    else:
        rr.set_time_seconds(SIM_TIME_TIMELINE, timestamp)


def log_scalar(path: str, value: float) -> None:
    rr = import_rerun()
    if hasattr(rr, "Scalars"):
        rr.log(path, rr.Scalars(float(value)))
    else:
        rr.log(path, rr.Scalar(float(value)))


def log_guidance_metrics(frame: Any) -> None:
    log_scalar("guidance/mode", float(frame.mode))
    log_scalar("guidance/phase", float(frame.phase))
    log_scalar("guidance/required_thrust_n", frame.required_thrust_n)
    log_scalar("guidance/available_thrust_n", frame.available_thrust_n)
    log_scalar("guidance/measured_thrust_n", frame.measured_thrust_n)
    log_scalar("guidance/thrust_margin_n", frame.thrust_margin_n)
    log_scalar("guidance/remaining_delta_v_m_s", frame.remaining_delta_v_m_s)
    log_scalar("guidance/unalloc_torque_norm", frame.unalloc_torque_norm)


def init_recording(application_id: str, recording_path: Path | None, *, spawn: bool) -> None:
    rr = import_rerun()
    rr.init(application_id, spawn=False)
    rec = rr.get_global_data_recording()
    if recording_path is not None:
        recording_path.parent.mkdir(parents=True, exist_ok=True)
        rec.save(recording_path)
    if spawn:
        executable = rerun_executable()
        if executable is None:
            raise SystemExit(
                "Rerun viewer not found on PATH. Run ./scripts/setup_viz_env.sh, "
                "then use the repo viz scripts (they prepend the viz venv to PATH)."
            )
        rr.spawn(executable_path=executable, recording=rec)


def log_replay_instructions() -> None:
    rr = import_rerun()
    rr.log(
        "info",
        rr.TextDocument(
            "\n".join(
                [
                    "TV3 ULog replay",
                    f"Select the '{SIM_TIME_TIMELINE}' timeline in the time panel (not log_time).",
                    "sim_time is seconds from log start and matches archived ULog duration.",
                    "Open this .rrd with the rerun viewer from the same SDK version used to write it",
                    "(use the repo scripts or ../.work/tv3-viz-venv/bin/rerun).",
                ]
            )
        ),
        static=True,
    )


def log_world_frame() -> None:
    rr = import_rerun()
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "world/axes",
        rr.Arrows3D(
            origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            vectors=[[5.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 5.0]],
            colors=[[200, 80, 80], [80, 180, 80], [80, 120, 200]],
            labels=["North", "East", "Up"],
        ),
        static=True,
    )


def log_vehicle_body_mesh(manifest: dict | None, mesh_path: Path | None) -> None:
    rr = import_rerun()
    try:
        mesh = rerun_mesh_from_path(mesh_path, manifest)
    except SystemExit:
        return
    rr.log("world/vehicle/body_mesh", mesh, static=True)


def log_engine_mounts(
    engines: Sequence[EngineGeometry],
    *,
    axis_length: float,
    build_stage: int,
) -> None:
    rr = import_rerun()
    for index, engine in enumerate(engines):
        origin = [float(engine.position_m[0]), float(engine.position_m[1]), float(engine.position_m[2])]
        color = color_to_rgba(ENGINE_COLORS[index % len(ENGINE_COLORS)])
        rr.log(
            f"world/vehicle/engines/{index}",
            rr.Points3D([origin], colors=[color], radii=[0.05], labels=[f"engine_{index}"]),
            static=True,
        )
        if build_stage >= 1:
            rr.log(
                f"world/vehicle/engines/{index}/mount_axis",
                rr.Arrows3D(
                    origins=[origin],
                    vectors=[[axis_length, 0.0, 0.0]],
                    colors=[[180, 180, 180]],
                ),
                static=True,
            )
        if build_stage >= 2:
            direction = plant_thrust_direction(engine, 0.0, 0.0)
            rr.log(
                f"world/vehicle/engines/{index}/thrust_ref",
                rr.Arrows3D(
                    origins=[origin],
                    vectors=[[direction[0] * axis_length * 0.9, direction[1] * axis_length * 0.9, direction[2] * axis_length * 0.9]],
                    colors=[color],
                ),
                static=True,
            )


def _nearest_frame(frames: Sequence[Any], time_s: float) -> Any | None:
    if not frames:
        return None
    return frames[frame_index_at_time(frames, time_s)]


def replay_trajectory(
    frames: Sequence[Any],
    log_name: str,
    *,
    axis_length: float,
    recording_path: Path | None = None,
    spawn: bool = True,
    manifest: dict | None = None,
    mesh_path: Path | None = None,
) -> None:
    del log_name
    rr = import_rerun()
    init_recording("tv3_trajectory_replay", recording_path, spawn=spawn)
    log_replay_instructions()
    log_world_frame()
    log_vehicle_body_mesh(manifest, mesh_path)

    path = trajectory_path_points(frames)
    rr.log("world/path", rr.LineStrips3D([path], colors=[160, 160, 160]), static=True)

    for frame in frames:
        set_log_time(frame.time_s)
        position = ned_to_plot_xyz(*frame.position_ned)
        x, y, z, w = px4_quat_to_xyzw(frame.quaternion)
        rr.log(
            "world/vehicle",
            rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=(x, y, z, w))),
        )
        rr.log(
            "world/vehicle/body_axes",
            rr.Arrows3D(
                origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                vectors=[
                    [float(axis_length), 0.0, 0.0],
                    [0.0, float(axis_length), 0.0],
                    [0.0, 0.0, float(axis_length)],
                ],
                colors=[[255, 127, 14], [44, 160, 44], [214, 39, 40]],
                labels=["F", "R", "D"],
            ),
        )

        com_x = 0.0
        try:
            if manifest is not None:
                com_x = float(manifest.get("vehicle", {}).get("body_com_x_m", 0.0))
        except Exception:
            com_x = 0.0
        rr.log(
            "world/vehicle/com",
            rr.Points3D([[com_x, 0.0, 0.0]], radii=[0.035], colors=[[255, 200, 50]], labels=["CoM"]),
        )

        if frame.setpoint_ned is not None:
            sp = ned_to_plot_xyz(*frame.setpoint_ned)
            rr.log("world/setpoint", rr.Points3D([sp], colors=[106, 155, 209], radii=[0.4], labels=["setpoint"]))
        if frame.target_ned is not None:
            target = ned_to_plot_xyz(*frame.target_ned)
            rr.log("world/target", rr.Points3D([target], colors=[176, 140, 209], radii=[0.5], labels=["target"]))

        phase_label = GUIDANCE_PHASE_LABELS.get(frame.phase, f"phase_{frame.phase}")
        rr.log(
            "summary",
            rr.TextDocument(
                "\n".join(
                    [
                        f"t={frame.time_s:6.2f} s",
                        f"alt={altitude_from_ned(frame.position_ned[2]):6.2f} m",
                        f"phase={phase_label}",
                    ]
                )
            ),
        )


def replay_guidance(
    frames: Sequence[Any],
    log_name: str,
    *,
    recording_path: Path | None = None,
    spawn: bool = True,
    trajectory_frames: Sequence[Any] | None = None,
) -> None:
    del log_name
    rr = import_rerun()
    init_recording("tv3_guidance_replay", recording_path, spawn=spawn)
    log_replay_instructions()
    if trajectory_frames:
        log_world_frame()
        path = trajectory_path_points(trajectory_frames)
        rr.log("world/path", rr.LineStrips3D([path], colors=[160, 160, 160]), static=True)

    for frame in frames:
        set_log_time(frame.time_s)
        mode_label = TV3_MODE_LABELS.get(frame.mode, f"mode_{frame.mode}")
        phase_label = GUIDANCE_PHASE_LABELS.get(frame.phase, f"phase_{frame.phase}")
        control_label = CONTROL_UNREACHABLE_LABELS.get(frame.control_unreachable, str(frame.control_unreachable))
        guidance_label = GUIDANCE_UNREACHABLE_LABELS.get(frame.guidance_unreachable, str(frame.guidance_unreachable))
        log_guidance_metrics(frame)
        if trajectory_frames:
            traj = _nearest_frame(trajectory_frames, frame.time_s)
            if traj is not None:
                position = ned_to_plot_xyz(*traj.position_ned)
                x, y, z, w = px4_quat_to_xyzw(traj.quaternion)
                rr.log(
                    "world/vehicle",
                    rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=(x, y, z, w))),
                )
        rr.log(
            "summary",
            rr.TextDocument(
                "\n".join(
                    [
                        f"t={frame.time_s:6.2f} s  mode={mode_label}  phase={phase_label}",
                        (
                            f"thrust req={frame.required_thrust_n:6.1f} N  avail={frame.available_thrust_n:6.1f} N  "
                            f"margin={frame.thrust_margin_n:+.1f} N"
                        ),
                        f"ctrl={control_label}  guid={guidance_label}",
                    ]
                )
            ),
        )


def replay_engines(
    frames: Sequence[Any],
    engines: Sequence[EngineGeometry],
    manifest: dict,
    log_name: str,
    *,
    axis_length: float,
    build_stage: int,
    recording_path: Path | None = None,
    spawn: bool = True,
    mesh_path: Path | None = None,
) -> None:
    del log_name
    rr = import_rerun()
    init_recording("tv3_engine_replay", recording_path, spawn=spawn)
    log_replay_instructions()
    log_world_frame()
    log_vehicle_body_mesh(manifest, mesh_path)
    log_engine_mounts(engines, axis_length=axis_length, build_stage=build_stage)

    ref_thrust_n = float(manifest["vehicle"]["ca_reference_thrust_n"])
    for frame in frames:
        set_log_time(frame.time_s)
        position = ned_to_plot_xyz(*frame.position_ned)
        x, y, z, w = px4_quat_to_xyzw(frame.quaternion)
        rr.log(
            "world/vehicle",
            rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=(x, y, z, w))),
        )
        origins, vectors, colors = engine_thrust_arrows(
            engines,
            frame,
            axis_length=axis_length,
            ref_thrust_n=ref_thrust_n,
            build_stage=build_stage,
        )
        if origins:
            rr.log("world/vehicle/thrust", rr.Arrows3D(origins=origins, vectors=vectors, colors=colors))

        triad_origins, triad_vectors, triad_colors, triad_labels = world_triad_arrows_body_frame(
            frame.quaternion,
            axis_length,
        )
        rr.log(
            "world/vehicle/world_triad",
            rr.Arrows3D(
                origins=triad_origins,
                vectors=triad_vectors,
                colors=triad_colors,
                labels=triad_labels,
            ),
        )

        # CoM marker (body frame) for this replay too
        com_x = 0.0
        try:
            if manifest is not None:
                com_x = float(manifest.get("vehicle", {}).get("body_com_x_m", 0.0))
        except Exception:
            com_x = 0.0
        rr.log(
            "world/vehicle/com",
            rr.Points3D([[com_x, 0.0, 0.0]], radii=[0.035], colors=[[255, 200, 50]], labels=["CoM"]),
        )

        total_thrust = float(sum(frame.engine_thrust_n))
        rr.log(
            "summary",
            rr.TextDocument(
                "\n".join(
                    [
                        f"t={frame.time_s:6.2f} s  total thrust={total_thrust:6.1f} N",
                        f"ignition=0b{frame.ignition_mask:03b}  active=0b{frame.active_mask:03b}",
                    ]
                )
            ),
        )


def replay_unified(
    trajectory_frames: Sequence[Any],
    guidance_frames: Sequence[Any],
    engine_frames: Sequence[Any],
    engines: Sequence[EngineGeometry],
    manifest: dict,
    log_name: str,
    *,
    axis_length: float,
    build_stage: int,
    recording_path: Path | None = None,
    spawn: bool = True,
    mesh_path: Path | None = None,
) -> None:
    del log_name
    rr = import_rerun()
    init_recording("tv3_unified_replay", recording_path, spawn=spawn)
    log_replay_instructions()
    log_world_frame()
    log_vehicle_body_mesh(manifest, mesh_path)
    log_engine_mounts(engines, axis_length=axis_length, build_stage=build_stage)

    path = trajectory_path_points(trajectory_frames)
    rr.log("world/path", rr.LineStrips3D([path], colors=[160, 160, 160]), static=True)

    ref_thrust_n = float(manifest["vehicle"]["ca_reference_thrust_n"])
    master_frames = trajectory_frames if trajectory_frames else engine_frames if engine_frames else guidance_frames

    for frame in master_frames:
        time_s = frame.time_s
        set_log_time(time_s)

        traj = _nearest_frame(trajectory_frames, time_s) if trajectory_frames else None
        engine_frame = _nearest_frame(engine_frames, time_s) if engine_frames else None
        guidance_frame = _nearest_frame(guidance_frames, time_s) if guidance_frames else None

        if traj is not None:
            position = ned_to_plot_xyz(*traj.position_ned)
            x, y, z, w = px4_quat_to_xyzw(traj.quaternion)
            rr.log(
                "world/vehicle",
                rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=(x, y, z, w))),
            )
            rr.log(
                "world/vehicle/body_axes",
                rr.Arrows3D(
                    origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                    vectors=[
                        [float(axis_length), 0.0, 0.0],
                        [0.0, float(axis_length), 0.0],
                        [0.0, 0.0, float(axis_length)],
                    ],
                    colors=[[255, 127, 14], [44, 160, 44], [214, 39, 40]],
                    labels=["F", "R", "D"],
                ),
            )
            # Mark the declared body CoM (from manifest) so reviewers can see mass location
            # relative to engine mounts and thrust arrows. This explains "upside down" impressions
            # when the mesh vertices are concentrated near the engine plane (X≈0) while CoM is at +X.
            com_x = 0.0
            try:
                if manifest is not None:
                    com_x = float(manifest.get("vehicle", {}).get("body_com_x_m", 0.0))
            except Exception:
                com_x = 0.0
            rr.log(
                "world/vehicle/com",
                rr.Points3D([[com_x, 0.0, 0.0]], radii=[0.035], colors=[[255, 200, 50]], labels=["CoM"]),
            )
            if traj.setpoint_ned is not None:
                sp = ned_to_plot_xyz(*traj.setpoint_ned)
                rr.log("world/setpoint", rr.Points3D([sp], colors=[106, 155, 209], radii=[0.4], labels=["setpoint"]))
            if traj.target_ned is not None:
                target = ned_to_plot_xyz(*traj.target_ned)
                rr.log("world/target", rr.Points3D([target], colors=[176, 140, 209], radii=[0.5], labels=["target"]))

        if engine_frame is not None:
            origins, vectors, colors = engine_thrust_arrows(
                engines,
                engine_frame,
                axis_length=axis_length,
                ref_thrust_n=ref_thrust_n,
                build_stage=build_stage,
            )
            if origins:
                rr.log("world/vehicle/thrust", rr.Arrows3D(origins=origins, vectors=vectors, colors=colors))
            triad_origins, triad_vectors, triad_colors, triad_labels = world_triad_arrows_body_frame(
                engine_frame.quaternion,
                axis_length,
            )
            rr.log(
                "world/vehicle/world_triad",
                rr.Arrows3D(
                    origins=triad_origins,
                    vectors=triad_vectors,
                    colors=triad_colors,
                    labels=triad_labels,
                ),
            )

        if guidance_frame is not None:
            log_guidance_metrics(guidance_frame)

        summary_lines = [f"t={time_s:6.2f} s"]
        if traj is not None:
            phase_label = GUIDANCE_PHASE_LABELS.get(traj.phase, f"phase_{traj.phase}")
            summary_lines.append(f"alt={altitude_from_ned(traj.position_ned[2]):6.2f} m  phase={phase_label}")
        if guidance_frame is not None:
            mode_label = TV3_MODE_LABELS.get(guidance_frame.mode, f"mode_{guidance_frame.mode}")
            summary_lines.append(
                f"mode={mode_label}  thrust margin={guidance_frame.thrust_margin_n:+.1f} N"
            )
        if engine_frame is not None:
            summary_lines.append(
                f"total thrust={sum(engine_frame.engine_thrust_n):6.1f} N  "
                f"ignition=0b{engine_frame.ignition_mask:03b}"
            )
        rr.log("summary", rr.TextDocument("\n".join(summary_lines)))