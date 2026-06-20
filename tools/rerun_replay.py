"""Rerun-based timed playback for archived TV3 ULogs."""

from __future__ import annotations

import math
import shutil
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from tools.ulog_replay_common import (
    CONTROL_UNREACHABLE_LABELS,
    GUIDANCE_PHASE_LABELS,
    GUIDANCE_UNREACHABLE_LABELS,
    TV3_MODE_LABELS,
    altitude_from_ned,
    body_axes_in_world,
    ned_to_plot_xyz,
    world_axes_in_body,
)
from tools.tv3_control_allocator import EngineGeometry, plant_thrust_direction

MISSING_DEP_MSG = "missing dependency: install rerun-sdk with `python3 -m pip install -r requirements-viz.txt`"


def import_rerun():
    try:
        import rerun as rr
    except ImportError as exc:
        raise SystemExit(MISSING_DEP_MSG) from exc
    return rr


def px4_quat_to_xyzw(quat: Sequence[float]) -> tuple[float, float, float, float]:
    w, x, y, z = [float(value) for value in quat]
    return x, y, z, w


def color_to_rgba(color: str, alpha: float = 1.0) -> list[int]:
    color = color.lstrip("#")
    rgb = [int(color[index : index + 2], 16) for index in (0, 2, 4)]
    return rgb + [int(max(0.0, min(1.0, alpha)) * 255)]


def rerun_executable() -> str | None:
    return shutil.which("rerun")


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


def replay_trajectory(
    frames: Sequence[Any],
    log_name: str,
    *,
    axis_length: float,
    recording_path: Path | None = None,
    spawn: bool = True,
) -> None:
    del log_name
    rr = import_rerun()
    init_recording("tv3_trajectory_replay", recording_path, spawn=spawn)
    log_world_frame()

    path = [ned_to_plot_xyz(*frame.position_ned) for frame in frames]
    rr.log("world/path", rr.LineStrips3D([path], colors=[160, 160, 160]), static=True)

    for frame in frames:
        rr.set_time("log_time", duration=frame.time_s)
        position = ned_to_plot_xyz(*frame.position_ned)
        x, y, z, w = px4_quat_to_xyzw(frame.quaternion)
        rr.log(
            "world/vehicle",
            rr.Transform3D(translation=position, rotation=rr.Quaternion(xyzw=(x, y, z, w))),
        )

        # Place body visual and body axes relative to the vehicle transform (so they follow the path exactly)
        rr.log("world/vehicle/body", rr.InstancePoses3D(translations=[[0.0, 0.0, 0.0]]))

        # Body-fixed axes (F/R/D along body X/Y/Z). Parent transform will orient + place them in world.
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
) -> None:
    del log_name
    rr = import_rerun()
    init_recording("tv3_guidance_replay", recording_path, spawn=spawn)

    for frame in frames:
        rr.set_time("log_time", duration=frame.time_s)
        mode_label = TV3_MODE_LABELS.get(frame.mode, f"mode_{frame.mode}")
        phase_label = GUIDANCE_PHASE_LABELS.get(frame.phase, f"phase_{frame.phase}")
        control_label = CONTROL_UNREACHABLE_LABELS.get(frame.control_unreachable, str(frame.control_unreachable))
        guidance_label = GUIDANCE_UNREACHABLE_LABELS.get(frame.guidance_unreachable, str(frame.guidance_unreachable))
        rr.log("guidance/mode", rr.Scalars(float(frame.mode)))
        rr.log("guidance/phase", rr.Scalars(float(frame.phase)))
        rr.log("guidance/required_thrust_n", rr.Scalars(frame.required_thrust_n))
        rr.log("guidance/available_thrust_n", rr.Scalars(frame.available_thrust_n))
        rr.log("guidance/measured_thrust_n", rr.Scalars(frame.measured_thrust_n))
        rr.log("guidance/thrust_margin_n", rr.Scalars(frame.thrust_margin_n))
        rr.log("guidance/remaining_delta_v_m_s", rr.Scalars(frame.remaining_delta_v_m_s))
        rr.log("guidance/unalloc_torque_norm", rr.Scalars(frame.unalloc_torque_norm))
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


def _engine_arrow(
    engine: EngineGeometry,
    *,
    roll_deg: float,
    yaw_deg: float,
    thrust_n: float,
    ref_thrust_n: float,
    axis_length: float,
    color: str,
) -> tuple[list[float], list[float], list[int]]:
    if thrust_n <= 1e-3:
        return [], [], []
    direction = plant_thrust_direction(engine, roll_deg, yaw_deg)
    scale = thrust_n / max(ref_thrust_n, 1.0)
    length = axis_length * 1.2 * min(max(scale, 0.05), 2.5)
    origin = [float(engine.position_m[0]), float(engine.position_m[1]), float(engine.position_m[2])]
    vector = [direction[0] * length, direction[1] * length, direction[2] * length]
    return [origin], [vector], [color_to_rgba(color)]


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
) -> None:
    del log_name, build_stage
    rr = import_rerun()
    init_recording("tv3_engine_replay", recording_path, spawn=spawn)

    rr.log("vehicle", rr.ViewCoordinates.RIGHT_HAND_Z_DOWN, static=True)
    engine_colors = ("#ff7f0e", "#e377c2", "#17becf", "#bcbd22")
    for index, engine in enumerate(engines):
        origin = [float(engine.position_m[0]), float(engine.position_m[1]), float(engine.position_m[2])]
        color = color_to_rgba(engine_colors[index % len(engine_colors)])
        rr.log(
            f"vehicle/engines/{index}",
            rr.Points3D([origin], colors=[color], radii=[0.05], labels=[f"engine_{index}"]),
            static=True,
        )
        rr.log(
            f"vehicle/engines/{index}/mount_axis",
            rr.Arrows3D(
                origins=[origin],
                vectors=[[axis_length, 0.0, 0.0]],
                colors=[[180, 180, 180]],
            ),
            static=True,
        )

    ref_thrust_n = float(manifest["vehicle"]["ca_reference_thrust_n"])
    for frame in frames:
        rr.set_time("log_time", duration=frame.time_s)
        origins: list[list[float]] = []
        vectors: list[list[float]] = []
        colors: list[list[int]] = []
        for index, engine in enumerate(engines):
            roll_deg = frame.engine_roll_deg[index] if index < len(frame.engine_roll_deg) else 0.0
            yaw_deg = frame.engine_yaw_deg[index] if index < len(frame.engine_yaw_deg) else 0.0
            thrust_n = frame.engine_thrust_n[index] if index < len(frame.engine_thrust_n) else 0.0
            o, v, c = _engine_arrow(
                engine,
                roll_deg=roll_deg,
                yaw_deg=yaw_deg,
                thrust_n=thrust_n,
                ref_thrust_n=ref_thrust_n,
                axis_length=axis_length,
                color=engine_colors[index % len(engine_colors)],
            )
            origins.extend(o)
            vectors.extend(v)
            colors.extend(c)

        if origins:
            rr.log("vehicle/thrust", rr.Arrows3D(origins=origins, vectors=vectors, colors=colors))

        north, east, down = world_axes_in_body(frame.quaternion)
        triad_len = axis_length * 0.85
        rr.log(
            "vehicle/world_triad",
            rr.Arrows3D(
                origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                vectors=[
                    [float(north[0]) * triad_len, float(north[1]) * triad_len, float(north[2]) * triad_len],
                    [float(east[0]) * triad_len, float(east[1]) * triad_len, float(east[2]) * triad_len],
                    [float(down[0]) * triad_len, float(down[1]) * triad_len, float(down[2]) * triad_len],
                ],
                colors=[[106, 155, 209], [106, 184, 106], [176, 140, 209]],
                labels=["world N", "world E", "world D"],
            ),
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