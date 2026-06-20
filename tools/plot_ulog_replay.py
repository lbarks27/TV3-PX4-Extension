#!/usr/bin/env python3
"""Interactive TV3 ULog replay scenes (trajectory, guidance, engines)."""

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
    InteractiveReplayShell,
    TV3_MODE_LABELS,
    altitude_from_ned,
    body_axes_in_world,
    build_query_times,
    euler_angles_deg,
    find_latest_ulog,
    frame_index_at_time,
    import_pyplot,
    import_ulog,
    interpolate_series,
    ned_to_plot_xyz,
    resolve_manifest,
    save_animation,
    scalar_series_or_zeros,
    topic_dataset,
    topic_field,
    topic_times_us,
)
from tools.view_vehicle_frame import (  # noqa: E402
    DARK_BG,
    DARK_GRID,
    DARK_MUTED,
    DARK_TEXT,
    apply_dark_theme,
    apply_zoom_limits,
    clear_artists,
    draw_vector,
    zoom_limits,
)

TRAJECTORY_CAMERA_PRESETS = {
    "overview": (25, -55),
    "track": (20, -70),
}
DEFAULT_TRAJECTORY_CAMERA = "overview"

ArtistList = list[Any]


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


def trajectory_view_center(frames: Sequence[TrajectoryFrame], camera: str, frame_index: int) -> tuple[float, float, float]:
    if camera == "track":
        north, east, down = frames[frame_index].position_ned
        return ned_to_plot_xyz(north, east, down)
    north = [frame.position_ned[0] for frame in frames]
    east = [frame.position_ned[1] for frame in frames]
    down = [frame.position_ned[2] for frame in frames]
    return ned_to_plot_xyz(float(np.mean(north)), float(np.mean(east)), float(np.mean(down)))


def trajectory_view_half_span(frames: Sequence[TrajectoryFrame], *, pad: float = 2.0, min_half: float = 3.0) -> float:
    north = [frame.position_ned[0] for frame in frames]
    east = [frame.position_ned[1] for frame in frames]
    alt = [altitude_from_ned(frame.position_ned[2]) for frame in frames]
    span = max(max(north) - min(north), max(east) - min(east), max(alt) - min(alt), min_half)
    return span * 0.55 + pad


def draw_trajectory_path(ax, frames: Sequence[TrajectoryFrame]) -> None:
    coords = [ned_to_plot_xyz(*frame.position_ned) for frame in frames]
    xs, ys, zs = zip(*coords)
    ax.plot(xs, ys, zs, color=DARK_MUTED, linewidth=1.2, alpha=0.65, label="path")


def draw_body_triad_at(
    ax,
    position_ned: tuple[float, float, float],
    quat: tuple[float, float, float, float],
    axis_length: float,
) -> ArtistList:
    origin = ned_to_plot_xyz(*position_ned)
    forward, right, down = body_axes_in_world(quat)
    artists: ArtistList = []
    specs = (
        ("F", forward, "#ff7f0e"),
        ("R", right, "#2ca02c"),
        ("D", down, "#d62728"),
    )
    for label, direction, color in specs:
        direction_tuple = (float(direction[0]), float(direction[1]), float(-direction[2]))
        artists.extend(draw_vector(ax, origin, direction_tuple, axis_length, color, f"body {label}", alpha=0.9))
    return artists


def draw_trajectory_markers(ax, frame: TrajectoryFrame, axis_length: float) -> ArtistList:
    artists: ArtistList = []
    vehicle = ned_to_plot_xyz(*frame.position_ned)
    artists.append(
        ax.scatter([vehicle[0]], [vehicle[1]], [vehicle[2]], color="#ff7f0e", s=55, depthshade=False, label="vehicle")
    )
    artists.extend(draw_body_triad_at(ax, frame.position_ned, frame.quaternion, axis_length))

    if frame.setpoint_ned is not None:
        sp = ned_to_plot_xyz(*frame.setpoint_ned)
        artists.append(
            ax.scatter([sp[0]], [sp[1]], [sp[2]], color="#6a9bd1", s=40, marker="x", label="setpoint")
        )
    if frame.target_ned is not None:
        target = ned_to_plot_xyz(*frame.target_ned)
        artists.append(
            ax.scatter([target[0]], [target[1]], [target[2]], color="#b08cd1", s=50, marker="*", label="guidance target")
        )
    return artists


def configure_trajectory_axes(ax, frames: Sequence[TrajectoryFrame], *, camera: str, frame_index: int = 0) -> float:
    ax.set_title("Trajectory replay (NED, altitude up)", color=DARK_TEXT, fontsize=10)
    ax.set_xlabel("North (m)", color=DARK_TEXT)
    ax.set_ylabel("East (m)", color=DARK_TEXT)
    ax.set_zlabel("Altitude (m)", color=DARK_TEXT)
    center = trajectory_view_center(frames, camera, frame_index)
    half_span = trajectory_view_half_span(frames)
    apply_zoom_limits(ax, center, half_span)
    elev, azim = TRAJECTORY_CAMERA_PRESETS[camera]
    ax.view_init(elev=elev, azim=azim)
    return half_span


def build_trajectory_figure(
    frames: list[TrajectoryFrame],
    log_name: str,
    *,
    axis_length: float,
    camera: str,
    include_timeseries: bool,
    show: bool = False,
):
    plt = import_pyplot(show)
    if include_timeseries:
        figure = plt.figure(figsize=(13.5, 10.0))
        figure.patch.set_facecolor(DARK_BG)
        summary_artist = figure.text(
            0.08,
            0.97,
            trajectory_summary_text(frames[0], log_name),
            fontsize=9,
            family="monospace",
            va="top",
            color=DARK_TEXT,
        )
        ax3d = figure.add_axes((0.08, 0.30, 0.88, 0.64), projection="3d")
        ax_ts = figure.add_axes((0.08, 0.12, 0.88, 0.14))
    else:
        figure = plt.figure(figsize=(13.5, 9.0))
        figure.patch.set_facecolor(DARK_BG)
        summary_artist = figure.text(
            0.08,
            0.97,
            trajectory_summary_text(frames[0], log_name),
            fontsize=9,
            family="monospace",
            va="top",
            color=DARK_TEXT,
        )
        ax3d = figure.add_axes((0.08, 0.08, 0.88, 0.84), projection="3d")
        ax_ts = None

    draw_trajectory_path(ax3d, frames)
    view_half = configure_trajectory_axes(ax3d, frames, camera=camera)

    cursor = None
    if ax_ts is not None:
        times = [frame.time_s for frame in frames]
        altitude_series = [altitude_from_ned(frame.position_ned[2]) for frame in frames]
        ax_ts.plot(times, altitude_series, label="altitude m", color="#6a9bd1", linewidth=1.4)
        ax_ts.set_xlabel("time since log start (s)")
        ax_ts.set_ylabel("altitude m")
        ax_ts.legend(loc="upper right", fontsize=8)
        cursor = ax_ts.axvline(frames[0].time_s, color="#d62728", linewidth=1.2)

    theme_axes: list[Any] = [ax3d]
    if ax_ts is not None:
        theme_axes.append(ax_ts)
    apply_dark_theme(figure, theme_axes, [summary_artist])

    dynamic_artists: ArtistList = draw_trajectory_markers(ax3d, frames[0], axis_length)
    return figure, ax3d, ax_ts, summary_artist, cursor, dynamic_artists, view_half, camera


def refresh_trajectory_artists(
    ax3d,
    frame: TrajectoryFrame,
    dynamic_artists: ArtistList,
    *,
    axis_length: float,
    frames: Sequence[TrajectoryFrame],
    camera: str,
    frame_index: int,
    view_half_holder: dict[str, float],
) -> None:
    clear_artists(dynamic_artists)
    dynamic_artists.clear()
    dynamic_artists.extend(draw_trajectory_markers(ax3d, frame, axis_length))
    if camera == "track":
        center = trajectory_view_center(frames, camera, frame_index)
        apply_zoom_limits(ax3d, center, view_half_holder["half"])


def run_trajectory_interactive(
    frames: list[TrajectoryFrame],
    log_path: Path,
    *,
    axis_length: float,
    camera: str,
    fps: float,
) -> None:
    import_pyplot(True)
    figure, ax3d, ax_ts, summary_artist, cursor, dynamic_artists, default_half, camera_name = build_trajectory_figure(
        frames,
        log_path.name,
        axis_length=axis_length,
        camera=camera,
        include_timeseries=True,
        show=True,
    )
    view_half_holder = {"half": default_half}
    shell_holder: dict[str, InteractiveReplayShell] = {}

    def show_frame(frame_index: int) -> None:
        frame = frames[frame_index]
        refresh_trajectory_artists(
            ax3d,
            frame,
            dynamic_artists,
            axis_length=axis_length,
            frames=frames,
            camera=camera_name,
            frame_index=frame_index,
            view_half_holder=view_half_holder,
        )
        summary_artist.set_text(trajectory_summary_text(frame, log_path.name))
        if cursor is not None:
            cursor.set_xdata([frame.time_s, frame.time_s])
        shell_holder["shell"].sync_slider(frame.time_s)
        figure.canvas.draw_idle()

    def on_scroll(event) -> None:
        if event.inaxes != ax3d:
            return
        step = getattr(event, "step", 0)
        slider_val = shell_holder["shell"].time_slider.val
        center = trajectory_view_center(frames, camera_name, frame_index_at_time(frames, slider_val))
        if step > 0 or event.button == "up":
            view_half_holder["half"] = zoom_limits(ax3d, center, view_half_holder["half"], 0.9)
        elif step < 0 or event.button == "down":
            view_half_holder["half"] = zoom_limits(ax3d, center, view_half_holder["half"], 1.1)

    def on_key_extra(event) -> None:
        if event.key == "r":
            slider_val = shell_holder["shell"].time_slider.val
            center = trajectory_view_center(frames, camera_name, frame_index_at_time(frames, slider_val))
            view_half_holder["half"] = apply_zoom_limits(ax3d, center, default_half)
            figure.canvas.draw_idle()

    shell_holder["shell"] = InteractiveReplayShell(
        figure,
        frames,
        show_frame=show_frame,
        window_title=f"TV3 trajectory replay — {log_path.name}",
        fps=fps,
        on_scroll=on_scroll,
        on_key_extra=on_key_extra,
    )
    show_frame(0)
    shell_holder["shell"].run()


def build_guidance_figure(frames: list[GuidanceFrame], log_name: str, *, show: bool = False):
    plt = import_pyplot(show)
    figure, axes = plt.subplots(4, 1, figsize=(13.5, 10.0), sharex=True)
    figure.patch.set_facecolor(DARK_BG)
    figure.subplots_adjust(left=0.08, right=0.96, top=0.90, bottom=0.12, hspace=0.28)

    summary_artist = figure.text(
        0.08,
        0.96,
        guidance_summary_text(frames[0], log_name),
        fontsize=9,
        family="monospace",
        va="top",
        color=DARK_TEXT,
    )

    times = [frame.time_s for frame in frames]
    mode = [frame.mode for frame in frames]
    phase = [frame.phase for frame in frames]
    required = [frame.required_thrust_n for frame in frames]
    available = [frame.available_thrust_n for frame in frames]
    measured = [frame.measured_thrust_n for frame in frames]
    margin = [frame.thrust_margin_n for frame in frames]
    remaining_dv = [frame.remaining_delta_v_m_s for frame in frames]
    unalloc = [frame.unalloc_torque_norm for frame in frames]
    control_unreachable = [frame.control_unreachable for frame in frames]
    guidance_unreachable = [frame.guidance_unreachable for frame in frames]

    axes[0].step(times, mode, where="post", label="tv3 mode", color="#ff7f0e", linewidth=1.4)
    axes[0].step(times, phase, where="post", label="guidance phase", color="#6a9bd1", linewidth=1.4)
    axes[0].set_ylabel("mode/phase")
    axes[0].legend(loc="upper right", fontsize=8, ncols=2)

    axes[1].plot(times, required, label="required N", color="#d62728", linewidth=1.2)
    axes[1].plot(times, available, label="available N", color="#2ca02c", linewidth=1.2)
    axes[1].plot(times, measured, label="measured N", color="#ff7f0e", linewidth=1.2)
    axes[1].set_ylabel("thrust N")
    axes[1].legend(loc="upper right", fontsize=8, ncols=3)

    axes[2].plot(times, margin, label="thrust margin N", color="#b08cd1", linewidth=1.2)
    axes[2].plot(times, remaining_dv, label="remaining dV m/s", color="#6a9bd1", linewidth=1.2)
    axes[2].set_ylabel("margin")
    axes[2].legend(loc="upper right", fontsize=8, ncols=2)

    axes[3].plot(times, unalloc, label="unalloc torque norm Nm", color="#ff7f0e", linewidth=1.2)
    axes[3].step(times, control_unreachable, where="post", label="control unreachable", color="#d62728", linewidth=1.0)
    axes[3].step(times, guidance_unreachable, where="post", label="guidance unreachable", color="#2ca02c", linewidth=1.0)
    axes[3].set_ylabel("control health")
    axes[3].set_xlabel("time since log start (s)")
    axes[3].legend(loc="upper right", fontsize=8, ncols=3)

    cursors = [ax.axvline(frames[0].time_s, color="#d62728", linewidth=1.2) for ax in axes]
    apply_dark_theme(figure, list(axes), [summary_artist])
    return figure, axes, summary_artist, cursors


def run_guidance_interactive(frames: list[GuidanceFrame], log_path: Path, *, fps: float) -> None:
    import_pyplot(True)
    figure, axes, summary_artist, cursors = build_guidance_figure(frames, log_path.name, show=True)
    shell_holder: dict[str, InteractiveReplayShell] = {}

    def show_frame(frame_index: int) -> None:
        frame = frames[frame_index]
        summary_artist.set_text(guidance_summary_text(frame, log_path.name))
        for cursor in cursors:
            cursor.set_xdata([frame.time_s, frame.time_s])
        shell_holder["shell"].sync_slider(frame.time_s)
        figure.canvas.draw_idle()

    shell_holder["shell"] = InteractiveReplayShell(
        figure,
        frames,
        show_frame=show_frame,
        window_title=f"TV3 guidance replay — {log_path.name}",
        fps=fps,
        help_text="space play/pause | drag slider scrub",
    )
    show_frame(0)
    shell_holder["shell"].run()


def default_output_path(log_path: Path, scene: str, suffix: str) -> Path:
    return log_path.with_name(f"{log_path.stem}.tv3_{scene}{suffix}")


def run_trajectory_export(
    frames: list[TrajectoryFrame],
    log_path: Path,
    *,
    axis_length: float,
    camera: str,
    fps: float,
    output: Path | None,
    show: bool,
    at_time: float | None,
) -> int:
    if at_time is not None:
        figure, ax3d, _ax_ts, summary_artist, _cursor, dynamic_artists, _view_half, _camera = build_trajectory_figure(
            frames,
            log_path.name,
            axis_length=axis_length,
            camera=camera,
            include_timeseries=False,
            show=show,
        )
        index = frame_index_at_time(frames, at_time)
        clear_artists(dynamic_artists)
        dynamic_artists.extend(draw_trajectory_markers(ax3d, frames[index], axis_length))
        summary_artist.set_text(trajectory_summary_text(frames[index], log_path.name))
        out = output or default_output_path(log_path, "trajectory", f"_t{at_time:g}.png")
        out.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(out, dpi=160, facecolor=DARK_BG)
        print(f"wrote {out}")
        if show:
            import_pyplot(True).show()
        return 0

    figure, ax3d, ax_ts, summary_artist, cursor, dynamic_artists, _view_half, _camera = build_trajectory_figure(
        frames,
        log_path.name,
        axis_length=axis_length,
        camera=camera,
        include_timeseries=True,
        show=show,
    )

    from matplotlib import animation

    def update(frame_index: int):
        frame = frames[frame_index]
        clear_artists(dynamic_artists)
        dynamic_artists.extend(draw_trajectory_markers(ax3d, frame, axis_length))
        summary_artist.set_text(trajectory_summary_text(frame, log_path.name))
        if cursor is not None:
            cursor.set_xdata([frame.time_s, frame.time_s])
        return dynamic_artists + ([cursor] if cursor is not None else []) + [summary_artist]

    anim = animation.FuncAnimation(
        figure,
        update,
        frames=len(frames),
        interval=max(1000.0 / fps, 1.0),
        blit=False,
        repeat=True,
    )
    out = output or default_output_path(log_path, "trajectory", ".gif")
    if out.suffix.lower() == ".png":
        update(0)
        out.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(out, dpi=160, facecolor=DARK_BG)
        print(f"wrote {out}")
    else:
        save_animation(figure, anim, out, fps)
        print(f"wrote {out}")
    if show:
        import_pyplot(True).show()
    return 0


def run_guidance_export(
    frames: list[GuidanceFrame],
    log_path: Path,
    *,
    fps: float,
    output: Path | None,
    show: bool,
    at_time: float | None,
) -> int:
    if at_time is not None:
        figure, _axes, summary_artist, cursors = build_guidance_figure(frames, log_path.name, show=show)
        index = frame_index_at_time(frames, at_time)
        frame = frames[index]
        summary_artist.set_text(guidance_summary_text(frame, log_path.name))
        for cursor in cursors:
            cursor.set_xdata([frame.time_s, frame.time_s])
        out = output or default_output_path(log_path, "guidance", f"_t{at_time:g}.png")
        out.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(out, dpi=160, facecolor=DARK_BG)
        print(f"wrote {out}")
        if show:
            import_pyplot(True).show()
        return 0

    figure, _axes, summary_artist, cursors = build_guidance_figure(frames, log_path.name, show=show)

    from matplotlib import animation

    def update(frame_index: int):
        frame = frames[frame_index]
        summary_artist.set_text(guidance_summary_text(frame, log_path.name))
        for cursor in cursors:
            cursor.set_xdata([frame.time_s, frame.time_s])
        return cursors + [summary_artist]

    anim = animation.FuncAnimation(
        figure,
        update,
        frames=len(frames),
        interval=max(1000.0 / fps, 1.0),
        blit=False,
        repeat=True,
    )
    out = output or default_output_path(log_path, "guidance", ".gif")
    if out.suffix.lower() == ".png":
        update(0)
        out.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(out, dpi=160, facecolor=DARK_BG)
        print(f"wrote {out}")
    else:
        save_animation(figure, anim, out, fps)
        print(f"wrote {out}")
    if show:
        import_pyplot(True).show()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", nargs="?", type=Path, help="PX4 .ulg path (default: latest archived SITL log)")
    parser.add_argument("--latest", action="store_true", help="Use the newest archived SITL log")
    parser.add_argument(
        "--scene",
        choices=("trajectory", "guidance", "engines"),
        default="trajectory",
        help="Replay scene to render (default: %(default)s)",
    )
    parser.add_argument("--vehicle", type=Path, help="Vehicle manifest JSON (trajectory/engines only)")
    parser.add_argument("-o", "--output", type=Path, help="Output .mp4, .gif, or .png path")
    parser.add_argument("--show", action="store_true", help="Open the interactive replay viewer")
    parser.add_argument("--interactive", action="store_true", help="Alias for --show")
    parser.add_argument(
        "--camera",
        choices=tuple(TRAJECTORY_CAMERA_PRESETS),
        default=DEFAULT_TRAJECTORY_CAMERA,
        help="Trajectory camera preset",
    )
    parser.add_argument("--fps", type=float, default=10.0, help="Animation frames per second")
    parser.add_argument("--stride", type=int, default=1, help="Use every Nth animation frame")
    parser.add_argument("--axis-length", type=float, default=0.8, help="Body triad arrow length (m)")
    parser.add_argument("--time", type=float, help="Export a single PNG at this time (seconds from log start)")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.scene == "engines":
        from tools import plot_ulog_engines

        engine_argv: list[str] = []
        if argv is not None:
            skip_next = False
            for token in argv:
                if skip_next:
                    skip_next = False
                    continue
                if token == "--scene":
                    skip_next = True
                    continue
                engine_argv.append(token)
        return plot_ulog_engines.main(engine_argv or None)

    log_path = find_latest_ulog() if args.latest or args.ulog is None else args.ulog
    if not log_path.exists():
        raise SystemExit(f"ULog not found: {log_path}")

    ULog = import_ulog()
    ulog = ULog(str(log_path))
    interactive = args.show or args.interactive

    if args.scene == "trajectory":
        frames = build_trajectory_frames(ulog, fps=args.fps, stride=args.stride)
        if not frames:
            raise SystemExit("no trajectory frames could be built from the ULog")
        if interactive and args.time is None:
            run_trajectory_interactive(
                frames,
                log_path,
                axis_length=args.axis_length,
                camera=args.camera,
                fps=args.fps,
            )
            return 0
        show = interactive or args.output is None
        return run_trajectory_export(
            frames,
            log_path,
            axis_length=args.axis_length,
            camera=args.camera,
            fps=args.fps,
            output=args.output,
            show=show,
            at_time=args.time,
        )

    frames = build_guidance_frames(ulog, fps=args.fps, stride=args.stride)
    if not frames:
        raise SystemExit("no guidance frames could be built from the ULog")
    if interactive and args.time is None:
        run_guidance_interactive(frames, log_path, fps=args.fps)
        return 0
    show = interactive or args.output is None
    return run_guidance_export(
        frames,
        log_path,
        fps=args.fps,
        output=args.output,
        show=show,
        at_time=args.time,
    )


if __name__ == "__main__":
    raise SystemExit(main())