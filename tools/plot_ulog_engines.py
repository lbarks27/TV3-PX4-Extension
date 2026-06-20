#!/usr/bin/env python3
"""Animate TV3 engine mounts and thrust vectors from a PX4 ULog.

Renders in the vehicle reference frame (same layout and styling as
``tools/view_vehicle_frame.py``): body-fixed geometry with logged thrust/gimbal
overlays and a rotating world-NED triad for attitude context.
"""

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

from tools.ulog_replay_common import (  # noqa: E402
    InteractiveReplayShell,
    build_query_times,
    euler_angles_deg,
    find_latest_ulog,
    frame_index_at_time,
    import_pyplot,
    import_ulog,
    interpolate_series,
    load_manifest,
    resolve_manifest,
    rotation_matrix_from_quat,
    save_animation,
    topic_dataset,
    topic_field,
    topic_times_us,
    world_axes_in_body,
)
from tools.tv3_control_allocator import EngineGeometry, engines_from_vehicle, plant_thrust_direction  # noqa: E402
from tools.tv3_engine_frame import MAX_BUILD_STAGE  # noqa: E402
from tools.view_vehicle_frame import (  # noqa: E402
    CAMERA_PRESETS,
    DARK_BG,
    DARK_TEXT,
    ENGINE_COLORS,
    apply_dark_theme,
    apply_zoom_limits,
    body_size_m,
    build_scene_layers,
    clear_artists,
    draw_coupled_yaw_axis,
    draw_vector,
    engines_from_manifest,
    motor_label,
    vec3,
    zoom_limits,
)

REPLAY_CAMERA_PRESETS = {
    **CAMERA_PRESETS,
    "forward_up": (-90, 0),
}
DEFAULT_CAMERA = "forward_up"

ArtistList = list[Any]


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


def draw_world_attitude_triad(ax, frame: ReplayFrame, axis_length: float) -> ArtistList:
    """Show where world N/E/D point as seen from the body frame."""
    north, east, down = world_axes_in_body(frame.quaternion)
    length = axis_length * 0.85
    artists: ArtistList = []
    specs = (
        ("N", north, "#6a9bd1"),
        ("E", east, "#6ab86a"),
        ("D", down, "#b08cd1"),
    )
    for label, direction, color in specs:
        direction_tuple = (float(direction[0]), float(direction[1]), float(direction[2]))
        artists.extend(
            draw_vector(ax, (0.0, 0.0, 0.0), direction_tuple, length, color, f"world {label}", alpha=0.75)
        )
    return artists


def draw_replay_engine_thrust(
    ax,
    engine: EngineGeometry,
    *,
    roll_deg: float,
    yaw_deg: float,
    thrust_n: float,
    color: str,
    axis_length: float,
    ref_thrust_n: float,
    engine_id: str,
    show_gimbal_axes: bool,
) -> ArtistList:
    if thrust_n <= 1e-3:
        return []

    direction = plant_thrust_direction(engine, roll_deg, yaw_deg)
    scale = thrust_n / max(ref_thrust_n, 1.0)
    arrow_length = axis_length * 1.2 * min(max(scale, 0.05), 2.5)
    artists: ArtistList = []
    if show_gimbal_axes and (abs(roll_deg) > 0.05 or abs(yaw_deg) > 0.05):
        from tools.view_vehicle_frame import EngineActuatorControl

        control = EngineActuatorControl(
            engine=engine,
            engine_id=engine_id,
            color=color,
            roll_deg=roll_deg,
            yaw_deg=yaw_deg,
        )
        artists.extend(draw_coupled_yaw_axis(ax, control, axis_length))

    artists.extend(draw_vector(ax, engine.position_m, direction, arrow_length, color, None))
    ox, oy, oz = engine.position_m
    tip = (
        ox + direction[0] * arrow_length,
        oy + direction[1] * arrow_length,
        oz + direction[2] * arrow_length,
    )
    artists.append(
        ax.text(
            tip[0],
            tip[1],
            tip[2],
            f"  {thrust_n:.0f} N",
            color=color,
            fontsize=8,
        )
    )
    return artists


def draw_dynamic_frame(
    ax,
    frame: ReplayFrame,
    engines: Sequence[EngineGeometry],
    manifest: dict,
    *,
    axis_length: float,
    build_stage: int,
) -> dict[str, ArtistList]:
    ref_thrust_n = float(manifest["vehicle"]["ca_reference_thrust_n"])
    engine_manifests = engines_from_manifest(manifest)
    artists: dict[str, ArtistList] = {"world": [], "thrust": []}

    artists["world"].extend(draw_world_attitude_triad(ax, frame, axis_length))

    for index, engine in enumerate(engines):
        roll_deg = frame.engine_roll_deg[index] if index < len(frame.engine_roll_deg) else 0.0
        yaw_deg = frame.engine_yaw_deg[index] if index < len(frame.engine_yaw_deg) else 0.0
        thrust_n = frame.engine_thrust_n[index] if index < len(frame.engine_thrust_n) else 0.0
        engine_id = str(engine_manifests[index].get("id", f"engine_{index}")) if index < len(engine_manifests) else f"engine_{index}"
        color = ENGINE_COLORS[index % len(ENGINE_COLORS)]
        artists["thrust"].extend(
            draw_replay_engine_thrust(
                ax,
                engine,
                roll_deg=roll_deg,
                yaw_deg=yaw_deg,
                thrust_n=thrust_n,
                color=color,
                axis_length=axis_length,
                ref_thrust_n=ref_thrust_n,
                engine_id=engine_id,
                show_gimbal_axes=build_stage >= 3,
            )
        )
    return artists


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


def apply_replay_camera(ax, camera: str) -> None:
    if camera not in REPLAY_CAMERA_PRESETS:
        raise SystemExit(f"unknown camera preset: {camera} (choices: {', '.join(REPLAY_CAMERA_PRESETS)})")
    elev, azim = REPLAY_CAMERA_PRESETS[camera]
    ax.view_init(elev=elev, azim=azim)


def configure_scene_axes(
    ax,
    manifest: dict,
    axis_length: float,
    *,
    view_zoom: float = DEFAULT_VIEW_ZOOM,
    camera: str = DEFAULT_CAMERA,
) -> float:
    ax.set_title("Vehicle frame replay (body-fixed, origin-centered)", color=DARK_TEXT, fontsize=10)
    ax.set_xlabel("X forward (m)", color=DARK_TEXT)
    ax.set_ylabel("Y right (m)", color=DARK_TEXT)
    ax.set_zlabel("Z down (m)", color=DARK_TEXT)
    half_span = origin_view_half_span(manifest, axis_length, view_zoom=view_zoom)
    apply_zoom_limits(ax, VEHICLE_ORIGIN, half_span)
    apply_replay_camera(ax, camera)
    return half_span


def refresh_dynamic_artists(
    ax,
    frame: ReplayFrame,
    engines: Sequence[EngineGeometry],
    manifest: dict,
    dynamic_artists: dict[str, ArtistList],
    *,
    axis_length: float,
    build_stage: int,
) -> None:
    for group in dynamic_artists.values():
        clear_artists(group)
    dynamic_artists.clear()
    dynamic_artists.update(
        draw_dynamic_frame(
            ax,
            frame,
            engines,
            manifest,
            axis_length=axis_length,
            build_stage=build_stage,
        )
    )


def build_replay_figure(
    frames: list[ReplayFrame],
    engines: Sequence[EngineGeometry],
    manifest: dict,
    log_name: str,
    *,
    axis_length: float,
    build_stage: int,
    include_timeseries: bool,
    view_zoom: float,
    camera: str,
    show: bool = False,
):
    plt = import_pyplot(show)
    body_size = body_size_m(manifest, None)

    if include_timeseries:
        figure = plt.figure(figsize=(13.5, 10.0))
        figure.patch.set_facecolor(DARK_BG)
        summary_artist = figure.text(
            0.08,
            0.97,
            replay_summary_text(manifest, frames[0], log_name, build_stage=build_stage),
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
            replay_summary_text(manifest, frames[0], log_name, build_stage=build_stage),
            fontsize=9,
            family="monospace",
            va="top",
            color=DARK_TEXT,
        )
        ax3d = figure.add_axes((0.08, 0.08, 0.88, 0.84), projection="3d")
        ax_ts = None

    build_scene_layers(ax3d, manifest, body_size, axis_length, build_stage=build_stage)
    view_half = configure_scene_axes(ax3d, manifest, axis_length, view_zoom=view_zoom, camera=camera)

    cursor = None
    if ax_ts is not None:
        times = [frame.time_s for frame in frames]
        altitude_series = [-frame.position_ned[2] for frame in frames]
        total_thrust_series = [sum(frame.engine_thrust_n) for frame in frames]
        ax_ts.plot(times, altitude_series, label="altitude m", color="#6a9bd1", linewidth=1.4)
        ax_ts.plot(times, total_thrust_series, label="total thrust N", color="#ff7f0e", linewidth=1.4)
        ax_ts.set_xlabel("time since log start (s)")
        ax_ts.set_ylabel("altitude / thrust")
        ax_ts.legend(loc="upper right", fontsize=8)
        cursor = ax_ts.axvline(frames[0].time_s, color="#d62728", linewidth=1.2)

    theme_axes: list[Any] = [ax3d]
    if ax_ts is not None:
        theme_axes.append(ax_ts)
    apply_dark_theme(figure, theme_axes, [summary_artist])

    dynamic_artists: dict[str, ArtistList] = {}
    dynamic_groups = draw_dynamic_frame(
        ax3d,
        frames[0],
        engines,
        manifest,
        axis_length=axis_length,
        build_stage=build_stage,
    )
    dynamic_artists.update(dynamic_groups)

    return figure, ax3d, ax_ts, summary_artist, cursor, dynamic_artists, body_size, view_half


def run_interactive_replay(
    frames: list[ReplayFrame],
    engines: Sequence[EngineGeometry],
    manifest: dict,
    log_path: Path,
    *,
    axis_length: float,
    build_stage: int,
    view_zoom: float,
    camera: str,
    fps: float,
) -> None:
    import_pyplot(True)
    figure, ax3d, ax_ts, summary_artist, cursor, dynamic_artists, _body_size, default_half = build_replay_figure(
        frames,
        engines,
        manifest,
        log_path.name,
        axis_length=axis_length,
        build_stage=build_stage,
        include_timeseries=True,
        view_zoom=view_zoom,
        camera=camera,
        show=True,
    )

    view_half_holder = {"half": default_half}
    shell_holder: dict[str, InteractiveReplayShell] = {}

    def show_frame(frame_index: int) -> None:
        frame = frames[frame_index]
        refresh_dynamic_artists(
            ax3d,
            frame,
            engines,
            manifest,
            dynamic_artists,
            axis_length=axis_length,
            build_stage=build_stage,
        )
        summary_artist.set_text(replay_summary_text(manifest, frame, log_path.name, build_stage=build_stage))
        if cursor is not None:
            cursor.set_xdata([frame.time_s, frame.time_s])
        shell_holder["shell"].sync_slider(frame.time_s)
        figure.canvas.draw_idle()

    def on_scroll(event) -> None:
        if event.inaxes != ax3d:
            return
        step = getattr(event, "step", 0)
        if step > 0 or event.button == "up":
            view_half_holder["half"] = zoom_limits(ax3d, VEHICLE_ORIGIN, view_half_holder["half"], 0.9)
        elif step < 0 or event.button == "down":
            view_half_holder["half"] = zoom_limits(ax3d, VEHICLE_ORIGIN, view_half_holder["half"], 1.1)

    def on_key_extra(event) -> None:
        if event.key == "r":
            view_half_holder["half"] = apply_zoom_limits(ax3d, VEHICLE_ORIGIN, default_half)
            figure.canvas.draw_idle()

    shell_holder["shell"] = InteractiveReplayShell(
        figure,
        frames,
        show_frame=show_frame,
        window_title=f"TV3 engine replay — {log_path.name}",
        fps=fps,
        on_scroll=on_scroll,
        on_key_extra=on_key_extra,
    )
    show_frame(0)
    shell_holder["shell"].run()


def total_thrust(frame: ReplayFrame) -> float:
    return float(sum(frame.engine_thrust_n))


def default_output_path(log_path: Path, suffix: str) -> Path:
    return log_path.with_name(f"{log_path.stem}.tv3_engines{suffix}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", nargs="?", type=Path, help="PX4 .ulg path (default: latest archived SITL log)")
    parser.add_argument("--latest", action="store_true", help="Use the newest archived SITL log")
    parser.add_argument("--vehicle", type=Path, help="Vehicle manifest JSON (default: vehicle.json beside log)")
    parser.add_argument("-o", "--output", type=Path, help="Output .mp4, .gif, or .png path")
    parser.add_argument("--show", action="store_true", help="Open the interactive replay viewer")
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Alias for --show (slider scrub, play/pause, zoom)",
    )
    parser.add_argument(
        "--camera",
        choices=tuple(REPLAY_CAMERA_PRESETS),
        default=DEFAULT_CAMERA,
        help="3D camera preset (default: %(default)s puts +X forward up)",
    )
    parser.add_argument("--fps", type=float, default=10.0, help="Animation frames per second")
    parser.add_argument("--stride", type=int, default=1, help="Use every Nth animation frame")
    parser.add_argument("--axis-length", type=float, default=0.12, help="Arrow length for frame and thrust axes (m)")
    parser.add_argument(
        "--view-zoom",
        type=float,
        default=DEFAULT_VIEW_ZOOM,
        help="Zoom factor around vehicle origin (<1 tighter, >1 wider; default: %(default)s)",
    )
    parser.add_argument(
        "--build-stage",
        type=int,
        choices=(1, 2, 3),
        default=MAX_BUILD_STAGE,
        help="Engine-frame axis detail (matches view_vehicle_frame build stages)",
    )
    parser.add_argument("--time", type=float, help="Export a single PNG at this time (seconds from log start)")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    log_path = find_latest_ulog() if args.latest or args.ulog is None else args.ulog
    if not log_path.exists():
        raise SystemExit(f"ULog not found: {log_path}")

    manifest = resolve_manifest(log_path, args.vehicle)
    engines = engines_from_vehicle(manifest)
    if not engines:
        raise SystemExit("vehicle manifest has no engines to plot")

    ULog = import_ulog()
    ulog = ULog(str(log_path))
    frames = build_replay_frames(ulog, manifest, fps=args.fps, stride=args.stride)
    if not frames:
        raise SystemExit("no replay frames could be built from the ULog")

    interactive = args.show or args.interactive
    show = interactive or args.output is None
    include_timeseries = args.time is None

    if interactive and args.time is None:
        run_interactive_replay(
            frames,
            engines,
            manifest,
            log_path,
            axis_length=args.axis_length,
            build_stage=args.build_stage,
            view_zoom=args.view_zoom,
            camera=args.camera,
            fps=args.fps,
        )
        return 0

    figure, ax3d, ax_ts, summary_artist, cursor, dynamic_artists, _body_size, _view_half = build_replay_figure(
        frames,
        engines,
        manifest,
        log_path.name,
        axis_length=args.axis_length,
        build_stage=args.build_stage,
        include_timeseries=include_timeseries,
        view_zoom=args.view_zoom,
        camera=args.camera,
    )

    if args.time is not None:
        index = min(range(len(frames)), key=lambda i: abs(frames[i].time_s - args.time))
        for group in dynamic_artists.values():
            clear_artists(group)
        dynamic_artists.clear()
        dynamic_artists.update(
            draw_dynamic_frame(
                ax3d,
                frames[index],
                engines,
                manifest,
                axis_length=args.axis_length,
                build_stage=args.build_stage,
            )
        )
        summary_artist.set_text(replay_summary_text(manifest, frames[index], log_path.name, build_stage=args.build_stage))
        output = args.output or default_output_path(log_path, f"_t{args.time:g}.png")
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=160, facecolor=DARK_BG)
        print(f"wrote {output}")
        if show:
            import_pyplot(True).show()
        return 0

    from matplotlib import animation

    def update(frame_index: int):
        refresh_dynamic_artists(
            ax3d,
            frames[frame_index],
            engines,
            manifest,
            dynamic_artists,
            axis_length=args.axis_length,
            build_stage=args.build_stage,
        )
        frame = frames[frame_index]
        summary_artist.set_text(replay_summary_text(manifest, frame, log_path.name, build_stage=args.build_stage))
        if cursor is not None:
            cursor.set_xdata([frame.time_s, frame.time_s])
        artists = [item for group in dynamic_artists.values() for item in group]
        if cursor is not None:
            artists.append(cursor)
        artists.append(summary_artist)
        return artists

    anim = animation.FuncAnimation(
        figure,
        update,
        frames=len(frames),
        interval=max(1000.0 / args.fps, 1.0),
        blit=False,
        repeat=True,
    )

    output = args.output
    if output is None and not show:
        output = default_output_path(log_path, ".gif")

    if output is not None:
        if output.suffix.lower() == ".png":
            update(0)
            output.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(output, dpi=160, facecolor=DARK_BG)
            print(f"wrote {output}")
        else:
            save_animation(figure, anim, output, args.fps)
            print(f"wrote {output}")

    if show:
        import_pyplot(True).show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())