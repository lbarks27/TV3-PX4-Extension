#!/usr/bin/env python3
"""Generate TV3-focused review plots from a PX4 ULog."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
ARCHIVED_SIM_LOG_ROOT = REPO_ROOT / "logs" / "sim"
SITL_ROOTFS_LOG_ROOT = REPO_ROOT.parent / ".work" / "px4-tv3" / "build" / "px4_sitl_default" / "rootfs" / "log"
DEFAULT_LOG_ROOTS = (ARCHIVED_SIM_LOG_ROOT, SITL_ROOTFS_LOG_ROOT)
REVIEW_TOPICS = [
    "actuator_armed",
    "actuator_motors",
    "actuator_servos",
    "control_allocator_status",
    "home_position",
    "internal_combustion_engine_control",
    "tv3_command",
    "tv3_engine_command",
    "tv3_engine_state",
    "tv3_guidance_status",
    "tv3_load_cell",
    "tv3_mode_status",
    "tv3_motor_reference",
    "tv3_status",
    "tv3_thrust",
    "sensor_combined",
    "trajectory_setpoint",
    "vehicle_acceleration",
    "vehicle_attitude",
    "vehicle_attitude_setpoint",
    "vehicle_angular_velocity",
    "vehicle_command",
    "vehicle_command_ack",
    "vehicle_control_mode",
    "vehicle_global_position",
    "vehicle_land_detected",
    "vehicle_local_position",
    "vehicle_rates_setpoint",
    "vehicle_status",
    "vehicle_thrust_setpoint",
    "vehicle_torque_setpoint",
]


def import_ulog():
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise SystemExit("missing dependency: install pyulog with `python3 -m pip install -r requirements-viz.txt`") from exc

    return ULog


def import_pyplot(show: bool):
    try:
        import matplotlib
    except ImportError as exc:
        raise SystemExit("missing dependency: install matplotlib with `python3 -m pip install -r requirements-viz.txt`") from exc

    if not show:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    return plt


def find_latest_ulog(log_roots: Iterable[Path] = DEFAULT_LOG_ROOTS) -> Path:
    logs = []
    for log_root in log_roots:
        if log_root.exists():
            logs.extend(log_root.rglob("*.ulg"))

    logs = sorted(logs, key=lambda path: path.stat().st_mtime)
    if not logs:
        roots = ", ".join(str(path) for path in log_roots)
        raise SystemExit(f"no .ulg files found under {roots}")
    return logs[-1]


def load_ulog(path: Path, list_topics: bool):
    ULog = import_ulog()
    filters = None if list_topics else REVIEW_TOPICS
    return ULog(str(path), message_name_filter_list=filters)


def datasets_by_name(ulog) -> dict[str, list]:
    datasets: dict[str, list] = {}
    for dataset in ulog.data_list:
        datasets.setdefault(dataset.name, []).append(dataset)
    return datasets


def dataset(datasets: dict[str, list], name: str):
    matches = datasets.get(name, [])
    return matches[0] if matches else None


def topic_names(ulog) -> list[str]:
    names = []
    for item in ulog.data_list:
        suffix = f"#{item.multi_id}" if getattr(item, "multi_id", 0) else ""
        names.append(f"{item.name}{suffix}")
    return sorted(names)


def timestamps_us(dataset_obj) -> Iterable:
    data = dataset_obj.data
    if "timestamp" in data:
        return data["timestamp"]
    if "timestamp_sample" in data:
        return data["timestamp_sample"]
    return []


def start_timestamp_us(datasets: dict[str, list]) -> int:
    starts = []
    for dataset_list in datasets.values():
        for item in dataset_list:
            times = timestamps_us(item)
            if len(times):
                starts.append(int(times[0]))
    return min(starts) if starts else 0


def time_s(dataset_obj, start_us: int):
    return (dataset_obj.data["timestamp"] - start_us) * 1e-6


def field(dataset_obj, *names: str):
    for name in names:
        if name in dataset_obj.data:
            return dataset_obj.data[name]
    return None


def plotted(ax, datasets: dict[str, list], topic: str, fields: list[tuple[str, tuple[str, ...]]], start_us: int) -> int:
    topic_dataset = dataset(datasets, topic)
    if topic_dataset is None or "timestamp" not in topic_dataset.data:
        return 0

    count = 0
    time = time_s(topic_dataset, start_us)
    for label, candidates in fields:
        values = field(topic_dataset, *candidates)
        if values is None:
            continue
        ax.plot(time, values, label=label, linewidth=1.2)
        count += 1
    return count


def mark_empty(ax, text: str) -> None:
    ax.text(0.5, 0.5, text, transform=ax.transAxes, ha="center", va="center", color="0.45")


def finish_axis(ax, ylabel: str) -> None:
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    handles, _ = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="upper right", fontsize="small", ncols=2)


def build_figure(ulog, log_path: Path, show: bool):
    plt = import_pyplot(show)
    datasets = datasets_by_name(ulog)
    start_us = start_timestamp_us(datasets)
    figure, axes = plt.subplots(6, 1, figsize=(13, 13), sharex=True, constrained_layout=True)
    figure.suptitle(f"TV3 ULog Review: {log_path.name}")

    count = 0
    count += plotted(
        axes[0],
        datasets,
        "tv3_status",
        [
            ("tv3 mode", ("mode",)),
            ("rail distance m", ("rail_distance_m",)),
            ("burn fraction", ("burn_fraction",)),
        ],
        start_us,
    )
    count += plotted(axes[0], datasets, "tv3_guidance_status", [("guidance phase", ("phase",))], start_us)
    if count == 0:
        mark_empty(axes[0], "missing tv3_status / tv3_guidance_status")
    finish_axis(axes[0], "mode/state")

    count = plotted(
        axes[1],
        datasets,
        "tv3_thrust",
        [
            ("measured N", ("measured_thrust_n",)),
            ("filtered N", ("filtered_thrust_n",)),
            ("expected N", ("expected_thrust_n",)),
        ],
        start_us,
    )
    count += plotted(
        axes[1],
        datasets,
        "tv3_motor_reference",
        [
            ("reference N", ("expected_thrust_n",)),
            ("vehicle mass kg", ("expected_vehicle_mass_kg",)),
        ],
        start_us,
    )
    if count == 0:
        mark_empty(axes[1], "missing tv3_thrust / tv3_motor_reference")
    finish_axis(axes[1], "thrust/mass")

    count = plotted(
        axes[2],
        datasets,
        "vehicle_angular_velocity",
        [
            ("roll rate", ("xyz[0]",)),
            ("pitch rate", ("xyz[1]",)),
            ("yaw rate", ("xyz[2]",)),
        ],
        start_us,
    )
    if count == 0:
        mark_empty(axes[2], "missing vehicle_angular_velocity")
    finish_axis(axes[2], "rad/s")

    count = plotted(
        axes[3],
        datasets,
        "vehicle_torque_setpoint",
        [
            ("torque x", ("xyz[0]",)),
            ("torque y", ("xyz[1]",)),
            ("torque z", ("xyz[2]",)),
        ],
        start_us,
    )
    count += plotted(
        axes[3],
        datasets,
        "vehicle_thrust_setpoint",
        [
            ("thrust x", ("xyz[0]",)),
            ("thrust y", ("xyz[1]",)),
            ("thrust z", ("xyz[2]",)),
        ],
        start_us,
    )
    if count == 0:
        mark_empty(axes[3], "missing vehicle_torque_setpoint / vehicle_thrust_setpoint")
    finish_axis(axes[3], "setpoint")

    count = plotted(
        axes[4],
        datasets,
        "vehicle_local_position",
        [
            ("x", ("x",)),
            ("y", ("y",)),
            ("z", ("z",)),
        ],
        start_us,
    )
    count += plotted(
        axes[4],
        datasets,
        "trajectory_setpoint",
        [
            ("sp x", ("position[0]",)),
            ("sp y", ("position[1]",)),
            ("sp z", ("position[2]",)),
        ],
        start_us,
    )
    if count == 0:
        mark_empty(axes[4], "missing vehicle_local_position / trajectory_setpoint")
    finish_axis(axes[4], "position m")

    count = plotted(
        axes[5],
        datasets,
        "tv3_guidance_status",
        [
            ("target distance m", ("target_distance_m",)),
            ("thrust margin N", ("thrust_margin_n",)),
            ("remaining delta-v m/s", ("remaining_delta_v_m_s",)),
        ],
        start_us,
    )
    count += plotted(
        axes[5],
        datasets,
        "control_allocator_status",
        [
            ("unalloc torque x", ("unallocated_torque[0]",)),
            ("unalloc torque y", ("unallocated_torque[1]",)),
            ("unalloc torque z", ("unallocated_torque[2]",)),
        ],
        start_us,
    )
    if count == 0:
        mark_empty(axes[5], "missing tv3_guidance_status / control_allocator_status")
    finish_axis(axes[5], "margin/error")
    axes[5].set_xlabel("time since first logged sample (s)")

    return figure


def default_output_path(log_path: Path) -> Path:
    return log_path.with_name(f"{log_path.stem}.tv3_review.png")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", nargs="?", type=Path, help="Path to a PX4 .ulg file. Defaults to the latest SITL log.")
    parser.add_argument("--latest", action="store_true", help="Plot the newest archived SITL .ulg, falling back to the PX4 rootfs.")
    parser.add_argument("--list-topics", action="store_true", help="Print topics in the selected log and exit.")
    parser.add_argument("-o", "--output", type=Path, help="Output PNG path. Defaults beside the .ulg file.")
    parser.add_argument("--show", action="store_true", help="Open an interactive Matplotlib window after saving.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_path = find_latest_ulog() if args.latest or args.ulog is None else args.ulog
    if not log_path.exists():
        raise SystemExit(f"ULog not found: {log_path}")

    ulog = load_ulog(log_path, args.list_topics)
    if args.list_topics:
        for name in topic_names(ulog):
            print(name)
        return 0

    output = args.output or default_output_path(log_path)
    figure = build_figure(ulog, log_path, args.show)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    print(f"wrote {output}")

    if args.show:
        import matplotlib.pyplot as plt

        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
