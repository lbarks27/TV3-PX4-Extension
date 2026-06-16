#!/usr/bin/env python3
"""Review a PX4 ULog against a TV3 flight profile's pass criteria."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / "config/flight_profiles/lander_hover_window.yaml"

TV3_STATUS_FAULT_SENSOR_STALE = 4
TV3_STATUS_FAULT_MOTOR_DATA = 16

PHASE_LAUNCH_ASCENT = 1
PHASE_APOGEE_TRACK = 2
PHASE_WAYPOINT_TRACK = 3
PHASE_LANDING_APPROACH = 4
ACTIVE_GUIDANCE_PHASES = (
    PHASE_LAUNCH_ASCENT,
    PHASE_APOGEE_TRACK,
    PHASE_WAYPOINT_TRACK,
    PHASE_LANDING_APPROACH,
)


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


@dataclass
class ReviewReport:
    profile: str
    ulog: str
    passed: bool
    checks: list[CheckResult] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)


def import_ulog():
    try:
        from pyulog import ULog
    except ImportError as exc:
        raise SystemExit(
            "missing dependency: install pyulog with `python3 -m pip install -r requirements-viz.txt`"
        ) from exc
    return ULog


def resolve_path(value: str | None, default: Path) -> Path:
    path = Path(value) if value else default
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def topic_aliases(name: str) -> list[str]:
    if name.startswith("tv3_"):
        legacy = "rocket_" + name[4:]
        return [name, legacy]
    return [name]


def topic_data(ulog, name: str):
    for alias in topic_aliases(name):
        for dataset in ulog.data_list:
            if dataset.name == alias:
                return dataset.data
    return None


def topic_names(ulog) -> set[str]:
    return {dataset.name for dataset in ulog.data_list}


def load_profile(path: Path) -> dict:
    profile = yaml.safe_load(path.read_text())
    if not isinstance(profile, dict):
        raise ValueError(f"invalid flight profile: {path}")
    return profile


def vehicle_body_mass_kg(profile: dict, profile_path: Path, default: float = 4.0) -> float:
    vehicle = profile.get("vehicle")
    if isinstance(vehicle, dict):
        mass = vehicle.get("body_mass_kg")
        if mass is not None:
            return float(mass)

    vehicle_ref = vehicle if isinstance(vehicle, str) else profile.get("name")
    if isinstance(vehicle_ref, str) and vehicle_ref:
        vehicle_path = REPO_ROOT / "config/vehicles" / f"{vehicle_ref}.yaml"
        if not vehicle_path.exists():
            vehicle_path = profile_path.parent.parent / "vehicles" / f"{vehicle_ref}.yaml"
        if vehicle_path.exists():
            manifest = load_profile(vehicle_path)
            manifest_vehicle = manifest.get("vehicle")
            if isinstance(manifest_vehicle, dict) and manifest_vehicle.get("body_mass_kg") is not None:
                return float(manifest_vehicle["body_mass_kg"])

    return default


def sample_durations_s(timestamps_us) -> list[float]:
    import numpy as np

    if len(timestamps_us) < 2:
        return [0.05] * max(len(timestamps_us), 1)
    t_s = np.asarray(timestamps_us, dtype=float) * 1e-6
    dt = np.diff(t_s, prepend=t_s[0])
    if len(dt) > 1:
        dt[0] = dt[1]
    return dt.tolist()


def longest_true_run(mask: Iterable[bool], durations_s: list[float]) -> float:
    best = 0.0
    current = 0.0
    for flag, dt in zip(mask, durations_s, strict=False):
        if flag:
            current += dt
            best = max(best, current)
        else:
            current = 0.0
    return best


def review_ulog(ulog_path: Path, profile_path: Path) -> ReviewReport:
    import numpy as np

    profile = load_profile(profile_path)
    review = profile.get("review", {})
    guidance = profile.get("guidance", {})
    scenario = profile.get("scenario", {})

    ULog = import_ulog()
    ulog = ULog(str(ulog_path))
    names = topic_names(ulog)

    checks: list[CheckResult] = []
    metrics: dict = {}

    required_topics = list(review.get("required_topics", []))
    missing_topics = [topic for topic in required_topics if not any(alias in names for alias in topic_aliases(topic))]
    checks.append(
        CheckResult(
            "required_topics",
            not missing_topics,
            "missing: " + ", ".join(missing_topics) if missing_topics else f"found {len(required_topics)} topics",
        )
    )

    engine_state = topic_data(ulog, "tv3_engine_state")
    engine_count = int(guidance.get("engine_count", profile.get("propulsion", {}).get("engine_count", 3)))
    if engine_state is None:
        checks.append(CheckResult("engine_sequence", False, "tv3_engine_state missing"))
    else:
        seq_any = bool(np.any(engine_state["sequence_complete"]))
        all_ignited = bool(np.max(engine_state["all_ignited"]))
        fault_mask = int(np.max(engine_state["fault_mask"]))
        metrics["engine_sequence_complete"] = seq_any
        metrics["engine_all_ignited"] = all_ignited
        metrics["engine_fault_mask"] = fault_mask
        checks.append(
            CheckResult(
                "engine_sequence",
                seq_any and all_ignited and fault_mask == 0,
                f"sequence_complete={seq_any} all_ignited={all_ignited} fault_mask={fault_mask}",
            )
        )

    tv3_status = topic_data(ulog, "tv3_status")
    motor_reference = topic_data(ulog, "tv3_motor_reference")
    load_cell = topic_data(ulog, "tv3_load_cell")
    fault_values = set()
    if tv3_status is not None:
        fault_values.update(int(value) for value in np.unique(tv3_status["fault_reason"]))
    stale_faults = {TV3_STATUS_FAULT_SENSOR_STALE, TV3_STATUS_FAULT_MOTOR_DATA}
    stale_seen = sorted(fault_values & stale_faults)
    metrics["tv3_fault_reasons"] = sorted(fault_values)
    checks.append(
        CheckResult(
            "propulsion_faults",
            not stale_seen,
            "stale/motor faults: " + ", ".join(str(v) for v in stale_seen) if stale_seen else "no stale faults",
        )
    )

    if motor_reference is not None and "fault_flags" in motor_reference:
        motor_faults = int(np.max(motor_reference["fault_flags"]))
        metrics["motor_reference_fault_flags_max"] = motor_faults
        checks.append(
            CheckResult(
                "motor_reference_faults",
                motor_faults == 0,
                f"fault_flags max={motor_faults}",
            )
        )

    if load_cell is not None and "fault_flags" in load_cell:
        load_cell_faults = int(np.max(load_cell["fault_flags"]))
        metrics["load_cell_fault_flags_max"] = load_cell_faults
        checks.append(
            CheckResult(
                "load_cell_faults",
                load_cell_faults == 0,
                f"fault_flags max={load_cell_faults}",
            )
        )

    guidance_status = topic_data(ulog, "tv3_guidance_status")
    groundtruth = topic_data(ulog, "vehicle_local_position_groundtruth")
    local_position = topic_data(ulog, "vehicle_local_position")
    position_topic = groundtruth if groundtruth is not None else local_position
    hold_alt_m = float(guidance.get("hold_alt_m", guidance.get("takeoff_alt_m", 8.0)))
    acceptance_m = float(guidance.get("acceptance_m", 3.0))
    min_hover_s = float(review.get("min_hover_s", 3.0))

    if guidance_status is None or position_topic is None:
        checks.append(CheckResult("hover_window", False, "guidance or position topic missing"))
    else:
        t0 = min(guidance_status["timestamp"][0], position_topic["timestamp"][0])
        t_guidance = (guidance_status["timestamp"] - t0) * 1e-6
        t_position = (position_topic["timestamp"] - t0) * 1e-6
        altitude_m = -position_topic["z"]
        target_distance = np.interp(
            t_position,
            t_guidance,
            guidance_status["target_distance_m"],
            left=np.nan,
            right=np.nan,
        )
        phase = np.interp(
            t_position,
            t_guidance,
            guidance_status["phase"],
            left=np.nan,
            right=np.nan,
        )
        dt = sample_durations_s(position_topic["timestamp"])

        metrics["max_altitude_m"] = float(np.nanmax(altitude_m))
        metrics["hold_alt_m"] = hold_alt_m
        reached_hover = bool(np.nanmax(altitude_m) >= hold_alt_m - 1.0)
        checks.append(
            CheckResult(
                "reach_hover_altitude",
                reached_hover,
                f"max_alt={metrics['max_altitude_m']:.2f} m target>={hold_alt_m - 1.0:.2f} m",
            )
        )

        tracking_mask = np.isfinite(target_distance) & (target_distance <= acceptance_m)
        tracking_phase_mask = np.isfinite(phase) & np.isin(phase, [PHASE_LAUNCH_ASCENT, PHASE_WAYPOINT_TRACK])
        hover_mask = (altitude_m >= hold_alt_m - 1.0) & (altitude_m <= hold_alt_m + acceptance_m + 2.0)
        hover_hold_mask = tracking_mask & hover_mask & tracking_phase_mask
        hover_hold_s = longest_true_run(hover_hold_mask.tolist(), dt)
        metrics["hover_hold_within_acceptance_s"] = hover_hold_s
        metrics["acceptance_m"] = acceptance_m
        metrics["min_hover_s"] = min_hover_s
        checks.append(
            CheckResult(
                "hover_window",
                hover_hold_s >= min_hover_s,
                f"held within {acceptance_m:.1f} m for {hover_hold_s:.2f} s (required {min_hover_s:.2f} s)",
            )
        )

    allocator = topic_data(ulog, "control_allocator_status")
    if allocator is None:
        checks.append(CheckResult("allocator_saturation", False, "control_allocator_status missing"))
    else:
        saturation_keys = sorted(
            [key for key in allocator if key.startswith("actuator_saturation[")],
            key=lambda item: int(item.split("[", 1)[1].rstrip("]")),
        )
        if saturation_keys:
            import numpy as np

            saturation = np.column_stack([allocator[key] for key in saturation_keys])
            any_sat = np.any(saturation != 0, axis=1)
            dt = sample_durations_s(allocator["timestamp"])
            sat_logged = bool(np.any(any_sat))
            sustained_sat_s = longest_true_run(any_sat.tolist(), dt)
            diverged = False
            if guidance_status is not None and position_topic is not None:
                t0 = min(guidance_status["timestamp"][0], position_topic["timestamp"][0])
                t_guidance = (guidance_status["timestamp"] - t0) * 1e-6
                t_position = (position_topic["timestamp"] - t0) * 1e-6
                target_distance = np.interp(
                    t_position,
                    t_guidance,
                    guidance_status["target_distance_m"],
                    left=np.nan,
                    right=np.nan,
                )
                altitude_m = -position_topic["z"]
                hover_reached = altitude_m >= hold_alt_m - 1.0
                if np.any(hover_reached):
                    post_hover_distance = target_distance[hover_reached]
                    diverged = bool(np.nanmax(post_hover_distance) > acceptance_m * 4.0)
            metrics["allocator_saturation_logged"] = sat_logged
            metrics["allocator_sustained_saturation_s"] = sustained_sat_s
            checks.append(
                CheckResult(
                    "allocator_saturation",
                    sat_logged and not diverged,
                    f"logged={sat_logged} sustained_sat={sustained_sat_s:.2f}s diverged={diverged}",
                )
            )
        else:
            checks.append(CheckResult("allocator_saturation", False, "allocator saturation fields missing"))

    if guidance_status is not None:
        import numpy as np

        min_twr = float(guidance.get("min_twr", 1.0))
        landing_twr = float(guidance.get("landing_twr", min_twr))
        mass_estimate = vehicle_body_mass_kg(profile, profile_path)
        if mass_estimate <= 0:
            mass_estimate = 4.0

        phases = guidance_status["phase"]
        required_twr = np.where(
            np.isin(phases, [PHASE_WAYPOINT_TRACK, PHASE_LANDING_APPROACH]),
            landing_twr,
            min_twr,
        )
        available = guidance_status["available_thrust_n"]
        twr = available / (mass_estimate * 9.80665)
        invalid_margin = guidance_status["thrust_solution_valid"] == 0
        active_guidance = np.isin(phases, ACTIVE_GUIDANCE_PHASES)
        powered = (guidance_status["tv3_boosting"] > 0) | (guidance_status["tv3_coasting"] > 0)
        false_invalid_mask = (
            invalid_margin
            & active_guidance
            & powered
            & (available > 1.0)
            & (twr >= required_twr)
        )
        false_invalid = bool(np.any(false_invalid_mask))
        metrics["guidance_false_no_solution"] = false_invalid
        checks.append(
            CheckResult(
                "guidance_thrust_margin",
                not false_invalid,
                "guidance reported no solution while thrust margin looked sufficient"
                if false_invalid
                else "no false no-solution samples",
            )
        )

    passed = all(check.passed for check in checks)
    return ReviewReport(
        profile=str(profile_path),
        ulog=str(ulog_path),
        passed=passed,
        checks=checks,
        metrics={**metrics, "scenario_type": scenario.get("type"), "profile_name": profile.get("name")},
    )


def find_latest_ulog(log_root: Path) -> Path | None:
    logs = sorted(log_root.rglob("*.ulg"), key=lambda path: path.stat().st_mtime)
    return logs[-1] if logs else None


def print_report(report: ReviewReport) -> None:
    print(f"profile: {report.profile}")
    print(f"ulog:    {report.ulog}")
    print(f"result:  {'PASS' if report.passed else 'FAIL'}")
    for check in report.checks:
        status = "pass" if check.passed else "FAIL"
        print(f"  [{status}] {check.name}: {check.detail}")
    if report.metrics:
        print("metrics:")
        for key, value in sorted(report.metrics.items()):
            print(f"  {key}: {value}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Review a PX4 ULog against a TV3 flight profile.")
    parser.add_argument("ulog", nargs="?", type=Path, help="Path to a PX4 .ulg file")
    parser.add_argument(
        "--flight-profile",
        default=str(DEFAULT_PROFILE),
        help="Flight profile YAML with review criteria",
    )
    parser.add_argument("--latest", action="store_true", help="Use the newest archived sim ULog")
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of human-readable text")
    args = parser.parse_args()

    profile_path = resolve_path(args.flight_profile, DEFAULT_PROFILE)
    if args.latest or args.ulog is None:
        ulog_path = find_latest_ulog(REPO_ROOT / "logs" / "sim")
        if ulog_path is None:
            print("no archived sim .ulg files found", file=sys.stderr)
            return 1
    else:
        ulog_path = args.ulog if args.ulog.is_absolute() else REPO_ROOT / args.ulog

    if not profile_path.exists():
        print(f"flight profile not found: {profile_path}", file=sys.stderr)
        return 1
    if not ulog_path.exists():
        print(f"ulog not found: {ulog_path}", file=sys.stderr)
        return 1

    report = review_ulog(ulog_path, profile_path)
    if args.json:
        payload = asdict(report)
        print(json.dumps(payload, indent=2))
    else:
        print_report(report)
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())