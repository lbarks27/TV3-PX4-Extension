#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TV3_ROOT = REPO_ROOT.parent
DEFAULT_SOURCE = TV3_ROOT / "vendor" / "Thrust-Curves-Apogee"


@dataclass
class ValidationResult:
    motor_index: int
    motor_id: str
    manufacturer: str
    designation: str
    active: bool
    curve_file: str
    specs_file: str
    errors: list[str] = field(default_factory=list)


@dataclass
class CurvePoint:
    time_s: float
    thrust_n: float
    motor_mass_kg: float
    burn_fraction: float
    cumulative_impulse_ns: float


@dataclass
class MotorSpecs:
    motor_id: str
    manufacturer: str
    designation: str
    loaded_mass_kg: float
    dry_mass_kg: float
    diameter_m: float
    length_m: float
    total_impulse_ns: float
    burn_duration_s: float


def sanitize_motor_id(manufacturer: str, designation: str) -> str:
    raw = f"{manufacturer}-{designation}".lower()
    sanitized = re.sub(r"[^a-z0-9]+", "-", raw).strip("-")
    return sanitized or "motor"


def parse_float(row: dict[str, str], key: str) -> float:
    value = (row.get(key) or "").strip()
    if not value:
        raise ValueError(f"missing {key}")
    return float(value)


def load_inventory(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def report_path(path: Path) -> str:
    try:
        return str(path.relative_to(TV3_ROOT))
    except ValueError:
        pass

    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def normalize_curve(points: list[tuple[float, float, float]], loaded_mass_kg: float, dry_mass_kg: float) -> list[CurvePoint]:
    if not points:
        raise ValueError("no points")

    if points[0][0] > 0.0:
        points = [(0.0, 0.0, loaded_mass_kg), *points]

    if points[-1][1] != 0.0:
        points = [*points, (points[-1][0] + 1e-3, 0.0, max(points[-1][2], dry_mass_kg))]

    cumulative_impulse = 0.0
    normalized: list[CurvePoint] = []

    for index, (time_s, thrust_n, motor_mass_kg) in enumerate(points):
        if index > 0:
            prev = points[index - 1]
            dt = time_s - prev[0]
            cumulative_impulse += 0.5 * (prev[1] + thrust_n) * dt

        burn_fraction = 0.0
        denominator = max(loaded_mass_kg - dry_mass_kg, 1e-6)
        burn_fraction = min(max((loaded_mass_kg - motor_mass_kg) / denominator, 0.0), 1.0)

        normalized.append(
            CurvePoint(
                time_s=time_s,
                thrust_n=max(thrust_n, 0.0),
                motor_mass_kg=max(motor_mass_kg, 0.0),
                burn_fraction=burn_fraction,
                cumulative_impulse_ns=cumulative_impulse,
            )
        )

    return normalized


def validate_motor(row: dict[str, str], source_root: Path, motor_index: int) -> tuple[ValidationResult, MotorSpecs | None, list[CurvePoint] | None]:
    manufacturer = row["manufacturer"].strip()
    designation = row["designation"].strip()
    motor_id = sanitize_motor_id(manufacturer, designation)
    result = ValidationResult(
        motor_index=motor_index,
        motor_id=motor_id,
        manufacturer=manufacturer,
        designation=designation,
        active=False,
        curve_file=f"{motor_id}/curve.csv",
        specs_file=f"{motor_id}/specs.csv",
    )

    specs_path = source_root / row["specs_file"]
    dynamics_path = source_root / row["dynamics_file"]

    try:
        with specs_path.open(newline="") as stream:
            specs_row = next(csv.DictReader(stream), None)
        if specs_row is None:
            raise ValueError("no specs rows")

        loaded_mass_kg = parse_float(specs_row, "initial_mass_g") / 1000.0
        dry_mass_kg = parse_float(specs_row, "dry_mass_g") / 1000.0
        diameter_m = parse_float(specs_row, "diameter_mm") / 1000.0
        length_m = parse_float(specs_row, "length_mm") / 1000.0

        if loaded_mass_kg <= 0.0:
            raise ValueError("loaded mass must be positive")
        if dry_mass_kg < 0.0:
            raise ValueError("dry mass must be non-negative")
        if dry_mass_kg > loaded_mass_kg:
            raise ValueError("dry mass exceeds loaded mass")

        points: list[tuple[float, float, float]] = []
        with dynamics_path.open(newline="") as stream:
            for dynamics_row in csv.DictReader(stream):
                time_s = parse_float(dynamics_row, "time_s")
                thrust_n = parse_float(dynamics_row, "thrust_N")
                motor_mass_kg = parse_float(dynamics_row, "motor_mass_kg")
                points.append((time_s, thrust_n, motor_mass_kg))

        if not points:
            raise ValueError("no thrust samples")

        previous_time = -1.0
        previous_mass = math.inf
        for time_s, thrust_n, motor_mass_kg in points:
            if time_s <= previous_time:
                raise ValueError("time is not strictly increasing")
            if thrust_n < -1e-4:
                raise ValueError("negative thrust sample")
            if motor_mass_kg < -1e-4:
                raise ValueError("negative motor mass sample")
            if motor_mass_kg > previous_mass + 1e-4:
                raise ValueError("motor mass increased during burn")
            previous_time = time_s
            previous_mass = motor_mass_kg

        normalized_curve = normalize_curve(points, loaded_mass_kg, dry_mass_kg)
        total_impulse_ns = normalized_curve[-1].cumulative_impulse_ns
        burn_duration_s = normalized_curve[-1].time_s

        if total_impulse_ns <= 0.0:
            raise ValueError("total impulse must be positive")

        specs = MotorSpecs(
            motor_id=motor_id,
            manufacturer=manufacturer,
            designation=designation,
            loaded_mass_kg=loaded_mass_kg,
            dry_mass_kg=dry_mass_kg,
            diameter_m=diameter_m,
            length_m=length_m,
            total_impulse_ns=total_impulse_ns,
            burn_duration_s=burn_duration_s,
        )

        result.active = True
        return result, specs, normalized_curve

    except Exception as exc:
        result.errors.append(str(exc))
        return result, None, None


def write_catalog(output_root: Path, results: list[ValidationResult]) -> None:
    catalog_path = output_root / "catalog.csv"
    with catalog_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "motor_index",
                "motor_id",
                "manufacturer",
                "designation",
                "active",
                "curve_file",
                "specs_file",
                "errors",
            ]
        )
        for result in results:
            writer.writerow(
                [
                    result.motor_index,
                    result.motor_id,
                    result.manufacturer,
                    result.designation,
                    1 if result.active else 0,
                    result.curve_file,
                    result.specs_file,
                    "; ".join(result.errors),
                ]
            )


def write_motor_assets(output_root: Path, result: ValidationResult, specs: MotorSpecs, curve: list[CurvePoint]) -> None:
    motor_root = output_root / result.motor_id
    motor_root.mkdir(parents=True, exist_ok=True)

    specs_path = motor_root / "specs.csv"
    with specs_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "motor_id",
                "manufacturer",
                "designation",
                "loaded_mass_kg",
                "dry_mass_kg",
                "diameter_m",
                "length_m",
                "total_impulse_ns",
                "burn_duration_s",
            ]
        )
        writer.writerow(
            [
                specs.motor_id,
                specs.manufacturer,
                specs.designation,
                f"{specs.loaded_mass_kg:.6f}",
                f"{specs.dry_mass_kg:.6f}",
                f"{specs.diameter_m:.6f}",
                f"{specs.length_m:.6f}",
                f"{specs.total_impulse_ns:.6f}",
                f"{specs.burn_duration_s:.6f}",
            ]
        )

    curve_path = motor_root / "curve.csv"
    with curve_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "thrust_n",
                "motor_mass_kg",
                "burn_fraction",
                "cumulative_impulse_ns",
            ]
        )
        for point in curve:
            writer.writerow(
                [
                    f"{point.time_s:.6f}",
                    f"{point.thrust_n:.6f}",
                    f"{point.motor_mass_kg:.6f}",
                    f"{point.burn_fraction:.6f}",
                    f"{point.cumulative_impulse_ns:.6f}",
                ]
            )


def generate_catalog(source_root: Path, output_root: Path) -> dict[str, object]:
    inventory_path = source_root / "Apogee_motor_inventory.csv"
    rows = load_inventory(inventory_path)
    output_root.mkdir(parents=True, exist_ok=True)

    results: list[ValidationResult] = []
    valid_count = 0
    invalid_count = 0

    for index, row in enumerate(rows):
        result, specs, curve = validate_motor(row, source_root, index)
        results.append(result)
        if result.active and specs and curve:
            write_motor_assets(output_root, result, specs, curve)
            valid_count += 1
        else:
            invalid_count += 1

    write_catalog(output_root, results)

    report = {
        "source_root": report_path(source_root),
        "output_root": report_path(output_root),
        "summary": {
            "motors_total": len(results),
            "motors_active": valid_count,
            "motors_rejected": invalid_count,
        },
        "results": [asdict(result) for result in results],
    }

    with (output_root / "validation_report.json").open("w") as stream:
        json.dump(report, stream, indent=2)

    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE, help="Path to the thrust-curves repository")
    parser.add_argument("--output", type=Path, required=True, help="Output directory for normalized catalog assets")
    args = parser.parse_args()

    report = generate_catalog(args.source, args.output)
    summary = report["summary"]
    print(
        f"normalized {summary['motors_active']} motors, "
        f"rejected {summary['motors_rejected']} motors, "
        f"catalog written to {args.output}"
    )


if __name__ == "__main__":
    main()
