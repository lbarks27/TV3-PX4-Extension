#!/usr/bin/env python3
"""Validate TV3 vehicle manifests against the intake schema and unit conventions."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.tv3_control_allocator import (  # noqa: E402
    axes_close,
    dot,
    normalize,
    outward_radial_axis,
)
from tools.tv3_engine_frame import build_engine_frame_axes  # noqa: E402
from tools.tv3_motor_catalog import load_motor_catalog, resolve_motor_id  # noqa: E402

DEFAULT_SCHEMA = REPO_ROOT / "config/schemas/vehicle_intake_schema.json"
VEHICLE_MANIFESTS = (
    REPO_ROOT / "config/vehicles/tv3_v1.json",
    REPO_ROOT / "config/vehicles/tv3_lander_v1.json",
)

DATA_STATUS_VALUES = {"measured", "preliminary", "placeholder"}
PHYSICAL_MODEL_STATUS = {"measured", "preliminary", "placeholder"}


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


@dataclass
class ValidationReport:
    manifest: str
    passed: bool
    checks: list[CheckResult] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)


def load_json(path: Path) -> dict:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object: {path}")
    return data


def get_path(data: dict, dotted: str) -> Any:
    current: Any = data
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            return None
        current = current[part]
    return current


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def vec_norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def engines_from_manifest(manifest: dict) -> list[dict]:
    propulsion = manifest.get("propulsion", {})
    engines = propulsion.get("engines")
    if engines:
        return engines

    body = manifest["vehicle"]
    motor = manifest["motor_selection"]
    load_cell = manifest["hardware"]["load_cell"]
    return [
        {
            "id": "engine_0",
            "motor_index": motor["index"],
            "load_cell_channel": load_cell.get("adc_channel", 0),
            "position_m": [body["motor_com_x_m"], 0.0, 0.0],
            "thrust_axis": [1.0, 0.0, 0.0],
            "roll_axis": [0.0, -1.0, 0.0],
            "yaw_axis": [0.0, 0.0, -1.0],
            "thrust_fraction": 1.0,
            "gimbal": {
                "roll_max_deg": body["tvc_max_deg"],
                "yaw_max_deg": body["tvc_max_deg"],
                "splay_max_deg": body["tvc_max_deg"],
                "slew_dps": body["tvc_slew_dps"],
                "roll_trim": 0.0,
                "yaw_trim": 0.0,
            },
        }
    ]


def ignition_sequence(manifest: dict, engines: list[dict]) -> list[int]:
    propulsion = manifest.get("propulsion", {})
    ignition = propulsion.get("ignition", {})
    sequence = ignition.get("sequence", list(range(len(engines))))
    return [int(value) for value in sequence]


def placeholder_fields(data_status: dict) -> dict[str, str]:
    fields = data_status.get("fields", {})
    if not isinstance(fields, dict):
        return {}
    return {str(key): str(value) for key, value in fields.items()}


def validate_manifest(manifest: dict, manifest_path: Path, schema: dict) -> ValidationReport:
    checks: list[CheckResult] = []
    metrics: dict[str, Any] = {"manifest_name": manifest.get("name")}

    required_top = schema.get("required_top_level", [])
    missing_top = [section for section in required_top if section not in manifest]
    checks.append(
        CheckResult(
            "required_sections",
            not missing_top,
            "missing: " + ", ".join(missing_top) if missing_top else f"found {len(required_top)} sections",
        )
    )

    data_status = manifest.get("data_status", {})
    flight_ready = bool(data_status.get("flight_ready"))
    summary = str(data_status.get("summary", "")).strip()
    metrics["flight_ready"] = flight_ready
    metrics["data_status_summary"] = summary
    checks.append(
        CheckResult(
            "data_status",
            isinstance(data_status, dict) and len(summary) >= 8 and "flight_ready" in data_status,
            "flight_ready and summary declared" if summary else "data_status.summary missing or too short",
        )
    )

    physical_model = manifest.get("physical_model", {})
    phys_status = str(physical_model.get("status", "")).strip()
    metrics["physical_model_status"] = phys_status
    checks.append(
        CheckResult(
            "physical_model_status",
            phys_status in PHYSICAL_MODEL_STATUS,
            f"status={phys_status or 'missing'}",
        )
    )

    if phys_status != "measured":
        notes = physical_model.get("notes", [])
        has_notes = isinstance(notes, list) and any(str(note).strip() for note in notes)
        checks.append(
            CheckResult(
                "non_flight_label",
                (not flight_ready) and has_notes,
                "non-measured manifest marked flight_ready=false with notes"
                if (not flight_ready) and has_notes
                else "preliminary/placeholder manifests must set flight_ready=false and document notes",
            )
        )
    else:
        checks.append(
            CheckResult(
                "non_flight_label",
                flight_ready,
                "measured manifest may set flight_ready=true",
            )
        )

    field_map = placeholder_fields(data_status)
    non_measured = {key: status for key, status in field_map.items() if status != "measured"}
    metrics["tracked_fields"] = len(field_map)
    metrics["non_measured_fields"] = len(non_measured)
    checks.append(
        CheckResult(
            "field_provenance",
            len(field_map) >= 8,
            f"tracked {len(field_map)} fields ({len(non_measured)} not measured)",
        )
    )

    vehicle = manifest.get("vehicle", {})
    for key in schema.get("vehicle", {}).get("required_fields", {}):
        value = vehicle.get(key)
        ok = value is not None
        if key == "torque_limits_nm":
            ok = isinstance(value, dict) and len(value) >= 1
        elif key.endswith("_kg") or key.endswith("_m") or key.endswith("_n") or key.endswith("_deg") or key.endswith("_dps"):
            ok = is_number(value)
            if ok and key.endswith("_kg") and key != "motor_dry_mass_kg":
                ok = float(value) > 0.0
        checks.append(CheckResult(f"vehicle.{key}", ok, "present" if ok else "missing or invalid"))

    hardware = manifest.get("hardware", {})
    load_cell = hardware.get("load_cell", {})
    calibration = load_cell.get("calibration", {})
    for key in ("driver", "source", "adc_instance", "adc_channel", "mode", "alpha", "timeout_ms"):
        checks.append(
            CheckResult(
                f"hardware.load_cell.{key}",
                key in load_cell,
                "present" if key in load_cell else "missing",
            )
        )
    for key in ("tare", "scale", "kg_per_count"):
        checks.append(
            CheckResult(
                f"hardware.load_cell.calibration.{key}",
                key in calibration,
                "present" if key in calibration else "missing",
            )
        )

    engines = engines_from_manifest(manifest)
    metrics["engine_count"] = len(engines)
    checks.append(
        CheckResult(
            "engine_count",
            1 <= len(engines) <= schema.get("defaults", {}).get("max_engines", 4),
            f"engines={len(engines)}",
        )
    )

    thrust_sum = 0.0
    seen_ids: set[str] = set()
    for index, engine in enumerate(engines):
        engine_id = engine.get("id", f"engine_{index}")
        for axis_name in ("thrust_axis", "roll_axis", "yaw_axis"):
            axis = engine.get(axis_name)
            if axis_name == "roll_axis" and axis is None:
                axis = engine.get("pitch_axis")
            ok = isinstance(axis, list) and len(axis) == 3 and all(is_number(v) for v in axis)
            if ok:
                norm = vec_norm([float(v) for v in axis])
                ok = abs(norm - 1.0) <= 0.05 or norm <= 1e-6
            checks.append(
                CheckResult(
                    f"{engine_id}.{axis_name}",
                    ok,
                    f"norm={vec_norm([float(v) for v in axis]):.3f}" if ok and axis else "invalid axis",
                )
            )

        fraction = engine.get("thrust_fraction", 1.0 / len(engines))
        if is_number(fraction):
            thrust_sum += float(fraction)

        if engine_id in seen_ids:
            checks.append(CheckResult(f"{engine_id}.id", False, "duplicate engine id"))
        seen_ids.add(engine_id)

        gimbal = engine.get("gimbal", {})
        for gkey in ("roll_max_deg", "yaw_max_deg", "slew_dps"):
            value = gimbal.get(gkey, gimbal.get("pitch_max_deg") if gkey == "roll_max_deg" else None)
            checks.append(
                CheckResult(
                    f"{engine_id}.gimbal.{gkey}",
                    is_number(value) and float(value) >= 0.0,
                    f"value={value}",
                )
            )
        roll_max = (
            float(gimbal["roll_max_deg"])
            if is_number(gimbal.get("roll_max_deg"))
            else float(gimbal["pitch_max_deg"])
            if is_number(gimbal.get("pitch_max_deg"))
            else None
        )
        yaw_max = float(gimbal["yaw_max_deg"]) if is_number(gimbal.get("yaw_max_deg")) else None
        roll_min = float(
            gimbal.get("roll_min_deg", gimbal.get("pitch_min_deg", -(roll_max or 0.0)))
        )
        yaw_min = float(gimbal.get("yaw_min_deg", -(yaw_max or 0.0)))
        if roll_max is not None:
            checks.append(
                CheckResult(
                    f"{engine_id}.gimbal.roll_range",
                    is_number(gimbal.get("roll_min_deg", gimbal.get("pitch_min_deg", roll_min)))
                    and roll_min <= roll_max,
                    f"[{roll_min:+.1f}, {roll_max:+.1f}] deg",
                )
            )
        position_m = engine.get("position_m", [0.0, 0.0, 0.0])
        is_lander = manifest.get("variant", {}).get("role") == "three_engine_lander"
        if is_lander:
            expected = build_engine_frame_axes(position_m)
            for axis_name, expected_axis, manifest_key in (
                ("thrust_axis", expected.thrust_axis, "thrust_axis"),
                ("roll_axis", expected.primary_axis, "roll_axis"),
                ("yaw_axis", expected.secondary_axis, "yaw_axis"),
            ):
                manifest_axis = engine.get(manifest_key, engine.get("pitch_axis") if manifest_key == "roll_axis" else None)
                if isinstance(manifest_axis, list) and len(manifest_axis) == 3:
                    checks.append(
                        CheckResult(
                            f"{engine_id}.{axis_name}_matches_builder",
                            axes_close(manifest_axis, expected_axis),
                            f"expected {[round(v, 6) for v in expected_axis]}",
                        )
                    )
            thrust_axis = expected.thrust_axis
            roll_axis = expected.primary_axis
            yaw_axis = expected.secondary_axis
        else:
            thrust_axis = normalize(
                engine.get("thrust_axis", outward_radial_axis(position_m)),
                outward_radial_axis(position_m),
            )
            roll_axis = normalize(
                engine.get("roll_axis", engine.get("pitch_axis", [0.0, -1.0, 0.0])),
                (0.0, -1.0, 0.0),
            )
            yaw_axis = normalize(engine.get("yaw_axis", [0.0, 0.0, -1.0]), (0.0, 0.0, -1.0))
        orthogonality = (
            abs(dot(thrust_axis, roll_axis)) <= 0.02
            and abs(dot(thrust_axis, yaw_axis)) <= 0.02
            and abs(dot(roll_axis, yaw_axis)) <= 0.02
        )
        checks.append(
            CheckResult(
                f"{engine_id}.gimbal_axes_orthogonal",
                orthogonality,
                "thrust, roll, and yaw must be mutually perpendicular",
            )
        )
        if yaw_max is not None:
            checks.append(
                CheckResult(
                    f"{engine_id}.gimbal.yaw_range",
                    is_number(gimbal.get("yaw_min_deg", yaw_min)) and yaw_min <= yaw_max,
                    f"[{yaw_min:+.1f}, {yaw_max:+.1f}] deg",
                )
            )

    checks.append(
        CheckResult(
            "thrust_fraction_sum",
            abs(thrust_sum - 1.0) <= 0.01,
            f"sum={thrust_sum:.6f}",
        )
    )

    motor_selection = manifest.get("motor_selection", {})
    catalog_source = motor_selection.get("catalog_source")
    if catalog_source:
        catalog = load_motor_catalog(str(catalog_source))
        checks.append(
            CheckResult(
                "motor_catalog_loaded",
                bool(catalog),
                f"source={catalog_source} motors={len(catalog)}",
            )
        )
        for index, engine in enumerate(engines):
            engine_id = engine.get("id", f"engine_{index}")
            motor_id = resolve_motor_id(engine, motor_selection)
            checks.append(
                CheckResult(
                    f"{engine_id}.motor_id_catalog",
                    bool(motor_id) and motor_id in catalog,
                    f"motor_id={motor_id or 'missing'}",
                )
            )

    sequence = ignition_sequence(manifest, engines)
    checks.append(
        CheckResult(
            "ignition_sequence",
            sorted(sequence) == list(range(len(engines))),
            f"sequence={sequence}",
        )
    )

    mission = manifest.get("mission_profile", {})
    checks.append(
        CheckResult(
            "mission_profile",
            isinstance(mission.get("required_sim_gates"), list) and len(mission["required_sim_gates"]) >= 1,
            f"gates={len(mission.get('required_sim_gates', []))}",
        )
    )

    guidance = manifest.get("guidance", {})
    checks.append(
        CheckResult(
            "guidance.enable",
            guidance.get("enable") in (0, 1),
            f"enable={guidance.get('enable')}",
        )
    )

    inertia = {}
    for link in physical_model.get("links", []) or []:
        if isinstance(link, dict) and link.get("id") in ("body", "base", "base_vehicle_without_tvc_moving_links"):
            inertia = link.get("inertia_kg_m2", {}) or inertia
            break
    if not inertia:
        for assembly in physical_model.get("assemblies", []) or []:
            if isinstance(assembly, dict):
                inertia = assembly.get("inertia_about_origin_kg_m2", {}) or inertia
                if inertia:
                    break
    if inertia:
        for key in ("ixx", "iyy", "izz"):
            value = inertia.get(key)
            checks.append(
                CheckResult(
                    f"inertia.{key}",
                    is_number(value) and float(value) > 0.0,
                    f"value={value}",
                )
            )

    passed = all(check.passed for check in checks)
    return ValidationReport(
        manifest=str(manifest_path),
        passed=passed,
        checks=checks,
        metrics=metrics,
    )


def validate_param_parity(manifest: dict, manifest_path: Path, schema: dict) -> list[CheckResult]:
    import importlib.util

    generator_path = REPO_ROOT / "tools/generate_vehicle_assets.py"
    spec = importlib.util.spec_from_file_location("generate_vehicle_assets", generator_path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    from tempfile import TemporaryDirectory

    checks: list[CheckResult] = []
    mappings: dict[str, str] = schema.get("param_parity", {}).get("mappings", {})

    with TemporaryDirectory() as tmp:
        output = Path(tmp) / "generated"
        module.generate_assets(manifest_path, output)
        params_path = output / "runtime" / "fs" / "microsd" / "tv3" / "airframes" / f"{manifest['name']}.params"
        generated: dict[str, str] = {}
        for line in params_path.read_text().splitlines():
            fields = line.split("\t")
            if len(fields) >= 4:
                generated[fields[2]] = fields[3]

        for param_name, manifest_ref in mappings.items():
            expected = get_path(manifest, manifest_ref)
            actual = generated.get(param_name)
            if expected is None:
                checks.append(CheckResult(f"parity.{param_name}", False, f"missing manifest field {manifest_ref}"))
                continue
            if actual is None:
                checks.append(CheckResult(f"parity.{param_name}", False, "missing generated param"))
                continue
            try:
                match = math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=1e-4)
            except (TypeError, ValueError):
                match = str(actual) == str(expected)
            checks.append(
                CheckResult(
                    f"parity.{param_name}",
                    match,
                    f"generated={actual} manifest={expected}",
                )
            )

    return checks


def validate_all(schema_path: Path = DEFAULT_SCHEMA, manifests: list[Path] | None = None) -> list[ValidationReport]:
    schema = load_json(schema_path)
    manifest_paths = manifests or list(VEHICLE_MANIFESTS)
    reports: list[ValidationReport] = []

    for manifest_path in manifest_paths:
        manifest = load_json(manifest_path)
        report = validate_manifest(manifest, manifest_path, schema)
        parity_checks = validate_param_parity(manifest, manifest_path, schema)
        report.checks.extend(parity_checks)
        report.passed = all(check.passed for check in report.checks)
        reports.append(report)

    return reports


def print_report(report: ValidationReport) -> None:
    print(f"manifest: {report.manifest}")
    print(f"result:   {'PASS' if report.passed else 'FAIL'}")
    for check in report.checks:
        status = "pass" if check.passed else "FAIL"
        print(f"  [{status}] {check.name}: {check.detail}")
    if report.metrics:
        print("metrics:")
        for key, value in sorted(report.metrics.items()):
            print(f"  {key}: {value}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate TV3 vehicle manifests.")
    parser.add_argument("manifest", nargs="*", type=Path, help="Vehicle manifest JSON paths")
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    manifests = None
    if args.manifest:
        manifests = [path if path.is_absolute() else REPO_ROOT / path for path in args.manifest]

    reports = validate_all(args.schema, manifests)
    if args.json:
        print(json.dumps([asdict(report) for report in reports], indent=2))
    else:
        for index, report in enumerate(reports):
            if index:
                print()
            print_report(report)

    return 0 if all(report.passed for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())