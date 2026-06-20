#!/usr/bin/env python3
"""Replay synthetic or recorded ADC load-cell traces through the TV3 propulsion model."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.tv3_propulsion_model import (  # noqa: E402
    FAULT_IGNITION_TIMEOUT,
    FAULT_SENSOR_STALE,
    MODE_ABORT,
    MODE_BOOST,
    MODE_COAST,
    MODE_IGNITION_PENDING,
    LoadCellConfig,
    LoadCellModel,
    ModeManagerModel,
    load_cell_config_from_manifest,
    load_manifest,
    mode_manager_config_from_manifest,
    motor_reference_from_manifest,
    replay_adc_trace,
)


def load_adc_csv(path: Path) -> list[tuple[float, int | None]]:
    samples: list[tuple[float, int | None]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            time_s = float(row["time_s"])
            raw_value = row.get("raw_count", "").strip()
            raw_count = None if raw_value == "" else int(float(raw_value))
            samples.append((time_s, raw_count))
    return samples


def summarize_timeline(timeline: list[dict]) -> dict:
    modes = [entry["mode"].mode for entry in timeline if entry["mode"] is not None]
    faults = [entry["mode"].fault_reason for entry in timeline if entry["mode"] is not None]
    confirmed = [
        entry["engine_state"].confirmed_mask for entry in timeline if entry["engine_state"] is not None
    ]

    return {
        "samples": len(timeline),
        "final_mode": modes[-1] if modes else None,
        "max_confirmed_mask": max(confirmed) if confirmed else 0,
        "saw_boost": MODE_BOOST in modes,
        "saw_coast": MODE_COAST in modes,
        "saw_abort": MODE_ABORT in modes,
        "saw_ignition_pending": MODE_IGNITION_PENDING in modes,
        "final_fault": faults[-1] if faults else 0,
        "ignition_timeout": FAULT_IGNITION_TIMEOUT in faults,
        "sensor_stale": FAULT_SENSOR_STALE in faults,
        "final_thrust_n": timeline[-1]["thrust"].filtered_thrust_n if timeline else 0.0,
        "final_sequence_complete": timeline[-1]["engine_state"].sequence_complete if timeline else False,
    }


def serialize_timeline(timeline: list[dict]) -> list[dict]:
    serialized: list[dict] = []
    for entry in timeline:
        item = {
            "time_s": entry["time_s"],
            "adc_raw": entry["adc_raw"],
            "thrust": asdict(entry["thrust"]),
            "engine_state": asdict(entry["engine_state"]),
        }
        if entry["mode"] is not None:
            item["mode"] = asdict(entry["mode"])
        serialized.append(item)
    return serialized


def build_models(vehicle_config: Path, *, source: int | None = None) -> tuple[LoadCellModel, ModeManagerModel, dict]:
    manifest = load_manifest(vehicle_config)
    load_cell_config = load_cell_config_from_manifest(manifest)
    if source is not None:
        load_cell_config.source = source

    mode_config = mode_manager_config_from_manifest(manifest)
    reference = motor_reference_from_manifest(manifest, thrust_n=0.0, ignition_mask=0)

    load_cell = LoadCellModel(config=load_cell_config, reference=reference)
    mode_manager = ModeManagerModel(config=mode_config)
    return load_cell, mode_manager, manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("adc_csv", type=Path, help="CSV with time_s and raw_count columns")
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=REPO_ROOT / "config/vehicles/tv3_v1.json",
        help="Vehicle manifest for load-cell and state-machine params",
    )
    parser.add_argument("--launch-at-s", type=float, default=0.0, help="Simulated launch command time")
    parser.add_argument("--source", type=int, choices=[0, 1], help="Override RK_LC_SRC (0=ADC, 1=reference)")
    parser.add_argument("--json", action="store_true", help="Print full replay timeline as JSON")
    parser.add_argument("--output", type=Path, help="Optional JSON output path")
    args = parser.parse_args()

    vehicle_config = args.vehicle if args.vehicle.is_absolute() else REPO_ROOT / args.vehicle
    samples = load_adc_csv(args.adc_csv if args.adc_csv.is_absolute() else REPO_ROOT / args.adc_csv)
    load_cell, mode_manager, manifest = build_models(vehicle_config, source=args.source)

    def reference_builder(time_s: float, thrust_n: float):
        ignition_mask = 0
        if time_s >= args.launch_at_s:
            active_slot = mode_manager.active_sequence_slot
            for slot in range(active_slot + 1):
                ignition_mask |= 1 << mode_manager.config.ignition_sequence[slot]
        return motor_reference_from_manifest(
            manifest,
            thrust_n=thrust_n,
            ignition_mask=ignition_mask,
        )

    timeline = replay_adc_trace(
        samples,
        load_cell=load_cell,
        mode_manager=mode_manager,
        reference_builder=reference_builder,
        launch_at_s=args.launch_at_s,
    )
    summary = summarize_timeline(timeline)

    if args.json:
        payload = {"summary": summary, "timeline": serialize_timeline(timeline)}
        text = json.dumps(payload, indent=2)
        if args.output:
            args.output.write_text(text + "\n")
        else:
            print(text)
    else:
        print(f"trace:   {args.adc_csv}")
        print(f"vehicle: {vehicle_config}")
        print(f"result:  {'PASS' if summary['samples'] > 0 else 'FAIL'}")
        for key, value in summary.items():
            print(f"  {key}: {value}")
        if args.output:
            args.output.write_text(json.dumps({"summary": summary}, indent=2) + "\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
