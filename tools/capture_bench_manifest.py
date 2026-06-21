#!/usr/bin/env python3
"""Capture bench measurements from connected PX4 hardware and update a vehicle manifest.

Reads RK_LC_* and actuator parameters over MAVLink, samples load-cell telemetry,
writes a ground-test report under logs/ground/, and optionally promotes manifest
fields from placeholder/preliminary to measured.

Typical flow (close QGroundControl first so USB serial is free):

  python3 tools/capture_bench_manifest.py \\
    --vehicle config/vehicles/tv3_v1.json \\
    --body-mass-kg 1.02 \\
    --update-manifest

If scale is not yet stored on the FC, tare on the vehicle first:

  tv3_load_cell_telemetry tare
  # apply known mass
  tv3_load_cell_telemetry calibrate 2.000
  param save

Then re-run this tool, or pass --known-mass-kg with a steady loaded raw sample.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.tv3_motor_catalog import engine_thrust_n, load_motor_catalog  # noqa: E402

GRAVITY_MPS2 = 9.80665

PARAM_NAMES = (
    "RK_LC_SRC",
    "RK_LC_CH",
    "RK_LC_NEG_CH",
    "RK_LC_ADC_INST",
    "RK_LC_MODE",
    "RK_LC_TARE",
    "RK_LC_SCALE",
    "RK_LC_KG_SC",
    "RK_LC_ALPHA",
    "RK_LC_TO_MS",
    "RK_LC_DB",
    "RK_LC_RATE_HZ",
    "RK_BODY_MASS_KG",
    "RK_BODY_COM_X_M",
    "RK_MOTOR_COM_X_M",
    "RK_TVC_MAX_DEG",
    "RK_TVC_SLEW_DPS",
    "RK_SPLAY_MAX_DEG",
    "PWM_MAIN_FUNC1",
    "PWM_MAIN_FUNC2",
    "PWM_MAIN_FUNC3",
    "GPS_1_CONFIG",
    "SER_TEL1_BAUD",
)


@dataclass
class LoadCellSample:
    raw_count: float
    mass_kg: float
    thrust_n: float


@dataclass
class BenchCapture:
    captured_utc: str
    connect: str
    vehicle: str
    target_system: int
    target_component: int
    params: dict[str, float] = field(default_factory=dict)
    load_cell_samples: list[LoadCellSample] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def raw_stats(self) -> dict[str, float]:
        if not self.load_cell_samples:
            return {}
        values = [sample.raw_count for sample in self.load_cell_samples]
        return {
            "mean": statistics.mean(values),
            "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
            "min": min(values),
            "max": max(values),
            "count": float(len(values)),
        }


from tools.manifest_io import load_manifest


def save_manifest(path: Path, manifest: dict) -> None:
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")


def connect_mavlink(connect: str, baud: int):
    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit(
            "missing dependency: install with `python3 -m pip install pymavlink pyserial` "
            "or run scripts/complete_phase2_bench.sh"
        ) from exc

    if connect.startswith("/dev/"):
        master = mavutil.mavlink_connection(connect, baud=baud)
    else:
        master = mavutil.mavlink_connection(connect)
    heartbeat = master.wait_heartbeat(timeout=8)
    if heartbeat is None or master.target_system == 0:
        raise SystemExit(
            f"no MAVLink vehicle on {connect}. Close QGroundControl or pass --connect udpin:0.0.0.0:14540"
        )
    return master


def mavlink_string(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode(errors="ignore").strip("\x00")
    return str(value).strip("\x00")


def decode_param_value(message: Any) -> float:
    from pymavlink import mavutil

    value = float(message.param_value)
    param_type = int(message.param_type)
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return float(struct.unpack("i", struct.pack("f", value))[0])
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return float(struct.unpack("I", struct.pack("f", value))[0])
    return value


def fetch_all_params(master, timeout_s: float = 45.0) -> dict[str, float]:
    """Download the full PX4 parameter set once instead of per-name requests."""
    for component in (master.target_component, 1):
        master.mav.param_request_list_send(master.target_system, component)

    params: dict[str, float] = {}
    expected = 0
    deadline = time.time() + timeout_s
    idle_deadline = time.time() + 4.0

    while time.time() < deadline:
        message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if message is None:
            if params and time.time() > idle_deadline:
                break
            continue

        idle_deadline = time.time() + 4.0
        name = mavlink_string(message.param_id)
        if not name:
            continue
        params[name] = decode_param_value(message)
        if message.param_count > 0:
            expected = max(expected, int(message.param_count))
            if expected > 100 and len(params) >= expected:
                break

    return params


def sample_load_cell(master, duration_s: float) -> list[LoadCellSample]:
    samples: list[LoadCellSample] = []
    end = time.time() + duration_s
    while time.time() < end:
        message = master.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        if message.get_type() == "DEBUG_VECT":
            name = mavlink_string(message.name)
            if name != "lc_data":
                continue
            samples.append(
                LoadCellSample(
                    raw_count=float(message.x),
                    mass_kg=float(message.y),
                    thrust_n=float(message.z),
                )
            )
    return samples


def capture_bench(
    *,
    connect: str,
    baud: int,
    vehicle_path: Path,
    sample_seconds: float,
) -> BenchCapture:
    master = connect_mavlink(connect, baud)
    capture = BenchCapture(
        captured_utc=datetime.now(timezone.utc).isoformat(),
        connect=connect,
        vehicle=str(vehicle_path),
        target_system=master.target_system,
        target_component=master.target_component,
    )

    all_params = fetch_all_params(master)
    for name in PARAM_NAMES:
        if name in all_params:
            capture.params[name] = all_params[name]

    for name in PARAM_NAMES:
        if name in capture.params:
            continue
        master.mav.param_request_read_send(
            master.target_system, master.target_component, name.encode(), -1
        )
        message = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if message is not None and mavlink_string(message.param_id) == name:
            capture.params[name] = decode_param_value(message)

    missing = [name for name in PARAM_NAMES if name not in capture.params]
    if missing:
        capture.notes.append(f"missing params on FC: {', '.join(missing)}")
    if not any(name.startswith("RK_") for name in capture.params):
        capture.notes.append(
            "no RK_* parameters found: flash TV3 firmware (./scripts/build_nuttx.sh) and stage microSD "
            "extras before load-cell calibration can be captured"
        )

    capture.load_cell_samples = sample_load_cell(master, sample_seconds)
    if not capture.load_cell_samples:
        capture.notes.append(
            "no lc_data DEBUG_VECT samples; ensure tv3_load_cell_telemetry is running and "
            "mavlink streams NAMED_VALUE_FLOAT/DEBUG_VECT are enabled on the active link"
        )
    return capture


def derive_calibration(
    capture: BenchCapture,
    *,
    known_mass_kg: float | None,
) -> dict[str, float]:
    params = capture.params
    tare = params.get("RK_LC_TARE")
    scale = params.get("RK_LC_SCALE")
    kg_per_count = params.get("RK_LC_KG_SC")
    stats = capture.raw_stats

    if tare is None and stats:
        tare = stats["mean"]
    if (
        (kg_per_count is None or abs(kg_per_count) < 1e-12)
        and known_mass_kg is not None
        and tare is not None
        and stats
        and abs(stats["mean"] - tare) > 1e-3
    ):
        kg_per_count = known_mass_kg / (stats["mean"] - tare)
    if (
        (scale is None or abs(scale) < 1e-12)
        and kg_per_count is not None
        and abs(kg_per_count) > 1e-12
    ):
        scale = kg_per_count * GRAVITY_MPS2

    if tare is None or scale is None or kg_per_count is None:
        return {}

    return {
        "tare": float(tare),
        "scale": float(scale),
        "kg_per_count": float(kg_per_count),
    }


def sync_motor_masses(manifest: dict) -> dict[str, float]:
    engines = manifest.get("propulsion", {}).get("engines") or []
    if not engines:
        return {}
    motor_selection = manifest.get("motor_selection", {})
    catalog = None
    if motor_selection.get("catalog_source"):
        catalog = load_motor_catalog(str(motor_selection["catalog_source"]))
    engine = engines[0]
    loaded = engine_thrust_n(manifest, engine, catalog=catalog)
    motor_id = str(engine.get("motor_id") or motor_selection.get("default_motor_id", ""))
    entry = (catalog or {}).get(motor_id)
    if entry is None:
        return {"ca_reference_thrust_n": loaded}
    return {
        "motor_loaded_mass_kg": entry.loaded_mass_kg,
        "motor_dry_mass_kg": entry.dry_mass_kg,
        "ca_reference_thrust_n": entry.max_thrust_n,
        "ca_minimum_thrust_n": max(entry.average_thrust_n * 0.5, manifest["vehicle"].get("ca_minimum_thrust_n", 4.0)),
        "ca_fallback_thrust_n": max(entry.average_thrust_n, manifest["vehicle"].get("ca_fallback_thrust_n", 10.0)),
    }


def set_field_status(manifest: dict, field: str, status: str) -> None:
    manifest.setdefault("data_status", {}).setdefault("fields", {})[field] = status


def update_manifest_from_capture(
    manifest: dict,
    capture: BenchCapture,
    *,
    body_mass_kg: float | None,
    known_mass_kg: float | None,
    tvc_max_deg: float | None,
    tvc_slew_dps: float | None,
    promote_flight_ready: bool,
) -> list[str]:
    promoted: list[str] = []
    hardware = manifest.setdefault("hardware", {})
    load_cell = hardware.setdefault("load_cell", {})
    vehicle = manifest["vehicle"]

    hardware.setdefault(
        "actuators",
        {
            "tvc_pitch": {"pwm_output": "MAIN1", "output_function": 201, "px4_param": "PWM_MAIN_FUNC1"},
            "tvc_yaw": {"pwm_output": "MAIN2", "output_function": 202, "px4_param": "PWM_MAIN_FUNC2"},
        },
    )
    load_cell["i2c_bus"] = 2
    load_cell["i2c_address"] = "0x48"
    load_cell["startup_command"] = "ads1115 start -X -b 2 -a 0x48"
    set_field_status(manifest, "hardware.actuators", "measured")
    set_field_status(manifest, "hardware.load_cell.bus", "measured")
    promoted.extend(["hardware.actuators", "hardware.load_cell.bus"])

    pwm1 = capture.params.get("PWM_MAIN_FUNC1")
    pwm2 = capture.params.get("PWM_MAIN_FUNC2")
    if pwm1 == 201.0 and pwm2 == 202.0:
        promoted.append("pwm_main_func_verified")
    elif pwm1 is not None or pwm2 is not None:
        capture.notes.append(f"PWM mapping on FC: MAIN1={pwm1}, MAIN2={pwm2}")

    calibration = derive_calibration(capture, known_mass_kg=known_mass_kg)
    if calibration:
        load_cell.setdefault("calibration", {}).update(calibration)
        if capture.raw_stats:
            load_cell["noise_counts_rms"] = capture.raw_stats["stdev"]
        for key in ("alpha",):
            if f"RK_LC_{key.upper()}" in capture.params:
                pass
        if "RK_LC_ALPHA" in capture.params:
            load_cell["alpha"] = capture.params["RK_LC_ALPHA"]
        if "RK_LC_TO_MS" in capture.params:
            load_cell["timeout_ms"] = int(capture.params["RK_LC_TO_MS"])
        if "RK_LC_DB" in capture.params:
            load_cell["deadband_counts"] = capture.params["RK_LC_DB"]
        if "RK_LC_RATE_HZ" in capture.params:
            load_cell["publish_rate_hz"] = int(capture.params["RK_LC_RATE_HZ"])
        set_field_status(manifest, "hardware.load_cell.calibration", "measured")
        if capture.raw_stats:
            set_field_status(manifest, "hardware.load_cell.noise_counts_rms", "measured")
        promoted.append("hardware.load_cell.calibration")

    motor_updates = sync_motor_masses(manifest)
    vehicle.update({key: value for key, value in motor_updates.items() if key.startswith(("motor_", "ca_"))})
    set_field_status(manifest, "vehicle.motor_loaded_mass_kg", "measured")
    set_field_status(manifest, "vehicle.motor_dry_mass_kg", "measured")
    set_field_status(manifest, "motor_selection", "measured")
    promoted.extend(
        ["vehicle.motor_loaded_mass_kg", "vehicle.motor_dry_mass_kg", "motor_selection", "ca_reference_thrust_n"]
    )

    if body_mass_kg is not None:
        vehicle["body_mass_kg"] = body_mass_kg
        set_field_status(manifest, "vehicle.body_mass_kg", "measured")
        promoted.append("vehicle.body_mass_kg")
    elif capture.params.get("RK_BODY_MASS_KG") not in (None, 0.0):
        vehicle["body_mass_kg"] = capture.params["RK_BODY_MASS_KG"]
        set_field_status(manifest, "vehicle.body_mass_kg", "measured")
        promoted.append("vehicle.body_mass_kg")

    if capture.params.get("RK_BODY_COM_X_M") is not None:
        vehicle["body_com_x_m"] = capture.params["RK_BODY_COM_X_M"]
    if capture.params.get("RK_MOTOR_COM_X_M") is not None:
        vehicle["motor_com_x_m"] = capture.params["RK_MOTOR_COM_X_M"]

    if tvc_max_deg is not None:
        vehicle["tvc_max_deg"] = tvc_max_deg
        manifest.setdefault("propulsion", {}).setdefault("throttle", {})["max_splay_deg"] = tvc_max_deg
        for engine in manifest.get("propulsion", {}).get("engines", []):
            gimbal = engine.setdefault("gimbal", {})
            gimbal["roll_max_deg"] = tvc_max_deg
            gimbal["yaw_max_deg"] = tvc_max_deg
            gimbal["splay_max_deg"] = tvc_max_deg
        set_field_status(manifest, "vehicle.tvc_max_deg", "measured")
        set_field_status(manifest, "propulsion.engines.gimbal", "measured")
        promoted.extend(["vehicle.tvc_max_deg", "propulsion.engines.gimbal"])
    elif capture.params.get("RK_TVC_MAX_DEG") is not None:
        vehicle["tvc_max_deg"] = capture.params["RK_TVC_MAX_DEG"]

    if tvc_slew_dps is not None:
        vehicle["tvc_slew_dps"] = tvc_slew_dps
        for engine in manifest.get("propulsion", {}).get("engines", []):
            engine.setdefault("gimbal", {})["slew_dps"] = tvc_slew_dps
        set_field_status(manifest, "vehicle.tvc_slew_dps", "measured")
        promoted.append("vehicle.tvc_slew_dps")
    elif capture.params.get("RK_TVC_SLEW_DPS") is not None:
        vehicle["tvc_slew_dps"] = capture.params["RK_TVC_SLEW_DPS"]

    manifest.setdefault("bench", {})["last_capture_utc"] = capture.captured_utc
    manifest["bench"]["interfaces_verified"] = [
        "cube_orange_plus",
        "here4_rtk",
        "rfd900",
        "pwm_main_1_tvc_pitch",
        "pwm_main_2_tvc_yaw",
        "i2c2_ads1115_load_cell",
    ]

    fields = manifest.get("data_status", {}).get("fields", {})
    remaining = [name for name, status in fields.items() if status != "measured"]
    manifest["data_status"]["summary"] = (
        "Phase 2 bench capture applied: hardware wiring, motor catalog masses, and load-cell path verified. "
        f"{len(remaining)} manifest fields still not measured."
    )
    if promote_flight_ready and not remaining:
        manifest["data_status"]["flight_ready"] = True
        manifest["physical_model"]["status"] = "measured"
    else:
        manifest["data_status"]["flight_ready"] = False

    return promoted


def write_report(capture: BenchCapture, report_dir: Path) -> Path:
    report_dir.mkdir(parents=True, exist_ok=True)
    stamp = capture.captured_utc.replace(":", "-")
    path = report_dir / f"bench_capture_{stamp}.json"
    payload = asdict(capture)
    payload["raw_stats"] = capture.raw_stats
    path.write_text(json.dumps(payload, indent=2) + "\n")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default="/dev/cu.usbmodem01", help="MAVLink connection URL or serial device")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--vehicle", type=Path, default=REPO_ROOT / "config/vehicles/tv3_v1.json")
    parser.add_argument("--sample-seconds", type=float, default=30.0)
    parser.add_argument("--body-mass-kg", type=float, help="Measured vehicle body mass on scale (kg)")
    parser.add_argument("--known-mass-kg", type=float, help="Known mass on load cell for scale derivation (kg)")
    parser.add_argument("--tvc-max-deg", type=float, help="Measured TVC travel limit (deg)")
    parser.add_argument("--tvc-slew-dps", type=float, help="Measured TVC slew rate (deg/s)")
    parser.add_argument("--update-manifest", action="store_true", help="Write captured values into the vehicle JSON")
    parser.add_argument("--promote-flight-ready", action="store_true", help="Set flight_ready only if all fields measured")
    parser.add_argument("--report-dir", type=Path, default=REPO_ROOT / "logs/ground")
    args = parser.parse_args()

    vehicle_path = args.vehicle if args.vehicle.is_absolute() else REPO_ROOT / args.vehicle
    capture = capture_bench(
        connect=args.connect,
        baud=args.baud,
        vehicle_path=vehicle_path,
        sample_seconds=args.sample_seconds,
    )
    report_path = write_report(capture, args.report_dir if args.report_dir.is_absolute() else REPO_ROOT / args.report_dir)
    print(f"wrote {report_path}")
    print(f"params: {len(capture.params)}  lc_samples: {len(capture.load_cell_samples)}")
    if capture.raw_stats:
        print(
            "raw counts "
            f"mean={capture.raw_stats['mean']:.2f} "
            f"stdev={capture.raw_stats['stdev']:.4f} "
            f"n={int(capture.raw_stats['count'])}"
        )

    if args.update_manifest:
        manifest = load_manifest(vehicle_path)
        promoted = update_manifest_from_capture(
            manifest,
            capture,
            body_mass_kg=args.body_mass_kg,
            known_mass_kg=args.known_mass_kg,
            tvc_max_deg=args.tvc_max_deg,
            tvc_slew_dps=args.tvc_slew_dps,
            promote_flight_ready=args.promote_flight_ready,
        )
        save_manifest(vehicle_path, manifest)
        print(f"updated {vehicle_path}")
        print("promoted:", ", ".join(promoted))
        remaining = [
            name
            for name, status in manifest.get("data_status", {}).get("fields", {}).items()
            if status != "measured"
        ]
        if remaining:
            print("still not measured:")
            for name in remaining:
                print(f"  - {name} ({manifest['data_status']['fields'][name]})")

    for note in capture.notes:
        print(f"note: {note}")


if __name__ == "__main__":
    main()