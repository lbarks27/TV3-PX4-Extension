#!/usr/bin/env python3
"""Generate TV3 runtime assets from vehicle manifest and v2 flight profile."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from copy import deepcopy
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MAX_ENGINES = 4
MAX_CONTROL_PHASES = 8

TV3_STATUS_MODES = {
    "MODE_DISARMED_SAFE": 0,
    "MODE_ARMED_STANDBY": 1,
    "MODE_READY": 2,
    "MODE_IGNITION_PENDING": 3,
    "MODE_BOOST": 4,
    "MODE_COAST": 5,
    "MODE_ABORT": 6,
}

GUIDANCE_MODES = {"off": 0, "up": 1, "waypoint_fly_through": 2}
ATTITUDE_MODES = {"off": 0, "on": 1, "large_error": 1, "small_error": 1, "deadband": 1}
MIXER_MODES = {"off": 0, "torque_only": 1, "torque_and_thrust": 2}
LOAD_CELL_MODES = {"off": 0, "monitor": 1}

LOGGER_TOPICS = [
    ("# TV3 simplified stack", None),
    ("vehicle_status", 100),
    ("vehicle_local_position", 20),
    ("vehicle_global_position", 20),
    ("vehicle_attitude_groundtruth", 20),
    ("vehicle_attitude_euler", 20),
    ("vehicle_angular_velocity", 20),
    ("vehicle_torque_setpoint", 20),
    ("tv3_sm_status", 20),
    ("tv3_sm_modes", 20),
    ("tv3_gd_att_sp", 20),
    ("tv3_gd_thr_sp", 20),
    ("tv3_mix_eng_cmd", 20),
    ("tv3_lc_eng_st", 20),
    ("tv3_mix_alloc_st", 20),
    ("tv3_lc_thrust", 50),
    ("tv3_lc_ch", 50),
    ("tv3_sih_wrench", 20),
]


def append_param(lines: list[str], name: str, value, type_code: int) -> None:
    lines.append(f"1\t1\t{name}\t{value}\t{type_code}")


def load_json(path: Path) -> dict:
    with path.open() as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object in {path}")
    return data


def vehicle_engines(vehicle: dict) -> list[dict]:
    engines = vehicle.get("propulsion", {}).get("engines")
    if engines:
        return engines
    body = vehicle["vehicle"]
    return [
        {
            "id": "engine_0",
            "motor_index": 0,
            "position_m": [body["motor_com_x_m"], 0.0, 0.0],
            "thrust_axis": [1.0, 0.0, 0.0],
            "roll_axis": [0.0, -1.0, 0.0],
            "yaw_axis": [0.0, 0.0, -1.0],
            "thrust_fraction": 1.0,
            "gimbal": {"roll_max_deg": body["tvc_max_deg"], "yaw_max_deg": body["tvc_max_deg"]},
        }
    ]


def pack_module_modes(modules: dict) -> int:
    guidance = GUIDANCE_MODES[modules["guidance"]]
    attitude = ATTITUDE_MODES[modules["attitude"]]
    mixer = MIXER_MODES[modules["mixer"]]
    load_cell = LOAD_CELL_MODES[modules["load_cell"]]
    return guidance | (attitude << 8) | (mixer << 16) | (load_cell << 24)


def apply_flight_profile(vehicle: dict, profile: dict, profile_path: Path) -> dict:
    schema = profile.get("schema", "")
    if schema not in {"tv3_flight_profile_v2", "tv3_flight_profile_v1"}:
        raise ValueError(f"{profile_path} must declare schema tv3_flight_profile_v2")

    if schema == "tv3_flight_profile_v1":
        raise ValueError(f"{profile_path} uses deprecated v1 schema; rewrite with control.phases")

    profile_vehicle = profile["vehicle"]
    if vehicle["name"] != profile_vehicle and profile_vehicle not in profile.get("compatible_vehicles", []):
        raise ValueError(f"profile {profile['name']} targets {profile_vehicle}, vehicle is {vehicle['name']}")

    merged = deepcopy(vehicle)
    merged["_active_flight_profile"] = {"name": profile["name"], "source": str(profile_path), "data": profile}
    merged["_control_phases"] = profile["control"]["phases"]
    if "guidance" in profile:
        merged["guidance"] = {**merged.get("guidance", {}), **profile["guidance"]}
    return merged


def motor_slug(manufacturer: str, designation: str) -> str:
    return f"{manufacturer}-{designation}".replace(" ", "-").lower()


def default_motor_id(vehicle: dict) -> str:
    selection = vehicle.get("motor_selection", {})
    return selection.get("default_motor_id", "aerotech-g12")


def load_motor_inventory() -> dict[str, dict[str, str]]:
    inventory_path = REPO_ROOT / "config" / "thrust_curves" / "motor_inventory.csv"
    inventory: dict[str, dict[str, str]] = {}
    with inventory_path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            slug = motor_slug(row["manufacturer"], row["designation"])
            inventory[slug] = row
    return inventory


def unique_vehicle_motor_ids(vehicle: dict) -> list[str]:
    motor_ids: list[str] = []
    seen: set[str] = set()
    fallback = default_motor_id(vehicle)
    for engine in vehicle_engines(vehicle):
        motor_id = engine.get("motor_id", fallback)
        if motor_id not in seen:
            seen.add(motor_id)
            motor_ids.append(motor_id)
    return motor_ids


def motor_catalog_index_for_engine(vehicle: dict, engine: dict) -> int:
    motor_id = engine.get("motor_id", default_motor_id(vehicle))
    return unique_vehicle_motor_ids(vehicle).index(motor_id)


def write_curve_csv(dynamics_path: Path, curve_path: Path) -> None:
    curve_path.parent.mkdir(parents=True, exist_ok=True)
    with dynamics_path.open(newline="") as src, curve_path.open("w", newline="") as dst:
        reader = csv.DictReader(src)
        writer = csv.DictWriter(
            dst,
            fieldnames=[
                "time_s",
                "thrust_n",
                "motor_mass_kg",
                "burn_fraction",
                "cumulative_impulse_ns",
            ],
        )
        writer.writeheader()
        for row in reader:
            writer.writerow(
                {
                    "time_s": row["time_s"],
                    "thrust_n": row["thrust_N"],
                    "motor_mass_kg": row["motor_mass_kg"],
                    "burn_fraction": row["burn_fraction"],
                    "cumulative_impulse_ns": row["cumulative_impulse_Ns"],
                }
            )


def write_specs_csv(raw_specs_path: Path, specs_path: Path, motor_id: str) -> None:
    with raw_specs_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        row = next(reader)
    specs_path.parent.mkdir(parents=True, exist_ok=True)
    with specs_path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "motor_id",
                "manufacturer",
                "designation",
                "loaded_mass_kg",
                "dry_mass_kg",
                "diameter_m",
                "length_m",
                "total_impulse_ns",
                "burn_duration_s",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "motor_id": motor_id,
                "manufacturer": row["manufacturer"],
                "designation": row["designation"],
                "loaded_mass_kg": f"{float(row['initial_mass_g']) / 1000.0:.6f}",
                "dry_mass_kg": f"{float(row['dry_mass_g']) / 1000.0:.6f}",
                "diameter_m": f"{float(row['diameter_mm']) / 1000.0:.6f}",
                "length_m": f"{float(row['length_mm']) / 1000.0:.6f}",
                "total_impulse_ns": f"{float(row['total_impulse_curve_Ns']):.6f}",
                "burn_duration_s": f"{float(row['burn_time_curve_s']):.6f}",
            }
        )


def write_motor_assets(vehicle: dict, output_root: Path) -> None:
    motors_root = output_root / "runtime" / "fs" / "microsd" / "tv3" / "motors"
    if motors_root.exists():
        shutil.rmtree(motors_root)
    motors_root.mkdir(parents=True, exist_ok=True)

    curves_root = REPO_ROOT / "config" / "thrust_curves"
    inventory = load_motor_inventory()
    catalog_rows: list[dict[str, str | int]] = []

    for index, motor_id in enumerate(unique_vehicle_motor_ids(vehicle)):
        inventory_row = inventory.get(motor_id)
        if inventory_row is None:
            raise ValueError(f"motor_id {motor_id} missing from motor_inventory.csv")

        dynamics_path = curves_root / inventory_row["dynamics_file"]
        raw_specs_path = curves_root / inventory_row["specs_file"]
        motor_dir = motors_root / motor_id
        curve_path = motor_dir / "curve.csv"
        specs_path = motor_dir / "specs.csv"
        write_curve_csv(dynamics_path, curve_path)
        write_specs_csv(raw_specs_path, specs_path, motor_id)
        catalog_rows.append(
            {
                "motor_index": index,
                "motor_id": motor_id,
                "curve_file": f"{motor_id}/curve.csv",
                "specs_file": f"{motor_id}/specs.csv",
                "active": 1,
            }
        )

    catalog_path = motors_root / "catalog.csv"
    with catalog_path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["motor_index", "motor_id", "curve_file", "specs_file", "active"],
        )
        writer.writeheader()
        writer.writerows(catalog_rows)


def write_control_phase_params(vehicle: dict, lines: list[str]) -> None:
    phases = vehicle.get("_control_phases", [])
    append_param(lines, "RK_CTRL_NPHASE", len(phases), 6)

    for index in range(MAX_CONTROL_PHASES):
        if index < len(phases):
            phase = phases[index]
            on_mode = TV3_STATUS_MODES[phase["on_mode"]]
            packed = pack_module_modes(phase["modules"])
            append_param(lines, f"RK_CTRL_P{index}_ON", on_mode, 6)
            append_param(lines, f"RK_CTRL_P{index}_MODES", packed, 6)
        else:
            append_param(lines, f"RK_CTRL_P{index}_ON", 0, 6)
            append_param(lines, f"RK_CTRL_P{index}_MODES", 0, 6)


def write_guidance_params(vehicle: dict, lines: list[str]) -> None:
    guidance = vehicle.get("guidance", {})
    append_param(lines, "RK_GD_WP1_N", guidance.get("wp1_n_m", 0.0), 9)
    append_param(lines, "RK_GD_WP1_E", guidance.get("wp1_e_m", 0.0), 9)
    append_param(lines, "RK_GD_WP1_D", guidance.get("wp1_d_m", -20.0), 9)
    append_param(lines, "RK_GD_POS_P", guidance.get("pos_p", 0.15), 9)
    append_param(lines, "RK_GD_ACC_RAD", guidance.get("acceptance_m", 5.0), 9)
    append_param(lines, "RK_GD_VMAX_MS", guidance.get("vel_max_m_s", 12.0), 9)
    append_param(lines, "RK_GD_TILT_MAX", guidance.get("tilt_max_deg", 35.0), 9)
    append_param(lines, "RK_GD_SIM_GT", guidance.get("sim_groundtruth_fallback", 1), 6)


def write_px4_params(vehicle: dict, path: Path) -> None:
    body = vehicle["vehicle"]
    state_machine = vehicle["state_machine"]
    hardware = vehicle["hardware"]
    load_cell = hardware["load_cell"]
    simulation = vehicle.get("simulation", {})
    controller = vehicle.get("controller", {})
    engines = vehicle_engines(vehicle)
    propulsion = vehicle.get("propulsion", {})
    ignition = propulsion.get("ignition", {})
    throttle = propulsion.get("throttle", {})
    sequence = [int(value) for value in ignition.get("sequence", list(range(len(engines))))]
    body_mass_kg = float(simulation.get("sih_body_mass_kg", body["body_mass_kg"]))
    per_engine_thrust_n = float(body["ca_reference_thrust_n"]) / max(len(engines), 1)

    lines: list[str] = []
    append_param(lines, "RK_ENG_COUNT", len(engines), 6)

    for index, engine in enumerate(engines):
        gimbal = engine["gimbal"]
        position = engine["position_m"]
        axis = engine["thrust_axis"]
        roll_axis = engine.get("roll_axis", engine.get("pitch_axis", [0.0, -1.0, 0.0]))
        yaw_axis = engine["yaw_axis"]
        motor_catalog_index = motor_catalog_index_for_engine(vehicle, engine)
        append_param(lines, f"RK_ENG{index}_MOT", motor_catalog_index, 6)
        append_param(lines, f"RK_G{index}_PX", position[0], 9)
        append_param(lines, f"RK_G{index}_PY", position[1], 9)
        append_param(lines, f"RK_G{index}_PZ", position[2], 9)
        append_param(lines, f"RK_G{index}_AX", axis[0], 9)
        append_param(lines, f"RK_G{index}_AY", axis[1], 9)
        append_param(lines, f"RK_G{index}_AZ", axis[2], 9)
        append_param(lines, f"RK_G{index}_PAX", roll_axis[0], 9)
        append_param(lines, f"RK_G{index}_PAY", roll_axis[1], 9)
        append_param(lines, f"RK_G{index}_PAZ", roll_axis[2], 9)
        append_param(lines, f"RK_G{index}_YAX", yaw_axis[0], 9)
        append_param(lines, f"RK_G{index}_YAY", yaw_axis[1], 9)
        append_param(lines, f"RK_G{index}_YAZ", yaw_axis[2], 9)
        append_param(
            lines,
            f"RK_G{index}_PMAX",
            gimbal.get("roll_max_deg", gimbal.get("pitch_max_deg", body["tvc_max_deg"])),
            9,
        )
        append_param(lines, f"RK_G{index}_YMIN", gimbal.get("yaw_min_deg", 0.0), 9)
        append_param(lines, f"RK_G{index}_YMAX", gimbal.get("yaw_max_deg", body["tvc_max_deg"]), 9)
        append_param(lines, f"RK_G{index}_TF", engine.get("thrust_fraction", 1.0 / len(engines)), 9)
        append_param(lines, f"RK_G{index}_PTR", gimbal.get("roll_trim", gimbal.get("pitch_trim", 0.0)), 9)
        append_param(lines, f"RK_G{index}_YTR", gimbal.get("yaw_trim", 0.0), 9)

    append_param(lines, "RK_ENABLE", 1, 6)
    append_param(lines, "RK_CMD_SRC", state_machine.get("command_source", 1), 6)
    append_param(lines, "RK_IGN_DWELL_MS", ignition.get("dwell_ms", 0), 6)
    append_param(lines, "RK_SPLAY_MAX_DEG", throttle.get("max_splay_deg", body["tvc_max_deg"]), 9)

    for index in range(MAX_ENGINES):
        append_param(lines, f"RK_IGN_IDX{index}", sequence[index] if index < len(sequence) else index, 6)

    append_param(lines, "RK_LAUNCH_THR_N", state_machine["launch_threshold_n"], 9)
    append_param(lines, "RK_IGNITION_MS", state_machine["ignition_pulse_ms"], 6)
    append_param(lines, "RK_IGN_TO_MS", state_machine["ignition_timeout_ms"], 6)
    append_param(lines, "RK_BURN_MIN_MS", state_machine["minimum_burn_ms"], 6)
    append_param(lines, "RK_BURN_MAX_MS", state_machine["maximum_burn_ms"], 6)
    append_param(lines, "RK_BURNOUT_N", state_machine["burnout_threshold_n"], 9)
    append_param(lines, "RK_BURNOUT_MS", state_machine["burnout_dwell_ms"], 6)
    append_param(lines, "RK_RAIL_LEN_M", body["rail_length_m"], 9)
    append_param(lines, "RK_ABORT_GCS", state_machine.get("abort_on_gcs_loss", 0), 6)

    append_param(lines, "RK_BODY_MASS_KG", body_mass_kg, 9)
    append_param(lines, "RK_BODY_COM_X_M", body["body_com_x_m"], 9)
    append_param(lines, "RK_MOTOR_COM_X_M", body["motor_com_x_m"], 9)
    append_param(lines, "RK_TVC_MAX_DEG", body["tvc_max_deg"], 9)
    append_param(lines, "RK_TVC_SLEW_DPS", body["tvc_slew_dps"], 9)
    append_param(lines, "RK_TQ_R_MAX", body["torque_limits_nm"].get("roll", 0.0), 9)
    append_param(lines, "RK_TQ_P_MAX", body["torque_limits_nm"]["pitch"], 9)
    append_param(lines, "RK_TQ_Y_MAX", body["torque_limits_nm"]["yaw"], 9)

    inertia = {}
    for link in vehicle.get("physical_model", {}).get("links", []) or []:
        if isinstance(link, dict) and link.get("id") in ("body", "base", "base_vehicle_without_tvc_moving_links"):
            inertia = link.get("inertia_kg_m2", {}) or {}
            break
    if not inertia:
        for assy in vehicle.get("physical_model", {}).get("assemblies", []) or []:
            if isinstance(assy, dict):
                inertia = assy.get("inertia_about_origin_kg_m2", {}) or inertia
    sih_inertia = simulation.get("sih_inertia_kg_m2", {})
    if isinstance(sih_inertia, dict):
        inertia = {**inertia, **sih_inertia}
    append_param(lines, "RK_IXX", float(inertia.get("ixx", 0.43)), 9)
    append_param(lines, "RK_IYY", float(inertia.get("iyy", 0.43)), 9)
    append_param(lines, "RK_IZZ", float(inertia.get("izz", 0.05)), 9)

    append_param(lines, "RK_ATT_LD", controller.get("attitude_ld", 0.05), 9)
    append_param(lines, "RK_ATT_POS_KP", controller.get("attitude_p", {}).get("free", 3.0), 9)
    append_param(lines, "RK_ATT_VEL_KP", controller.get("att_vel_p", 2.0), 9)
    append_param(lines, "RK_ATT_VEL_KI", controller.get("rate_i", 0.0), 9)
    append_param(lines, "RK_ATT_VEL_KD", controller.get("rate_d", 0.003), 9)

    append_param(lines, "RK_LC_SRC", 1 if simulation.get("load_cell_source", "reference") == "reference" else load_cell.get("source", 0), 6)
    append_param(lines, "RK_LC_CH", load_cell["adc_channel"], 6)
    append_param(lines, "RK_LC_NEG_CH", load_cell.get("negative_channel", 1), 6)
    append_param(lines, "RK_LC_ADC_INST", load_cell.get("adc_instance", 1), 6)
    append_param(lines, "RK_LC_TARE", load_cell["calibration"]["tare"], 9)
    append_param(lines, "RK_LC_SCALE", load_cell["calibration"]["scale"], 9)
    append_param(lines, "RK_LC_ALPHA", load_cell.get("alpha", 0.25), 9)
    append_param(lines, "RK_LC_TO_MS", load_cell.get("timeout_ms", 200), 6)
    append_param(lines, "RK_LC_EXP_THR_N", per_engine_thrust_n, 9)
    append_param(lines, "RK_LC_EXP_MASS", body["motor_loaded_mass_kg"], 9)
    append_param(lines, "RK_LC_EXP_VEH", body_mass_kg + body["motor_loaded_mass_kg"] * len(engines), 9)

    write_guidance_params(vehicle, lines)
    write_control_phase_params(vehicle, lines)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def write_logger_topics(path: Path) -> None:
    lines = [
        "# TV3 ULog review profile.",
        "# Format: <uORB topic> <minimum interval ms> [instance]",
    ]
    for topic, interval_ms in LOGGER_TOPICS:
        if interval_ms is None:
            lines.append("")
            lines.append(topic)
        else:
            lines.append(f"{topic} {interval_ms} 0")
    text = "\n".join(lines).rstrip() + "\n"
    for logging_path in (path / "etc" / "logging", path / "fs" / "microsd" / "etc" / "logging"):
        logging_path.mkdir(parents=True, exist_ok=True)
        (logging_path / "logger_topics.txt").write_text(text)


def write_active_flight_profile(vehicle: dict, path: Path) -> None:
    active_profile = vehicle.get("_active_flight_profile")
    if not active_profile:
        return
    profile_text = json.dumps(active_profile["data"], indent=2, ensure_ascii=False) + "\n"
    for profile_path in (path / "etc" / "flight_profiles", path / "fs" / "microsd" / "tv3" / "flight_profiles"):
        profile_path.mkdir(parents=True, exist_ok=True)
        (profile_path / "active.json").write_text(profile_text)
        (profile_path / f"{active_profile['name']}.json").write_text(profile_text)


def write_runtime_assets(vehicle: dict, output_root: Path) -> None:
    runtime = output_root / "runtime"
    airframe_path = runtime / "fs" / "microsd" / "tv3" / "airframes"
    write_px4_params(vehicle, airframe_path / f"{vehicle['name']}.params")
    write_motor_assets(vehicle, output_root)
    write_logger_topics(runtime)
    write_active_flight_profile(vehicle, runtime)


def generate_assets(vehicle_path: Path, output_root: Path, flight_profile_path: Path | None = None) -> None:
    vehicle = load_json(vehicle_path)
    if flight_profile_path is not None:
        vehicle = apply_flight_profile(vehicle, load_json(flight_profile_path), flight_profile_path)
    write_runtime_assets(vehicle, output_root)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, required=True)
    parser.add_argument("--flight-profile", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate_assets(args.vehicle, args.output, args.flight_profile)
    profile_note = f" with profile {args.flight_profile}" if args.flight_profile else ""
    print(f"generated assets for {args.vehicle}{profile_note} into {args.output}")


if __name__ == "__main__":
    main()
