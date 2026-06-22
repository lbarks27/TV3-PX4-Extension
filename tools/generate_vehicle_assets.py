#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
MAX_ENGINES = 4
GUIDANCE_KEYS = {
    "enable",
    "takeoff_alt_m",
    "apex_alt_m",
    "pos_p",
    "vel_max_m_s",
    "vel_up_m_s",
    "vel_dn_m_s",
    "yaw_deg",
    "hold_alt_m",
    "acceptance_m",
    "min_twr",
    "landing_twr",
    "min_remaining_impulse_ns",
    "wp1_n_m",
    "wp1_e_m",
    "wp1_d_m",
    "wp2_n_m",
    "wp2_e_m",
    "wp2_d_m",
    "wp3_n_m",
    "wp3_e_m",
    "wp3_d_m",
    "land_n_m",
    "land_e_m",
    "land_d_m",
    "sim_groundtruth_fallback",
    "tilt_gain",
    "tilt_max_deg",
    "ascent_mode",
    "apogee_mode",
    "landing_mode",
    "wp1_mode",
    "wp2_mode",
    "wp3_mode",
    "wp1_hold_s",
    "wp2_hold_s",
    "wp3_hold_s",
    "wp1_acceptance_m",
    "wp2_acceptance_m",
    "wp3_acceptance_m",
    "wp1_cruise_m_s",
    "wp2_cruise_m_s",
    "wp3_cruise_m_s",
    "roll_prog_deg",
    "roll_prog_start_s",
    "roll_prog_dur_s",
}
ASCENT_MODES = {"launch_ascent": 0, "hover_window": 1}
APOGEE_MODES = {"track": 0, "skip": 1}
LANDING_MODES = {"approach": 0, "skip": 1}
WP_MODES = {"fly_through": 0, "position_hold": 1}


def guidance_mode_value(mapping: dict[str, int], value, default: str) -> int:
    if value is None:
        return mapping[default]
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        key = value.strip().lower()
        if key not in mapping:
            allowed = ", ".join(sorted(mapping))
            raise ValueError(f"unknown guidance mode '{value}'; expected one of: {allowed}")
        return mapping[key]
    raise ValueError(f"guidance mode must be a string or integer, got {type(value).__name__}")
LOGGER_TOPICS = [
    ("# Core PX4 state", None),
    ("vehicle_status", 100),
    ("vehicle_status_flags", 100),
    ("health_report", 100),
    ("failsafe_flags", 100),
    ("actuator_armed", 100),
    ("vehicle_attitude", 20),
    ("vehicle_local_position", 50),
    ("vehicle_local_position_groundtruth", 50),
    ("# Control allocation and commanded wrench", None),
    ("control_allocator_status", 50),
    ("actuator_motors", 50),
    ("actuator_servos", 50),
    ("internal_combustion_engine_control", 50),
    ("vehicle_torque_setpoint", 50),
    ("vehicle_thrust_setpoint", 50),
    ("trajectory_setpoint", 50),
    ("# TV3 tv3-specific review topics", None),
    ("tv3_command", 0),
    ("tv3_engine_command", 20),
    ("tv3_engine_state", 20),
    ("tv3_guidance_status", 20),
    ("tv3_load_cell", 50),
    ("tv3_mode_status", 20),
    ("tv3_motor_reference", 20),
    ("tv3_status", 20),
    ("tv3_thrust", 50),
]


from tools.manifest_io import load_manifest as load_vehicle_manifest


def load_vehicle(path: Path) -> dict:
    return load_vehicle_manifest(path)


def load_flight_profile(path: Path) -> dict:
    with path.open() as stream:
        profile = json.load(stream)
    if not isinstance(profile, dict):
        raise ValueError(f"flight profile must be a JSON object: {path}")
    return profile


def resolve_flight_profile(path: Path) -> dict:
    profile = load_flight_profile(path)
    extends = profile.get("extends")
    if not extends:
        return profile
    base_path = Path(extends)
    if not base_path.is_absolute():
        base_path = (path.parent / base_path).resolve()
    base = resolve_flight_profile(base_path)
    merged = deepcopy(base)
    for key, value in profile.items():
        if key == "extends":
            continue
        if key == "guidance" and isinstance(value, dict):
            merged_guidance = dict(merged.get("guidance", {}))
            merged_guidance.update(value)
            merged[key] = merged_guidance
        else:
            merged[key] = value
    return merged


def repo_display_path(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def apply_flight_profile(vehicle: dict, profile: dict, profile_path: Path) -> dict:
    if profile.get("schema") != "tv3_flight_profile_v1":
        raise ValueError(f"flight profile {profile_path} must declare schema: tv3_flight_profile_v1")

    for key in ("name", "vehicle", "scenario", "guidance", "mission_profile"):
        if key not in profile:
            raise ValueError(f"flight profile {profile_path} missing required section: {key}")

    vehicle_name = vehicle["name"]
    profile_vehicle = profile["vehicle"]
    compatible_vehicles = profile.get("compatible_vehicles", [profile_vehicle])
    if isinstance(compatible_vehicles, str):
        compatible_vehicles = [compatible_vehicles]
    if vehicle_name not in compatible_vehicles:
        raise ValueError(
            f"flight profile {profile['name']} targets {compatible_vehicles}, "
            f"but selected vehicle is {vehicle_name}"
        )

    guidance = profile.get("guidance", {})
    if not isinstance(guidance, dict):
        raise ValueError(f"flight profile {profile['name']} guidance must be a mapping")

    unknown_guidance = sorted(set(guidance) - GUIDANCE_KEYS)
    if unknown_guidance:
        raise ValueError(f"flight profile {profile['name']} has unknown guidance fields: {unknown_guidance}")

    merged = deepcopy(vehicle)
    merged_guidance = dict(merged.get("guidance", {}))
    merged_guidance.update(guidance)
    merged["guidance"] = merged_guidance

    merged_mission = dict(merged.get("mission_profile", {}))
    merged_mission.update(profile.get("mission_profile", {}))
    merged["mission_profile"] = merged_mission
    merged["_active_flight_profile"] = {
        "name": profile["name"],
        "source": repo_display_path(profile_path),
        "data": profile,
    }
    return merged


def append_param(lines: list[str], name: str, value, type_code: int) -> None:
    lines.append(f"1\t1\t{name}\t{value}\t{type_code}")


def vehicle_engines(vehicle: dict) -> list[dict]:
    propulsion = vehicle.get("propulsion", {})
    engines = propulsion.get("engines")
    if engines:
        return engines

    body = vehicle["vehicle"]
    motor = vehicle["motor_selection"]
    return [
        {
            "id": "engine_0",
            "motor_index": motor["index"],
            "load_cell_channel": vehicle["hardware"]["load_cell"]["adc_channel"],
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


def ignition_sequence(vehicle: dict, engines: list[dict]) -> list[int]:
    ignition = vehicle.get("propulsion", {}).get("ignition", {})
    sequence = ignition.get("sequence", list(range(len(engines))))
    return [int(value) for value in sequence]


def catalog_motor_index(vehicle: dict, engine: dict | None = None) -> int:
    """Map an engine's motor_id to the firmware motor-catalog index (RK_ENGx_MOT)."""
    from tools.tv3_motor_catalog import load_motor_catalog, resolve_motor_id

    motor_selection = vehicle.get("motor_selection", {})
    engines = vehicle_engines(vehicle)
    target_engine = engine if engine is not None else (engines[0] if engines else {})

    catalog_source = motor_selection.get("catalog_source")
    if catalog_source:
        catalog = load_motor_catalog(str(catalog_source))
        motor_id = resolve_motor_id(target_engine, motor_selection)
        if motor_id in catalog:
            return catalog[motor_id].motor_index

    if engine is not None:
        return int(engine.get("motor_index", motor_selection.get("index", 0)))
    return int(motor_selection.get("index", 0))


def _validator_module():
    import importlib.util
    import sys

    validator_path = Path(__file__).resolve().parent / "validate_vehicle_manifest.py"
    spec = importlib.util.spec_from_file_location("validate_vehicle_manifest", validator_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"unable to load validator module from {validator_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def validate_vehicle(vehicle: dict, vehicle_path: Path | None = None) -> None:
    validator = _validator_module()
    manifest_path = vehicle_path or Path("<in-memory>")
    schema = validator.load_json(validator.DEFAULT_SCHEMA)
    report = validator.validate_manifest(vehicle, manifest_path, schema)
    if report.passed:
        return

    failures = [f"{check.name}: {check.detail}" for check in report.checks if not check.passed]
    raise ValueError("vehicle manifest failed intake validation:\n  - " + "\n  - ".join(failures))


def allocator_thrust_fields(vehicle: dict) -> dict[str, float]:
    from tools.tv3_motor_catalog import allocator_thrust_fields_from_catalog

    body = vehicle["vehicle"]
    catalog_fields = allocator_thrust_fields_from_catalog(vehicle)
    if catalog_fields is not None:
        return catalog_fields
    return {
        "ca_reference_thrust_n": float(body["ca_reference_thrust_n"]),
        "ca_minimum_thrust_n": float(body["ca_minimum_thrust_n"]),
        "ca_fallback_thrust_n": float(body["ca_fallback_thrust_n"]),
    }


def write_px4_params(vehicle: dict, path: Path) -> None:
    controller = vehicle["controller"]
    state_machine = vehicle["state_machine"]
    hardware = vehicle["hardware"]
    body = vehicle["vehicle"]
    allocator_thrust = allocator_thrust_fields(vehicle)
    motor = vehicle["motor_selection"]
    load_cell = hardware["load_cell"]
    guidance = vehicle.get("guidance", {})
    engines = vehicle_engines(vehicle)
    propulsion = vehicle.get("propulsion", {})
    ignition = propulsion.get("ignition", {})
    throttle = propulsion.get("throttle", {})
    sequence = ignition_sequence(vehicle, engines)

    def g(key: str, default):
        return guidance.get(key, default)

    lines: list[str] = []
    append_param(lines, "CA_AIRFRAME", 16, 6)
    append_param(lines, "CA_RK_GRP_CNT", len(engines), 6)
    append_param(lines, "CA_RK_REF_THR", allocator_thrust["ca_reference_thrust_n"], 9)
    append_param(lines, "CA_RK_MIN_THR", allocator_thrust["ca_minimum_thrust_n"], 9)
    append_param(lines, "CA_RK_FAL_THR", allocator_thrust["ca_fallback_thrust_n"], 9)
    append_param(lines, "CA_RK_BODY_M", body["body_mass_kg"], 9)
    append_param(lines, "CA_RK_BODY_CMX", body["body_com_x_m"], 9)
    append_param(lines, "CA_RK_MOT_WET", body["motor_loaded_mass_kg"], 9)
    append_param(lines, "CA_RK_MOT_DRY", body["motor_dry_mass_kg"], 9)
    append_param(lines, "CA_RK_MOT_CMX", body["motor_com_x_m"], 9)

    for index, engine in enumerate(engines):
        gimbal = engine["gimbal"]
        position = engine["position_m"]
        axis = engine["thrust_axis"]
        roll_axis = engine.get("roll_axis", engine.get("pitch_axis", [0.0, -1.0, 0.0]))
        yaw_axis = engine["yaw_axis"]
        append_param(lines, f"CA_RK_G{index}_PX", position[0], 9)
        append_param(lines, f"CA_RK_G{index}_PY", position[1], 9)
        append_param(lines, f"CA_RK_G{index}_PZ", position[2], 9)
        append_param(lines, f"CA_RK_G{index}_AX", axis[0], 9)
        append_param(lines, f"CA_RK_G{index}_AY", axis[1], 9)
        append_param(lines, f"CA_RK_G{index}_AZ", axis[2], 9)
        # Firmware params retain the legacy PAX/PMAX/PTR names for the primary (+/-90 deg) gimbal DOF.
        append_param(lines, f"CA_RK_G{index}_PAX", roll_axis[0], 9)
        append_param(lines, f"CA_RK_G{index}_PAY", roll_axis[1], 9)
        append_param(lines, f"CA_RK_G{index}_PAZ", roll_axis[2], 9)
        append_param(lines, f"CA_RK_G{index}_YAX", yaw_axis[0], 9)
        append_param(lines, f"CA_RK_G{index}_YAY", yaw_axis[1], 9)
        append_param(lines, f"CA_RK_G{index}_YAZ", yaw_axis[2], 9)
        append_param(
            lines,
            f"CA_RK_G{index}_PMAX",
            gimbal.get("roll_max_deg", gimbal.get("pitch_max_deg", body["tvc_max_deg"])),
            9,
        )
        append_param(lines, f"CA_RK_G{index}_YMAX", gimbal.get("yaw_max_deg", body["tvc_max_deg"]), 9)
        append_param(lines, f"CA_RK_G{index}_TF", engine.get("thrust_fraction", 1.0 / len(engines)), 9)
        append_param(lines, f"CA_RK_G{index}_PTR", gimbal.get("roll_trim", gimbal.get("pitch_trim", 0.0)), 9)
        append_param(lines, f"CA_RK_G{index}_YTR", gimbal.get("yaw_trim", 0.0), 9)

    append_param(lines, "RK_ENABLE", 1, 6)
    append_param(lines, "RK_CMD_SRC", state_machine.get("command_source", 1), 6)
    append_param(lines, "RK_MOT_IDX", catalog_motor_index(vehicle, engines[0] if engines else None), 6)
    append_param(lines, "RK_ENG_COUNT", len(engines), 6)
    append_param(lines, "RK_IGN_DWELL_MS", ignition.get("dwell_ms", 0), 6)
    append_param(lines, "RK_SPLAY_MAX_DEG", throttle.get("max_splay_deg", body["tvc_max_deg"]), 9)
    default_catalog_index = catalog_motor_index(vehicle)
    for index in range(MAX_ENGINES):
        engine = engines[index] if index < len(engines) else None
        sequence_value = sequence[index] if index < len(sequence) else index
        append_param(lines, f"RK_IGN_IDX{index}", sequence_value, 6)
        motor_catalog_index = catalog_motor_index(vehicle, engine) if engine else default_catalog_index
        append_param(lines, f"RK_ENG{index}_MOT", motor_catalog_index, 6)

    append_param(lines, "RK_LAUNCH_THR_N", state_machine["launch_threshold_n"], 9)
    append_param(lines, "RK_IGNITION_MS", state_machine["ignition_pulse_ms"], 6)
    append_param(lines, "RK_IGN_TO_MS", state_machine["ignition_timeout_ms"], 6)
    append_param(lines, "RK_BURN_MIN_MS", state_machine["minimum_burn_ms"], 6)
    append_param(lines, "RK_BURN_MAX_MS", state_machine["maximum_burn_ms"], 6)
    append_param(lines, "RK_BURNOUT_N", state_machine["burnout_threshold_n"], 9)
    append_param(lines, "RK_BURNOUT_MS", state_machine["burnout_dwell_ms"], 6)
    append_param(lines, "RK_RAIL_LEN_M", body["rail_length_m"], 9)
    append_param(lines, "RK_ABORT_GCS", state_machine.get("abort_on_gcs_loss", 0), 6)
    append_param(lines, "RK_BODY_MASS_KG", body["body_mass_kg"], 9)
    append_param(lines, "RK_BODY_COM_X_M", body["body_com_x_m"], 9)
    append_param(lines, "RK_MOTOR_COM_X_M", body["motor_com_x_m"], 9)

    # Inertia tensor (diagonal) from physical_model if present. Lets SIH (and other consumers)
    # use the manifest values instead of hard-coded defaults. Falls back to small positive values.
    ixx = 0.1
    iyy = 0.1
    izz = 0.01
    phys = vehicle.get("physical_model", {}) or {}
    inertia = {}
    for link in phys.get("links", []) or []:
        if isinstance(link, dict):
            if link.get("id") in ("body", "base", "base_vehicle_without_tvc_moving_links"):
                inertia = link.get("inertia_kg_m2", {}) or inertia
                break
            if not inertia:
                inertia = link.get("inertia_kg_m2", {}) or {}
    if not inertia:
        # assemblies form (tv3_v1 style) or direct
        for assy in phys.get("assemblies", []) or []:
            if isinstance(assy, dict) and "inertia_about_origin_kg_m2" in assy:
                inertia = assy["inertia_about_origin_kg_m2"]
                break
        if not inertia:
            inertia = phys.get("inertia_kg_m2", {}) or phys.get("inertia_about_origin_kg_m2", {}) or {}
    if isinstance(inertia, dict):
        ixx = float(inertia.get("ixx", ixx))
        iyy = float(inertia.get("iyy", iyy))
        izz = float(inertia.get("izz", izz))
    append_param(lines, "RK_IXX", ixx, 9)
    append_param(lines, "RK_IYY", iyy, 9)
    append_param(lines, "RK_IZZ", izz, 9)

    append_param(lines, "RK_TVC_MAX_DEG", body["tvc_max_deg"], 9)
    append_param(lines, "RK_TVC_SLEW_DPS", body["tvc_slew_dps"], 9)
    append_param(lines, "RK_TQ_R_MAX", body["torque_limits_nm"].get("roll", 0.0), 9)
    append_param(lines, "RK_TQ_P_MAX", body["torque_limits_nm"]["pitch"], 9)
    append_param(lines, "RK_TQ_Y_MAX", body["torque_limits_nm"]["yaw"], 9)
    append_param(lines, "RK_ATT_P_RAIL", controller["attitude_p"]["rail"], 9)
    append_param(lines, "RK_ATT_P_FREE", controller["attitude_p"]["free"], 9)
    append_param(lines, "RK_ATT_P_BOOST", controller["attitude_p"].get("boost", controller["attitude_p"]["free"]), 9)
    append_param(lines, "RK_RATE_P_RAIL", controller["rate_p"]["rail"], 9)
    append_param(lines, "RK_RATE_P_FREE", controller["rate_p"]["free"], 9)
    append_param(lines, "RK_RATE_P_BOOST", controller["rate_p"].get("boost", controller["rate_p"]["free"]), 9)
    append_param(lines, "RK_RATE_I", controller["rate_i"], 9)
    append_param(lines, "RK_RATE_D", controller["rate_d"], 9)
    append_param(lines, "RK_INT_LIM_BOOST", controller.get("integrator_limit_boost", 15.0), 9)
    append_param(lines, "RK_LC_SRC", load_cell.get("source", 0), 6)
    append_param(lines, "RK_LC_CH", load_cell["adc_channel"], 6)
    append_param(lines, "RK_LC_NEG_CH", load_cell.get("negative_channel", 1), 6)
    append_param(lines, "RK_LC_ADC_INST", load_cell.get("adc_instance", 1), 6)
    append_param(lines, "RK_LC_MODE", 1 if load_cell.get("mode", "single_ended") == "differential" else 0, 6)
    append_param(lines, "RK_LC_TARE", load_cell["calibration"]["tare"], 9)
    append_param(lines, "RK_LC_SCALE", load_cell["calibration"]["scale"], 9)
    append_param(lines, "RK_LC_KG_SC", load_cell["calibration"].get("kg_per_count", 0.0), 9)
    append_param(lines, "RK_LC_ALPHA", load_cell.get("alpha", 0.45), 9)
    append_param(lines, "RK_LC_DB", load_cell.get("deadband_counts", 2.0), 9)
    append_param(lines, "RK_LC_TO_MS", load_cell.get("timeout_ms", 100), 6)
    append_param(lines, "RK_LC_RATE_HZ", load_cell.get("publish_rate_hz", 10), 6)
    append_param(lines, "RK_GD_ENABLE", g("enable", 0), 6)
    append_param(lines, "RK_GD_TAKE_ALT", g("takeoff_alt_m", 35.0), 9)
    append_param(lines, "RK_GD_APEX_ALT", g("apex_alt_m", 120.0), 9)
    append_param(lines, "RK_GD_POS_P", g("pos_p", 0.15), 9)
    append_param(lines, "RK_GD_VMAX_MS", g("vel_max_m_s", 30.0), 9)
    append_param(lines, "RK_GD_VUP_MS", g("vel_up_m_s", 15.0), 9)
    append_param(lines, "RK_GD_VDN_MS", g("vel_dn_m_s", 8.0), 9)
    append_param(lines, "RK_GD_YAW_DEG", g("yaw_deg", 0.0), 9)
    append_param(lines, "RK_GD_HOLD_ALT", g("hold_alt_m", 5.0), 9)
    append_param(lines, "RK_GD_ACC_RAD", g("acceptance_m", 15.0), 9)
    append_param(lines, "RK_GD_TWR_MIN", g("min_twr", 1.05), 9)
    append_param(lines, "RK_GD_LAND_TWR", g("landing_twr", 1.15), 9)
    append_param(lines, "RK_GD_MIN_IMP_NS", g("min_remaining_impulse_ns", 0.0), 9)
    append_param(lines, "RK_GD_WP1_N", g("wp1_n_m", 60.0), 9)
    append_param(lines, "RK_GD_WP1_E", g("wp1_e_m", 0.0), 9)
    append_param(lines, "RK_GD_WP1_D", g("wp1_d_m", -60.0), 9)
    append_param(lines, "RK_GD_WP2_N", g("wp2_n_m", 150.0), 9)
    append_param(lines, "RK_GD_WP2_E", g("wp2_e_m", 30.0), 9)
    append_param(lines, "RK_GD_WP2_D", g("wp2_d_m", -90.0), 9)
    append_param(lines, "RK_GD_WP3_N", g("wp3_n_m", 220.0), 9)
    append_param(lines, "RK_GD_WP3_E", g("wp3_e_m", 80.0), 9)
    append_param(lines, "RK_GD_WP3_D", g("wp3_d_m", -75.0), 9)
    append_param(lines, "RK_GD_LAND_N", g("land_n_m", 0.0), 9)
    append_param(lines, "RK_GD_LAND_E", g("land_e_m", 0.0), 9)
    append_param(lines, "RK_GD_LAND_D", g("land_d_m", 0.0), 9)
    append_param(lines, "RK_GD_SIM_GT", g("sim_groundtruth_fallback", 0), 6)
    append_param(lines, "RK_GD_ASC_MODE", guidance_mode_value(ASCENT_MODES, g("ascent_mode", None), "launch_ascent"), 6)
    append_param(lines, "RK_GD_APX_MODE", guidance_mode_value(APOGEE_MODES, g("apogee_mode", None), "track"), 6)
    append_param(lines, "RK_GD_LAND_MODE", guidance_mode_value(LANDING_MODES, g("landing_mode", None), "approach"), 6)
    append_param(lines, "RK_GD_WP1_MODE", guidance_mode_value(WP_MODES, g("wp1_mode", None), "fly_through"), 6)
    append_param(lines, "RK_GD_WP2_MODE", guidance_mode_value(WP_MODES, g("wp2_mode", None), "fly_through"), 6)
    append_param(lines, "RK_GD_WP3_MODE", guidance_mode_value(WP_MODES, g("wp3_mode", None), "fly_through"), 6)
    append_param(lines, "RK_GD_WP1_HOLD_S", g("wp1_hold_s", 0.0), 9)
    append_param(lines, "RK_GD_WP2_HOLD_S", g("wp2_hold_s", 0.0), 9)
    append_param(lines, "RK_GD_WP3_HOLD_S", g("wp3_hold_s", 0.0), 9)
    append_param(lines, "RK_GD_WP1_ACC_M", g("wp1_acceptance_m", 0.0), 9)
    append_param(lines, "RK_GD_WP2_ACC_M", g("wp2_acceptance_m", 0.0), 9)
    append_param(lines, "RK_GD_WP3_ACC_M", g("wp3_acceptance_m", 0.0), 9)
    append_param(lines, "RK_GD_WP1_C_MS", g("wp1_cruise_m_s", 0.0), 9)
    append_param(lines, "RK_GD_WP2_C_MS", g("wp2_cruise_m_s", 0.0), 9)
    append_param(lines, "RK_GD_WP3_C_MS", g("wp3_cruise_m_s", 0.0), 9)
    append_param(lines, "RK_ATT_TILT_GAIN", g("tilt_gain", 0.12), 9)
    append_param(lines, "RK_ATT_TILT_MAX", g("tilt_max_deg", 20.0), 9)
    append_param(lines, "RK_ATT_ROLL_DEG", g("roll_prog_deg", 0.0), 9)
    append_param(lines, "RK_ATT_ROLL_T0", g("roll_prog_start_s", 0.0), 9)
    append_param(lines, "RK_ATT_ROLL_DT", g("roll_prog_dur_s", 0.0), 9)

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def write_motor_catalog(vehicle: dict, path: Path) -> None:
    motor_selection = vehicle.get("motor_selection", {})
    catalog_source = motor_selection.get("catalog_source")
    if catalog_source:
        from tools.generate_motor_catalog import DEFAULT_SOURCE, generate_catalog

        source_root = Path(catalog_source)
        if not source_root.is_absolute():
            source_root = REPO_ROOT / source_root
        if source_root.exists():
            generate_catalog(source_root, path)
            return

    write_preliminary_motor_catalog(vehicle, path)


def write_preliminary_motor_catalog(vehicle: dict, path: Path) -> None:
    body = vehicle["vehicle"]
    state_machine = vehicle["state_machine"]
    engines = vehicle_engines(vehicle)
    burn_duration_s = state_machine.get("maximum_burn_ms", 1000) / 1000.0
    reference_thrust_n = body["ca_reference_thrust_n"] / len(engines)
    total_impulse_ns = reference_thrust_n * burn_duration_s
    loaded_mass_kg = body["motor_loaded_mass_kg"]
    dry_mass_kg = body["motor_dry_mass_kg"]
    catalog_rows = ["motor_index,motor_id,manufacturer,designation,active,curve_file,specs_file,errors"]

    path.mkdir(parents=True, exist_ok=True)
    specs = (
        "loaded_mass_kg,dry_mass_kg,diameter_m,length_m,total_impulse_ns,burn_duration_s\n"
        f"{loaded_mass_kg},{dry_mass_kg},0.029,0.2,{total_impulse_ns},{burn_duration_s}\n"
    )
    curve = (
        "time_s,thrust_n,motor_mass_kg,burn_fraction,cumulative_impulse_ns\n"
        f"0.0,0.0,{loaded_mass_kg},0.0,0.0\n"
        f"{burn_duration_s / 2.0},{reference_thrust_n},{(loaded_mass_kg + dry_mass_kg) / 2.0},0.5,{total_impulse_ns / 2.0}\n"
        f"{burn_duration_s},0.0,{dry_mass_kg},1.0,{total_impulse_ns}\n"
    )

    for index, engine in enumerate(engines):
        motor_id = f"preliminary-{vehicle['name']}-{engine['id']}"
        motor_path = path / motor_id
        motor_path.mkdir(parents=True, exist_ok=True)
        catalog_rows.append(
            f"{engine['motor_index']},{motor_id},TV3,Preliminary,1,{motor_id}/curve.csv,{motor_id}/specs.csv,"
            "preliminary SITL placeholder; replace with build/motors generated from measured data"
        )
        (motor_path / "specs.csv").write_text(specs)
        (motor_path / "curve.csv").write_text(curve)

    (path / "catalog.csv").write_text("\n".join(catalog_rows) + "\n")


def logger_topics_text() -> str:
    lines = [
        "# TV3 ULog review profile.",
        "# Format: <uORB topic> <minimum interval ms> [instance]",
        "# PX4 uses this file instead of the stock logger profile when it exists.",
    ]

    for topic, interval_ms in LOGGER_TOPICS:
        if interval_ms is None:
            lines.append("")
            lines.append(topic)
        else:
            lines.append(f"{topic} {interval_ms}")

    return "\n".join(lines).rstrip() + "\n"


def write_logger_topics(path: Path) -> None:
    topic_text = logger_topics_text()
    for logging_path in (path / "etc" / "logging", path / "fs" / "microsd" / "etc" / "logging"):
        logging_path.mkdir(parents=True, exist_ok=True)
        (logging_path / "logger_topics.txt").write_text(topic_text)


def write_active_flight_profile(vehicle: dict, path: Path) -> None:
    active_profile = vehicle.get("_active_flight_profile")
    if not active_profile:
        return

    profile_name = active_profile["name"]
    profile_text = json.dumps(active_profile["data"], indent=2, ensure_ascii=False) + "\n"
    for profile_path in (path / "etc" / "flight_profiles", path / "fs" / "microsd" / "tv3" / "flight_profiles"):
        profile_path.mkdir(parents=True, exist_ok=True)
        (profile_path / "active.json").write_text(profile_text)
        (profile_path / f"{profile_name}.json").write_text(profile_text)


def write_runtime_assets(vehicle: dict, path: Path) -> None:
    etc_path = path / "etc"
    airframe_path = path / "fs" / "microsd" / "tv3" / "airframes"
    motor_path = path / "fs" / "microsd" / "tv3" / "motors"
    etc_path.mkdir(parents=True, exist_ok=True)

    config_template = (REPO_ROOT / "runtime" / "nuttx" / "etc" / "config.txt").read_text()
    config_text = config_template.replace("tv3_v1.params", f"{vehicle['name']}.params")
    (etc_path / "config.txt").write_text(config_text)

    extras_text = (REPO_ROOT / "runtime" / "nuttx" / "etc" / "extras.txt").read_text()
    if vehicle.get("guidance", {}).get("enable", 0) and "tv3_guidance start" not in extras_text:
        extras_text = extras_text.rstrip() + "\ntv3_guidance start\n"
    (etc_path / "extras.txt").write_text(extras_text)

    write_px4_params(vehicle, airframe_path / f"{vehicle['name']}.params")
    write_motor_catalog(vehicle, motor_path)
    write_logger_topics(path)
    write_active_flight_profile(vehicle, path)


def generate_assets(vehicle_path: Path, output_root: Path, flight_profile_path: Path | None = None) -> None:
    vehicle = load_vehicle(vehicle_path)
    if flight_profile_path is not None:
        vehicle = apply_flight_profile(vehicle, resolve_flight_profile(flight_profile_path), flight_profile_path)
    validate_vehicle(vehicle, vehicle_path)
    write_runtime_assets(vehicle, output_root / "runtime")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, required=True, help="Path to the shared vehicle definition")
    parser.add_argument("--flight-profile", type=Path, help="Optional scenario profile to overlay on the vehicle")
    parser.add_argument("--output", type=Path, required=True, help="Output directory for generated assets")
    args = parser.parse_args()

    generate_assets(args.vehicle, args.output, args.flight_profile)
    profile_note = f" with profile {args.flight_profile}" if args.flight_profile else ""
    print(f"generated assets for {args.vehicle}{profile_note} into {args.output}")


if __name__ == "__main__":
    main()
