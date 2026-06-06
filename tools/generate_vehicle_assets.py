#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import shutil
from copy import deepcopy
from pathlib import Path
from xml.sax.saxutils import escape

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
MAX_ENGINES = 4
COM_MARKER_RADIUS_M = 0.04
BODY_CONTACT_COLLISION_RADIUS_M = 0.12
RENDERABLE_MESH_SUFFIXES = {".dae", ".glb", ".gltf", ".obj", ".stl"}
VISUAL_MESH_POLICIES = {"fallback", "omit_unavailable", "require_renderable"}
ORIENTATION_AXIS_COLORS = {
    "x": [1.0, 0.05, 0.05, 0.85],
    "y": [0.05, 0.75, 0.12, 0.85],
    "z": [0.05, 0.25, 1.0, 0.85],
}
ORIENTATION_AXIS_ROTATIONS = {
    "x": [0.0, 1.57079632679, 0.0],
    "y": [-1.57079632679, 0.0, 0.0],
    "z": [0.0, 0.0, 0.0],
}
ORIENTATION_AXIS_VECTORS = {
    "x": [1.0, 0.0, 0.0],
    "y": [0.0, 1.0, 0.0],
    "z": [0.0, 0.0, 1.0],
}
JOINT_MARKER_COLOR = [1.0, 0.0, 0.85, 0.9]
ENGINE_PIVOT_MARKER_COLOR = [0.1, 1.0, 0.95, 0.9]
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
}
LOGGER_TOPICS = [
    ("# Core PX4 state", None),
    ("vehicle_status", 100),
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
    ("# TV3 rocket-specific review topics", None),
    ("rocket_command", 0),
    ("rocket_engine_command", 20),
    ("rocket_engine_state", 20),
    ("rocket_guidance_status", 20),
    ("rocket_load_cell", 50),
    ("rocket_mode_status", 20),
    ("rocket_motor_reference", 20),
    ("rocket_status", 20),
    ("rocket_thrust", 20),
]


def load_vehicle(path: Path) -> dict:
    with path.open() as stream:
        return yaml.safe_load(stream)


def load_flight_profile(path: Path) -> dict:
    with path.open() as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict):
        raise ValueError(f"flight profile must be a YAML mapping: {path}")
    return profile


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
            "pitch_axis": [0.0, -1.0, 0.0],
            "yaw_axis": [0.0, 0.0, -1.0],
            "thrust_fraction": 1.0,
            "gimbal": {
                "pitch_max_deg": body["tvc_max_deg"],
                "yaw_max_deg": body["tvc_max_deg"],
                "splay_max_deg": body["tvc_max_deg"],
                "slew_dps": body["tvc_slew_dps"],
                "pitch_trim": 0.0,
                "yaw_trim": 0.0,
            },
        }
    ]


def ignition_sequence(vehicle: dict, engines: list[dict]) -> list[int]:
    ignition = vehicle.get("propulsion", {}).get("ignition", {})
    sequence = ignition.get("sequence", list(range(len(engines))))
    return [int(value) for value in sequence]


def validate_vehicle(vehicle: dict) -> None:
    for section in ("name", "hardware", "vehicle", "controller", "state_machine", "motor_selection", "guidance"):
        if section not in vehicle:
            raise ValueError(f"vehicle manifest missing required section: {section}")

    engines = vehicle_engines(vehicle)
    if not engines:
        raise ValueError("vehicle manifest must define at least one engine")
    if len(engines) > MAX_ENGINES:
        raise ValueError(f"vehicle manifest supports at most {MAX_ENGINES} engines")

    seen_ids = set()
    for index, engine in enumerate(engines):
        for key in ("id", "motor_index", "position_m", "thrust_axis", "pitch_axis", "yaw_axis", "thrust_fraction", "gimbal"):
            if key not in engine:
                raise ValueError(f"engine {index} missing required field: {key}")
        if engine["id"] in seen_ids:
            raise ValueError(f"duplicate engine id: {engine['id']}")
        seen_ids.add(engine["id"])
        for key in ("position_m", "thrust_axis", "pitch_axis", "yaw_axis"):
            if len(engine[key]) != 3:
                raise ValueError(f"{engine['id']} {key} must have exactly 3 values")

    sequence = ignition_sequence(vehicle, engines)
    if sorted(sequence) != list(range(len(engines))):
        raise ValueError("ignition sequence must contain each engine index exactly once")


def write_px4_params(vehicle: dict, path: Path) -> None:
    controller = vehicle["controller"]
    state_machine = vehicle["state_machine"]
    hardware = vehicle["hardware"]
    body = vehicle["vehicle"]
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
    append_param(lines, "CA_RK_REF_THR", body["ca_reference_thrust_n"], 9)
    append_param(lines, "CA_RK_MIN_THR", body["ca_minimum_thrust_n"], 9)
    append_param(lines, "CA_RK_FAL_THR", body["ca_fallback_thrust_n"], 9)
    append_param(lines, "CA_RK_BODY_M", body["body_mass_kg"], 9)
    append_param(lines, "CA_RK_BODY_CMX", body["body_com_x_m"], 9)
    append_param(lines, "CA_RK_MOT_WET", body["motor_loaded_mass_kg"], 9)
    append_param(lines, "CA_RK_MOT_DRY", body["motor_dry_mass_kg"], 9)
    append_param(lines, "CA_RK_MOT_CMX", body["motor_com_x_m"], 9)

    for index, engine in enumerate(engines):
        gimbal = engine["gimbal"]
        position = engine["position_m"]
        axis = engine["thrust_axis"]
        pitch_axis = engine["pitch_axis"]
        yaw_axis = engine["yaw_axis"]
        append_param(lines, f"CA_RK_G{index}_PX", position[0], 9)
        append_param(lines, f"CA_RK_G{index}_PY", position[1], 9)
        append_param(lines, f"CA_RK_G{index}_PZ", position[2], 9)
        append_param(lines, f"CA_RK_G{index}_AX", axis[0], 9)
        append_param(lines, f"CA_RK_G{index}_AY", axis[1], 9)
        append_param(lines, f"CA_RK_G{index}_AZ", axis[2], 9)
        append_param(lines, f"CA_RK_G{index}_PAX", pitch_axis[0], 9)
        append_param(lines, f"CA_RK_G{index}_PAY", pitch_axis[1], 9)
        append_param(lines, f"CA_RK_G{index}_PAZ", pitch_axis[2], 9)
        append_param(lines, f"CA_RK_G{index}_YAX", yaw_axis[0], 9)
        append_param(lines, f"CA_RK_G{index}_YAY", yaw_axis[1], 9)
        append_param(lines, f"CA_RK_G{index}_YAZ", yaw_axis[2], 9)
        append_param(lines, f"CA_RK_G{index}_PMAX", gimbal.get("pitch_max_deg", body["tvc_max_deg"]), 9)
        append_param(lines, f"CA_RK_G{index}_YMAX", gimbal.get("yaw_max_deg", body["tvc_max_deg"]), 9)
        append_param(lines, f"CA_RK_G{index}_TF", engine.get("thrust_fraction", 1.0 / len(engines)), 9)
        append_param(lines, f"CA_RK_G{index}_PTR", gimbal.get("pitch_trim", 0.0), 9)
        append_param(lines, f"CA_RK_G{index}_YTR", gimbal.get("yaw_trim", 0.0), 9)

    append_param(lines, "RK_ENABLE", 1, 6)
    append_param(lines, "RK_CMD_SRC", state_machine.get("command_source", 1), 6)
    append_param(lines, "RK_MOT_IDX", motor["index"], 6)
    append_param(lines, "RK_ENG_COUNT", len(engines), 6)
    append_param(lines, "RK_IGN_DWELL_MS", ignition.get("dwell_ms", 0), 6)
    append_param(lines, "RK_SPLAY_MAX_DEG", throttle.get("max_splay_deg", body["tvc_max_deg"]), 9)
    for index in range(MAX_ENGINES):
        engine = engines[index] if index < len(engines) else None
        sequence_value = sequence[index] if index < len(sequence) else index
        append_param(lines, f"RK_IGN_IDX{index}", sequence_value, 6)
        append_param(lines, f"RK_ENG{index}_MOT", engine.get("motor_index", motor["index"]) if engine else motor["index"], 6)

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
    append_param(lines, "RK_TVC_MAX_DEG", body["tvc_max_deg"], 9)
    append_param(lines, "RK_TVC_SLEW_DPS", body["tvc_slew_dps"], 9)
    append_param(lines, "RK_TQ_R_MAX", body["torque_limits_nm"].get("roll", 0.0), 9)
    append_param(lines, "RK_TQ_P_MAX", body["torque_limits_nm"]["pitch"], 9)
    append_param(lines, "RK_TQ_Y_MAX", body["torque_limits_nm"]["yaw"], 9)
    append_param(lines, "RK_ATT_P_RAIL", controller["attitude_p"]["rail"], 9)
    append_param(lines, "RK_ATT_P_FREE", controller["attitude_p"]["free"], 9)
    append_param(lines, "RK_RATE_P_RAIL", controller["rate_p"]["rail"], 9)
    append_param(lines, "RK_RATE_P_FREE", controller["rate_p"]["free"], 9)
    append_param(lines, "RK_RATE_I", controller["rate_i"], 9)
    append_param(lines, "RK_RATE_D", controller["rate_d"], 9)
    append_param(lines, "RK_LC_SRC", load_cell.get("source", 0), 6)
    append_param(lines, "RK_LC_CH", load_cell["adc_channel"], 6)
    append_param(lines, "RK_LC_TARE", load_cell["calibration"]["tare"], 9)
    append_param(lines, "RK_LC_SCALE", load_cell["calibration"]["scale"], 9)
    append_param(lines, "RK_LC_ALPHA", load_cell.get("alpha", 0.25), 9)
    append_param(lines, "RK_LC_TO_MS", load_cell.get("timeout_ms", 200), 6)
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

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def as_pose(values: list[float] | None) -> str:
    pose = list(values or [0.0, 0.0, 0.0])
    if len(pose) == 3:
        pose.extend([0.0, 0.0, 0.0])
    if len(pose) != 6:
        raise ValueError("poses must have either 3 or 6 entries")
    return " ".join(str(value) for value in pose)


def as_vector(values: list[float]) -> str:
    if len(values) != 3:
        raise ValueError("vectors must have exactly 3 entries")
    return " ".join(str(value) for value in values)


def inertia_xml(inertia: dict, indent: str = "        ") -> str:
    terms = {
        "ixx": inertia["ixx"],
        "iyy": inertia["iyy"],
        "izz": inertia["izz"],
        "ixy": inertia.get("ixy", 0.0),
        "ixz": inertia.get("ixz", 0.0),
        "iyz": inertia.get("iyz", 0.0),
    }
    return "\n".join(f"{indent}<{key}>{value}</{key}>" for key, value in terms.items())


def gazebo_link_name(link_id: str) -> str:
    return "base_link" if link_id == "body" else link_id


def gazebo_base_sensors_xml(link_id: str) -> str:
    if link_id != "body":
        return ""

    return """
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure>
          <pressure>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>0.01</stddev>
            </noise>
          </pressure>
        </air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <magnetometer>
          <x>
            <noise type="gaussian">
              <stddev>0.0001</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <stddev>0.0001</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <stddev>0.0001</stddev>
            </noise>
          </z>
        </magnetometer>
      </sensor>
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00018665</stddev>
                <dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.00186</stddev>
                <dynamic_bias_stddev>0.006</dynamic_bias_stddev>
                <dynamic_bias_correlation_time>300</dynamic_bias_correlation_time>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>"""


def gazebo_com_visual_xml(pose: str) -> str:
    return f"""
      <visual name="center_of_mass_marker">
        <pose>{pose}</pose>
        <geometry>
          <sphere>
            <radius>{COM_MARKER_RADIUS_M}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1.0 0.15 0.05 1.0</ambient>
          <diffuse>1.0 0.15 0.05 1.0</diffuse>
          <specular>0.2 0.2 0.2 1.0</specular>
        </material>
      </visual>"""


def offset_pose(origin: list[float], axis: list[float], offset_m: float, rotation_rpy: list[float]) -> str:
    position = [float(origin[i]) + float(axis[i]) * offset_m for i in range(3)]
    return as_pose([*position, *rotation_rpy])


def normalized_vector(values: list[float]) -> list[float]:
    vector = [float(value) for value in values[:3]]
    if len(vector) != 3:
        raise ValueError("vectors must have exactly three entries")
    magnitude = math.sqrt(sum(value * value for value in vector))
    if magnitude <= 1e-12:
        raise ValueError("vectors must not be zero length")
    return [value / magnitude for value in vector]


def cylinder_rotation_from_z_axis(axis: list[float]) -> list[float]:
    normalized = normalized_vector(axis)
    dominant = max(range(3), key=lambda index: abs(normalized[index]))
    sign = 1.0 if normalized[dominant] >= 0.0 else -1.0
    if dominant == 0:
        return [0.0, sign * 1.57079632679, 0.0]
    if dominant == 1:
        return [-sign * 1.57079632679, 0.0, 0.0]
    if sign < 0.0:
        return [3.14159265359, 0.0, 0.0]
    return [0.0, 0.0, 0.0]


def visual_safe_name(value: str) -> str:
    return "".join(character if character.isalnum() or character == "_" else "_" for character in value)


def gazebo_orientation_axes_xml(vehicle: dict, link_id: str, com_m: list[float] | None) -> str:
    if link_id != "body":
        return ""

    visual_model = vehicle.get("gazebo", {}).get("visual_model", {})
    axes_config = visual_model.get("orientation_axes", {})
    if not axes_config.get("enabled", False):
        return ""

    origin = list(com_m or [vehicle["vehicle"].get("body_com_x_m", 0.0), 0.0, 0.0])[:3]
    if len(origin) != 3:
        raise ValueError("body COM must have three entries for orientation axes")

    length_m = float(axes_config.get("length_m", 0.45))
    head_length_m = float(axes_config.get("head_length_m", 0.09))
    shaft_radius_m = float(axes_config.get("shaft_radius_m", 0.008))
    head_radius_m = float(axes_config.get("head_radius_m", 0.026))
    shaft_length_m = max(length_m - head_length_m, 0.01)

    rendered = []
    for axis_name in ("x", "y", "z"):
        axis = ORIENTATION_AXIS_VECTORS[axis_name]
        rotation = ORIENTATION_AXIS_ROTATIONS[axis_name]
        color = axes_config.get(f"{axis_name}_color_rgba", ORIENTATION_AXIS_COLORS[axis_name])
        material = gazebo_material_xml(color)
        shaft_pose = offset_pose(origin, axis, shaft_length_m * 0.5, rotation)
        head_pose = offset_pose(origin, axis, shaft_length_m + head_length_m * 0.5, rotation)
        rendered.append(
            f"""      <visual name="orientation_axis_{axis_name}_shaft">
        <pose>{shaft_pose}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <cylinder>
            <radius>{shaft_radius_m}</radius>
            <length>{shaft_length_m}</length>
          </cylinder>
        </geometry>{material}
      </visual>
      <visual name="orientation_axis_{axis_name}_head">
        <pose>{head_pose}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <cone>
            <radius>{head_radius_m}</radius>
            <length>{head_length_m}</length>
          </cone>
        </geometry>{material}
      </visual>"""
        )

    return "\n" + "\n".join(rendered)


def gazebo_marker_sphere_visual_xml(name: str, origin: list[float], radius_m: float, color: list[float]) -> str:
    return f"""      <visual name="{escape(name)}">
        <pose>{as_pose(origin)}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <sphere>
            <radius>{radius_m}</radius>
          </sphere>
        </geometry>{gazebo_material_xml(color)}
      </visual>"""


def gazebo_marker_axis_visual_xml(name: str, origin: list[float], axis: list[float], length_m: float, radius_m: float, color: list[float]) -> str:
    normalized = normalized_vector(axis)
    pose = offset_pose(origin, normalized, length_m * 0.5, cylinder_rotation_from_z_axis(normalized))
    return f"""      <visual name="{escape(name)}">
        <pose>{pose}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <cylinder>
            <radius>{radius_m}</radius>
            <length>{length_m}</length>
          </cylinder>
        </geometry>{gazebo_material_xml(color)}
      </visual>"""


def gazebo_joint_markers_xml(vehicle: dict, link_id: str) -> str:
    if link_id != "body":
        return ""

    visual_model = vehicle.get("gazebo", {}).get("visual_model", {})
    marker_config = visual_model.get("joint_markers", {})
    if not marker_config.get("enabled", False):
        return ""

    radius_m = float(marker_config.get("radius_m", 0.022))
    axis_length_m = float(marker_config.get("axis_length_m", 0.18))
    axis_radius_m = float(marker_config.get("axis_radius_m", 0.006))
    rendered = []

    if marker_config.get("physical_joints", True):
        for joint in vehicle.get("physical_model", {}).get("joints", []):
            origin = list(joint.get("origin_m", [0.0, 0.0, 0.0]))[:3]
            axis = list(joint.get("axis", [0.0, 0.0, 1.0]))[:3]
            joint_id = visual_safe_name(str(joint["id"]))
            rendered.append(
                gazebo_marker_sphere_visual_xml(
                    f"joint_marker_{joint_id}_origin",
                    origin,
                    radius_m,
                    marker_config.get("joint_color_rgba", JOINT_MARKER_COLOR),
                )
            )
            rendered.append(
                gazebo_marker_axis_visual_xml(
                    f"joint_axis_{joint_id}",
                    origin,
                    axis,
                    axis_length_m,
                    axis_radius_m,
                    marker_config.get("joint_axis_color_rgba", JOINT_MARKER_COLOR),
                )
            )

    if marker_config.get("engine_pivots", True):
        gazebo = vehicle.get("gazebo", {})
        thrust_axis_override = gazebo.get("thrust_axis_world") if gazebo.get("thrust_axis_frame") == "world" else None
        for index, engine in enumerate(vehicle_engines(vehicle)):
            if "gimbal" not in engine or "position_m" not in engine:
                continue
            origin = list(engine["position_m"])[:3]
            axis = list(thrust_axis_override or engine.get("thrust_axis", [1.0, 0.0, 0.0]))[:3]
            marker_index = int(engine.get("motor_index", index))
            rendered.append(
                gazebo_marker_sphere_visual_xml(
                    f"engine_pivot_marker_{marker_index}",
                    origin,
                    radius_m,
                    marker_config.get("engine_pivot_color_rgba", ENGINE_PIVOT_MARKER_COLOR),
                )
            )
            rendered.append(
                gazebo_marker_axis_visual_xml(
                    f"engine_pivot_thrust_axis_{marker_index}",
                    origin,
                    axis,
                    axis_length_m,
                    axis_radius_m,
                    marker_config.get("engine_pivot_axis_color_rgba", ENGINE_PIVOT_MARKER_COLOR),
                )
            )

    if not rendered:
        return ""
    return "\n" + "\n".join(rendered)


def gazebo_body_contact_collision_xml(link_id: str, pose: str) -> str:
    if link_id != "body":
        return ""

    return f"""
      <collision name="body_contact_collision">
        <pose>{pose}</pose>
        <geometry>
          <sphere>
            <radius>{BODY_CONTACT_COLLISION_RADIUS_M}</radius>
          </sphere>
        </geometry>
      </collision>"""


def gazebo_material_xml(color: list[float] | None) -> str:
    rgba = list(color or [0.82, 0.84, 0.84, 1.0])
    if len(rgba) != 4:
        raise ValueError("visual colors must have exactly 4 RGBA entries")
    color_text = " ".join(str(value) for value in rgba)
    transparency = max(0.0, min(1.0, 1.0 - float(rgba[3])))
    return f"""
        <material>
          <ambient>{color_text}</ambient>
          <diffuse>{color_text}</diffuse>
          <specular>0.2 0.2 0.2 1.0</specular>
        </material>
        <transparency>{transparency}</transparency>"""


def gazebo_visual_geometry_xml(visual: dict, path: Path, mesh_policy: str = "fallback") -> tuple[str | None, str]:
    if mesh_policy not in VISUAL_MESH_POLICIES:
        raise ValueError(
            f"unsupported Gazebo visual mesh_policy: {mesh_policy}; "
            f"expected one of {sorted(VISUAL_MESH_POLICIES)}"
        )

    visual_name = str(visual.get("name", "unnamed"))
    mesh_uri = visual.get("mesh_uri")
    kind = visual.get("kind", visual.get("fallback_kind", "box"))
    if mesh_uri:
        mesh_source = resolve_repo_path(mesh_uri)
        if mesh_source.exists() and mesh_source.suffix.lower() in RENDERABLE_MESH_SUFFIXES:
            meshes_path = path / "meshes"
            meshes_path.mkdir(parents=True, exist_ok=True)
            mesh_destination = meshes_path / mesh_source.name
            shutil.copy2(mesh_source, mesh_destination)
            scale = as_vector(visual.get("mesh_scale", [1.0, 1.0, 1.0]))
            return (
                f"""
          <mesh>
            <uri>meshes/{escape(mesh_destination.name)}</uri>
            <scale>{scale}</scale>
          </mesh>""",
                "",
            )

        if mesh_policy == "require_renderable":
            raise FileNotFoundError(
                f"renderable Gazebo mesh unavailable for visual {visual_name}: "
                f"{repo_display_path(mesh_source)}"
            )
        if mesh_policy == "omit_unavailable" or kind == "mesh":
            return (
                None,
                f"<!-- CAD mesh unavailable for {escape(str(mesh_uri))}; visual {escape(visual_name)} omitted. -->",
            )

        comment = f"<!-- Renderable mesh unavailable for {escape(str(mesh_uri))}; using procedural fallback. -->"
    elif kind == "mesh":
        if mesh_policy == "require_renderable":
            raise ValueError(f"Gazebo mesh visual {visual_name} requires mesh_uri")
        return (
            None,
            f"<!-- CAD mesh visual {escape(visual_name)} requires mesh_uri; visual omitted. -->",
        )
    else:
        comment = ""

    if kind == "box":
        size = as_vector(visual["size_m"])
        return (
            f"""
          <box>
            <size>{size}</size>
          </box>""",
            comment,
        )
    if kind == "cylinder":
        return (
            f"""
          <cylinder>
            <radius>{visual["radius_m"]}</radius>
            <length>{visual["length_m"]}</length>
          </cylinder>""",
            comment,
        )
    if kind == "cone":
        return (
            f"""
          <cone>
            <radius>{visual["radius_m"]}</radius>
            <length>{visual["length_m"]}</length>
          </cone>""",
            comment,
        )
    if kind == "sphere":
        return (
            f"""
          <sphere>
            <radius>{visual["radius_m"]}</radius>
          </sphere>""",
            comment,
        )

    raise ValueError(f"unsupported Gazebo visual geometry kind: {kind}")


def gazebo_visual_model_xml(vehicle: dict, path: Path) -> str:
    visual_model = vehicle.get("gazebo", {}).get("visual_model", {})
    visuals = visual_model.get("visuals", [])
    if not visuals:
        return ""

    mesh_policy = visual_model.get("mesh_policy", "fallback")
    rendered = []
    for visual in visuals:
        name = escape(visual["name"])
        pose = as_pose(visual.get("pose_m"))
        geometry, comment = gazebo_visual_geometry_xml(visual, path, mesh_policy)
        if geometry is None:
            if comment:
                rendered.append(f"      {comment}")
            continue
        material = gazebo_material_xml(visual.get("color_rgba"))
        cast_shadows = str(visual.get("cast_shadows", True)).lower()
        comment_text = f"\n      {comment}" if comment else ""
        rendered.append(
            f"""      <visual name="{name}">
        <pose>{pose}</pose>
        <cast_shadows>{cast_shadows}</cast_shadows>
        <geometry>{geometry}
        </geometry>{material}{comment_text}
      </visual>"""
        )

    return "\n" + "\n".join(rendered)


def resolve_repo_path(raw_path: str) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path
    return REPO_ROOT / path


def copy_cad_sources(physical_model: dict, path: Path) -> list[dict[str, str]]:
    copied_sources = []
    cad_path = path / "cad"
    for link in physical_model.get("links", []):
        cad = link.get("cad", {})
        source_step = cad.get("source_step")
        if not source_step:
            continue

        source = resolve_repo_path(source_step)
        if not source.exists():
            raise FileNotFoundError(f"missing CAD source for {link['id']}: {source}")

        cad_path.mkdir(parents=True, exist_ok=True)
        destination = cad_path / source.name
        shutil.copy2(source, destination)
        copied_sources.append(
            {
                "link": link["id"],
                "source_step": str(source.relative_to(REPO_ROOT) if source.is_relative_to(REPO_ROOT) else source),
                "generated_path": str(destination.relative_to(path)),
            }
        )

    if copied_sources:
        (path / "cad_sources.yaml").write_text(yaml.safe_dump({"cad_sources": copied_sources}, sort_keys=False))

    return copied_sources


def gazebo_link_xml(link: dict, cad_sources: dict[str, str], vehicle: dict, path: Path) -> str:
    name = escape(gazebo_link_name(link["id"]))
    mass = link["mass_kg"]
    pose = as_pose(link.get("pose_m"))
    com_pose = as_pose(link.get("com_m"))
    inertia = inertia_xml(link["inertia_kg_m2"])
    cad_comment = ""
    if link["id"] in cad_sources:
        cad_comment = f"\n      <!-- CAD source reference: {escape(cad_sources[link['id']])}. Convert STEP to STL/DAE for Gazebo visuals. -->"
    sensors = gazebo_base_sensors_xml(link["id"])
    visual_model = gazebo_visual_model_xml(vehicle, path) if link["id"] == "body" else ""
    orientation_axes = gazebo_orientation_axes_xml(vehicle, link["id"], link.get("com_m"))
    joint_markers = gazebo_joint_markers_xml(vehicle, link["id"])

    return f"""    <link name="{name}">
      <pose>{pose}</pose>
      <inertial>
        <pose>{com_pose}</pose>
        <mass>{mass}</mass>
        <inertia>
{inertia}
        </inertia>
      </inertial>{gazebo_com_visual_xml(com_pose)}{orientation_axes}{joint_markers}{visual_model}{gazebo_body_contact_collision_xml(link["id"], com_pose)}{cad_comment}{sensors}
    </link>"""


def gazebo_joint_xml(joint: dict) -> str:
    name = escape(joint["id"])
    joint_type = escape(joint["type"])
    if joint_type == "fixed":
        return f"""    <joint name="{name}" type="{joint_type}">
      <pose>{as_pose(joint.get("origin_m"))}</pose>
      <parent>{escape(gazebo_link_name(joint["parent"]))}</parent>
      <child>{escape(gazebo_link_name(joint["child"]))}</child>
    </joint>"""

    lower, upper = joint.get("limit_rad", [0.0, 0.0])
    return f"""    <joint name="{name}" type="{joint_type}">
      <pose>{as_pose(joint.get("origin_m"))}</pose>
      <parent>{escape(gazebo_link_name(joint["parent"]))}</parent>
      <child>{escape(gazebo_link_name(joint["child"]))}</child>
      <axis>
        <xyz>{as_vector(joint["axis"])}</xyz>
        <limit>
          <lower>{lower}</lower>
          <upper>{upper}</upper>
          <effort>{joint.get("effort_nm", 0.0)}</effort>
          <velocity>{joint.get("velocity_rad_s", 0.0)}</velocity>
        </limit>
        <dynamics>
          <damping>{joint.get("damping", 0.0)}</damping>
          <friction>{joint.get("friction", 0.0)}</friction>
        </dynamics>
      </axis>
    </joint>"""


def gazebo_physical_model_xml(vehicle: dict, path: Path) -> str:
    physical_model = vehicle["physical_model"]
    copied_sources = copy_cad_sources(physical_model, path)
    cad_sources = {source["link"]: source["generated_path"] for source in copied_sources}
    links = "\n".join(gazebo_link_xml(link, cad_sources, vehicle, path) for link in physical_model.get("links", []))
    joints = "\n".join(gazebo_joint_xml(joint) for joint in physical_model.get("joints", []))
    return "\n".join(part for part in (links, joints) if part)


def gazebo_plugin_xml(vehicle: dict) -> str:
    gazebo = vehicle.get("gazebo", {})
    if not gazebo.get("include_tv3_plugin", False):
        return ""

    engines = vehicle_engines(vehicle)
    ignition = vehicle.get("propulsion", {}).get("ignition", {})
    throttle = vehicle.get("propulsion", {}).get("throttle", {})
    thrust_axis_override = gazebo.get("thrust_axis_world")
    engine_xml = []
    for index, engine in enumerate(engines):
        position = as_vector(engine["position_m"])
        thrust_axis = as_vector(thrust_axis_override or engine["thrust_axis"])
        pitch_axis = as_vector(engine["pitch_axis"])
        yaw_axis = as_vector(engine["yaw_axis"])
        gimbal = engine["gimbal"]
        engine_xml.append(
            f"""      <engine index="{index}" id="{escape(engine['id'])}">
        <index>{index}</index>
        <id>{escape(engine['id'])}</id>
        <motor_index>{engine['motor_index']}</motor_index>
        <load_cell_channel>{engine.get('load_cell_channel', index)}</load_cell_channel>
        <position_m>{position}</position_m>
        <thrust_axis>{thrust_axis}</thrust_axis>
        <pitch_axis>{pitch_axis}</pitch_axis>
        <yaw_axis>{yaw_axis}</yaw_axis>
        <thrust_fraction>{engine.get('thrust_fraction', 1.0 / len(engines))}</thrust_fraction>
        <pitch_max_deg>{gimbal.get('pitch_max_deg', vehicle['vehicle']['tvc_max_deg'])}</pitch_max_deg>
        <yaw_max_deg>{gimbal.get('yaw_max_deg', vehicle['vehicle']['tvc_max_deg'])}</yaw_max_deg>
        <splay_max_deg>{gimbal.get('splay_max_deg', throttle.get('max_splay_deg', vehicle['vehicle']['tvc_max_deg']))}</splay_max_deg>
        <slew_dps>{gimbal.get('slew_dps', vehicle['vehicle']['tvc_slew_dps'])}</slew_dps>
      </engine>"""
        )

    sequence = " ".join(str(value) for value in ignition_sequence(vehicle, engines))
    engines_block = "\n".join(engine_xml)
    burn_duration_s = vehicle["state_machine"].get("maximum_burn_ms", 0) / 1000.0
    reference_thrust_n = gazebo.get("reference_thrust_n", vehicle["vehicle"]["ca_reference_thrust_n"])
    apply_engine_torques = str(gazebo.get("apply_engine_torques", True)).lower()
    commanded_thrust = str(gazebo.get("commanded_thrust", True)).lower()
    command_timeout_s = gazebo.get("command_timeout_s", 0.25)
    command_scale = gazebo.get("command_scale", 1.0)
    force_application_m = as_vector(gazebo.get("force_application_m", [0.0, 0.0, 0.0]))
    return f"""
    <plugin name="tv3::Tv3RocketSystem" filename="libtv3_rocket_gz.so">
      <base_link>base_link</base_link>
      <rail_length_m>{vehicle['vehicle']['rail_length_m']}</rail_length_m>
      <reference_thrust_n>{reference_thrust_n}</reference_thrust_n>
      <thrust_axis_frame>{escape(gazebo.get('thrust_axis_frame', 'link'))}</thrust_axis_frame>
      <apply_engine_torques>{apply_engine_torques}</apply_engine_torques>
      <commanded_thrust>{commanded_thrust}</commanded_thrust>
      <command_timeout_s>{command_timeout_s}</command_timeout_s>
      <command_scale>{command_scale}</command_scale>
      <force_application_m>{force_application_m}</force_application_m>
      <ignition_delay_s>{gazebo.get('ignition_delay_s', 0.0)}</ignition_delay_s>
      <burn_duration_s>{burn_duration_s}</burn_duration_s>
      <engine_count>{len(engines)}</engine_count>
      <ignition_sequence>{sequence}</ignition_sequence>
      <ignition_confirm_threshold_n>{ignition.get('confirmation_threshold_n', vehicle['state_machine']['launch_threshold_n'])}</ignition_confirm_threshold_n>
      <ignition_dwell_ms>{ignition.get('dwell_ms', 0)}</ignition_dwell_ms>
      <throttle_mechanism>{escape(throttle.get('mechanism', 'fixed_tvc'))}</throttle_mechanism>
      <splay_max_deg>{throttle.get('max_splay_deg', vehicle['vehicle']['tvc_max_deg'])}</splay_max_deg>
{engines_block}
    </plugin>"""


def write_gazebo_assets(vehicle: dict, path: Path) -> None:
    name = vehicle["name"]
    gazebo = vehicle.get("gazebo", {})
    model_name = gazebo.get("model_name", name)
    path.mkdir(parents=True, exist_ok=True)

    model_config = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>
"""

    body_com_pose = as_pose([vehicle["vehicle"].get("body_com_x_m", 0.0), 0.0, 0.0])
    body_xml = f"""    <link name="body">
      <inertial>
        <pose>{body_com_pose}</pose>
        <mass>{vehicle['vehicle']['body_mass_kg']}</mass>
      </inertial>{gazebo_com_visual_xml(body_com_pose)}{gazebo_orientation_axes_xml(vehicle, "body", [vehicle["vehicle"].get("body_com_x_m", 0.0), 0.0, 0.0])}{gazebo_joint_markers_xml(vehicle, "body")}{gazebo_visual_model_xml(vehicle, path)}{gazebo_body_contact_collision_xml("body", body_com_pose)}
    </link>"""
    if "physical_model" in vehicle:
        body_xml = gazebo_physical_model_xml(vehicle, path)

    plugin_xml = gazebo_plugin_xml(vehicle)

    model_sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>false</static>
{body_xml}{plugin_xml}
  </model>
</sdf>
"""

    (path / "model.config").write_text(model_config)
    (path / "model.sdf").write_text(model_sdf)


def write_jsbsim_assets(vehicle: dict, path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    engines = vehicle_engines(vehicle)
    engine_lines = "\n".join(
        f"""  <engine id="{engine['id']}">
    <motor_index>{engine['motor_index']}</motor_index>
  </engine>"""
        for engine in engines
    )

    aircraft_xml = f"""<?xml version="1.0"?>
<fdm_config name="{vehicle['name']}" version="2.0">
  <metrics>
    <wingarea unit="FT2">0.01</wingarea>
    <wingspan unit="FT">0.1</wingspan>
    <chord unit="FT">0.1</chord>
  </metrics>
  <mass_balance>
    <emptywt unit="KG">{vehicle['vehicle']['body_mass_kg']}</emptywt>
    <location name="CG" unit="M">
      <x>{vehicle['vehicle']['body_com_x_m']}</x>
      <y>0.0</y>
      <z>0.0</z>
    </location>
  </mass_balance>
</fdm_config>
"""

    propulsion_xml = f"""<?xml version="1.0"?>
<propulsion>
  <engine file="propulsion_motor.xml"/>
</propulsion>
"""

    motor_xml = f"""<?xml version="1.0"?>
<rocket_motor>
  <catalog_root>{vehicle['motor_selection']['catalog_root']}</catalog_root>
{engine_lines}
</rocket_motor>
"""

    (path / "aircraft.xml").write_text(aircraft_xml)
    (path / "propulsion.xml").write_text(propulsion_xml)
    (path / "propulsion_motor.xml").write_text(motor_xml)


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
    profile_text = yaml.safe_dump(active_profile["data"], sort_keys=False)
    for profile_path in (path / "etc" / "flight_profiles", path / "fs" / "microsd" / "tv3" / "flight_profiles"):
        profile_path.mkdir(parents=True, exist_ok=True)
        (profile_path / "active.yaml").write_text(profile_text)
        (profile_path / f"{profile_name}.yaml").write_text(profile_text)


def write_runtime_assets(vehicle: dict, path: Path) -> None:
    etc_path = path / "etc"
    airframe_path = path / "fs" / "microsd" / "tv3" / "airframes"
    motor_path = path / "fs" / "microsd" / "tv3" / "motors"
    etc_path.mkdir(parents=True, exist_ok=True)

    config_template = (REPO_ROOT / "runtime" / "nuttx" / "etc" / "config.txt").read_text()
    config_text = config_template.replace("tv3_v1.params", f"{vehicle['name']}.params")
    (etc_path / "config.txt").write_text(config_text)

    extras_text = (REPO_ROOT / "runtime" / "nuttx" / "etc" / "extras.txt").read_text()
    if vehicle.get("guidance", {}).get("enable", 0) and "rocket_guidance start" not in extras_text:
        extras_text = extras_text.rstrip() + "\nrocket_guidance start\n"
    (etc_path / "extras.txt").write_text(extras_text)

    write_px4_params(vehicle, airframe_path / f"{vehicle['name']}.params")
    write_preliminary_motor_catalog(vehicle, motor_path)
    write_logger_topics(path)
    write_active_flight_profile(vehicle, path)


def generate_assets(vehicle_path: Path, output_root: Path, flight_profile_path: Path | None = None) -> None:
    vehicle = load_vehicle(vehicle_path)
    if flight_profile_path is not None:
        vehicle = apply_flight_profile(vehicle, load_flight_profile(flight_profile_path), flight_profile_path)
    validate_vehicle(vehicle)
    write_runtime_assets(vehicle, output_root / "runtime")
    write_gazebo_assets(vehicle, output_root / "gazebo" / vehicle["name"])
    write_jsbsim_assets(vehicle, output_root / "jsbsim" / vehicle["name"])


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
