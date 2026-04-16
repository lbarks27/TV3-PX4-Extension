#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def load_vehicle(path: Path) -> dict:
    with path.open() as stream:
        return yaml.safe_load(stream)


def write_px4_params(vehicle: dict, path: Path) -> None:
    controller = vehicle["controller"]
    state_machine = vehicle["state_machine"]
    hardware = vehicle["hardware"]
    body = vehicle["vehicle"]
    motor = vehicle["motor_selection"]
    load_cell = hardware["load_cell"]
    guidance = vehicle.get("guidance", {})

    def g(key: str, default):
        return guidance.get(key, default)

    lines = [
        "1\t1\tCA_AIRFRAME\t16\t6",
        "1\t1\tCA_RK_GRP_CNT\t1\t6",
        f"1\t1\tCA_RK_REF_THR\t{body['ca_reference_thrust_n']}\t9",
        f"1\t1\tCA_RK_MIN_THR\t{body['ca_minimum_thrust_n']}\t9",
        f"1\t1\tCA_RK_FAL_THR\t{body['ca_fallback_thrust_n']}\t9",
        f"1\t1\tCA_RK_BODY_M\t{body['body_mass_kg']}\t9",
        f"1\t1\tCA_RK_BODY_CMX\t{body['body_com_x_m']}\t9",
        f"1\t1\tCA_RK_MOT_WET\t{body['motor_loaded_mass_kg']}\t9",
        f"1\t1\tCA_RK_MOT_DRY\t{body['motor_dry_mass_kg']}\t9",
        f"1\t1\tCA_RK_MOT_CMX\t{body['motor_com_x_m']}\t9",
        f"1\t1\tCA_RK_G0_PX\t{body['motor_com_x_m']}\t9",
        "1\t1\tCA_RK_G0_PY\t0.0\t9",
        "1\t1\tCA_RK_G0_PZ\t0.0\t9",
        "1\t1\tCA_RK_G0_AX\t1.0\t9",
        "1\t1\tCA_RK_G0_AY\t0.0\t9",
        "1\t1\tCA_RK_G0_AZ\t0.0\t9",
        "1\t1\tCA_RK_G0_PAX\t0.0\t9",
        "1\t1\tCA_RK_G0_PAY\t-1.0\t9",
        "1\t1\tCA_RK_G0_PAZ\t0.0\t9",
        "1\t1\tCA_RK_G0_YAX\t0.0\t9",
        "1\t1\tCA_RK_G0_YAY\t0.0\t9",
        "1\t1\tCA_RK_G0_YAZ\t-1.0\t9",
        f"1\t1\tCA_RK_G0_PMAX\t{body['tvc_max_deg']}\t9",
        f"1\t1\tCA_RK_G0_YMAX\t{body['tvc_max_deg']}\t9",
        "1\t1\tCA_RK_G0_TF\t1.0\t9",
        "1\t1\tCA_RK_G0_PTR\t0.0\t9",
        "1\t1\tCA_RK_G0_YTR\t0.0\t9",
        "1\t1\tRK_ENABLE\t1\t6",
        f"1\t1\tRK_CMD_SRC\t{state_machine.get('command_source', 1)}\t6",
        f"1\t1\tRK_MOT_IDX\t{motor['index']}\t6",
        f"1\t1\tRK_LAUNCH_THR_N\t{state_machine['launch_threshold_n']}\t9",
        f"1\t1\tRK_IGNITION_MS\t{state_machine['ignition_pulse_ms']}\t6",
        f"1\t1\tRK_IGN_TIMEOUT_MS\t{state_machine['ignition_timeout_ms']}\t6",
        f"1\t1\tRK_BURN_MIN_MS\t{state_machine['minimum_burn_ms']}\t6",
        f"1\t1\tRK_BURN_MAX_MS\t{state_machine['maximum_burn_ms']}\t6",
        f"1\t1\tRK_BURNOUT_N\t{state_machine['burnout_threshold_n']}\t9",
        f"1\t1\tRK_BURNOUT_MS\t{state_machine['burnout_dwell_ms']}\t6",
        f"1\t1\tRK_RAIL_LEN_M\t{body['rail_length_m']}\t9",
        f"1\t1\tRK_BODY_MASS_KG\t{body['body_mass_kg']}\t9",
        f"1\t1\tRK_BODY_COM_X_M\t{body['body_com_x_m']}\t9",
        f"1\t1\tRK_MOTOR_COM_X_M\t{body['motor_com_x_m']}\t9",
        f"1\t1\tRK_TVC_MAX_DEG\t{body['tvc_max_deg']}\t9",
        f"1\t1\tRK_TVC_SLEW_DPS\t{body['tvc_slew_dps']}\t9",
        f"1\t1\tRK_TQ_P_MAX\t{body['torque_limits_nm']['pitch']}\t9",
        f"1\t1\tRK_TQ_Y_MAX\t{body['torque_limits_nm']['yaw']}\t9",
        f"1\t1\tRK_ATT_P_RAIL\t{controller['attitude_p']['rail']}\t9",
        f"1\t1\tRK_ATT_P_FREE\t{controller['attitude_p']['free']}\t9",
        f"1\t1\tRK_RATE_P_RAIL\t{controller['rate_p']['rail']}\t9",
        f"1\t1\tRK_RATE_P_FREE\t{controller['rate_p']['free']}\t9",
        f"1\t1\tRK_RATE_I\t{controller['rate_i']}\t9",
        f"1\t1\tRK_RATE_D\t{controller['rate_d']}\t9",
        f"1\t1\tRK_LC_SRC\t{load_cell.get('source', 0)}\t6",
        f"1\t1\tRK_LC_CH\t{load_cell['adc_channel']}\t6",
        f"1\t1\tRK_LC_TARE\t{load_cell['calibration']['tare']}\t9",
        f"1\t1\tRK_LC_SCALE\t{load_cell['calibration']['scale']}\t9",
        f"1\t1\tRK_LC_ALPHA\t{load_cell.get('alpha', 0.25)}\t9",
        f"1\t1\tRK_LC_TO_MS\t{load_cell.get('timeout_ms', 200)}\t6",
        f"1\t1\tRK_GD_ENABLE\t{g('enable', 1)}\t6",
        f"1\t1\tRK_GD_TAKEOFF_ALT_M\t{g('takeoff_alt_m', 35.0)}\t9",
        f"1\t1\tRK_GD_APEX_ALT_M\t{g('apex_alt_m', 120.0)}\t9",
        f"1\t1\tRK_GD_POS_P\t{g('pos_p', 0.15)}\t9",
        f"1\t1\tRK_GD_VEL_MAX_M_S\t{g('vel_max_m_s', 30.0)}\t9",
        f"1\t1\tRK_GD_VEL_UP_M_S\t{g('vel_up_m_s', 15.0)}\t9",
        f"1\t1\tRK_GD_VEL_DN_M_S\t{g('vel_dn_m_s', 8.0)}\t9",
        f"1\t1\tRK_GD_YAW_DEG\t{g('yaw_deg', 0.0)}\t9",
        f"1\t1\tRK_GD_HOLD_ALT_M\t{g('hold_alt_m', 5.0)}\t9",
        f"1\t1\tRK_GD_ACCEPTANCE_M\t{g('acceptance_m', 15.0)}\t9",
        f"1\t1\tRK_GD_WP1_N_M\t{g('wp1_n_m', 60.0)}\t9",
        f"1\t1\tRK_GD_WP1_E_M\t{g('wp1_e_m', 0.0)}\t9",
        f"1\t1\tRK_GD_WP1_D_M\t{g('wp1_d_m', -60.0)}\t9",
        f"1\t1\tRK_GD_WP2_N_M\t{g('wp2_n_m', 150.0)}\t9",
        f"1\t1\tRK_GD_WP2_E_M\t{g('wp2_e_m', 30.0)}\t9",
        f"1\t1\tRK_GD_WP2_D_M\t{g('wp2_d_m', -90.0)}\t9",
        f"1\t1\tRK_GD_WP3_N_M\t{g('wp3_n_m', 220.0)}\t9",
        f"1\t1\tRK_GD_WP3_E_M\t{g('wp3_e_m', 80.0)}\t9",
        f"1\t1\tRK_GD_WP3_D_M\t{g('wp3_d_m', -75.0)}\t9",
        f"1\t1\tRK_GD_LAND_N_M\t{g('land_n_m', 0.0)}\t9",
        f"1\t1\tRK_GD_LAND_E_M\t{g('land_e_m', 0.0)}\t9",
        f"1\t1\tRK_GD_LAND_D_M\t{g('land_d_m', 0.0)}\t9",
    ]

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def write_gazebo_assets(vehicle: dict, path: Path) -> None:
    name = vehicle["name"]
    path.mkdir(parents=True, exist_ok=True)

    model_config = f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>
"""

    model_sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="body">
      <inertial>
        <mass>{vehicle['vehicle']['body_mass_kg']}</mass>
      </inertial>
    </link>
    <plugin name="tv3_rocket" filename="libtv3_rocket_gz.so">
      <rail_length_m>{vehicle['vehicle']['rail_length_m']}</rail_length_m>
      <tvc_max_deg>{vehicle['vehicle']['tvc_max_deg']}</tvc_max_deg>
    </plugin>
  </model>
</sdf>
"""

    (path / "model.config").write_text(model_config)
    (path / "model.sdf").write_text(model_sdf)


def write_jsbsim_assets(vehicle: dict, path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)

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
  <motor_index>{vehicle['motor_selection']['index']}</motor_index>
</rocket_motor>
"""

    (path / "aircraft.xml").write_text(aircraft_xml)
    (path / "propulsion.xml").write_text(propulsion_xml)
    (path / "propulsion_motor.xml").write_text(motor_xml)


def generate_assets(vehicle_path: Path, output_root: Path) -> None:
	vehicle = load_vehicle(vehicle_path)
	write_px4_params(vehicle, output_root / "runtime" / f"{vehicle['name']}.params")
	write_gazebo_assets(vehicle, output_root / "gazebo" / vehicle["name"])
	write_jsbsim_assets(vehicle, output_root / "jsbsim" / vehicle["name"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", type=Path, required=True, help="Path to the shared vehicle definition")
    parser.add_argument("--output", type=Path, required=True, help="Output directory for generated assets")
    args = parser.parse_args()

    generate_assets(args.vehicle, args.output)
    print(f"generated assets for {args.vehicle} into {args.output}")


if __name__ == "__main__":
    main()
