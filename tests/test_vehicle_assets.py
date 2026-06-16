from __future__ import annotations

import csv
import importlib.util
import re
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def generated_params_path(output: Path, vehicle_name: str = "tv3_v1") -> Path:
    return output / "runtime" / "fs" / "microsd" / "tv3" / "airframes" / f"{vehicle_name}.params"


def generated_param_values(output: Path, vehicle_name: str = "tv3_v1") -> dict[str, str]:
    values = {}
    for line in generated_params_path(output, vehicle_name).read_text().splitlines():
        fields = line.split("\t")
        if len(fields) >= 4:
            values[fields[2]] = fields[3]
    return values


class VehicleAssetTests(unittest.TestCase):
    def test_generated_tv3_params_match_firmware_definitions(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = Path("config/vehicles/tv3_v1.yaml")

        defined_params = set(
            re.findall(r"PARAM_DEFINE_(?:INT32|FLOAT)\((RK_[A-Z0-9_]+),", Path("src/modules/flight_modes/tv3_params.c").read_text())
        )

        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "generated"
            module.generate_assets(vehicle, output)

            generated_params = set()
            generated_values = {}
            for line in generated_params_path(output).read_text().splitlines():
                fields = line.split("\t")
                if len(fields) >= 3 and fields[2].startswith("RK_"):
                    generated_params.add(fields[2])
                    generated_values[fields[2]] = fields[3]

            self.assertFalse(generated_params - defined_params)
            self.assertIn("RK_IGN_TO_MS", generated_params)
            self.assertIn("RK_GD_TAKE_ALT", generated_params)
            self.assertIn("RK_GD_WP1_N", generated_params)
            self.assertIn("RK_GD_TWR_MIN", generated_params)
            self.assertIn("RK_ABORT_GCS", generated_params)
            self.assertIn("RK_ENG_COUNT", generated_params)
            self.assertIn("RK_IGN_IDX0", generated_params)
            self.assertIn("RK_SPLAY_MAX_DEG", generated_params)
            self.assertEqual("0", generated_values["RK_GD_ENABLE"])
            self.assertEqual("1", generated_values["RK_ENG_COUNT"])
            self.assertNotIn("RK_IGN_TIMEOUT_MS", generated_params)
            self.assertNotIn("RK_GD_WP1_N_M", generated_params)

    def test_generate_vehicle_assets(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = Path("config/vehicles/tv3_v1.yaml")

        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "generated"
            module.generate_assets(vehicle, output)

            params = generated_params_path(output).read_text()
            self.assertIn("RK_LAUNCH_THR_N", params)
            self.assertIn("CA_RK_REF_THR", params)
            self.assertIn("RK_GD_ENABLE", params)
            self.assertIn("RK_GD_WP1_N", params)
            self.assertIn("RK_GD_MIN_IMP_NS", params)
            self.assertIn("RK_LC_ADC_INST\t1", params)
            self.assertIn("RK_LC_NEG_CH\t1", params)
            self.assertIn("RK_LC_MODE\t1", params)
            self.assertIn("RK_LC_KG_SC\t0.0", params)
            self.assertIn("RK_LC_RATE_HZ\t10", params)

            runtime_config = (output / "runtime" / "etc" / "config.txt").read_text()
            runtime_extras = (output / "runtime" / "etc" / "extras.txt").read_text()
            logger_topics = (output / "runtime" / "etc" / "logging" / "logger_topics.txt").read_text()
            self.assertIn("set TV3_AIRFRAME ${TV3_ROOT}/airframes/tv3_v1.params", runtime_config)
            self.assertIn("ads1115 start -X -b 2 -a 0x48", runtime_extras)
            self.assertIn("tv3_load_cell_telemetry start", runtime_extras)
            self.assertIn("mavlink stream -d /dev/ttyACM0 -s NAMED_VALUE_FLOAT -r 10", runtime_extras)
            self.assertIn("mavlink stream -d /dev/ttyACM0 -s DEBUG_VECT -r 10", runtime_extras)
            self.assertNotIn("tv3_mode_manager start", runtime_extras)
            self.assertNotIn("tv3_att_control start", runtime_extras)
            self.assertNotIn("tv3_guidance start", runtime_extras)
            self.assertIn("vehicle_attitude 20", logger_topics)
            self.assertIn("vehicle_local_position_groundtruth 50", logger_topics)
            self.assertIn("vehicle_torque_setpoint 50", logger_topics)
            self.assertIn("tv3_status 20", logger_topics)
            self.assertIn("tv3_thrust 20", logger_topics)
            self.assertTrue((output / "runtime" / "fs" / "microsd" / "etc" / "logging" / "logger_topics.txt").exists())
            self.assertTrue((output / "runtime" / "fs" / "microsd" / "tv3" / "motors" / "catalog.csv").exists())
            motor_curve = (
                output
                / "runtime"
                / "fs"
                / "microsd"
                / "tv3"
                / "motors"
                / "preliminary-tv3_v1-engine_0"
                / "curve.csv"
            ).read_text()
            self.assertIn("250.0", motor_curve)
            self.assertTrue(
                (
                    output
                    / "runtime"
                    / "fs"
                    / "microsd"
                    / "tv3"
                    / "motors"
                    / "preliminary-tv3_v1-engine_0"
                    / "curve.csv"
                ).exists()
            )

            self.assertFalse((output / "gazebo").exists())

    def test_generate_lander_manifest_assets(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = Path("config/vehicles/tv3_lander_v1.yaml")

        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "generated"
            module.generate_assets(vehicle, output)

            params = generated_params_path(output, "tv3_lander_v1").read_text()
            self.assertIn("CA_RK_GRP_CNT\t3", params)
            self.assertIn("RK_ENG_COUNT\t3", params)
            self.assertIn("RK_ENG2_MOT\t2", params)
            self.assertIn("RK_SPLAY_MAX_DEG\t35.0", params)
            self.assertIn("RK_GD_ENABLE\t1", params)
            self.assertIn("RK_GD_LAND_TWR\t1.15", params)

            runtime_config = (output / "runtime" / "etc" / "config.txt").read_text()
            runtime_extras = (output / "runtime" / "etc" / "extras.txt").read_text()
            self.assertIn("tv3_lander_v1.params", runtime_config)
            self.assertIn("tv3_guidance start", runtime_extras)

            with (output / "runtime" / "fs" / "microsd" / "tv3" / "motors" / "catalog.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(3, len(rows))
            self.assertEqual(["0", "1", "2"], [row["motor_index"] for row in rows])

            self.assertFalse((output / "gazebo").exists())

    def test_flight_profile_overlay_generates_guidance_params(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = Path("config/vehicles/tv3_lander_v1.yaml")
        profile = Path("config/flight_profiles/lander_hover_window.yaml")

        with TemporaryDirectory() as tmp:
            output = Path(tmp) / "generated"
            module.generate_assets(vehicle, output, profile)

            params = generated_param_values(output, "tv3_lander_v1")
            self.assertEqual("1", params["RK_GD_ENABLE"])
            self.assertEqual("1", params["RK_GD_SIM_GT"])
            self.assertEqual("8.0", params["RK_GD_TAKE_ALT"])
            self.assertEqual("18.0", params["RK_GD_APEX_ALT"])
            self.assertEqual("3.0", params["RK_GD_ACC_RAD"])
            self.assertEqual("0.0", params["RK_GD_WP1_N"])
            self.assertEqual("-8.0", params["RK_GD_WP1_D"])

            runtime_extras = (output / "runtime" / "etc" / "extras.txt").read_text()
            self.assertIn("tv3_guidance start", runtime_extras)
            active_profile = (output / "runtime" / "fs" / "microsd" / "tv3" / "flight_profiles" / "active.yaml").read_text()
            self.assertIn("name: lander_hover_window", active_profile)
            self.assertIn("type: hover_window", active_profile)
            self.assertTrue((output / "runtime" / "etc" / "flight_profiles" / "lander_hover_window.yaml").exists())

    def test_vehicle_intake_schema_exists(self) -> None:
        schema = Path("config/schemas/vehicle_intake_schema.yaml").read_text()
        self.assertIn("data_status_values", schema)
        self.assertIn("param_parity", schema)
        self.assertIn("unit_vec3", schema)

    def test_flight_profile_schema_and_examples_exist(self) -> None:
        schema = Path("config/schemas/flight_profile_schema.yaml").read_text()
        self.assertIn("tv3_flight_profile_schema_v1", schema)
        self.assertIn("TV3_FLIGHT_PROFILE", Path("config/flight_profiles/README.md").read_text())

        profiles = [
            "single_engine_ascent",
            "lander_ignition_sequence",
            "lander_hover_window",
            "lander_waypoint_track",
            "lander_abort_fault_path",
            "lander_impossible_guidance",
        ]
        for name in profiles:
            profile = yaml.safe_load(Path(f"config/flight_profiles/{name}.yaml").read_text())
            self.assertEqual("tv3_flight_profile_v1", profile["schema"])
            self.assertEqual(name, profile["name"])
            self.assertIn(profile["vehicle"], {"tv3_v1", "tv3_lander_v1"})
            self.assertIn("guidance", profile)
            self.assertIn("required_sim_gates", profile["mission_profile"])

    def test_allocator_reachability_for_lander(self) -> None:
        allocator = load_module(Path("tools/tv3_control_allocator.py"))
        vehicle = allocator.load_manifest(Path("config/vehicles/tv3_lander_v1.yaml"))
        engines = allocator.engines_from_vehicle(vehicle)

        reachable = allocator.allocate(engines, (0.0, 0.0, 0.0), 620.0)
        self.assertTrue(reachable.reachable, reachable)

        unreachable = allocator.allocate(engines, (0.0, 0.0, 0.0), 100.0)
        self.assertFalse(unreachable.reachable)
        self.assertEqual(allocator.REASON_THRUST_ENVELOPE, unreachable.reason)

    def test_sitl_airframes_share_common_defaults(self) -> None:
        common = Path("overlay/ROMFS/init.d-posix/airframes/tv3_common.inc").read_text()
        self.assertIn("param set-default CA_AIRFRAME 16", common)
        self.assertIn("param set-default RK_LC_SRC 1", common)
        self.assertNotIn("tv3_guidance start", common)

        sih = Path("overlay/ROMFS/init.d-posix/airframes/11002_tv3_lander").read_text()
        self.assertIn(". ${R}etc/init.d-posix/airframes/tv3_common.inc", sih)
        self.assertIn("PX4_SIMULATOR=${PX4_SIMULATOR:=sihsim}", sih)
        self.assertIn("PX4_SIM_MODEL=${PX4_SIM_MODEL:=tv3_lander}", sih)

        post = Path("overlay/ROMFS/init.d-posix/airframes/tv3_common.post").read_text()
        self.assertIn("tv3_mode_manager start", post)
        self.assertIn("tv3_att_control start", post)
        self.assertNotIn("tv3_guidance start", post)

        prepare = Path("scripts/prepare_px4_tree.sh").read_text()
        self.assertIn("tv3_guidance start", prepare)
        self.assertIn("RK_GD_ENABLE", prepare)


if __name__ == "__main__":
    unittest.main()
