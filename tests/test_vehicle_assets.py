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
    def test_generated_rocket_params_match_firmware_definitions(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = Path("config/vehicles/tv3_v1.yaml")

        defined_params = set(
            re.findall(r"PARAM_DEFINE_(?:INT32|FLOAT)\((RK_[A-Z0-9_]+),", Path("src/modules/flight_modes/rocket_params.c").read_text())
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

            runtime_config = (output / "runtime" / "etc" / "config.txt").read_text()
            runtime_extras = (output / "runtime" / "etc" / "extras.txt").read_text()
            logger_topics = (output / "runtime" / "etc" / "logging" / "logger_topics.txt").read_text()
            self.assertIn("set TV3_AIRFRAME ${TV3_ROOT}/airframes/tv3_v1.params", runtime_config)
            self.assertIn("rocket_mode_manager start", runtime_extras)
            self.assertNotIn("rocket_guidance start", runtime_extras)
            self.assertIn("vehicle_attitude 20", logger_topics)
            self.assertIn("vehicle_local_position_groundtruth 50", logger_topics)
            self.assertIn("vehicle_torque_setpoint 50", logger_topics)
            self.assertIn("rocket_status 20", logger_topics)
            self.assertIn("rocket_thrust 20", logger_topics)
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

            model_config = (output / "gazebo" / "tv3_v1" / "model.config").read_text()
            self.assertIn("<name>tv3_rocket</name>", model_config)

            model_sdf = (output / "gazebo" / "tv3_v1" / "model.sdf").read_text()
            self.assertIn('<model name="tv3_rocket">', model_sdf)
            self.assertIn('<link name="base_link">', model_sdf)
            self.assertIn("<mass>1.0</mass>", model_sdf)
            self.assertIn("<ixx>0.14435</ixx>", model_sdf)
            self.assertIn('<visual name="center_of_mass_marker">', model_sdf)
            self.assertIn("CAD mesh unavailable for assets/cad/tv3_v1/tv3_v1_static_structure.glb", model_sdf)
            self.assertIn("visual tv3_static_structure omitted", model_sdf)
            self.assertIn("CAD mesh unavailable for assets/cad/tv3_v1/tv3_v1_engine_nozzle.glb", model_sdf)
            self.assertIn("visual engine_nozzle_0 omitted", model_sdf)
            self.assertNotIn('<visual name="body_shell">', model_sdf)
            self.assertNotIn('<visual name="nose_cone">', model_sdf)
            self.assertNotIn('<visual name="aft_tvc_housing">', model_sdf)
            self.assertNotIn('<visual name="engine_nozzle_0">', model_sdf)
            self.assertNotIn('<visual name="fin_top">', model_sdf)
            self.assertIn('<visual name="thrust_cue_0">', model_sdf)
            self.assertIn('<visual name="orientation_axis_x_shaft">', model_sdf)
            self.assertIn('<visual name="orientation_axis_x_head">', model_sdf)
            self.assertIn('<visual name="orientation_axis_y_shaft">', model_sdf)
            self.assertIn('<visual name="orientation_axis_y_head">', model_sdf)
            self.assertIn('<visual name="orientation_axis_z_shaft">', model_sdf)
            self.assertIn('<visual name="orientation_axis_z_head">', model_sdf)
            self.assertIn("<pose>0.0 0.15 0.0 -1.57079632679 0.0 0.0</pose>", model_sdf)
            self.assertIn("<pose>0.0 0.0 0.15 0.0 0.0 0.0</pose>", model_sdf)
            self.assertIn('<visual name="joint_marker_tvc_pitch_axis_origin">', model_sdf)
            self.assertIn('<visual name="joint_axis_tvc_pitch_axis">', model_sdf)
            self.assertIn('<visual name="joint_marker_tvc_yaw_axis_origin">', model_sdf)
            self.assertIn('<visual name="joint_axis_tvc_yaw_axis">', model_sdf)
            self.assertIn("<pose>0.0 0.0 0.09 0.0 0.0 0.0</pose>", model_sdf)
            self.assertNotIn('<visual name="engine_pivot_marker_0">', model_sdf)
            self.assertEqual(1, model_sdf.count('<visual name="thrust_cue_0">'))
            self.assertEqual(1, model_sdf.count("visual tv3_static_structure omitted"))
            self.assertIn("<sphere>", model_sdf)
            self.assertIn("<radius>0.04</radius>", model_sdf)
            self.assertIn('<collision name="body_contact_collision">', model_sdf)
            self.assertEqual(1, model_sdf.count("<collision name="))
            self.assertIn("<radius>0.12</radius>", model_sdf)
            self.assertIn("<mass>0.2</mass>", model_sdf)
            self.assertIn('<sensor name="imu_sensor" type="imu">', model_sdf)
            self.assertIn('<joint name="tvc_pitch_axis" type="revolute">', model_sdf)
            self.assertIn("<parent>base_link</parent>", model_sdf)
            self.assertNotIn("tvc_mount_fixed", model_sdf)
            self.assertNotIn("libtv3_rocket_gz.so", model_sdf)
            self.assertTrue((output / "gazebo" / "tv3_v1" / "cad" / "swerve_vector_control_v4.step").exists())
            cad_manifest = (output / "gazebo" / "tv3_v1" / "cad_sources.yaml").read_text()
            self.assertIn("swerve_axis_v2.step", cad_manifest)

            jsbsim = (output / "jsbsim" / "tv3_v1" / "propulsion_motor.xml").read_text()
            self.assertIn('<engine id="engine_0">', jsbsim)
            self.assertIn("<motor_index>0</motor_index>", jsbsim)

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
            self.assertIn("rocket_guidance start", runtime_extras)

            with (output / "runtime" / "fs" / "microsd" / "tv3" / "motors" / "catalog.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(3, len(rows))
            self.assertEqual(["0", "1", "2"], [row["motor_index"] for row in rows])

            model_sdf = (output / "gazebo" / "tv3_lander_v1" / "model.sdf").read_text()
            self.assertIn('filename="libtv3_rocket_gz.so"', model_sdf)
            self.assertIn('<plugin name="tv3::Tv3RocketSystem" filename="libtv3_rocket_gz.so">', model_sdf)
            self.assertIn("<reference_thrust_n>55.0</reference_thrust_n>", model_sdf)
            self.assertIn("<thrust_axis_frame>world</thrust_axis_frame>", model_sdf)
            self.assertIn("<apply_engine_torques>false</apply_engine_torques>", model_sdf)
            self.assertIn("<commanded_thrust>true</commanded_thrust>", model_sdf)
            self.assertIn("<command_timeout_s>0.25</command_timeout_s>", model_sdf)
            self.assertIn("<command_scale>0.07333333333333333</command_scale>", model_sdf)
            self.assertIn("<force_application_m>0.72 0.0 0.0</force_application_m>", model_sdf)
            self.assertIn("<ignition_delay_s>0.0</ignition_delay_s>", model_sdf)
            self.assertIn("<burn_duration_s>15.0</burn_duration_s>", model_sdf)
            self.assertIn("<mass>3.85</mass>", model_sdf)
            self.assertIn("<air_pressure>", model_sdf)
            self.assertIn("<magnetometer>", model_sdf)
            self.assertIn("<imu>", model_sdf)
            self.assertIn("<pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>", model_sdf)
            self.assertIn("<pose>0.72 0.0 0.0 0.0 0.0 0.0</pose>", model_sdf)
            self.assertIn('<visual name="center_of_mass_marker">', model_sdf)
            self.assertIn("CAD mesh unavailable for assets/cad/lander/tv3_lander_static_structure.glb", model_sdf)
            self.assertIn("visual lander_static_structure omitted", model_sdf)
            self.assertIn("CAD mesh unavailable for assets/cad/lander/tv3_lander_engine_nozzle.glb", model_sdf)
            self.assertIn("visual engine_nozzle_0 omitted", model_sdf)
            self.assertNotIn('<visual name="lander_core">', model_sdf)
            self.assertNotIn('<visual name="avionics_deck">', model_sdf)
            self.assertNotIn('<visual name="engine_pod_0">', model_sdf)
            self.assertNotIn('<visual name="engine_nozzle_0">', model_sdf)
            self.assertIn('<visual name="thrust_cue_0">', model_sdf)
            self.assertNotIn('<visual name="engine_pod_1">', model_sdf)
            self.assertNotIn('<visual name="engine_nozzle_1">', model_sdf)
            self.assertIn('<visual name="thrust_cue_1">', model_sdf)
            self.assertNotIn('<visual name="engine_pod_2">', model_sdf)
            self.assertNotIn('<visual name="engine_nozzle_2">', model_sdf)
            self.assertIn('<visual name="thrust_cue_2">', model_sdf)
            self.assertNotIn('<visual name="landing_leg_0">', model_sdf)
            self.assertIn('<visual name="orientation_axis_x_shaft">', model_sdf)
            self.assertIn('<visual name="orientation_axis_x_head">', model_sdf)
            self.assertIn('<visual name="orientation_axis_y_shaft">', model_sdf)
            self.assertIn('<visual name="orientation_axis_y_head">', model_sdf)
            self.assertIn('<visual name="orientation_axis_z_shaft">', model_sdf)
            self.assertIn('<visual name="orientation_axis_z_head">', model_sdf)
            self.assertIn("<pose>0.845 0.0 0.0 0.0 1.57079632679 0.0</pose>", model_sdf)
            self.assertIn("<pose>0.72 0.125 0.0 -1.57079632679 0.0 0.0</pose>", model_sdf)
            self.assertIn("<pose>0.72 0.0 0.125 0.0 0.0 0.0</pose>", model_sdf)
            self.assertIn('<visual name="engine_pivot_marker_0">', model_sdf)
            self.assertIn('<visual name="engine_pivot_thrust_axis_0">', model_sdf)
            self.assertIn('<visual name="engine_pivot_marker_1">', model_sdf)
            self.assertIn('<visual name="engine_pivot_thrust_axis_1">', model_sdf)
            self.assertIn('<visual name="engine_pivot_marker_2">', model_sdf)
            self.assertIn('<visual name="engine_pivot_thrust_axis_2">', model_sdf)
            self.assertIn("<pose>0.12 0.12 0.08 0.0 0.0 0.0</pose>", model_sdf)
            self.assertIn("<pose>0.12 -0.06 0.183923 0.0 0.0 0.0</pose>", model_sdf)
            self.assertIn("<pose>0.12 -0.06 -0.023923 0.0 0.0 0.0</pose>", model_sdf)
            self.assertNotIn("<pose>0.2 0.12 0.0 0.0 1.57079632679 0.0</pose>", model_sdf)
            self.assertEqual(16, model_sdf.count("<visual name="))
            self.assertIn('<collision name="body_contact_collision">', model_sdf)
            self.assertEqual(1, model_sdf.count("<collision name="))
            self.assertIn("<engine_count>3</engine_count>", model_sdf)
            self.assertIn("<ignition_sequence>0 1 2</ignition_sequence>", model_sdf)
            self.assertEqual(3, model_sdf.count("<thrust_axis>0.0 0.0 1.0</thrust_axis>"))
            self.assertEqual(3, model_sdf.count("<engine index="))

            jsbsim = (output / "jsbsim" / "tv3_lander_v1" / "propulsion_motor.xml").read_text()
            self.assertIn('<engine id="engine_2">', jsbsim)

    def test_gazebo_visual_model_copies_renderable_meshes(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = yaml.safe_load(Path("config/vehicles/tv3_v1.yaml").read_text())

        with TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            mesh = tmp_path / "probe.stl"
            mesh.write_text("solid probe\nendsolid probe\n")
            vehicle["gazebo"]["visual_model"] = {
                "mesh_policy": "omit_unavailable",
                "visuals": [
                    {
                        "name": "mesh_probe",
                        "kind": "mesh",
                        "mesh_uri": str(mesh),
                        "mesh_scale": [1.0, 2.0, 3.0],
                        "pose_m": [0.0, 0.0, 0.0],
                        "color_rgba": [0.4, 0.5, 0.6, 1.0],
                    }
                ]
            }
            manifest = tmp_path / "vehicle.yaml"
            manifest.write_text(yaml.safe_dump(vehicle, sort_keys=False))
            output = tmp_path / "generated"

            module.generate_assets(manifest, output)

            model_sdf = (output / "gazebo" / "tv3_v1" / "model.sdf").read_text()
            self.assertIn('<visual name="mesh_probe">', model_sdf)
            self.assertIn("<mesh>", model_sdf)
            self.assertIn("<uri>meshes/probe.stl</uri>", model_sdf)
            self.assertIn("<scale>1.0 2.0 3.0</scale>", model_sdf)
            self.assertTrue((output / "gazebo" / "tv3_v1" / "meshes" / "probe.stl").exists())

    def test_gazebo_visual_model_require_renderable_meshes_rejects_missing_assets(self) -> None:
        module = load_module(Path("tools/generate_vehicle_assets.py"))
        vehicle = yaml.safe_load(Path("config/vehicles/tv3_v1.yaml").read_text())

        with TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            vehicle["gazebo"]["visual_model"] = {
                "mesh_policy": "require_renderable",
                "visuals": [
                    {
                        "name": "missing_mesh_probe",
                        "kind": "mesh",
                        "mesh_uri": str(tmp_path / "missing.glb"),
                        "pose_m": [0.0, 0.0, 0.0],
                    }
                ],
            }
            manifest = tmp_path / "vehicle.yaml"
            manifest.write_text(yaml.safe_dump(vehicle, sort_keys=False))

            with self.assertRaises(FileNotFoundError):
                module.generate_assets(manifest, tmp_path / "generated")

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
            self.assertIn("rocket_guidance start", runtime_extras)
            active_profile = (output / "runtime" / "fs" / "microsd" / "tv3" / "flight_profiles" / "active.yaml").read_text()
            self.assertIn("name: lander_hover_window", active_profile)
            self.assertIn("type: hover_window", active_profile)
            self.assertTrue((output / "runtime" / "etc" / "flight_profiles" / "lander_hover_window.yaml").exists())

    def test_vehicle_intake_schema_exists(self) -> None:
        schema = Path("config/schemas/vehicle_intake_schema.yaml").read_text()
        self.assertIn("mass, CG, inertia", schema)
        self.assertIn("splay/cosine-loss throttle mechanism", schema)
        self.assertIn("load-cell ADC", schema)

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
        ]
        for name in profiles:
            profile = yaml.safe_load(Path(f"config/flight_profiles/{name}.yaml").read_text())
            self.assertEqual("tv3_flight_profile_v1", profile["schema"])
            self.assertEqual(name, profile["name"])
            self.assertIn(profile["vehicle"], {"tv3_v1", "tv3_lander_v1"})
            self.assertIn("guidance", profile)
            self.assertIn("required_sim_gates", profile["mission_profile"])

    def test_allocator_reachability_for_lander(self) -> None:
        allocator = load_module(Path("tools/rocket_allocator.py"))
        vehicle = allocator.yaml.safe_load(Path("config/vehicles/tv3_lander_v1.yaml").read_text())
        engines = allocator.engines_from_vehicle(vehicle)

        reachable = allocator.allocate(engines, (0.0, 0.0, 0.0), 620.0)
        self.assertTrue(reachable["reachable"], reachable)

        unreachable = allocator.allocate(engines, (0.0, 0.0, 0.0), 100.0)
        self.assertFalse(unreachable["reachable"])
        self.assertEqual("net thrust outside splay envelope", unreachable["reason"])

    def test_sitl_airframes_share_common_defaults(self) -> None:
        common = Path("overlay/ROMFS/init.d-posix/airframes/tv3_rocket_common.inc").read_text()
        self.assertIn("param set-default CA_AIRFRAME 16", common)
        self.assertIn("param set-default RK_LC_SRC 1", common)
        self.assertNotIn("rocket_guidance start", common)

        gz = Path("overlay/ROMFS/init.d-posix/airframes/11000_gz_tv3_rocket").read_text()
        self.assertIn(". ${R}etc/init.d-posix/airframes/tv3_rocket_common.inc", gz)
        self.assertIn("PX4_SIMULATOR=${PX4_SIMULATOR:=gz}", gz)

        jsbsim = Path("overlay/ROMFS/init.d-posix/airframes/11001_jsbsim_tv3_rocket").read_text()
        self.assertIn(". ${R}etc/init.d-posix/airframes/tv3_rocket_common.inc", jsbsim)
        self.assertIn("param set-default SYS_HITL 1", jsbsim)

        post = Path("overlay/ROMFS/init.d-posix/airframes/tv3_rocket_common.post").read_text()
        self.assertIn("rocket_mode_manager start", post)
        self.assertIn("rocket_att_control start", post)
        self.assertNotIn("rocket_guidance start", post)

        prepare = Path("scripts/prepare_px4_tree.sh").read_text()
        self.assertIn("rocket_guidance start", prepare)
        self.assertIn("RK_GD_ENABLE", prepare)


if __name__ == "__main__":
    unittest.main()
