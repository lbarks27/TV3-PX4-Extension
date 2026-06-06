#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENDOR_DIR=$("${SCRIPT_DIR}/bootstrap_px4.sh")
PX4_REF="${PX4_REF:-v1.16.1}"
WORK_ROOT="${PX4_WORK_ROOT:-${TV3_ROOT}/.work}"
WORKTREE="${WORK_ROOT}/px4-tv3"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.yaml}"
FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"
if [[ "${VEHICLE_CONFIG}" != /* ]]; then
	VEHICLE_CONFIG="${REPO_ROOT}/${VEHICLE_CONFIG}"
fi
if [ -n "${FLIGHT_PROFILE}" ] && [[ "${FLIGHT_PROFILE}" != /* ]]; then
	FLIGHT_PROFILE="${REPO_ROOT}/${FLIGHT_PROFILE}"
fi

mkdir -p "${WORK_ROOT}"

git -C "${VENDOR_DIR}" worktree prune >/dev/null 2>&1 || true
git -C "${VENDOR_DIR}" worktree remove --force "${WORKTREE}" >/dev/null 2>&1 || true
rm -rf "${WORKTREE}"
git -C "${VENDOR_DIR}" worktree add --detach "${WORKTREE}" "${PX4_REF}" >/dev/null 2>&1
git -C "${WORKTREE}" submodule update --init --recursive --jobs 8 >/dev/null 2>&1

for patch in "${REPO_ROOT}"/patches/px4/*.patch; do
	[ -f "${patch}" ] || continue
	git -C "${WORKTREE}" apply --reject "${patch}" >/dev/null 2>&1 || true
done

python3 - "${REPO_ROOT}" "${WORKTREE}" "${VEHICLE_CONFIG}" "${FLIGHT_PROFILE}" <<'PY'
from pathlib import Path
import shutil
import subprocess
import sys

import yaml

repo_root = Path(sys.argv[1])
worktree = Path(sys.argv[2])
vehicle_config = Path(sys.argv[3])
flight_profile = Path(sys.argv[4]) if sys.argv[4] else None
patch = (repo_root / "patches/px4/0001-rocket-control-allocation.patch").read_text()

def write_text_if_changed(path: Path, text: str) -> None:
	current = path.read_text() if path.exists() else None
	if current != text:
		path.parent.mkdir(parents=True, exist_ok=True)
		path.write_text(text)

def extract_added_file(patch_text: str, relative_path: str) -> str:
	header = f"diff --git a/{relative_path} b/{relative_path}"
	block = patch_text.split(header, 1)[1].split("\ndiff --git ", 1)[0]
	lines = []

	for line in block.splitlines():
		if line.startswith("+++ ") or line.startswith("--- ") or line.startswith("@@"):
			continue
		if line.startswith("+"):
			lines.append(line[1:])

	return "\n".join(lines).rstrip("\n") + "\n"

module_yaml = worktree / "src/modules/control_allocator/module.yaml"
text = module_yaml.read_text()

text = text.replace("                15: Spacecraft 3D\n            default: 0\n", "                15: Spacecraft 3D\n                16: Rocket\n            default: 0\n", 1)

param_marker = "+        # rocket TVC\n"
rocket_marker = "+            16: # Rocket\n"
module_patch = patch.split("diff --git a/src/modules/control_allocator/module.yaml b/src/modules/control_allocator/module.yaml", 1)[1]
module_patch = module_patch.split("--- a/src/modules/control_allocator/VehicleActuatorEffectiveness/apogee_f10_mass_curve.hpp", 1)[0]
param_block_raw = module_patch.split(param_marker, 1)[1].split(rocket_marker, 1)[0]
rocket_block_raw = module_patch.split(rocket_marker, 1)[1]

def strip_plus(block: str) -> str:
	lines = []
	for line in block.splitlines():
		if line.startswith("+"):
			lines.append(line[1:])
	return "\n".join(lines)

param_block = strip_plus(param_block_raw).rstrip("\n")
rocket_block = strip_plus(rocket_block_raw).rstrip("\n")

if "# rocket TVC" not in text:
	insert_at = text.index("        # Tilts\n")
	text = text[:insert_at] + param_block + "\n\n" + text[insert_at:]

if "16: # Rocket" not in text:
	text = text.rstrip("\n") + "\n\n" + rocket_block + "\n"

module_yaml.write_text(text)

commander = worktree / "src/modules/commander/Commander.cpp"
commander_text = commander.read_text()
if "case 31010:" not in commander_text:
	commander_text = commander_text.replace(
		"\tcase vehicle_command_s::VEHICLE_CMD_REQUEST_CAMERA_INFORMATION:\n"
		"\t\t/* ignore commands that are handled by other parts of the system */\n",
		"\tcase vehicle_command_s::VEHICLE_CMD_REQUEST_CAMERA_INFORMATION:\n"
		"\tcase 31010: // MAV_CMD_USER_1, handled by rocket_mode when enabled\n"
		"\t\t/* ignore commands that are handled by other parts of the system */\n",
		1,
	)
	write_text_if_changed(commander, commander_text)

control_allocator_hpp = worktree / "src/modules/control_allocator/ControlAllocator.hpp"
control_allocator_hpp_text = control_allocator_hpp.read_text()
if "#include <ActuatorEffectivenessRocket.hpp>" not in control_allocator_hpp_text:
	control_allocator_hpp_text = control_allocator_hpp_text.replace(
		"#include <ActuatorEffectivenessHelicopterCoaxial.hpp>\n",
		"#include <ActuatorEffectivenessHelicopterCoaxial.hpp>\n"
		"#include <ActuatorEffectivenessRocket.hpp>\n",
		1,
	)
if "\t\tROCKET = 16," not in control_allocator_hpp_text:
	control_allocator_hpp_text = control_allocator_hpp_text.replace(
		"\t\tSPACECRAFT_3D = 14,\n",
		"\t\tSPACECRAFT_3D = 14,\n"
		"\t\tROCKET = 16,\n",
		1,
	)
write_text_if_changed(control_allocator_hpp, control_allocator_hpp_text)

control_allocator_cpp = worktree / "src/modules/control_allocator/ControlAllocator.cpp"
control_allocator_cpp_text = control_allocator_cpp.read_text()
if "EffectivenessSource::ROCKET" not in control_allocator_cpp_text:
	control_allocator_cpp_text = control_allocator_cpp_text.replace(
		"\t\tcase EffectivenessSource::SPACECRAFT_3D:\n"
		"\t\t\t// spacecraft_allocation does allocation and publishes directly to actuator_motors topic\n"
		"\t\t\tbreak;\n\n"
		"\t\tdefault:\n",
		"\t\tcase EffectivenessSource::SPACECRAFT_3D:\n"
		"\t\t\t// spacecraft_allocation does allocation and publishes directly to actuator_motors topic\n"
		"\t\t\tbreak;\n\n"
		"\t\tcase EffectivenessSource::ROCKET:\n"
		"\t\t\ttmp = new ActuatorEffectivenessRocket(this);\n"
		"\t\t\tbreak;\n\n"
		"\t\tdefault:\n",
		1,
	)
	write_text_if_changed(control_allocator_cpp, control_allocator_cpp_text)

vehicle_effectiveness = worktree / "src/modules/control_allocator/VehicleActuatorEffectiveness"
for relative_path in (
	"src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRocket.hpp",
	"src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRocket.cpp",
):
	write_text_if_changed(worktree / relative_path, extract_added_file(patch, relative_path))

plugin_source = repo_root / "src/gazebo_plugins/tv3_rocket"
plugin_target = worktree / "src/modules/simulation/gz_plugins/tv3_rocket"
if plugin_source.exists():
	if plugin_target.exists():
		shutil.rmtree(plugin_target)
	shutil.copytree(plugin_source, plugin_target)

vehicle_effectiveness_cmake = vehicle_effectiveness / "CMakeLists.txt"
vehicle_effectiveness_cmake_text = vehicle_effectiveness_cmake.read_text()
if "ActuatorEffectivenessRocket.cpp" not in vehicle_effectiveness_cmake_text:
	vehicle_effectiveness_cmake_text = vehicle_effectiveness_cmake_text.replace(
		"\tActuatorEffectivenessRoverAckermann.hpp\n"
		"\tActuatorEffectivenessRoverAckermann.cpp\n",
		"\tActuatorEffectivenessRoverAckermann.hpp\n"
		"\tActuatorEffectivenessRoverAckermann.cpp\n"
		"\tActuatorEffectivenessRocket.hpp\n"
		"\tActuatorEffectivenessRocket.cpp\n",
		1,
	)
	write_text_if_changed(vehicle_effectiveness_cmake, vehicle_effectiveness_cmake_text)

gz_plugins_cmake = worktree / "src/modules/simulation/gz_plugins/CMakeLists.txt"
gz_plugins_cmake_text = gz_plugins_cmake.read_text()
if "add_subdirectory(tv3_rocket)" not in gz_plugins_cmake_text:
	gz_plugins_cmake_text = gz_plugins_cmake_text.replace(
		"    add_subdirectory(moving_platform_controller)\n",
		"    add_subdirectory(moving_platform_controller)\n"
		"    add_subdirectory(tv3_rocket)\n",
		1,
	)
	gz_plugins_cmake_text = gz_plugins_cmake_text.replace(
		"DEPENDS OpticalFlowSystem MovingPlatformController TemplatePlugin GstCameraSystem)",
		"DEPENDS OpticalFlowSystem MovingPlatformController TemplatePlugin GstCameraSystem tv3_rocket_gz)",
	)
	gz_plugins_cmake_text = gz_plugins_cmake_text.replace(
		"DEPENDS OpticalFlowSystem MovingPlatformController TemplatePlugin)",
		"DEPENDS OpticalFlowSystem MovingPlatformController TemplatePlugin tv3_rocket_gz)",
	)
	write_text_if_changed(gz_plugins_cmake, gz_plugins_cmake_text)

gz_bridge_cmake = worktree / "src/modules/simulation/gz_bridge/CMakeLists.txt"
gz_bridge_cmake_text = gz_bridge_cmake.read_text()
if "$<TARGET_FILE:px4> -w ${PX4_BINARY_DIR} ${PX4_BINARY_DIR}/etc" not in gz_bridge_cmake_text:
	gz_bridge_cmake_text = gz_bridge_cmake_text.replace(
		"$<TARGET_FILE:px4>",
		"$<TARGET_FILE:px4> -w ${PX4_BINARY_DIR} ${PX4_BINARY_DIR}/etc",
	)
if "configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/gz_env.sh)" not in gz_bridge_cmake_text:
	gz_bridge_cmake_text = gz_bridge_cmake_text.replace(
		"configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/rootfs/gz_env.sh)",
		"configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/rootfs/gz_env.sh)\n"
		"\tconfigure_file(gz_env.sh.in ${PX4_BINARY_DIR}/gz_env.sh)",
	)
if "WORKING_DIRECTORY ${SITL_WORKING_DIR}" in gz_bridge_cmake_text:
	gz_bridge_cmake_text = gz_bridge_cmake_text.replace(
		"WORKING_DIRECTORY ${SITL_WORKING_DIR}",
		"WORKING_DIRECTORY ${PX4_BINARY_DIR}",
	)
write_text_if_changed(gz_bridge_cmake, gz_bridge_cmake_text)

gz_bridge_hpp = worktree / "src/modules/simulation/gz_bridge/GZBridge.hpp"
gz_bridge_hpp_text = gz_bridge_hpp.read_text()
if "#include <uORB/topics/rocket_engine_command.h>" not in gz_bridge_hpp_text:
	gz_bridge_hpp_text = gz_bridge_hpp_text.replace(
		"#include <uORB/topics/parameter_update.h>\n",
		"#include <uORB/topics/parameter_update.h>\n"
		"#include <uORB/topics/rocket_engine_command.h>\n",
		1,
	)
if "#include <uORB/topics/rocket_engine_state.h>" not in gz_bridge_hpp_text:
	gz_bridge_hpp_text = gz_bridge_hpp_text.replace(
		"#include <uORB/topics/parameter_update.h>\n",
		"#include <uORB/topics/parameter_update.h>\n"
		"#include <uORB/topics/rocket_engine_state.h>\n",
		1,
	)
if "\tvoid publishRocketEngineState();" not in gz_bridge_hpp_text:
	gz_bridge_hpp_text = gz_bridge_hpp_text.replace(
		"\tvoid addGpsNoise(double &latitude, double &longitude, double &altitude,\n"
		"\t\t\t float &vel_north, float &vel_east, float &vel_down);\n",
		"\tvoid addGpsNoise(double &latitude, double &longitude, double &altitude,\n"
		"\t\t\t float &vel_north, float &vel_east, float &vel_down);\n"
		"\tvoid publishRocketEngineState();\n",
		1,
	)
if "\tuORB::Subscription _rocket_engine_state_sub{ORB_ID(rocket_engine_state)};" not in gz_bridge_hpp_text:
	gz_bridge_hpp_text = gz_bridge_hpp_text.replace(
		"\tuORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};\n",
		"\tuORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};\n"
		"\tuORB::Subscription                            _rocket_engine_state_sub{ORB_ID(rocket_engine_state)};\n",
		1,
	)
if "\tuORB::Subscription _rocket_engine_command_sub{ORB_ID(rocket_engine_command)};" not in gz_bridge_hpp_text:
	gz_bridge_hpp_text = gz_bridge_hpp_text.replace(
		"\tuORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};\n",
		"\tuORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};\n"
		"\tuORB::Subscription                            _rocket_engine_command_sub{ORB_ID(rocket_engine_command)};\n",
		1,
	)
if "\tgz::transport::Node::Publisher _rocket_engine_actuators_pub;" not in gz_bridge_hpp_text:
	gz_bridge_hpp_text = gz_bridge_hpp_text.replace(
		"\tgz::transport::Node _node;\n",
		"\tgz::transport::Node _node;\n"
		"\tgz::transport::Node::Publisher _rocket_engine_actuators_pub;\n",
		1,
	)
write_text_if_changed(gz_bridge_hpp, gz_bridge_hpp_text)

gz_bridge_cpp = worktree / "src/modules/simulation/gz_bridge/GZBridge.cpp"
gz_bridge_cpp_text = gz_bridge_cpp.read_text()
if "_rocket_engine_actuators_pub = _node.Advertise<gz::msgs::Actuators>" not in gz_bridge_cpp_text:
	gz_bridge_cpp_text = gz_bridge_cpp_text.replace(
		"\tif (!_gimbal.init(_world_name, _model_name)) {\n"
		"\t\tPX4_ERR(\"failed to init gimbal\");\n"
		"\t\treturn PX4_ERROR;\n"
		"\t}\n\n"
		"\tScheduleNow();\n",
		"\tif (!_gimbal.init(_world_name, _model_name)) {\n"
		"\t\tPX4_ERR(\"failed to init gimbal\");\n"
		"\t\treturn PX4_ERROR;\n"
		"\t}\n\n"
		"\tstd::string rocket_engine_topic = \"/\" + _model_name + \"/command/rocket_thrust\";\n"
		"\t_rocket_engine_actuators_pub = _node.Advertise<gz::msgs::Actuators>(rocket_engine_topic);\n\n"
		"\tif (!_rocket_engine_actuators_pub.Valid()) {\n"
		"\t\tPX4_WARN(\"failed to advertise %s\", rocket_engine_topic.c_str());\n"
		"\t}\n\n"
		"\tScheduleNow();\n",
		1,
	)
if "void GZBridge::publishRocketEngineState()" not in gz_bridge_cpp_text:
	gz_bridge_cpp_text = gz_bridge_cpp_text.replace(
		"\nvoid GZBridge::Run()\n{\n",
		"\nvoid GZBridge::publishRocketEngineState()\n"
		"{\n"
		"\tif (!_rocket_engine_actuators_pub.Valid()) {\n"
		"\t\treturn;\n"
		"\t}\n\n"
		"\trocket_engine_state_s state{};\n"
		"\trocket_engine_command_s command{};\n"
		"\tbool updated = false;\n\n"
		"\twhile (_rocket_engine_state_sub.update(&state)) {\n"
		"\t\tupdated = true;\n"
		"\t}\n\n"
		"\twhile (_rocket_engine_command_sub.update(&command)) {\n"
		"\t}\n\n"
		"\tif (!updated) {\n"
		"\t\treturn;\n"
		"\t}\n\n"
		"\tconst int command_count = state.engine_count < rocket_engine_state_s::MAX_ENGINES\n"
		"\t\t? state.engine_count : rocket_engine_state_s::MAX_ENGINES;\n\n"
		"\tgz::msgs::Actuators actuators;\n"
		"\tactuators.mutable_velocity()->Resize(command_count, 0.0);\n\n"
		"\tactuators.mutable_position()->Resize(command_count * 3, 0.0);\n\n"
		"\tconst uint64_t timestamp = hrt_absolute_time();\n"
		"\tauto *stamp = actuators.mutable_header()->mutable_stamp();\n"
		"\tstamp->set_sec(timestamp / 1000000ULL);\n"
		"\tstamp->set_nsec((timestamp % 1000000ULL) * 1000ULL);\n\n"
		"\tfor (int i = 0; i < command_count; ++i) {\n"
		"\t\tfloat thrust_n = state.filtered_thrust_n[i];\n\n"
		"\t\tif (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {\n"
		"\t\t\tthrust_n = state.measured_thrust_n[i];\n"
		"\t\t}\n\n"
		"\t\tif ((state.active_mask & (1u << i)) == 0 || !PX4_ISFINITE(thrust_n) || thrust_n < 0.f) {\n"
		"\t\t\tthrust_n = 0.f;\n"
		"\t\t}\n\n"
		"\t\tactuators.set_velocity(i, thrust_n);\n"
		"\t\tactuators.set_position(i, command.commanded_pitch_deg[i]);\n"
		"\t\tactuators.set_position(command_count + i, command.commanded_yaw_deg[i]);\n"
		"\t\tactuators.set_position((command_count * 2) + i, command.commanded_splay_deg[i]);\n"
		"\t}\n\n"
		"\t_rocket_engine_actuators_pub.Publish(actuators);\n"
		"}\n"
		"\nvoid GZBridge::Run()\n{\n",
		1,
	)
if "\t\t\tlocal_position_groundtruth.xy_valid = true;\n" not in gz_bridge_cpp_text:
	gz_bridge_cpp_text = gz_bridge_cpp_text.replace(
		"\t\t\tlocal_position_groundtruth.z = position(2);\n\n"
		"\t\t\tlocal_position_groundtruth.heading = euler.psi();\n",
		"\t\t\tlocal_position_groundtruth.z = position(2);\n"
		"\t\t\tlocal_position_groundtruth.xy_valid = true;\n"
		"\t\t\tlocal_position_groundtruth.z_valid = true;\n"
		"\t\t\tlocal_position_groundtruth.v_xy_valid = true;\n"
		"\t\t\tlocal_position_groundtruth.v_z_valid = true;\n"
		"\t\t\tlocal_position_groundtruth.heading_good_for_control = true;\n"
		"\t\t\tlocal_position_groundtruth.eph = 0.f;\n"
		"\t\t\tlocal_position_groundtruth.epv = 0.f;\n"
		"\t\t\tlocal_position_groundtruth.evh = 0.f;\n"
		"\t\t\tlocal_position_groundtruth.evv = 0.f;\n\n"
		"\t\t\tlocal_position_groundtruth.heading = euler.psi();\n",
		1,
	)
if "\tpublishRocketEngineState();\n\n\tScheduleDelayed(10_ms);" not in gz_bridge_cpp_text:
	gz_bridge_cpp_text = gz_bridge_cpp_text.replace(
		"\tScheduleDelayed(10_ms);\n",
		"\tpublishRocketEngineState();\n\n"
		"\tScheduleDelayed(10_ms);\n",
		1,
	)
write_text_if_changed(gz_bridge_cpp, gz_bridge_cpp_text)

cmake_lists = worktree / "CMakeLists.txt"
cmake_text = cmake_lists.read_text()
cmake_text = cmake_text.replace("set(CMAKE_CXX_STANDARD 14)", "set(CMAKE_CXX_STANDARD 17)", 1)
cmake_lists.write_text(cmake_text)

common_flags = worktree / "cmake/px4_add_common_flags.cmake"
common_flags_text = common_flags.read_text()
common_flags_text = common_flags_text.replace("-Wdouble-promotion", "-Wno-double-promotion")
common_flags_text = common_flags_text.replace("-Werror", "-Wno-error")
common_flags.write_text(common_flags_text)

optical_flow_cmake = worktree / "src/modules/simulation/gz_plugins/optical_flow/optical_flow.cmake"
optical_flow_text = optical_flow_cmake.read_text()
if "libOpticalFlow${CMAKE_SHARED_LIBRARY_SUFFIX}" not in optical_flow_text:
    optical_flow_text = optical_flow_text.replace(
        "if(NOT TARGET OpticalFlow)\n",
        "if(NOT TARGET OpticalFlow)\n"
        "    set(_opticalflow_install_dir ${CMAKE_BINARY_DIR}/external/Install)\n"
        "    set(_opticalflow_lib ${_opticalflow_install_dir}/lib/libOpticalFlow${CMAKE_SHARED_LIBRARY_SUFFIX})\n\n",
        1,
    )
    optical_flow_text = optical_flow_text.replace(
        "INSTALL_DIR ${CMAKE_BINARY_DIR}/OpticalFlow/install",
        "INSTALL_DIR ${_opticalflow_install_dir}",
        1,
    )
    optical_flow_text = optical_flow_text.replace(
        "BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/OpticalFlow/install/lib/libOpticalFlow.so",
        "BUILD_BYPRODUCTS ${_opticalflow_lib}",
        1,
    )
    optical_flow_text = optical_flow_text.replace(
        "set(OpticalFlow_LIBS ${install_dir}/lib/libOpticalFlow.so CACHE INTERNAL \"\")",
        "set(OpticalFlow_LIBS ${_opticalflow_lib} CACHE INTERNAL \"\")",
        1,
    )
    optical_flow_cmake.write_text(optical_flow_text)

sitl_board = worktree / "boards/px4/sitl/default.px4board"
sitl_board_text = sitl_board.read_text()
if "CONFIG_MODULES_INTERNAL_COMBUSTION_ENGINE_CONTROL=y" not in sitl_board_text:
	sitl_board_text = sitl_board_text.replace(
		"CONFIG_MODULES_CONTROL_ALLOCATOR=y\n",
		"CONFIG_MODULES_CONTROL_ALLOCATOR=y\nCONFIG_MODULES_INTERNAL_COMBUSTION_ENGINE_CONTROL=y\n",
		1,
	)
	write_text_if_changed(sitl_board, sitl_board_text)

generated_assets = repo_root / "build/barebones"
generator_args = [
	sys.executable,
	str(repo_root / "tools/generate_vehicle_assets.py"),
	"--vehicle",
	str(vehicle_config),
	"--output",
	str(generated_assets),
]
if flight_profile is not None:
	generator_args.extend(["--flight-profile", str(flight_profile)])
subprocess.run(generator_args, check=True, stdout=subprocess.DEVNULL)

vehicle = yaml.safe_load(vehicle_config.read_text())
gazebo_model_name = vehicle.get("gazebo", {}).get("model_name", vehicle["name"])
source_model = generated_assets / "gazebo" / vehicle["name"]
target_model = worktree / "Tools/simulation/gz/models" / gazebo_model_name
if target_model.exists():
	shutil.rmtree(target_model)
shutil.copytree(source_model, target_model)

# PX4's gz_tv3_rocket target looks up the model by this fixed directory/name.
# Keep manifest-specific outputs too, but mirror the selected vehicle into the
# launcher-compatible path so alternate manifests can use the same airframe.
launcher_model_name = "tv3_rocket"
launcher_model = worktree / "Tools/simulation/gz/models" / launcher_model_name
if gazebo_model_name != launcher_model_name:
	if launcher_model.exists():
		shutil.rmtree(launcher_model)
	shutil.copytree(source_model, launcher_model)
	model_config = launcher_model / "model.config"
	model_sdf = launcher_model / "model.sdf"
	model_config.write_text(model_config.read_text().replace(f"<name>{gazebo_model_name}</name>", f"<name>{launcher_model_name}</name>"))
	model_sdf.write_text(model_sdf.read_text().replace(f'<model name="{gazebo_model_name}">', f'<model name="{launcher_model_name}">'))
PY

if find "${WORKTREE}" -name '*.rej' -not -name 'module.yaml.rej' -print -quit | grep -q .; then
	echo "unexpected patch rejects remain in ${WORKTREE}" >&2
	exit 1
fi

if [ -d "${REPO_ROOT}/overlay/ROMFS/init.d-posix" ]; then
	rsync -a "${REPO_ROOT}/overlay/ROMFS/init.d-posix/" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/" >/dev/null 2>&1
fi

GENERATED_PARAM_FILE=$(find "${REPO_ROOT}/build/barebones/runtime/fs/microsd/tv3/airframes" -name '*.params' -print -quit 2>/dev/null || true)
if [ -n "${GENERATED_PARAM_FILE}" ]; then
	python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_rocket_common.post" <<'PY'
from pathlib import Path
import sys

params = Path(sys.argv[1])
post_path = Path(sys.argv[2])
guidance_enabled = False
for raw_line in params.read_text().splitlines():
	fields = raw_line.split("\t")
	if len(fields) >= 4 and fields[2] == "RK_GD_ENABLE":
		guidance_enabled = float(fields[3]) != 0.0
		break

lines = post_path.read_text().splitlines()
lines = [line for line in lines if line.strip() != "rocket_guidance start"]

if guidance_enabled:
    insert_after = "rocket_mode_manager start"
    try:
        index = lines.index(insert_after) + 1
    except ValueError:
        index = len(lines)
    lines.insert(index, "rocket_guidance start")

post_path.write_text("\n".join(lines) + "\n")
PY

	python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_rocket_common.inc" <<'PY'
from pathlib import Path
import sys

params = Path(sys.argv[1])
target = Path(sys.argv[2])
lines = [
	"param set-default CA_ROTOR_COUNT 0",
	"param set-default CA_SV_CS_COUNT 0",
]
for raw_line in params.read_text().splitlines():
	fields = raw_line.split("\t")
	if len(fields) >= 4:
		lines.append(f"param set-default {fields[2]} {fields[3]}")
target.write_text("\n".join(lines) + "\n")
PY
fi

printf '%s\n' "${WORKTREE}"
