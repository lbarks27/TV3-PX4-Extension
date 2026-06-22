#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
TV3_ROOT=$(cd -- "${REPO_ROOT}/.." && pwd)
VENDOR_DIR=$("${SCRIPT_DIR}/bootstrap_px4.sh")
PX4_REF="${PX4_REF:-v1.16.1}"
WORK_ROOT="${PX4_WORK_ROOT:-${TV3_ROOT}/.work}"
WORKTREE="${WORK_ROOT}/px4-tv3"
VEHICLE_CONFIG="${TV3_VEHICLE_CONFIG:-${REPO_ROOT}/config/vehicles/tv3_v1.json}"
FLIGHT_PROFILE="${TV3_FLIGHT_PROFILE:-}"
if [[ "${VEHICLE_CONFIG}" != /* ]]; then
	VEHICLE_CONFIG="${REPO_ROOT}/${VEHICLE_CONFIG}"
fi
if [ -n "${FLIGHT_PROFILE}" ] && [[ "${FLIGHT_PROFILE}" != /* ]]; then
	FLIGHT_PROFILE="${REPO_ROOT}/${FLIGHT_PROFILE}"
fi

mkdir -p "${WORK_ROOT}"

if [ "${TV3_REUSE_PX4_WORKTREE:-0}" = "1" ] && [ -x "${WORKTREE}/build/px4_sitl_default/bin/px4" ]; then
	python3 - "${REPO_ROOT}" "${WORKTREE}" "${VEHICLE_CONFIG}" "${FLIGHT_PROFILE}" <<'PY'
from pathlib import Path
import subprocess
import sys

repo_root = Path(sys.argv[1])
worktree = Path(sys.argv[2])
vehicle_config = Path(sys.argv[3])
flight_profile = Path(sys.argv[4]) if sys.argv[4] else None
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
PY

	if [ -d "${REPO_ROOT}/overlay/ROMFS/init.d-posix" ]; then
		rsync -a "${REPO_ROOT}/overlay/ROMFS/init.d-posix/" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/" >/dev/null 2>&1
	fi

	GENERATED_PARAM_FILE=$(find "${REPO_ROOT}/build/barebones/runtime/fs/microsd/tv3/airframes" -name '*.params' -print -quit 2>/dev/null || true)
	if [ -n "${GENERATED_PARAM_FILE}" ]; then
		python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.post" <<'PY'
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
lines = [line for line in lines if line.strip() != "tv3_guidance start"]

if guidance_enabled:
    insert_after = "tv3_mode_manager start"
    try:
        index = lines.index(insert_after) + 1
    except ValueError:
        index = len(lines)
    lines.insert(index, "tv3_guidance start")

post_path.write_text("\n".join(lines) + "\n")
PY

		python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.inc" <<'PY'
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
	exit 0
fi

git -C "${VENDOR_DIR}" worktree prune >/dev/null 2>&1 || true
git -C "${VENDOR_DIR}" worktree remove --force "${WORKTREE}" >/dev/null 2>&1 || true
rm -rf "${WORKTREE}"
git -C "${VENDOR_DIR}" worktree add --detach "${WORKTREE}" "${PX4_REF}" >/dev/null 2>&1
git -C "${WORKTREE}" submodule update --init --recursive --jobs 8 -- \
	. \
	':!Tools/simulation/gz' \
	':!Tools/simulation/gazebo-classic/sitl_gazebo-classic' \
	':!Tools/simulation/flightgear/flightgear_bridge' \
	':!Tools/simulation/jmavsim/jMAVSim' \
	':!platforms/nuttx/NuttX/apps' \
	':!platforms/nuttx/NuttX/nuttx' >/dev/null 2>&1

for patch in "${REPO_ROOT}"/patches/px4/*.patch; do
	[ -f "${patch}" ] || continue
	git -C "${WORKTREE}" apply --reject "${patch}" >/dev/null 2>&1 || true
done

python3 - "${REPO_ROOT}" "${WORKTREE}" "${VEHICLE_CONFIG}" "${FLIGHT_PROFILE}" <<'PY'
from pathlib import Path
import subprocess
import sys

repo_root = Path(sys.argv[1])
worktree = Path(sys.argv[2])
vehicle_config = Path(sys.argv[3])
flight_profile = Path(sys.argv[4]) if sys.argv[4] else None
patch = (repo_root / "patches/px4/0001-tv3-control-allocation.patch").read_text()

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

text = text.replace("                15: Spacecraft 3D\n            default: 0\n", "                15: Spacecraft 3D\n                16: TV3\n            default: 0\n", 1)

param_marker = "+        # tv3 TVC\n"
tv3_marker = "+            16: # TV3\n"
module_patch = patch.split("diff --git a/src/modules/control_allocator/module.yaml b/src/modules/control_allocator/module.yaml", 1)[1]
module_patch = module_patch.split("--- a/src/modules/control_allocator/VehicleActuatorEffectiveness/apogee_f10_mass_curve.hpp", 1)[0]
param_block_raw = module_patch.split(param_marker, 1)[1].split(tv3_marker, 1)[0]
tv3_block_raw = module_patch.split(tv3_marker, 1)[1]

def strip_plus(block: str) -> str:
	lines = []
	for line in block.splitlines():
		if line.startswith("+"):
			lines.append(line[1:])
	return "\n".join(lines)

param_block = strip_plus(param_block_raw).rstrip("\n")
tv3_block = strip_plus(tv3_block_raw).rstrip("\n")

if "# tv3 TVC" not in text:
	insert_at = text.index("        # Tilts\n")
	text = text[:insert_at] + param_block + "\n\n" + text[insert_at:]

if "16: # TV3" not in text:
	text = text.rstrip("\n") + "\n\n" + tv3_block + "\n"

module_yaml.write_text(text)

commander = worktree / "src/modules/commander/Commander.cpp"
commander_text = commander.read_text()
if "case 31010:" not in commander_text:
	commander_text = commander_text.replace(
		"\tcase vehicle_command_s::VEHICLE_CMD_REQUEST_CAMERA_INFORMATION:\n"
		"\t\t/* ignore commands that are handled by other parts of the system */\n",
		"\tcase vehicle_command_s::VEHICLE_CMD_REQUEST_CAMERA_INFORMATION:\n"
		"\tcase 31010: // MAV_CMD_USER_1, handled by tv3_mode when enabled\n"
		"\t\t/* ignore commands that are handled by other parts of the system */\n",
		1,
	)
	write_text_if_changed(commander, commander_text)

mode_management = worktree / "src/modules/commander/ModeManagement.cpp"
mode_management_text = mode_management.read_text()
if "RK_ENABLE" not in mode_management_text:
	if '#include <px4_platform_common/events.h>' in mode_management_text:
		mode_management_text = mode_management_text.replace(
			"#include <px4_platform_common/events.h>\n",
			"#include <lib/parameters/param.h>\n"
			"#include <px4_platform_common/events.h>\n",
			1,
		)
	mode_management_text = mode_management_text.replace(
		"\t\t}\n"
		"\t}\n"
		"}\n\n"
		"#endif /* CONSTRAINED_FLASH */",
		"\t\t}\n"
		"\t}\n\n"
		"\t// TV3 uses tv3_mode_manager for launch sequencing. Hide standard PX4 selectable\n"
		"\t// modes from GCS menus when the ascent manager is enabled (RK_ENABLE=1).\n"
		"\tparam_t rk_enable = param_find(\"RK_ENABLE\");\n\n"
		"\tif (rk_enable != PARAM_INVALID) {\n"
		"\t\tint32_t enabled = 0;\n\n"
		"\t\tif (param_get(rk_enable, &enabled) == 0 && enabled > 0) {\n"
		"\t\t\tvalid_nav_state_mask = (1u << vehicle_status_s::NAVIGATION_STATE_MANUAL);\n"
		"\t\t\tcan_set_nav_state_mask = 0;\n"
		"\t\t}\n"
		"\t}\n"
		"}\n\n"
		"#endif /* CONSTRAINED_FLASH */",
		1,
	)
	write_text_if_changed(mode_management, mode_management_text)

control_allocator_hpp = worktree / "src/modules/control_allocator/ControlAllocator.hpp"
control_allocator_hpp_text = control_allocator_hpp.read_text()
# Note: ActuatorEffectivenessTV3 is still instantiated by the allocator for CA_RK param handling
# and control_allocator_status, but tv3_mode_manager bypasses its servo outputs and runs the
# joint projected-GD solver instead.
if "#include <ActuatorEffectivenessTV3.hpp>" not in control_allocator_hpp_text:
	control_allocator_hpp_text = control_allocator_hpp_text.replace(
		"#include <ActuatorEffectivenessHelicopterCoaxial.hpp>\n",
		"#include <ActuatorEffectivenessHelicopterCoaxial.hpp>\n"
		"#include <ActuatorEffectivenessTV3.hpp>\n",
		1,
	)
if "\t\tTV3 = 16," not in control_allocator_hpp_text:
	control_allocator_hpp_text = control_allocator_hpp_text.replace(
		"\t\tSPACECRAFT_3D = 14,\n",
		"\t\tSPACECRAFT_3D = 14,\n"
		"\t\tTV3 = 16,\n",
		1,
	)
write_text_if_changed(control_allocator_hpp, control_allocator_hpp_text)

control_allocator_cpp = worktree / "src/modules/control_allocator/ControlAllocator.cpp"
control_allocator_cpp_text = control_allocator_cpp.read_text()
if "EffectivenessSource::TV3" not in control_allocator_cpp_text:
	control_allocator_cpp_text = control_allocator_cpp_text.replace(
		"\t\tcase EffectivenessSource::SPACECRAFT_3D:\n"
		"\t\t\t// spacecraft_allocation does allocation and publishes directly to actuator_motors topic\n"
		"\t\t\tbreak;\n\n"
		"\t\tdefault:\n",
		"\t\tcase EffectivenessSource::SPACECRAFT_3D:\n"
		"\t\t\t// spacecraft_allocation does allocation and publishes directly to actuator_motors topic\n"
		"\t\t\tbreak;\n\n"
		"\t\tcase EffectivenessSource::TV3:\n"
		"\t\t\ttmp = new ActuatorEffectivenessTV3(this);\n"
		"\t\t\tbreak;\n\n"
		"\t\tdefault:\n",
		1,
	)
	write_text_if_changed(control_allocator_cpp, control_allocator_cpp_text)

vehicle_effectiveness = worktree / "src/modules/control_allocator/VehicleActuatorEffectiveness"
for relative_path in (
	"src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTV3.hpp",
	"src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTV3.cpp",
):
	write_text_if_changed(worktree / relative_path, extract_added_file(patch, relative_path))

vehicle_effectiveness_cmake = vehicle_effectiveness / "CMakeLists.txt"
vehicle_effectiveness_cmake_text = vehicle_effectiveness_cmake.read_text()
if "ActuatorEffectivenessTV3.cpp" not in vehicle_effectiveness_cmake_text:
	vehicle_effectiveness_cmake_text = vehicle_effectiveness_cmake_text.replace(
		"\tActuatorEffectivenessRoverAckermann.hpp\n"
		"\tActuatorEffectivenessRoverAckermann.cpp\n",
		"\tActuatorEffectivenessRoverAckermann.hpp\n"
		"\tActuatorEffectivenessRoverAckermann.cpp\n"
		"\tActuatorEffectivenessTV3.hpp\n"
		"\tActuatorEffectivenessTV3.cpp\n",
		1,
	)
	write_text_if_changed(vehicle_effectiveness_cmake, vehicle_effectiveness_cmake_text)

cmake_lists = worktree / "CMakeLists.txt"
cmake_text = cmake_lists.read_text()
cmake_text = cmake_text.replace("set(CMAKE_CXX_STANDARD 14)", "set(CMAKE_CXX_STANDARD 17)", 1)
cmake_lists.write_text(cmake_text)

common_flags = worktree / "cmake/px4_add_common_flags.cmake"
common_flags_text = common_flags.read_text()
common_flags_text = common_flags_text.replace("-Wdouble-promotion", "-Wno-double-promotion")
common_flags_text = common_flags_text.replace("-Werror", "-Wno-error")
common_flags.write_text(common_flags_text)

sitl_board = worktree / "boards/px4/sitl/default.px4board"
sitl_board_text = sitl_board.read_text()
for disabled_module in (
	"CONFIG_MODULES_SIMULATION_GZ_MSGS=y\n",
	"CONFIG_MODULES_SIMULATION_GZ_BRIDGE=y\n",
	"CONFIG_MODULES_SIMULATION_GZ_PLUGINS=y\n",
):
	sitl_board_text = sitl_board_text.replace(disabled_module, "")
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
	python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.post" <<'PY'
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
lines = [line for line in lines if line.strip() != "tv3_guidance start"]

if guidance_enabled:
    insert_after = "tv3_mode_manager start"
    try:
        index = lines.index(insert_after) + 1
    except ValueError:
        index = len(lines)
    lines.insert(index, "tv3_guidance start")

post_path.write_text("\n".join(lines) + "\n")
PY

	python3 - "${GENERATED_PARAM_FILE}" "${WORKTREE}/ROMFS/px4fmu_common/init.d-posix/airframes/tv3_common.inc" <<'PY'
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
