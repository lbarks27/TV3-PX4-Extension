#pragma once

#include <cstdint>

namespace tv3
{

enum class AttitudeMode : uint8_t {
	Off = 0,
	LargeError = 1,
	SmallError = 2,
	Deadband = 3,
};

enum class GuidanceMode : uint8_t {
	Off = 0,
	Up = 1,
	WaypointFlyThrough = 2,
};

enum class MixerMode : uint8_t {
	Off = 0,
	TorqueOnly = 1,
	TorqueAndThrust = 2,
};

enum class LoadCellMode : uint8_t {
	Off = 0,
	Monitor = 1,
};

constexpr int kMaxControlPhases = 8;

struct ControlPhaseConfig {
	uint8_t on_tv3_mode{0};
	uint8_t guidance_mode{0};
	uint8_t attitude_mode{0}; // ATTITUDE_OFF or ATTITUDE_ON from tv3_sm_modes
	uint8_t mixer_mode{0};
	uint8_t load_cell_mode{0};
};

} // namespace tv3
