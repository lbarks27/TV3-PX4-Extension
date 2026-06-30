#pragma once

#include <matrix/matrix/math.hpp>

namespace tv3
{

constexpr int kControlMixerMaxEngines = 4;

struct ControlMixerPlantGeometry {
	matrix::Vector3f body_com{};
	matrix::Vector3f pos[kControlMixerMaxEngines]{};
	matrix::Vector3f thrust_axis[kControlMixerMaxEngines]{};
	matrix::Vector3f primary_axis[kControlMixerMaxEngines]{};
	matrix::Vector3f secondary_axis[kControlMixerMaxEngines]{};
};

struct ControlMixerWrench {
	matrix::Vector3f torque_nm{};
	float axial_thrust_n{0.f};
};

struct ControlMixerWrenchResult {
	matrix::Vector3f torque_nm{};
	matrix::Vector3f body_force_n{};
	float axial_thrust_n{0.f};
};

class ControlMixerPlant
{
public:
	ControlMixerPlantGeometry geometry{};
	int engine_count{0};

	matrix::Vector3f thrust_direction(int engine, float primary_rad, float yaw_rad) const;

	matrix::Vector3f engine_torque(int engine, float primary_rad, float yaw_rad, float thrust_n) const;

	float engine_axial_thrust(int engine, float primary_rad, float yaw_rad, float thrust_n) const;

	ControlMixerWrenchResult total_wrench(const float primary_rad[kControlMixerMaxEngines],
					      const float yaw_rad[kControlMixerMaxEngines],
					      const float thrust_n[kControlMixerMaxEngines],
					      int ignition_mask) const;
};

} // namespace tv3
