#pragma once

#include <matrix/matrix/math.hpp>

namespace tv3
{

constexpr int kGimbalMaxEngines = 4;

struct GimbalPlantGeometry {
	matrix::Vector3f pos[kGimbalMaxEngines]{};
	matrix::Vector3f thrust_axis[kGimbalMaxEngines]{};
	matrix::Vector3f primary_axis[kGimbalMaxEngines]{};
	matrix::Vector3f secondary_axis[kGimbalMaxEngines]{};
};

struct GimbalWrench {
	matrix::Vector3f torque_nm{};
	float axial_thrust_n{0.f};
};

struct GimbalWrenchResult {
	matrix::Vector3f torque_nm{};
	matrix::Vector3f body_force_n{};
	float axial_thrust_n{0.f};
};

class GimbalPlant
{
public:
	GimbalPlantGeometry geometry{};
	int engine_count{0};

	matrix::Vector3f thrust_direction(int engine, float primary_rad, float yaw_rad) const;

	matrix::Vector3f engine_torque(int engine, float primary_rad, float yaw_rad, float thrust_n) const;

	float engine_axial_thrust(int engine, float primary_rad, float yaw_rad, float thrust_n) const;

	GimbalWrenchResult total_wrench(const float primary_rad[kGimbalMaxEngines],
					const float yaw_rad[kGimbalMaxEngines],
					const float thrust_n[kGimbalMaxEngines],
					int ignition_mask) const;
};

} // namespace tv3
