#pragma once

#include "tv3_gimbal_lm.hpp"

#include <matrix/matrix/math.hpp>

namespace tv3
{

struct AchievableTorque {
	matrix::Vector3f positive_nm{};
	matrix::Vector3f negative_nm{};
};

struct TorqueScaleResult {
	matrix::Vector3f scaled_nm{};
	float scale{1.f};
	bool saturated{false};
};

AchievableTorque estimate_achievable_torque(const GimbalPlant &plant,
		const float thrust_n[kGimbalMaxEngines],
		int ignition_mask,
		const GimbalLimits &limits);

TorqueScaleResult scale_torque_preserve_direction(const matrix::Vector3f &demand_nm,
		const AchievableTorque &achievable);

bool torque_wrench_aligned(const matrix::Vector3f &desired_nm,
			   const matrix::Vector3f &achieved_nm,
			   float min_demand_nm = 0.05f);

} // namespace tv3