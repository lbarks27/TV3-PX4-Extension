#include "tv3_gimbal_authority.hpp"

#include <mathlib/mathlib.h>

namespace tv3
{

namespace
{

void update_axis_bounds(const matrix::Vector3f &torque_nm, AchievableTorque &achievable)
{
	for (int axis = 0; axis < 3; ++axis) {
		achievable.positive_nm(axis) = math::max(achievable.positive_nm(axis), torque_nm(axis));
		achievable.negative_nm(axis) = math::max(achievable.negative_nm(axis), -torque_nm(axis));
	}
}

void sample_wrench(const GimbalPlant &plant,
		   const float primary_rad[kGimbalMaxEngines],
		   const float yaw_rad[kGimbalMaxEngines],
		   const float thrust_n[kGimbalMaxEngines],
		   int ignition_mask,
		   AchievableTorque &achievable)
{
	const GimbalWrenchResult wrench = plant.total_wrench(primary_rad, yaw_rad, thrust_n, ignition_mask);
	update_axis_bounds(wrench.torque_nm, achievable);
}

bool engine_active(int engine, int ignition_mask, float thrust_n)
{
	return engine >= 0
	       && engine < kGimbalMaxEngines
	       && (ignition_mask & (1 << engine)) != 0
	       && thrust_n > 0.5f;
}

} // namespace

AchievableTorque estimate_achievable_torque(const GimbalPlant &plant,
		const float thrust_n[kGimbalMaxEngines],
		int ignition_mask,
		const GimbalLimits &limits)
{
	AchievableTorque achievable{};
	float primary_rad[kGimbalMaxEngines]{};
	float yaw_rad[kGimbalMaxEngines]{};

	sample_wrench(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, achievable);

	const int count = math::min(plant.engine_count, kGimbalMaxEngines);

	for (int i = 0; i < count; ++i) {
		if (!engine_active(i, ignition_mask, thrust_n[i])) {
			continue;
		}

		for (int corner = 0; corner < 2; ++corner) {
			const float primary_limit = corner == 0 ? limits.primary_min_rad[i] : limits.primary_max_rad[i];
			primary_rad[i] = primary_limit;
			yaw_rad[i] = 0.f;
			sample_wrench(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, achievable);
			primary_rad[i] = 0.f;
		}

		for (int corner = 0; corner < 2; ++corner) {
			const float yaw_limit = corner == 0 ? limits.yaw_min_rad[i] : limits.yaw_max_rad[i];
			yaw_rad[i] = yaw_limit;
			primary_rad[i] = 0.f;
			sample_wrench(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, achievable);
			yaw_rad[i] = 0.f;
		}

		for (int p_corner = 0; p_corner < 2; ++p_corner) {
			for (int y_corner = 0; y_corner < 2; ++y_corner) {
				primary_rad[i] = p_corner == 0 ? limits.primary_min_rad[i] : limits.primary_max_rad[i];
				yaw_rad[i] = y_corner == 0 ? limits.yaw_min_rad[i] : limits.yaw_max_rad[i];
				sample_wrench(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, achievable);
			}
		}

		primary_rad[i] = 0.f;
		yaw_rad[i] = 0.f;
	}

	for (int i = 0; i < count; ++i) {
		if (!engine_active(i, ignition_mask, thrust_n[i])) {
			continue;
		}

		for (int j = i + 1; j < count; ++j) {
			if (!engine_active(j, ignition_mask, thrust_n[j])) {
				continue;
			}

			primary_rad[i] = limits.primary_max_rad[i];
			primary_rad[j] = limits.primary_min_rad[j];
			sample_wrench(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, achievable);

			primary_rad[i] = limits.primary_min_rad[i];
			primary_rad[j] = limits.primary_max_rad[j];
			sample_wrench(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, achievable);

			primary_rad[i] = 0.f;
			primary_rad[j] = 0.f;
		}
	}

	return achievable;
}

TorqueScaleResult scale_torque_preserve_direction(const matrix::Vector3f &demand_nm,
		const AchievableTorque &achievable)
{
	TorqueScaleResult result{};
	result.scaled_nm.zero();
	result.scale = 1.f;

	float min_axis_scale = 1.f;

	for (int axis = 0; axis < 3; ++axis) {
		const float demand = demand_nm(axis);

		if (fabsf(demand) < 1e-4f) {
			continue;
		}

		const float limit = demand >= 0.f ? achievable.positive_nm(axis) : achievable.negative_nm(axis);

		if (limit <= 1e-4f) {
			result.scaled_nm(axis) = 0.f;
			min_axis_scale = 0.f;
			result.saturated = true;
			continue;
		}

		if (fabsf(demand) > limit) {
			result.scaled_nm(axis) = copysignf(limit, demand);
			min_axis_scale = math::min(min_axis_scale, limit / fabsf(demand));
			result.saturated = true;
		} else {
			result.scaled_nm(axis) = demand;
			min_axis_scale = math::min(min_axis_scale, 1.f);
		}
	}

	result.scale = math::constrain(min_axis_scale, 0.f, 1.f);
	return result;
}

bool torque_wrench_aligned(const matrix::Vector3f &desired_nm,
			   const matrix::Vector3f &achieved_nm,
			   float min_demand_nm)
{
	const float desired_norm = desired_nm.norm();
	const float achieved_norm = achieved_nm.norm();

	if (desired_norm < min_demand_nm) {
		return true;
	}

	if (achieved_norm < min_demand_nm) {
		return false;
	}

	if (desired_nm.dot(achieved_nm) < 0.f) {
		return false;
	}

	for (int axis = 0; axis < 3; ++axis) {
		const float demand = desired_nm(axis);
		const float achieved = achieved_nm(axis);

		if (fabsf(demand) < min_demand_nm) {
			continue;
		}

		if (fabsf(achieved) < min_demand_nm) {
			return false;
		}

		if (demand * achieved < 0.f) {
			return false;
		}
	}

	return true;
}

} // namespace tv3