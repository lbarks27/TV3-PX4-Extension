#include "tv3_control_mixer_core.hpp"


#include <mathlib/mathlib.h>

namespace tv3
{

ControlMixerPlant ControlMixerCore::build_plant() const
{
	ControlMixerPlant plant{};
	plant.engine_count = _geometry.engine_count;
	plant.geometry.body_com = _geometry.body_com;

	for (int i = 0; i < _geometry.engine_count; ++i) {
		plant.geometry.pos[i] = _geometry.group_pos[i];
		plant.geometry.thrust_axis[i] = _geometry.group_thrust[i];
		plant.geometry.primary_axis[i] = _geometry.group_primary[i];
		plant.geometry.secondary_axis[i] = _geometry.group_secondary[i];
	}

	return plant;
}

ControlMixerAngleLimits ControlMixerCore::build_angle_limits(const ControlMixerLimits &limits) const
{
	ControlMixerAngleLimits out{};
	const float tvc_max_rad = math::radians(math::max(limits.tvc_max_deg, 0.f));
	const float boost_tvc_rad = math::radians(limits.boost_tvc_limit_deg);

	for (int i = 0; i < _geometry.engine_count; ++i) {
		if (limits.boost_limits) {
			const float lim = (tvc_max_rad > 0.f) ? fminf(tvc_max_rad, boost_tvc_rad) : boost_tvc_rad;
			out.primary_min_rad[i] = -lim;
			out.primary_max_rad[i] = lim;
			out.yaw_min_rad[i] = -lim;
			out.yaw_max_rad[i] = lim;
		} else {
			out.primary_min_rad[i] = -_geometry.group_pmax_rad[i];
			out.primary_max_rad[i] = _geometry.group_pmax_rad[i];
			out.yaw_min_rad[i] = _geometry.group_ymin_rad[i];
			out.yaw_max_rad[i] = _geometry.group_ymax_rad[i];
		}
	}

	return out;
}

ControlMixerSolveOutput ControlMixerCore::solve(const ControlMixerSolveInput &input, const ControlMixerLimits &limits,
				  const float initial_primary_rad[kControlMixerMaxEngines],
				  const float initial_yaw_rad[kControlMixerMaxEngines],
				  bool warm_start_valid) const
{
	ControlMixerSolveOutput output{};
	const ControlMixerPlant plant = build_plant();
	const ControlMixerAngleLimits gimbal_limits = build_angle_limits(limits);
	const int engine_count = _geometry.engine_count;

	ControlMixerWrench desired{};
	desired.torque_nm = input.torque_nm;

	float init_p[kControlMixerMaxEngines]{};
	float init_y[kControlMixerMaxEngines]{};

	for (int i = 0; i < engine_count; ++i) {
		init_p[i] = warm_start_valid ? initial_primary_rad[i] : 0.f;
		init_y[i] = warm_start_valid ? initial_yaw_rad[i] : 0.f;
	}

	const float torque_demand_nm = input.torque_nm.length();
	constexpr float kNeutralTorqueNm = 0.05f;

	ControlMixerLmConfig lm_config{};
	lm_config.max_iter = _tuning.max_iter;
	lm_config.torque_tol_nm = _tuning.tol_nm;
	lm_config.lambda0 = _tuning.lambda0;
	lm_config.fd_eps = _tuning.fd_eps;

	ControlMixerLmResult lm_result{};

	if (torque_demand_nm < kNeutralTorqueNm) {
		lm_result.converged = true;
	} else {
		lm_result = solve_control_mixer_lm(plant, input.thrust_n, input.ignition_mask, desired,
					      init_p, init_y, gimbal_limits, lm_config);
	}

	bool solution_accepted = false;

	if (lm_result.converged && torque_demand_nm < kNeutralTorqueNm) {
		solution_accepted = true;
	} else if (lm_result.converged || lm_result.residual_torque_nm < torque_demand_nm) {
		for (int i = 0; i < engine_count; ++i) {
			output.primary_rad[i] = lm_result.primary_rad[i];
			output.yaw_rad[i] = lm_result.yaw_rad[i];
		}

		const ControlMixerWrenchResult achieved = plant.total_wrench(output.primary_rad, output.yaw_rad,
				input.thrust_n, input.ignition_mask);
		solution_accepted = lm_result.converged
				    || torque_wrench_aligned(desired.torque_nm, achieved.torque_nm)
				    || achieved.torque_nm.norm() > kNeutralTorqueNm;
	}

	if (!solution_accepted) {
		output.used_fallback = true;

		for (int i = 0; i < engine_count; ++i) {
			output.primary_rad[i] = warm_start_valid ? initial_primary_rad[i] : init_p[i];
			output.yaw_rad[i] = warm_start_valid ? initial_yaw_rad[i] : init_y[i];
			output.primary_rad[i] = math::constrain(output.primary_rad[i], gimbal_limits.primary_min_rad[i],
								gimbal_limits.primary_max_rad[i]);
			output.yaw_rad[i] = math::constrain(output.yaw_rad[i], gimbal_limits.yaw_min_rad[i],
							    gimbal_limits.yaw_max_rad[i]);
		}
	}

	output.converged = solution_accepted;
	return output;
}

} // namespace tv3
