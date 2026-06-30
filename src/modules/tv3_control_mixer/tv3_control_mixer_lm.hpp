#pragma once

#include "tv3_control_mixer_plant.hpp"

namespace tv3
{

constexpr int kControlMixerMaxDof = kControlMixerMaxEngines * 2;
constexpr int kControlMixerMaxResidual = 3;

struct ControlMixerAngleLimits {
	float primary_min_rad[kControlMixerMaxEngines]{};
	float primary_max_rad[kControlMixerMaxEngines]{};
	float yaw_min_rad[kControlMixerMaxEngines]{};
	float yaw_max_rad[kControlMixerMaxEngines]{};
};

struct ControlMixerLmConfig {
	int max_iter{12};
	float torque_tol_nm{0.15f};
	float lambda0{1e-2f};
	float fd_eps{0.01f};
};

struct ControlMixerLmResult {
	float primary_rad[kControlMixerMaxEngines]{};
	float yaw_rad[kControlMixerMaxEngines]{};
	float residual_torque_nm{0.f};
	float cost{0.f};
	float lambda_final{0.f};
	int iterations_used{0};
	bool converged{false};
};

ControlMixerLmResult solve_control_mixer_lm(const ControlMixerPlant &plant,
		const float thrust_n[kControlMixerMaxEngines],
		int ignition_mask,
		const ControlMixerWrench &desired,
		const float initial_primary_rad[kControlMixerMaxEngines],
		const float initial_yaw_rad[kControlMixerMaxEngines],
		const ControlMixerAngleLimits &limits,
		const ControlMixerLmConfig &config);

bool torque_wrench_aligned(const matrix::Vector3f &desired_nm,
			   const matrix::Vector3f &achieved_nm,
			   float min_demand_nm = 0.05f);

} // namespace tv3
