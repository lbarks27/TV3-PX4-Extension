#pragma once

#include "tv3_gimbal_plant.hpp"

namespace tv3
{

constexpr int kGimbalMaxDof = kGimbalMaxEngines * 2;
constexpr int kGimbalMaxResidual = 4 + kGimbalMaxEngines;

struct GimbalLimits {
	float primary_min_rad[kGimbalMaxEngines]{};
	float primary_max_rad[kGimbalMaxEngines]{};
	float yaw_min_rad[kGimbalMaxEngines]{};
	float yaw_max_rad[kGimbalMaxEngines]{};
};

struct LmConfig {
	int max_iter{12};
	float torque_tol_nm{0.15f};
	float lambda0{1e-2f};
	float thrust_weight{0.02f};
	float splay_weight{0.1f};
	float fd_eps{0.01f};
};

struct LmSolveResult {
	float primary_rad[kGimbalMaxEngines]{};
	float yaw_rad[kGimbalMaxEngines]{};
	float residual_torque_nm{0.f};
	float residual_thrust_n{0.f};
	float cost{0.f};
	float lambda_final{0.f};
	int iterations_used{0};
	bool converged{false};
};

LmSolveResult solve_gimbal_lm(const GimbalPlant &plant,
			      const float thrust_n[kGimbalMaxEngines],
			      int ignition_mask,
			      const GimbalWrench &desired,
			      const float initial_primary_rad[kGimbalMaxEngines],
			      const float initial_yaw_rad[kGimbalMaxEngines],
			      const GimbalLimits &limits,
			      const LmConfig &config);

} // namespace tv3
