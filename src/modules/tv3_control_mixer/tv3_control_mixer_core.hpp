#pragma once

#include "tv3_control_mixer_lm.hpp"
#include "tv3_control_mixer_plant.hpp"

#include <matrix/matrix/math.hpp>

namespace tv3
{

using matrix::Vector3f;

struct ControlMixerGeometry {
	int engine_count{1};
	Vector3f body_com{};
	Vector3f group_pos[kControlMixerMaxEngines]{};
	Vector3f group_thrust[kControlMixerMaxEngines]{};
	Vector3f group_primary[kControlMixerMaxEngines]{};
	Vector3f group_secondary[kControlMixerMaxEngines]{};
	float group_pmax_rad[kControlMixerMaxEngines]{};
	float group_ymin_rad[kControlMixerMaxEngines]{};
	float group_ymax_rad[kControlMixerMaxEngines]{};
};

struct ControlMixerLimits {
	float tvc_max_deg{5.f};
	float boost_tvc_limit_deg{8.f};
	bool boost_limits{false};
};

struct ControlMixerSolveInput {
	Vector3f torque_nm{};
	float thrust_n[kControlMixerMaxEngines]{};
	int ignition_mask{0};
};

struct ControlMixerSolveOutput {
	float primary_rad[kControlMixerMaxEngines]{};
	float yaw_rad[kControlMixerMaxEngines]{};
	bool converged{false};
	bool used_fallback{false};
};

struct ControlMixerLmTuning {
	int max_iter{12};
	float tol_nm{0.15f};
	float lambda0{0.01f};
	float fd_eps{0.01f};
};

class ControlMixerCore {
public:
	void set_geometry(const ControlMixerGeometry &geometry) { _geometry = geometry; }

	void set_lm_tuning(const ControlMixerLmTuning &tuning) { _tuning = tuning; }

	ControlMixerSolveOutput solve(const ControlMixerSolveInput &input, const ControlMixerLimits &limits,
			       const float initial_primary_rad[kControlMixerMaxEngines],
			       const float initial_yaw_rad[kControlMixerMaxEngines],
			       bool warm_start_valid) const;

private:
	ControlMixerPlant build_plant() const;

	ControlMixerAngleLimits build_angle_limits(const ControlMixerLimits &limits) const;

	ControlMixerGeometry _geometry{};
	ControlMixerLmTuning _tuning{};
};

} // namespace tv3
