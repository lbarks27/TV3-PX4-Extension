#include "tv3_control_mixer_lm.hpp"

#include <mathlib/mathlib.h>

namespace tv3
{

namespace
{

bool dof_active(int dof_index, int engine_count, int ignition_mask, const float thrust_n[kControlMixerMaxEngines])
{
	const int engine = dof_index / 2;

	if (engine < 0 || engine >= engine_count || engine >= kControlMixerMaxEngines) {
		return false;
	}

	return (ignition_mask & (1 << engine)) != 0 && thrust_n[engine] > 0.5f;
}

void clip_angles(float primary_rad[kControlMixerMaxEngines],
		 float yaw_rad[kControlMixerMaxEngines],
		 int engine_count,
		 const ControlMixerAngleLimits &limits)
{
	for (int i = 0; i < engine_count && i < kControlMixerMaxEngines; ++i) {
		primary_rad[i] = math::constrain(primary_rad[i], limits.primary_min_rad[i], limits.primary_max_rad[i]);
		yaw_rad[i] = math::constrain(yaw_rad[i], limits.yaw_min_rad[i], limits.yaw_max_rad[i]);
	}
}

void state_to_angles(const float state[kControlMixerMaxDof],
		     int engine_count,
		     float primary_rad[kControlMixerMaxEngines],
		     float yaw_rad[kControlMixerMaxEngines])
{
	for (int i = 0; i < engine_count && i < kControlMixerMaxEngines; ++i) {
		primary_rad[i] = state[2 * i];
		yaw_rad[i] = state[2 * i + 1];
	}
}

void angles_to_state(const float primary_rad[kControlMixerMaxEngines],
		     const float yaw_rad[kControlMixerMaxEngines],
		     int engine_count,
		     float state[kControlMixerMaxDof])
{
	for (int i = 0; i < engine_count && i < kControlMixerMaxEngines; ++i) {
		state[2 * i] = primary_rad[i];
		state[2 * i + 1] = yaw_rad[i];
	}
}

int count_active_engines(int engine_count, int ignition_mask, const float thrust_n[kControlMixerMaxEngines])
{
	int active = 0;

	for (int i = 0; i < engine_count && i < kControlMixerMaxEngines; ++i) {
		if ((ignition_mask & (1 << i)) != 0 && thrust_n[i] > 0.5f) {
			active++;
		}
	}

	return active;
}

struct ResidualEvaluation {
	float residual[kControlMixerMaxResidual]{};
	int residual_count{3};
	float cost{0.f};
	float torque_error_nm{0.f};
};

ResidualEvaluation evaluate_residual(const ControlMixerPlant &plant,
				     const float primary_rad[kControlMixerMaxEngines],
				     const float yaw_rad[kControlMixerMaxEngines],
				     const float thrust_n[kControlMixerMaxEngines],
				     int ignition_mask,
				     const ControlMixerWrench &desired)
{
	ResidualEvaluation eval{};
	const ControlMixerWrenchResult achieved = plant.total_wrench(primary_rad, yaw_rad, thrust_n, ignition_mask);
	const matrix::Vector3f torque_error = desired.torque_nm - achieved.torque_nm;
	eval.torque_error_nm = torque_error.norm();

	eval.residual[0] = torque_error(0);
	eval.residual[1] = torque_error(1);
	eval.residual[2] = torque_error(2);

	for (int i = 0; i < eval.residual_count; ++i) {
		eval.cost += eval.residual[i] * eval.residual[i];
	}

	eval.cost *= 0.5f;
	return eval;
}

bool solve_linear_system(float a[kControlMixerMaxDof][kControlMixerMaxDof], float b[kControlMixerMaxDof], int n)
{
	for (int col = 0; col < n; ++col) {
		int pivot = col;
		float pivot_abs = fabsf(a[col][col]);

		for (int row = col + 1; row < n; ++row) {
			const float candidate = fabsf(a[row][col]);

			if (candidate > pivot_abs) {
				pivot_abs = candidate;
				pivot = row;
			}
		}

		if (pivot_abs < 1e-9f) {
			return false;
		}

		if (pivot != col) {
			for (int k = col; k < n; ++k) {
				const float tmp = a[col][k];
				a[col][k] = a[pivot][k];
				a[pivot][k] = tmp;
			}

			const float tmp_b = b[col];
			b[col] = b[pivot];
			b[pivot] = tmp_b;
		}

		const float inv_pivot = 1.f / a[col][col];

		for (int row = col + 1; row < n; ++row) {
			const float factor = a[row][col] * inv_pivot;

			for (int k = col; k < n; ++k) {
				a[row][k] -= factor * a[col][k];
			}

			b[row] -= factor * b[col];
		}
	}

	for (int row = n - 1; row >= 0; --row) {
		float sum = b[row];

		for (int col = row + 1; col < n; ++col) {
			sum -= a[row][col] * b[col];
		}

		if (fabsf(a[row][row]) < 1e-9f) {
			return false;
		}

		b[row] = sum / a[row][row];
	}

	return true;
}

bool compute_lm_step(const ControlMixerPlant &plant,
		     const float state[kControlMixerMaxDof],
		     const float thrust_n[kControlMixerMaxEngines],
		     int ignition_mask,
		     const ControlMixerWrench &desired,
		     const ControlMixerLmConfig &config,
		     float lambda,
		     float delta[kControlMixerMaxDof])
{
	const int dof_count = math::min(plant.engine_count, kControlMixerMaxEngines) * 2;

	float primary_rad[kControlMixerMaxEngines]{};
	float yaw_rad[kControlMixerMaxEngines]{};
	state_to_angles(state, plant.engine_count, primary_rad, yaw_rad);

	const ResidualEvaluation base = evaluate_residual(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, desired);

	float jacobian[kControlMixerMaxResidual][kControlMixerMaxDof] {};

	for (int dof = 0; dof < dof_count; ++dof) {
		if (!dof_active(dof, plant.engine_count, ignition_mask, thrust_n)) {
			continue;
		}

		const float saved = state[dof];
		float perturbed_primary[kControlMixerMaxEngines]{};
		float perturbed_yaw[kControlMixerMaxEngines]{};
		state_to_angles(state, plant.engine_count, perturbed_primary, perturbed_yaw);

		if (dof % 2 == 0) {
			perturbed_primary[dof / 2] = saved + config.fd_eps;
		} else {
			perturbed_yaw[dof / 2] = saved + config.fd_eps;
		}

		const ResidualEvaluation plus = evaluate_residual(plant, perturbed_primary, perturbed_yaw, thrust_n,
				ignition_mask, desired);

		if (dof % 2 == 0) {
			perturbed_primary[dof / 2] = saved - config.fd_eps;
		} else {
			perturbed_yaw[dof / 2] = saved - config.fd_eps;
		}

		const ResidualEvaluation minus = evaluate_residual(plant, perturbed_primary, perturbed_yaw, thrust_n,
				ignition_mask, desired);

		const float inv_2eps = 1.f / (2.f * config.fd_eps);

		for (int row = 0; row < base.residual_count; ++row) {
			jacobian[row][dof] = (plus.residual[row] - minus.residual[row]) * inv_2eps;
		}
	}

	float normal[kControlMixerMaxDof][kControlMixerMaxDof] {};
	float gradient[kControlMixerMaxDof] {};

	for (int dof = 0; dof < dof_count; ++dof) {
		for (int other = 0; other < dof_count; ++other) {
			float sum = 0.f;

			for (int row = 0; row < base.residual_count; ++row) {
				sum += jacobian[row][dof] * jacobian[row][other];
			}

			normal[dof][other] = sum;
		}

		normal[dof][dof] += lambda;

		float sum = 0.f;

		for (int row = 0; row < base.residual_count; ++row) {
			sum += jacobian[row][dof] * base.residual[row];
		}

		gradient[dof] = -sum;
	}

	for (int dof = 0; dof < dof_count; ++dof) {
		delta[dof] = gradient[dof];
	}

	if (!solve_linear_system(normal, delta, dof_count)) {
		return false;
	}

	return true;
}

bool converged(const ResidualEvaluation &eval, const ControlMixerLmConfig &config)
{
	return eval.torque_error_nm < config.torque_tol_nm;
}

} // namespace

ControlMixerLmResult solve_control_mixer_lm(const ControlMixerPlant &plant,
			      const float thrust_n[kControlMixerMaxEngines],
			      int ignition_mask,
			      const ControlMixerWrench &desired,
			      const float initial_primary_rad[kControlMixerMaxEngines],
			      const float initial_yaw_rad[kControlMixerMaxEngines],
			      const ControlMixerAngleLimits &limits,
			      const ControlMixerLmConfig &config)
{
	ControlMixerLmResult result{};

	if (plant.engine_count <= 0 || count_active_engines(plant.engine_count, ignition_mask, thrust_n) == 0) {
		for (int i = 0; i < kControlMixerMaxEngines; ++i) {
			result.primary_rad[i] = initial_primary_rad[i];
			result.yaw_rad[i] = initial_yaw_rad[i];
		}

		return result;
	}

	float primary_rad[kControlMixerMaxEngines]{};
	float yaw_rad[kControlMixerMaxEngines]{};

	for (int i = 0; i < kControlMixerMaxEngines; ++i) {
		primary_rad[i] = initial_primary_rad[i];
		yaw_rad[i] = initial_yaw_rad[i];
	}

	clip_angles(primary_rad, yaw_rad, plant.engine_count, limits);

	float state[kControlMixerMaxDof]{};
	angles_to_state(primary_rad, yaw_rad, plant.engine_count, state);

	float lambda = math::max(config.lambda0, 1e-6f);
	ResidualEvaluation eval = evaluate_residual(plant, primary_rad, yaw_rad, thrust_n, ignition_mask, desired);
	result.cost = eval.cost;
	result.residual_torque_nm = eval.torque_error_nm;

	if (converged(eval, config)) {
		result.converged = true;
		result.lambda_final = lambda;

		for (int i = 0; i < kControlMixerMaxEngines; ++i) {
			result.primary_rad[i] = primary_rad[i];
			result.yaw_rad[i] = yaw_rad[i];
		}

		return result;
	}

	const int max_iter = math::max(config.max_iter, 1);

	for (int iter = 0; iter < max_iter; ++iter) {
		result.iterations_used = iter + 1;
		float delta[kControlMixerMaxDof] {};
		const bool step_ok = compute_lm_step(plant, state, thrust_n, ignition_mask, desired, config, lambda, delta);

		if (!step_ok) {
			lambda = math::min(lambda * 10.f, 1e6f);
			continue;
		}

		float trial_state[kControlMixerMaxDof]{};

		for (int dof = 0; dof < kControlMixerMaxDof; ++dof) {
			trial_state[dof] = state[dof] + delta[dof];
		}

		float trial_primary[kControlMixerMaxEngines]{};
		float trial_yaw[kControlMixerMaxEngines]{};
		state_to_angles(trial_state, plant.engine_count, trial_primary, trial_yaw);
		clip_angles(trial_primary, trial_yaw, plant.engine_count, limits);
		angles_to_state(trial_primary, trial_yaw, plant.engine_count, trial_state);

		const ResidualEvaluation trial_eval = evaluate_residual(plant, trial_primary, trial_yaw, thrust_n,
				ignition_mask, desired);

		if (trial_eval.cost < eval.cost) {
			eval = trial_eval;
			angles_to_state(trial_primary, trial_yaw, plant.engine_count, state);
			state_to_angles(state, plant.engine_count, primary_rad, yaw_rad);
			lambda = math::max(lambda * 0.1f, 1e-6f);

			if (converged(eval, config)) {
				result.converged = true;
				break;
			}

		} else {
			lambda = math::min(lambda * 10.f, 1e6f);
		}
	}

	result.cost = eval.cost;
	result.residual_torque_nm = eval.torque_error_nm;
	result.lambda_final = lambda;

	for (int i = 0; i < kControlMixerMaxEngines; ++i) {
		result.primary_rad[i] = primary_rad[i];
		result.yaw_rad[i] = yaw_rad[i];
	}

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
