#include "tv3_engine_geometry.hpp"
#include "tv3_plant_kinematics.hpp"

#include <mathlib/mathlib.h>

namespace tv3
{

bool allocate_projected_gradient(const AllocationInput &input, AllocationOutput &output)
{
	float pitch_rad[kMaxEngines] = {};
	float yaw_rad[kMaxEngines] = {};

	float total_chamber_thrust_n = 0.f;

	for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
		if (input.ignition_mask & (1u << engine_index)) {
			total_chamber_thrust_n += input.chamber_thrust_n[engine_index];
		}
	}

	if (input.have_warm_start) {
		for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
			pitch_rad[engine_index] = input.warm_start_pitch_rad[engine_index];
			yaw_rad[engine_index] = input.warm_start_yaw_rad[engine_index];
		}

	} else if (input.desired_thrust_n < total_chamber_thrust_n - 0.5f && total_chamber_thrust_n > 1.f) {
		const float ratio = math::constrain(input.desired_thrust_n / total_chamber_thrust_n, 0.f, 1.f);
		const float yaw_guess_rad = acosf(ratio);

		for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
			if (input.ignition_mask & (1u << engine_index)) {
				yaw_rad[engine_index] = yaw_guess_rad;
			}
		}
	}

	const float thrust_weight = 0.02f;
	const float gain = 0.8f;
	const float epsilon_rad = 0.002f;
	const int max_iterations = 20;
	const float torque_tolerance_nm = 0.2f;
	const float thrust_tolerance_n = 0.5f;

	float best_pitch_rad[kMaxEngines] = {};
	float best_yaw_rad[kMaxEngines] = {};
	float best_score = 1e30f;
	matrix::Vector3f best_torque{};
	float best_axial_thrust_n = 0.f;

	for (int iteration = 0; iteration < max_iterations; ++iteration) {
		const matrix::Vector3f current_torque = total_torque_nm(input, pitch_rad, yaw_rad);
		const float current_axial_thrust_n = total_axial_thrust_n(input, pitch_rad, yaw_rad);
		const matrix::Vector3f torque_error = input.desired_torque_nm - current_torque;
		const float thrust_error_n = input.desired_thrust_n - current_axial_thrust_n;
		const float score = torque_error.norm() + thrust_weight * fabsf(thrust_error_n);

		if (score < best_score) {
			best_score = score;

			for (int engine_index = 0; engine_index < kMaxEngines; ++engine_index) {
				best_pitch_rad[engine_index] = pitch_rad[engine_index];
				best_yaw_rad[engine_index] = yaw_rad[engine_index];
			}

			best_torque = current_torque;
			best_axial_thrust_n = current_axial_thrust_n;
		}

		if (torque_error.norm() <= torque_tolerance_nm && fabsf(thrust_error_n) <= thrust_tolerance_n) {
			break;
		}

		const matrix::Vector3f torque_error_snapshot = torque_error;
		const float thrust_error_snapshot = thrust_error_n;
		float pitch_delta_rad[kMaxEngines] = {};
		float yaw_delta_rad[kMaxEngines] = {};

		for (int variable_index = 0; variable_index < input.engine_count * 2; ++variable_index) {
			const int engine_index = variable_index / 2;

			if ((input.ignition_mask & (1u << engine_index)) == 0
			    || input.chamber_thrust_n[engine_index] < 0.5f) {
				continue;
			}

			const bool pitch_axis = (variable_index % 2 == 0);
			const float saved_angle_rad = pitch_axis ? pitch_rad[engine_index] : yaw_rad[engine_index];

			if (pitch_axis) {
				pitch_rad[engine_index] = saved_angle_rad + epsilon_rad;
			} else {
				yaw_rad[engine_index] = saved_angle_rad + epsilon_rad;
			}

			const matrix::Vector3f torque_plus = total_torque_nm(input, pitch_rad, yaw_rad);
			const float thrust_plus_n = total_axial_thrust_n(input, pitch_rad, yaw_rad);

			if (pitch_axis) {
				pitch_rad[engine_index] = saved_angle_rad - epsilon_rad;
			} else {
				yaw_rad[engine_index] = saved_angle_rad - epsilon_rad;
			}

			const matrix::Vector3f torque_minus = total_torque_nm(input, pitch_rad, yaw_rad);
			const float thrust_minus_n = total_axial_thrust_n(input, pitch_rad, yaw_rad);

			if (pitch_axis) {
				pitch_rad[engine_index] = saved_angle_rad;
			} else {
				yaw_rad[engine_index] = saved_angle_rad;
			}

			const matrix::Vector3f torque_derivative = (torque_plus - torque_minus) / (2.f * epsilon_rad);
			const float thrust_derivative = (thrust_plus_n - thrust_minus_n) / (2.f * epsilon_rad);
			const float gradient = torque_error_snapshot.dot(torque_derivative)
					       + thrust_weight * thrust_error_snapshot * thrust_derivative;
			const float denominator = torque_derivative.norm_squared()
						    + (thrust_weight * thrust_derivative) * (thrust_weight * thrust_derivative)
						    + 1e-8f;
			const float step = -gradient / denominator * gain;

			if (pitch_axis) {
				pitch_delta_rad[engine_index] = step;
			} else {
				yaw_delta_rad[engine_index] = step;
			}
		}

		for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
			pitch_rad[engine_index] += pitch_delta_rad[engine_index];
			yaw_rad[engine_index] += yaw_delta_rad[engine_index];
		}

		for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
			pitch_rad[engine_index] = math::constrain(pitch_rad[engine_index],
						     -input.geometry[engine_index].pitch_max_rad,
						     input.geometry[engine_index].pitch_max_rad);
			yaw_rad[engine_index] = math::constrain(yaw_rad[engine_index],
						   input.geometry[engine_index].yaw_min_rad,
						   input.geometry[engine_index].yaw_max_rad);
		}
	}

	bool sane = true;

	for (int engine_index = 0; engine_index < kMaxEngines; ++engine_index) {
		if (!PX4_ISFINITE(best_pitch_rad[engine_index]) || !PX4_ISFINITE(best_yaw_rad[engine_index])) {
			sane = false;
		}
	}

	if (!sane || best_score > 1000.f) {
		for (int engine_index = 0; engine_index < kMaxEngines; ++engine_index) {
			output.pitch_rad[engine_index] = 0.f;
			output.yaw_rad[engine_index] = 0.f;
		}

	} else {
		for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
			output.pitch_rad[engine_index] = best_pitch_rad[engine_index];
			output.yaw_rad[engine_index] = best_yaw_rad[engine_index];
		}
	}

	output.best_score = best_score;
	output.achieved_torque = best_torque;
	output.achieved_thrust_n = best_axial_thrust_n;
	return sane && best_score <= 1000.f;
}

} // namespace tv3
