#include "tv3_control_mixer_fsm.hpp"

#include "tv3_control_mixer_plant.hpp"

#include <lib/tv3_msg_fields.hpp>

#include <mathlib/mathlib.h>

namespace tv3
{

using matrix::Vector3f;

void ControlMixerFsm::apply_module_mode(const tv3_sm_modes_s &modes)
{
	switch (modes.mixer_mode) {
	case tv3_sm_modes_s::MIXER_TORQUE_ONLY:
		_fsm.request(MixerMode::TorqueOnly);
		break;

	case tv3_sm_modes_s::MIXER_TORQUE_AND_THRUST:
		_fsm.request(MixerMode::TorqueAndThrust);
		break;

	default:
		_fsm.request(MixerMode::Off);
		break;
	}

	_fsm.apply_request();
}

float ControlMixerFsm::engine_chamber_thrust_n(const float engine_thrust_n[kControlMixerMaxEngines], int index)
{
	if (index < 0 || index >= kControlMixerMaxEngines) {
		return 0.f;
	}

	return PX4_ISFINITE(engine_thrust_n[index]) ? math::max(engine_thrust_n[index], 0.f) : 0.f;
}

ControlMixerRunOutput ControlMixerFsm::run(const ControlMixerCore &core, const ControlMixerRunInput &input, hrt_abstime now)
{
	ControlMixerRunOutput output{};
	const int engine_count = math::min(static_cast<int>(input.engine_state.engine_count), kControlMixerMaxEngines);
	const uint8_t ignition_mask = input.engine_state.ignition_mask;

	output.engine_command.timestamp = now;
	output.engine_command.engine_count = input.engine_state.engine_count;
	output.engine_command.ignition_mask = ignition_mask;
	output.engine_command.active_ignition_index = input.engine_state.active_ignition_index;
	output.engine_command.sequence_active = input.engine_state.sequence_active;
	output.engine_command.sequence_complete = input.engine_state.sequence_complete;

	for (int i = 0; i < kControlMixerMaxEngines; ++i) {
		selected_motor_index_ref(output.engine_command, i) = input.selected_motor_index[i];
	}

	if (_fsm.in_mode(MixerMode::Off)) {
		return output;
	}

	const bool boost_limits = _fsm.in_mode(MixerMode::TorqueOnly);

	const float torque_roll = math::constrain(input.torque_sp.xyz[0], -input.torque_roll_max, input.torque_roll_max);
	const float torque_pitch = math::constrain(input.torque_sp.xyz[1], -input.torque_pitch_max, input.torque_pitch_max);
	const float torque_yaw = math::constrain(input.torque_sp.xyz[2], -input.torque_yaw_max, input.torque_yaw_max);

	ControlMixerLimits limits{};
	limits.boost_limits = boost_limits;

	const bool use_warm_start = _prev_warm_valid && ignition_mask == _prev_mask;

	ControlMixerSolveInput solve_input{};
	solve_input.torque_nm = Vector3f{torque_roll, torque_pitch, torque_yaw};
	solve_input.ignition_mask = ignition_mask;

	for (int i = 0; i < engine_count; ++i) {
		solve_input.thrust_n[i] = engine_chamber_thrust_n(input.engine_thrust_n, i);
	}

	const ControlMixerSolveOutput solved = core.solve(solve_input, limits, _prev_primary_rad, _prev_yaw_rad, use_warm_start);

	for (int i = 0; i < engine_count; ++i) {
		if (ignition_mask & (1u << i)) {
			commanded_pitch_deg_ref(output.engine_command, i) = math::degrees(solved.primary_rad[i]);
			commanded_yaw_deg_ref(output.engine_command, i) = math::degrees(solved.yaw_rad[i]);
			commanded_splay_deg_ref(output.engine_command, i) = 0.f;
		}
	}

	if (solved.converged) {
		_prev_warm_valid = true;

		for (int i = 0; i < kControlMixerMaxEngines; ++i) {
			_prev_primary_rad[i] = solved.primary_rad[i];
			_prev_yaw_rad[i] = solved.yaw_rad[i];
		}
	} else {
		_prev_warm_valid = false;
	}

	_prev_mask = ignition_mask;

	output.publish_allocator_status = true;
	output.allocator_status.timestamp = now;
	output.allocator_status.iterations_used = 0;
	output.allocator_status.lm_path_active = true;
	output.allocator_status.converged = solved.converged;
	output.allocator_status.used_fallback_solution = solved.used_fallback;
	output.allocator_status.torque_direction_valid = solved.converged;
	set_demanded_torque_nm(output.allocator_status, Vector3f{torque_roll, torque_pitch, torque_yaw});
	set_demanded_body_force_n(output.allocator_status, Vector3f{});

	return output;
}

} // namespace tv3
