#include "tv3_state_machine_fsm.hpp"

#include <lib/geo/geo.h>
#include <lib/tv3_msg_fields.hpp>
#include <mathlib/mathlib.h>

namespace tv3
{

namespace
{

const char *mode_name_for(uint8_t mode)
{
	switch (mode) {
	case tv3_sm_status_s::MODE_DISARMED_SAFE: return "DISARMED_SAFE";
	case tv3_sm_status_s::MODE_ARMED_STANDBY: return "ARMED_STANDBY";
	case tv3_sm_status_s::MODE_READY: return "READY";
	case tv3_sm_status_s::MODE_IGNITION_PENDING: return "IGNITION_PENDING";
	case tv3_sm_status_s::MODE_BOOST: return "BOOST";
	case tv3_sm_status_s::MODE_COAST: return "COAST";
	case tv3_sm_status_s::MODE_ABORT: return "ABORT";
	default: return "UNKNOWN";
	}
}

const char *fault_name_for(uint32_t fault)
{
	switch (fault) {
	case tv3_sm_status_s::FAULT_NONE: return "none";
	case tv3_sm_status_s::FAULT_COMMAND_ABORT: return "command_abort";
	case tv3_sm_status_s::FAULT_IGNITION_TIMEOUT: return "ignition_timeout";
	case tv3_sm_status_s::FAULT_SENSOR_STALE: return "sensor_stale";
	case tv3_sm_status_s::FAULT_GCS_LOSS: return "gcs_loss";
	case tv3_sm_status_s::FAULT_MOTOR_DATA: return "motor_data";
	case tv3_sm_status_s::FAULT_ARMING: return "arming";
	default: return "unknown";
	}
}

float total_vehicle_thrust_n(const StateMachineInputs &inputs, int32_t engine_count)
{
	const int count = math::constrain(static_cast<int>(inputs.engine_state.engine_count), 0,
					  static_cast<int>(tv3_lc_eng_st_s::MAX_ENGINES));
	float total = 0.f;

	for (int i = 0; i < count; ++i) {
		const float thr = math::max(filtered_thrust_n(inputs.engine_state, i),
					    math::max(measured_thrust_n(inputs.engine_state, i),
						      expected_thrust_n(inputs.engine_state, i)));

		if (thr > 0.f) {
			total += thr;
		}
	}

	if (total > 0.f) {
		return total;
	}

	const float single = math::max(inputs.thrust.measured_thrust_n, inputs.thrust.expected_thrust_n);
	return single * math::max(engine_count, static_cast<int32_t>(1));
}

} // namespace

uint8_t VehicleStateMachine::engine_bit(int engine_index)
{
	return engine_index >= 0 && engine_index < 8 ? static_cast<uint8_t>(1u << engine_index) : 0;
}

const char *VehicleStateMachine::mode_name() const
{
	return mode_name_for(_mode);
}

const char *VehicleStateMachine::fault_name() const
{
	return fault_name_for(_fault_reason);
}

bool VehicleStateMachine::mode_or_fault_changed(uint8_t &last_mode, uint32_t &last_fault) const
{
	if (_mode == last_mode && _fault_reason == last_fault) {
		return false;
	}

	last_mode = _mode;
	last_fault = _fault_reason;
	return true;
}

void VehicleStateMachine::reset_state()
{
	_mode = tv3_sm_status_s::MODE_DISARMED_SAFE;
	_fault_reason = tv3_sm_status_s::FAULT_NONE;
	_ignition_on = false;
	_launch_requested = false;
	_abort_requested = false;
	_reset_requested = false;
	_ignition_timestamp = 0;
	_boost_timestamp = 0;
	_burnout_low_timestamp = 0;
	reset_engine_sequence();
	_last_update = 0;
}

void VehicleStateMachine::set_fault(uint32_t fault_reason)
{
	_fault_reason = fault_reason;
	_mode = tv3_sm_status_s::MODE_ABORT;
	_ignition_on = false;
	_ignition_mask = 0;
}

void VehicleStateMachine::reset_engine_sequence()
{
	_ignition_mask = 0;
	_active_sequence_slot = 0;
	_current_engine_confirm_timestamp = 0;
	_sequence_complete = false;
}

void VehicleStateMachine::start_engine_sequence(hrt_abstime now)
{
	_active_sequence_slot = 0;
	_current_engine_confirm_timestamp = 0;
	_sequence_complete = false;
	_ignition_mask = engine_bit(_config.ignition_sequence[0]);
	_ignition_timestamp = now;
}

bool VehicleStateMachine::active_sequence_engine_confirmed(const StateMachineInputs &inputs) const
{
	if (_config.engine_count <= 1 || inputs.engine_state.engine_count == 0) {
		return inputs.thrust.ignition_confirmed;
	}

	const int engine = _config.ignition_sequence[_active_sequence_slot];
	return (inputs.engine_state.confirmed_mask & engine_bit(engine)) != 0;
}

bool VehicleStateMachine::all_sequence_engines_confirmed(const StateMachineInputs &inputs) const
{
	if (_config.engine_count <= 1) {
		return inputs.thrust.ignition_confirmed;
	}

	uint8_t required_mask = 0;

	for (int i = 0; i < _config.engine_count; ++i) {
		required_mask |= engine_bit(_config.ignition_sequence[i]);
	}

	return required_mask != 0 && (inputs.engine_state.confirmed_mask & required_mask) == required_mask;
}

void VehicleStateMachine::update_engine_sequence(hrt_abstime now, const StateMachineInputs &inputs)
{
	if (_config.engine_count <= 1) {
		_sequence_complete = inputs.thrust.ignition_confirmed;
		return;
	}

	if (active_sequence_engine_confirmed(inputs)) {
		if (_current_engine_confirm_timestamp == 0) {
			_current_engine_confirm_timestamp = now;
		}

		if (_active_sequence_slot + 1 < _config.engine_count
		    && hrt_elapsed_time(&_current_engine_confirm_timestamp) >= static_cast<hrt_abstime>(_config.ignition_dwell_ms) * 1000ULL) {
			++_active_sequence_slot;
			_ignition_mask |= engine_bit(_config.ignition_sequence[_active_sequence_slot]);
			_current_engine_confirm_timestamp = 0;
			_ignition_timestamp = now;
		}
	} else {
		_current_engine_confirm_timestamp = 0;
	}

	_sequence_complete = all_sequence_engines_confirmed(inputs);
}

void VehicleStateMachine::update(hrt_abstime now, const StateMachineInputs &inputs)
{
	if (_config.enabled <= 0) {
		reset_state();
		return;
	}

	const bool armed = inputs.vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

	if (!armed) {
		reset_state();
		return;
	}

	const bool thrust_valid = inputs.thrust.valid;
	const bool ignition_confirmed = inputs.thrust.ignition_confirmed;
	const bool gcs_ok = !inputs.vehicle_status.gcs_connection_lost;

	if (_reset_requested) {
		_mode = tv3_sm_status_s::MODE_READY;
		_fault_reason = tv3_sm_status_s::FAULT_NONE;
		_ignition_on = false;
		_reset_requested = false;
		_abort_requested = false;
		_launch_requested = false;
		_ignition_timestamp = 0;
		_boost_timestamp = 0;
		_burnout_low_timestamp = 0;
		reset_engine_sequence();
	}

	if (_abort_requested) {
		set_fault(tv3_sm_status_s::FAULT_COMMAND_ABORT);
		_abort_requested = false;
	}

	if (_mode == tv3_sm_status_s::MODE_DISARMED_SAFE) {
		_mode = tv3_sm_status_s::MODE_ARMED_STANDBY;
	}

	const bool motor_loaded = inputs.thrust.expected_thrust_n > 0.f || inputs.thrust.selected_motor_id[0] != '\0';

	if ((_mode == tv3_sm_status_s::MODE_ARMED_STANDBY || _mode == tv3_sm_status_s::MODE_READY) && motor_loaded) {
		_mode = tv3_sm_status_s::MODE_READY;
	}

	if (_mode == tv3_sm_status_s::MODE_READY && _launch_requested) {
		_mode = tv3_sm_status_s::MODE_IGNITION_PENDING;
		_ignition_on = true;
		start_engine_sequence(now);
		_launch_requested = false;
		_last_update = now;
	}

	if (_config.abort_on_gcs_loss > 0 && !gcs_ok
	    && (_mode == tv3_sm_status_s::MODE_IGNITION_PENDING || _mode == tv3_sm_status_s::MODE_BOOST)) {
		set_fault(tv3_sm_status_s::FAULT_GCS_LOSS);
	}

	if (_mode == tv3_sm_status_s::MODE_IGNITION_PENDING) {
		update_engine_sequence(now, inputs);

		if (!active_sequence_engine_confirmed(inputs) && _ignition_timestamp != 0
		    && hrt_elapsed_time(&_ignition_timestamp) > static_cast<hrt_abstime>(_config.ignition_timeout_ms) * 1000ULL) {
			set_fault(tv3_sm_status_s::FAULT_IGNITION_TIMEOUT);
		}

		const bool ignition_sequence_complete = _config.engine_count > 1 ? _sequence_complete : ignition_confirmed;

		if (ignition_sequence_complete) {
			_mode = tv3_sm_status_s::MODE_BOOST;
			_boost_timestamp = now;
			_last_update = now;
		}
	}

	if (_mode == tv3_sm_status_s::MODE_BOOST) {
		if (!thrust_valid) {
			set_fault(tv3_sm_status_s::FAULT_SENSOR_STALE);
		}

		_last_update = now;

		const float burnout_thrust_n = math::max(inputs.thrust.filtered_thrust_n, inputs.thrust.expected_thrust_n);
		const bool below_burnout_threshold = burnout_thrust_n < _config.burnout_threshold_n;
		const hrt_abstime burn_time_us = _boost_timestamp != 0 ? now - _boost_timestamp : 0;

		if (below_burnout_threshold && burn_time_us > static_cast<hrt_abstime>(_config.minimum_burn_ms) * 1000ULL) {
			if (_burnout_low_timestamp == 0) {
				_burnout_low_timestamp = now;
			} else if (hrt_elapsed_time(&_burnout_low_timestamp) > static_cast<hrt_abstime>(_config.burnout_dwell_ms) * 1000ULL) {
				_mode = tv3_sm_status_s::MODE_COAST;
				_ignition_on = false;
			}
		} else {
			_burnout_low_timestamp = 0;
		}

		if (burn_time_us > static_cast<hrt_abstime>(_config.maximum_burn_ms) * 1000ULL) {
			_mode = tv3_sm_status_s::MODE_COAST;
			_ignition_on = false;
		}
	}

	if (_mode == tv3_sm_status_s::MODE_COAST) {
		_ignition_on = false;
		_ignition_mask = 0;
	}

	if (_mode == tv3_sm_status_s::MODE_ABORT) {
		_ignition_on = false;
		_ignition_mask = 0;
	}
}

void VehicleStateMachine::build_module_modes(hrt_abstime now, tv3_sm_modes_s &modes) const
{
	uint8_t phase_index = 0;
	ControlPhaseConfig selected{};

	for (int i = 0; i < _config.phase_count; ++i) {
		if (_config.phases[i].on_tv3_mode == _mode) {
			phase_index = static_cast<uint8_t>(i);
			selected = _config.phases[i];
		}
	}

	if (_config.phase_count > 0 && selected.on_tv3_mode != _mode) {
		selected = _config.phases[0];
		phase_index = 0;
	}

	modes.timestamp = now;
	modes.flight_phase = phase_index;
	modes.guidance_mode = selected.guidance_mode;
	modes.attitude_mode = selected.attitude_mode;
	modes.mixer_mode = selected.mixer_mode;
	modes.load_cell_mode = selected.load_cell_mode;
}

void VehicleStateMachine::build_status(hrt_abstime now, const StateMachineInputs &inputs, tv3_sm_status_s &status) const
{
	status.timestamp = now;
	status.mode = _mode;
	status.mode_active = _mode != tv3_sm_status_s::MODE_DISARMED_SAFE;
	status.armed = inputs.vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	status.ready = _mode >= tv3_sm_status_s::MODE_READY && _mode != tv3_sm_status_s::MODE_ABORT;
	status.ignition_on = _ignition_on;
	status.ignition_confirmed = inputs.thrust.ignition_confirmed;
	status.engine_count = static_cast<uint8_t>(_config.engine_count);
	status.ignition_mask = _ignition_mask;
	status.active_ignition_index = static_cast<uint8_t>(_config.ignition_sequence[_active_sequence_slot]);
	status.sequence_active = _mode == tv3_sm_status_s::MODE_IGNITION_PENDING || _mode == tv3_sm_status_s::MODE_BOOST;
	status.sequence_complete = _sequence_complete;
	status.rail_exit = true;
	status.burnout_detected = _mode == tv3_sm_status_s::MODE_COAST;
	status.thrust_valid = inputs.thrust.valid;
	status.gcs_link_ok = !inputs.vehicle_status.gcs_connection_lost;
	status.ignition_timestamp = _ignition_timestamp;
	status.boost_timestamp = _boost_timestamp;
	status.measured_thrust_n = inputs.thrust.measured_thrust_n;
	status.filtered_thrust_n = inputs.thrust.filtered_thrust_n;
	status.expected_thrust_n = inputs.thrust.expected_thrust_n;
	status.burn_time_s = _boost_timestamp != 0 ? static_cast<float>(now - _boost_timestamp) * 1e-6f : 0.f;
	status.rail_distance_m = 0.f;
	status.rail_velocity_m_s = 0.f;
	status.expected_motor_mass_kg = inputs.thrust.expected_motor_mass_kg;
	status.expected_vehicle_mass_kg = inputs.thrust.expected_vehicle_mass_kg;
	status.burn_fraction = inputs.thrust.burn_fraction;
	status.fault_reason = _fault_reason;
	status.selected_motor_index = inputs.thrust.selected_motor_index;
	memcpy(status.selected_motor_id, inputs.thrust.selected_motor_id, sizeof(status.selected_motor_id));
}

} // namespace tv3
