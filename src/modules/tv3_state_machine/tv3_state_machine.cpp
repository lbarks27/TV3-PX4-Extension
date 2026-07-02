#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <lib/systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_sm_cmd.h>
#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/tv3_sm_status.h>
#include <uORB/topics/tv3_lc_thrust.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

#include "../../lib/tv3_module_modes.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

namespace tv3
{

constexpr int kStateMachineMaxEngines = tv3_lc_eng_st_s::MAX_ENGINES;

struct StateMachineConfig {
	int32_t enabled{1};
	float launch_threshold_n{10.f};
	int32_t ignition_pulse_ms{300};
	int32_t ignition_timeout_ms{2000};
	int32_t minimum_burn_ms{150};
	int32_t maximum_burn_ms{6000};
	float burnout_threshold_n{4.f};
	int32_t burnout_dwell_ms{100};
	int32_t abort_on_gcs_loss{0};
	int32_t engine_count{1};
	int32_t ignition_sequence[kStateMachineMaxEngines]{0, 1, 2};
	int32_t ignition_dwell_ms{0};
	int32_t phase_count{0};
	ControlPhaseConfig phases[kMaxControlPhases]{};
};

struct StateMachineInputs {
	vehicle_status_s vehicle_status{};
	tv3_lc_thrust_s thrust{};
	tv3_lc_eng_st_s engine_state{};
};

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

uint8_t engine_bit(int engine_index)
{
	return engine_index >= 0 && engine_index < 8 ? static_cast<uint8_t>(1u << engine_index) : 0;
}

} // namespace

class VehicleStateMachine {
public:
	void set_config(const StateMachineConfig &config) { _config = config; }

	void request_launch() { _launch_requested = true; }

	void request_abort() { _abort_requested = true; }

	void request_reset() { _reset_requested = true; }

	void update(hrt_abstime now, const StateMachineInputs &inputs)
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

	void build_module_modes(hrt_abstime now, tv3_sm_modes_s &modes) const
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

	void build_status(hrt_abstime now, const StateMachineInputs &inputs, tv3_sm_status_s &status) const
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

	uint8_t mode() const { return _mode; }

	uint32_t fault_reason() const { return _fault_reason; }

	bool mode_or_fault_changed(uint8_t &last_mode, uint32_t &last_fault) const
	{
		if (_mode == last_mode && _fault_reason == last_fault) {
			return false;
		}

		last_mode = _mode;
		last_fault = _fault_reason;
		return true;
	}

	const char *mode_name() const { return mode_name_for(_mode); }

	const char *fault_name() const { return fault_name_for(_fault_reason); }

private:
	void reset_state()
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

	void set_fault(uint32_t fault_reason)
	{
		_fault_reason = fault_reason;
		_mode = tv3_sm_status_s::MODE_ABORT;
		_ignition_on = false;
		_ignition_mask = 0;
	}

	void reset_engine_sequence()
	{
		_ignition_mask = 0;
		_active_sequence_slot = 0;
		_current_engine_confirm_timestamp = 0;
		_sequence_complete = false;
	}

	void start_engine_sequence(hrt_abstime now)
	{
		_active_sequence_slot = 0;
		_current_engine_confirm_timestamp = 0;
		_sequence_complete = false;
		_ignition_mask = engine_bit(_config.ignition_sequence[0]);
		_ignition_timestamp = now;
	}

	bool active_sequence_engine_confirmed(const StateMachineInputs &inputs) const
	{
		if (_config.engine_count <= 1 || inputs.engine_state.engine_count == 0) {
			return inputs.thrust.ignition_confirmed;
		}

		const int engine = _config.ignition_sequence[_active_sequence_slot];
		return (inputs.engine_state.confirmed_mask & engine_bit(engine)) != 0;
	}

	bool all_sequence_engines_confirmed(const StateMachineInputs &inputs) const
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

	void update_engine_sequence(hrt_abstime now, const StateMachineInputs &inputs)
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

	StateMachineConfig _config{};

	uint8_t _mode{tv3_sm_status_s::MODE_DISARMED_SAFE};
	uint32_t _fault_reason{tv3_sm_status_s::FAULT_NONE};
	bool _launch_requested{false};
	bool _abort_requested{false};
	bool _reset_requested{false};
	bool _ignition_on{false};
	bool _sequence_complete{false};
	uint8_t _ignition_mask{0};
	int _active_sequence_slot{0};
	hrt_abstime _ignition_timestamp{0};
	hrt_abstime _boost_timestamp{0};
	hrt_abstime _burnout_low_timestamp{0};
	hrt_abstime _current_engine_confirm_timestamp{0};
	hrt_abstime _last_update{0};
};

} // namespace tv3

namespace
{
constexpr uint32_t kTV3VehicleCommand = 31010;

const char *command_name(uint8_t command)
{
	switch (command) {
	case tv3_sm_cmd_s::COMMAND_LAUNCH: return "launch";
	case tv3_sm_cmd_s::COMMAND_ABORT: return "abort";
	case tv3_sm_cmd_s::COMMAND_RESET: return "reset";
	default: return "unknown";
	}
}
}

class TV3StateMachine : public ModuleBase<TV3StateMachine>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3StateMachine() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3StateMachine *instance = new TV3StateMachine();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	static int custom_command(int argc, char *argv[])
	{
		if (argc < 1) {
			return print_usage("missing command");
		}

		uint8_t command = tv3_sm_cmd_s::COMMAND_NONE;

		if (!strcmp(argv[0], "launch")) {
			command = tv3_sm_cmd_s::COMMAND_LAUNCH;
		} else if (!strcmp(argv[0], "abort")) {
			command = tv3_sm_cmd_s::COMMAND_ABORT;
		} else if (!strcmp(argv[0], "reset")) {
			command = tv3_sm_cmd_s::COMMAND_RESET;
		} else {
			return print_usage("unknown command");
		}

		TV3StateMachine *instance = get_instance();

		if (instance == nullptr) {
			PX4_WARN("not running");
			return PX4_ERROR;
		}

		instance->publish_state_machine_command(command, tv3_sm_cmd_s::SOURCE_SCRIPT);
		instance->handle_state_machine_command(command);
		return PX4_OK;
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION("TV3 master vehicle state machine and module orchestrator.");
		PRINT_MODULE_USAGE_NAME("tv3_state_machine", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_COMMAND_DESCR("launch", "Publish a launch command");
		PRINT_MODULE_USAGE_COMMAND_DESCR("abort", "Publish an abort command");
		PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Publish a reset command");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(10_ms);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("mode: %s fault: %s", _fsm.mode_name(), _fsm.fault_name());
		return 0;
	}

private:
	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		const hrt_abstime now = hrt_absolute_time();

		if (_parameter_update_sub.updated()) {
			parameter_update_s update{};
			_parameter_update_sub.copy(&update);
			update_parameters();
		}

		_vehicle_status_sub.update(&_vehicle_status);
		_tv3_lc_thrust_sub.update(&_thrust);
		_tv3_lc_eng_st_sub.update(&_engine_state);

		process_commands();

		tv3::StateMachineInputs inputs{};
		inputs.vehicle_status = _vehicle_status;
		inputs.thrust = _thrust;
		inputs.engine_state = _engine_state;

		_fsm.update(now, inputs);
		announce_state_if_changed();

		tv3_sm_modes_s module_modes{};
		_fsm.build_module_modes(now, module_modes);
		_module_modes_pub.publish(module_modes);

		tv3_sm_status_s status{};
		_fsm.build_status(now, inputs, status);
		_status_pub.publish(status);
	}

	void process_commands()
	{
		tv3_sm_cmd_s tv3_cmd{};

		while (_state_machine_command_sub.update(&tv3_cmd)) {
			handle_state_machine_command(tv3_cmd.command);
		}

		vehicle_command_s vehicle_cmd{};

		while (_vehicle_command_sub.update(&vehicle_cmd)) {
			if (vehicle_cmd.command == kTV3VehicleCommand) {
				const uint8_t command = static_cast<uint8_t>(vehicle_cmd.param1);
				const bool accepted = handle_state_machine_command(command);
				publish_ack(vehicle_cmd, accepted ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
					    vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);

				if (accepted) {
					mavlink_log_info(&_mavlink_log_pub, "TV3 command %s accepted\t", command_name(command));
				} else {
					mavlink_log_warning(&_mavlink_log_pub, "TV3 command rejected param1 %.1f\t", (double)vehicle_cmd.param1);
				}
			}
		}
	}

	void publish_state_machine_command(uint8_t command, uint8_t source)
	{
		tv3_sm_cmd_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.command = command;
		msg.source = source;
		msg.sequence = static_cast<uint32_t>(msg.timestamp & 0xffffffff);
		_command_pub.publish(msg);
	}

	bool handle_state_machine_command(uint8_t command)
	{
		switch (command) {
		case tv3_sm_cmd_s::COMMAND_LAUNCH:
			_fsm.request_launch();
			return true;

		case tv3_sm_cmd_s::COMMAND_ABORT:
			_fsm.request_abort();
			return true;

		case tv3_sm_cmd_s::COMMAND_RESET:
			_fsm.request_reset();
			return true;

		default:
			return false;
		}
	}

	void publish_ack(const vehicle_command_s &cmd, uint8_t result)
	{
		vehicle_command_ack_s ack{};
		ack.timestamp = hrt_absolute_time();
		ack.command = cmd.command;
		ack.result = result;
		ack.target_system = cmd.source_system;
		ack.target_component = cmd.source_component;
		ack.from_external = false;
		_ack_pub.publish(ack);
	}

	void announce_state_if_changed()
	{
		if (!_fsm.mode_or_fault_changed(_last_announced_mode, _last_announced_fault)) {
			return;
		}

		if (_fsm.mode() == tv3_sm_status_s::MODE_ABORT || _fsm.fault_reason() != tv3_sm_status_s::FAULT_NONE) {
			mavlink_log_critical(&_mavlink_log_pub, "TV3 state %s fault %s\t", _fsm.mode_name(), _fsm.fault_name());
		} else {
			mavlink_log_info(&_mavlink_log_pub, "TV3 state %s\t", _fsm.mode_name());
		}
	}

	void update_parameters()
	{
		tv3::StateMachineConfig config{};

		param_t p = param_find("RK_ENABLE");

		if (p != PARAM_INVALID) {
			param_get(p, &config.enabled);
		}

		p = param_find("RK_LAUNCH_THR_N");

		if (p != PARAM_INVALID) {
			param_get(p, &config.launch_threshold_n);
		}

		p = param_find("RK_IGNITION_MS");

		if (p != PARAM_INVALID) {
			param_get(p, &config.ignition_pulse_ms);
		}

		p = param_find("RK_IGN_TO_MS");

		if (p != PARAM_INVALID) {
			param_get(p, &config.ignition_timeout_ms);
		}

		p = param_find("RK_BURN_MIN_MS");

		if (p != PARAM_INVALID) {
			param_get(p, &config.minimum_burn_ms);
		}

		p = param_find("RK_BURN_MAX_MS");

		if (p != PARAM_INVALID) {
			param_get(p, &config.maximum_burn_ms);
		}

		p = param_find("RK_BURNOUT_N");

		if (p != PARAM_INVALID) {
			param_get(p, &config.burnout_threshold_n);
		}

		p = param_find("RK_BURNOUT_MS");

		if (p != PARAM_INVALID) {
			param_get(p, &config.burnout_dwell_ms);
		}

		p = param_find("RK_ABORT_GCS");

		if (p != PARAM_INVALID) {
			param_get(p, &config.abort_on_gcs_loss);
		}

		p = param_find("RK_ENG_COUNT");

		if (p != PARAM_INVALID) {
			param_get(p, &config.engine_count);
			config.engine_count = math::constrain(config.engine_count, static_cast<int32_t>(1),
							      static_cast<int32_t>(tv3::kStateMachineMaxEngines));
		}

		p = param_find("RK_IGN_DWELL_MS");

		if (p != PARAM_INVALID) {
			param_get(p, &config.ignition_dwell_ms);
			config.ignition_dwell_ms = math::max(config.ignition_dwell_ms, static_cast<int32_t>(0));
		}

		for (int i = 0; i < tv3::kStateMachineMaxEngines; ++i) {
			char name[16];
			snprintf(name, sizeof(name), "RK_IGN_IDX%d", i);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				param_get(p, &config.ignition_sequence[i]);
				config.ignition_sequence[i] = math::constrain(config.ignition_sequence[i], static_cast<int32_t>(0),
						 static_cast<int32_t>(tv3::kStateMachineMaxEngines - 1));
			}
		}

		p = param_find("RK_CTRL_NPHASE");

		if (p != PARAM_INVALID) {
			param_get(p, &config.phase_count);
			config.phase_count = math::constrain(config.phase_count, static_cast<int32_t>(0),
							   static_cast<int32_t>(tv3::kMaxControlPhases));
		}

		for (int i = 0; i < tv3::kMaxControlPhases; ++i) {
			char name[24];
			snprintf(name, sizeof(name), "RK_CTRL_P%d_ON", i);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				int32_t on_mode = 0;
				param_get(p, &on_mode);
				config.phases[i].on_tv3_mode = static_cast<uint8_t>(on_mode);
			}

			snprintf(name, sizeof(name), "RK_CTRL_P%d_MODES", i);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				int32_t packed = 0;
				param_get(p, &packed);
				config.phases[i].guidance_mode = static_cast<uint8_t>(packed & 0xff);
				config.phases[i].attitude_mode = static_cast<uint8_t>((packed >> 8) & 0xff);
				config.phases[i].mixer_mode = static_cast<uint8_t>((packed >> 16) & 0xff);
				config.phases[i].load_cell_mode = static_cast<uint8_t>((packed >> 24) & 0xff);
			}
		}

		_fsm.set_config(config);
	}

	uint8_t _last_announced_mode{UINT8_MAX};
	uint32_t _last_announced_fault{UINT32_MAX};

	tv3::VehicleStateMachine _fsm{};

	vehicle_status_s _vehicle_status{};
	tv3_lc_thrust_s _thrust{};
	tv3_lc_eng_st_s _engine_state{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _state_machine_command_sub{ORB_ID(tv3_sm_cmd)};
	uORB::Subscription _tv3_lc_thrust_sub{ORB_ID(tv3_lc_thrust)};
	uORB::Subscription _tv3_lc_eng_st_sub{ORB_ID(tv3_lc_eng_st)};

	uORB::Publication<tv3_sm_cmd_s> _command_pub{ORB_ID(tv3_sm_cmd)};
	uORB::Publication<tv3_sm_status_s> _status_pub{ORB_ID(tv3_sm_status)};
	uORB::Publication<tv3_sm_modes_s> _module_modes_pub{ORB_ID(tv3_sm_modes)};
	uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};
	orb_advert_t _mavlink_log_pub{nullptr};
};

extern "C" __EXPORT int tv3_state_machine_main(int argc, char *argv[])
{
	return TV3StateMachine::main(argc, argv);
}
