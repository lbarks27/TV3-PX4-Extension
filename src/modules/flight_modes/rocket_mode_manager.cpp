#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/internal_combustion_engine_control.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rocket_command.h>
#include <uORB/topics/rocket_engine_command.h>
#include <uORB/topics/rocket_engine_state.h>
#include <uORB/topics/rocket_mode_status.h>
#include <uORB/topics/rocket_status.h>
#include <uORB/topics/rocket_thrust.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

#include <stdio.h>

using namespace time_literals;

namespace
{
constexpr uint32_t kRocketVehicleCommand = 31010;
constexpr int kMaxEngines = 4;

static void copy_motor_id(char (&dst)[32], const char (&src)[32])
{
	memcpy(dst, src, sizeof(dst));
}

static uint8_t engine_bit(int engine_index)
{
	return engine_index >= 0 && engine_index < 8 ? static_cast<uint8_t>(1u << engine_index) : 0;
}
}

class RocketModeManager : public ModuleBase<RocketModeManager>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RocketModeManager() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		RocketModeManager *instance = new RocketModeManager();

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

		uint8_t command = rocket_command_s::COMMAND_NONE;

		if (!strcmp(argv[0], "launch")) {
			command = rocket_command_s::COMMAND_LAUNCH;
		} else if (!strcmp(argv[0], "abort")) {
			command = rocket_command_s::COMMAND_ABORT;
		} else if (!strcmp(argv[0], "reset")) {
			command = rocket_command_s::COMMAND_RESET;
		} else {
			return print_usage("unknown command");
		}

		RocketModeManager *instance = get_instance();

		if (instance == nullptr) {
			PX4_WARN("not running");
			return PX4_ERROR;
		}

		instance->publish_rocket_command(command, rocket_command_s::SOURCE_SCRIPT);
		instance->handle_rocket_command(command);
		return PX4_OK;
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION("Rocket ascent state machine and ignition manager.");
		PRINT_MODULE_USAGE_NAME("rocket_mode_manager", "modules");
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
		PX4_INFO("mode: %u ignition: %d rail_exit: %d fault: %u", (unsigned)_mode, _ignition_on, _rail_exit, (unsigned)_fault_reason);
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
		_rocket_thrust_sub.update(&_thrust);
		_rocket_engine_state_sub.update(&_engine_state);

		process_commands();
		update_state(now);
		publish_outputs(now);
	}

	void process_commands()
	{
		rocket_command_s rocket_cmd{};

		while (_rocket_command_sub.update(&rocket_cmd)) {
			handle_rocket_command(rocket_cmd.command);
		}

		vehicle_command_s vehicle_cmd{};

		while (_vehicle_command_sub.update(&vehicle_cmd)) {
			if (vehicle_cmd.command == kRocketVehicleCommand) {
				handle_rocket_command(static_cast<uint8_t>(vehicle_cmd.param1));
				publish_ack(vehicle_cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
			}
		}
	}

	void publish_rocket_command(uint8_t command, uint8_t source)
	{
		rocket_command_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.command = command;
		msg.source = source;
		msg.sequence = static_cast<uint32_t>(msg.timestamp & 0xffffffff);
		_command_pub.publish(msg);
	}

	void handle_rocket_command(uint8_t command)
	{
		switch (command) {
		case rocket_command_s::COMMAND_LAUNCH:
			_launch_requested = true;
			break;

		case rocket_command_s::COMMAND_ABORT:
			_abort_requested = true;
			break;

		case rocket_command_s::COMMAND_RESET:
			_reset_requested = true;
			break;

		default:
			break;
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
		ack.from_external = true;
		_ack_pub.publish(ack);
	}

	void update_parameters()
	{
		param_t p = param_find("RK_ENABLE");
		if (p != PARAM_INVALID) {
			param_get(p, &_enabled);
		}

		p = param_find("RK_LAUNCH_THR_N");
		if (p != PARAM_INVALID) {
			param_get(p, &_launch_threshold_n);
		}

		p = param_find("RK_IGNITION_MS");
		if (p != PARAM_INVALID) {
			param_get(p, &_ignition_pulse_ms);
		}

		p = param_find("RK_IGN_TO_MS");
		if (p != PARAM_INVALID) {
			param_get(p, &_ignition_timeout_ms);
		}

		p = param_find("RK_BURN_MIN_MS");
		if (p != PARAM_INVALID) {
			param_get(p, &_minimum_burn_ms);
		}

		p = param_find("RK_BURN_MAX_MS");
		if (p != PARAM_INVALID) {
			param_get(p, &_maximum_burn_ms);
		}

		p = param_find("RK_BURNOUT_N");
		if (p != PARAM_INVALID) {
			param_get(p, &_burnout_threshold_n);
		}

		p = param_find("RK_BURNOUT_MS");
		if (p != PARAM_INVALID) {
			param_get(p, &_burnout_dwell_ms);
		}

		p = param_find("RK_RAIL_LEN_M");
		if (p != PARAM_INVALID) {
			param_get(p, &_rail_length_m);
		}

			p = param_find("RK_ABORT_GCS");
			if (p != PARAM_INVALID) {
				param_get(p, &_abort_on_gcs_loss);
			}

			p = param_find("RK_ENG_COUNT");
			if (p != PARAM_INVALID) {
				param_get(p, &_engine_count);
				_engine_count = math::constrain(_engine_count, 1, kMaxEngines);
			}

			p = param_find("RK_IGN_DWELL_MS");
			if (p != PARAM_INVALID) {
				param_get(p, &_ignition_dwell_ms);
				_ignition_dwell_ms = math::max(_ignition_dwell_ms, 0);
			}

			for (int i = 0; i < kMaxEngines; ++i) {
				char name[16];
				snprintf(name, sizeof(name), "RK_IGN_IDX%d", i);
				p = param_find(name);
				if (p != PARAM_INVALID) {
					param_get(p, &_ignition_sequence[i]);
					_ignition_sequence[i] = math::constrain(_ignition_sequence[i], 0, kMaxEngines - 1);
				}
			}
		}

		void reset_state()
		{
		_mode = rocket_status_s::MODE_DISARMED_SAFE;
		_fault_reason = rocket_status_s::FAULT_NONE;
		_ignition_on = false;
		_launch_requested = false;
		_abort_requested = false;
		_reset_requested = false;
		_ignition_timestamp = 0;
		_boost_timestamp = 0;
			_burnout_low_timestamp = 0;
			_rail_exit = false;
			_rail_distance_m = 0.f;
			_rail_velocity_m_s = 0.f;
			reset_engine_sequence();
			_last_update = 0;
		}

		void set_fault(uint32_t fault_reason)
		{
			_fault_reason = fault_reason;
			_mode = rocket_status_s::MODE_ABORT;
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
			_ignition_mask = engine_bit(_ignition_sequence[0]);
			_ignition_timestamp = now;
		}

		bool active_sequence_engine_confirmed() const
		{
			if (_engine_count <= 1 || _engine_state.engine_count == 0) {
				return _thrust.ignition_confirmed;
			}

			const int engine = _ignition_sequence[_active_sequence_slot];
			return (_engine_state.confirmed_mask & engine_bit(engine)) != 0;
		}

		bool all_sequence_engines_confirmed() const
		{
			if (_engine_count <= 1) {
				return _thrust.ignition_confirmed;
			}

			uint8_t required_mask = 0;
			for (int i = 0; i < _engine_count; ++i) {
				required_mask |= engine_bit(_ignition_sequence[i]);
			}

			return required_mask != 0 && (_engine_state.confirmed_mask & required_mask) == required_mask;
		}

		void update_engine_sequence(hrt_abstime now)
		{
			if (_engine_count <= 1) {
				_sequence_complete = _thrust.ignition_confirmed;
				return;
			}

			if (active_sequence_engine_confirmed()) {
				if (_current_engine_confirm_timestamp == 0) {
					_current_engine_confirm_timestamp = now;
				}

				if (_active_sequence_slot + 1 < _engine_count
				    && hrt_elapsed_time(&_current_engine_confirm_timestamp) >= static_cast<hrt_abstime>(_ignition_dwell_ms) * 1000ULL) {
					++_active_sequence_slot;
					_ignition_mask |= engine_bit(_ignition_sequence[_active_sequence_slot]);
					_current_engine_confirm_timestamp = 0;
					_ignition_timestamp = now;
				}
			} else {
				_current_engine_confirm_timestamp = 0;
			}

			_sequence_complete = all_sequence_engines_confirmed();
		}

	void update_state(hrt_abstime now)
	{
		if (_enabled <= 0) {
			reset_state();
			return;
		}

		const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

		if (!armed) {
			reset_state();
			return;
		}

		const bool thrust_valid = _thrust.valid;
		const bool ignition_confirmed = _thrust.ignition_confirmed;
		const bool gcs_ok = !_vehicle_status.gcs_connection_lost;

			if (_reset_requested) {
				_mode = rocket_status_s::MODE_READY;
				_fault_reason = rocket_status_s::FAULT_NONE;
				_ignition_on = false;
			_reset_requested = false;
			_abort_requested = false;
			_launch_requested = false;
			_ignition_timestamp = 0;
			_boost_timestamp = 0;
				_burnout_low_timestamp = 0;
				_rail_exit = false;
				_rail_distance_m = 0.f;
				_rail_velocity_m_s = 0.f;
				reset_engine_sequence();
			}

		if (_abort_requested) {
			set_fault(rocket_status_s::FAULT_COMMAND_ABORT);
			_abort_requested = false;
		}

		if (_mode == rocket_status_s::MODE_DISARMED_SAFE) {
			_mode = rocket_status_s::MODE_ARMED_STANDBY;
		}

		const bool motor_loaded = _thrust.selected_motor_id[0] != '\0';

		if ((_mode == rocket_status_s::MODE_ARMED_STANDBY || _mode == rocket_status_s::MODE_READY) && motor_loaded) {
			_mode = rocket_status_s::MODE_READY;
		}

			if (_mode == rocket_status_s::MODE_READY && _launch_requested) {
				_mode = rocket_status_s::MODE_IGNITION_PENDING;
				_ignition_on = true;
				start_engine_sequence(now);
				_launch_requested = false;
				_last_update = now;
			}

		if (_abort_on_gcs_loss > 0 && !gcs_ok
		    && (_mode == rocket_status_s::MODE_IGNITION_PENDING || _mode == rocket_status_s::MODE_BOOST)) {
			set_fault(rocket_status_s::FAULT_GCS_LOSS);
		}

			if (_mode == rocket_status_s::MODE_IGNITION_PENDING) {
				update_engine_sequence(now);

				if (!active_sequence_engine_confirmed() && _ignition_timestamp != 0
				    && hrt_elapsed_time(&_ignition_timestamp) > static_cast<hrt_abstime>(_ignition_timeout_ms) * 1000ULL) {
					set_fault(rocket_status_s::FAULT_IGNITION_TIMEOUT);
				}

				const bool ignition_sequence_complete = _engine_count > 1 ? _sequence_complete : ignition_confirmed;

				if (ignition_sequence_complete) {
					_mode = rocket_status_s::MODE_BOOST;
					_boost_timestamp = now;
					_last_update = now;
			}
		}

		if (_mode == rocket_status_s::MODE_BOOST) {
			if (!thrust_valid) {
				set_fault(rocket_status_s::FAULT_SENSOR_STALE);
			}

			const float dt_s = _last_update != 0 ? static_cast<float>(now - _last_update) * 1e-6f : 0.f;
			_last_update = now;
			const float thrust_n = math::max(_thrust.filtered_thrust_n, _thrust.expected_thrust_n);
			const float mass_kg = math::max(_thrust.expected_vehicle_mass_kg, 0.1f);

			if (!_rail_exit && dt_s > 0.f) {
				const float accel_m_s2 = math::max(thrust_n / mass_kg - CONSTANTS_ONE_G, 0.f);
				_rail_distance_m += _rail_velocity_m_s * dt_s + 0.5f * accel_m_s2 * dt_s * dt_s;
				_rail_velocity_m_s += accel_m_s2 * dt_s;
				_rail_exit = _rail_distance_m >= _rail_length_m;
			}

			const bool below_burnout_threshold = thrust_n < _burnout_threshold_n;
			const hrt_abstime burn_time_us = _boost_timestamp != 0 ? now - _boost_timestamp : 0;

			if (below_burnout_threshold && burn_time_us > static_cast<hrt_abstime>(_minimum_burn_ms) * 1000ULL) {
				if (_burnout_low_timestamp == 0) {
					_burnout_low_timestamp = now;
				} else if (hrt_elapsed_time(&_burnout_low_timestamp) > static_cast<hrt_abstime>(_burnout_dwell_ms) * 1000ULL) {
					_mode = rocket_status_s::MODE_COAST;
					_ignition_on = false;
				}
			} else {
				_burnout_low_timestamp = 0;
			}

			if (burn_time_us > static_cast<hrt_abstime>(_maximum_burn_ms) * 1000ULL) {
				_mode = rocket_status_s::MODE_COAST;
				_ignition_on = false;
			}
		}

			if (_mode == rocket_status_s::MODE_COAST) {
				_ignition_on = false;
				_ignition_mask = 0;
			}

			if (_mode == rocket_status_s::MODE_ABORT) {
				_ignition_on = false;
				_ignition_mask = 0;
			}
		}

	void publish_outputs(hrt_abstime now)
	{
		internal_combustion_engine_control_s engine{};
		engine.timestamp = now;
			engine.ignition_on = _ignition_on;
			engine.throttle_control = _ignition_on ? 1.f : 0.f;
			_engine_pub.publish(engine);

			rocket_engine_command_s engine_command{};
			engine_command.timestamp = now;
			engine_command.engine_count = static_cast<uint8_t>(_engine_count);
			engine_command.ignition_mask = _ignition_mask;
			engine_command.active_ignition_index = static_cast<uint8_t>(_ignition_sequence[_active_sequence_slot]);
			engine_command.sequence_active = _mode == rocket_status_s::MODE_IGNITION_PENDING || _mode == rocket_status_s::MODE_BOOST;
			engine_command.sequence_complete = _sequence_complete;
			_engine_command_pub.publish(engine_command);

		rocket_status_s status{};
		status.timestamp = now;
		status.mode = _mode;
		status.mode_active = _mode != rocket_status_s::MODE_DISARMED_SAFE;
		status.armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		status.ready = _mode >= rocket_status_s::MODE_READY && _mode != rocket_status_s::MODE_ABORT;
		status.ignition_on = _ignition_on;
		status.ignition_confirmed = _thrust.ignition_confirmed;
		status.rail_exit = _rail_exit;
		status.burnout_detected = _mode == rocket_status_s::MODE_COAST;
		status.thrust_valid = _thrust.valid;
		status.gcs_link_ok = !_vehicle_status.gcs_connection_lost;
		status.ignition_timestamp = _ignition_timestamp;
		status.boost_timestamp = _boost_timestamp;
		status.measured_thrust_n = _thrust.measured_thrust_n;
		status.filtered_thrust_n = _thrust.filtered_thrust_n;
		status.expected_thrust_n = _thrust.expected_thrust_n;
		status.burn_time_s = _boost_timestamp != 0 ? static_cast<float>(now - _boost_timestamp) * 1e-6f : 0.f;
		status.rail_distance_m = _rail_distance_m;
		status.rail_velocity_m_s = _rail_velocity_m_s;
		status.expected_motor_mass_kg = _thrust.expected_motor_mass_kg;
		status.expected_vehicle_mass_kg = _thrust.expected_vehicle_mass_kg;
		status.burn_fraction = _thrust.burn_fraction;
		status.fault_reason = _fault_reason;
		status.selected_motor_index = _thrust.selected_motor_index;
		copy_motor_id(status.selected_motor_id, _thrust.selected_motor_id);
		_status_pub.publish(status);

		rocket_mode_status_s compat{};
		compat.timestamp = now;
		compat.state = _mode;
		compat.mode_active = status.mode_active;
		compat.armed = status.armed;
		compat.ignition_on = status.ignition_on;
		compat.load_cell_valid = status.thrust_valid;
		compat.gcs_link_ok = status.gcs_link_ok;
		compat.thrust_n = status.filtered_thrust_n;
		compat.burn_time_s = status.burn_time_s;
		compat.motor_mass_kg = status.expected_motor_mass_kg;
		compat.vehicle_mass_kg = status.expected_vehicle_mass_kg;
		compat.rail_distance_m = status.rail_distance_m;

		switch (_fault_reason) {
		case rocket_status_s::FAULT_COMMAND_ABORT:
			compat.abort_reason = rocket_mode_status_s::ABORT_REASON_COMMAND;
			break;

		case rocket_status_s::FAULT_IGNITION_TIMEOUT:
			compat.abort_reason = rocket_mode_status_s::ABORT_REASON_IGNITION_TIMEOUT;
			break;

		case rocket_status_s::FAULT_SENSOR_STALE:
			compat.abort_reason = rocket_mode_status_s::ABORT_REASON_SENSOR_STALE;
			break;

		case rocket_status_s::FAULT_GCS_LOSS:
			compat.abort_reason = rocket_mode_status_s::ABORT_REASON_GCS_LOSS;
			break;

		default:
			compat.abort_reason = rocket_mode_status_s::ABORT_REASON_NONE;
			break;
		}

		_compat_status_pub.publish(compat);
	}

	int32_t _enabled{1};
	float _launch_threshold_n{10.f};
	int32_t _ignition_pulse_ms{300};
	int32_t _ignition_timeout_ms{2000};
	int32_t _minimum_burn_ms{150};
	int32_t _maximum_burn_ms{6000};
	float _burnout_threshold_n{4.f};
	int32_t _burnout_dwell_ms{100};
	float _rail_length_m{3.5f};
		int32_t _abort_on_gcs_loss{0};
		int32_t _engine_count{1};
		int32_t _ignition_sequence[kMaxEngines]{0, 1, 2, 3};
		int32_t _ignition_dwell_ms{0};

		uint8_t _mode{rocket_status_s::MODE_DISARMED_SAFE};
		uint32_t _fault_reason{rocket_status_s::FAULT_NONE};
	bool _launch_requested{false};
	bool _abort_requested{false};
	bool _reset_requested{false};
		bool _ignition_on{false};
		bool _sequence_complete{false};
		uint8_t _ignition_mask{0};
		int _active_sequence_slot{0};
		bool _rail_exit{false};
	float _rail_distance_m{0.f};
	float _rail_velocity_m_s{0.f};
		hrt_abstime _ignition_timestamp{0};
		hrt_abstime _boost_timestamp{0};
		hrt_abstime _burnout_low_timestamp{0};
		hrt_abstime _current_engine_confirm_timestamp{0};
		hrt_abstime _last_update{0};

		vehicle_status_s _vehicle_status{};
		rocket_thrust_s _thrust{};
		rocket_engine_state_s _engine_state{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
		uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
		uORB::Subscription _rocket_command_sub{ORB_ID(rocket_command)};
		uORB::Subscription _rocket_thrust_sub{ORB_ID(rocket_thrust)};
		uORB::Subscription _rocket_engine_state_sub{ORB_ID(rocket_engine_state)};

		uORB::Publication<rocket_command_s> _command_pub{ORB_ID(rocket_command)};
		uORB::Publication<internal_combustion_engine_control_s> _engine_pub{ORB_ID(internal_combustion_engine_control)};
		uORB::Publication<rocket_engine_command_s> _engine_command_pub{ORB_ID(rocket_engine_command)};
		uORB::Publication<rocket_status_s> _status_pub{ORB_ID(rocket_status)};
	uORB::Publication<rocket_mode_status_s> _compat_status_pub{ORB_ID(rocket_mode_status)};
	uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};
};

extern "C" __EXPORT int rocket_mode_manager_main(int argc, char *argv[])
{
	return RocketModeManager::main(argc, argv);
}
