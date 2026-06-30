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

#include "tv3_state_machine_fsm.hpp"

using namespace time_literals;
using tv3::StateMachineConfig;
using tv3::StateMachineInputs;
using tv3::VehicleStateMachine;

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

		StateMachineInputs inputs{};
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
		StateMachineConfig config{};

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

	VehicleStateMachine _fsm{};

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
