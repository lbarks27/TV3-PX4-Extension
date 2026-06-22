#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <matrix/matrix/math.hpp>

#include <tv3_engine_geometry.hpp>
#include <tv3_plant_kinematics.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_engine_command.h>
#include <uORB/topics/tv3_engine_state.h>
#include <uORB/topics/tv3_gimbal_command.h>
#include <uORB/topics/tv3_guidance_status.h>
#include <uORB/topics/tv3_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

namespace
{
constexpr int kMaxEngines = tv3::kMaxEngines;

static uint8_t engine_bit(int engine_index)
{
	return engine_index >= 0 && engine_index < 8 ? static_cast<uint8_t>(1u << engine_index) : 0;
}
}

class TV3TvcAllocator : public ModuleBase<TV3TvcAllocator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3TvcAllocator() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3TvcAllocator *instance = new TV3TvcAllocator();

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
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION("TV3 joint TVC allocator mapping torque and thrust demands to per-engine gimbal commands.");
		PRINT_MODULE_USAGE_NAME("tv3_tvc_allocator", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(10_ms);
		return true;
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

		_tv3_status_sub.update(&_status);
		_tv3_engine_state_sub.update(&_engine_state);
		_tv3_guidance_status_sub.update(&_guidance_status);
		_actuator_servos_sub.update(&_actuator_servos);
		_tv3_gimbal_command_sub.update(&_tv3_gimbal_command);
		_torque_setpoint_sub.update(&_torque_sp);

		publish_engine_command(now);
	}

	void publish_engine_command(hrt_abstime now)
	{
		tv3_engine_command_s engine_command{};
		engine_command.timestamp = now;
		engine_command.engine_count = _status.engine_count > 0 ? _status.engine_count : static_cast<uint8_t>(_engine_count);
		engine_command.ignition_mask = _status.ignition_mask;
		engine_command.active_ignition_index = _status.active_ignition_index;
		engine_command.sequence_active = _status.sequence_active;
		engine_command.sequence_complete = _status.sequence_complete;

		for (int engine_index = 0; engine_index < _engine_count && engine_index < kMaxEngines; ++engine_index) {
			const int pitch_index = 2 * engine_index;

			if (pitch_index < actuator_servos_s::NUM_CONTROLS) {
				const float pitch_normalized = _actuator_servos.control[pitch_index];

				if (PX4_ISFINITE(pitch_normalized) && fabsf(pitch_normalized) > 1e-4f) {
					engine_command.commanded_pitch_deg[engine_index] = pitch_normalized * _engine_pitch_max_deg[engine_index];
				}
			}

			const int yaw_index = 2 * engine_index + 1;

			if (yaw_index < actuator_servos_s::NUM_CONTROLS) {
				const float yaw_normalized = _actuator_servos.control[yaw_index];

				if (PX4_ISFINITE(yaw_normalized) && fabsf(yaw_normalized) > 1e-4f) {
					engine_command.commanded_yaw_deg[engine_index] = yaw_normalized * _engine_yaw_max_deg[engine_index];
				}
			}
		}

		bool have_primary = false;

		for (int engine_index = 0; engine_index < _engine_count && engine_index < kMaxEngines; ++engine_index) {
			if (fabsf(engine_command.commanded_pitch_deg[engine_index]) > 1e-4f) {
				have_primary = true;
				break;
			}
		}

		if (!have_primary && _tv3_gimbal_command.timestamp > 0) {
			const int engine_limit = math::min(static_cast<int>(_tv3_gimbal_command.engine_count), _engine_count);

			for (int engine_index = 0; engine_index < engine_limit && engine_index < kMaxEngines; ++engine_index) {
				engine_command.commanded_pitch_deg[engine_index] = _tv3_gimbal_command.commanded_pitch_deg[engine_index];
			}
		}

		tv3::AllocationInput allocation_input{};
		allocation_input.engine_count = _engine_count;
		allocation_input.ignition_mask = _status.ignition_mask;
		allocation_input.desired_torque_nm = matrix::Vector3f{_torque_sp.xyz[0], _torque_sp.xyz[1], _torque_sp.xyz[2]};

		float total_chamber_thrust_n = 0.f;

		for (int engine_index = 0; engine_index < _engine_count && engine_index < kMaxEngines; ++engine_index) {
			if (_status.ignition_mask & engine_bit(engine_index)) {
				const float chamber_thrust_n = tv3::engine_chamber_thrust_n(
								       _engine_state.filtered_thrust_n[engine_index],
								       _engine_state.measured_thrust_n[engine_index],
								       _engine_state.expected_thrust_n[engine_index]);
				allocation_input.chamber_thrust_n[engine_index] = chamber_thrust_n;
				total_chamber_thrust_n += chamber_thrust_n;
			}

			allocation_input.geometry[engine_index] = _geometry[engine_index];
		}

		allocation_input.desired_thrust_n = total_chamber_thrust_n;

		if (_guidance_status.thrust_solution_valid && _guidance_status.required_thrust_n > 0.1f) {
			allocation_input.desired_thrust_n = _guidance_status.required_thrust_n;
		}

		bool have_warm_start = false;

		for (int engine_index = 0; engine_index < _engine_count; ++engine_index) {
			if (fabsf(_last_cmd_pitch_rad[engine_index]) > 1e-6f || fabsf(_last_cmd_yaw_rad[engine_index]) > 1e-6f) {
				have_warm_start = true;
				break;
			}
		}

		allocation_input.have_warm_start = have_warm_start;

		for (int engine_index = 0; engine_index < _engine_count && engine_index < kMaxEngines; ++engine_index) {
			allocation_input.warm_start_pitch_rad[engine_index] = _last_cmd_pitch_rad[engine_index];
			allocation_input.warm_start_yaw_rad[engine_index] = _last_cmd_yaw_rad[engine_index];
		}

		tv3::AllocationOutput allocation_output{};
		tv3::allocate_projected_gradient(allocation_input, allocation_output);

		for (int engine_index = 0; engine_index < kMaxEngines; ++engine_index) {
			_last_cmd_pitch_rad[engine_index] = allocation_output.pitch_rad[engine_index];
			_last_cmd_yaw_rad[engine_index] = allocation_output.yaw_rad[engine_index];
		}

		for (int engine_index = 0; engine_index < _engine_count && engine_index < kMaxEngines; ++engine_index) {
			if (_status.ignition_mask & engine_bit(engine_index)) {
				engine_command.commanded_pitch_deg[engine_index] = math::degrees(allocation_output.pitch_rad[engine_index]);
				engine_command.commanded_yaw_deg[engine_index] = math::degrees(allocation_output.yaw_rad[engine_index]);
				engine_command.commanded_splay_deg[engine_index] = math::degrees(allocation_output.yaw_rad[engine_index]);
			} else {
				engine_command.commanded_splay_deg[engine_index] = 0.f;
			}
		}

		_engine_command_pub.publish(engine_command);
	}

	void update_parameters()
	{
		param_t p = param_find("RK_ENG_COUNT");

		if (p != PARAM_INVALID) {
			param_get(p, &_engine_count);
			_engine_count = math::constrain(_engine_count, static_cast<int32_t>(1), static_cast<int32_t>(kMaxEngines));
		}

		for (int engine_index = 0; engine_index < kMaxEngines; ++engine_index) {
			char name[20];
			snprintf(name, sizeof(name), "CA_RK_G%u_PMAX", engine_index);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				param_get(p, &_engine_pitch_max_deg[engine_index]);
			}

			snprintf(name, sizeof(name), "CA_RK_G%u_YMAX", engine_index);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				param_get(p, &_engine_yaw_max_deg[engine_index]);
			}
		}

		for (int engine_index = 0; engine_index < kMaxEngines; ++engine_index) {
			char buffer[32];

			auto read_geometry = [&](const char *suffix, float default_value) -> float {
				snprintf(buffer, sizeof(buffer), "CA_RK_G%u_%s", engine_index, suffix);
				param_t handle = param_find(buffer);
				float value = default_value;

				if (handle != PARAM_INVALID) {
					param_get(handle, &value);
				}

				return value;
			};

			_geometry[engine_index].position(0) = read_geometry("PX", 0.f);
			_geometry[engine_index].position(1) = read_geometry("PY", 0.f);
			_geometry[engine_index].position(2) = read_geometry("PZ", 0.f);
			_geometry[engine_index].thrust_axis(0) = read_geometry("AX", 1.f);
			_geometry[engine_index].thrust_axis(1) = read_geometry("AY", 0.f);
			_geometry[engine_index].thrust_axis(2) = read_geometry("AZ", 0.f);
			_geometry[engine_index].primary_axis(0) = read_geometry("PAX", 0.f);
			_geometry[engine_index].primary_axis(1) = read_geometry("PAY", -1.f);
			_geometry[engine_index].primary_axis(2) = read_geometry("PAZ", 0.f);
			_geometry[engine_index].secondary_axis(0) = read_geometry("YAX", 0.f);
			_geometry[engine_index].secondary_axis(1) = read_geometry("YAY", 0.f);
			_geometry[engine_index].secondary_axis(2) = read_geometry("YAZ", -1.f);
			_geometry[engine_index].pitch_max_rad = math::radians(read_geometry("PMAX", 5.f));
			_geometry[engine_index].yaw_min_rad = math::radians(read_geometry("YMIN", 0.f));
			_geometry[engine_index].yaw_max_rad = math::radians(read_geometry("YMAX", 5.f));
			_geometry[engine_index].thrust_fraction = read_geometry("TF", _engine_count > 0 ? 1.f / _engine_count : 1.f);
		}
	}

	int32_t _engine_count{1};
	float _engine_pitch_max_deg[kMaxEngines]{};
	float _engine_yaw_max_deg[kMaxEngines]{};
	tv3::EngineGeometry _geometry[kMaxEngines]{};
	float _last_cmd_pitch_rad[kMaxEngines]{};
	float _last_cmd_yaw_rad[kMaxEngines]{};

	tv3_status_s _status{};
	tv3_engine_state_s _engine_state{};
	tv3_guidance_status_s _guidance_status{};
	actuator_servos_s _actuator_servos{};
	tv3_gimbal_command_s _tv3_gimbal_command{};
	vehicle_torque_setpoint_s _torque_sp{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _tv3_status_sub{ORB_ID(tv3_status)};
	uORB::Subscription _tv3_engine_state_sub{ORB_ID(tv3_engine_state)};
	uORB::Subscription _tv3_guidance_status_sub{ORB_ID(tv3_guidance_status)};
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _tv3_gimbal_command_sub{ORB_ID(tv3_gimbal_command)};
	uORB::Subscription _torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<tv3_engine_command_s> _engine_command_pub{ORB_ID(tv3_engine_command)};
};

extern "C" __EXPORT int tv3_tvc_allocator_main(int argc, char *argv[])
{
	return TV3TvcAllocator::main(argc, argv);
}
