#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_gd_att_sp.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include "tv3_attitude_fsm.hpp"

using namespace time_literals;
using tv3::AttitudeControllerConfig;
using tv3::AttitudeFsm;

class TV3Attitude : public ModuleBase<TV3Attitude>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3Attitude() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3Attitude *instance = new TV3Attitude();

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

		PRINT_MODULE_DESCRIPTION("TV3 BetterController attitude module publishing body-frame torque setpoints.");
		PRINT_MODULE_USAGE_NAME("tv3_attitude", "modules");
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

		if (_parameter_update_sub.updated()) {
			parameter_update_s update{};
			_parameter_update_sub.copy(&update);
			update_parameters();
		}

		if (!_vehicle_attitude_sub.update(&_attitude)) {
			return;
		}

		_vehicle_angular_velocity_sub.update(&_angular_velocity);
		_module_modes_sub.copy(&_module_modes);
		_attitude_setpoint_sub.copy(&_attitude_setpoint);

		_fsm.apply_module_mode(_module_modes);

		const hrt_abstime now = hrt_absolute_time();
		const float dt = _last_update != 0 ? static_cast<float>(now - _last_update) * 1e-6f : 0.01f;
		_last_update = now;

		vehicle_torque_setpoint_s torque_sp{};
		_fsm.step(now, dt, _attitude, _angular_velocity, _attitude_setpoint, torque_sp);
		_torque_pub.publish(torque_sp);
	}

	void update_parameters()
	{
		AttitudeControllerConfig config{};
		config.ld_rad = read_float("RK_ATT_LD", config.ld_rad);
		config.pos_kp = read_float("RK_ATT_POS_KP", config.pos_kp);
		config.vel_kp = read_float("RK_ATT_VEL_KP", config.vel_kp);
		config.vel_ki = read_float("RK_ATT_VEL_KI", config.vel_ki);
		config.vel_kd = read_float("RK_ATT_VEL_KD", config.vel_kd);
		config.soften = read_float("RK_ATT_SOFTEN", config.soften);
		config.max_stopping_time_s = read_float("RK_ATT_MAX_STOP", config.max_stopping_time_s);
		config.min_flip_time_s = read_float("RK_ATT_MIN_FLIP", config.min_flip_time_s);
		config.roll_control_range_rad = read_float("RK_ATT_ROLL_RAD", config.roll_control_range_rad);
		config.deadband_rad = read_float("RK_ATT_DB_RAD", config.deadband_rad);
		config.large_error_rad = read_float("RK_ATT_LARGE_RAD", config.large_error_rad);
		config.integrator_limit = read_float("RK_ATT_INT_LIM", config.integrator_limit);
		config.moi_roll_kgm2 = read_float("RK_IXX", config.moi_roll_kgm2);
		config.moi_pitch_kgm2 = read_float("RK_IYY", config.moi_pitch_kgm2);
		config.moi_yaw_kgm2 = read_float("RK_IZZ", config.moi_yaw_kgm2);
		config.torque_roll_max = read_float("RK_TQ_R_MAX", config.torque_roll_max);
		config.torque_pitch_max = read_float("RK_TQ_P_MAX", config.torque_pitch_max);
		config.torque_yaw_max = read_float("RK_TQ_Y_MAX", config.torque_yaw_max);
		_fsm.set_controller_config(config);
	}

	float read_float(const char *name, float fallback)
	{
		param_t p = param_find(name);

		if (p != PARAM_INVALID) {
			param_get(p, &fallback);
		}

		return fallback;
	}

	AttitudeFsm _fsm{};
	hrt_abstime _last_update{0};
	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	tv3_sm_modes_s _module_modes{};
	tv3_gd_att_sp_s _attitude_setpoint{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _module_modes_sub{ORB_ID(tv3_sm_modes)};
	uORB::Subscription _attitude_setpoint_sub{ORB_ID(tv3_gd_att_sp)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
};

extern "C" __EXPORT int tv3_attitude_main(int argc, char *argv[])
{
	return TV3Attitude::main(argc, argv);
}
