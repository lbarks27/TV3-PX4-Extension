#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <lib/tv3_engine_geometry.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_gd_att_sp.h>
#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/tv3_gd_thr_sp.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

#include "tv3_guidance_fsm.hpp"

using namespace time_literals;
using tv3::GuidanceFsm;
using tv3::GuidanceWaypointConfig;
using tv3::read_param_float;

class TV3Guidance : public ModuleBase<TV3Guidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3Guidance() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3Guidance *instance = new TV3Guidance();

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

		PRINT_MODULE_DESCRIPTION("TV3 guidance publishes attitude and thrust setpoints.");
		PRINT_MODULE_USAGE_NAME("tv3_guidance", "modules");
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

		_module_modes_sub.copy(&_module_modes);
		_vehicle_attitude_sub.update(&_attitude);
		_local_position_sub.update(&_local_position);
		_groundtruth_position_sub.update(&_groundtruth_position);
		_engine_state_sub.update(&_engine_state);

		_fsm.apply_module_mode(_module_modes);

		tv3_gd_att_sp_s attitude_sp{};
		tv3_gd_thr_sp_s thrust_sp{};
		_fsm.step(hrt_absolute_time(), _attitude, _local_position, _groundtruth_position, _engine_state,
			  attitude_sp, thrust_sp);

		_attitude_setpoint_pub.publish(attitude_sp);
		_thrust_setpoint_pub.publish(thrust_sp);
	}

	void update_parameters()
	{
		GuidanceWaypointConfig config{};
		config.wp_n_m = read_param_float("RK_GD_WP1_N", config.wp_n_m);
		config.wp_e_m = read_param_float("RK_GD_WP1_E", config.wp_e_m);
		config.wp_d_m = read_param_float("RK_GD_WP1_D", config.wp_d_m);
		config.pos_p = read_param_float("RK_GD_POS_P", config.pos_p);
		config.acceptance_m = read_param_float("RK_GD_ACC_RAD", config.acceptance_m);
		config.max_velocity_m_s = read_param_float("RK_GD_VMAX_MS", config.max_velocity_m_s);
		config.max_tilt_deg = read_param_float("RK_GD_TILT_MAX", config.max_tilt_deg);

		int32_t sim_gt = config.sim_groundtruth_fallback;
		param_t handle = param_find("RK_GD_SIM_GT");

		if (handle != PARAM_INVALID) {
			param_get(handle, &sim_gt);
		}

		config.sim_groundtruth_fallback = sim_gt;
		_fsm.set_waypoint_config(config);
	}

	GuidanceFsm _fsm{};

	vehicle_attitude_s _attitude{};
	vehicle_local_position_s _local_position{};
	vehicle_local_position_s _groundtruth_position{};
	tv3_lc_eng_st_s _engine_state{};
	tv3_sm_modes_s _module_modes{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _module_modes_sub{ORB_ID(tv3_sm_modes)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _groundtruth_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _engine_state_sub{ORB_ID(tv3_lc_eng_st)};
	uORB::Publication<tv3_gd_att_sp_s> _attitude_setpoint_pub{ORB_ID(tv3_gd_att_sp)};
	uORB::Publication<tv3_gd_thr_sp_s> _thrust_setpoint_pub{ORB_ID(tv3_gd_thr_sp)};
};

extern "C" __EXPORT int tv3_guidance_main(int argc, char *argv[])
{
	return TV3Guidance::main(argc, argv);
}
