#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_mix_alloc_st.h>
#include <uORB/topics/tv3_mix_eng_cmd.h>
#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include "lib/tv3_engine_geometry.hpp"
#include "lib/tv3_msg_fields.hpp"

#include "tv3_control_mixer_core.hpp"
#include "tv3_control_mixer_fsm.hpp"

using namespace time_literals;
using matrix::Vector3f;
using tv3::EngineGeometry;
using tv3::ControlMixerCore;
using tv3::ControlMixerFsm;
using tv3::ControlMixerGeometry;
using tv3::ControlMixerLmTuning;
using tv3::ControlMixerRunInput;
using tv3::filtered_thrust_n;
using tv3::load_engine_geometry;
using tv3::measured_thrust_n;
using tv3::expected_thrust_n;
using tv3::selected_motor_index;
using tv3::read_param_float;

namespace
{
constexpr int kMaxEngines = tv3::kControlMixerMaxEngines;

static float engine_thrust_from_state(const tv3_lc_eng_st_s &state, int index)
{
	if (index < 0 || index >= tv3_lc_eng_st_s::MAX_ENGINES) {
		return 0.f;
	}

	float thrust_n = filtered_thrust_n(state, index);

	if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
		thrust_n = measured_thrust_n(state, index);
	}

	if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
		thrust_n = expected_thrust_n(state, index);
	}

	return PX4_ISFINITE(thrust_n) ? math::max(thrust_n, 0.f) : 0.f;
}
}

class TV3ControlMixer : public ModuleBase<TV3ControlMixer>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3ControlMixer() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3ControlMixer *instance = new TV3ControlMixer();

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

		PRINT_MODULE_DESCRIPTION("TV3 control mixer: torque setpoints to per-engine gimbal commands.");
		PRINT_MODULE_USAGE_NAME("tv3_control_mixer", "modules");
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

		_engine_state_sub.copy(&_engine_state);

		if (_engine_state.timestamp == 0) {
			return;
		}

		_module_modes_sub.copy(&_module_modes);
		_torque_setpoint_sub.update(&_torque_sp);

		_mixer_fsm.apply_module_mode(_module_modes);

		ControlMixerRunInput input{};
		input.module_modes = _module_modes;
		input.engine_state = _engine_state;
		input.torque_sp = _torque_sp;
		input.torque_roll_max = _torque_roll_max;
		input.torque_pitch_max = _torque_pitch_max;
		input.torque_yaw_max = _torque_yaw_max;

		const int engine_count = math::min(static_cast<int>(_engine_state.engine_count), kMaxEngines);

		for (int i = 0; i < engine_count; ++i) {
			input.engine_thrust_n[i] = engine_thrust_from_state(_engine_state, i);
		}

		for (int i = 0; i < kMaxEngines; ++i) {
			input.selected_motor_index[i] = selected_motor_index(_engine_state, i);
		}

		const auto output = _mixer_fsm.run(_mixer_core, input, hrt_absolute_time());
		_engine_command_pub.publish(output.engine_command);

		if (output.publish_allocator_status) {
			_allocator_status_pub.publish(output.allocator_status);
		}
	}

	void update_parameters()
	{
		const float tvc_limit_deg = read_param_float("RK_TVC_MAX_DEG", 5.f);
		const float splay_max_deg = read_param_float("RK_SPLAY_MAX_DEG", 5.f);

		EngineGeometry geometry{};
		load_engine_geometry(geometry, tvc_limit_deg, splay_max_deg);

		ControlMixerGeometry mixer_geometry{};
		mixer_geometry.engine_count = geometry.engine_count;
		mixer_geometry.body_com(0) = read_param_float("RK_BODY_COM_X_M", 0.95f);

		for (int i = 0; i < kMaxEngines; ++i) {
			const auto &engine = geometry.engines[i];
			mixer_geometry.group_pos[i] = engine.position;
			mixer_geometry.group_thrust[i] = engine.thrust_axis;
			mixer_geometry.group_primary[i] = engine.pitch_axis;
			mixer_geometry.group_secondary[i] = engine.yaw_axis;
			mixer_geometry.group_pmax_rad[i] = engine.pitch_max_rad;
			mixer_geometry.group_ymin_rad[i] = engine.yaw_min_rad;
			mixer_geometry.group_ymax_rad[i] = engine.yaw_max_rad;
		}

		_mixer_core.set_geometry(mixer_geometry);

		ControlMixerLmTuning tuning{};
		int32_t max_iter = tuning.max_iter;
		param_t p = param_find("RK_ALC_MAX_ITR");

		if (p != PARAM_INVALID) {
			param_get(p, &max_iter);
			tuning.max_iter = math::constrain(max_iter, static_cast<int32_t>(1), static_cast<int32_t>(16));
		}

		tuning.tol_nm = read_param_float("RK_ALLOC_TOL", tuning.tol_nm);
		tuning.lambda0 = read_param_float("RK_ALLOC_LAMBDA0", tuning.lambda0);
		tuning.fd_eps = read_param_float("RK_ALLOC_FD_EPS", tuning.fd_eps);
		_mixer_core.set_lm_tuning(tuning);

		_torque_roll_max = read_param_float("RK_TQ_R_MAX", _torque_roll_max);
		_torque_pitch_max = read_param_float("RK_TQ_P_MAX", _torque_pitch_max);
		_torque_yaw_max = read_param_float("RK_TQ_Y_MAX", _torque_yaw_max);
	}

	float _torque_roll_max{8.f};
	float _torque_pitch_max{16.f};
	float _torque_yaw_max{16.f};

	ControlMixerCore _mixer_core{};
	ControlMixerFsm _mixer_fsm{};

	tv3_lc_eng_st_s _engine_state{};
	tv3_sm_modes_s _module_modes{};
	vehicle_torque_setpoint_s _torque_sp{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _module_modes_sub{ORB_ID(tv3_sm_modes)};
	uORB::Subscription _engine_state_sub{ORB_ID(tv3_lc_eng_st)};
	uORB::Subscription _torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<tv3_mix_eng_cmd_s> _engine_command_pub{ORB_ID(tv3_mix_eng_cmd)};
	uORB::Publication<tv3_mix_alloc_st_s> _allocator_status_pub{ORB_ID(tv3_mix_alloc_st)};
};

extern "C" __EXPORT int tv3_control_mixer_main(int argc, char *argv[])
{
	return TV3ControlMixer::main(argc, argv);
}
