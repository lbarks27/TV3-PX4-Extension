#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <matrix/matrix/math.hpp>

#include <cstdio>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_allocator_status.h>
#include <uORB/topics/tv3_control_authority.h>
#include <uORB/topics/tv3_engine_command.h>
#include <uORB/topics/tv3_engine_state.h>
#include <uORB/topics/tv3_gimbal_command.h>
#include <uORB/topics/tv3_guidance_status.h>
#include <uORB/topics/tv3_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include "tv3_gimbal_authority.hpp"
#include "tv3_gimbal_lm.hpp"
#include "tv3_gimbal_plant.hpp"

using namespace time_literals;
using matrix::Vector3f;
using tv3::AchievableTorque;
using tv3::GimbalLimits;
using tv3::GimbalPlant;
using tv3::GimbalWrench;
using tv3::GimbalWrenchResult;
using tv3::LmConfig;
using tv3::LmSolveResult;
using tv3::TorqueScaleResult;
using tv3::torque_wrench_aligned;

namespace
{
constexpr int kMaxEngines = 4;

static float engine_chamber_thrust_n(const tv3_engine_state_s &engine_state, int engine_index)
{
	if (engine_index < 0 || engine_index >= tv3_engine_state_s::MAX_ENGINES) {
		return 0.f;
	}

	float thrust_n = engine_state.filtered_thrust_n[engine_index];

	if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
		thrust_n = engine_state.measured_thrust_n[engine_index];
	}

	if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
		thrust_n = engine_state.expected_thrust_n[engine_index];
	}

	return PX4_ISFINITE(thrust_n) ? math::max(thrust_n, 0.f) : 0.f;
}

static float collective_throttle_yaw_deg(float desired_net_thrust_n, float total_chamber_thrust_n, float throttle_max_deg)
{
	if (total_chamber_thrust_n < 1.f) {
		return 0.f;
	}

	if (desired_net_thrust_n <= 0.f) {
		// Secondary-axis splay can remove net axial thrust (>= 90 deg available on the lander).
		return math::constrain(90.f, 0.f, throttle_max_deg);
	}

	if (desired_net_thrust_n >= total_chamber_thrust_n - 1e-3f) {
		return 0.f;
	}

	const float ratio = math::constrain(desired_net_thrust_n / total_chamber_thrust_n, 0.f, 1.f);
	const float throttle_rad = acosf(ratio);
	const float throttle_deg = throttle_rad * 57.2957795f;
	return math::constrain(throttle_deg, 0.f, throttle_max_deg);
}

static uint8_t engine_bit(int engine_index)
{
	return engine_index >= 0 && engine_index < 8 ? static_cast<uint8_t>(1u << engine_index) : 0;
}

static GimbalWrenchResult neutral_baseline_wrench(const GimbalPlant &plant,
		const float thrust_n[kMaxEngines],
		int ignition_mask)
{
	float zero_primary[kMaxEngines]{};
	float zero_yaw[kMaxEngines]{};
	return plant.total_wrench(zero_primary, zero_yaw, thrust_n, ignition_mask);
}

static void gimbal_angles_from_engine_command(const tv3_engine_command_s &engine_command, int engine_count,
		float primary_rad[kMaxEngines], float yaw_rad[kMaxEngines])
{
	for (int i = 0; i < engine_count; ++i) {
		primary_rad[i] = math::radians(engine_command.commanded_pitch_deg[i]);
		yaw_rad[i] = math::radians(engine_command.commanded_yaw_deg[i]);
	}
}

static void fill_allocator_wrench_fields(tv3_allocator_status_s &status,
		const GimbalPlant &plant,
		const float primary_rad[kMaxEngines],
		const float yaw_rad[kMaxEngines],
		const float thrust_n[kMaxEngines],
		int ignition_mask,
		const Vector3f &demanded_torque_nm,
		float demanded_axial_thrust_n)
{
	const GimbalWrenchResult at_command = plant.total_wrench(primary_rad, yaw_rad, thrust_n, ignition_mask);
	const GimbalWrenchResult at_neutral = neutral_baseline_wrench(plant, thrust_n, ignition_mask);
	const Vector3f incremental_torque_nm = at_command.torque_nm - at_neutral.torque_nm;
	const Vector3f incremental_force_n = at_command.body_force_n - at_neutral.body_force_n;

	status.demanded_torque_nm[0] = demanded_torque_nm(0);
	status.demanded_torque_nm[1] = demanded_torque_nm(1);
	status.demanded_torque_nm[2] = demanded_torque_nm(2);
	status.achieved_torque_nm[0] = incremental_torque_nm(0);
	status.achieved_torque_nm[1] = incremental_torque_nm(1);
	status.achieved_torque_nm[2] = incremental_torque_nm(2);
	status.demanded_body_force_n[0] = demanded_axial_thrust_n;
	status.demanded_body_force_n[1] = 0.f;
	status.demanded_body_force_n[2] = 0.f;
	status.achieved_body_force_n[0] = incremental_force_n(0);
	status.achieved_body_force_n[1] = incremental_force_n(1);
	status.achieved_body_force_n[2] = incremental_force_n(2);
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

		PRINT_MODULE_DESCRIPTION("TV3 control mixer mapping allocator servos and collective splay onto per-engine gimbal commands.");
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

		_tv3_status_sub.copy(&_status);

		if (_status.timestamp == 0) {
			return;
		}

		_actuator_servos_sub.update(&_actuator_servos);
		_tv3_gimbal_command_sub.update(&_tv3_gimbal_command);
		_torque_setpoint_sub.update(&_torque_sp);
		_tv3_engine_state_sub.update(&_engine_state);
		_tv3_guidance_status_sub.update(&_guidance_status);

		publish_engine_command(hrt_absolute_time());
	}

	float active_chamber_thrust_n() const
	{
		float total_chamber_thrust_n = 0.f;
		const int engine_count = math::min(static_cast<int>(_status.engine_count), kMaxEngines);

		for (int i = 0; i < engine_count; ++i) {
			if (_status.ignition_mask & engine_bit(i)) {
				total_chamber_thrust_n += engine_chamber_thrust_n(_engine_state, i);
			}
		}

		return total_chamber_thrust_n;
	}

	float collective_throttle_max_deg() const
	{
		float throttle_max_deg = 0.f;
		const int engine_count = math::min(static_cast<int>(_status.engine_count), kMaxEngines);

		for (int i = 0; i < engine_count; ++i) {
			throttle_max_deg = math::max(throttle_max_deg, _engine_yaw_max_deg[i]);
		}

		if (throttle_max_deg <= 0.f) {
			throttle_max_deg = _splay_max_deg;
		}

		return throttle_max_deg;
	}

	bool powered_boost_active() const
	{
		if (!_status.ignition_on) {
			return false;
		}

		return _status.mode == tv3_status_s::MODE_IGNITION_PENDING
		       || _status.mode == tv3_status_s::MODE_BOOST;
	}

	bool hover_window_throttle_active() const
	{
		if (_guidance_enabled <= 0 || !powered_boost_active()) {
			return false;
		}

		return _guidance_status.timestamp != 0
		       && _guidance_status.ascent_mode == tv3_guidance_status_s::ASCENT_HOVER_WINDOW
		       && _guidance_status.thrust_solution_valid
		       && _guidance_status.control_solution_valid;
	}

	bool boost_tvc_pitch_limit_active() const
	{
		return powered_boost_active() && !hover_window_throttle_active();
	}

	bool boost_attitude_only_active() const
	{
		if (_boost_attitude_only <= 0) {
			return false;
		}

		return powered_boost_active();
	}

	bool powered_gimbal_mixing() const
	{
		const bool powered = _status.mode == tv3_status_s::MODE_IGNITION_PENDING
				     || _status.mode == tv3_status_s::MODE_BOOST
				     || _status.mode == tv3_status_s::MODE_COAST;

		return powered && _status.ignition_on;
	}

	bool lm_allocator_active() const
	{
		if (boost_attitude_only_active()) {
			if (_status.engine_count <= 1 || !mixing_active()) {
				return false;
			}

			return powered_gimbal_mixing() && _status.sequence_complete
			       && (_status.rail_exit || powered_boost_active())
			       && active_chamber_thrust_n() >= 1.f;
		}

		return collective_throttle_mixer_active();
	}

	float desired_axial_thrust_n() const
	{
		if (boost_attitude_only_active()) {
			return active_chamber_thrust_n();
		}

		if (hover_window_throttle_active()) {
			return math::max(_guidance_status.required_thrust_n, 0.f);
		}

		if (powered_boost_active()) {
			return active_chamber_thrust_n();
		}

		return _guidance_status.required_thrust_n;
	}

	bool collective_throttle_mixer_active() const
	{
		if (_guidance_enabled <= 0 || _status.engine_count <= 1 || collective_throttle_max_deg() <= 0.f) {
			return false;
		}

		const bool powered = _status.mode == tv3_status_s::MODE_IGNITION_PENDING
				     || _status.mode == tv3_status_s::MODE_BOOST
				     || _status.mode == tv3_status_s::MODE_COAST;

		if (!powered || !_status.ignition_on) {
			return false;
		}

		if (_guidance_status.timestamp == 0 || !_guidance_status.thrust_solution_valid
		    || !_guidance_status.control_solution_valid) {
			return false;
		}

		return active_chamber_thrust_n() >= 1.f;
	}

	float clip_pitch_deg(int engine_index, float pitch_deg) const
	{
		if (engine_index < 0 || engine_index >= kMaxEngines) {
			return 0.f;
		}

		return math::constrain(pitch_deg, -_engine_pitch_max_deg[engine_index], _engine_pitch_max_deg[engine_index]);
	}

	float clip_yaw_deg(int engine_index, float yaw_deg) const
	{
		if (engine_index < 0 || engine_index >= kMaxEngines) {
			return 0.f;
		}

		return math::constrain(yaw_deg, math::degrees(_group_ymin_rad[engine_index]),
				       math::degrees(_group_ymax_rad[engine_index]));
	}

	void clip_engine_command_degrees(tv3_engine_command_s &engine_command, int engine_count) const
	{
		for (int i = 0; i < engine_count; ++i) {
			engine_command.commanded_pitch_deg[i] = clip_pitch_deg(i, engine_command.commanded_pitch_deg[i]);
			engine_command.commanded_yaw_deg[i] = clip_yaw_deg(i, engine_command.commanded_yaw_deg[i]);
		}
	}

	float update_collective_throttle_yaw_deg() const
	{
		if (boost_attitude_only_active()) {
			return 0.f;
		}

		if (powered_boost_active() && !hover_window_throttle_active()) {
			return 0.f;
		}

		if (!collective_throttle_mixer_active()) {
			return 0.f;
		}

		const float chamber = active_chamber_thrust_n();
		const float required = _guidance_status.required_thrust_n;

		return collective_throttle_yaw_deg(required, chamber, collective_throttle_max_deg());
	}

	bool mixing_active() const
	{
		if (_status.mode < tv3_status_s::MODE_READY || _status.mode == tv3_status_s::MODE_ABORT) {
			return false;
		}

		const bool coast_without_guidance = _status.mode == tv3_status_s::MODE_COAST && _guidance_enabled <= 0;

		return !coast_without_guidance;
	}

	void publish_engine_command(hrt_abstime now)
	{
		tv3_engine_command_s engine_command{};
		engine_command.timestamp = now;
		engine_command.engine_count = _status.engine_count;
		engine_command.ignition_mask = _status.ignition_mask;
		engine_command.active_ignition_index = _status.active_ignition_index;
		engine_command.sequence_active = _status.sequence_active;
		engine_command.sequence_complete = _status.sequence_complete;

		for (int i = 0; i < kMaxEngines; ++i) {
			engine_command.selected_motor_index[i] = _engine_state.selected_motor_index[i];
		}

		if (!mixing_active()) {
			_engine_command_pub.publish(engine_command);
			return;
		}

		const int engine_count = math::min(static_cast<int>(_status.engine_count), kMaxEngines);

		for (int i = 0; i < engine_count; ++i) {
			const int pidx = 2 * i;

			if (pidx < actuator_servos_s::NUM_CONTROLS) {
				const float p = _actuator_servos.control[pidx];

				if (PX4_ISFINITE(p) && fabsf(p) > 1e-4f) {
					float pd = p * _engine_pitch_max_deg[i];
					if (boost_tvc_pitch_limit_active()) {
						pd = math::constrain(pd, -8.f, 8.f);
					}
					engine_command.commanded_pitch_deg[i] = pd;
				}
			}

			const int yidx = 2 * i + 1;

			if (yidx < actuator_servos_s::NUM_CONTROLS) {
				const float y = _actuator_servos.control[yidx];

				if (PX4_ISFINITE(y) && fabsf(y) > 1e-4f) {
					float yd = y * _engine_yaw_max_deg[i];
					if (boost_tvc_pitch_limit_active()) {
						yd = math::constrain(yd, -8.f, 8.f);
					}
					engine_command.commanded_yaw_deg[i] = clip_yaw_deg(i, yd);
				}
			}
		}

		clip_engine_command_degrees(engine_command, engine_count);

		// During non-hover boost, keep TVC small on both axes. Hover-window ascent keeps full
		// secondary-axis travel for collective splay throttling.
		if (boost_tvc_pitch_limit_active()) {
			for (int i = 0; i < engine_count; ++i) {
				engine_command.commanded_pitch_deg[i] = math::constrain(engine_command.commanded_pitch_deg[i], -8.f, 8.f);
				engine_command.commanded_yaw_deg[i] = math::constrain(engine_command.commanded_yaw_deg[i], -8.f, 8.f);
			}
		} else if (hover_window_throttle_active()) {
			for (int i = 0; i < engine_count; ++i) {
				engine_command.commanded_pitch_deg[i] = math::constrain(engine_command.commanded_pitch_deg[i], -8.f, 8.f);
			}
		}

		bool have_primary = false;

		for (int i = 0; i < engine_count; ++i) {
			if (fabsf(engine_command.commanded_pitch_deg[i]) > 1e-4f) {
				have_primary = true;
				break;
			}
		}

		if (!have_primary && _tv3_gimbal_command.timestamp > 0) {
			const int n = math::min(static_cast<int>(_tv3_gimbal_command.engine_count), engine_count);

			for (int i = 0; i < n; ++i) {
				engine_command.commanded_pitch_deg[i] = _tv3_gimbal_command.commanded_pitch_deg[i];
			}
		}

		const bool rail_or_boost_ready = _status.rail_exit || powered_boost_active();
		const bool boost_lm_only = powered_boost_active() && !hover_window_throttle_active();
		const int mask = _status.ignition_mask;
		float thr[kMaxEngines]{};

		for (int i = 0; i < engine_count; ++i) {
			thr[i] = engine_chamber_thrust_n(_engine_state, i);
		}

		GimbalPlant plant{};
		plant.engine_count = engine_count;
		GimbalLimits limits{};
		float tvc_limit_deg = math::max(_tvc_max_deg, 0.f);

		for (int i = 0; i < engine_count; ++i) {
			plant.geometry.pos[i] = _group_pos[i];
			plant.geometry.thrust_axis[i] = _group_thrust[i];
			plant.geometry.primary_axis[i] = _group_primary[i];
			plant.geometry.secondary_axis[i] = _group_secondary[i];
		}

		const float tvc_max_rad = math::radians(tvc_limit_deg);
		constexpr float boost_tvc_limit_deg = 8.0f;
		const float boost_tvc_rad = math::radians(boost_tvc_limit_deg);

		for (int i = 0; i < engine_count; ++i) {
			if (hover_window_throttle_active()) {
				const float lim = (tvc_max_rad > 0.f) ? fminf(tvc_max_rad, boost_tvc_rad) : boost_tvc_rad;
				limits.primary_min_rad[i] = -lim;
				limits.primary_max_rad[i] = lim;
				limits.yaw_min_rad[i] = _group_ymin_rad[i];
				limits.yaw_max_rad[i] = _group_ymax_rad[i];
				tvc_limit_deg = boost_tvc_limit_deg;
			} else if (powered_boost_active()) {
				const float lim = (tvc_max_rad > 0.f) ? fminf(tvc_max_rad, boost_tvc_rad) : boost_tvc_rad;
				limits.primary_min_rad[i] = -lim;
				limits.primary_max_rad[i] = lim;
				limits.yaw_min_rad[i] = -lim;
				limits.yaw_max_rad[i] = lim;
				tvc_limit_deg = boost_tvc_limit_deg;
			} else {
				limits.primary_min_rad[i] = -_group_pmax_rad[i];
				limits.primary_max_rad[i] = _group_pmax_rad[i];
				limits.yaw_min_rad[i] = _group_ymin_rad[i];
				limits.yaw_max_rad[i] = _group_ymax_rad[i];
			}
		}

		const bool publish_authority = mixing_active()
					       && (_status.mode == tv3_status_s::MODE_IGNITION_PENDING
						   || _status.mode == tv3_status_s::MODE_BOOST
						   || _status.mode == tv3_status_s::MODE_COAST);

		if (publish_authority) {
			tv3_control_authority_s authority{};
			authority.timestamp = now;
			authority.valid = true;
			authority.tvc_limit_deg = tvc_limit_deg;

			if (_status.ignition_on && active_chamber_thrust_n() >= 1.f) {
				const AchievableTorque achievable = estimate_achievable_torque(plant, thr, mask, limits);
				authority.achievable_torque_positive_nm[0] = achievable.positive_nm(0);
				authority.achievable_torque_positive_nm[1] = achievable.positive_nm(1);
				authority.achievable_torque_positive_nm[2] = achievable.positive_nm(2);
				authority.achievable_torque_negative_nm[0] = achievable.negative_nm(0);
				authority.achievable_torque_negative_nm[1] = achievable.negative_nm(1);
				authority.achievable_torque_negative_nm[2] = achievable.negative_nm(2);
			}

			_control_authority_pub.publish(authority);
		}

		// LM re-solve uses live chamber thrust and CA servo seeds. During boost we solve torque-only
		// (no collective splay, thrust_weight=0); after boost/coast the full thrust+splay problem runs.
		const bool can_apply_lm = lm_allocator_active() && _status.sequence_complete && rail_or_boost_ready;

		if (can_apply_lm) {
			const float splay_deg = boost_lm_only ? 0.f : update_collective_throttle_yaw_deg();
			const float splay_rad = math::radians(splay_deg);
			float p_rad[4] = {};
			float y_alloc_rad[4] = {};

			for (int i = 0; i < engine_count; ++i) {
				p_rad[i] = math::radians(engine_command.commanded_pitch_deg[i]);
				y_alloc_rad[i] = math::radians(engine_command.commanded_yaw_deg[i]);
			}

			float init_p[4] = {};
			float init_y[4] = {};
			const bool use_warm_start = _prev_warm_valid && mask == _prev_mask;

			for (int i = 0; i < engine_count; ++i) {
				if (use_warm_start) {
					init_p[i] = _prev_primary_rad[i];
					init_y[i] = _prev_yaw_rad[i];
				} else {
					init_p[i] = p_rad[i];
					init_y[i] = boost_lm_only ? y_alloc_rad[i] : (y_alloc_rad[i] + splay_rad);
				}
			}

			GimbalWrench desired{};
			const float torque_roll = math::constrain(_torque_sp.xyz[0], -_torque_roll_max, _torque_roll_max);
			const float torque_pitch = math::constrain(_torque_sp.xyz[1], -_torque_pitch_max, _torque_pitch_max);
			const float torque_yaw = math::constrain(_torque_sp.xyz[2], -_torque_yaw_max, _torque_yaw_max);
			const bool torque_saturated = fabsf(torque_roll - _torque_sp.xyz[0]) > 1e-3f
						      || fabsf(torque_pitch - _torque_sp.xyz[1]) > 1e-3f
						      || fabsf(torque_yaw - _torque_sp.xyz[2]) > 1e-3f;
			desired.torque_nm(0) = torque_roll;
			desired.torque_nm(1) = torque_pitch;
			desired.torque_nm(2) = torque_yaw;
			desired.axial_thrust_n = desired_axial_thrust_n();

			LmConfig lm_config{};
			lm_config.max_iter = _alloc_max_iter;
			lm_config.torque_tol_nm = _alloc_tol_nm;
			lm_config.lambda0 = _alloc_lambda0;
			lm_config.thrust_weight = _alloc_thr_weight;
			lm_config.splay_weight = _alloc_splay_weight;
			lm_config.fd_eps = _alloc_fd_eps;

			if (boost_lm_only) {
				lm_config.splay_weight = 0.f;
				lm_config.thrust_weight = 0.f;
			}

			const float torque_demand_nm = sqrtf(torque_roll * torque_roll + torque_pitch * torque_pitch
							     + torque_yaw * torque_yaw);
			constexpr float kBoostNeutralTorqueNm = 0.05f;
			LmSolveResult lm_result{};

			// Asymmetric chamber thrust can produce bias torque at neutral gimbals. When the
			// attitude loop is not commanding TVC, hold zero deflection instead of chasing plant bias.
			if (boost_lm_only && torque_demand_nm < kBoostNeutralTorqueNm) {
				for (int i = 0; i < kMaxEngines; ++i) {
					lm_result.primary_rad[i] = 0.f;
					lm_result.yaw_rad[i] = 0.f;
				}

				lm_result.converged = true;
				lm_result.iterations_used = 0;
				lm_result.residual_torque_nm = 0.f;
				lm_result.residual_thrust_n = 0.f;
				lm_result.cost = 0.f;
				lm_result.lambda_final = lm_config.lambda0;
			} else {
				lm_result = solve_gimbal_lm(plant, thr, mask, desired, init_p, init_y, limits, lm_config);
			}

			float output_primary_rad[kMaxEngines]{};
			float output_yaw_rad[kMaxEngines]{};
			bool used_fallback = false;
			bool torque_direction_valid = false;
			Vector3f achieved_torque_nm{};

			bool solution_accepted = false;

			if (lm_result.converged && torque_demand_nm < kBoostNeutralTorqueNm) {
				solution_accepted = true;
				torque_direction_valid = true;

				for (int i = 0; i < engine_count; ++i) {
					output_primary_rad[i] = 0.f;
					output_yaw_rad[i] = 0.f;
				}
			} else if (lm_result.converged) {
				for (int i = 0; i < engine_count; ++i) {
					output_primary_rad[i] = lm_result.primary_rad[i];
					output_yaw_rad[i] = lm_result.yaw_rad[i];
				}

				const GimbalWrenchResult achieved_wrench = plant.total_wrench(output_primary_rad, output_yaw_rad, thr, mask);
				achieved_torque_nm = achieved_wrench.torque_nm;
				torque_direction_valid = torque_wrench_aligned(desired.torque_nm, achieved_torque_nm);
				solution_accepted = torque_direction_valid;

				if (solution_accepted) {
					_prev_warm_valid = true;

					for (int i = 0; i < kMaxEngines; ++i) {
						_prev_primary_rad[i] = lm_result.primary_rad[i];
						_prev_yaw_rad[i] = lm_result.yaw_rad[i];
					}
				}
			}

			if (!solution_accepted) {
				used_fallback = true;

				if (use_warm_start) {
					for (int i = 0; i < engine_count; ++i) {
						output_primary_rad[i] = _prev_primary_rad[i];
						output_yaw_rad[i] = _prev_yaw_rad[i];
					}
				} else {
					for (int i = 0; i < engine_count; ++i) {
						output_primary_rad[i] = init_p[i];
						output_yaw_rad[i] = init_y[i];
						output_primary_rad[i] = math::constrain(output_primary_rad[i], limits.primary_min_rad[i],
											limits.primary_max_rad[i]);
						output_yaw_rad[i] = math::constrain(output_yaw_rad[i], limits.yaw_min_rad[i], limits.yaw_max_rad[i]);
					}

					_prev_warm_valid = false;
				}

				const GimbalWrenchResult achieved_wrench = plant.total_wrench(output_primary_rad, output_yaw_rad, thr, mask);
				achieved_torque_nm = achieved_wrench.torque_nm;
				torque_direction_valid = torque_wrench_aligned(desired.torque_nm, achieved_torque_nm);
			}

			float mean_yaw_rad = 0.f;
			int active_count = 0;

			for (int i = 0; i < engine_count; ++i) {
				if ((mask & engine_bit(i)) && thr[i] > 0.5f) {
					mean_yaw_rad += output_yaw_rad[i];
					active_count++;
				}
			}

			if (active_count > 0) {
				mean_yaw_rad /= static_cast<float>(active_count);
			}

			const float mean_splay_deg = boost_lm_only ? 0.f : math::degrees(mean_yaw_rad);

			for (int i = 0; i < engine_count; ++i) {
				if (_status.ignition_mask & engine_bit(i)) {
					engine_command.commanded_pitch_deg[i] = math::degrees(output_primary_rad[i]);
					engine_command.commanded_yaw_deg[i] = math::degrees(output_yaw_rad[i]);
					engine_command.commanded_splay_deg[i] = mean_splay_deg;
				}
			}

			clip_engine_command_degrees(engine_command, engine_count);
			_prev_mask = mask;

			const GimbalWrenchResult final_wrench = plant.total_wrench(output_primary_rad, output_yaw_rad, thr, mask);
			achieved_torque_nm = final_wrench.torque_nm;

			const AchievableTorque achievable = estimate_achievable_torque(plant, thr, mask, limits);
			const TorqueScaleResult authority_scale = scale_torque_preserve_direction(desired.torque_nm, achievable);

			tv3_allocator_status_s allocator_status{};
			allocator_status.timestamp = now;
			allocator_status.iterations_used = static_cast<uint8_t>(lm_result.iterations_used);
			allocator_status.lm_path_active = true;
			allocator_status.converged = solution_accepted;
			allocator_status.demand_saturated = !solution_accepted || torque_saturated || authority_scale.saturated;
			allocator_status.used_fallback_solution = used_fallback;
			allocator_status.torque_direction_valid = torque_direction_valid;
			allocator_status.torque_authority_scale = authority_scale.scale;
			fill_allocator_wrench_fields(allocator_status, plant, output_primary_rad, output_yaw_rad, thr, mask,
						     desired.torque_nm, desired.axial_thrust_n);
			allocator_status.residual_torque_nm = lm_result.residual_torque_nm;
			allocator_status.residual_thrust_n = lm_result.residual_thrust_n;
			allocator_status.cost = lm_result.cost;
			allocator_status.lambda_final = lm_result.lambda_final;
			_allocator_status_pub.publish(allocator_status);
		} else {
			_prev_warm_valid = false;

			for (int i = 0; i < engine_count; ++i) {
				engine_command.commanded_splay_deg[i] = 0.f;
			}

			clip_engine_command_degrees(engine_command, engine_count);

			// For pure boost (direct torque path) or other non-splay cases, publish a lightweight
			// allocator status so review logs and tools that expect the topic still have it.
			// Indicates "direct from CA servos" (no LM re-solve).
			if (powered_boost_active() || boost_attitude_only_active()) {
				const float torque_roll = math::constrain(_torque_sp.xyz[0], -_torque_roll_max, _torque_roll_max);
				const float torque_pitch = math::constrain(_torque_sp.xyz[1], -_torque_pitch_max, _torque_pitch_max);
				const float torque_yaw = math::constrain(_torque_sp.xyz[2], -_torque_yaw_max, _torque_yaw_max);
				const Vector3f demanded_torque_nm{torque_roll, torque_pitch, torque_yaw};
				float primary_rad[kMaxEngines]{};
				float yaw_rad[kMaxEngines]{};
				gimbal_angles_from_engine_command(engine_command, engine_count, primary_rad, yaw_rad);

				tv3_allocator_status_s st{};
				st.timestamp = now;
				st.iterations_used = 0;
				st.lm_path_active = false;
				st.converged = true;
				st.demand_saturated = false;
				st.used_fallback_solution = false;
				st.torque_direction_valid = true;
				st.torque_authority_scale = 1.f;
				fill_allocator_wrench_fields(st, plant, primary_rad, yaw_rad, thr, mask, demanded_torque_nm,
							     desired_axial_thrust_n());
				st.residual_torque_nm = 0.f;
				st.residual_thrust_n = 0.f;
				st.cost = 0.f;
				st.lambda_final = 0.f;
				_allocator_status_pub.publish(st);
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

		p = param_find("RK_SPLAY_MAX_DEG");

		if (p != PARAM_INVALID) {
			param_get(p, &_splay_max_deg);
		}

		p = param_find("RK_TVC_MAX_DEG");

		if (p != PARAM_INVALID) {
			param_get(p, &_tvc_max_deg);
		}

		const float tvc_limit_deg = math::max(_tvc_max_deg, 0.f);
		const float yaw_limit_deg = math::max(math::max(_splay_max_deg, tvc_limit_deg), 0.f);

		for (int i = 0; i < kMaxEngines; ++i) {
			char name[20];
			snprintf(name, sizeof(name), "CA_RK_G%u_PMAX", i);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				param_get(p, &_engine_pitch_max_deg[i]);
			}

			snprintf(name, sizeof(name), "CA_RK_G%u_YMAX", i);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				param_get(p, &_engine_yaw_max_deg[i]);
			}

			if (tvc_limit_deg > 0.f) {
				_engine_pitch_max_deg[i] = math::min(_engine_pitch_max_deg[i], tvc_limit_deg);
			}

			if (yaw_limit_deg > 0.f) {
				_engine_yaw_max_deg[i] = math::min(_engine_yaw_max_deg[i], yaw_limit_deg);
			}
		}

		for (int i = 0; i < kMaxEngines; ++i) {
			char buf[32];

			auto gf = [&](const char *suffix, float def) -> float {
				snprintf(buf, sizeof(buf), "CA_RK_G%u_%s", i, suffix);
				param_t h = param_find(buf);
				float v = def;

				if (h != PARAM_INVALID) {
					param_get(h, &v);
				}

				return v;
			};

			_group_pos[i](0) = gf("PX", 0.f);
			_group_pos[i](1) = gf("PY", 0.f);
			_group_pos[i](2) = gf("PZ", 0.f);
			_group_thrust[i](0) = gf("AX", 1.f);
			_group_thrust[i](1) = gf("AY", 0.f);
			_group_thrust[i](2) = gf("AZ", 0.f);
			_group_primary[i](0) = gf("PAX", 0.f);
			_group_primary[i](1) = gf("PAY", -1.f);
			_group_primary[i](2) = gf("PAZ", 0.f);
			_group_secondary[i](0) = gf("YAX", 0.f);
			_group_secondary[i](1) = gf("YAY", 0.f);
			_group_secondary[i](2) = gf("YAZ", -1.f);
			_group_pmax_rad[i] = math::radians(gf("PMAX", 5.f));
			_group_ymin_rad[i] = math::radians(gf("YMIN", 0.f));
			_group_ymax_rad[i] = math::radians(gf("YMAX", 5.f));

			if (tvc_limit_deg > 0.f) {
				_group_pmax_rad[i] = math::min(_group_pmax_rad[i], math::radians(tvc_limit_deg));
			}

			if (yaw_limit_deg > 0.f) {
				_group_ymax_rad[i] = math::min(_group_ymax_rad[i], math::radians(yaw_limit_deg));
			}
		}

		p = param_find("RK_GD_ENABLE");

		if (p != PARAM_INVALID) {
			param_get(p, &_guidance_enabled);
		}

		p = param_find("RK_GD_BOOST_ATT");

		if (p != PARAM_INVALID) {
			param_get(p, &_boost_attitude_only);
		}

		p = param_find("RK_ALLOC_MAX_ITER");

		if (p != PARAM_INVALID) {
			int32_t value = _alloc_max_iter;
			param_get(p, &value);
			_alloc_max_iter = math::constrain(value, static_cast<int32_t>(1), static_cast<int32_t>(16));
		}

		p = param_find("RK_ALLOC_TOL");

		if (p != PARAM_INVALID) {
			param_get(p, &_alloc_tol_nm);
		}

		p = param_find("RK_ALLOC_LAMBDA0");

		if (p != PARAM_INVALID) {
			param_get(p, &_alloc_lambda0);
		}

		p = param_find("RK_ALLOC_THR_W");

		if (p != PARAM_INVALID) {
			param_get(p, &_alloc_thr_weight);
		}

		p = param_find("RK_ALLOC_SPLAY_W");

		if (p != PARAM_INVALID) {
			param_get(p, &_alloc_splay_weight);
		}

		p = param_find("RK_ALLOC_FD_EPS");

		if (p != PARAM_INVALID) {
			param_get(p, &_alloc_fd_eps);
		}

		p = param_find("RK_TQ_R_MAX");

		if (p != PARAM_INVALID) {
			param_get(p, &_torque_roll_max);
		}

		p = param_find("RK_TQ_P_MAX");

		if (p != PARAM_INVALID) {
			param_get(p, &_torque_pitch_max);
		}

		p = param_find("RK_TQ_Y_MAX");

		if (p != PARAM_INVALID) {
			param_get(p, &_torque_yaw_max);
		}
	}

	int32_t _engine_count{1};
	int32_t _guidance_enabled{0};
	int32_t _boost_attitude_only{0};
	int32_t _alloc_max_iter{12};
	float _splay_max_deg{0.f};
	float _tvc_max_deg{35.f};
	float _alloc_tol_nm{0.15f};
	float _alloc_lambda0{0.01f};
	float _alloc_thr_weight{1.f};
	float _alloc_splay_weight{0.1f};
	float _alloc_fd_eps{0.01f};
	float _torque_roll_max{8.f};
	float _torque_pitch_max{16.f};
	float _torque_yaw_max{16.f};
	float _engine_pitch_max_deg[kMaxEngines]{};
	float _engine_yaw_max_deg[kMaxEngines]{};
	Vector3f _group_pos[kMaxEngines]{};
	Vector3f _group_thrust[kMaxEngines]{};
	Vector3f _group_primary[kMaxEngines]{};
	Vector3f _group_secondary[kMaxEngines]{};
	float _group_pmax_rad[kMaxEngines]{};
	float _group_ymin_rad[kMaxEngines]{};
	float _group_ymax_rad[kMaxEngines]{};
	float _prev_primary_rad[kMaxEngines]{};
	float _prev_yaw_rad[kMaxEngines]{};
	int _prev_mask{0};
	bool _prev_warm_valid{false};

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
	uORB::Publication<tv3_allocator_status_s> _allocator_status_pub{ORB_ID(tv3_allocator_status)};
	uORB::Publication<tv3_control_authority_s> _control_authority_pub{ORB_ID(tv3_control_authority)};
};

extern "C" __EXPORT int tv3_control_mixer_main(int argc, char *argv[])
{
	return TV3ControlMixer::main(argc, argv);
}
