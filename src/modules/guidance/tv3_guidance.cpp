#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_guidance_status.h>
#include <uORB/topics/tv3_motor_reference.h>
#include <uORB/topics/tv3_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <cmath>

using namespace time_literals;
using matrix::Vector3f;

namespace
{
constexpr float kDegToRad = 0.017453292519943295769f;
constexpr float kGravityMps2 = 9.80665f;
constexpr float kLandingDeltaVMargin = 1.15f;
constexpr float kAbortDeltaVMargin = 1.2f;
constexpr size_t kMissionWaypointCount = 3;

static constexpr uint8_t kAscentLaunch = 0;
static constexpr uint8_t kAscentHoverWindow = 1;
static constexpr uint8_t kApogeeTrack = 0;
static constexpr uint8_t kApogeeSkip = 1;
static constexpr uint8_t kLandingApproach = 0;
static constexpr uint8_t kLandingSkip = 1;
static constexpr uint8_t kWaypointFlyThrough = 0;
static constexpr uint8_t kWaypointPositionHold = 1;

struct MissionPoint {
	Vector3f position{};
	float acceptance_radius_m{15.f};
	float cruise_speed_m_s{25.f};
	uint8_t mode{kWaypointFlyThrough};
	float hold_time_s{0.f};
	bool hold_active{false};
	hrt_abstime hold_start{0};
};

static void set_nan_trajectory(trajectory_setpoint_s &sp)
{
	for (float &value : sp.position) {
		value = NAN;
	}

	for (float &value : sp.velocity) {
		value = NAN;
	}

	for (float &value : sp.acceleration) {
		value = NAN;
	}

	for (float &value : sp.jerk) {
		value = NAN;
	}

	sp.yaw = NAN;
	sp.yawspeed = NAN;
}

static Vector3f make_vector(float x, float y, float z)
{
	return Vector3f{x, y, z};
}

static float read_param_float(const char *name, float fallback)
{
	param_t p = param_find(name);

	if (p != PARAM_INVALID) {
		param_get(p, &fallback);
	}

	return fallback;
}

static int32_t read_param_int32(const char *name, int32_t fallback)
{
	param_t p = param_find(name);

	if (p != PARAM_INVALID) {
		param_get(p, &fallback);
	}

	return fallback;
}
} // namespace

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

		PRINT_MODULE_DESCRIPTION("TV3 launch, waypoint, and landing guidance stack.");
		PRINT_MODULE_USAGE_NAME("tv3_guidance", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(20_ms);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("phase: %u waypoint: %u origin_valid: %d", (unsigned)_phase, (unsigned)_waypoint_index, _origin_valid);
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

		if (_parameter_update_sub.updated()) {
			parameter_update_s update{};
			_parameter_update_sub.copy(&update);
			update_parameters();
		}

		_vehicle_status_sub.update(&_vehicle_status);
		_local_position_sub.update(&_local_position);
		_groundtruth_position_sub.update(&_groundtruth_position);
		_home_position_sub.update(&_home_position);
		_motor_reference_sub.update(&_motor_reference);
		_tv3_status_sub.update(&_tv3_status);

		const hrt_abstime now = hrt_absolute_time();
		update_thrust_envelope();
		update_origin(now);
		update_phase(now);

		if (guidance_position_valid(now)) {
			update_commanded_thrust(guidance_position(now));
		}

		publish_outputs(now);
	}

	void update_parameters()
	{
		_enabled = read_param_int32("RK_GD_ENABLE", _enabled);
		_splay_max_deg = read_param_float("RK_SPLAY_MAX_DEG", _splay_max_deg);
		_torque_pitch_max = read_param_float("RK_TQ_P_MAX", _torque_pitch_max);
		_torque_yaw_max = read_param_float("RK_TQ_Y_MAX", _torque_yaw_max);
		_takeoff_alt_m = read_param_float("RK_GD_TAKE_ALT", _takeoff_alt_m);
		_apex_alt_m = read_param_float("RK_GD_APEX_ALT", _apex_alt_m);
		_position_gain = read_param_float("RK_GD_POS_P", _position_gain);
		_max_velocity_m_s = read_param_float("RK_GD_VMAX_MS", _max_velocity_m_s);
		_max_climb_rate_m_s = read_param_float("RK_GD_VUP_MS", _max_climb_rate_m_s);
		_max_descent_rate_m_s = read_param_float("RK_GD_VDN_MS", _max_descent_rate_m_s);
		_fixed_yaw_deg = read_param_float("RK_GD_YAW_DEG", _fixed_yaw_deg);
		_hold_altitude_m = read_param_float("RK_GD_HOLD_ALT", _hold_altitude_m);
		_acceptance_radius_m = read_param_float("RK_GD_ACC_RAD", _acceptance_radius_m);
		_min_twr = read_param_float("RK_GD_TWR_MIN", _min_twr);
		_landing_twr = read_param_float("RK_GD_LAND_TWR", _landing_twr);
		_min_remaining_impulse_ns = read_param_float("RK_GD_MIN_IMP_NS", _min_remaining_impulse_ns);

		_mission_waypoints[0].position(0) = read_param_float("RK_GD_WP1_N", _mission_waypoints[0].position(0));
		_mission_waypoints[0].position(1) = read_param_float("RK_GD_WP1_E", _mission_waypoints[0].position(1));
		_mission_waypoints[0].position(2) = read_param_float("RK_GD_WP1_D", _mission_waypoints[0].position(2));
		_mission_waypoints[1].position(0) = read_param_float("RK_GD_WP2_N", _mission_waypoints[1].position(0));
		_mission_waypoints[1].position(1) = read_param_float("RK_GD_WP2_E", _mission_waypoints[1].position(1));
		_mission_waypoints[1].position(2) = read_param_float("RK_GD_WP2_D", _mission_waypoints[1].position(2));
		_mission_waypoints[2].position(0) = read_param_float("RK_GD_WP3_N", _mission_waypoints[2].position(0));
		_mission_waypoints[2].position(1) = read_param_float("RK_GD_WP3_E", _mission_waypoints[2].position(1));
		_mission_waypoints[2].position(2) = read_param_float("RK_GD_WP3_D", _mission_waypoints[2].position(2));

		_landing_point(0) = read_param_float("RK_GD_LAND_N", _landing_point(0));
		_landing_point(1) = read_param_float("RK_GD_LAND_E", _landing_point(1));
		_landing_point(2) = read_param_float("RK_GD_LAND_D", _landing_point(2));
		_sim_groundtruth_fallback = read_param_int32("RK_GD_SIM_GT", _sim_groundtruth_fallback);
		_ascent_mode = static_cast<uint8_t>(read_param_int32("RK_GD_ASC_MODE", _ascent_mode));
		_apogee_mode = static_cast<uint8_t>(read_param_int32("RK_GD_APX_MODE", _apogee_mode));
		_landing_mode = static_cast<uint8_t>(read_param_int32("RK_GD_LAND_MODE", _landing_mode));

		load_waypoint_parameters(0, 1);
		load_waypoint_parameters(1, 2);
		load_waypoint_parameters(2, 3);
	}

	void load_waypoint_parameters(size_t index, int waypoint_number)
	{
		if (index >= kMissionWaypointCount) {
			return;
		}

		MissionPoint &waypoint = _mission_waypoints[index];
		char buffer[24];
		const float default_cruise = math::min(math::max(_max_velocity_m_s, 1.f), 100.f);
		const float default_acceptance = math::max(_acceptance_radius_m, 1.f);

		snprintf(buffer, sizeof(buffer), "RK_GD_WP%u_MODE", waypoint_number);
		waypoint.mode = static_cast<uint8_t>(read_param_int32(buffer, waypoint.mode));

		snprintf(buffer, sizeof(buffer), "RK_GD_WP%u_HOLD_S", waypoint_number);
		waypoint.hold_time_s = math::max(read_param_float(buffer, waypoint.hold_time_s), 0.f);

		snprintf(buffer, sizeof(buffer), "RK_GD_WP%u_ACC_M", waypoint_number);
		const float acceptance_override = read_param_float(buffer, 0.f);
		waypoint.acceptance_radius_m = acceptance_override > 0.f ? acceptance_override : default_acceptance;

		snprintf(buffer, sizeof(buffer), "RK_GD_WP%u_C_MS", waypoint_number);
		const float cruise_override = read_param_float(buffer, 0.f);
		waypoint.cruise_speed_m_s = cruise_override > 0.f ? cruise_override : default_cruise;
	}

	void clear_waypoint_hold_states()
	{
		for (MissionPoint &waypoint : _mission_waypoints) {
			waypoint.hold_active = false;
			waypoint.hold_start = 0;
		}

		_waypoint_hold_remaining_s = 0.f;
	}

	void update_thrust_envelope()
	{
		const float mass_kg = math::max(_motor_reference.expected_vehicle_mass_kg, 0.f);
		_available_thrust_n = math::max(_motor_reference.expected_thrust_n, 0.f);
		_remaining_impulse_ns = math::max(_motor_reference.total_impulse_ns * (1.f - math::constrain(
						 _motor_reference.burn_fraction, 0.f, 1.f)), 0.f);
		_remaining_delta_v_m_s = mass_kg > 0.01f ? _remaining_impulse_ns / mass_kg : 0.f;
		_last_required_thrust_n = 0.f;
		_estimated_torque_pitch_nm = 0.f;
		_estimated_torque_yaw_nm = 0.f;
		_thrust_solution_valid = _motor_reference.loaded && mass_kg > 0.01f;
		_control_solution_valid = _thrust_solution_valid;
		_control_unreachable_reason = tv3_guidance_status_s::CONTROL_OK;
		_landing_reserve_valid = _thrust_solution_valid;
		_abort_corridor_valid = _thrust_solution_valid;
		_guidance_unreachable_reason = tv3_guidance_status_s::GUIDANCE_OK;
		_impulse_margin_ns = _remaining_impulse_ns - math::max(_min_remaining_impulse_ns, 0.f);
		_landing_delta_v_required_m_s = 0.f;
		_landing_delta_v_margin_m_s = 0.f;
		_abort_delta_v_required_m_s = 0.f;
		_abort_delta_v_margin_m_s = 0.f;
	}

	bool require_thrust_solution(uint8_t candidate_phase)
	{
		if (!_thrust_solution_valid) {
			_last_required_thrust_n = 0.f;
			_control_solution_valid = false;
			enter_no_solution_state();
			return false;
		}

		const float mass_kg = math::max(_motor_reference.expected_vehicle_mass_kg, 0.f);
		const float twr = (candidate_phase == tv3_guidance_status_s::PHASE_LANDING_APPROACH
				   || candidate_phase == tv3_guidance_status_s::PHASE_WAYPOINT_TRACK)
				  ? math::max(_landing_twr, 1.f)
				  : math::max(_min_twr, 0.1f);

		const float minimum_thrust_n = mass_kg * kGravityMps2 * twr;
		const bool thrust_ok = _available_thrust_n >= minimum_thrust_n;
		const bool impulse_ok = _impulse_margin_ns >= 0.f;
		_thrust_solution_valid = thrust_ok && impulse_ok;
		_control_solution_valid = _thrust_solution_valid;

		if (!_thrust_solution_valid) {
			_last_required_thrust_n = 0.f;
			_guidance_unreachable_reason = impulse_ok
						       ? tv3_guidance_status_s::GUIDANCE_THRUST_MARGIN
						       : tv3_guidance_status_s::GUIDANCE_IMPULSE;
			enter_no_solution_state();
			return false;
		}

		if (!require_control_solution()) {
			_thrust_solution_valid = false;
			_guidance_unreachable_reason = tv3_guidance_status_s::GUIDANCE_CONTROL;
			enter_no_solution_state();
			return false;
		}

		if (!require_mission_envelope(candidate_phase)) {
			_thrust_solution_valid = false;
			enter_no_solution_state();
			return false;
		}

		return true;
	}

	bool require_mission_envelope(uint8_t candidate_phase)
	{
		_landing_reserve_valid = true;
		_abort_corridor_valid = true;
		_landing_delta_v_required_m_s = 0.f;
		_landing_delta_v_margin_m_s = 0.f;
		_abort_delta_v_required_m_s = 0.f;
		_abort_delta_v_margin_m_s = 0.f;

		if (candidate_phase == tv3_guidance_status_s::PHASE_WAYPOINT_TRACK
		    || candidate_phase == tv3_guidance_status_s::PHASE_LANDING_APPROACH) {
			const float altitude_m = math::max(-(_last_target(2) - _origin(2)), 0.f);
			_landing_delta_v_required_m_s = altitude_m > 1e-3f ? sqrtf(2.f * kGravityMps2 * altitude_m) : 0.f;
			_landing_delta_v_margin_m_s = _remaining_delta_v_m_s - _landing_delta_v_required_m_s * kLandingDeltaVMargin;
			_landing_reserve_valid = _landing_delta_v_margin_m_s >= 0.f;

			if (!_landing_reserve_valid) {
				_guidance_unreachable_reason = tv3_guidance_status_s::GUIDANCE_LANDING_RESERVE;
				return false;
			}
		}

		if (_mission_started && candidate_phase != tv3_guidance_status_s::PHASE_STANDBY
		    && candidate_phase != tv3_guidance_status_s::PHASE_ABORT
		    && candidate_phase != tv3_guidance_status_s::PHASE_COMPLETE) {
			const Vector3f landing_global = _origin + _landing_point;
			const Vector3f position{_local_position.x, _local_position.y, _local_position.z};
			const Vector3f error = landing_global - position;
			const float horizontal_distance_m = sqrtf(error(0) * error(0) + error(1) * error(1));
			const float altitude_m = math::max(-error(2), 0.f);
			const float vertical_dv = altitude_m > 1e-3f ? sqrtf(2.f * kGravityMps2 * altitude_m) : 0.f;
			const float horizontal_dv = math::min(horizontal_distance_m * 0.15f, _max_velocity_m_s);
			const float descent_dv = math::min(_max_descent_rate_m_s, _max_velocity_m_s) * 0.5f;
			_abort_delta_v_required_m_s = vertical_dv + horizontal_dv + descent_dv;
			_abort_delta_v_margin_m_s = _remaining_delta_v_m_s - _abort_delta_v_required_m_s * kAbortDeltaVMargin;
			_abort_corridor_valid = _abort_delta_v_margin_m_s >= 0.f;

			if (!_abort_corridor_valid) {
				_guidance_unreachable_reason = tv3_guidance_status_s::GUIDANCE_ABORT_CORRIDOR;
				return false;
			}
		}

		return true;
	}

	bool require_control_solution()
	{
		_control_solution_valid = _thrust_solution_valid;
		_control_unreachable_reason = tv3_guidance_status_s::CONTROL_OK;
		_estimated_torque_pitch_nm = 0.f;
		_estimated_torque_yaw_nm = 0.f;

		if (!_control_solution_valid) {
			return false;
		}

		float max_thrust_n = 0.f;
		bool have_active_engine = false;

		for (int i = 0; i < _motor_reference.engine_count && i < 4; ++i) {
			if ((_motor_reference.active_mask & (1u << i)) == 0) {
				continue;
			}

			const float thrust = math::max(_motor_reference.expected_thrust_n_engine[i], 0.f);
			max_thrust_n += thrust;
			have_active_engine = true;
		}

		if (!have_active_engine) {
			_control_solution_valid = false;
			_control_unreachable_reason = tv3_guidance_status_s::CONTROL_NO_ACTIVE_ENGINES;
			return false;
		}

		const float required_thrust_n = math::max(_last_required_thrust_n, 0.f);

		// Pre-command checks use required_thrust_n == 0. Do not require the splay minimum
		// when validating the hover demand (splay is the mechanism that reaches lower net thrust).
		if (required_thrust_n > 1e-3f && required_thrust_n > max_thrust_n + 1.f) {
			_control_solution_valid = false;
			_control_unreachable_reason = tv3_guidance_status_s::CONTROL_THRUST_ENVELOPE;
			return false;
		}

		const float horiz_speed = sqrtf(_last_velocity_sp(0) * _last_velocity_sp(0)
						+ _last_velocity_sp(1) * _last_velocity_sp(1));
		if (horiz_speed > 0.5f) {
			const float mass_kg = math::max(_motor_reference.expected_vehicle_mass_kg, 0.1f);
			const float lateral_accel = mass_kg * math::min(horiz_speed * _position_gain, 20.f);
			const float lever_arm_m = 0.12f;
			const float estimated_torque = lateral_accel * lever_arm_m;
			_estimated_torque_pitch_nm = estimated_torque;
			_estimated_torque_yaw_nm = estimated_torque;

			if (estimated_torque > _torque_pitch_max + 1e-3f || estimated_torque > _torque_yaw_max + 1e-3f) {
				_control_solution_valid = false;
				_control_unreachable_reason = tv3_guidance_status_s::CONTROL_TORQUE_ENVELOPE;
				return false;
			}
		}

		return true;
	}

	void update_commanded_thrust(const vehicle_local_position_s &guidance_pos)
	{
		if (!_thrust_solution_valid) {
			_last_required_thrust_n = 0.f;
			_control_solution_valid = false;
			return;
		}

		if (_phase == tv3_guidance_status_s::PHASE_STANDBY
		    || _phase == tv3_guidance_status_s::PHASE_ABORT
		    || _phase == tv3_guidance_status_s::PHASE_COMPLETE) {
			_last_required_thrust_n = 0.f;
			return;
		}

		const float mass_kg = math::max(_motor_reference.expected_vehicle_mass_kg, 0.1f);
		const Vector3f position{guidance_pos.x, guidance_pos.y, guidance_pos.z};
		const Vector3f velocity{guidance_pos.vx, guidance_pos.vy, guidance_pos.vz};
		const float z_error = _last_target(2) - position(2);
		const float vz_error = _last_velocity_sp(2) - velocity(2);
		const float vel_gain = _position_gain * 2.f;
		const float a_z_cmd = _position_gain * z_error + vel_gain * vz_error;

		_last_required_thrust_n = math::constrain(mass_kg * (kGravityMps2 - a_z_cmd), 0.f, _available_thrust_n);
		_control_solution_valid = require_control_solution();
		if (!_control_solution_valid) {
			_thrust_solution_valid = false;
		}
	}

	void enter_no_solution_state()
	{
		_phase = _mission_started ? tv3_guidance_status_s::PHASE_ABORT : tv3_guidance_status_s::PHASE_STANDBY;
		_last_target = _origin_valid ? _origin + _landing_point : Vector3f{NAN, NAN, NAN};
		_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
		_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
	}

	bool ascent_uses_hover_window(Vector3f &target) const
	{
		if (_ascent_mode == kAscentHoverWindow) {
			target = Vector3f{_mission_waypoints[1].position(0), _mission_waypoints[1].position(1), 0.f};
			return true;
		}

		const Vector3f &wp1 = _mission_waypoints[0].position;
		const Vector3f &wp2 = _mission_waypoints[1].position;
		const Vector3f &wp3 = _mission_waypoints[2].position;

		if (fabsf(wp1(0) - wp2(0)) < 0.1f && fabsf(wp2(0) - wp3(0)) < 0.1f
		    && fabsf(wp1(1) - wp2(1)) < 0.1f && fabsf(wp2(1) - wp3(1)) < 0.1f) {
			target = Vector3f{wp2(0), wp2(1), 0.f};
			return true;
		}

		return false;
	}

	bool update_active_waypoint(const Vector3f &position, hrt_abstime now)
	{
		if (_waypoint_index >= kMissionWaypointCount) {
			return false;
		}

		MissionPoint &waypoint = _mission_waypoints[_waypoint_index];
		const Vector3f target_global = _origin + waypoint.position;
		const Vector3f error = target_global - position;
		const float distance = error.norm();
		_current_waypoint_mode = waypoint.mode;

		if (waypoint.mode == kWaypointPositionHold && distance <= waypoint.acceptance_radius_m) {
			if (!waypoint.hold_active) {
				waypoint.hold_active = true;
				waypoint.hold_start = now;
			}

			_last_target = target_global;
			_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
			_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);

			if (waypoint.hold_time_s > 0.f) {
				const float elapsed_s = waypoint.hold_start != 0
							  ? static_cast<float>(now - waypoint.hold_start) * 1e-6f
							  : 0.f;
				_waypoint_hold_remaining_s = math::max(waypoint.hold_time_s - elapsed_s, 0.f);

				if (elapsed_s >= waypoint.hold_time_s) {
					waypoint.hold_active = false;
					waypoint.hold_start = 0;
					++_waypoint_index;
					_waypoint_hold_remaining_s = 0.f;
				}
			} else {
				_waypoint_hold_remaining_s = NAN;
			}

			return true;
		}

		waypoint.hold_active = false;
		waypoint.hold_start = 0;
		_waypoint_hold_remaining_s = 0.f;

		if (distance <= waypoint.acceptance_radius_m) {
			if (waypoint.mode == kWaypointFlyThrough) {
				++_waypoint_index;
			}

			return true;
		}

		const float speed = math::constrain(distance * _position_gain, 0.f, waypoint.cruise_speed_m_s);
		const Vector3f direction = distance > 1e-3f ? error / distance : Vector3f{0.f, 0.f, 0.f};
		_last_target = target_global;
		_last_velocity_sp = direction * speed;
		_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
		return true;
	}

	static bool finite_position(const vehicle_local_position_s &position)
	{
		return PX4_ISFINITE(position.x) && PX4_ISFINITE(position.y) && PX4_ISFINITE(position.z)
		       && PX4_ISFINITE(position.vx) && PX4_ISFINITE(position.vy) && PX4_ISFINITE(position.vz);
	}

	static bool fresh_position(const vehicle_local_position_s &position, hrt_abstime now)
	{
		if (position.timestamp == 0) {
			return false;
		}

		const hrt_abstime age = now > position.timestamp ? now - position.timestamp : 0;
		return age <= 500_ms;
	}

	bool local_position_valid(hrt_abstime now) const
	{
		return _local_position.xy_valid && _local_position.z_valid
		       && finite_position(_local_position) && fresh_position(_local_position, now);
	}

	bool groundtruth_position_valid(hrt_abstime now) const
	{
		return _sim_groundtruth_fallback > 0
		       && finite_position(_groundtruth_position) && fresh_position(_groundtruth_position, now);
	}

	bool guidance_position_valid(hrt_abstime now) const
	{
		return local_position_valid(now) || groundtruth_position_valid(now);
	}

	const vehicle_local_position_s &guidance_position(hrt_abstime now) const
	{
		return local_position_valid(now) ? _local_position : _groundtruth_position;
	}

	void update_origin(hrt_abstime now)
	{
		if (_origin_valid) {
			return;
		}

		if (_home_position.valid_lpos || _home_position.valid_hpos) {
			_origin = make_vector(_home_position.x, _home_position.y, _home_position.z);
			_origin_valid = true;
			return;
		}

		if (guidance_position_valid(now)) {
			const vehicle_local_position_s &position = guidance_position(now);
			_origin = make_vector(position.x, position.y, position.z);
			_origin_valid = true;
		}
	}

	void reset_mission()
	{
		_phase = tv3_guidance_status_s::PHASE_STANDBY;
		_waypoint_index = 0;
		_mission_started = false;
		_apogee_reached = false;
		_last_target = Vector3f{NAN, NAN, NAN};
		_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
		_current_yaw_sp = 0.f;
		clear_waypoint_hold_states();
	}

	void update_phase(hrt_abstime now)
	{
		if (!_enabled) {
			reset_mission();
			return;
		}

		const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		const bool position_valid = guidance_position_valid(now);
		const bool ready = _tv3_status.mode >= tv3_status_s::MODE_READY && _tv3_status.mode != tv3_status_s::MODE_ABORT;

		if (_tv3_status.mode == tv3_status_s::MODE_ABORT) {
			_phase = tv3_guidance_status_s::PHASE_ABORT;
			_last_target = _origin_valid ? _origin + _landing_point : Vector3f{NAN, NAN, NAN};
			_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
			_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
			return;
		}

		if (!armed || !ready) {
			reset_mission();
			return;
		}

		if (!position_valid || !_origin_valid) {
			_phase = tv3_guidance_status_s::PHASE_STANDBY;
			return;
		}

		const vehicle_local_position_s &guidance_pos = guidance_position(now);
		const Vector3f position{guidance_pos.x, guidance_pos.y, guidance_pos.z};
		const Vector3f velocity{guidance_pos.vx, guidance_pos.vy, guidance_pos.vz};
		const Vector3f rel_pos = position - _origin;
		const float altitude_m = -rel_pos(2);

		if (_tv3_status.mode == tv3_status_s::MODE_IGNITION_PENDING || _tv3_status.mode == tv3_status_s::MODE_BOOST) {
			if (!require_thrust_solution(tv3_guidance_status_s::PHASE_LAUNCH_ASCENT)) {
				return;
			}

			_phase = tv3_guidance_status_s::PHASE_LAUNCH_ASCENT;
			_mission_started = true;
			Vector3f ascent_target{0.f, 0.f, -math::max(_apex_alt_m, _takeoff_alt_m)};
			Vector3f lateral_target{};

			if (ascent_uses_hover_window(lateral_target)) {
				ascent_target(0) = lateral_target(0);
				ascent_target(1) = lateral_target(1);
				ascent_target(2) = -math::max(_takeoff_alt_m, _hold_altitude_m);
			}

			_last_target = _origin + ascent_target;
			apply_vertical_velocity_limits(position);
			_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
			return;
		}

		if (_tv3_status.mode == tv3_status_s::MODE_COAST) {
			_mission_started = true;

			if (_apogee_mode == kApogeeSkip) {
				_apogee_reached = true;
			}

			if (!_apogee_reached) {
				if (!require_thrust_solution(tv3_guidance_status_s::PHASE_APOGEE_TRACK)) {
					return;
				}

				_phase = tv3_guidance_status_s::PHASE_APOGEE_TRACK;
				_last_target = _origin + Vector3f{0.f, 0.f, -math::max(_apex_alt_m, _takeoff_alt_m)};
				_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
				_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);

				if (velocity(2) >= 0.f && altitude_m >= _takeoff_alt_m) {
					_apogee_reached = true;
					_waypoint_index = 0;
				}

				return;
			}

			if (_waypoint_index < kMissionWaypointCount) {
				if (!require_thrust_solution(tv3_guidance_status_s::PHASE_WAYPOINT_TRACK)) {
					return;
				}

				_phase = tv3_guidance_status_s::PHASE_WAYPOINT_TRACK;
				update_active_waypoint(position, now);
				return;
			}

			if (_landing_mode == kLandingSkip) {
				_phase = tv3_guidance_status_s::PHASE_COMPLETE;
				_last_target = _origin + _landing_point;
				_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
				_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
				return;
			}

			_phase = tv3_guidance_status_s::PHASE_LANDING_APPROACH;
			if (!require_thrust_solution(tv3_guidance_status_s::PHASE_LANDING_APPROACH)) {
				return;
			}

			const Vector3f landing_global = _origin + _landing_point;
			const Vector3f error = landing_global - position;
			const float distance = error.norm();
			const float speed = math::constrain(distance * _position_gain, 0.f, _max_descent_rate_m_s);
			Vector3f velocity_sp = distance > 1e-3f ? (error / distance) * speed : Vector3f{0.f, 0.f, 0.f};
			velocity_sp(2) = math::constrain(velocity_sp(2), 0.f, _max_descent_rate_m_s);

			_last_target = landing_global;
			_last_velocity_sp = velocity_sp;
			_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);

			if (distance <= _acceptance_radius_m && altitude_m <= 5.f) {
				_phase = tv3_guidance_status_s::PHASE_COMPLETE;
				_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
				_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
			}

			return;
		}

		if (_mission_started) {
			if (!require_thrust_solution(tv3_guidance_status_s::PHASE_LANDING_APPROACH)) {
				return;
			}

			_phase = tv3_guidance_status_s::PHASE_LANDING_APPROACH;
			const Vector3f landing_global = _origin + _landing_point;
			const Vector3f error = landing_global - position;
			const float distance = error.norm();
			const float speed = math::constrain(distance * _position_gain, 0.f, _max_descent_rate_m_s);
			Vector3f velocity_sp = distance > 1e-3f ? (error / distance) * speed : Vector3f{0.f, 0.f, 0.f};
			velocity_sp(2) = math::constrain(velocity_sp(2), 0.f, _max_descent_rate_m_s);

			_last_target = landing_global;
			_last_velocity_sp = velocity_sp;
			_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
			return;
		}

		_phase = tv3_guidance_status_s::PHASE_STANDBY;
		_last_target = _origin + Vector3f{0.f, 0.f, -_hold_altitude_m};
		_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
		_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
	}

	void publish_outputs(hrt_abstime now)
	{
		const bool position_valid = guidance_position_valid(now);
		const vehicle_local_position_s &guidance_pos = guidance_position(now);

		tv3_guidance_status_s status{};
		status.timestamp = now;
		status.phase = _phase;
		status.active = _enabled > 0 && _phase != tv3_guidance_status_s::PHASE_STANDBY;
		status.armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		status.origin_valid = _origin_valid;
		status.position_valid = position_valid;
		status.tv3_ready = _tv3_status.ready;
		status.tv3_boosting = _tv3_status.mode == tv3_status_s::MODE_BOOST;
		status.tv3_coasting = _tv3_status.mode == tv3_status_s::MODE_COAST;
		status.tv3_mode = _tv3_status.mode;
		status.waypoint_index = _waypoint_index;
		status.waypoint_count = kMissionWaypointCount;
		status.waypoint_mode = _current_waypoint_mode;
		status.waypoint_hold_remaining_s = _waypoint_hold_remaining_s;
		status.ascent_mode = _ascent_mode;
		status.apogee_mode = _apogee_mode;
		status.landing_mode = _landing_mode;
		status.target_n = _last_target(0);
		status.target_e = _last_target(1);
		status.target_d = _last_target(2);
		status.target_distance_m = (position_valid && _origin_valid)
					   ? (_last_target - Vector3f{guidance_pos.x, guidance_pos.y, guidance_pos.z}).norm()
					   : NAN;
		status.cross_track_error_m = 0.f;
		status.down_track_error_m = 0.f;
		status.commanded_speed_m_s = _last_velocity_sp.norm();
		status.commanded_climb_rate_m_s = -_last_velocity_sp(2);
		status.commanded_yaw_rad = _current_yaw_sp;
		status.thrust_solution_valid = _thrust_solution_valid;
		status.control_solution_valid = _control_solution_valid;
		status.landing_reserve_valid = _landing_reserve_valid;
		status.abort_corridor_valid = _abort_corridor_valid;
		status.control_unreachable_reason = _control_unreachable_reason;
		status.guidance_unreachable_reason = _guidance_unreachable_reason;
		status.available_thrust_n = _available_thrust_n;
		status.required_thrust_n = _last_required_thrust_n;
		status.thrust_margin_n = _available_thrust_n - _last_required_thrust_n;
		status.impulse_margin_ns = _impulse_margin_ns;
		status.estimated_torque_pitch_nm = _estimated_torque_pitch_nm;
		status.estimated_torque_yaw_nm = _estimated_torque_yaw_nm;
		status.landing_delta_v_required_m_s = _landing_delta_v_required_m_s;
		status.landing_delta_v_margin_m_s = _landing_delta_v_margin_m_s;
		status.abort_delta_v_required_m_s = _abort_delta_v_required_m_s;
		status.abort_delta_v_margin_m_s = _abort_delta_v_margin_m_s;
		status.remaining_impulse_ns = _remaining_impulse_ns;
		status.remaining_delta_v_m_s = _remaining_delta_v_m_s;
		_status_pub.publish(status);

		trajectory_setpoint_s setpoint{};
		setpoint.timestamp = now;

		if (_origin_valid && position_valid) {
			const Vector3f position{guidance_pos.x, guidance_pos.y, guidance_pos.z};
			Vector3f position_sp = _last_target;
			Vector3f velocity_sp = _last_velocity_sp;

			if (_phase == tv3_guidance_status_s::PHASE_STANDBY && !_mission_started) {
				position_sp = _origin + Vector3f{0.f, 0.f, -_hold_altitude_m};
				velocity_sp = Vector3f{0.f, 0.f, 0.f};
			} else if (_phase == tv3_guidance_status_s::PHASE_ABORT) {
				position_sp = position;
				velocity_sp = Vector3f{0.f, 0.f, 0.f};
			} else if (_phase == tv3_guidance_status_s::PHASE_COMPLETE) {
				position_sp = _origin + _landing_point;
				velocity_sp = Vector3f{0.f, 0.f, 0.f};
			}

			setpoint.position[0] = position_sp(0);
			setpoint.position[1] = position_sp(1);
			setpoint.position[2] = position_sp(2);
			setpoint.velocity[0] = velocity_sp(0);
			setpoint.velocity[1] = velocity_sp(1);
			setpoint.velocity[2] = velocity_sp(2);
			setpoint.acceleration[0] = NAN;
			setpoint.acceleration[1] = NAN;
			setpoint.acceleration[2] = NAN;
			setpoint.jerk[0] = NAN;
			setpoint.jerk[1] = NAN;
			setpoint.jerk[2] = NAN;
			setpoint.yaw = _current_yaw_sp;
			setpoint.yawspeed = 0.f;
			_setpoint_pub.publish(setpoint);
			return;
		}

		set_nan_trajectory(setpoint);
		_setpoint_pub.publish(setpoint);
	}

	void apply_vertical_velocity_limits(const Vector3f &position)
	{
		const Vector3f error = _last_target - position;
		const float distance = error.norm();
		const float speed = math::constrain(distance * _position_gain, 0.f, _max_velocity_m_s);
		_last_velocity_sp = distance > 1e-3f ? (error / distance) * speed : Vector3f{0.f, 0.f, 0.f};

		if (error(2) < -0.1f) {
			_last_velocity_sp(2) = math::max(_last_velocity_sp(2), -math::max(_max_climb_rate_m_s, 1.f));
		} else if (error(2) > 0.1f) {
			_last_velocity_sp(2) = math::min(_last_velocity_sp(2), math::max(_max_descent_rate_m_s, 1.f));
		} else {
			_last_velocity_sp(2) = 0.f;
		}
	}

	float update_current_yaw_sp(const Vector3f &velocity_sp)
	{
		if (std::fabs(_fixed_yaw_deg) > 1e-3f) {
			return _fixed_yaw_deg * kDegToRad;
		}

		if (velocity_sp.norm() > 1e-3f) {
			return atan2f(velocity_sp(1), velocity_sp(0));
		}

		return 0.f;
	}

	int32_t _enabled{1};
	int32_t _sim_groundtruth_fallback{0};
	float _takeoff_alt_m{35.f};
	float _apex_alt_m{120.f};
	float _position_gain{0.15f};
	float _max_velocity_m_s{30.f};
	float _max_climb_rate_m_s{15.f};
	float _max_descent_rate_m_s{8.f};
	float _fixed_yaw_deg{0.f};
	float _hold_altitude_m{5.f};
	float _acceptance_radius_m{15.f};
	float _min_twr{1.05f};
	float _landing_twr{1.15f};
	float _min_remaining_impulse_ns{0.f};
	float _splay_max_deg{5.f};
	float _torque_pitch_max{10.f};
	float _torque_yaw_max{10.f};
	uint8_t _ascent_mode{kAscentLaunch};
	uint8_t _apogee_mode{kApogeeTrack};
	uint8_t _landing_mode{kLandingApproach};
	uint8_t _current_waypoint_mode{kWaypointFlyThrough};
	float _waypoint_hold_remaining_s{0.f};

	MissionPoint _mission_waypoints[kMissionWaypointCount]{};
	Vector3f _landing_point{0.f, 0.f, 0.f};

	uint8_t _phase{tv3_guidance_status_s::PHASE_STANDBY};
	uint8_t _waypoint_index{0};
	bool _mission_started{false};
	bool _apogee_reached{false};
	bool _origin_valid{false};
	Vector3f _origin{};
	Vector3f _last_target{NAN, NAN, NAN};
	Vector3f _last_velocity_sp{0.f, 0.f, 0.f};
	float _current_yaw_sp{0.f};
	bool _thrust_solution_valid{false};
	bool _control_solution_valid{false};
	bool _landing_reserve_valid{false};
	bool _abort_corridor_valid{false};
	uint8_t _control_unreachable_reason{tv3_guidance_status_s::CONTROL_OK};
	uint8_t _guidance_unreachable_reason{tv3_guidance_status_s::GUIDANCE_OK};
	float _available_thrust_n{0.f};
	float _impulse_margin_ns{0.f};
	float _estimated_torque_pitch_nm{0.f};
	float _estimated_torque_yaw_nm{0.f};
	float _landing_delta_v_required_m_s{0.f};
	float _landing_delta_v_margin_m_s{0.f};
	float _abort_delta_v_required_m_s{0.f};
	float _abort_delta_v_margin_m_s{0.f};
	float _last_required_thrust_n{0.f};
	float _remaining_impulse_ns{0.f};
	float _remaining_delta_v_m_s{0.f};

	vehicle_local_position_s _local_position{};
	vehicle_local_position_s _groundtruth_position{};
	home_position_s _home_position{};
	vehicle_status_s _vehicle_status{};
	tv3_status_s _tv3_status{};
	tv3_motor_reference_s _motor_reference{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _groundtruth_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _tv3_status_sub{ORB_ID(tv3_status)};
	uORB::Subscription _motor_reference_sub{ORB_ID(tv3_motor_reference)};
	uORB::Publication<trajectory_setpoint_s> _setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<tv3_guidance_status_s> _status_pub{ORB_ID(tv3_guidance_status)};
};

extern "C" __EXPORT int tv3_guidance_main(int argc, char *argv[])
{
	return TV3Guidance::main(argc, argv);
}
