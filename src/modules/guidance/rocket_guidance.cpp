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
#include <uORB/topics/rocket_guidance_status.h>
#include <uORB/topics/rocket_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <array>
#include <cmath>

using namespace time_literals;
using matrix::Vector3f;

namespace
{
constexpr float kDegToRad = 0.017453292519943295769f;
constexpr size_t kMissionWaypointCount = 3;

struct MissionPoint {
	Vector3f position{};
	float acceptance_radius_m{15.f};
	float cruise_speed_m_s{25.f};
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

class RocketGuidance : public ModuleBase<RocketGuidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RocketGuidance() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		RocketGuidance *instance = new RocketGuidance();

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

		PRINT_MODULE_DESCRIPTION("Rocket launch, waypoint, and landing guidance stack.");
		PRINT_MODULE_USAGE_NAME("rocket_guidance", "modules");
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
		_home_position_sub.update(&_home_position);
		_rocket_status_sub.update(&_rocket_status);

		const hrt_abstime now = hrt_absolute_time();
		update_origin();
		update_phase();
		publish_outputs(now);
	}

	void update_parameters()
	{
		_enabled = read_param_int32("RK_GD_ENABLE", _enabled);
		_takeoff_alt_m = read_param_float("RK_GD_TAKE_ALT", _takeoff_alt_m);
		_apex_alt_m = read_param_float("RK_GD_APEX_ALT", _apex_alt_m);
		_position_gain = read_param_float("RK_GD_POS_P", _position_gain);
		_max_velocity_m_s = read_param_float("RK_GD_VMAX_MS", _max_velocity_m_s);
		_max_climb_rate_m_s = read_param_float("RK_GD_VUP_MS", _max_climb_rate_m_s);
		_max_descent_rate_m_s = read_param_float("RK_GD_VDN_MS", _max_descent_rate_m_s);
		_fixed_yaw_deg = read_param_float("RK_GD_YAW_DEG", _fixed_yaw_deg);
		_hold_altitude_m = read_param_float("RK_GD_HOLD_ALT", _hold_altitude_m);
		_acceptance_radius_m = read_param_float("RK_GD_ACC_RAD", _acceptance_radius_m);

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

		for (MissionPoint &waypoint : _mission_waypoints) {
			waypoint.acceptance_radius_m = math::max(_acceptance_radius_m, 1.f);
			waypoint.cruise_speed_m_s = math::min(math::max(_max_velocity_m_s, 1.f), 100.f);
		}
	}

	void update_origin()
	{
		if (_origin_valid) {
			return;
		}

		if (_home_position.valid_lpos || _home_position.valid_hpos) {
			_origin = make_vector(_home_position.x, _home_position.y, _home_position.z);
			_origin_valid = true;
			return;
		}

		if (_local_position.xy_valid && _local_position.z_valid) {
			_origin = make_vector(_local_position.x, _local_position.y, _local_position.z);
			_origin_valid = true;
		}
	}

	void reset_mission()
	{
		_phase = rocket_guidance_status_s::PHASE_STANDBY;
		_waypoint_index = 0;
		_mission_started = false;
		_apogee_reached = false;
		_last_target = Vector3f{NAN, NAN, NAN};
		_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
		_current_yaw_sp = 0.f;
	}

	void update_phase()
	{
		if (!_enabled) {
			reset_mission();
			return;
		}

		const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		const bool position_valid = _local_position.xy_valid && _local_position.z_valid;
		const bool ready = _rocket_status.mode >= rocket_status_s::MODE_READY && _rocket_status.mode != rocket_status_s::MODE_ABORT;

		if (_rocket_status.mode == rocket_status_s::MODE_ABORT) {
			_phase = rocket_guidance_status_s::PHASE_ABORT;
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
			_phase = rocket_guidance_status_s::PHASE_STANDBY;
			return;
		}

		const Vector3f position{_local_position.x, _local_position.y, _local_position.z};
		const Vector3f velocity{_local_position.vx, _local_position.vy, _local_position.vz};
		const Vector3f rel_pos = position - _origin;
		const float altitude_m = -rel_pos(2);

		if (_rocket_status.mode == rocket_status_s::MODE_IGNITION_PENDING || _rocket_status.mode == rocket_status_s::MODE_BOOST) {
			_phase = rocket_guidance_status_s::PHASE_LAUNCH_ASCENT;
			_mission_started = true;
			_last_target = _origin + Vector3f{0.f, 0.f, -math::max(_apex_alt_m, _takeoff_alt_m)};
			_last_velocity_sp = Vector3f{0.f, 0.f, -math::max(_max_climb_rate_m_s, 1.f)};
			_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
			return;
		}

		if (_rocket_status.mode == rocket_status_s::MODE_COAST) {
			_mission_started = true;

			if (!_apogee_reached) {
				_phase = rocket_guidance_status_s::PHASE_APOGEE_TRACK;
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
				_phase = rocket_guidance_status_s::PHASE_WAYPOINT_TRACK;
				const MissionPoint &target = _mission_waypoints[_waypoint_index];
				const Vector3f target_global = _origin + target.position;
				const Vector3f error = target_global - position;
				const float distance = error.norm();

				if (distance <= target.acceptance_radius_m) {
					++_waypoint_index;
					return;
				}

				const float speed = math::constrain(distance * _position_gain, 0.f, target.cruise_speed_m_s);
				const Vector3f direction = distance > 1e-3f ? error / distance : Vector3f{0.f, 0.f, 0.f};

				_last_target = target_global;
				_last_velocity_sp = direction * speed;
				_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
				return;
			}

			_phase = rocket_guidance_status_s::PHASE_LANDING_APPROACH;
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
				_phase = rocket_guidance_status_s::PHASE_COMPLETE;
				_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
				_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
			}

			return;
		}

		if (_mission_started) {
			_phase = rocket_guidance_status_s::PHASE_LANDING_APPROACH;
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

		_phase = rocket_guidance_status_s::PHASE_STANDBY;
		_last_target = _origin + Vector3f{0.f, 0.f, -_hold_altitude_m};
		_last_velocity_sp = Vector3f{0.f, 0.f, 0.f};
		_current_yaw_sp = update_current_yaw_sp(_last_velocity_sp);
	}

	void publish_outputs(hrt_abstime now)
	{
		rocket_guidance_status_s status{};
		status.timestamp = now;
		status.phase = _phase;
		status.active = _enabled > 0 && _phase != rocket_guidance_status_s::PHASE_STANDBY;
		status.armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		status.origin_valid = _origin_valid;
		status.position_valid = _local_position.xy_valid && _local_position.z_valid;
		status.rocket_ready = _rocket_status.ready;
		status.rocket_boosting = _rocket_status.mode == rocket_status_s::MODE_BOOST;
		status.rocket_coasting = _rocket_status.mode == rocket_status_s::MODE_COAST;
		status.rocket_mode = _rocket_status.mode;
		status.waypoint_index = _waypoint_index;
		status.waypoint_count = kMissionWaypointCount;
		status.target_n = _last_target(0);
		status.target_e = _last_target(1);
		status.target_d = _last_target(2);
		status.target_distance_m = (status.position_valid && _origin_valid)
					   ? (_last_target - Vector3f{_local_position.x, _local_position.y, _local_position.z}).norm()
					   : NAN;
		status.cross_track_error_m = 0.f;
		status.down_track_error_m = 0.f;
		status.commanded_speed_m_s = _last_velocity_sp.norm();
		status.commanded_climb_rate_m_s = -_last_velocity_sp(2);
		status.commanded_yaw_rad = _current_yaw_sp;
		_status_pub.publish(status);

		trajectory_setpoint_s setpoint{};
		setpoint.timestamp = now;

		if (_origin_valid && status.position_valid) {
			const Vector3f position{_local_position.x, _local_position.y, _local_position.z};
			Vector3f position_sp = _last_target;
			Vector3f velocity_sp = _last_velocity_sp;

			if (_phase == rocket_guidance_status_s::PHASE_STANDBY && !_mission_started) {
				position_sp = _origin + Vector3f{0.f, 0.f, -_hold_altitude_m};
				velocity_sp = Vector3f{0.f, 0.f, 0.f};
			} else if (_phase == rocket_guidance_status_s::PHASE_ABORT) {
				position_sp = position;
				velocity_sp = Vector3f{0.f, 0.f, 0.f};
			} else if (_phase == rocket_guidance_status_s::PHASE_COMPLETE) {
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
	float _takeoff_alt_m{35.f};
	float _apex_alt_m{120.f};
	float _position_gain{0.15f};
	float _max_velocity_m_s{30.f};
	float _max_climb_rate_m_s{15.f};
	float _max_descent_rate_m_s{8.f};
	float _fixed_yaw_deg{0.f};
	float _hold_altitude_m{5.f};
	float _acceptance_radius_m{15.f};

	std::array<MissionPoint, kMissionWaypointCount> _mission_waypoints{{
		{make_vector(60.f, 0.f, -60.f), 15.f, 25.f},
		{make_vector(150.f, 30.f, -90.f), 15.f, 25.f},
		{make_vector(220.f, 80.f, -75.f), 15.f, 25.f},
	}};
	Vector3f _landing_point{0.f, 0.f, 0.f};

	uint8_t _phase{rocket_guidance_status_s::PHASE_STANDBY};
	uint8_t _waypoint_index{0};
	bool _mission_started{false};
	bool _apogee_reached{false};
	bool _origin_valid{false};
	Vector3f _origin{};
	Vector3f _last_target{NAN, NAN, NAN};
	Vector3f _last_velocity_sp{0.f, 0.f, 0.f};
	float _current_yaw_sp{0.f};

	vehicle_local_position_s _local_position{};
	home_position_s _home_position{};
	vehicle_status_s _vehicle_status{};
	rocket_status_s _rocket_status{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _rocket_status_sub{ORB_ID(rocket_status)};
	uORB::Publication<trajectory_setpoint_s> _setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<rocket_guidance_status_s> _status_pub{ORB_ID(rocket_guidance_status)};
};

extern "C" __EXPORT int rocket_guidance_main(int argc, char *argv[])
{
	return RocketGuidance::main(argc, argv);
}
