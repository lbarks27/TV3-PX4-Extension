#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <math.h>

#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_control_authority.h>
#include <uORB/topics/tv3_guidance_status.h>
#include <uORB/topics/tv3_status.h>
#include <uORB/topics/tv3_thrust.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;
using matrix::AxisAnglef;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

namespace
{
constexpr float kDegToRad = 0.017453292519943295769f;
constexpr float kGravityMps2 = 9.80665f;
}

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

		PRINT_MODULE_DESCRIPTION("TV3 attitude controller publishing body-frame torque setpoints.");
		PRINT_MODULE_USAGE_NAME("tv3_attitude", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(10_ms);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("integrator roll %.3f pitch %.3f yaw %.3f",
			 (double)_integrator(0), (double)_integrator(1), (double)_integrator(2));
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

		if (!_vehicle_attitude_sub.update(&_attitude)) {
			return;
		}

		_vehicle_angular_velocity_sub.update(&_angular_velocity);
		_tv3_status_sub.update(&_status);
		_tv3_thrust_sub.update(&_thrust);
		_tv3_guidance_status_sub.update(&_guidance_status);
		_tv3_control_authority_sub.update(&_control_authority);
		_local_position_sub.update(&_local_position);
		_groundtruth_position_sub.update(&_groundtruth_position);
		trajectory_setpoint_s trajectory_setpoint{};
		const bool trajectory_updated = _trajectory_setpoint_sub.update(&trajectory_setpoint);

		const hrt_abstime now = hrt_absolute_time();
		const float dt = _last_update != 0 ? static_cast<float>(now - _last_update) * 1e-6f : 0.01f;
		_last_update = now;

		const bool coast_without_guidance = _status.mode == tv3_status_s::MODE_COAST && _guidance_enabled <= 0;

		if (_status.mode < tv3_status_s::MODE_READY || _status.mode == tv3_status_s::MODE_ABORT || coast_without_guidance) {
			reset_controller();
			publish_zero(now, _attitude.timestamp_sample);
			return;
		}

		if (!_reference_initialized) {
			memcpy(_launch_reference_q, _attitude.q, sizeof(_launch_reference_q));
			_reference_initialized = true;
		}

		update_guidance_reference(trajectory_setpoint, trajectory_updated, now);
		apply_roll_program();

		Quatf q(_attitude.q);
		Quatf q_ref(_reference_q);
		Quatf q_error = q.inversed() * q_ref;

		if (q_error(0) < 0.f) {
			q_error = -q_error;
		}

		const Vector3f att_error = quaternion_attitude_error(q_error);
		const bool rail_mode = !_status.rail_exit;
		const bool powered_flight = _status.mode == tv3_status_s::MODE_IGNITION_PENDING
					    || _status.mode == tv3_status_s::MODE_BOOST
					    || _status.mode == tv3_status_s::MODE_COAST;
		float att_p = rail_mode ? _att_p_rail : _att_p_free;
		float rate_p = rail_mode ? _rate_p_rail : _rate_p_free;

		if (powered_flight) {
			att_p = _att_p_boost;
			rate_p = _rate_p_boost;
		}

		const bool on_rail = powered_flight && !_status.rail_exit;

		const bool in_boost_ignition = (_status.mode == tv3_status_s::MODE_IGNITION_PENDING
						|| _status.mode == tv3_status_s::MODE_BOOST);
		const bool pure_boost_hold = in_boost_ignition && (_boost_attitude_only > 0 || _boost_full_thrust > 0);
		const bool boost_attitude_experiment = in_boost_ignition && (_boost_attitude_only > 0);
		// Only boost-attitude-only experiments suppress yaw torque. Hover-window ascent must
		// correct tilt on all controllable axes after rail_exit (BS-009).
		const bool suppress_yaw_torque = boost_attitude_experiment;

		// Dump rail-period attitude error on release so the first free-flight torque command
		// does not inherit stale integrator state (BS-002).
		if (_status.rail_exit && !_prev_rail_exit) {
			_integrator.zero();
		}

		_prev_rail_exit = _status.rail_exit;

		const Vector3f rate_sp{att_p * att_error(0), att_p * att_error(1), att_p * att_error(2)};
		// SIH zeros body rates on rail but the published gyro can still carry transients; ignore
		// them until rail_exit so the rate loop does not inherit bogus wz (BS-002).
		const bool boost_rail_hold = on_rail && in_boost_ignition;
		const Vector3f rate_meas = boost_rail_hold ? Vector3f{} : Vector3f{_angular_velocity.xyz};
		const Vector3f rate_error = rate_sp - rate_meas;
		const float integrator_limit = powered_flight ? _integrator_limit_boost : _integrator_limit;

		Vector3f authority_positive{};
		Vector3f authority_negative{};
		const bool authority_valid = _control_authority.valid && _control_authority.timestamp != 0;

		if (authority_valid) {
			authority_positive = Vector3f{_control_authority.achievable_torque_positive_nm[0],
						      _control_authority.achievable_torque_positive_nm[1],
						      _control_authority.achievable_torque_positive_nm[2]};
			authority_negative = Vector3f{_control_authority.achievable_torque_negative_nm[0],
						      _control_authority.achievable_torque_negative_nm[1],
						      _control_authority.achievable_torque_negative_nm[2]};
		}

		if (!on_rail) {
			const float roll_command = rate_p * rate_error(0) + _integrator(0) - _rate_d * _angular_velocity.xyz_derivative[0];
			const float pitch_command = rate_p * rate_error(1) + _integrator(1) - _rate_d * _angular_velocity.xyz_derivative[1];
			const float yaw_command = suppress_yaw_torque ? 0.f :
						  rate_p * rate_error(2) + _integrator(2) - _rate_d * _angular_velocity.xyz_derivative[2];

			if (!axis_authority_saturated(roll_command, authority_positive(0), authority_negative(0), authority_valid)) {
				_integrator(0) = math::constrain(_integrator(0) + rate_error(0) * _rate_i * dt, -integrator_limit, integrator_limit);
			}

			if (!axis_authority_saturated(pitch_command, authority_positive(1), authority_negative(1), authority_valid)) {
				_integrator(1) = math::constrain(_integrator(1) + rate_error(1) * _rate_i * dt, -integrator_limit, integrator_limit);
			}

			if (!suppress_yaw_torque
			    && !axis_authority_saturated(yaw_command, authority_positive(2), authority_negative(2), authority_valid)) {
				_integrator(2) = math::constrain(_integrator(2) + rate_error(2) * _rate_i * dt, -integrator_limit, integrator_limit);
			}
		}

		// Hold all TVC torque at zero while kinematically on rail during boost/ignition so LM sees
		// a consistent zero-demand window until tv3_status.rail_exit (BS-001 / BS-009).
		float torque_roll = boost_rail_hold ? 0.f :
			math::constrain(rate_p * rate_error(0) + _integrator(0) - _rate_d * _angular_velocity.xyz_derivative[0],
					-_torque_roll_max, _torque_roll_max);
		float torque_pitch = boost_rail_hold ? 0.f :
			math::constrain(rate_p * rate_error(1) + _integrator(1) - _rate_d * _angular_velocity.xyz_derivative[1],
					-_torque_pitch_max, _torque_pitch_max);

		// Boost-attitude-only experiments only: large roll-about-thrust rates make transverse TVC
		// counterproductive. Hover-window ascent must keep commanding pitch/roll to pass the gate.
		constexpr float kBoostWzInhibitRadS = math::radians(45.f);
		if (boost_attitude_experiment && fabsf(rate_meas(2)) > kBoostWzInhibitRadS) {
			torque_roll = 0.f;
			torque_pitch = 0.f;
		}

		// Do not disable lateral torque near ground during landing approach — required for
		// 0.5 m window precision touchdown control (the 0.5 m threshold would otherwise kill
		// authority exactly when we need it).
		const bool landing_approach = (_guidance_status.phase == tv3_guidance_status_s::PHASE_LANDING_APPROACH);
		if (near_ground(now) && powered_flight && !landing_approach) {
			torque_roll = 0.f;
			torque_pitch = 0.f;
			_integrator(0) = 0.f;
			_integrator(1) = 0.f;
		}

		// Boost-attitude-only experiments suppress yaw torque; hover-window profiles need it to
		// recover from post-rail tilt (BS-009).
		const float torque_yaw = suppress_yaw_torque ? 0.f :
			math::constrain(rate_p * rate_error(2) + _integrator(2) - _rate_d * _angular_velocity.xyz_derivative[2],
					-_torque_yaw_max, _torque_yaw_max);

		const bool boost_attitude_only = _boost_attitude_only > 0
						 && (_status.mode == tv3_status_s::MODE_IGNITION_PENDING
						     || _status.mode == tv3_status_s::MODE_BOOST);
		// During ignition/boost we want the attitude controller to keep commanding torque
		// for "point straight up" experiments even if guidance reports control envelope issues.
		// The mixer will do its best (and report residuals). This prevents sudden loss of TVC
		// authority exactly at rail exit (BS-016 / gating).
		const bool control_envelope_valid = boost_attitude_only
						    || pure_boost_hold
						    || _guidance_enabled <= 0
						    || _guidance_status.timestamp == 0
						    || _guidance_status.control_solution_valid;
		const float torque_scale = control_envelope_valid ? 1.f : 0.f;
		const bool coast_without_thrust = _status.mode == tv3_status_s::MODE_COAST && !_status.ignition_on;
		Vector3f torque_command{torque_roll, torque_pitch, torque_yaw};
		torque_command *= torque_scale;

		if (coast_without_thrust) {
			torque_command.zero();
		} else if (authority_valid && torque_scale > 0.f) {
			torque_command = scale_torque_to_authority(torque_command, authority_positive, authority_negative);
		}

		vehicle_torque_setpoint_s torque{};
		torque.timestamp = now;
		torque.timestamp_sample = _attitude.timestamp_sample;
		torque.xyz[0] = torque_command(0);
		torque.xyz[1] = torque_command(1);
		torque.xyz[2] = torque_command(2);
		_torque_pub.publish(torque);

		vehicle_thrust_setpoint_s thrust{};
		thrust.timestamp = now;
		thrust.timestamp_sample = _attitude.timestamp_sample;
		// TV3 axial thrust is scheduled by tv3_guidance / engine_state, not PX4 servos.
		thrust.xyz[0] = 0.f;
		thrust.xyz[1] = 0.f;
		thrust.xyz[2] = 0.f;
		_thrust_pub.publish(thrust);
	}

	static bool axis_authority_saturated(float command_nm, float positive_limit_nm, float negative_limit_nm, bool authority_valid)
	{
		if (!authority_valid) {
			return false;
		}

		const float limit = command_nm >= 0.f ? positive_limit_nm : negative_limit_nm;
		return limit > 1e-4f && fabsf(command_nm) > limit + 1e-3f;
	}

	static Vector3f scale_torque_to_authority(const Vector3f &demand_nm,
			const Vector3f &positive_limit_nm,
			const Vector3f &negative_limit_nm)
	{
		Vector3f scaled{};

		for (int axis = 0; axis < 3; ++axis) {
			const float demand = demand_nm(axis);

			if (fabsf(demand) < 1e-4f) {
				continue;
			}

			float limit = demand >= 0.f ? positive_limit_nm(axis) : negative_limit_nm(axis);

			// TVC authority sampling can be one-sided on roll-about-thrust; use the opposite bound
			// when this sign has no envelope so hover ascent can still apply corrective torque.
			if (limit <= 1e-4f) {
				limit = demand >= 0.f ? negative_limit_nm(axis) : positive_limit_nm(axis);
			}

			if (limit <= 1e-4f) {
				continue;
			}

			scaled(axis) = copysignf(math::min(fabsf(demand), limit), demand);
		}

		return scaled;
	}

	static Vector3f quaternion_attitude_error(const Quatf &q_error)
	{
		Quatf qe = q_error;

		if (qe(0) < 0.f) {
			qe = -qe;
		}

		const float w = math::constrain(qe(0), 0.f, 1.f);
		const float sin_half = sqrtf(math::max(1.f - w * w, 0.f));
		const Vector3f imag = qe.imag();

		if (sin_half > 1e-3f) {
			const float angle = 2.f * acosf(w);
			return (angle / sin_half) * imag;
		}

		return 2.f * imag;
	}

	bool local_position_valid(hrt_abstime now) const
	{
		return _local_position.xy_valid && _local_position.z_valid
		       && PX4_ISFINITE(_local_position.x) && PX4_ISFINITE(_local_position.y) && PX4_ISFINITE(_local_position.z)
		       && _local_position.timestamp != 0
		       && (now <= _local_position.timestamp || now - _local_position.timestamp <= 500_ms);
	}

	bool groundtruth_position_valid(hrt_abstime now) const
	{
		return _sim_groundtruth_fallback > 0
		       && PX4_ISFINITE(_groundtruth_position.x) && PX4_ISFINITE(_groundtruth_position.y)
		       && PX4_ISFINITE(_groundtruth_position.z)
		       && _groundtruth_position.timestamp != 0
		       && (now <= _groundtruth_position.timestamp || now - _groundtruth_position.timestamp <= 500_ms);
	}

	bool near_ground(hrt_abstime now) const
	{
		if (local_position_valid(now)) {
			return -_local_position.z < 0.5f;
		}

		if (groundtruth_position_valid(now)) {
			return -_groundtruth_position.z < 0.5f;
		}

		return false;
	}

	const vehicle_local_position_s &guidance_position(hrt_abstime now) const
	{
		return local_position_valid(now) ? _local_position : _groundtruth_position;
	}

	void apply_roll_program()
	{
		if (_roll_program_deg <= 0.f || _roll_program_dur_s <= 0.f) {
			return;
		}

		const bool powered_flight = _status.mode == tv3_status_s::MODE_IGNITION_PENDING
					    || _status.mode == tv3_status_s::MODE_BOOST
					    || _status.mode == tv3_status_s::MODE_COAST;

		if (!powered_flight || _status.burn_time_s < _roll_program_start_s) {
			return;
		}

		const float elapsed_s = _status.burn_time_s - _roll_program_start_s;
		const float frac = math::constrain(elapsed_s / _roll_program_dur_s, 0.f, 1.f);
		const float roll_rad = _roll_program_deg * kDegToRad * frac;
		Quatf reference{_reference_q};
		const Quatf roll_offset{AxisAnglef{Vector3f{1.f, 0.f, 0.f}, roll_rad}};
		reference = reference * roll_offset;
		reference.copyTo(_reference_q);
	}

	bool hover_lateral_control_active() const
	{
		if (_guidance_enabled <= 0 || _boost_attitude_only > 0) {
			return false;
		}

		if (_boost_full_thrust > 0
		    && _guidance_status.ascent_mode != tv3_guidance_status_s::ASCENT_HOVER_WINDOW) {
			return false;
		}

		const bool powered_flight = _status.mode == tv3_status_s::MODE_IGNITION_PENDING
					    || _status.mode == tv3_status_s::MODE_BOOST
					    || _status.mode == tv3_status_s::MODE_COAST;

		if (!powered_flight || !_status.rail_exit) {
			return false;
		}

		if (_guidance_status.ascent_mode == tv3_guidance_status_s::ASCENT_HOVER_WINDOW
		    && (_guidance_status.phase == tv3_guidance_status_s::PHASE_LAUNCH_ASCENT
			|| _guidance_status.phase == tv3_guidance_status_s::PHASE_WAYPOINT_TRACK)) {
			return true;
		}

		if (_guidance_status.waypoint_mode == tv3_guidance_status_s::WP_MODE_POSITION_HOLD) {
			return true;
		}

		return _guidance_status.phase == tv3_guidance_status_s::PHASE_LANDING_APPROACH;
	}

	void update_guidance_reference(const trajectory_setpoint_s &trajectory_setpoint, bool trajectory_updated, hrt_abstime now)
	{
		memcpy(_reference_q, _launch_reference_q, sizeof(_reference_q));

		if (_boost_attitude_only > 0
		    && (_status.mode == tv3_status_s::MODE_IGNITION_PENDING
			|| _status.mode == tv3_status_s::MODE_BOOST)) {
			return;
		}

		if (_guidance_enabled <= 0 || !trajectory_updated) {
			return;
		}

		const bool powered_flight = _status.mode == tv3_status_s::MODE_IGNITION_PENDING
					    || _status.mode == tv3_status_s::MODE_BOOST
					    || _status.mode == tv3_status_s::MODE_COAST;

		if (!powered_flight || (!local_position_valid(now) && !groundtruth_position_valid(now))) {
			return;
		}

		const vehicle_local_position_s &position = guidance_position(now);
		const bool hover_lateral = hover_lateral_control_active();
		Vector3f velocity_sp{0.f, 0.f, 0.f};
		bool have_command = false;

		if (PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])
		    && PX4_ISFINITE(trajectory_setpoint.velocity[2])) {
			velocity_sp = Vector3f{trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1],
					       trajectory_setpoint.velocity[2]};
			have_command = true;
		}

		const bool have_position_sp = PX4_ISFINITE(trajectory_setpoint.position[0])
					    && PX4_ISFINITE(trajectory_setpoint.position[1])
					    && PX4_ISFINITE(trajectory_setpoint.position[2]);

		if (!hover_lateral && have_position_sp) {
			const Vector3f position_sp{trajectory_setpoint.position[0], trajectory_setpoint.position[1],
						   trajectory_setpoint.position[2]};
			const Vector3f position_meas{position.x, position.y, position.z};
			velocity_sp += _position_gain * (position_sp - position_meas);
			have_command = true;
		} else if (hover_lateral && !have_command && have_position_sp) {
			const Vector3f position_sp{trajectory_setpoint.position[0], trajectory_setpoint.position[1],
						   trajectory_setpoint.position[2]};
			const Vector3f position_meas{position.x, position.y, position.z};
			velocity_sp = _position_gain * (position_sp - position_meas);
			have_command = true;
		}

		if (!have_command) {
			return;
		}

		const float max_tilt_rad = math::min(_guidance_tilt_max_deg * kDegToRad, 35.f * kDegToRad);
		const Quatf launch_reference{_launch_reference_q};
		const Vector3f thrust_world = launch_reference.rotateVector(Vector3f{1.f, 0.f, 0.f});
		Vector3f horiz_command{velocity_sp(0), velocity_sp(1), 0.f};

		if (hover_lateral) {
			const Vector3f velocity_meas{position.vx, position.vy, position.vz};
			const Vector3f vel_error = velocity_sp - velocity_meas;
			horiz_command = Vector3f{vel_error(0), vel_error(1), 0.f};
		}

		const float horiz_norm = horiz_command.norm();

		if (horiz_norm > 1e-3f) {
			const Vector3f horiz_dir = horiz_command / horiz_norm;
			const float tilt_angle = hover_lateral
						 ? math::constrain(_hover_vel_p * horiz_norm / kGravityMps2, 0.f, max_tilt_rad)
						 : math::constrain(horiz_norm * _guidance_tilt_gain, 0.f, max_tilt_rad);
			Vector3f axis = thrust_world.cross(horiz_dir);

			if (axis.norm() > 1e-6f) {
				axis.normalize();
				const Quatf tilt_offset{AxisAnglef{axis, tilt_angle}};
				const Quatf guided_reference = tilt_offset * launch_reference;
				guided_reference.copyTo(_reference_q);
			}
		}
	}

	void reset_controller()
	{
		_integrator.zero();
		_reference_initialized = false;
		_prev_rail_exit = false;
	}

	void publish_zero(hrt_abstime now, hrt_abstime timestamp_sample)
	{
		vehicle_torque_setpoint_s torque{};
		torque.timestamp = now;
		torque.timestamp_sample = timestamp_sample;
		_torque_pub.publish(torque);

		vehicle_thrust_setpoint_s thrust{};
		thrust.timestamp = now;
		thrust.timestamp_sample = timestamp_sample;
		_thrust_pub.publish(thrust);
	}

	void update_parameters()
	{
		param_t p = param_find("RK_ATT_P_RAIL");
		if (p != PARAM_INVALID) {
			param_get(p, &_att_p_rail);
		}

		p = param_find("RK_ATT_P_FREE");
		if (p != PARAM_INVALID) {
			param_get(p, &_att_p_free);
		}

		p = param_find("RK_RATE_P_RAIL");
		if (p != PARAM_INVALID) {
			param_get(p, &_rate_p_rail);
		}

		p = param_find("RK_RATE_P_FREE");
		if (p != PARAM_INVALID) {
			param_get(p, &_rate_p_free);
		}

		p = param_find("RK_RATE_I");
		if (p != PARAM_INVALID) {
			param_get(p, &_rate_i);
		}

		p = param_find("RK_RATE_D");
		if (p != PARAM_INVALID) {
			param_get(p, &_rate_d);
		}

		p = param_find("RK_ATT_P_BOOST");
		if (p != PARAM_INVALID) {
			param_get(p, &_att_p_boost);
		}

		p = param_find("RK_RATE_P_BOOST");
		if (p != PARAM_INVALID) {
			param_get(p, &_rate_p_boost);
		}

		p = param_find("RK_INT_LIM_BOOST");
		if (p != PARAM_INVALID) {
			param_get(p, &_integrator_limit_boost);
		}

		p = param_find("RK_GD_ENABLE");
		if (p != PARAM_INVALID) {
			param_get(p, &_guidance_enabled);
		}

		p = param_find("RK_GD_BOOST_ATT");
		if (p != PARAM_INVALID) {
			param_get(p, &_boost_attitude_only);
		}

		p = param_find("RK_GD_BOOST_FULL");
		if (p != PARAM_INVALID) {
			param_get(p, &_boost_full_thrust);
		}

		p = param_find("RK_GD_POS_P");
		if (p != PARAM_INVALID) {
			float pos_p = _position_gain;
			param_get(p, &pos_p);
			_position_gain = pos_p;
		}

		p = param_find("RK_GD_SIM_GT");
		if (p != PARAM_INVALID) {
			param_get(p, &_sim_groundtruth_fallback);
		}

		p = param_find("RK_ATT_TILT_GAIN");
		if (p != PARAM_INVALID) {
			param_get(p, &_guidance_tilt_gain);
		}

		p = param_find("RK_ATT_VEL_P");
		if (p != PARAM_INVALID) {
			param_get(p, &_hover_vel_p);
		}

		p = param_find("RK_ATT_TILT_MAX");
		if (p != PARAM_INVALID) {
			param_get(p, &_guidance_tilt_max_deg);
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

		p = param_find("RK_ATT_ROLL_DEG");
		if (p != PARAM_INVALID) {
			param_get(p, &_roll_program_deg);
		}

		p = param_find("RK_ATT_ROLL_T0");
		if (p != PARAM_INVALID) {
			param_get(p, &_roll_program_start_s);
		}

		p = param_find("RK_ATT_ROLL_DT");
		if (p != PARAM_INVALID) {
			param_get(p, &_roll_program_dur_s);
		}
	}

	float _att_p_rail{2.f};
	float _att_p_free{3.f};
	float _att_p_boost{8.f};
	float _rate_p_rail{0.35f};
	float _rate_p_free{0.5f};
	float _rate_p_boost{2.f};
	float _rate_i{0.04f};
	float _rate_d{0.003f};
	float _torque_roll_max{0.f};
	float _torque_pitch_max{10.f};
	float _torque_yaw_max{10.f};
	float _integrator_limit{5.f};
	float _integrator_limit_boost{15.f};
	int32_t _guidance_enabled{0};
	int32_t _boost_attitude_only{0};
	int32_t _boost_full_thrust{0};
	int32_t _sim_groundtruth_fallback{0};
	float _position_gain{0.14f};
	float _guidance_tilt_gain{0.12f};
	float _hover_vel_p{2.f};
	float _guidance_tilt_max_deg{20.f};
	float _roll_program_deg{0.f};
	float _roll_program_start_s{0.f};
	float _roll_program_dur_s{0.f};

	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_local_position_s _local_position{};
	vehicle_local_position_s _groundtruth_position{};
	tv3_status_s _status{};
	tv3_thrust_s _thrust{};
	tv3_guidance_status_s _guidance_status{};
	tv3_control_authority_s _control_authority{};
	Vector3f _integrator{};
	float _launch_reference_q[4]{1.f, 0.f, 0.f, 0.f};
	bool _prev_rail_exit{false};
	float _reference_q[4]{1.f, 0.f, 0.f, 0.f};
	bool _reference_initialized{false};
	hrt_abstime _last_update{0};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _groundtruth_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _tv3_status_sub{ORB_ID(tv3_status)};
	uORB::Subscription _tv3_thrust_sub{ORB_ID(tv3_thrust)};
	uORB::Subscription _tv3_guidance_status_sub{ORB_ID(tv3_guidance_status)};
	uORB::Subscription _tv3_control_authority_sub{ORB_ID(tv3_control_authority)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
};

extern "C" __EXPORT int tv3_attitude_main(int argc, char *argv[])
{
	return TV3Attitude::main(argc, argv);
}
