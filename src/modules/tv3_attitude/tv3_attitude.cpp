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
}

class TV3AttControl : public ModuleBase<TV3AttControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3AttControl() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3AttControl *instance = new TV3AttControl();

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

		PRINT_MODULE_DESCRIPTION("TV3 thrust-vector control mixer publishing torque and thrust setpoints.");
		PRINT_MODULE_USAGE_NAME("tv3_att_control", "modules");
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

		const Vector3f rate_sp{att_p * att_error(0), att_p * att_error(1), att_p * att_error(2)};
		const Vector3f rate_meas{_angular_velocity.xyz};
		const Vector3f rate_error = rate_sp - rate_meas;
		const float integrator_limit = powered_flight ? _integrator_limit_boost : _integrator_limit;

		_integrator(0) = math::constrain(_integrator(0) + rate_error(0) * _rate_i * dt, -integrator_limit, integrator_limit);
		_integrator(1) = math::constrain(_integrator(1) + rate_error(1) * _rate_i * dt, -integrator_limit, integrator_limit);
		_integrator(2) = math::constrain(_integrator(2) + rate_error(2) * _rate_i * dt, -integrator_limit, integrator_limit);

		const float torque_roll = math::constrain(rate_p * rate_error(0) + _integrator(0) - _rate_d * _angular_velocity.xyz_derivative[0],
						     -_torque_roll_max, _torque_roll_max);
		const float torque_pitch = math::constrain(rate_p * rate_error(1) + _integrator(1) - _rate_d * _angular_velocity.xyz_derivative[1],
						      -_torque_pitch_max, _torque_pitch_max);
		const float torque_yaw = math::constrain(rate_p * rate_error(2) + _integrator(2) - _rate_d * _angular_velocity.xyz_derivative[2],
						    -_torque_yaw_max, _torque_yaw_max);

		vehicle_torque_setpoint_s torque{};
		torque.timestamp = now;
		torque.timestamp_sample = _attitude.timestamp_sample;
		torque.xyz[0] = torque_roll;
		torque.xyz[1] = torque_pitch;
		torque.xyz[2] = torque_yaw;
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

	void update_guidance_reference(const trajectory_setpoint_s &trajectory_setpoint, bool trajectory_updated, hrt_abstime now)
	{
		memcpy(_reference_q, _launch_reference_q, sizeof(_reference_q));

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
		Vector3f velocity_sp{0.f, 0.f, 0.f};
		bool have_command = false;

		if (PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])
		    && PX4_ISFINITE(trajectory_setpoint.velocity[2])) {
			velocity_sp = Vector3f{trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1],
					       trajectory_setpoint.velocity[2]};
			have_command = velocity_sp.norm_squared() > 1e-6f;
		}

		if (PX4_ISFINITE(trajectory_setpoint.position[0]) && PX4_ISFINITE(trajectory_setpoint.position[1])
		    && PX4_ISFINITE(trajectory_setpoint.position[2])) {
			const Vector3f position_sp{trajectory_setpoint.position[0], trajectory_setpoint.position[1],
						   trajectory_setpoint.position[2]};
			const Vector3f position_meas{position.x, position.y, position.z};
			velocity_sp += _position_gain * (position_sp - position_meas);
			have_command = true;
		}

		if (!have_command) {
			return;
		}

		const float max_tilt_rad = math::min(_guidance_tilt_max_deg * kDegToRad, 35.f * kDegToRad);
		const Quatf launch_reference{_launch_reference_q};
		const Vector3f thrust_world = launch_reference.rotateVector(Vector3f{1.f, 0.f, 0.f});
		Vector3f horiz_sp{velocity_sp(0), velocity_sp(1), 0.f};
		const float horiz_norm = horiz_sp.norm();

		if (horiz_norm > 1e-3f) {
			horiz_sp /= horiz_norm;
			const float tilt_angle = math::constrain(horiz_norm * _guidance_tilt_gain, 0.f, max_tilt_rad);
			Vector3f axis = thrust_world.cross(horiz_sp);

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
	int32_t _sim_groundtruth_fallback{0};
	float _position_gain{0.14f};
	float _guidance_tilt_gain{0.12f};
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
	Vector3f _integrator{};
	float _launch_reference_q[4]{1.f, 0.f, 0.f, 0.f};
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
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
};

extern "C" __EXPORT int tv3_att_control_main(int argc, char *argv[])
{
	return TV3AttControl::main(argc, argv);
}
