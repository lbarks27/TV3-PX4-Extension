#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rocket_status.h>
#include <uORB/topics/rocket_thrust.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector3f;

class RocketAttControl : public ModuleBase<RocketAttControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RocketAttControl() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		RocketAttControl *instance = new RocketAttControl();

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

		PRINT_MODULE_DESCRIPTION("Rocket thrust-vector control mixer publishing torque and thrust setpoints.");
		PRINT_MODULE_USAGE_NAME("rocket_att_control", "modules");
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
		PX4_INFO("integrator pitch %.3f yaw %.3f", (double)_integrator(1), (double)_integrator(2));
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
		_rocket_status_sub.update(&_status);
		_rocket_thrust_sub.update(&_thrust);

		const hrt_abstime now = hrt_absolute_time();
		const float dt = _last_update != 0 ? static_cast<float>(now - _last_update) * 1e-6f : 0.01f;
		_last_update = now;

		if (_status.mode < rocket_status_s::MODE_READY || _status.mode == rocket_status_s::MODE_ABORT || _status.mode == rocket_status_s::MODE_COAST) {
			reset_controller();
			publish_zero(now, _attitude.timestamp_sample);
			return;
		}

		if (!_reference_initialized) {
			memcpy(_reference_q, _attitude.q, sizeof(_reference_q));
			_reference_initialized = true;
		}

		Quatf q(_attitude.q);
		Quatf q_ref(_reference_q);
		Quatf q_error = q.inversed() * q_ref;

		if (q_error(0) < 0.f) {
			q_error = -q_error;
		}

		const Vector3f att_error = 2.f * q_error.imag();
		const bool rail_mode = !_status.rail_exit;
		const float att_p = rail_mode ? _att_p_rail : _att_p_free;
		const float rate_p = rail_mode ? _rate_p_rail : _rate_p_free;
		const Vector3f rate_sp{0.f, att_p * att_error(1), att_p * att_error(2)};
		const Vector3f rate_meas{_angular_velocity.xyz};
		const Vector3f rate_error = rate_sp - rate_meas;

		_integrator(1) = math::constrain(_integrator(1) + rate_error(1) * _rate_i * dt, -_integrator_limit, _integrator_limit);
		_integrator(2) = math::constrain(_integrator(2) + rate_error(2) * _rate_i * dt, -_integrator_limit, _integrator_limit);

		const float torque_pitch = math::constrain(rate_p * rate_error(1) + _integrator(1) - _rate_d * _angular_velocity.xyz_derivative[1],
						      -_torque_pitch_max, _torque_pitch_max);
		const float torque_yaw = math::constrain(rate_p * rate_error(2) + _integrator(2) - _rate_d * _angular_velocity.xyz_derivative[2],
						    -_torque_yaw_max, _torque_yaw_max);

		vehicle_torque_setpoint_s torque{};
		torque.timestamp = now;
		torque.timestamp_sample = _attitude.timestamp_sample;
		torque.xyz[0] = 0.f;
		torque.xyz[1] = torque_pitch;
		torque.xyz[2] = torque_yaw;
		_torque_pub.publish(torque);

		vehicle_thrust_setpoint_s thrust{};
		thrust.timestamp = now;
		thrust.timestamp_sample = _attitude.timestamp_sample;
		thrust.xyz[0] = (_status.mode == rocket_status_s::MODE_IGNITION_PENDING || _status.mode == rocket_status_s::MODE_BOOST) ? 1.f : 0.f;
		thrust.xyz[1] = 0.f;
		thrust.xyz[2] = 0.f;
		_thrust_pub.publish(thrust);
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

		p = param_find("RK_TQ_P_MAX");
		if (p != PARAM_INVALID) {
			param_get(p, &_torque_pitch_max);
		}

		p = param_find("RK_TQ_Y_MAX");
		if (p != PARAM_INVALID) {
			param_get(p, &_torque_yaw_max);
		}
	}

	float _att_p_rail{2.f};
	float _att_p_free{3.f};
	float _rate_p_rail{0.35f};
	float _rate_p_free{0.5f};
	float _rate_i{0.04f};
	float _rate_d{0.003f};
	float _torque_pitch_max{10.f};
	float _torque_yaw_max{10.f};
	float _integrator_limit{5.f};

	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	rocket_status_s _status{};
	rocket_thrust_s _thrust{};
	Vector3f _integrator{};
	float _reference_q[4]{1.f, 0.f, 0.f, 0.f};
	bool _reference_initialized{false};
	hrt_abstime _last_update{0};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _rocket_status_sub{ORB_ID(rocket_status)};
	uORB::Subscription _rocket_thrust_sub{ORB_ID(rocket_thrust)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
};

extern "C" __EXPORT int rocket_att_control_main(int argc, char *argv[])
{
	return RocketAttControl::main(argc, argv);
}
