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
#include <uORB/topics/tv3_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;
using matrix::AxisAnglef;
using matrix::Quatf;
using matrix::Vector3f;

namespace
{
constexpr float kDegToRad = 0.017453292519943295769f;
}

class TV3AttitudeReference : public ModuleBase<TV3AttitudeReference>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3AttitudeReference() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3AttitudeReference *instance = new TV3AttitudeReference();

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

		PRINT_MODULE_DESCRIPTION("TV3 attitude reference generator from launch frame, guidance, and roll program.");
		PRINT_MODULE_USAGE_NAME("tv3_attitude_reference", "modules");
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

		_tv3_status_sub.update(&_status);
		_local_position_sub.update(&_local_position);
		_groundtruth_position_sub.update(&_groundtruth_position);

		vehicle_attitude_s attitude{};

		if (!_vehicle_attitude_sub.update(&attitude)) {
			return;
		}

		trajectory_setpoint_s trajectory_setpoint{};
		const bool trajectory_updated = _trajectory_setpoint_sub.update(&trajectory_setpoint);
		const hrt_abstime now = hrt_absolute_time();
		const bool coast_without_guidance = _status.mode == tv3_status_s::MODE_COAST && _guidance_enabled <= 0;

		vehicle_attitude_setpoint_s setpoint{};
		setpoint.timestamp = now;

		if (_status.mode < tv3_status_s::MODE_READY || _status.mode == tv3_status_s::MODE_ABORT || coast_without_guidance) {
			reset_reference();
			setpoint.q_d_valid = false;
			_setpoint_pub.publish(setpoint);
			return;
		}

		if (!_reference_initialized) {
			memcpy(_launch_reference_q, attitude.q, sizeof(_launch_reference_q));
			_reference_initialized = true;
		}

		update_guidance_reference(trajectory_setpoint, trajectory_updated, now);
		apply_roll_program();

		memcpy(setpoint.q_d, _reference_q, sizeof(setpoint.q_d));
		setpoint.q_d_valid = true;
		_setpoint_pub.publish(setpoint);
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

	void reset_reference()
	{
		_reference_initialized = false;
	}

	void update_parameters()
	{
		param_t p = param_find("RK_GD_ENABLE");

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

	int32_t _guidance_enabled{0};
	int32_t _sim_groundtruth_fallback{0};
	float _position_gain{0.14f};
	float _guidance_tilt_gain{0.12f};
	float _guidance_tilt_max_deg{20.f};
	float _roll_program_deg{0.f};
	float _roll_program_start_s{0.f};
	float _roll_program_dur_s{0.f};

	tv3_status_s _status{};
	vehicle_local_position_s _local_position{};
	vehicle_local_position_s _groundtruth_position{};
	float _launch_reference_q[4]{1.f, 0.f, 0.f, 0.f};
	float _reference_q[4]{1.f, 0.f, 0.f, 0.f};
	bool _reference_initialized{false};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _groundtruth_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _tv3_status_sub{ORB_ID(tv3_status)};
	uORB::Publication<vehicle_attitude_setpoint_s> _setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
};

extern "C" __EXPORT int tv3_attitude_reference_main(int argc, char *argv[])
{
	return TV3AttitudeReference::main(argc, argv);
}
