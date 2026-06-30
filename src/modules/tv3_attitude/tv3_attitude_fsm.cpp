#include "tv3_attitude_fsm.hpp"

#include <lib/tv3_msg_fields.hpp>

#include <matrix/matrix/math.hpp>

namespace tv3
{

using matrix::Quatf;
using matrix::Vector3f;

void AttitudeFsm::apply_module_mode(const tv3_sm_modes_s &modes)
{
	_enabled = modes.attitude_mode != tv3_sm_modes_s::ATTITUDE_OFF;

	if (!_enabled) {
		_fsm.request(AttitudeMode::Off);
		_fsm.apply_request();
	}
}

AttitudeMode AttitudeFsm::mode_for_region(AttitudeControllerRegion region)
{
	switch (region) {
	case AttitudeControllerRegion::Deadband: return AttitudeMode::Deadband;
	case AttitudeControllerRegion::LargeError: return AttitudeMode::LargeError;
	case AttitudeControllerRegion::SmallError: return AttitudeMode::SmallError;
	default: return AttitudeMode::Off;
	}
}

void AttitudeFsm::step(hrt_abstime now,
		       float dt,
		       const vehicle_attitude_s &attitude,
		       const vehicle_angular_velocity_s &angular_velocity,
		       const tv3_gd_att_sp_s &attitude_setpoint,
		       vehicle_torque_setpoint_s &torque_setpoint)
{
	Vector3f torque{};

	if (_enabled && attitude_setpoint.valid) {
		const Quatf q_meas{attitude.q};
		const Quatf q_sp{attitude_setpoint_quat(attitude_setpoint)};
		const Vector3f rate_meas{angular_velocity.xyz};
		const Vector3f att_error = _controller.attitude_error(q_meas, q_sp);
		const AttitudeControllerRegion region = _controller.region_for_error(att_error);

		_fsm.request(mode_for_region(region));
		_fsm.apply_request();

		torque = _controller.update(region, q_meas, q_sp, rate_meas, dt);
	} else {
		_fsm.request(AttitudeMode::Off);
		_fsm.apply_request();
		_controller.reset();
	}

	torque_setpoint.timestamp = now;
	torque_setpoint.timestamp_sample = attitude.timestamp_sample;
	torque_setpoint.xyz[0] = torque(0);
	torque_setpoint.xyz[1] = torque(1);
	torque_setpoint.xyz[2] = torque(2);
}

} // namespace tv3
