#include "tv3_attitude_controller.hpp"

#include <mathlib/mathlib.h>

namespace tv3
{

using matrix::Quatf;
using matrix::Vector3f;

void AttitudeController::reset()
{
	_integrator.zero();
}

Vector3f AttitudeController::attitude_error(const Quatf &q_meas, const Quatf &q_sp) const
{
	return quaternion_attitude_error(q_meas.inversed() * q_sp);
}

AttitudeControllerRegion AttitudeController::region_for_error(const Vector3f &att_error) const
{
	float max_axis_error = 0.f;

	for (int axis = 0; axis < 3; ++axis) {
		max_axis_error = math::max(max_axis_error, fabsf(att_error(axis)));
	}

	if (max_axis_error <= _config.deadband_rad) {
		return AttitudeControllerRegion::Deadband;
	}

	if (max_axis_error >= _config.large_error_rad) {
		return AttitudeControllerRegion::LargeError;
	}

	return AttitudeControllerRegion::SmallError;
}

Vector3f AttitudeController::quaternion_attitude_error(const Quatf &q_error) const
{
	Quatf e = q_error;

	if (e(0) < 0.f) {
		e = -e;
	}

	const Vector3f imaginary{e(1), e(2), e(3)};
	const float sin_half = imaginary.norm();

	if (sin_half < 1e-6f) {
		return 2.f * imaginary;
	}

	const float angle = 2.f * atan2f(sin_half, e(0));
	return imaginary * (angle / sin_half);
}

float AttitudeController::max_alpha_rad_s2(int axis) const
{
	float moi = _config.moi_pitch_kgm2;
	float torque_max = _config.torque_pitch_max;

	if (axis == 0) {
		moi = _config.moi_roll_kgm2;
		torque_max = _config.torque_roll_max;
	} else if (axis == 2) {
		moi = _config.moi_yaw_kgm2;
		torque_max = _config.torque_yaw_max;
	}

	moi = math::max(moi, 1e-4f);
	return torque_max / moi;
}

Vector3f AttitudeController::position_loop(const Vector3f &att_error, AttitudeControllerRegion region) const
{
	Vector3f rate_sp{};

	for (int axis = 0; axis < 3; ++axis) {
		const float error = att_error(axis);
		const float abs_error = fabsf(error);

		if (region == AttitudeControllerRegion::Deadband || abs_error <= _config.deadband_rad) {
			rate_sp(axis) = 0.f;
			continue;
		}

		if (region == AttitudeControllerRegion::LargeError || abs_error >= _config.large_error_rad) {
			const float max_alpha = max_alpha_rad_s2(axis);
			const float eff_ld = _config.ld_rad;
			const float stopping_distance = math::max(abs_error - eff_ld, 0.f);
			rate_sp(axis) = _config.soften * copysignf(sqrtf(2.f * max_alpha * stopping_distance), error);
		} else {
			rate_sp(axis) = _config.pos_kp * error;
		}
	}

	const float total_error = att_error.norm();

	if (total_error > _config.roll_control_range_rad) {
		rate_sp(0) = 0.f;
	}

	return rate_sp;
}

Vector3f AttitudeController::velocity_loop(const Vector3f &rate_sp, const Vector3f &rate_meas,
		const Vector3f &att_error, float dt_s)
{
	Vector3f rate_error = rate_sp - rate_meas;
	Vector3f alpha_target{};

	for (int axis = 0; axis < 3; ++axis) {
		_integrator(axis) = math::constrain(_integrator(axis) + rate_error(axis) * _config.vel_ki * dt_s,
						    -_config.integrator_limit, _config.integrator_limit);
		alpha_target(axis) = _config.vel_kp * rate_error(axis) + _integrator(axis) - _config.vel_kd * rate_meas(axis);
		const float max_alpha = max_alpha_rad_s2(axis);
		alpha_target(axis) = math::constrain(alpha_target(axis), -max_alpha, max_alpha);
	}

	return alpha_target;
}

Vector3f AttitudeController::torque_from_alpha(const Vector3f &alpha_target) const
{
	return Vector3f{
		_config.moi_roll_kgm2 * alpha_target(0),
		_config.moi_pitch_kgm2 * alpha_target(1),
		_config.moi_yaw_kgm2 * alpha_target(2),
	};
}

Vector3f AttitudeController::update(AttitudeControllerRegion region,
				    const Quatf &q_meas,
				    const Quatf &q_sp,
				    const Vector3f &rate_meas,
				    float dt_s)
{
	if (region == AttitudeControllerRegion::Off) {
		reset();
		return Vector3f{};
	}

	const Quatf q_error = q_meas.inversed() * q_sp;
	const Vector3f att_error = quaternion_attitude_error(q_error);

	if (region == AttitudeControllerRegion::Deadband && att_error.norm() <= _config.deadband_rad) {
		reset();
		return Vector3f{};
	}

	const Vector3f rate_sp = position_loop(att_error, region);
	const Vector3f alpha_target = velocity_loop(rate_sp, rate_meas, att_error, dt_s);
	Vector3f torque = torque_from_alpha(alpha_target);

	torque(0) = math::constrain(torque(0), -_config.torque_roll_max, _config.torque_roll_max);
	torque(1) = math::constrain(torque(1), -_config.torque_pitch_max, _config.torque_pitch_max);
	torque(2) = math::constrain(torque(2), -_config.torque_yaw_max, _config.torque_yaw_max);
	return torque;
}

} // namespace tv3
