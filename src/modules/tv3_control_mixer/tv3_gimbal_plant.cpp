#include "tv3_gimbal_plant.hpp"

#include <mathlib/mathlib.h>

namespace tv3
{

namespace
{

matrix::Vector3f rotate_about_axis(const matrix::Vector3f &v, const matrix::Vector3f &axis, float angle_rad)
{
	if (fabsf(angle_rad) < 1e-6f) {
		return v;
	}

	matrix::Vector3f k = axis;
	const float nn = k.norm();

	if (nn > 1e-6f) {
		k /= nn;
	}

	const float c = cosf(angle_rad);
	const float s = sinf(angle_rad);
	const float kdv = k.dot(v);
	return v * c + k.cross(v) * s + k * kdv * (1.f - c);
}

bool engine_active(int engine, int ignition_mask, float thrust_n)
{
	return engine >= 0
	       && engine < kGimbalMaxEngines
	       && (ignition_mask & (1 << engine)) != 0
	       && thrust_n > 0.5f;
}

} // namespace

matrix::Vector3f GimbalPlant::thrust_direction(int engine, float primary_rad, float yaw_rad) const
{
	if (engine < 0 || engine >= kGimbalMaxEngines) {
		return matrix::Vector3f{1.f, 0.f, 0.f};
	}

	matrix::Vector3f direction = geometry.thrust_axis[engine];

	if (fabsf(primary_rad) > 1e-6f) {
		direction = rotate_about_axis(direction, geometry.primary_axis[engine], primary_rad);
	}

	if (fabsf(yaw_rad) > 1e-6f) {
		matrix::Vector3f yaw_axis = geometry.secondary_axis[engine];

		if (fabsf(primary_rad) > 1e-6f) {
			yaw_axis = rotate_about_axis(yaw_axis, geometry.primary_axis[engine], primary_rad);
		}

		direction = rotate_about_axis(direction, yaw_axis, yaw_rad);
	}

	const float n = direction.norm();

	if (n > 1e-6f) {
		direction /= n;
	}

	return direction;
}

matrix::Vector3f GimbalPlant::engine_torque(int engine, float primary_rad, float yaw_rad, float thrust_n) const
{
	if (engine < 0 || engine >= kGimbalMaxEngines || thrust_n < 0.5f) {
		return matrix::Vector3f{};
	}

	const matrix::Vector3f force = thrust_direction(engine, primary_rad, yaw_rad) * thrust_n;
	return geometry.pos[engine].cross(force);
}

float GimbalPlant::engine_axial_thrust(int engine, float primary_rad, float yaw_rad, float thrust_n) const
{
	if (engine < 0 || engine >= kGimbalMaxEngines || thrust_n < 0.5f) {
		return 0.f;
	}

	return thrust_direction(engine, primary_rad, yaw_rad)(0) * thrust_n;
}

GimbalWrenchResult GimbalPlant::total_wrench(const float primary_rad[kGimbalMaxEngines],
		const float yaw_rad[kGimbalMaxEngines],
		const float thrust_n[kGimbalMaxEngines],
		int ignition_mask) const
{
	GimbalWrenchResult result{};

	const int count = math::min(engine_count, kGimbalMaxEngines);

	for (int i = 0; i < count; ++i) {
		if (!engine_active(i, ignition_mask, thrust_n[i])) {
			continue;
		}

		const matrix::Vector3f direction = thrust_direction(i, primary_rad[i], yaw_rad[i]);
		const matrix::Vector3f force = direction * thrust_n[i];
		result.body_force_n += force;
		result.torque_nm += geometry.pos[i].cross(force);
		result.axial_thrust_n += direction(0) * thrust_n[i];
	}

	return result;
}

} // namespace tv3
