#include "tv3_plant_kinematics.hpp"

#include <mathlib/mathlib.h>

namespace tv3
{

matrix::Vector3f rotate_about_axis(const matrix::Vector3f &vector, const matrix::Vector3f &axis, float angle_rad)
{
	if (fabsf(angle_rad) < 1e-6f) {
		return vector;
	}

	matrix::Vector3f axis_unit = axis;

	if (axis_unit.norm() > 1e-6f) {
		axis_unit.normalize();
	}

	const float c = cosf(angle_rad);
	const float s = sinf(angle_rad);
	const float kdv = axis_unit.dot(vector);
	return vector * c + axis_unit.cross(vector) * s + axis_unit * kdv * (1.f - c);
}

matrix::Vector3f thrust_direction_at(const EngineGeometry &geometry, float pitch_rad, float yaw_rad)
{
	matrix::Vector3f direction = geometry.thrust_axis;

	if (fabsf(pitch_rad) > 1e-6f) {
		direction = rotate_about_axis(direction, geometry.primary_axis, pitch_rad);
	}

	if (fabsf(yaw_rad) > 1e-6f) {
		matrix::Vector3f yaw_axis = geometry.secondary_axis;

		if (fabsf(pitch_rad) > 1e-6f) {
			yaw_axis = rotate_about_axis(yaw_axis, geometry.primary_axis, pitch_rad);
		}

		direction = rotate_about_axis(direction, yaw_axis, yaw_rad);
	}

	const float norm = direction.norm();

	if (norm > 1e-6f) {
		direction /= norm;
	}

	return direction;
}

matrix::Vector3f group_torque_nm(const EngineGeometry &geometry, float chamber_thrust_n, float pitch_rad,
				 float yaw_rad)
{
	if (chamber_thrust_n < 0.5f) {
		return matrix::Vector3f{};
	}

	return geometry.position.cross(thrust_direction_at(geometry, pitch_rad, yaw_rad) * chamber_thrust_n);
}

matrix::Vector3f total_torque_nm(const AllocationInput &input, const float pitch_rad[kMaxEngines],
				 const float yaw_rad[kMaxEngines])
{
	matrix::Vector3f total{};

	for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
		if ((input.ignition_mask & (1u << engine_index)) == 0) {
			continue;
		}

		total += group_torque_nm(input.geometry[engine_index], input.chamber_thrust_n[engine_index],
					 pitch_rad[engine_index], yaw_rad[engine_index]);
	}

	return total;
}

float total_axial_thrust_n(const AllocationInput &input, const float pitch_rad[kMaxEngines],
			   const float yaw_rad[kMaxEngines])
{
	float total = 0.f;

	for (int engine_index = 0; engine_index < input.engine_count; ++engine_index) {
		if ((input.ignition_mask & (1u << engine_index)) == 0 || input.chamber_thrust_n[engine_index] < 0.5f) {
			continue;
		}

		total += thrust_direction_at(input.geometry[engine_index], pitch_rad[engine_index],
					     yaw_rad[engine_index])(0) * input.chamber_thrust_n[engine_index];
	}

	return total;
}

float engine_chamber_thrust_n(float filtered, float measured, float expected)
{
	float thrust_n = filtered;

	if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
		thrust_n = measured;
	}

	if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
		thrust_n = expected;
	}

	return PX4_ISFINITE(thrust_n) ? math::max(thrust_n, 0.f) : 0.f;
}

} // namespace tv3
