#pragma once

#include "tv3_engine_geometry.hpp"

namespace tv3
{

matrix::Vector3f rotate_about_axis(const matrix::Vector3f &vector, const matrix::Vector3f &axis, float angle_rad);

matrix::Vector3f thrust_direction_at(const EngineGeometry &geometry, float pitch_rad, float yaw_rad);

matrix::Vector3f group_torque_nm(const EngineGeometry &geometry, float chamber_thrust_n, float pitch_rad,
				 float yaw_rad);

matrix::Vector3f total_torque_nm(const AllocationInput &input, const float pitch_rad[kMaxEngines],
				 const float yaw_rad[kMaxEngines]);

float total_axial_thrust_n(const AllocationInput &input, const float pitch_rad[kMaxEngines],
			   const float yaw_rad[kMaxEngines]);

} // namespace tv3
