#include "tv3_guidance_fsm.hpp"

#include <lib/tv3_msg_fields.hpp>

#include <mathlib/mathlib.h>

#include <matrix/matrix/math.hpp>

#include <cstring>

namespace tv3
{

using matrix::AxisAnglef;
using matrix::Quatf;
using matrix::Vector3f;

void GuidanceFsm::apply_module_mode(const tv3_sm_modes_s &modes)
{
	switch (modes.guidance_mode) {
	case tv3_sm_modes_s::GUIDANCE_UP:
		_fsm.request(GuidanceMode::Up);
		break;

	case tv3_sm_modes_s::GUIDANCE_WP_FLY:
		_fsm.request(GuidanceMode::WaypointFlyThrough);
		break;

	default:
		_fsm.request(GuidanceMode::Off);
		break;
	}

	_fsm.apply_request();
}

float GuidanceFsm::active_chamber_thrust_n(const tv3_lc_eng_st_s &engine_state)
{
	float total = 0.f;
	const int engine_count = math::min(static_cast<int>(engine_state.engine_count), 4);

	for (int i = 0; i < engine_count; ++i) {
		if ((engine_state.ignition_mask & (1u << i)) == 0) {
			continue;
		}

		float thrust_n = filtered_thrust_n(engine_state, i);

		if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
			thrust_n = measured_thrust_n(engine_state, i);
		}

		if (!PX4_ISFINITE(thrust_n) || thrust_n <= 0.f) {
			thrust_n = expected_thrust_n(engine_state, i);
		}

		total += PX4_ISFINITE(thrust_n) ? math::max(thrust_n, 0.f) : 0.f;
	}

	return total;
}

bool GuidanceFsm::position_valid(const vehicle_local_position_s &position, hrt_abstime now) const
{
	if (position.timestamp == 0) {
		return false;
	}

	if (!position.xy_valid || !position.z_valid) {
		return false;
	}

	if (!PX4_ISFINITE(position.x) || !PX4_ISFINITE(position.y) || !PX4_ISFINITE(position.z)) {
		return false;
	}

	const hrt_abstime age = now > position.timestamp ? now - position.timestamp : 0;
	return age <= 500000ULL;
}

bool GuidanceFsm::guidance_position_valid(hrt_abstime now,
		const vehicle_local_position_s &local_position,
		const vehicle_local_position_s &groundtruth_position) const
{
	return position_valid(local_position, now)
	       || (_waypoint_config.sim_groundtruth_fallback > 0 && position_valid(groundtruth_position, now));
}

const vehicle_local_position_s &GuidanceFsm::guidance_position(hrt_abstime now,
		const vehicle_local_position_s &local_position,
		const vehicle_local_position_s &groundtruth_position) const
{
	if (position_valid(local_position, now)) {
		return local_position;
	}

	return groundtruth_position;
}

void GuidanceFsm::capture_launch_reference(const vehicle_attitude_s &attitude)
{
	if (!_launch_reference_valid && attitude.timestamp != 0) {
		memcpy(_launch_reference_q, attitude.q, sizeof(_launch_reference_q));
		_launch_reference_valid = true;
	}
}

void GuidanceFsm::set_attitude_toward_direction(const float launch_q[4], const Vector3f &direction_ned,
		float max_tilt_rad, float out_q[4]) const
{
	memcpy(out_q, launch_q, sizeof(float) * 4);

	const float dir_norm = direction_ned.norm();

	if (dir_norm < 1e-3f) {
		return;
	}

	const Vector3f direction = direction_ned / dir_norm;
	const Quatf launch_reference{launch_q};
	const Vector3f thrust_world = launch_reference.rotateVector(Vector3f{1.f, 0.f, 0.f});
	const float dot = math::constrain(thrust_world.dot(direction), -1.f, 1.f);
	float tilt_angle = acosf(dot);

	if (tilt_angle > max_tilt_rad) {
		tilt_angle = max_tilt_rad;
	}

	if (tilt_angle < 1e-4f) {
		return;
	}

	Vector3f axis = thrust_world.cross(direction);

	if (axis.norm() < 1e-6f) {
		return;
	}

	axis.normalize();
	const Quatf guided = Quatf{AxisAnglef{axis, tilt_angle}} * launch_reference;
	guided.copyTo(out_q);
}

void GuidanceFsm::capture_origin_if_needed(const vehicle_local_position_s &position)
{
	if (!_origin_valid) {
		_origin_ned[0] = position.x;
		_origin_ned[1] = position.y;
		_origin_ned[2] = position.z;
		_origin_valid = true;
	}
}

void GuidanceFsm::step_up(hrt_abstime now,
			  const vehicle_attitude_s &attitude,
			  const tv3_lc_eng_st_s &engine_state,
			  tv3_gd_att_sp_s &attitude_setpoint,
			  tv3_gd_thr_sp_s &thrust_setpoint)
{
	capture_launch_reference(attitude);

	if (_launch_reference_valid) {
		attitude_setpoint.valid = true;
		set_attitude_setpoint_q(attitude_setpoint, _launch_reference_q);
	}

	thrust_setpoint.valid = true;
	thrust_setpoint.axial_n = active_chamber_thrust_n(engine_state);
}

void GuidanceFsm::step_waypoint_fly_through(hrt_abstime now,
		const vehicle_attitude_s &attitude,
		const vehicle_local_position_s &local_position,
		const vehicle_local_position_s &groundtruth_position,
		const tv3_lc_eng_st_s &engine_state,
		tv3_gd_att_sp_s &attitude_setpoint,
		tv3_gd_thr_sp_s &thrust_setpoint)
{
	capture_launch_reference(attitude);

	if (!_launch_reference_valid) {
		return;
	}

	if (!guidance_position_valid(now, local_position, groundtruth_position)) {
		attitude_setpoint.valid = true;
		set_attitude_setpoint_q(attitude_setpoint, _launch_reference_q);
		thrust_setpoint.valid = true;
		thrust_setpoint.axial_n = active_chamber_thrust_n(engine_state);
		return;
	}

	const vehicle_local_position_s &position = guidance_position(now, local_position, groundtruth_position);

	capture_origin_if_needed(position);

	const Vector3f target{
		_origin_ned[0] + _waypoint_config.wp_n_m,
		_origin_ned[1] + _waypoint_config.wp_e_m,
		_origin_ned[2] + _waypoint_config.wp_d_m,
	};
	const Vector3f current{position.x, position.y, position.z};
	const Vector3f error = target - current;
	const float distance = error.norm();
	const float acceptance = math::max(_waypoint_config.acceptance_m, 0.5f);
	const float max_tilt_rad = math::radians(math::max(_waypoint_config.max_tilt_deg, 1.f));

	float guided_q[4]{};
	set_attitude_setpoint_q(attitude_setpoint, _launch_reference_q);
	copy_attitude_setpoint_q(guided_q, attitude_setpoint);

	if (!_waypoint_reached && distance > acceptance) {
		set_attitude_toward_direction(_launch_reference_q, error, max_tilt_rad, guided_q);
	} else {
		_waypoint_reached = true;
	}

	attitude_setpoint.valid = true;
	set_attitude_setpoint_q(attitude_setpoint, guided_q);
	thrust_setpoint.valid = true;
	thrust_setpoint.axial_n = active_chamber_thrust_n(engine_state);
}

void GuidanceFsm::step(hrt_abstime now,
		       const vehicle_attitude_s &attitude,
		       const vehicle_local_position_s &local_position,
		       const vehicle_local_position_s &groundtruth_position,
		       const tv3_lc_eng_st_s &engine_state,
		       tv3_gd_att_sp_s &attitude_setpoint,
		       tv3_gd_thr_sp_s &thrust_setpoint)
{
	attitude_setpoint.timestamp = now;
	thrust_setpoint.timestamp = now;

	if (_fsm.in_mode(GuidanceMode::Up)) {
		if (guidance_position_valid(now, local_position, groundtruth_position)) {
			capture_origin_if_needed(guidance_position(now, local_position, groundtruth_position));
		}

		step_up(now, attitude, engine_state, attitude_setpoint, thrust_setpoint);
		return;
	}

	if (_fsm.in_mode(GuidanceMode::WaypointFlyThrough)) {
		step_waypoint_fly_through(now, attitude, local_position, groundtruth_position, engine_state,
					  attitude_setpoint, thrust_setpoint);
	}
}

} // namespace tv3
