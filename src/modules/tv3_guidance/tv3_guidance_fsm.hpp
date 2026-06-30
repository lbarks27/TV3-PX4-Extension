#pragma once

#include "../../lib/tv3_module_fsm.hpp"
#include "../../lib/tv3_module_modes.hpp"

#include <uORB/topics/tv3_gd_att_sp.h>
#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/tv3_gd_thr_sp.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

#include <matrix/matrix/math.hpp>

#include <drivers/drv_hrt.h>

namespace tv3
{

struct GuidanceWaypointConfig {
	float wp_n_m{0.f};
	float wp_e_m{0.f};
	float wp_d_m{-20.f};
	float pos_p{0.15f};
	float acceptance_m{5.f};
	float max_velocity_m_s{12.f};
	float max_tilt_deg{35.f};
	int32_t sim_groundtruth_fallback{1};
};

class GuidanceFsm {
public:
	void apply_module_mode(const tv3_sm_modes_s &modes);

	void set_waypoint_config(const GuidanceWaypointConfig &config) { _waypoint_config = config; }

	bool in_mode(GuidanceMode mode) const { return _fsm.in_mode(mode); }

	void step(hrt_abstime now,
		  const vehicle_attitude_s &attitude,
		  const vehicle_local_position_s &local_position,
		  const vehicle_local_position_s &groundtruth_position,
		  const tv3_lc_eng_st_s &engine_state,
		  tv3_gd_att_sp_s &attitude_setpoint,
		  tv3_gd_thr_sp_s &thrust_setpoint);

private:
	static float active_chamber_thrust_n(const tv3_lc_eng_st_s &engine_state);

	bool position_valid(const vehicle_local_position_s &position, hrt_abstime now) const;

	bool guidance_position_valid(hrt_abstime now,
				     const vehicle_local_position_s &local_position,
				     const vehicle_local_position_s &groundtruth_position) const;

	const vehicle_local_position_s &guidance_position(hrt_abstime now,
			const vehicle_local_position_s &local_position,
			const vehicle_local_position_s &groundtruth_position) const;

	void step_up(hrt_abstime now,
		     const vehicle_attitude_s &attitude,
		     const tv3_lc_eng_st_s &engine_state,
		     tv3_gd_att_sp_s &attitude_setpoint,
		     tv3_gd_thr_sp_s &thrust_setpoint);

	void step_waypoint_fly_through(hrt_abstime now,
				       const vehicle_attitude_s &attitude,
				       const vehicle_local_position_s &local_position,
				       const vehicle_local_position_s &groundtruth_position,
				       const tv3_lc_eng_st_s &engine_state,
				       tv3_gd_att_sp_s &attitude_setpoint,
				       tv3_gd_thr_sp_s &thrust_setpoint);

	void capture_launch_reference(const vehicle_attitude_s &attitude);

	void capture_origin_if_needed(const vehicle_local_position_s &position);

	void set_attitude_toward_direction(const float launch_q[4], const matrix::Vector3f &direction_ned,
					   float max_tilt_rad, float out_q[4]) const;

	Tv3ModuleFsm<GuidanceMode> _fsm{GuidanceMode::Off};
	GuidanceWaypointConfig _waypoint_config{};

	bool _launch_reference_valid{false};
	float _launch_reference_q[4]{1.f, 0.f, 0.f, 0.f};
	bool _origin_valid{false};
	float _origin_ned[3]{0.f, 0.f, 0.f};
	bool _waypoint_reached{false};
};

} // namespace tv3
