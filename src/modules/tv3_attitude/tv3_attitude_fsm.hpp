#pragma once

#include "../../lib/tv3_module_fsm.hpp"
#include "../../lib/tv3_module_modes.hpp"

#include "tv3_attitude_controller.hpp"

#include <uORB/topics/tv3_gd_att_sp.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <drivers/drv_hrt.h>

namespace tv3
{

class AttitudeFsm {
public:
	void apply_module_mode(const tv3_sm_modes_s &modes);

	void set_controller_config(const AttitudeControllerConfig &config) { _controller.set_config(config); }

	void step(hrt_abstime now,
		  float dt,
		  const vehicle_attitude_s &attitude,
		  const vehicle_angular_velocity_s &angular_velocity,
		  const tv3_gd_att_sp_s &attitude_setpoint,
		  vehicle_torque_setpoint_s &torque_setpoint);

private:
	static AttitudeMode mode_for_region(AttitudeControllerRegion region);

	AttitudeController _controller{};
	Tv3ModuleFsm<AttitudeMode> _fsm{AttitudeMode::Off};
	bool _enabled{false};
};

} // namespace tv3
