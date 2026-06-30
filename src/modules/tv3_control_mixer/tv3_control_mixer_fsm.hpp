#pragma once

#include <drivers/drv_hrt.h>

#include "tv3_control_mixer_core.hpp"

#include "../../lib/tv3_module_fsm.hpp"
#include "../../lib/tv3_module_modes.hpp"

#include <uORB/topics/tv3_mix_alloc_st.h>
#include <uORB/topics/tv3_mix_eng_cmd.h>
#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <drivers/drv_hrt.h>

namespace tv3
{

struct ControlMixerRunInput {
	tv3_sm_modes_s module_modes{};
	tv3_lc_eng_st_s engine_state{};
	vehicle_torque_setpoint_s torque_sp{};
	float engine_thrust_n[kControlMixerMaxEngines]{};
	float torque_roll_max{8.f};
	float torque_pitch_max{16.f};
	float torque_yaw_max{16.f};
	uint8_t selected_motor_index[kControlMixerMaxEngines]{};
};

struct ControlMixerRunOutput {
	tv3_mix_eng_cmd_s engine_command{};
	tv3_mix_alloc_st_s allocator_status{};
	bool publish_allocator_status{false};
};

class ControlMixerFsm {
public:
	void apply_module_mode(const tv3_sm_modes_s &modes);

	ControlMixerRunOutput run(const ControlMixerCore &core, const ControlMixerRunInput &input, hrt_abstime now);

private:
	Tv3ModuleFsm<MixerMode> _fsm{MixerMode::Off};
	float _prev_primary_rad[kControlMixerMaxEngines]{};
	float _prev_yaw_rad[kControlMixerMaxEngines]{};
	int _prev_mask{0};
	bool _prev_warm_valid{false};

	static float engine_chamber_thrust_n(const float engine_thrust_n[kControlMixerMaxEngines], int index);
};

} // namespace tv3
