#pragma once

#include "../../lib/tv3_module_modes.hpp"

#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/tv3_sm_status.h>
#include <uORB/topics/tv3_lc_thrust.h>
#include <uORB/topics/vehicle_status.h>

#include <drivers/drv_hrt.h>

namespace tv3
{

constexpr int kStateMachineMaxEngines = 4;

struct StateMachineConfig {
	int32_t enabled{1};
	float launch_threshold_n{10.f};
	int32_t ignition_pulse_ms{300};
	int32_t ignition_timeout_ms{2000};
	int32_t minimum_burn_ms{150};
	int32_t maximum_burn_ms{6000};
	float burnout_threshold_n{4.f};
	int32_t burnout_dwell_ms{100};
	int32_t abort_on_gcs_loss{0};
	int32_t engine_count{1};
	int32_t ignition_sequence[kStateMachineMaxEngines]{0, 1, 2, 3};
	int32_t ignition_dwell_ms{0};
	int32_t phase_count{0};
	ControlPhaseConfig phases[kMaxControlPhases]{};
};

struct StateMachineInputs {
	vehicle_status_s vehicle_status{};
	tv3_lc_thrust_s thrust{};
	tv3_lc_eng_st_s engine_state{};
};

class VehicleStateMachine {
public:
	void set_config(const StateMachineConfig &config) { _config = config; }

	void request_launch() { _launch_requested = true; }

	void request_abort() { _abort_requested = true; }

	void request_reset() { _reset_requested = true; }

	void update(hrt_abstime now, const StateMachineInputs &inputs);

	void build_module_modes(hrt_abstime now, tv3_sm_modes_s &modes) const;

	void build_status(hrt_abstime now, const StateMachineInputs &inputs, tv3_sm_status_s &status) const;

	uint8_t mode() const { return _mode; }

	uint32_t fault_reason() const { return _fault_reason; }

	bool mode_or_fault_changed(uint8_t &last_mode, uint32_t &last_fault) const;

	const char *mode_name() const;

	const char *fault_name() const;

private:
	void reset_state();

	void set_fault(uint32_t fault_reason);

	void reset_engine_sequence();

	void start_engine_sequence(hrt_abstime now);

	bool active_sequence_engine_confirmed(const StateMachineInputs &inputs) const;

	bool all_sequence_engines_confirmed(const StateMachineInputs &inputs) const;

	void update_engine_sequence(hrt_abstime now, const StateMachineInputs &inputs);

	static uint8_t engine_bit(int engine_index);

	StateMachineConfig _config{};

	uint8_t _mode{tv3_sm_status_s::MODE_DISARMED_SAFE};
	uint32_t _fault_reason{tv3_sm_status_s::FAULT_NONE};
	bool _launch_requested{false};
	bool _abort_requested{false};
	bool _reset_requested{false};
	bool _ignition_on{false};
	bool _sequence_complete{false};
	uint8_t _ignition_mask{0};
	int _active_sequence_slot{0};
	hrt_abstime _ignition_timestamp{0};
	hrt_abstime _boost_timestamp{0};
	hrt_abstime _burnout_low_timestamp{0};
	hrt_abstime _current_engine_confirm_timestamp{0};
	hrt_abstime _last_update{0};
};

} // namespace tv3
