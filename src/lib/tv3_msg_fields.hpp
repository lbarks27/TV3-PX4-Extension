#pragma once

#include <uORB/topics/tv3_mix_alloc_st.h>
#include <uORB/topics/tv3_gd_att_sp.h>
#include <uORB/topics/tv3_mix_eng_cmd.h>
#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_sih_wrench.h>

#include <matrix/matrix/math.hpp>

#include <cmath>
#include <cstring>

namespace tv3
{

constexpr int kEngineSlots = 4;

inline float &engine_slot_float(float &v0, float &v1, float &v2, float &v3, int index)
{
	switch (index) {
	case 0: return v0;
	case 1: return v1;
	case 2: return v2;
	case 3: return v3;
	default: return v0;
	}
}

inline float engine_slot_float(const float &v0, const float &v1, const float &v2, const float &v3, int index)
{
	switch (index) {
	case 0: return v0;
	case 1: return v1;
	case 2: return v2;
	case 3: return v3;
	default: return NAN;
	}
}

inline uint8_t &engine_slot_uint8(uint8_t &v0, uint8_t &v1, uint8_t &v2, uint8_t &v3, int index)
{
	switch (index) {
	case 0: return v0;
	case 1: return v1;
	case 2: return v2;
	case 3: return v3;
	default: return v0;
	}
}

inline uint16_t &engine_slot_uint16(uint16_t &v0, uint16_t &v1, uint16_t &v2, uint16_t &v3, int index)
{
	switch (index) {
	case 0: return v0;
	case 1: return v1;
	case 2: return v2;
	case 3: return v3;
	default: return v0;
	}
}

inline float measured_thrust_n(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.measured_thrust_n_0, state.measured_thrust_n_1,
				 state.measured_thrust_n_2, state.measured_thrust_n_3, index);
}

inline float measured_thrust_n(const tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.measured_thrust_n_0, state.measured_thrust_n_1,
				 state.measured_thrust_n_2, state.measured_thrust_n_3, index);
}

inline float &measured_thrust_n_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.measured_thrust_n_0, state.measured_thrust_n_1,
				 state.measured_thrust_n_2, state.measured_thrust_n_3, index);
}

inline float &filtered_thrust_n_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.filtered_thrust_n_0, state.filtered_thrust_n_1,
				 state.filtered_thrust_n_2, state.filtered_thrust_n_3, index);
}

inline float filtered_thrust_n(const tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.filtered_thrust_n_0, state.filtered_thrust_n_1,
				 state.filtered_thrust_n_2, state.filtered_thrust_n_3, index);
}

inline float expected_thrust_n(const tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.expected_thrust_n_0, state.expected_thrust_n_1,
				 state.expected_thrust_n_2, state.expected_thrust_n_3, index);
}

inline float &expected_thrust_n_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.expected_thrust_n_0, state.expected_thrust_n_1,
				 state.expected_thrust_n_2, state.expected_thrust_n_3, index);
}

inline float &expected_motor_mass_kg_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.expected_motor_mass_kg_0, state.expected_motor_mass_kg_1,
				 state.expected_motor_mass_kg_2, state.expected_motor_mass_kg_3, index);
}

inline float expected_motor_mass_kg(const tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.expected_motor_mass_kg_0, state.expected_motor_mass_kg_1,
				 state.expected_motor_mass_kg_2, state.expected_motor_mass_kg_3, index);
}

inline float &burn_fraction_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.burn_fraction_0, state.burn_fraction_1,
				 state.burn_fraction_2, state.burn_fraction_3, index);
}

inline float &remaining_impulse_ns_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.remaining_impulse_ns_0, state.remaining_impulse_ns_1,
				 state.remaining_impulse_ns_2, state.remaining_impulse_ns_3, index);
}

inline uint8_t &fault_flags_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_uint8(state.fault_flags_0, state.fault_flags_1,
				 state.fault_flags_2, state.fault_flags_3, index);
}

inline uint16_t &selected_motor_index_ref(tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_uint16(state.selected_motor_index_0, state.selected_motor_index_1,
				  state.selected_motor_index_2, state.selected_motor_index_3, index);
}

inline uint16_t selected_motor_index(const tv3_lc_eng_st_s &state, int index)
{
	switch (index) {
	case 0: return state.selected_motor_index_0;
	case 1: return state.selected_motor_index_1;
	case 2: return state.selected_motor_index_2;
	case 3: return state.selected_motor_index_3;
	default: return 0;
	}
}

inline uint16_t &selected_motor_index_ref(tv3_mix_eng_cmd_s &command, int index)
{
	return engine_slot_uint16(command.selected_motor_index_0, command.selected_motor_index_1,
				  command.selected_motor_index_2, command.selected_motor_index_3, index);
}

inline float &commanded_pitch_deg_ref(tv3_mix_eng_cmd_s &command, int index)
{
	return engine_slot_float(command.commanded_pitch_deg_0, command.commanded_pitch_deg_1,
				 command.commanded_pitch_deg_2, command.commanded_pitch_deg_3, index);
}

inline float commanded_pitch_deg(const tv3_mix_eng_cmd_s &command, int index)
{
	return engine_slot_float(command.commanded_pitch_deg_0, command.commanded_pitch_deg_1,
				 command.commanded_pitch_deg_2, command.commanded_pitch_deg_3, index);
}

inline float &commanded_yaw_deg_ref(tv3_mix_eng_cmd_s &command, int index)
{
	return engine_slot_float(command.commanded_yaw_deg_0, command.commanded_yaw_deg_1,
				 command.commanded_yaw_deg_2, command.commanded_yaw_deg_3, index);
}

inline float commanded_yaw_deg(const tv3_mix_eng_cmd_s &command, int index)
{
	return engine_slot_float(command.commanded_yaw_deg_0, command.commanded_yaw_deg_1,
				 command.commanded_yaw_deg_2, command.commanded_yaw_deg_3, index);
}

inline float &commanded_splay_deg_ref(tv3_mix_eng_cmd_s &command, int index)
{
	return engine_slot_float(command.commanded_splay_deg_0, command.commanded_splay_deg_1,
				 command.commanded_splay_deg_2, command.commanded_splay_deg_3, index);
}

inline void set_attitude_setpoint_q(tv3_gd_att_sp_s &setpoint, const float q[4])
{
	setpoint.q_w = q[0];
	setpoint.q_x = q[1];
	setpoint.q_y = q[2];
	setpoint.q_z = q[3];
}

inline void copy_attitude_setpoint_q(float q[4], const tv3_gd_att_sp_s &setpoint)
{
	q[0] = setpoint.q_w;
	q[1] = setpoint.q_x;
	q[2] = setpoint.q_y;
	q[3] = setpoint.q_z;
}

inline matrix::Quatf attitude_setpoint_quat(const tv3_gd_att_sp_s &setpoint)
{
	return matrix::Quatf{setpoint.q_w, setpoint.q_x, setpoint.q_y, setpoint.q_z};
}

inline void set_body_vector3(float &x, float &y, float &z, const matrix::Vector3f &v)
{
	x = v(0);
	y = v(1);
	z = v(2);
}

inline void set_plant_wrench(tv3_sih_wrench_s &wrench, const matrix::Vector3f &body_force_n,
			     const matrix::Vector3f &engine_torque_nm, const matrix::Vector3f &net_torque_nm)
{
	set_body_vector3(wrench.body_force_n_x, wrench.body_force_n_y, wrench.body_force_n_z, body_force_n);
	set_body_vector3(wrench.engine_torque_nm_x, wrench.engine_torque_nm_y, wrench.engine_torque_nm_z, engine_torque_nm);
	set_body_vector3(wrench.net_torque_nm_x, wrench.net_torque_nm_y, wrench.net_torque_nm_z, net_torque_nm);
}

inline void set_demanded_torque_nm(tv3_mix_alloc_st_s &status, const matrix::Vector3f &torque_nm)
{
	status.demanded_torque_nm_roll = torque_nm(0);
	status.demanded_torque_nm_pitch = torque_nm(1);
	status.demanded_torque_nm_yaw = torque_nm(2);
}

inline void set_demanded_body_force_n(tv3_mix_alloc_st_s &status, const matrix::Vector3f &force_n)
{
	set_body_vector3(status.demanded_body_force_n_x, status.demanded_body_force_n_y,
		       status.demanded_body_force_n_z, force_n);
}

} // namespace tv3
