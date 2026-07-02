#pragma once

#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_lc_thrust.h>

#include <cmath>

namespace tv3
{

constexpr int kEngineSlots = tv3_lc_eng_st_s::MAX_ENGINES;
constexpr int kLoadCellsPerEngine = tv3_lc_thrust_s::MAX_CELLS;

inline float engine_slot_float(const float &v0, const float &v1, const float &v2, int index)
{
	switch (index) {
	case 0: return v0;
	case 1: return v1;
	case 2: return v2;
	default: return NAN;
	}
}

inline float cell_slot_float(const float &v0, const float &v1, const float &v2, int index)
{
	return engine_slot_float(v0, v1, v2, index);
}

inline float measured_thrust_n(const tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.measured_thrust_n_0, state.measured_thrust_n_1,
				 state.measured_thrust_n_2, index);
}

inline float filtered_thrust_n(const tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.filtered_thrust_n_0, state.filtered_thrust_n_1,
				 state.filtered_thrust_n_2, index);
}

inline float expected_thrust_n(const tv3_lc_eng_st_s &state, int index)
{
	return engine_slot_float(state.expected_thrust_n_0, state.expected_thrust_n_1,
				 state.expected_thrust_n_2, index);
}

inline float cell_measured_thrust_n(const tv3_lc_thrust_s &thrust, int index)
{
	return cell_slot_float(thrust.cell_measured_thrust_n_0, thrust.cell_measured_thrust_n_1,
			       thrust.cell_measured_thrust_n_2, index);
}

inline float cell_filtered_thrust_n(const tv3_lc_thrust_s &thrust, int index)
{
	return cell_slot_float(thrust.cell_filtered_thrust_n_0, thrust.cell_filtered_thrust_n_1,
			       thrust.cell_filtered_thrust_n_2, index);
}

} // namespace tv3
