#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_lc_eng_st.h>
#include <uORB/topics/tv3_lc_ch.h>
#include <uORB/topics/tv3_sm_modes.h>
#include <uORB/topics/tv3_sm_status.h>
#include <uORB/topics/tv3_lc_thrust.h>

#include <cstdio>
#include <cstdlib>

#include "lib/tv3_msg_fields.hpp"
#include "lib/tv3_motor_curve.hpp"
#include "tv3_load_cell_fsm.hpp"

using namespace time_literals;
using tv3::MotorCurveCatalog;
using tv3::MotorCurveSample;
using tv3::burn_fraction_ref;
using tv3::expected_motor_mass_kg_ref;
using tv3::expected_thrust_n_ref;
using tv3::fault_flags_ref;
using tv3::filtered_thrust_n_ref;
using tv3::measured_thrust_n_ref;
using tv3::remaining_impulse_ns_ref;
using tv3::selected_motor_index_ref;
using tv3::LoadCellFsm;
using tv3::LoadCellMode;

class TV3LoadCell : public ModuleBase<TV3LoadCell>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3LoadCell() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3LoadCell *instance = new TV3LoadCell();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION("ADC-backed load cell with thrust and optional debug telemetry.");
		PRINT_MODULE_USAGE_NAME("tv3_lc_ch", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(10_ms);
		return true;
	}

private:
	static constexpr int32_t SOURCE_ADC = 0;
	static constexpr int32_t SOURCE_REFERENCE = 1;
	static constexpr int kMaxEngines = 4;
	static constexpr float kSlowAlpha = 0.08f;
	static constexpr float kGravityMps2 = 9.80665f;

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		if (_parameter_update_sub.updated()) {
			parameter_update_s update{};
			_parameter_update_sub.copy(&update);
			update_parameters();
		}

		_module_modes_sub.copy(&_module_modes);
		_fsm.apply_module_mode(_module_modes);

		if (_fsm.in_mode(LoadCellMode::Off)) {
			return;
		}

		if (_source == SOURCE_REFERENCE && !_motors_loaded) {
			load_motor_catalog();
		}

		uint8_t fault_flags = tv3_lc_thrust_s::FAULT_NONE;

		if (_source == SOURCE_REFERENCE) {
			update_reference_thrust();
			_last_sample_timestamp = hrt_absolute_time();
			_last_raw = static_cast<int32_t>(lrintf(_measured_thrust_n * 100.f));
			_last_voltage_v = _measured_thrust_n;
		} else {
			adc_report_s adc{};
			bool found = false;

			while (_adc_report_sub.update(&adc)) {
				for (size_t i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); ++i) {
					if (adc.channel_id[i] == _channel) {
						_last_sample_timestamp = adc.timestamp != 0 ? adc.timestamp : hrt_absolute_time();
						_last_raw = adc.raw_data[i];
						_last_voltage_v = adc.resolution > 0 ?
								  static_cast<float>(_last_raw) * adc.v_ref / static_cast<float>(adc.resolution) : 0.f;
						const float thrust_n = math::max((static_cast<float>(_last_raw) - _tare) * _scale_n_per_count, 0.f);
						_measured_thrust_n = _alpha_fast * thrust_n + (1.f - _alpha_fast) * _measured_thrust_n;
						found = true;
						break;
					}
				}
			}

			if (!found && _last_sample_timestamp == 0) {
				fault_flags |= tv3_lc_thrust_s::FAULT_CHANNEL_MISSING;
			}
		}

		if (fabsf(_scale_n_per_count) < 1e-9f && _source == SOURCE_ADC) {
			fault_flags |= tv3_lc_thrust_s::FAULT_BAD_SCALE;
		}

		if (!_motors_loaded && _source == SOURCE_REFERENCE && _reference_thrust_n <= 0.f) {
			fault_flags |= tv3_lc_thrust_s::FAULT_NO_REFERENCE;
		} else if (_expected_thrust_n <= 0.f && _source != SOURCE_REFERENCE) {
			fault_flags |= tv3_lc_thrust_s::FAULT_NO_REFERENCE;
		}

		if (_last_sample_timestamp == 0) {
			fault_flags |= tv3_lc_thrust_s::FAULT_STALE;
		} else if (hrt_elapsed_time(&_last_sample_timestamp) > static_cast<hrt_abstime>(_timeout_ms) * 1000ULL) {
			fault_flags |= tv3_lc_thrust_s::FAULT_STALE;
		}

		_filtered_thrust_n = kSlowAlpha * _measured_thrust_n + (1.f - kSlowAlpha) * _filtered_thrust_n;
		update_engine_state(fault_flags);

		const bool sensor_valid = (fault_flags & (tv3_lc_thrust_s::FAULT_STALE | tv3_lc_thrust_s::FAULT_CHANNEL_MISSING
							 | tv3_lc_thrust_s::FAULT_BAD_SCALE)) == 0;
		const bool ignition_confirmed = _engine_confirmed_mask != 0 || _measured_thrust_n >= _ignition_threshold_n;

		tv3_lc_thrust_s out{};
		out.timestamp = hrt_absolute_time();
		out.timestamp_sample = _last_sample_timestamp;
		out.measured_thrust_n = _measured_thrust_n;
		out.filtered_thrust_n = _filtered_thrust_n;
		out.expected_thrust_n = _expected_thrust_n;
		out.expected_motor_mass_kg = _expected_motor_mass_kg;
		out.expected_vehicle_mass_kg = _expected_vehicle_mass_kg;
		out.total_impulse_ns = _total_impulse_ns;
		out.burn_fraction = _burn_fraction;
		out.valid = sensor_valid;
		out.ignition_confirmed = ignition_confirmed;
		out.fault_flags = fault_flags;
		out.selected_motor_index = static_cast<uint16_t>(_engine_motor_index[0]);
		const char *motor_id = (_motors_loaded && _motors.motor_id(0) != nullptr && _motors.motor_id(0)[0] != '\0')
				       ? _motors.motor_id(0) : "static";
		snprintf(out.selected_motor_id, sizeof(out.selected_motor_id), "%s", motor_id);
		_thrust_pub.publish(out);

		tv3_lc_ch_s compat{};
		compat.timestamp = out.timestamp;
		compat.timestamp_sample = out.timestamp_sample;
		compat.channel = static_cast<int8_t>(_channel);
		compat.raw_count = _last_raw;
		compat.voltage_v = _last_voltage_v;
		compat.thrust_n = out.measured_thrust_n;
		compat.valid = out.valid;
		_load_cell_pub.publish(compat);

		if (_debug_enabled > 0) {
			publish_debug(out.timestamp);
		}
	}

	void publish_debug(hrt_abstime now)
	{
		debug_key_value_s key{};
		key.timestamp = now;
		snprintf(key.key, sizeof(key.key), "lc_raw");
		key.value = static_cast<float>(_last_raw);
		_debug_key_pub.publish(key);

		debug_vect_s vect{};
		vect.timestamp = now;
		snprintf(vect.name, sizeof(vect.name), "lc_thrust");
		vect.x = _measured_thrust_n;
		vect.y = _filtered_thrust_n;
		vect.z = _expected_thrust_n;
		_debug_vect_pub.publish(vect);
	}

	void update_reference_thrust()
	{
		_state_machine_status_sub.copy(&_state_machine_status);
		update_engine_burn_state();

		const int engine_count = math::constrain(
						 _state_machine_status.engine_count > 0 ? _state_machine_status.engine_count : _engine_count,
						 1, kMaxEngines);

		float measured_sum = 0.f;
		float expected_sum = 0.f;
		float motor_mass_sum = 0.f;
		float impulse_sum = 0.f;
		float impulse_used_sum = 0.f;

		for (int i = 0; i < engine_count; ++i) {
			float thrust_n = 0.f;
			float motor_mass_kg = _expected_motor_mass_kg;
			float burn_fraction = 0.f;
			float impulse_ns = 0.f;

			if (_motors_loaded && _engine_burn_active[i] && _engine_burn_start[i] != 0) {
				float burn_time_s = static_cast<float>(hrt_absolute_time() - _engine_burn_start[i]) * 1e-6f;

				if (_state_machine_status.mode == tv3_sm_status_s::MODE_BOOST && _sim_burn_time_scale > 1.01f) {
					burn_time_s /= _sim_burn_time_scale;
				}

				MotorCurveSample sample{};
				if (_motors.sample(i, burn_time_s, sample)) {
					thrust_n = sample.thrust_n;
					motor_mass_kg = sample.motor_mass_kg;
					burn_fraction = sample.burn_fraction;
					impulse_ns = sample.cumulative_impulse_ns;
				}
			} else if (_motors_loaded) {
				motor_mass_kg = _motors.specs(i).loaded_mass_kg;
			}

			_engine_measured_thrust_n[i] = thrust_n;
			_engine_expected_thrust_n[i] = thrust_n;
			_engine_motor_mass_kg[i] = motor_mass_kg;
			_engine_burn_fraction[i] = burn_fraction;
			_engine_remaining_impulse_ns[i] = math::max(_motors.specs(i).total_impulse_ns - impulse_ns, 0.f);
			measured_sum += thrust_n;
			expected_sum += thrust_n;
			motor_mass_sum += motor_mass_kg;
			impulse_sum += _motors.specs(i).total_impulse_ns;
			impulse_used_sum += impulse_ns;
		}

		_measured_thrust_n = measured_sum;
		_expected_thrust_n = expected_sum;

		if (!_motors_loaded) {
			_expected_thrust_n = math::max(_reference_thrust_n, 0.f) * static_cast<float>(engine_count);
		}
		_expected_motor_mass_kg = engine_count > 0 ? motor_mass_sum / static_cast<float>(engine_count) : 0.f;
		_total_impulse_ns = impulse_sum;
		_burn_fraction = impulse_sum > 1e-3f ? math::constrain(impulse_used_sum / impulse_sum, 0.f, 1.f) : 0.f;
	}

	void update_engine_burn_state()
	{
		const uint8_t ignition_mask = _state_machine_status.ignition_mask;
		const bool should_burn = _state_machine_status.mode == tv3_sm_status_s::MODE_IGNITION_PENDING
					 || _state_machine_status.mode == tv3_sm_status_s::MODE_BOOST;

		for (int i = 0; i < kMaxEngines; ++i) {
			const bool ignited = (ignition_mask & static_cast<uint8_t>(1u << i)) != 0;

			if (should_burn && ignited && !_engine_burn_active[i]) {
				_engine_burn_active[i] = true;
				_engine_burn_start[i] = hrt_absolute_time();
			}

			if (!should_burn) {
				_engine_burn_active[i] = false;
				_engine_burn_start[i] = 0;
			}
		}
	}

	bool load_motor_catalog()
	{
		const char *motor_root = getenv("TV3_MOTOR_ROOT");
		_motors_loaded = false;

		if (!_motors.load(motor_root != nullptr ? motor_root : "/fs/microsd/tv3/motors")) {
			return false;
		}

		bool all_loaded = true;

		for (int i = 0; i < _engine_count; ++i) {
			int32_t motor_index = _engine_motor_index[i];

			if (_engine_count <= 1) {
				motor_index = _motor_index;
			}

			all_loaded = _motors.load_engine_slot(i, motor_index) && all_loaded;
		}

		_motors_loaded = all_loaded;
		return all_loaded;
	}

	void update_engine_state(uint8_t aggregate_fault_flags)
	{
		_state_machine_status_sub.copy(&_state_machine_status);

		const int engine_count = math::constrain(
						 _state_machine_status.engine_count > 0 ? _state_machine_status.engine_count : _engine_count,
						 1, kMaxEngines);
		float expected_sum = _expected_thrust_n * engine_count;

		_engine_confirmed_mask = 0;
		tv3_lc_eng_st_s state{};
		state.timestamp = hrt_absolute_time();
		state.timestamp_sample = _last_sample_timestamp;
		state.engine_count = static_cast<uint8_t>(engine_count);
		state.ignition_mask = _state_machine_status.ignition_mask;
		state.active_mask = state.ignition_mask;
		state.active_ignition_index = _state_machine_status.active_ignition_index;
		state.sequence_active = _state_machine_status.sequence_active;
		state.sequence_complete = _state_machine_status.sequence_complete;

		for (int i = 0; i < engine_count; ++i) {
			const float expected = _expected_thrust_n;

			if (_source == SOURCE_REFERENCE) {
				_engine_measured_thrust_n[i] = expected;
			} else if (expected_sum > 1e-3f) {
				_engine_measured_thrust_n[i] = _measured_thrust_n * expected / expected_sum;
			} else {
				_engine_measured_thrust_n[i] = i == 0 ? _measured_thrust_n : 0.f;
			}

			_engine_filtered_thrust_n[i] = kSlowAlpha * _engine_measured_thrust_n[i]
						       + (1.f - kSlowAlpha) * _engine_filtered_thrust_n[i];
			measured_thrust_n_ref(state, i) = _engine_measured_thrust_n[i];
			filtered_thrust_n_ref(state, i) = _engine_filtered_thrust_n[i];
			expected_thrust_n_ref(state, i) = expected;
			expected_motor_mass_kg_ref(state, i) = _expected_motor_mass_kg;
			burn_fraction_ref(state, i) = 0.f;
			remaining_impulse_ns_ref(state, i) = _total_impulse_ns;
			fault_flags_ref(state, i) = aggregate_fault_flags;
			selected_motor_index_ref(state, i) = 0;

			if (_engine_measured_thrust_n[i] >= _ignition_threshold_n) {
				_engine_confirmed_mask |= static_cast<uint8_t>(1u << i);
			}
		}

		state.confirmed_mask = _engine_confirmed_mask;
		state.fault_mask = aggregate_fault_flags == tv3_lc_thrust_s::FAULT_NONE ? 0 : ((1u << engine_count) - 1u);

		if (!state.sequence_complete) {
			state.sequence_complete = state.ignition_mask != 0
						  && (state.confirmed_mask & state.ignition_mask) == state.ignition_mask;
		}

		state.all_ignited = state.sequence_complete;
		_engine_state_pub.publish(state);
	}

	void update_parameters()
	{
		const int32_t previous_adc_instance = _adc_instance;
		param_t p = param_find("RK_LC_SRC");

		if (p != PARAM_INVALID) {
			param_get(p, &_source);
		}

		p = param_find("RK_LC_ADC_INST");

		if (p != PARAM_INVALID) {
			param_get(p, &_adc_instance);
			_adc_instance = math::constrain(_adc_instance, static_cast<int32_t>(0), static_cast<int32_t>(7));
		}

		p = param_find("RK_LC_CH");

		if (p != PARAM_INVALID) {
			param_get(p, &_channel);
		}

		p = param_find("RK_LC_DEBUG");

		if (p != PARAM_INVALID) {
			param_get(p, &_debug_enabled);
		}

		p = param_find("RK_ENG_COUNT");

		if (p != PARAM_INVALID) {
			param_get(p, &_engine_count);
			_engine_count = math::constrain(_engine_count, static_cast<int32_t>(1), static_cast<int32_t>(kMaxEngines));
		}

		p = param_find("RK_MOT_IDX");

		if (p != PARAM_INVALID) {
			param_get(p, &_motor_index);
		}

		for (int i = 0; i < kMaxEngines; ++i) {
			char name[16];
			snprintf(name, sizeof(name), "RK_ENG%d_MOT", i);
			p = param_find(name);

			if (p != PARAM_INVALID) {
				param_get(p, &_engine_motor_index[i]);
			}
		}

		p = param_find("RK_SIM_BURN_SCL");

		if (p != PARAM_INVALID) {
			param_get(p, &_sim_burn_time_scale);
			_sim_burn_time_scale = math::max(_sim_burn_time_scale, 1.f);
		}

		p = param_find("RK_LC_TARE");

		if (p != PARAM_INVALID) {
			param_get(p, &_tare);
		}

		p = param_find("RK_LC_ALPHA");

		if (p != PARAM_INVALID) {
			param_get(p, &_alpha_fast);
			_alpha_fast = math::constrain(_alpha_fast, 0.01f, 1.f);
		}

		p = param_find("RK_LC_TO_MS");

		if (p != PARAM_INVALID) {
			param_get(p, &_timeout_ms);
			_timeout_ms = math::max(_timeout_ms, static_cast<int32_t>(10));
		}

		p = param_find("RK_LAUNCH_THR_N");

		if (p != PARAM_INVALID) {
			param_get(p, &_ignition_threshold_n);
		}

		p = param_find("RK_LC_EXP_THR_N");

		if (p != PARAM_INVALID) {
			param_get(p, &_reference_thrust_n);
		}

		p = param_find("RK_LC_EXP_MASS");

		if (p != PARAM_INVALID) {
			param_get(p, &_expected_motor_mass_kg);
		}

		p = param_find("RK_LC_EXP_VEH");

		if (p != PARAM_INVALID) {
			param_get(p, &_expected_vehicle_mass_kg);
		}

		p = param_find("RK_LC_TOT_IMP");

		if (p != PARAM_INVALID) {
			param_get(p, &_total_impulse_ns);
		}

		float scale_n = 0.f;
		float kg_per_count = 0.f;
		p = param_find("RK_LC_SCALE");

		if (p != PARAM_INVALID) {
			param_get(p, &scale_n);
		}

		p = param_find("RK_LC_KG_SC");

		if (p != PARAM_INVALID) {
			param_get(p, &kg_per_count);
		}

		_scale_n_per_count = fabsf(scale_n) >= 1e-9f ? scale_n : kg_per_count * kGravityMps2;

		if (_adc_instance != previous_adc_instance) {
			_adc_report_sub.ChangeInstance(static_cast<uint8_t>(_adc_instance));
		}
	}

	LoadCellFsm _fsm{};

	int32_t _source{SOURCE_ADC};
	int32_t _adc_instance{1};
	int32_t _channel{0};
	int32_t _debug_enabled{0};
	int32_t _engine_count{1};
	float _tare{0.f};
	float _scale_n_per_count{0.f};
	float _alpha_fast{0.45f};
	int32_t _timeout_ms{100};
	float _ignition_threshold_n{10.f};
	float _reference_thrust_n{0.f};
	float _expected_thrust_n{0.f};
	float _expected_motor_mass_kg{0.f};
	float _expected_vehicle_mass_kg{6.5f};
	float _total_impulse_ns{0.f};
	float _burn_fraction{0.f};
	float _sim_burn_time_scale{1.f};
	int32_t _motor_index{0};

	MotorCurveCatalog _motors{};
	bool _motors_loaded{false};
	int32_t _engine_motor_index[kMaxEngines]{};
	bool _engine_burn_active[kMaxEngines]{};
	hrt_abstime _engine_burn_start[kMaxEngines]{};
	float _engine_expected_thrust_n[kMaxEngines]{};
	float _engine_motor_mass_kg[kMaxEngines]{};
	float _engine_burn_fraction[kMaxEngines]{};
	float _engine_remaining_impulse_ns[kMaxEngines]{};

	uint64_t _last_sample_timestamp{0};
	int32_t _last_raw{0};
	float _last_voltage_v{0.f};
	float _measured_thrust_n{0.f};
	float _filtered_thrust_n{0.f};
	float _engine_measured_thrust_n[kMaxEngines]{};
	float _engine_filtered_thrust_n[kMaxEngines]{};
	uint8_t _engine_confirmed_mask{0};

	tv3_sm_modes_s _module_modes{};
	tv3_sm_status_s _state_machine_status{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _adc_report_sub{ORB_ID(adc_report), 1};
	uORB::Subscription _module_modes_sub{ORB_ID(tv3_sm_modes)};
	uORB::Subscription _state_machine_status_sub{ORB_ID(tv3_sm_status)};
	uORB::Publication<tv3_lc_thrust_s> _thrust_pub{ORB_ID(tv3_lc_thrust)};
	uORB::Publication<tv3_lc_ch_s> _load_cell_pub{ORB_ID(tv3_lc_ch)};
	uORB::Publication<tv3_lc_eng_st_s> _engine_state_pub{ORB_ID(tv3_lc_eng_st)};
	uORB::Publication<debug_key_value_s> _debug_key_pub{ORB_ID(debug_key_value)};
	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
};

extern "C" __EXPORT int tv3_load_cell_main(int argc, char *argv[])
{
	return TV3LoadCell::main(argc, argv);
}
