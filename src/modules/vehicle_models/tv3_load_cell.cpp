#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_engine_state.h>
#include <uORB/topics/tv3_load_cell.h>
#include <uORB/topics/tv3_motor_reference.h>
#include <uORB/topics/tv3_thrust.h>

using namespace time_literals;

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

		PRINT_MODULE_DESCRIPTION("Combines ADC-backed load-cell data with expected motor references.");
		PRINT_MODULE_USAGE_NAME("tv3_load_cell", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(20_ms);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("source: %d channel: %d thrust: %.3f", _source, _channel, (double)_filtered_thrust_n);
		return 0;
	}

private:
	static constexpr int32_t SOURCE_ADC = 0;
	static constexpr int32_t SOURCE_REFERENCE = 1;
	static constexpr int kMaxEngines = 4;

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

		tv3_motor_reference_s ref{};
		if (_motor_reference_sub.update(&ref)) {
			_reference = ref;
		}

		uint8_t fault_flags = tv3_thrust_s::FAULT_NONE;

			if (_source == SOURCE_REFERENCE) {
				_measured_thrust_n = _reference.expected_thrust_n;
				_last_sample_timestamp = hrt_absolute_time();
				_last_raw = static_cast<int32_t>(lrintf(_measured_thrust_n * 100.f));
			_last_voltage_v = _measured_thrust_n;

		} else {
			adc_report_s adc{};
			bool found = false;

			if (_adc_report_sub.update(&adc)) {
				for (size_t i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); ++i) {
					if (adc.channel_id[i] == _channel) {
						_last_sample_timestamp = adc.timestamp;
						_last_raw = adc.raw_data[i];
						_last_voltage_v = adc.resolution > 0 ? static_cast<float>(_last_raw) * adc.v_ref / static_cast<float>(adc.resolution) : 0.f;
						_measured_thrust_n = math::max((_last_raw - _tare) * _scale, 0.f);
						found = true;
						break;
					}
				}
			}

			if (!found) {
				fault_flags |= tv3_thrust_s::FAULT_CHANNEL_MISSING;
			}
		}

		if (fabsf(_scale) < 1e-6f && _source == SOURCE_ADC) {
			fault_flags |= tv3_thrust_s::FAULT_BAD_SCALE;
		}

		if (_reference.loaded == false) {
			fault_flags |= tv3_thrust_s::FAULT_NO_REFERENCE;
		}

		if (_last_sample_timestamp == 0) {
			fault_flags |= tv3_thrust_s::FAULT_STALE;
		} else if (hrt_elapsed_time(&_last_sample_timestamp) > static_cast<hrt_abstime>(_timeout_ms) * 1000ULL) {
			fault_flags |= tv3_thrust_s::FAULT_STALE;
		}

			_filtered_thrust_n = _alpha * _measured_thrust_n + (1.f - _alpha) * _filtered_thrust_n;
			update_engine_state_from_reference(fault_flags);
			const bool ignition_confirmed = _engine_confirmed_mask != 0 || _filtered_thrust_n >= _ignition_threshold_n;

			tv3_thrust_s out{};
		out.timestamp = hrt_absolute_time();
		out.timestamp_sample = _last_sample_timestamp;
		out.measured_thrust_n = _measured_thrust_n;
		out.filtered_thrust_n = _filtered_thrust_n;
		out.expected_thrust_n = _reference.expected_thrust_n;
		out.expected_motor_mass_kg = _reference.expected_motor_mass_kg;
		out.expected_vehicle_mass_kg = _reference.expected_vehicle_mass_kg;
		out.total_impulse_ns = _reference.total_impulse_ns;
		out.burn_fraction = _reference.burn_fraction;
		out.valid = fault_flags == tv3_thrust_s::FAULT_NONE;
		out.ignition_confirmed = ignition_confirmed;
		out.fault_flags = fault_flags;
		out.selected_motor_index = _reference.selected_motor_index;
		memcpy(out.selected_motor_id, _reference.selected_motor_id, sizeof(out.selected_motor_id));
		_thrust_pub.publish(out);

		tv3_load_cell_s compat{};
		compat.timestamp = out.timestamp;
		compat.timestamp_sample = out.timestamp_sample;
		compat.channel = static_cast<int8_t>(_channel);
		compat.raw_count = _last_raw;
		compat.voltage_v = _last_voltage_v;
		compat.thrust_n = out.filtered_thrust_n;
		compat.valid = out.valid;

		uint8_t compat_faults = tv3_load_cell_s::FAULT_NONE;

		if (fault_flags & tv3_thrust_s::FAULT_STALE) {
			compat_faults |= tv3_load_cell_s::FAULT_STALE | tv3_load_cell_s::FAULT_NO_SAMPLE;
		}

		if (fault_flags & tv3_thrust_s::FAULT_CHANNEL_MISSING) {
			compat_faults |= tv3_load_cell_s::FAULT_CHANNEL_MISSING;
		}

		if (fault_flags & tv3_thrust_s::FAULT_BAD_SCALE) {
			compat_faults |= tv3_load_cell_s::FAULT_BAD_SCALE;
		}

		compat.fault_flags = compat_faults;
			_load_cell_pub.publish(compat);
		}

		void update_engine_state_from_reference(uint8_t aggregate_fault_flags)
		{
			const int engine_count = math::constrain(_reference.engine_count > 0 ? (int)_reference.engine_count : 1, 1, kMaxEngines);
			float expected_sum = 0.f;

			for (int i = 0; i < engine_count; ++i) {
				const float expected = _reference.engine_count > 0 ? _reference.expected_thrust_n_engine[i] : _reference.expected_thrust_n;
				expected_sum += math::max(expected, 0.f);
			}

			_engine_confirmed_mask = 0;
			tv3_engine_state_s state{};
			state.timestamp = hrt_absolute_time();
			state.timestamp_sample = _last_sample_timestamp;
			state.engine_count = static_cast<uint8_t>(engine_count);
			state.ignition_mask = _reference.ignition_mask;
			state.active_mask = _reference.active_mask;

			for (int i = 0; i < engine_count; ++i) {
				const float expected = _reference.engine_count > 0 ? _reference.expected_thrust_n_engine[i] : _reference.expected_thrust_n;

				if (_source == SOURCE_REFERENCE) {
					_engine_measured_thrust_n[i] = expected;
				} else if (expected_sum > 1e-3f) {
					_engine_measured_thrust_n[i] = _measured_thrust_n * math::max(expected, 0.f) / expected_sum;
				} else {
					_engine_measured_thrust_n[i] = i == 0 ? _measured_thrust_n : 0.f;
				}

				_engine_filtered_thrust_n[i] = _alpha * _engine_measured_thrust_n[i] + (1.f - _alpha) * _engine_filtered_thrust_n[i];

				state.measured_thrust_n[i] = _engine_measured_thrust_n[i];
				state.filtered_thrust_n[i] = _engine_filtered_thrust_n[i];
				state.expected_thrust_n[i] = expected;
				state.expected_motor_mass_kg[i] = _reference.engine_count > 0 ? _reference.expected_motor_mass_kg_engine[i]
									: _reference.expected_motor_mass_kg;
				state.burn_fraction[i] = _reference.engine_count > 0 ? _reference.burn_fraction_engine[i] : _reference.burn_fraction;
				state.remaining_impulse_ns[i] = math::max(_reference.total_impulse_ns * (1.f - state.burn_fraction[i]), 0.f);
				state.fault_flags[i] = aggregate_fault_flags;
				state.selected_motor_index[i] = _reference.engine_count > 0 ? _reference.selected_motor_index_engine[i]
										   : _reference.selected_motor_index;

				if (_engine_filtered_thrust_n[i] >= _ignition_threshold_n) {
					_engine_confirmed_mask |= static_cast<uint8_t>(1u << i);
				}
			}

			state.confirmed_mask = _engine_confirmed_mask;
			state.fault_mask = aggregate_fault_flags == tv3_thrust_s::FAULT_NONE ? 0 : ((1u << engine_count) - 1u);
			state.sequence_complete = state.ignition_mask != 0 && (state.confirmed_mask & state.ignition_mask) == state.ignition_mask;
			state.all_ignited = state.sequence_complete;
			_engine_state_pub.publish(state);
		}

	void update_parameters()
	{
		param_t p = param_find("RK_LC_SRC");
		if (p != PARAM_INVALID) {
			param_get(p, &_source);
		}

		p = param_find("RK_LC_CH");
		if (p != PARAM_INVALID) {
			param_get(p, &_channel);
		}

		p = param_find("RK_LC_TARE");
		if (p != PARAM_INVALID) {
			param_get(p, &_tare);
		}

		p = param_find("RK_LC_SCALE");
		if (p != PARAM_INVALID) {
			param_get(p, &_scale);
		}

		p = param_find("RK_LC_ALPHA");
		if (p != PARAM_INVALID) {
			param_get(p, &_alpha);
			_alpha = math::constrain(_alpha, 0.01f, 1.f);
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
	}

	int32_t _source{SOURCE_ADC};
	int32_t _channel{0};
	float _tare{0.f};
	float _scale{1.f};
	float _alpha{0.25f};
	int32_t _timeout_ms{200};
	float _ignition_threshold_n{10.f};

	uint64_t _last_sample_timestamp{0};
	int32_t _last_raw{0};
	float _last_voltage_v{0.f};
		float _measured_thrust_n{0.f};
		float _filtered_thrust_n{0.f};
		float _engine_measured_thrust_n[kMaxEngines]{};
		float _engine_filtered_thrust_n[kMaxEngines]{};
		uint8_t _engine_confirmed_mask{0};
		tv3_motor_reference_s _reference{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _adc_report_sub{ORB_ID(adc_report)};
		uORB::Subscription _motor_reference_sub{ORB_ID(tv3_motor_reference)};
		uORB::Publication<tv3_thrust_s> _thrust_pub{ORB_ID(tv3_thrust)};
		uORB::Publication<tv3_load_cell_s> _load_cell_pub{ORB_ID(tv3_load_cell)};
		uORB::Publication<tv3_engine_state_s> _engine_state_pub{ORB_ID(tv3_engine_state)};
	};

extern "C" __EXPORT int tv3_load_cell_main(int argc, char *argv[])
{
	return TV3LoadCell::main(argc, argv);
}
