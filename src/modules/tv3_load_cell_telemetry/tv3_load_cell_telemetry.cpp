#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/parameter_update.h>

#include <cmath>
#include <cstdlib>
#include <cstring>

using namespace time_literals;

namespace
{
constexpr float kGravityMps2 = 9.80665f;
constexpr int32_t kModeSingleEnded = 0;
constexpr int32_t kModeDifferential = 1;
constexpr int kRawWindowSize = 5;
constexpr float kDefaultMaxJumpCounts = 120.f;

static float read_param_float(const char *name, float fallback)
{
	param_t p = param_find(name);

	if (p != PARAM_INVALID) {
		param_get(p, &fallback);
	}

	return fallback;
}

static int32_t read_param_int32(const char *name, int32_t fallback)
{
	param_t p = param_find(name);

	if (p != PARAM_INVALID) {
		param_get(p, &fallback);
	}

	return fallback;
}

static bool set_param_float(const char *name, float value)
{
	param_t p = param_find(name);

	if (p == PARAM_INVALID) {
		PX4_ERR("parameter %s not found", name);
		return false;
	}

	return param_set(p, &value) == PX4_OK;
}

static void copy_debug_name(char *dst, size_t dst_size, const char *name)
{
	if (dst_size == 0) {
		return;
	}

	strncpy(dst, name, dst_size);
	dst[dst_size - 1] = '\0';
}
} // namespace

class TV3LoadCellTelemetry : public ModuleBase<TV3LoadCellTelemetry>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TV3LoadCellTelemetry() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		TV3LoadCellTelemetry *instance = new TV3LoadCellTelemetry();

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
		if (argc < 1) {
			return print_usage("missing command");
		}

		TV3LoadCellTelemetry *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("not running");
			return PX4_ERROR;
		}

		if (!strcmp(argv[0], "tare")) {
			return instance->set_tare_from_current_sample();
		}

		if (!strcmp(argv[0], "calibrate")) {
			if (argc < 2) {
				return print_usage("calibrate requires known mass in kg");
			}

			char *end = nullptr;
			const float known_mass_kg = strtof(argv[1], &end);

			if (end == argv[1] || !PX4_ISFINITE(known_mass_kg) || fabsf(known_mass_kg) < 1e-6f) {
				PX4_ERR("invalid known mass kg");
				return PX4_ERROR;
			}

			return instance->calibrate_from_current_sample(known_mass_kg);
		}

		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION("Publishes ADS1115 load-cell mass telemetry to MAVLink debug streams.");
		PRINT_MODULE_USAGE_NAME("tv3_load_cell_telemetry", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_COMMAND("tare");
		PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "set kg/count scale from current load");
		PRINT_MODULE_USAGE_ARG("<known_mass_kg>", "known mass currently on the load cell", false);
		return 0;
	}

	bool init()
	{
		update_schedule();
		return true;
	}

	int print_status() override
	{
		PX4_INFO("adc instance: %ld mode: %s pos_ch: %ld neg_ch: %ld",
			 (long)_adc_instance, _mode == kModeDifferential ? "diff" : "single", (long)_channel, (long)_negative_channel);
		PX4_INFO("raw: %.1f filt_raw: %.1f tare: %.1f kg/count: %.9f kg: %.3f N: %.3f valid: %d",
			 (double)_last_measurement_raw, (double)_filtered_raw, (double)_tare, (double)_kg_per_count,
			 (double)_filtered_mass_kg, (double)_force_n, _valid);
		PX4_INFO("spikes: %lu win: %d jmp: %.0f alpha: %.2f db: %.0f",
			 (unsigned long)_rejected_spikes, _raw_window_count, (double)_max_jump_counts, (double)_alpha,
			 (double)_deadband_counts);
		PX4_INFO("sample age us: %llu rate Hz: %ld", (unsigned long long)hrt_elapsed_time(&_last_sample_timestamp),
			 (long)_rate_hz);
		return 0;
	}

private:
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

		adc_report_s adc{};

		if (_adc_report_sub.update(&adc)) {
			update_measurement(adc);
		}

		if (_last_sample_timestamp != 0
		    && hrt_elapsed_time(&_last_sample_timestamp) > static_cast<hrt_abstime>(_timeout_ms) * 1000ULL) {
			_valid = false;
		}

		publish_debug();
	}

	void update_parameters()
	{
		const int32_t previous_adc_instance = _adc_instance;
		const int32_t previous_rate_hz = _rate_hz;

		_adc_instance = math::constrain(read_param_int32("RK_LC_ADC_INST", _adc_instance), static_cast<int32_t>(0),
						static_cast<int32_t>(7));
		_channel = math::constrain(read_param_int32("RK_LC_CH", _channel), static_cast<int32_t>(0),
					   static_cast<int32_t>(7));
		_negative_channel = math::constrain(read_param_int32("RK_LC_NEG_CH", _negative_channel),
						    static_cast<int32_t>(0), static_cast<int32_t>(7));
		_mode = read_param_int32("RK_LC_MODE", _mode) == kModeDifferential ? kModeDifferential : kModeSingleEnded;
		_tare = read_param_float("RK_LC_TARE", _tare);
		_kg_per_count = read_param_float("RK_LC_KG_SC", _kg_per_count);
		_alpha = 0.08f;
		_deadband_counts = math::max(read_param_float("RK_LC_DB", _deadband_counts), 0.f);
		_max_jump_counts = kDefaultMaxJumpCounts;
		_timeout_ms = math::max(read_param_int32("RK_LC_TO_MS", _timeout_ms), static_cast<int32_t>(10));
		_rate_hz = math::constrain(read_param_int32("RK_LC_RATE_HZ", _rate_hz), static_cast<int32_t>(1),
					   static_cast<int32_t>(50));

		if (_adc_instance != previous_adc_instance) {
			_adc_report_sub.ChangeInstance(static_cast<uint8_t>(_adc_instance));
		}

		if (_rate_hz != previous_rate_hz) {
			update_schedule();
		}
	}

	void update_schedule()
	{
		const uint32_t interval_us = 1000000UL / static_cast<uint32_t>(math::max(_rate_hz, static_cast<int32_t>(1)));
		ScheduleOnInterval(interval_us);
	}

	bool find_channel(const adc_report_s &adc, int32_t channel, int32_t &raw, float &voltage_v) const
	{
		for (size_t i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); ++i) {
			if (adc.channel_id[i] == channel) {
				raw = adc.raw_data[i];
				voltage_v = adc.resolution > 0 ? static_cast<float>(raw) * adc.v_ref / static_cast<float>(adc.resolution) : 0.f;
				return true;
			}
		}

		return false;
	}

	void update_measurement(const adc_report_s &adc)
	{
		int32_t positive_raw = 0;
		int32_t negative_raw = 0;
		float positive_voltage_v = 0.f;
		float negative_voltage_v = 0.f;

		const bool positive_found = find_channel(adc, _channel, positive_raw, positive_voltage_v);
		bool negative_found = true;

		if (_mode == kModeDifferential) {
			negative_found = find_channel(adc, _negative_channel, negative_raw, negative_voltage_v);
		}

		if (!positive_found || !negative_found) {
			_valid = false;
			return;
		}

		_last_sample_timestamp = adc.timestamp != 0 ? adc.timestamp : hrt_absolute_time();
		_last_positive_raw = positive_raw;
		_last_negative_raw = _mode == kModeDifferential ? negative_raw : 0;
		_last_measurement_raw = _mode == kModeDifferential ? static_cast<float>(positive_raw - negative_raw) : static_cast<float>(positive_raw);
		_last_voltage_v = _mode == kModeDifferential ? positive_voltage_v - negative_voltage_v : positive_voltage_v;

		if (!accept_raw_sample(_last_measurement_raw)) {
			_valid = fabsf(_kg_per_count) >= 1e-9f && _raw_window_count >= 3;
			return;
		}

		float delta_counts = _filtered_raw - _tare;

		if (fabsf(delta_counts) < _deadband_counts) {
			delta_counts = 0.f;
		}

		const float measured_mass_kg = delta_counts * _kg_per_count;

		if (!_has_filtered_sample) {
			_filtered_mass_kg = measured_mass_kg;
			_has_filtered_sample = true;
		} else {
			_filtered_mass_kg = _alpha * measured_mass_kg + (1.f - _alpha) * _filtered_mass_kg;
		}

		_force_n = _filtered_mass_kg * kGravityMps2;
		_valid = fabsf(_kg_per_count) >= 1e-9f;
	}

	void publish_debug()
	{
		const hrt_abstime now = hrt_absolute_time();

		debug_key_value_s named{};
		named.timestamp = now;
		copy_debug_name(named.key, sizeof(named.key), "lc_kg");
		named.value = _filtered_mass_kg;
		_named_value_pub.publish(named);

		debug_vect_s vect{};
		vect.timestamp = now;
		copy_debug_name(vect.name, sizeof(vect.name), "lc_data");
		vect.x = _filtered_raw;
		vect.y = _filtered_mass_kg;
		vect.z = _force_n;
		_debug_vect_pub.publish(vect);
	}

	bool accept_raw_sample(float raw)
	{
		if (_raw_window_count >= 3) {
			const float reference = median_raw_window();

			if (fabsf(raw - reference) > _max_jump_counts) {
				_rejected_spikes++;
				return false;
			}
		}

		_raw_window[_raw_window_index] = raw;
		_raw_window_index = (_raw_window_index + 1) % kRawWindowSize;

		if (_raw_window_count < kRawWindowSize) {
			_raw_window_count++;
		}

		_filtered_raw = median_raw_window();
		return true;
	}

	float median_raw_window() const
	{
		if (_raw_window_count == 0) {
			return _last_measurement_raw;
		}

		float sorted[kRawWindowSize]{};

		for (int i = 0; i < _raw_window_count; ++i) {
			sorted[i] = _raw_window[i];
		}

		for (int i = 1; i < _raw_window_count; ++i) {
			float key = sorted[i];
			int j = i - 1;

			while (j >= 0 && sorted[j] > key) {
				sorted[j + 1] = sorted[j];
				j--;
			}

			sorted[j + 1] = key;
		}

		return sorted[_raw_window_count / 2];
	}

	float robust_raw_sample() const
	{
		if (_raw_window_count >= 3) {
			return _filtered_raw;
		}

		return _last_measurement_raw;
	}

	int set_tare_from_current_sample()
	{
		if (_last_sample_timestamp == 0 || _raw_window_count < 3) {
			PX4_ERR("no ADC sample yet");
			return PX4_ERROR;
		}

		const float tare = robust_raw_sample();

		if (!set_param_float("RK_LC_TARE", tare)) {
			return PX4_ERROR;
		}

		_tare = tare;
		_filtered_mass_kg = 0.f;
		_has_filtered_sample = false;
		PX4_INFO("set RK_LC_TARE %.3f counts (median of %d samples); run 'param save' to persist", (double)tare,
			 _raw_window_count);
		return PX4_OK;
	}

	int calibrate_from_current_sample(float known_mass_kg)
	{
		if (_last_sample_timestamp == 0 || _raw_window_count < 3) {
			PX4_ERR("no ADC sample yet");
			return PX4_ERROR;
		}

		const float delta_counts = robust_raw_sample() - _tare;

		if (fabsf(delta_counts) < 1.f) {
			PX4_ERR("load delta too small: %.3f counts", (double)delta_counts);
			return PX4_ERROR;
		}

		const float kg_per_count = known_mass_kg / delta_counts;

		if (!set_param_float("RK_LC_KG_SC", kg_per_count)) {
			return PX4_ERROR;
		}

		_kg_per_count = kg_per_count;
		_has_filtered_sample = false;
		PX4_INFO("set RK_LC_KG_SC %.9f kg/count from %.3f kg over %.3f counts; run 'param save' to persist",
			 (double)kg_per_count, (double)known_mass_kg, (double)delta_counts);
		return PX4_OK;
	}

	int32_t _adc_instance{1};
	int32_t _channel{0};
	int32_t _negative_channel{1};
	int32_t _mode{kModeDifferential};
	float _tare{0.f};
	float _kg_per_count{0.f};
	float _alpha{0.08f};
	float _deadband_counts{2.f};
	float _max_jump_counts{120.f};
	int32_t _timeout_ms{200};
	int32_t _rate_hz{10};

	uint64_t _last_sample_timestamp{0};
	int32_t _last_positive_raw{0};
	int32_t _last_negative_raw{0};
	float _last_measurement_raw{0.f};
	float _filtered_raw{0.f};
	float _raw_window[kRawWindowSize]{};
	int _raw_window_count{0};
	int _raw_window_index{0};
	uint32_t _rejected_spikes{0};
	float _last_voltage_v{0.f};
	float _filtered_mass_kg{0.f};
	float _force_n{0.f};
	bool _valid{false};
	bool _has_filtered_sample{false};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _adc_report_sub{ORB_ID(adc_report), 1};
	uORB::Publication<debug_key_value_s> _named_value_pub{ORB_ID(debug_key_value)};
	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
};

extern "C" __EXPORT int tv3_load_cell_telemetry_main(int argc, char *argv[])
{
	return TV3LoadCellTelemetry::main(argc, argv);
}
