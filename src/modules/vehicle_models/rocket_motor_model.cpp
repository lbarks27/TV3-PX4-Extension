#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rocket_motor_reference.h>
#include <uORB/topics/rocket_status.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

using namespace time_literals;

namespace
{
constexpr const char *kDefaultMotorRoot = "/fs/microsd/tv3/motors";
constexpr size_t kMaxLineLength = 1024;

struct CatalogEntry {
	uint16_t index{0};
	std::string motor_id;
	std::string curve_file;
	std::string specs_file;
	bool active{false};
};

struct CurvePoint {
	float time_s{0.f};
	float thrust_n{0.f};
	float motor_mass_kg{0.f};
	float burn_fraction{0.f};
	float cumulative_impulse_ns{0.f};
};

struct MotorSpecs {
	float loaded_mass_kg{0.f};
	float dry_mass_kg{0.f};
	float diameter_m{0.f};
	float length_m{0.f};
	float total_impulse_ns{0.f};
	float burn_duration_s{0.f};
};

static inline std::string trim(const std::string &value)
{
	size_t start = 0;
	while (start < value.size() && isspace(static_cast<unsigned char>(value[start])) != 0) {
		++start;
	}

	size_t end = value.size();
	while (end > start && isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
		--end;
	}

	return value.substr(start, end - start);
}

static std::vector<std::string> parse_csv_line(const std::string &line)
{
	std::vector<std::string> fields;
	std::string current;
	bool in_quotes = false;

	for (char c : line) {
		if (c == '"') {
			in_quotes = !in_quotes;
			continue;
		}

		if (c == ',' && !in_quotes) {
			fields.push_back(trim(current));
			current.clear();
			continue;
		}

		current.push_back(c);
	}

	fields.push_back(trim(current));
	return fields;
}

static bool read_lines(const std::string &path, std::vector<std::string> &lines)
{
	FILE *fp = fopen(path.c_str(), "r");

	if (fp == nullptr) {
		return false;
	}

	char buffer[kMaxLineLength];

	while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
		std::string line(buffer);
		if (!line.empty() && line.back() == '\n') {
			line.pop_back();
		}
		if (!line.empty() && line.back() == '\r') {
			line.pop_back();
		}
		if (!line.empty()) {
			lines.push_back(line);
		}
	}

	fclose(fp);
	return true;
}

static void copy_motor_id(char (&dst)[32], const std::string &src)
{
	memset(dst, 0, sizeof(dst));
	strncpy(dst, src.c_str(), sizeof(dst) - 1);
}

static bool is_nonempty(const std::string &value)
{
	return !trim(value).empty();
}

static bool parse_float(const std::string &value, float &out)
{
	char *end = nullptr;
	out = strtof(value.c_str(), &end);
	return end != value.c_str();
}

static std::string join_path(const std::string &root, const std::string &relative)
{
	if (relative.empty()) {
		return root;
	}

	if (!relative.empty() && relative.front() == '/') {
		return relative;
	}

	if (!root.empty() && root.back() == '/') {
		return root + relative;
	}

	return root + "/" + relative;
}
} // namespace

class RocketMotorModel : public ModuleBase<RocketMotorModel>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RocketMotorModel() :
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	{
		const char *env_root = getenv("TV3_MOTOR_ROOT");
		_motor_root = env_root != nullptr ? env_root : kDefaultMotorRoot;
		_param_motor_index = param_find("RK_MOT_IDX");
		_param_body_mass_kg = param_find("RK_BODY_MASS_KG");
		_param_parameter_update = ORB_ID(parameter_update);
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		int ch;
		const char *data_root = nullptr;

		while ((ch = px4_getopt(argc, argv, "d:", nullptr, nullptr)) != EOF) {
			switch (ch) {
			case 'd':
				data_root = optarg;
				break;

			default:
				return print_usage("unknown option");
			}
		}

		RocketMotorModel *instance = new RocketMotorModel();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		if (data_root != nullptr) {
			instance->_motor_root = data_root;
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

		PRINT_MODULE_DESCRIPTION("Loads normalized motor data from SD and publishes expected thrust and mass.");
		PRINT_MODULE_USAGE_NAME("rocket_motor_model", "modules");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Motor root directory", true);
		return 0;
	}

	bool init()
	{
		ScheduleOnInterval(20_ms);
		return load_selected_motor();
	}

	int print_status() override
	{
		PX4_INFO("motor root: %s", _motor_root.c_str());
		PX4_INFO("motor index: %d", _motor_index);
		PX4_INFO("selected motor: %s", _selected.motor_id.c_str());
		PX4_INFO("curve points: %u", (unsigned)_curve.size());
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
			const int32_t previous_index = _motor_index;
			update_parameters();

			if (previous_index != _motor_index) {
				load_selected_motor();
			}
		}

		update_status();
		publish_reference();
	}

	void update_parameters()
	{
		if (_param_motor_index != PARAM_INVALID) {
			param_get(_param_motor_index, &_motor_index);
			_motor_index = math::max(_motor_index, 0);
		}

		if (_param_body_mass_kg != PARAM_INVALID) {
			param_get(_param_body_mass_kg, &_body_mass_kg);
			_body_mass_kg = math::max(_body_mass_kg, 0.f);
		}
	}

	bool load_catalog(std::vector<CatalogEntry> &entries)
	{
		const std::string catalog_path = join_path(_motor_root, "catalog.csv");
		std::vector<std::string> lines;

		if (!read_lines(catalog_path, lines) || lines.size() < 2) {
			PX4_ERR("failed to read %s", catalog_path.c_str());
			return false;
		}

		const std::vector<std::string> header = parse_csv_line(lines[0]);
		int idx_col = -1;
		int id_col = -1;
		int curve_col = -1;
		int specs_col = -1;
		int active_col = -1;

		for (size_t i = 0; i < header.size(); ++i) {
			if (header[i] == "motor_index") {
				idx_col = (int)i;
			} else if (header[i] == "motor_id") {
				id_col = (int)i;
			} else if (header[i] == "curve_file") {
				curve_col = (int)i;
			} else if (header[i] == "specs_file") {
				specs_col = (int)i;
			} else if (header[i] == "active") {
				active_col = (int)i;
			}
		}

		if (idx_col < 0 || id_col < 0 || curve_col < 0 || specs_col < 0) {
			PX4_ERR("catalog header invalid");
			return false;
		}

		for (size_t line_idx = 1; line_idx < lines.size(); ++line_idx) {
			const std::vector<std::string> fields = parse_csv_line(lines[line_idx]);

			if ((int)fields.size() <= std::max(std::max(idx_col, id_col), std::max(curve_col, specs_col))) {
				continue;
			}

			CatalogEntry entry{};
			entry.index = static_cast<uint16_t>(strtoul(fields[idx_col].c_str(), nullptr, 10));
			entry.motor_id = fields[id_col];
			entry.curve_file = fields[curve_col];
			entry.specs_file = fields[specs_col];
			entry.active = active_col >= 0 && active_col < (int)fields.size() ? strtoul(fields[active_col].c_str(), nullptr, 10) > 0 : true;
			entries.push_back(entry);
		}

		return !entries.empty();
	}

	bool load_specs(const std::string &path, MotorSpecs &specs)
	{
		std::vector<std::string> lines;

		if (!read_lines(path, lines) || lines.size() < 2) {
			PX4_ERR("failed to read specs %s", path.c_str());
			return false;
		}

		const std::vector<std::string> header = parse_csv_line(lines[0]);
		const std::vector<std::string> values = parse_csv_line(lines[1]);

		auto get_value = [&](const char *name, float &out) -> bool {
			for (size_t i = 0; i < header.size() && i < values.size(); ++i) {
				if (header[i] == name) {
					return parse_float(values[i], out);
				}
			}

			return false;
		};

		return get_value("loaded_mass_kg", specs.loaded_mass_kg)
		       && get_value("dry_mass_kg", specs.dry_mass_kg)
		       && get_value("diameter_m", specs.diameter_m)
		       && get_value("length_m", specs.length_m)
		       && get_value("total_impulse_ns", specs.total_impulse_ns)
		       && get_value("burn_duration_s", specs.burn_duration_s);
	}

	bool load_curve(const std::string &path, std::vector<CurvePoint> &curve)
	{
		std::vector<std::string> lines;

		if (!read_lines(path, lines) || lines.size() < 2) {
			PX4_ERR("failed to read curve %s", path.c_str());
			return false;
		}

		for (size_t i = 1; i < lines.size(); ++i) {
			const std::vector<std::string> fields = parse_csv_line(lines[i]);

			if (fields.size() < 5) {
				continue;
			}

			CurvePoint point{};

			if (!parse_float(fields[0], point.time_s)
			    || !parse_float(fields[1], point.thrust_n)
			    || !parse_float(fields[2], point.motor_mass_kg)
			    || !parse_float(fields[3], point.burn_fraction)
			    || !parse_float(fields[4], point.cumulative_impulse_ns)) {
				continue;
			}

			if (!curve.empty() && point.time_s <= curve.back().time_s) {
				PX4_ERR("non-monotonic curve %s", path.c_str());
				return false;
			}

			if (point.motor_mass_kg < -1e-3f || point.thrust_n < -1e-3f) {
				PX4_ERR("invalid curve sample %s", path.c_str());
				return false;
			}

			curve.push_back(point);
		}

		return !curve.empty();
	}

	bool load_selected_motor()
	{
		std::vector<CatalogEntry> entries;

		if (!load_catalog(entries)) {
			_motor_loaded = false;
			return false;
		}

		auto it = std::find_if(entries.begin(), entries.end(), [&](const CatalogEntry &entry) {
			return entry.index == static_cast<uint16_t>(_motor_index) && entry.active;
		});

		if (it == entries.end()) {
			PX4_ERR("motor index %d unavailable", _motor_index);
			_motor_loaded = false;
			return false;
		}

		std::vector<CurvePoint> curve;
		MotorSpecs specs{};
		const std::string curve_path = join_path(_motor_root, it->curve_file);
		const std::string specs_path = join_path(_motor_root, it->specs_file);

		if (!load_curve(curve_path, curve) || !load_specs(specs_path, specs)) {
			_motor_loaded = false;
			return false;
		}

		_selected = *it;
		_curve = curve;
		_specs = specs;
		_motor_loaded = true;
		_burn_active = false;
		_burn_start = 0;
		return true;
	}

	void update_status()
	{
		rocket_status_s status{};

		if (_rocket_status_sub.update(&status)) {
			_status = status;
		}

		const bool should_burn = (_status.mode == rocket_status_s::MODE_IGNITION_PENDING
					|| _status.mode == rocket_status_s::MODE_BOOST);

		if (should_burn && !_burn_active) {
			_burn_active = true;
			_burn_start = hrt_absolute_time();
		}

		if (!should_burn) {
			_burn_active = false;
			_burn_start = 0;
		}
	}

	void sample_curve(float burn_time_s, float &thrust_n, float &motor_mass_kg, float &burn_fraction, float &impulse_ns) const
	{
		if (_curve.empty()) {
			thrust_n = 0.f;
			motor_mass_kg = 0.f;
			burn_fraction = 0.f;
			impulse_ns = 0.f;
			return;
		}

		if (burn_time_s <= _curve.front().time_s) {
			thrust_n = _curve.front().thrust_n;
			motor_mass_kg = _curve.front().motor_mass_kg;
			burn_fraction = _curve.front().burn_fraction;
			impulse_ns = _curve.front().cumulative_impulse_ns;
			return;
		}

		for (size_t i = 1; i < _curve.size(); ++i) {
			const CurvePoint &a = _curve[i - 1];
			const CurvePoint &b = _curve[i];

			if (burn_time_s <= b.time_s) {
				const float alpha = (burn_time_s - a.time_s) / math::max(b.time_s - a.time_s, 1e-4f);
				thrust_n = a.thrust_n + alpha * (b.thrust_n - a.thrust_n);
				motor_mass_kg = a.motor_mass_kg + alpha * (b.motor_mass_kg - a.motor_mass_kg);
				burn_fraction = a.burn_fraction + alpha * (b.burn_fraction - a.burn_fraction);
				impulse_ns = a.cumulative_impulse_ns + alpha * (b.cumulative_impulse_ns - a.cumulative_impulse_ns);
				return;
			}
		}

		thrust_n = 0.f;
		motor_mass_kg = math::max(_specs.dry_mass_kg, 0.f);
		burn_fraction = 1.f;
		impulse_ns = _specs.total_impulse_ns;
	}

	void publish_reference()
	{
		rocket_motor_reference_s ref{};
		ref.timestamp = hrt_absolute_time();
		ref.loaded = _motor_loaded;
		ref.active = _motor_loaded && _burn_active;
		ref.selected_motor_index = static_cast<uint16_t>(_motor_index);
		copy_motor_id(ref.selected_motor_id, _selected.motor_id);

		if (_motor_loaded) {
			ref.burn_duration_s = _specs.burn_duration_s;
			ref.total_impulse_ns = _specs.total_impulse_ns;
			float thrust_n = 0.f;
			float motor_mass_kg = _specs.loaded_mass_kg;
			float burn_fraction = 0.f;
			float impulse_ns = 0.f;
			float burn_time_s = 0.f;

			if (_burn_active && _burn_start != 0) {
				burn_time_s = static_cast<float>(hrt_absolute_time() - _burn_start) * 1e-6f;
				sample_curve(burn_time_s, thrust_n, motor_mass_kg, burn_fraction, impulse_ns);
			}

			ref.burn_time_s = burn_time_s;
			ref.expected_thrust_n = thrust_n;
			ref.expected_motor_mass_kg = math::max(motor_mass_kg, _specs.dry_mass_kg);
			ref.expected_vehicle_mass_kg = _body_mass_kg + ref.expected_motor_mass_kg;
			ref.burn_fraction = math::constrain(burn_fraction, 0.f, 1.f);
			ref.total_impulse_ns = _specs.total_impulse_ns;
		}

		_ref_pub.publish(ref);
	}

	std::string _motor_root{kDefaultMotorRoot};
	int32_t _motor_index{0};
	float _body_mass_kg{0.f};
	bool _motor_loaded{false};
	bool _burn_active{false};
	hrt_abstime _burn_start{0};

	CatalogEntry _selected{};
	MotorSpecs _specs{};
	std::vector<CurvePoint> _curve{};
	rocket_status_s _status{};

	param_t _param_motor_index{PARAM_INVALID};
	param_t _param_body_mass_kg{PARAM_INVALID};
	orb_id_t _param_parameter_update{};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _rocket_status_sub{ORB_ID(rocket_status)};
	uORB::Publication<rocket_motor_reference_s> _ref_pub{ORB_ID(rocket_motor_reference)};
};

extern "C" __EXPORT int rocket_motor_model_main(int argc, char *argv[])
{
	return RocketMotorModel::main(argc, argv);
}
