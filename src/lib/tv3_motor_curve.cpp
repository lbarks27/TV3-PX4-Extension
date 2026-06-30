#include "tv3_motor_curve.hpp"

#include <px4_platform_common/log.h>

#include <mathlib/mathlib.h>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

namespace tv3
{

namespace
{

constexpr size_t kMaxLineLength = 1024;

std::string trim(const std::string &value)
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

bool parse_csv_line(const char *line, char fields[][64], int max_fields, int &field_count)
{
	field_count = 0;
	std::string current;
	bool in_quotes = false;

	for (const char *c = line; *c != '\0'; ++c) {
		if (*c == '"') {
			in_quotes = !in_quotes;
			continue;
		}

		if (*c == ',' && !in_quotes) {
			if (field_count < max_fields) {
				strncpy(fields[field_count], trim(current).c_str(), sizeof(fields[field_count]) - 1);
				fields[field_count][sizeof(fields[field_count]) - 1] = '\0';
				field_count++;
			}

			current.clear();
			continue;
		}

		current.push_back(*c);
	}

	if (field_count < max_fields) {
		strncpy(fields[field_count], trim(current).c_str(), sizeof(fields[field_count]) - 1);
		fields[field_count][sizeof(fields[field_count]) - 1] = '\0';
		field_count++;
	}

	return field_count > 0;
}

bool read_lines(const char *path, char lines[][kMaxLineLength], int max_lines, int &line_count)
{
	FILE *fp = fopen(path, "r");

	if (fp == nullptr) {
		return false;
	}

	line_count = 0;

	while (line_count < max_lines && fgets(lines[line_count], kMaxLineLength, fp) != nullptr) {
		const size_t len = strlen(lines[line_count]);

		if (len > 0 && lines[line_count][len - 1] == '\n') {
			lines[line_count][len - 1] = '\0';
		}

		if (len > 1 && lines[line_count][len - 2] == '\r') {
			lines[line_count][len - 2] = '\0';
		}

		if (lines[line_count][0] != '\0') {
			line_count++;
		}
	}

	fclose(fp);
	return line_count > 0;
}

bool parse_float(const char *value, float &out)
{
	char *end = nullptr;
	out = strtof(value, &end);
	return end != value;
}

void join_path(const char *root, const char *relative, char *out, size_t out_len)
{
	if (relative == nullptr || relative[0] == '\0') {
		strncpy(out, root, out_len - 1);
		out[out_len - 1] = '\0';
		return;
	}

	if (relative[0] == '/') {
		strncpy(out, relative, out_len - 1);
		out[out_len - 1] = '\0';
		return;
	}

	const size_t root_len = strlen(root);
	const bool needs_sep = root_len > 0 && root[root_len - 1] != '/';
	snprintf(out, out_len, needs_sep ? "%s/%s" : "%s%s", root, relative);
}

int find_column(char fields[][64], int field_count, const char *name)
{
	for (int i = 0; i < field_count; ++i) {
		if (strcmp(fields[i], name) == 0) {
			return i;
		}
	}

	return -1;
}

int find_column_any(char fields[][64], int field_count, const char *primary, const char *fallback)
{
	const int primary_col = find_column(fields, field_count, primary);
	return primary_col >= 0 ? primary_col : find_column(fields, field_count, fallback);
}

} // namespace

bool MotorCurveCatalog::load(const char *motor_root)
{
	_loaded = false;
	_catalog_count = 0;

	for (auto &engine : _engines) {
		engine = EngineSlot{};
	}

	if (motor_root != nullptr && motor_root[0] != '\0') {
		strncpy(_motor_root, motor_root, sizeof(_motor_root) - 1);
		_motor_root[sizeof(_motor_root) - 1] = '\0';
	}

	char catalog_path[192]{};
	join_path(_motor_root, "catalog.csv", catalog_path, sizeof(catalog_path));

	char lines[32][kMaxLineLength]{};
	int line_count = 0;

	if (!read_lines(catalog_path, lines, 32, line_count) || line_count < 2) {
		PX4_ERR("motor catalog unavailable at %s", catalog_path);
		return false;
	}

	char header_fields[16][64]{};
	int header_count = 0;

	if (!parse_csv_line(lines[0], header_fields, 16, header_count)) {
		return false;
	}

	const int idx_col = find_column(header_fields, header_count, "motor_index");
	const int id_col = find_column(header_fields, header_count, "motor_id");
	const int curve_col = find_column(header_fields, header_count, "curve_file");
	const int specs_col = find_column(header_fields, header_count, "specs_file");
	const int active_col = find_column(header_fields, header_count, "active");

	if (idx_col < 0 || id_col < 0 || curve_col < 0 || specs_col < 0) {
		PX4_ERR("motor catalog header invalid");
		return false;
	}

	for (int line_idx = 1; line_idx < line_count && _catalog_count < kMotorCurveMaxCatalog; ++line_idx) {
		char fields[8][64]{};
		int field_count = 0;

		if (!parse_csv_line(lines[line_idx], fields, 8, field_count)) {
			continue;
		}

		const int max_col = idx_col > id_col ? idx_col : id_col;
		const int max_col2 = curve_col > specs_col ? curve_col : specs_col;

		if (field_count <= (max_col > max_col2 ? max_col : max_col2)) {
			continue;
		}

		CatalogEntry &entry = _catalog[_catalog_count];
		entry.index = static_cast<uint16_t>(strtoul(fields[idx_col], nullptr, 10));
		strncpy(entry.motor_id, fields[id_col], sizeof(entry.motor_id) - 1);
		strncpy(entry.curve_file, fields[curve_col], sizeof(entry.curve_file) - 1);
		strncpy(entry.specs_file, fields[specs_col], sizeof(entry.specs_file) - 1);
		entry.active = active_col < 0 || active_col >= field_count
			       || strtoul(fields[active_col], nullptr, 10) > 0;
		_catalog_count++;
	}

	if (_catalog_count == 0) {
		PX4_ERR("motor catalog empty at %s", catalog_path);
		return false;
	}

	_loaded = true;
	return true;
}

bool MotorCurveCatalog::load_curve_file(const char *path, EngineSlot &slot)
{
	FILE *fp = fopen(path, "r");

	if (fp == nullptr) {
		PX4_ERR("failed to read motor curve %s", path);
		return false;
	}

	char line[kMaxLineLength]{};
	char header_fields[16][64]{};
	int header_count = 0;
	bool header_parsed = false;
	int time_col = -1;
	int thrust_col = -1;
	int mass_col = -1;
	int burn_col = -1;
	int impulse_col = -1;
	slot.curve_count = 0;

	while (fgets(line, sizeof(line), fp) != nullptr) {
		const size_t len = strlen(line);

		if (len > 0 && line[len - 1] == '\n') {
			line[len - 1] = '\0';
		}

		if (len > 1 && line[len - 2] == '\r') {
			line[len - 2] = '\0';
		}

		if (line[0] == '\0') {
			continue;
		}

		if (!header_parsed) {
			if (!parse_csv_line(line, header_fields, 16, header_count)) {
				fclose(fp);
				return false;
			}

			time_col = find_column(header_fields, header_count, "time_s");
			thrust_col = find_column_any(header_fields, header_count, "thrust_N", "thrust_n");
			mass_col = find_column(header_fields, header_count, "motor_mass_kg");
			burn_col = find_column(header_fields, header_count, "burn_fraction");
			impulse_col = find_column_any(header_fields, header_count, "cumulative_impulse_Ns",
						      "cumulative_impulse_ns");

			if (time_col < 0 || thrust_col < 0 || mass_col < 0 || burn_col < 0 || impulse_col < 0) {
				PX4_ERR("motor curve header invalid in %s", path);
				fclose(fp);
				return false;
			}

			header_parsed = true;
			continue;
		}

		if (slot.curve_count >= static_cast<int>(sizeof(slot.curve) / sizeof(slot.curve[0]))) {
			break;
		}

		char fields[16][64]{};
		int field_count = 0;

		if (!parse_csv_line(line, fields, 16, field_count) || field_count <= impulse_col) {
			continue;
		}

		CurvePoint point{};

		if (!parse_float(fields[time_col], point.time_s)
		    || !parse_float(fields[thrust_col], point.thrust_n)
		    || !parse_float(fields[mass_col], point.motor_mass_kg)
		    || !parse_float(fields[burn_col], point.burn_fraction)
		    || !parse_float(fields[impulse_col], point.cumulative_impulse_ns)) {
			continue;
		}

		if (slot.curve_count > 0 && point.time_s <= slot.curve[slot.curve_count - 1].time_s) {
			PX4_ERR("non-monotonic motor curve %s", path);
			fclose(fp);
			return false;
		}

		slot.curve[slot.curve_count++] = point;
	}

	fclose(fp);
	return slot.curve_count > 0;
}

bool MotorCurveCatalog::load_specs_file(const char *path, MotorCurveSpecs &specs)
{
	char lines[4][kMaxLineLength]{};
	int line_count = 0;

	if (!read_lines(path, lines, 4, line_count) || line_count < 2) {
		PX4_ERR("failed to read motor specs %s", path);
		return false;
	}

	char header_fields[32][64]{};
	int header_count = 0;
	char value_fields[32][64]{};
	int value_count = 0;

	if (!parse_csv_line(lines[0], header_fields, 32, header_count)
	    || !parse_csv_line(lines[1], value_fields, 32, value_count)) {
		return false;
	}

	float initial_mass_g = 0.f;
	float dry_mass_g = 0.f;

	if (find_column(header_fields, header_count, "initial_mass_g") < 0
	    || find_column(header_fields, header_count, "dry_mass_g") < 0
	    || find_column(header_fields, header_count, "total_impulse_curve_Ns") < 0
	    || find_column(header_fields, header_count, "burn_time_curve_s") < 0) {
		return false;
	}

	const int initial_col = find_column(header_fields, header_count, "initial_mass_g");
	const int dry_col = find_column(header_fields, header_count, "dry_mass_g");
	const int impulse_col = find_column(header_fields, header_count, "total_impulse_curve_Ns");
	const int burn_col = find_column(header_fields, header_count, "burn_time_curve_s");

	if (!parse_float(value_fields[initial_col], initial_mass_g)
	    || !parse_float(value_fields[dry_col], dry_mass_g)
	    || !parse_float(value_fields[impulse_col], specs.total_impulse_ns)
	    || !parse_float(value_fields[burn_col], specs.burn_duration_s)) {
		return false;
	}

	specs.loaded_mass_kg = initial_mass_g * 0.001f;
	specs.dry_mass_kg = dry_mass_g * 0.001f;
	return true;
}

bool MotorCurveCatalog::load_engine_slot(int engine_index, int motor_catalog_index)
{
	if (engine_index < 0 || engine_index >= kMotorCurveMaxEngines || !_loaded) {
		return false;
	}

	EngineSlot &slot = _engines[engine_index];
	slot = EngineSlot{};
	slot.motor_index = motor_catalog_index;

	const CatalogEntry *selected = nullptr;

	for (int i = 0; i < _catalog_count; ++i) {
		if (_catalog[i].index == static_cast<uint16_t>(motor_catalog_index) && _catalog[i].active) {
			selected = &_catalog[i];
			break;
		}
	}

	if (selected == nullptr) {
		PX4_ERR("motor index %d unavailable in catalog", motor_catalog_index);
		return false;
	}

	char curve_path[192]{};
	char specs_path[192]{};
	join_path(_motor_root, selected->curve_file, curve_path, sizeof(curve_path));
	join_path(_motor_root, selected->specs_file, specs_path, sizeof(specs_path));

	if (!load_curve_file(curve_path, slot) || !load_specs_file(specs_path, slot.specs)) {
		return false;
	}

	slot.selected = *selected;
	slot.loaded = true;
	return true;
}

int MotorCurveCatalog::motor_catalog_index(int engine_index) const
{
	if (engine_index < 0 || engine_index >= kMotorCurveMaxEngines) {
		return -1;
	}

	return _engines[engine_index].motor_index;
}

const char *MotorCurveCatalog::motor_id(int engine_index) const
{
	if (engine_index < 0 || engine_index >= kMotorCurveMaxEngines) {
		return "";
	}

	return _engines[engine_index].selected.motor_id;
}

const MotorCurveSpecs &MotorCurveCatalog::specs(int engine_index) const
{
	static const MotorCurveSpecs kEmpty{};

	if (engine_index < 0 || engine_index >= kMotorCurveMaxEngines) {
		return kEmpty;
	}

	return _engines[engine_index].specs;
}

bool MotorCurveCatalog::sample(int engine_index, float burn_time_s, MotorCurveSample &out) const
{
	if (engine_index < 0 || engine_index >= kMotorCurveMaxEngines) {
		return false;
	}

	const EngineSlot &slot = _engines[engine_index];

	if (!slot.loaded || slot.curve_count == 0) {
		out = MotorCurveSample{};
		return false;
	}

	if (burn_time_s <= slot.curve[0].time_s) {
		const CurvePoint &p = slot.curve[0];
		out.thrust_n = p.thrust_n;
		out.motor_mass_kg = p.motor_mass_kg;
		out.burn_fraction = p.burn_fraction;
		out.cumulative_impulse_ns = p.cumulative_impulse_ns;
		return true;
	}

	for (int i = 1; i < slot.curve_count; ++i) {
		const CurvePoint &a = slot.curve[i - 1];
		const CurvePoint &b = slot.curve[i];

		if (burn_time_s <= b.time_s) {
			const float alpha = (burn_time_s - a.time_s) / math::max(b.time_s - a.time_s, 1e-4f);
			out.thrust_n = a.thrust_n + alpha * (b.thrust_n - a.thrust_n);
			out.motor_mass_kg = a.motor_mass_kg + alpha * (b.motor_mass_kg - a.motor_mass_kg);
			out.burn_fraction = a.burn_fraction + alpha * (b.burn_fraction - a.burn_fraction);
			out.cumulative_impulse_ns = a.cumulative_impulse_ns
						    + alpha * (b.cumulative_impulse_ns - a.cumulative_impulse_ns);
			return true;
		}
	}

	out.thrust_n = 0.f;
	out.motor_mass_kg = math::max(slot.specs.dry_mass_kg, 0.f);
	out.burn_fraction = 1.f;
	out.cumulative_impulse_ns = slot.specs.total_impulse_ns;
	return true;
}

} // namespace tv3
