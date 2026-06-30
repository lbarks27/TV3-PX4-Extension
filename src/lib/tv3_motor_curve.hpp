#pragma once

#include <cstdint>

namespace tv3
{

constexpr int kMotorCurveMaxEngines = 4;
constexpr int kMotorCurveMaxCatalog = 8;

struct MotorCurveSample {
	float thrust_n{0.f};
	float motor_mass_kg{0.f};
	float burn_fraction{0.f};
	float cumulative_impulse_ns{0.f};
};

struct MotorCurveSpecs {
	float loaded_mass_kg{0.f};
	float dry_mass_kg{0.f};
	float total_impulse_ns{0.f};
	float burn_duration_s{0.f};
};

class MotorCurveCatalog {
public:
	bool load(const char *motor_root);

	bool loaded() const { return _loaded; }

	bool load_engine_slot(int engine_index, int motor_catalog_index);

	int motor_catalog_index(int engine_index) const;

	const char *motor_id(int engine_index) const;

	const MotorCurveSpecs &specs(int engine_index) const;

	bool sample(int engine_index, float burn_time_s, MotorCurveSample &out) const;

private:
	struct CatalogEntry {
		uint16_t index{0};
		char motor_id[32]{};
		char curve_file[96]{};
		char specs_file[96]{};
		bool active{false};
	};

	struct CurvePoint {
		float time_s{0.f};
		float thrust_n{0.f};
		float motor_mass_kg{0.f};
		float burn_fraction{0.f};
		float cumulative_impulse_ns{0.f};
	};

	struct EngineSlot {
		int motor_index{-1};
		bool loaded{false};
		CatalogEntry selected{};
		MotorCurveSpecs specs{};
		int curve_count{0};
		CurvePoint curve[256]{};
	};

	bool load_curve_file(const char *path, EngineSlot &slot);
	bool load_specs_file(const char *path, MotorCurveSpecs &specs);

	char _motor_root[128]{"/fs/microsd/tv3/motors"};
	bool _loaded{false};
	int _catalog_count{0};
	CatalogEntry _catalog[kMotorCurveMaxCatalog]{};
	EngineSlot _engines[kMotorCurveMaxEngines]{};
};

} // namespace tv3
