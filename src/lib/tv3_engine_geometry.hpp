#pragma once

#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>

#include <matrix/matrix/math.hpp>

#include <cstdio>

namespace tv3
{

constexpr int kMaxEngineGroups = 4;

struct EngineMountGeometry {
	matrix::Vector3f position{};
	matrix::Vector3f thrust_axis{1.f, 0.f, 0.f};
	matrix::Vector3f pitch_axis{0.f, -1.f, 0.f};
	matrix::Vector3f yaw_axis{0.f, 0.f, -1.f};
	float thrust_fraction{1.f};
	float pitch_trim{0.f};
	float yaw_trim{0.f};
	float pitch_max_rad{math::radians(5.f)};
	float yaw_min_rad{0.f};
	float yaw_max_rad{math::radians(5.f)};
};

struct EngineGeometry {
	int engine_count{1};
	EngineMountGeometry engines[kMaxEngineGroups]{};
};

inline float read_param_float(const char *name, float fallback)
{
	param_t handle = param_find(name);

	if (handle != PARAM_INVALID) {
		param_get(handle, &fallback);
	}

	return fallback;
}

inline matrix::Vector3f normalize_or_default(const matrix::Vector3f &v, const matrix::Vector3f &fallback)
{
	const float n = v.norm();

	if (n > 1e-4f) {
		return v / n;
	}

	return fallback;
}

inline void load_engine_geometry(EngineGeometry &geometry, float tvc_max_deg = 5.f, float splay_max_deg = 5.f)
{
	int32_t engine_count = geometry.engine_count;
	param_t count_handle = param_find("RK_ENG_COUNT");

	if (count_handle != PARAM_INVALID) {
		param_get(count_handle, &engine_count);
	}

	geometry.engine_count = math::constrain(engine_count, static_cast<int32_t>(1), static_cast<int32_t>(kMaxEngineGroups));

	const float yaw_limit_deg = math::max(math::max(splay_max_deg, tvc_max_deg), 0.f);

	for (int i = 0; i < kMaxEngineGroups; ++i) {
		char buf[32];

		auto gf = [&](const char *suffix, float def) -> float {
			snprintf(buf, sizeof(buf), "RK_G%d_%s", i, suffix);
			return read_param_float(buf, def);
		};

		EngineMountGeometry &engine = geometry.engines[i];
		engine.position(0) = gf("PX", 0.f);
		engine.position(1) = gf("PY", 0.f);
		engine.position(2) = gf("PZ", 0.f);
		engine.thrust_axis = normalize_or_default(
					     matrix::Vector3f{gf("AX", 1.f), gf("AY", 0.f), gf("AZ", 0.f)},
					     matrix::Vector3f{1.f, 0.f, 0.f});
		engine.pitch_axis = normalize_or_default(
					    matrix::Vector3f{gf("PAX", 0.f), gf("PAY", -1.f), gf("PAZ", 0.f)},
					    matrix::Vector3f{0.f, -1.f, 0.f});
		engine.yaw_axis = normalize_or_default(
					  matrix::Vector3f{gf("YAX", 0.f), gf("YAY", 0.f), gf("YAZ", -1.f)},
					  matrix::Vector3f{0.f, 0.f, -1.f});
		engine.thrust_fraction = gf("TF", geometry.engine_count > 0 ? 1.f / geometry.engine_count : 1.f);
		engine.pitch_trim = gf("PTR", 0.f);
		engine.yaw_trim = gf("YTR", 0.f);
		engine.pitch_max_rad = math::radians(gf("PMAX", 5.f));
		engine.yaw_min_rad = math::radians(gf("YMIN", 0.f));
		engine.yaw_max_rad = math::radians(gf("YMAX", 5.f));

		if (tvc_max_deg > 0.f) {
			engine.pitch_max_rad = math::min(engine.pitch_max_rad, math::radians(tvc_max_deg));
		}

		if (yaw_limit_deg > 0.f) {
			engine.yaw_max_rad = math::min(engine.yaw_max_rad, math::radians(yaw_limit_deg));
		}
	}
}

} // namespace tv3
