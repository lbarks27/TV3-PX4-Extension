#pragma once

#include <cstdint>

#include <matrix/matrix/math.hpp>

namespace tv3
{

static constexpr int kMaxEngines = 4;

struct EngineGeometry {
	matrix::Vector3f position{};
	matrix::Vector3f thrust_axis{1.f, 0.f, 0.f};
	matrix::Vector3f primary_axis{0.f, -1.f, 0.f};
	matrix::Vector3f secondary_axis{0.f, 0.f, -1.f};
	float pitch_max_rad{0.f};
	float yaw_min_rad{0.f};
	float yaw_max_rad{0.f};
	float thrust_fraction{1.f};
};

struct AllocationInput {
	int engine_count{kMaxEngines};
	uint8_t ignition_mask{0};
	float chamber_thrust_n[kMaxEngines]{};
	matrix::Vector3f desired_torque_nm{};
	float desired_thrust_n{0.f};
	EngineGeometry geometry[kMaxEngines]{};
	float warm_start_pitch_rad[kMaxEngines]{};
	float warm_start_yaw_rad[kMaxEngines]{};
	bool have_warm_start{false};
};

struct AllocationOutput {
	float pitch_rad[kMaxEngines]{};
	float yaw_rad[kMaxEngines]{};
	float best_score{0.f};
	matrix::Vector3f achieved_torque{};
	float achieved_thrust_n{0.f};
};

float engine_chamber_thrust_n(float filtered, float measured, float expected);

bool allocate_projected_gradient(const AllocationInput &input, AllocationOutput &output);

} // namespace tv3
