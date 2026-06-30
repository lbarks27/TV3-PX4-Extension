#pragma once

#include <matrix/matrix/math.hpp>

namespace tv3
{

struct AttitudeControllerConfig {
	float ld_rad{0.05f};
	float pos_kp{3.f};
	float vel_kp{2.f};
	float vel_ki{0.f};
	float vel_kd{0.003f};
	float soften{1.f};
	float max_stopping_time_s{2.f};
	float min_flip_time_s{0.5f};
	float roll_control_range_rad{0.5f};
	float deadband_rad{0.01f};
	float large_error_rad{0.15f};
	float integrator_limit{5.f};
	float moi_roll_kgm2{0.1f};
	float moi_pitch_kgm2{0.1f};
	float moi_yaw_kgm2{0.01f};
	float torque_roll_max{8.f};
	float torque_pitch_max{16.f};
	float torque_yaw_max{16.f};
};

enum class AttitudeControllerRegion {
	Off,
	Deadband,
	LargeError,
	SmallError,
};

class AttitudeController {
public:
	void reset();

	void set_config(const AttitudeControllerConfig &config) { _config = config; }

	matrix::Vector3f attitude_error(const matrix::Quatf &q_meas, const matrix::Quatf &q_sp) const;

	AttitudeControllerRegion region_for_error(const matrix::Vector3f &att_error) const;

	matrix::Vector3f update(AttitudeControllerRegion region,
				const matrix::Quatf &q_meas,
				const matrix::Quatf &q_sp,
				const matrix::Vector3f &rate_meas,
				float dt_s);

private:
	matrix::Vector3f quaternion_attitude_error(const matrix::Quatf &q_error) const;

	matrix::Vector3f position_loop(const matrix::Vector3f &att_error, AttitudeControllerRegion region) const;

	matrix::Vector3f velocity_loop(const matrix::Vector3f &rate_sp, const matrix::Vector3f &rate_meas,
					 const matrix::Vector3f &att_error, float dt_s);

	matrix::Vector3f torque_from_alpha(const matrix::Vector3f &alpha_target) const;

	float max_alpha_rad_s2(int axis) const;

	AttitudeControllerConfig _config{};
	matrix::Vector3f _integrator{};
};

} // namespace tv3
