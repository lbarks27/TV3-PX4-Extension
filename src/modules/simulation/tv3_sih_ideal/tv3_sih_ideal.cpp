#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tv3_engine_state.h>
#include <uORB/topics/tv3_gimbal_command.h>
#include <uORB/topics/tv3_guidance_status.h>
#include <uORB/topics/tv3_plant_wrench.h>
#include <uORB/topics/tv3_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_euler.h>
#include <uORB/topics/vehicle_attitude_groundtruth_euler.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

using namespace time_literals;
using matrix::AxisAnglef;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

namespace
{
constexpr float kGravityMps2 = 9.80665f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kEarthMetersPerDegLat = 111320.f;

static float get_param_float(const char *name, float fallback)
{
	param_t handle = param_find(name);

	if (handle == PARAM_INVALID) {
		return fallback;
	}

	float value = fallback;
	param_get(handle, &value);
	return value;
}

static int32_t get_param_int32(const char *name, int32_t fallback)
{
	param_t handle = param_find(name);

	if (handle == PARAM_INVALID) {
		return fallback;
	}

	int32_t value = fallback;
	param_get(handle, &value);
	return value;
}

static uint64_t wall_time_us()
{
	struct timeval t;
	gettimeofday(&t, nullptr);
	return t.tv_sec * 1000000ULL + t.tv_usec;
}

struct EngineMount {
	Vector3f position{};
};

static void apply_imu_dither(float &x, float &y, float &z, hrt_abstime now, uint8_t salt)
{
	constexpr float kAmp = 2.5e-4f;
	const float t = static_cast<float>((now / 2500U) + salt * 17U);
	const float w = 0.37f + 0.11f * static_cast<float>(salt & 3U);
	x += kAmp * sinf(t * w);
	y += kAmp * sinf(t * w + 2.1f);
	z += kAmp * sinf(t * w + 4.2f);
}
}

class Tv3SihIdeal : public ModuleBase<Tv3SihIdeal>, public ModuleParams
{
public:
	Tv3SihIdeal() :
		ModuleParams(nullptr)
	{
		_px4_accel.set_temperature(15.0f);
		_px4_gyro.set_temperature(15.0f);
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("tv3_sih_ideal",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_MAX,
					      1800,
					      (px4_main_t)&run_trampoline,
					      (char *const *)argv);

		if (_task_id < 0) {
			_task_id = -1;
			return -errno;
		}

		return PX4_OK;
	}

	static Tv3SihIdeal *instantiate(int argc, char *argv[])
	{
		Tv3SihIdeal *instance = new Tv3SihIdeal();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
		}

		return instance;
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

		PRINT_MODULE_DESCRIPTION(
			"Ideal TV3 SIH plant for guidance/attitude validation. Applies tv3_guidance required axial "
			"thrust and vehicle_torque_setpoint body torques directly (no mixer, allocator, or gimbal dynamics).");
		PRINT_MODULE_USAGE_NAME("tv3_sih_ideal", "simulation");
		PRINT_MODULE_USAGE_COMMAND("start");
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("ideal plant pos NED %.2f %.2f %.2f vel %.2f %.2f %.2f thrust %.1f N",
			 (double)_position(0), (double)_position(1), (double)_position(2),
			 (double)_velocity(0), (double)_velocity(1), (double)_velocity(2),
			 (double)_last_thrust_n);
		return 0;
	}

	void run() override
	{
		const int sim_interval_us = 2500;
		float speed_factor = 1.f;
		const char *speedup = getenv("PX4_SIM_SPEED_FACTOR");

		if (speedup != nullptr) {
			speed_factor = math::max(static_cast<float>(atof(speedup)), 0.1f);
		}

		const int wall_interval_us = math::max(static_cast<int>(roundf(sim_interval_us / speed_factor)), 1);
		PX4_INFO("TV3 ideal SIH lockstep loop at %.1f Hz, %.1fx speed", (double)(1e6f / sim_interval_us),
			 (double)speed_factor);

		while (!should_exit()) {
			const uint64_t loop_start_us = wall_time_us();
			_current_simulation_time_us += sim_interval_us;

			struct timespec ts;
			abstime_to_ts(&ts, _current_simulation_time_us);
			px4_clock_settime(CLOCK_MONOTONIC, &ts);

			step_once();

			const uint64_t elapsed_us = wall_time_us() - loop_start_us;

			if (elapsed_us < static_cast<uint64_t>(wall_interval_us)) {
				::usleep(static_cast<useconds_t>(wall_interval_us - elapsed_us));
			}
		}

		exit_and_cleanup();
	}

private:
	void step_once()
	{
		if (_parameter_update_sub.updated()) {
			parameter_update_s update{};
			_parameter_update_sub.copy(&update);
			update_parameters();
		}

		_engine_state_sub.update(&_engine_state);
		_tv3_status_sub.update(&_tv3_status);
		_guidance_status_sub.update(&_guidance_status);
		_torque_setpoint_sub.update(&_torque_setpoint);

		const hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain((now - _last_update) * 1e-6f, 0.001f, 0.05f);
		_last_update = now;

		step_dynamics(dt);
		publish_state(now);
	}

	void update_parameters()
	{
		_body_mass_kg = get_param_float("RK_BODY_MASS_KG", get_param_float("CA_RK_BODY_M", 1.0f));
		_body_com_x_m = get_param_float("RK_BODY_COM_X_M", get_param_float("CA_RK_BODY_CMX", 0.0f));
		_rail_length_m = get_param_float("RK_RAIL_LEN_M", 0.0f);

		_home_lat_deg = get_param_float("SIH_LOC_LAT0", 47.397742f);
		_home_lon_deg = get_param_float("SIH_LOC_LON0", 8.545594f);
		_home_alt_m = get_param_float("SIH_LOC_H0", 488.0f);

		int32_t grp_cnt = get_param_int32("CA_RK_GRP_CNT", 0);
		_num_groups = math::constrain(grp_cnt, 0, static_cast<int32_t>(tv3_engine_state_s::MAX_ENGINES));

		for (int i = 0; i < _num_groups; ++i) {
			char buf[32];
			snprintf(buf, sizeof(buf), "CA_RK_G%d_PX", i);
			_groups[i].position(0) = get_param_float(buf, 0.f);
			snprintf(buf, sizeof(buf), "CA_RK_G%d_PY", i);
			_groups[i].position(1) = get_param_float(buf, 0.f);
			snprintf(buf, sizeof(buf), "CA_RK_G%d_PZ", i);
			_groups[i].position(2) = get_param_float(buf, 0.f);
		}

		const float ixx = get_param_float("RK_IXX", (_num_groups >= 3 ? 0.43f : 0.144f));
		const float iyy = get_param_float("RK_IYY", (_num_groups >= 3 ? 0.43f : 0.144f));
		const float izz = get_param_float("RK_IZZ", (_num_groups >= 3 ? 0.05f : 0.010f));
		_inertia_diag = Vector3f{ixx, iyy, izz};
		_inertia_inv = Vector3f{1.f / math::max(ixx, 1e-6f),
				       1.f / math::max(iyy, 1e-6f),
				       1.f / math::max(izz, 1e-6f)};
	}

	float vehicle_mass_kg() const
	{
		float mass = math::max(_body_mass_kg, 0.1f);
		const int count = math::constrain(static_cast<int>(_engine_state.engine_count), 0,
						  static_cast<int>(tv3_engine_state_s::MAX_ENGINES));

		for (int i = 0; i < count; ++i) {
			if (PX4_ISFINITE(_engine_state.expected_motor_mass_kg[i])) {
				mass += math::max(_engine_state.expected_motor_mass_kg[i], 0.f);
			}
		}

		return math::max(mass, 0.1f);
	}

	Vector3f current_com_body() const
	{
		float total = math::max(_body_mass_kg, 0.1f);
		Vector3f weighted{_body_com_x_m, 0.f, 0.f};
		const int neng = math::min(_num_groups, static_cast<int>(tv3_engine_state_s::MAX_ENGINES));

		for (int i = 0; i < neng; ++i) {
			float mi = 0.f;

			if (i < tv3_engine_state_s::MAX_ENGINES && PX4_ISFINITE(_engine_state.expected_motor_mass_kg[i])) {
				mi = math::max(_engine_state.expected_motor_mass_kg[i], 0.f);
			}

			if (mi > 1e-4f) {
				weighted += _groups[i].position * mi;
				total += mi;
			}
		}

		if (total > 1e-4f) {
			return weighted / total;
		}

		return Vector3f{_body_com_x_m, 0.f, 0.f};
	}

	float ideal_axial_thrust_n() const
	{
		if (_guidance_status.timestamp == 0 || !_guidance_status.thrust_solution_valid) {
			return 0.f;
		}

		return math::max(_guidance_status.required_thrust_n, 0.f);
	}

	Vector3f ideal_body_torque_nm() const
	{
		if (_torque_setpoint.timestamp == 0) {
			return Vector3f{};
		}

		return Vector3f{_torque_setpoint.xyz[0], _torque_setpoint.xyz[1], _torque_setpoint.xyz[2]};
	}

	void step_dynamics(float dt)
	{
		const float mass = vehicle_mass_kg();
		_last_thrust_n = ideal_axial_thrust_n();
		const Vector3f force_b{_last_thrust_n, 0.f, 0.f};
		Vector3f tau_b = ideal_body_torque_nm();

		publish_gimbal_command();

		Vector3f f_world = _att_q.rotateVector(force_b);
		const Vector3f g_world{0.f, 0.f, kGravityMps2};
		Vector3f a_world = f_world / math::max(mass, 0.1f) + g_world;

		const bool powered_boost = _tv3_status.mode == tv3_status_s::MODE_BOOST
					   || _tv3_status.mode == tv3_status_s::MODE_IGNITION_PENDING;
		bool on_rail = false;

		if (_rail_length_m > 0.f) {
			if (powered_boost) {
				on_rail = !_tv3_status.rail_exit;
			} else {
				on_rail = -_position(2) < _rail_length_m;
			}
		}

		if (_prev_on_rail && !on_rail) {
			_omega_b.zero();
			_angular_velocity.zero();
			_rail_torque_scale = 0.f;
			_rail_release_us = hrt_absolute_time();
		}

		_prev_on_rail = on_rail;

		bool skip_attitude_integration = false;

		if (on_rail) {
			a_world(0) = 0.f;
			a_world(1) = 0.f;
			_velocity(0) = 0.f;
			_velocity(1) = 0.f;
			_position(0) = 0.f;
			_position(1) = 0.f;
			_omega_b(0) = 0.f;
			_omega_b(1) = 0.f;
			_omega_b(2) = 0.f;
			tau_b.zero();
			skip_attitude_integration = true;
		}

		_velocity += a_world * dt;
		_position += _velocity * dt;

		if (_position(2) > 0.f) {
			_position(2) = 0.f;

			if (_velocity(2) > 0.f) {
				_velocity(2) = 0.f;
			}

			_velocity(0) *= 0.6f;
			_velocity(1) *= 0.6f;
			_omega_b *= 0.4f;
		}

		const Vector3f Iomega{_inertia_diag(0) * _omega_b(0),
				      _inertia_diag(1) * _omega_b(1),
				      _inertia_diag(2) * _omega_b(2)};
		const Vector3f omega_x_Iw = _omega_b.cross(Iomega);
		const float rate_damp_nm = get_param_float("RK_SIH_RATE_DAMP", 0.f);
		Vector3f tau_net = tau_b - omega_x_Iw;

		if (!on_rail && rate_damp_nm > 0.f) {
			tau_net -= _omega_b * rate_damp_nm;
		}

		float rail_torque_scale = 1.f;

		if (on_rail) {
			rail_torque_scale = 0.f;
		} else if (_rail_release_us > 0) {
			const float ramp_s = get_param_float("RK_SIH_RAIL_RAMP", 0.05f);

			if (ramp_s > 1e-6f) {
				const float elapsed_s = (hrt_absolute_time() - _rail_release_us) * 1e-6f;
				rail_torque_scale = math::constrain(elapsed_s / ramp_s, 0.f, 1.f);
			}
		}

		tau_net *= rail_torque_scale;
		_rail_torque_scale = rail_torque_scale;

		publish_plant_wrench(on_rail, rail_torque_scale, force_b, tau_b, tau_net);

		const Vector3f alpha_b{_inertia_inv(0) * tau_net(0),
				       _inertia_inv(1) * tau_net(1),
				       _inertia_inv(2) * tau_net(2)};

		if (!skip_attitude_integration) {
			_omega_b += alpha_b * dt;

			Quatf omega_q(0.f, _omega_b(0), _omega_b(1), _omega_b(2));
			Quatf qdot = _att_q * omega_q * 0.5f;
			_att_q = _att_q + (qdot * dt);
			_att_q.normalize();
		} else {
			_omega_b.zero();
		}

		_angular_velocity = _omega_b;
		Eulerf e(_att_q);
		_euler(0) = e(0);
		_euler(1) = e(1);
		_euler(2) = e(2);
		_specific_force = force_b / math::max(mass, 0.1f);
	}

	void publish_gimbal_command()
	{
		tv3_gimbal_command_s gimbal{};
		gimbal.timestamp = hrt_absolute_time();
		gimbal.engine_count = static_cast<uint8_t>(_num_groups);
		_gimbal_command_pub.publish(gimbal);
	}

	void publish_plant_wrench(bool on_rail, float rail_torque_scale, const Vector3f &force_b,
				  const Vector3f &commanded_torque_b, const Vector3f &net_torque_b)
	{
		tv3_plant_wrench_s wrench{};
		wrench.timestamp = hrt_absolute_time();
		wrench.on_rail = on_rail;
		wrench.rail_torque_scale = rail_torque_scale;
		wrench.body_force_n[0] = force_b(0);
		wrench.body_force_n[1] = force_b(1);
		wrench.body_force_n[2] = force_b(2);
		wrench.engine_torque_nm[0] = commanded_torque_b(0);
		wrench.engine_torque_nm[1] = commanded_torque_b(1);
		wrench.engine_torque_nm[2] = commanded_torque_b(2);
		wrench.net_torque_nm[0] = net_torque_b(0);
		wrench.net_torque_nm[1] = net_torque_b(1);
		wrench.net_torque_nm[2] = net_torque_b(2);
		_plant_wrench_pub.publish(wrench);
	}

	void publish_state(hrt_abstime now)
	{
		Quatf q(_att_q);
		q.normalize();

		vehicle_attitude_s attitude{};
		attitude.timestamp = now;
		attitude.timestamp_sample = now;
		q.copyTo(attitude.q);
		_attitude_pub.publish(attitude);
		_attitude_groundtruth_pub.publish(attitude);

		vehicle_attitude_euler_s attitude_euler{};
		attitude_euler.timestamp = now;
		attitude_euler.timestamp_sample = now;
		attitude_euler.roll_rad = _euler(0);
		attitude_euler.pitch_rad = _euler(1);
		attitude_euler.yaw_rad = _euler(2);
		_attitude_euler_pub.publish(attitude_euler);

		vehicle_attitude_groundtruth_euler_s attitude_groundtruth_euler{};
		attitude_groundtruth_euler.timestamp = now;
		attitude_groundtruth_euler.timestamp_sample = now;
		attitude_groundtruth_euler.roll_rad = _euler(0);
		attitude_groundtruth_euler.pitch_rad = _euler(1);
		attitude_groundtruth_euler.yaw_rad = _euler(2);
		_attitude_groundtruth_euler_pub.publish(attitude_groundtruth_euler);

		vehicle_angular_velocity_s angular_velocity{};
		angular_velocity.timestamp = now;
		angular_velocity.timestamp_sample = now;
		_angular_velocity.copyTo(angular_velocity.xyz);
		_angular_velocity_groundtruth_pub.publish(angular_velocity);
		_angular_velocity_pub.publish(angular_velocity);

		float accel_x = _specific_force(0);
		float accel_y = _specific_force(1);
		float accel_z = _specific_force(2);
		float gyro_x = _angular_velocity(0);
		float gyro_y = _angular_velocity(1);
		float gyro_z = _angular_velocity(2);
		apply_imu_dither(accel_x, accel_y, accel_z, now, 1);
		apply_imu_dither(gyro_x, gyro_y, gyro_z, now, 2);
		_px4_accel.update(now, accel_x, accel_y, accel_z);
		_px4_gyro.update(now, gyro_x, gyro_y, gyro_z);

		vehicle_local_position_s local{};
		local.timestamp = now;
		local.timestamp_sample = now;
		local.ref_timestamp = now;
		local.ref_lat = _home_lat_deg;
		local.ref_lon = _home_lon_deg;
		local.ref_alt = _home_alt_m;
		local.x = _position(0);
		local.y = _position(1);
		local.z = _position(2);
		local.vx = _velocity(0);
		local.vy = _velocity(1);
		local.vz = _velocity(2);
		local.heading = _euler(2);
		local.xy_valid = true;
		local.z_valid = true;
		local.v_xy_valid = true;
		local.v_z_valid = true;
		local.xy_global = true;
		local.z_global = true;
		local.heading_good_for_control = true;
		local.eph = 0.01f;
		local.epv = 0.01f;
		local.evh = 0.01f;
		local.evv = 0.01f;
		local.dist_bottom = math::max(-_position(2), 0.f);
		local.dist_bottom_valid = true;
		_local_position_pub.publish(local);
		_local_position_groundtruth_pub.publish(local);

		const float cos_lat = math::max(cosf(_home_lat_deg * kDegToRad), 0.1f);
		vehicle_global_position_s global{};
		global.timestamp = now;
		global.timestamp_sample = now;
		global.lat = static_cast<double>(_home_lat_deg + _position(0) / kEarthMetersPerDegLat);
		global.lon = static_cast<double>(_home_lon_deg + _position(1) / (kEarthMetersPerDegLat * cos_lat));
		global.alt = _home_alt_m - _position(2);
		global.alt_ellipsoid = global.alt;
		global.eph = 0.01f;
		global.epv = 0.01f;
		global.lat_lon_valid = true;
		global.alt_valid = true;
		_global_position_pub.publish(global);
		_global_position_groundtruth_pub.publish(global);

		manual_control_setpoint_s manual{};
		manual.timestamp = now;
		manual.timestamp_sample = now;
		manual.valid = true;
		manual.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
		manual.roll = 0.f;
		manual.pitch = 0.f;
		manual.yaw = 0.f;
		manual.throttle = -1.f;
		_manual_control_pub.publish(manual);
	}

	hrt_abstime _last_update{0};
	uint64_t _current_simulation_time_us{0};
	float _body_mass_kg{1.f};
	float _body_com_x_m{0.f};
	float _rail_length_m{0.f};
	bool _prev_on_rail{false};
	hrt_abstime _rail_release_us{0};
	float _rail_torque_scale{1.f};
	float _last_thrust_n{0.f};
	float _home_lat_deg{47.397742f};
	float _home_lon_deg{8.545594f};
	float _home_alt_m{488.f};
	Vector3f _position{};
	Vector3f _velocity{};
	Vector3f _angular_velocity{};
	Vector3f _specific_force{0.f, 0.f, -kGravityMps2};
	Vector3f _euler{};
	Quatf _att_q{AxisAnglef(Vector3f{0.f, 1.f, 0.f}, M_PI_F * 0.5f)};
	Vector3f _omega_b{};
	Vector3f _inertia_diag{0.144f, 0.144f, 0.010f};
	Vector3f _inertia_inv{1.f / 0.144f, 1.f / 0.144f, 1.f / 0.010f};
	EngineMount _groups[tv3_engine_state_s::MAX_ENGINES]{};
	int _num_groups{0};
	tv3_engine_state_s _engine_state{};
	tv3_status_s _tv3_status{};
	tv3_guidance_status_s _guidance_status{};
	vehicle_torque_setpoint_s _torque_setpoint{};

	PX4Accelerometer _px4_accel{1310988};
	PX4Gyroscope _px4_gyro{1310988};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _engine_state_sub{ORB_ID(tv3_engine_state)};
	uORB::Subscription _tv3_status_sub{ORB_ID(tv3_status)};
	uORB::Subscription _guidance_status_sub{ORB_ID(tv3_guidance_status)};
	uORB::Subscription _torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};

	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_attitude_s> _attitude_groundtruth_pub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Publication<vehicle_attitude_euler_s> _attitude_euler_pub{ORB_ID(vehicle_attitude_euler)};
	uORB::Publication<vehicle_attitude_groundtruth_euler_s> _attitude_groundtruth_euler_pub{ORB_ID(vehicle_attitude_groundtruth_euler)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_groundtruth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_local_position_s> _local_position_groundtruth_pub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_global_position_s> _global_position_groundtruth_pub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Publication<manual_control_setpoint_s> _manual_control_pub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<tv3_gimbal_command_s> _gimbal_command_pub{ORB_ID(tv3_gimbal_command)};
	uORB::Publication<tv3_plant_wrench_s> _plant_wrench_pub{ORB_ID(tv3_plant_wrench)};
};

extern "C" __EXPORT int tv3_sih_ideal_main(int argc, char *argv[])
{
	return Tv3SihIdeal::main(argc, argv);
}