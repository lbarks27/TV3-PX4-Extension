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
#include <uORB/topics/tv3_engine_command.h>
#include <uORB/topics/tv3_engine_state.h>
#include <uORB/topics/tv3_gimbal_command.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

using namespace time_literals;
using matrix::AxisAnglef;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector;
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

struct EngineGroup {
	Vector3f position{};
	Vector3f thrust_axis{1.f, 0.f, 0.f};
	Vector3f pitch_axis{0.f, -1.f, 0.f};
	Vector3f yaw_axis{0.f, 0.f, -1.f};
	float thrust_fraction{1.f};
	float pitch_trim{0.f};
	float yaw_trim{0.f};
	float pitch_max_rad{math::radians(5.f)};
	float yaw_max_rad{math::radians(5.f)};
};

static Vector3f normalize_or_default(const Vector3f &v, const Vector3f &def)
{
	const float n = v.norm();
	if (n > 1e-6f) {
		return v / n;
	}
	return def;
}
}

class Tv3Sih : public ModuleBase<Tv3Sih>, public ModuleParams
{
public:
	Tv3Sih() :
		ModuleParams(nullptr)
	{
		_px4_accel.set_temperature(15.0f);
		_px4_gyro.set_temperature(15.0f);
		update_parameters();
	}

	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("tv3_sih",
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

	static Tv3Sih *instantiate(int argc, char *argv[])
	{
		Tv3Sih *instance = new Tv3Sih();

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

		PRINT_MODULE_DESCRIPTION("Deterministic TV3 tv3 Simulation-In-Hardware dynamics. "
		"6DOF rigid-body plant driven by per-engine gimbaled thrust vectors (using CA_RK geometry), "
		"variable mass, COM migration, and diagonal inertia from manifests.");
		PRINT_MODULE_USAGE_NAME("tv3_sih", "simulation");
		PRINT_MODULE_USAGE_COMMAND("start");
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("pos NED %.2f %.2f %.2f vel %.2f %.2f %.2f",
			 (double)_position(0), (double)_position(1), (double)_position(2),
			 (double)_velocity(0), (double)_velocity(1), (double)_velocity(2));
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
		PX4_INFO("TV3 SIH lockstep loop at %.1f Hz, %.1fx speed", (double)(1e6f / sim_interval_us),
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
		_engine_command_sub.update(&_engine_command);

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
		_motor_com_x_m = get_param_float("RK_MOTOR_COM_X_M", get_param_float("CA_RK_MOT_CMX", 0.0f));
		_rail_length_m = get_param_float("RK_RAIL_LEN_M", 0.0f);

		_home_lat_deg = get_param_float("SIH_LOC_LAT0", 47.397742f);
		_home_lon_deg = get_param_float("SIH_LOC_LON0", 8.545594f);
		_home_alt_m = get_param_float("SIH_LOC_H0", 488.0f);

		// Load per-group geometry (positions relative to reference, axes, fractions, trims).
		// These are the same CA_RK_G* params used by ActuatorEffectivenessTV3.
		int32_t grp_cnt = get_param_int32("CA_RK_GRP_CNT", 0);
		_num_groups = math::constrain(grp_cnt, 0, (int32_t)tv3_engine_command_s::MAX_ENGINES);

		for (int i = 0; i < _num_groups; ++i) {
			char buf[32];
			auto getg = [&](const char *suffix, float def) -> float {
				snprintf(buf, sizeof(buf), "CA_RK_G%d_%s", i, suffix);
				return get_param_float(buf, def);
			};

			_groups[i].position(0) = getg("PX", 0.f);
			_groups[i].position(1) = getg("PY", 0.f);
			_groups[i].position(2) = getg("PZ", 0.f);

			Vector3f tax{getg("AX", 1.f), getg("AY", 0.f), getg("AZ", 0.f)};
			Vector3f pax{getg("PAX", 0.f), getg("PAY", -1.f), getg("PAZ", 0.f)};
			Vector3f yax{getg("YAX", 0.f), getg("YAY", 0.f), getg("YAZ", -1.f)};

			_groups[i].thrust_axis = normalize_or_default(tax, Vector3f{1.f, 0.f, 0.f});
			_groups[i].pitch_axis  = normalize_or_default(pax, Vector3f{0.f, -1.f, 0.f});
			_groups[i].yaw_axis    = normalize_or_default(yax, Vector3f{0.f, 0.f, -1.f});

			_groups[i].thrust_fraction = getg("TF", _num_groups > 0 ? 1.f / _num_groups : 1.f);
			_groups[i].pitch_trim = getg("PTR", 0.f);
			_groups[i].yaw_trim   = getg("YTR", 0.f);
			_groups[i].pitch_max_rad = math::radians(getg("PMAX", 5.f));
			_groups[i].yaw_max_rad = math::radians(getg("YMAX", 5.f));
		}

		// Inertia (diagonal). Allow explicit RK_I** override; otherwise pick sensible default by engine count.
		float ixx = get_param_float("RK_IXX", (_num_groups >= 3 ? 0.43f : 0.144f));
		float iyy = get_param_float("RK_IYY", (_num_groups >= 3 ? 0.43f : 0.144f));
		float izz = get_param_float("RK_IZZ", (_num_groups >= 3 ? 0.05f : 0.010f));
		_inertia_diag = Vector3f{ixx, iyy, izz};
		_inertia_inv  = Vector3f{1.f / math::max(ixx, 1e-6f),
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
		// Body mass at its COM (from RK_BODY_COM_X / CA), plus each motor's mass at its group position.
		// This produces realistic COM migration toward the nose as tail-mounted motors burn.
		float m = math::max(_body_mass_kg, 0.1f);
		Vector3f weighted{_body_com_x_m, 0.f, 0.f};
		float total = m;

		const int neng = math::min(_num_groups, (int)tv3_engine_state_s::MAX_ENGINES);

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

	void step_dynamics(float dt)
	{
		const int command_count = math::constrain(static_cast<int>(_engine_command.engine_count), 0,
							  static_cast<int>(tv3_engine_command_s::MAX_ENGINES));

		// Gimbal angles (primary roll + secondary splay/yaw) come from tv3_engine_command.
		// The plant is a pure forward dynamics model: force and torque are computed from
		// current engine thrusts (from motor model) and the instantaneous (rate-limited) gimbal angles.
		for (int i = 0; i < command_count && i < tv3_engine_command_s::MAX_ENGINES; ++i) {
			_cmd_pitch_rad[i] = _engine_command.commanded_pitch_deg[i] * kDegToRad;
			_cmd_yaw_rad[i] = _engine_command.commanded_yaw_deg[i] * kDegToRad;
		}

		for (int i = command_count; i < tv3_engine_command_s::MAX_ENGINES; ++i) {
			_cmd_pitch_rad[i] = 0.f;
			_cmd_yaw_rad[i] = 0.f;
		}

		// Basic first-order actuator / rate limit for gimbals *in the plant* (commands not applied instantly).
		// Respects RK_TVC_SLEW_DPS from the manifest (250 dps v1, ~220 lander).
		{
			float slew_dps = get_param_float("RK_TVC_SLEW_DPS", 220.f);
			float max_step = math::max(slew_dps * kDegToRad * math::max(dt, 0.001f), 0.f);
			for (int i = 0; i < tv3_engine_command_s::MAX_ENGINES; ++i) {
				_applied_pitch_rad[i] = math::constrain(_cmd_pitch_rad[i],
					_applied_pitch_rad[i] - max_step, _applied_pitch_rad[i] + max_step);
				_applied_yaw_rad[i] = math::constrain(_cmd_yaw_rad[i],
					_applied_yaw_rad[i] - max_step, _applied_yaw_rad[i] + max_step);
			}
		}

		// === New wrench-driven 6DOF plant (addresses puppet logic, mean-only, no r×F, no inertia) ===
		const float mass = vehicle_mass_kg();
		const Vector3f com_b = current_com_body();

		Vector3f engine_force_b{0.f, 0.f, 0.f};
		Vector3f engine_tau_b{0.f, 0.f, 0.f};

		const int neng = math::min(_num_groups, (int)tv3_engine_state_s::MAX_ENGINES);
		for (int i = 0; i < neng; ++i) {
			float thr = 0.f;
			if (i < tv3_engine_state_s::MAX_ENGINES) {
				thr = _engine_state.filtered_thrust_n[i];
				if (!PX4_ISFINITE(thr) || thr <= 0.f) {
					thr = _engine_state.measured_thrust_n[i];
				}
				if (!PX4_ISFINITE(thr) || thr <= 0.f) {
					thr = _engine_state.expected_thrust_n[i];
				}
				thr = math::max(thr, 0.f);
			}
			if (thr < 1e-3f) {
				continue;
			}

			const Vector3f dir_b = engine_thrust_dir_body(i);
			const Vector3f f_b = dir_b * thr;
			engine_force_b += f_b;

			const Vector3f r_b = _groups[i].position - com_b;
			engine_tau_b += r_b.cross(f_b);
		}

		// Net force and torque are produced solely by the current chamber thrusts (engine_state)
		// deflected by the commanded (slew-limited) gimbal angles. Net axial reduction for splay-throttle
		// vehicles is achieved by the upstream splay angle commands (see tv3_mode_manager collective logic).
		Vector3f force_b = engine_force_b;
		Vector3f tau_b = engine_tau_b;

		publish_gimbal_command();

		// Translational (world/NED z-down)
		// Rotate body force to world. In PX4 matrix Quatf, rotateVector(v_body) produces v_world for the attitude quat.
		Vector3f f_world = _att_q.rotateVector(force_b);
		const Vector3f g_world{0.f, 0.f, kGravityMps2};
		Vector3f a_world = f_world / math::max(mass, 0.1f) + g_world;

		// Rail constraint (kinematic lock + zero rates while on rail; use length + position as proxy for tv3_status.rail_exit)
		const bool on_rail = (_rail_length_m > 0.f) && (-_position(2) < _rail_length_m);
		if (on_rail) {
			a_world(0) = 0.f;
			a_world(1) = 0.f;
			_velocity(0) = 0.f;
			_velocity(1) = 0.f;
			_position(0) = 0.f;
			_position(1) = 0.f;
			_omega_b(0) = 0.f;
			_omega_b(1) = 0.f;
			_omega_b(2) = 0.f; // no roll/yaw on rail
		}

		_velocity += a_world * dt;
		_position += _velocity * dt;

		// Ground (simple non-penetration + velocity damping + light lateral friction for lander)
		if (_position(2) > 0.f) {
			_position(2) = 0.f;
			// vertical
			if (_velocity(2) > 0.f) {
				_velocity(2) *= 0.15f; // energy loss on impact
				_velocity(2) = 0.f;
			}
			// light ground friction / damping on horizontal
			_velocity(0) *= 0.6f;
			_velocity(1) *= 0.6f;
			// damp rates (prevents spinning on "deck")
			_omega_b *= 0.4f;
		}

		// Rotational (body)
		// tau - omega x (I omega)
		const Vector3f Iomega{_inertia_diag(0) * _omega_b(0),
				      _inertia_diag(1) * _omega_b(1),
				      _inertia_diag(2) * _omega_b(2)};
		const Vector3f omega_x_Iw = _omega_b.cross(Iomega);
		// Optional light angular rate damping (off by default). Can be enabled via RK_SIH_RATE_DAMP
		// for numerical stability during ignition spikes if needed; not part of the nominal physical model.
		const float rate_damp_nm = get_param_float("RK_SIH_RATE_DAMP", 0.f);
		Vector3f tau_net = tau_b - omega_x_Iw;

		if (!on_rail && rate_damp_nm > 0.f) {
			tau_net -= _omega_b * rate_damp_nm;
		}

		const Vector3f alpha_b{_inertia_inv(0) * tau_net(0),
				       _inertia_inv(1) * tau_net(1),
				       _inertia_inv(2) * tau_net(2)};

		_omega_b += alpha_b * dt;

		// Attitude integration (quat)
		// \dot q = 1/2 q \otimes [0, omega]
		Quatf omega_q(0.f, _omega_b(0), _omega_b(1), _omega_b(2));
		Quatf qdot = _att_q * omega_q * 0.5f;
		_att_q = _att_q + (qdot * dt);
		_att_q.normalize();

		// Sync legacy state for publish / old consumers
		_angular_velocity = _omega_b;
		Eulerf e(_att_q);
		_euler(0) = e(0);
		_euler(1) = e(1);
		_euler(2) = e(2);

		// Specific force in body (non-grav accel felt by IMU = net thrust accel in body)
		_specific_force = force_b / math::max(mass, 0.1f);
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

		vehicle_angular_velocity_s angular_velocity{};
		angular_velocity.timestamp = now;
		angular_velocity.timestamp_sample = now;
		_angular_velocity.copyTo(angular_velocity.xyz);
		_angular_velocity_groundtruth_pub.publish(angular_velocity);
		_angular_velocity_pub.publish(angular_velocity);
		_px4_accel.update(now, _specific_force(0), _specific_force(1), _specific_force(2));
		_px4_gyro.update(now, _angular_velocity(0), _angular_velocity(1), _angular_velocity(2));

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
	float _motor_com_x_m{0.f};
	float _rail_length_m{0.f};
	float _home_lat_deg{47.397742f};
	float _home_lon_deg{8.545594f};
	float _home_alt_m{488.f};
	Vector3f _position{};
		Vector3f _velocity{};
		Vector3f _angular_velocity{};
		Vector3f _specific_force{0.f, 0.f, -kGravityMps2};
		Vector3f _euler{};
	tv3_engine_state_s _engine_state{};
	tv3_engine_command_s _engine_command{};

	// Proper 6DOF state (replaces puppet euler/attitude logic)
	Quatf _att_q{AxisAnglef(Vector3f{0.f, 1.f, 0.f}, M_PI_F * 0.5f)};
	Vector3f _omega_b{};   // body rates

	// Diagonal inertia (body frame). Defaults chosen from manifest physical_model for the vehicles.
	Vector3f _inertia_diag{0.144f, 0.144f, 0.010f}; // tv3_v1 baseline; lander ~0.43/0.43/0.05
	Vector3f _inertia_inv{1.f/0.144f, 1.f/0.144f, 1.f/0.010f};

	// Geometry loaded from CA_RK_* params (matches allocator effectiveness model)
	EngineGroup _groups[tv3_engine_command_s::MAX_ENGINES]{};
	int _num_groups{0};

	// Latest per-engine gimbal commands (from TV3EngineCommand). These are the "desired".
	float _cmd_pitch_rad[tv3_engine_command_s::MAX_ENGINES]{};
	float _cmd_yaw_rad[tv3_engine_command_s::MAX_ENGINES]{};


	// Actually applied (rate-limited) angles used for thrust direction this tick. Provides basic actuator dynamics in plant.
	float _applied_pitch_rad[tv3_engine_command_s::MAX_ENGINES]{};
	float _applied_yaw_rad[tv3_engine_command_s::MAX_ENGINES]{};

	Vector3f engine_thrust_dir_body_angles(int i, float pitch_rad, float yaw_rad) const
	{
		if (i < 0 || i >= _num_groups) {
			return Vector3f{1.f, 0.f, 0.f};
		}

		const EngineGroup &g = _groups[i];
		const float p = pitch_rad + g.pitch_trim;
		const float y = yaw_rad + g.yaw_trim;
		Vector3f d = g.thrust_axis;

		if (fabsf(p) > 1e-6f) {
			Quatf qp(AxisAnglef(g.pitch_axis, p));
			d = qp.rotateVector(d);
		}

		if (fabsf(y) > 1e-6f) {
			Vector3f yaw_axis = g.yaw_axis;

			if (fabsf(p) > 1e-6f) {
				Quatf qp_axis(AxisAnglef(g.pitch_axis, p));
				yaw_axis = qp_axis.rotateVector(yaw_axis);
			}

			Quatf qy(AxisAnglef(yaw_axis, y));
			d = qy.rotateVector(d);
		}

		const float n = d.norm();

		if (n > 1e-6f) {
			d /= n;
		} else {
			d = g.thrust_axis;
		}

		return d;
	}

	Vector3f engine_thrust_dir_body(int i) const
	{
		const float p = (_applied_pitch_rad[i] != 0.f ? _applied_pitch_rad[i] : _cmd_pitch_rad[i]);
		const float y = (_applied_yaw_rad[i] != 0.f ? _applied_yaw_rad[i] : _cmd_yaw_rad[i]);
		return engine_thrust_dir_body_angles(i, p, y);
	}

	void publish_gimbal_command()
	{
		tv3_gimbal_command_s gimbal{};
		gimbal.timestamp = hrt_absolute_time();
		gimbal.engine_count = static_cast<uint8_t>(_num_groups);

		for (int i = 0; i < _num_groups && i < tv3_gimbal_command_s::MAX_ENGINES; ++i) {
			const float pitch_rad = (_applied_pitch_rad[i] != 0.f ? _applied_pitch_rad[i] : _cmd_pitch_rad[i]);
			const float yaw_rad = (_applied_yaw_rad[i] != 0.f ? _applied_yaw_rad[i] : _cmd_yaw_rad[i]);
			gimbal.commanded_pitch_deg[i] = pitch_rad / kDegToRad;
			gimbal.commanded_yaw_deg[i] = yaw_rad / kDegToRad;
		}

		_gimbal_command_pub.publish(gimbal);
	}

	PX4Accelerometer _px4_accel{1310988};
	PX4Gyroscope _px4_gyro{1310988};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _engine_state_sub{ORB_ID(tv3_engine_state)};
	uORB::Subscription _engine_command_sub{ORB_ID(tv3_engine_command)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_attitude_s> _attitude_groundtruth_pub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_groundtruth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_local_position_s> _local_position_groundtruth_pub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_global_position_s> _global_position_groundtruth_pub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Publication<manual_control_setpoint_s> _manual_control_pub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<tv3_gimbal_command_s> _gimbal_command_pub{ORB_ID(tv3_gimbal_command)};
};

extern "C" __EXPORT int tv3_sih_main(int argc, char *argv[])
{
	return Tv3Sih::main(argc, argv);
}
