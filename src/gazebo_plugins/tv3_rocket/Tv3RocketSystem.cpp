#include "Tv3RocketSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Transparency.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>

using namespace tv3;

GZ_ADD_PLUGIN(
	Tv3RocketSystem,
	gz::sim::System,
	Tv3RocketSystem::ISystemConfigure,
	Tv3RocketSystem::ISystemPreUpdate
)

gz::math::Vector3d Tv3RocketSystem::ParseVector(const std::string &value,
		const gz::math::Vector3d &fallback)
{
	std::istringstream stream(value);
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;

	if (stream >> x >> y >> z) {
		return {x, y, z};
	}

	return fallback;
}

gz::math::Quaterniond Tv3RocketSystem::RotationFromXAxis(const gz::math::Vector3d &axis)
{
	const gz::math::Vector3d normalized = axis.SquaredLength() > 1e-12 ? axis.Normalized() : gz::math::Vector3d{1.0, 0.0, 0.0};
	gz::math::Quaterniond rotation;
	rotation.SetFrom2Axes({1.0, 0.0, 0.0}, normalized);
	return rotation;
}

gz::math::Quaterniond Tv3RocketSystem::RotationFromZAxis(const gz::math::Vector3d &axis)
{
	const gz::math::Vector3d normalized = axis.SquaredLength() > 1e-12 ? axis.Normalized() : gz::math::Vector3d{0.0, 0.0, 1.0};
	gz::math::Quaterniond rotation;
	rotation.SetFrom2Axes({0.0, 0.0, 1.0}, normalized);
	return rotation;
}

void Tv3RocketSystem::Configure(const gz::sim::Entity &_entity,
				const std::shared_ptr<const sdf::Element> &_sdf,
				gz::sim::EntityComponentManager &_ecm,
				gz::sim::EventManager &)
{
	_model = gz::sim::Model(_entity);

	if (_sdf->HasElement("base_link")) {
		_base_link_name = _sdf->Get<std::string>("base_link");
	}

	if (_sdf->HasElement("reference_thrust_n")) {
		_reference_thrust_n = _sdf->Get<double>("reference_thrust_n");
	}

	if (_sdf->HasElement("thrust_axis_frame")) {
		_thrust_axes_in_world = _sdf->Get<std::string>("thrust_axis_frame") == "world";
	}

		if (_sdf->HasElement("apply_engine_torques")) {
			_apply_engine_torques = _sdf->Get<bool>("apply_engine_torques");
		}

		if (_sdf->HasElement("commanded_thrust")) {
			_commanded_thrust = _sdf->Get<bool>("commanded_thrust");
		}

		if (_sdf->HasElement("command_timeout_s")) {
			_command_timeout_s = std::max(_sdf->Get<double>("command_timeout_s"), 0.0);
		}

		if (_sdf->HasElement("command_scale")) {
			_command_scale = _sdf->Get<double>("command_scale");
		}

		if (_sdf->HasElement("force_application_m")) {
			_force_application_m = ParseVector(_sdf->Get<std::string>("force_application_m"), _force_application_m);
		}

	if (_sdf->HasElement("ignition_delay_s")) {
		_ignition_delay_s = std::max(_sdf->Get<double>("ignition_delay_s"), 0.0);
	}

	if (_sdf->HasElement("ignition_dwell_ms")) {
		_ignition_dwell_s = _sdf->Get<double>("ignition_dwell_ms") * 0.001;
	}

	if (_sdf->HasElement("burn_duration_s")) {
		_burn_duration_s = std::max(_sdf->Get<double>("burn_duration_s"), 0.0);
	}

	if (_sdf->HasElement("splay_max_deg")) {
		_splay_max_deg = _sdf->Get<double>("splay_max_deg");
	}

	if (_sdf->HasElement("ignition_sequence")) {
		std::istringstream sequence_stream(_sdf->Get<std::string>("ignition_sequence"));
		int engine_index = 0;
		while (sequence_stream >> engine_index) {
			_ignition_sequence.push_back(engine_index);
		}
	}

	for (auto engine_elem = _sdf->GetFirstElement(); engine_elem; engine_elem = engine_elem->GetNextElement()) {
		if (engine_elem->GetName() != "engine") {
			continue;
		}

		Engine engine{};
		engine.index = engine_elem->Get<int>("index", static_cast<int>(_engines.size())).first;
		engine.id = engine_elem->Get<std::string>("id", "engine").first;
		engine.position_m = ParseVector(engine_elem->Get<std::string>("position_m", "0 0 0").first, engine.position_m);
		engine.thrust_axis = ParseVector(engine_elem->Get<std::string>("thrust_axis", "1 0 0").first, engine.thrust_axis).Normalized();
		engine.pitch_axis = ParseVector(engine_elem->Get<std::string>("pitch_axis", "0 -1 0").first, engine.pitch_axis).Normalized();
		engine.yaw_axis = ParseVector(engine_elem->Get<std::string>("yaw_axis", "0 0 -1").first, engine.yaw_axis).Normalized();
		engine.thrust_fraction = engine_elem->Get<double>("thrust_fraction", 1.0).first;
		engine.pitch_max_deg = engine_elem->Get<double>("pitch_max_deg", 5.0).first;
		engine.yaw_max_deg = engine_elem->Get<double>("yaw_max_deg", 5.0).first;
		engine.splay_max_deg = engine_elem->Get<double>("splay_max_deg", _splay_max_deg).first;
		engine.slew_dps = engine_elem->Get<double>("slew_dps", 250.0).first;
		_engines.push_back(engine);
	}

		if (_ignition_sequence.empty()) {
			for (const Engine &engine : _engines) {
				_ignition_sequence.push_back(engine.index);
			}
		}

		_commanded_thrust_n.assign(_engines.size(), 0.0);
		_commanded_pitch_deg.assign(_engines.size(), 0.0);
		_commanded_yaw_deg.assign(_engines.size(), 0.0);
		_commanded_splay_deg.assign(_engines.size(), 0.0);

		if (_sdf->HasElement("command_topic")) {
			_command_topic = _sdf->Get<std::string>("command_topic");
		} else {
			_command_topic = "/" + _model.Name(_ecm) + "/command/rocket_thrust";
		}

		if (!_node.Subscribe(_command_topic, &Tv3RocketSystem::CommandCallback, this)) {
			std::cerr << "TV3 Rocket: failed to subscribe to " << _command_topic << std::endl;
		}

		const auto link_entity = _model.LinkByName(_ecm, _base_link_name);
		_link = gz::sim::Link(link_entity);
		ResolveEngineVisuals(_ecm);
	}

	void Tv3RocketSystem::CommandCallback(const gz::msgs::Actuators &msg)
	{
		std::lock_guard<std::mutex> lock(_command_mutex);
		const int count = static_cast<int>(_commanded_thrust_n.size());
		const int thrust_count = std::min<int>(msg.velocity_size(), count);

		for (double &thrust_n : _commanded_thrust_n) {
			thrust_n = 0.0;
		}
		for (double &pitch_deg : _commanded_pitch_deg) {
			pitch_deg = 0.0;
		}
		for (double &yaw_deg : _commanded_yaw_deg) {
			yaw_deg = 0.0;
		}
		for (double &splay_deg : _commanded_splay_deg) {
			splay_deg = 0.0;
		}

		for (int i = 0; i < thrust_count; ++i) {
			const double value = msg.velocity(i);
			_commanded_thrust_n[i] = std::isfinite(value) ? std::max(value * _command_scale, 0.0) : 0.0;
		}

		if (msg.position_size() >= count * 3) {
			for (int i = 0; i < count; ++i) {
				const double pitch = msg.position(i);
				const double yaw = msg.position(count + i);
				const double splay = msg.position((count * 2) + i);
				_commanded_pitch_deg[i] = std::isfinite(pitch) ? pitch : 0.0;
				_commanded_yaw_deg[i] = std::isfinite(yaw) ? yaw : 0.0;
				_commanded_splay_deg[i] = std::isfinite(splay) ? splay : 0.0;
			}
		}

		if (msg.header().has_stamp()) {
			_last_command_time_s = static_cast<double>(msg.header().stamp().sec())
				+ static_cast<double>(msg.header().stamp().nsec()) * 1e-9;
		} else {
			_last_command_time_s = std::numeric_limits<double>::quiet_NaN();
		}
	}

void Tv3RocketSystem::ResolveEngineVisuals(gz::sim::EntityComponentManager &_ecm)
{
	if (_link.Entity() == gz::sim::kNullEntity) {
		return;
	}

	for (Engine &engine : _engines) {
		engine.nozzle_visual = _link.VisualByName(_ecm, "engine_nozzle_" + std::to_string(engine.index));
		engine.thrust_visual = _link.VisualByName(_ecm, "thrust_cue_" + std::to_string(engine.index));
	}
}

void Tv3RocketSystem::UpdateEngineVisuals(gz::sim::EntityComponentManager &_ecm,
		const std::vector<double> &thrust_n,
		const std::vector<double> &pitch_deg,
		const std::vector<double> &yaw_deg,
		const std::vector<double> &splay_deg)
{
	const double reference = _reference_thrust_n > 1e-6 ? _reference_thrust_n : 1.0;

	for (const Engine &engine : _engines) {
		const int index = engine.index;
		if (index < 0 || index >= static_cast<int>(thrust_n.size())) {
			continue;
		}

		const double pitch_rad = index < static_cast<int>(pitch_deg.size()) ? pitch_deg[index] * M_PI / 180.0 : 0.0;
		const double yaw_rad = index < static_cast<int>(yaw_deg.size()) ? yaw_deg[index] * M_PI / 180.0 : 0.0;
		const double splay_rad = index < static_cast<int>(splay_deg.size()) ? splay_deg[index] * M_PI / 180.0 : 0.0;

		const gz::math::Quaterniond pitch_rotation(engine.pitch_axis, pitch_rad);
		const gz::math::Quaterniond yaw_rotation(engine.yaw_axis, yaw_rad + splay_rad);
		const gz::math::Vector3d animated_axis = yaw_rotation.RotateVector(pitch_rotation.RotateVector(engine.thrust_axis)).Normalized();
		const gz::math::Quaterniond nozzle_rotation = RotationFromXAxis(animated_axis);
		const gz::math::Quaterniond thrust_cue_rotation = RotationFromZAxis(animated_axis);

		if (engine.nozzle_visual != gz::sim::kNullEntity) {
			_ecm.SetComponentData<gz::sim::components::Pose>(engine.nozzle_visual, gz::math::Pose3d(engine.position_m, nozzle_rotation));
		}

		if (engine.thrust_visual != gz::sim::kNullEntity) {
			const double clamped = std::max(thrust_n[index], 0.0);
			const double normalized = std::min(clamped / reference, 1.0);
			const double length = clamped > 1e-6
				? _thrust_visual_min_length_m + normalized * (_thrust_visual_max_length_m - _thrust_visual_min_length_m)
				: _thrust_visual_min_length_m;
			const gz::math::Vector3d center = engine.position_m + animated_axis * (length * 0.5);

			sdf::Cylinder cylinder;
			cylinder.SetRadius(_thrust_visual_radius_m);
			cylinder.SetLength(length);
			sdf::Geometry geometry;
			geometry.SetType(sdf::GeometryType::CYLINDER);
			geometry.SetCylinderShape(cylinder);

			_ecm.SetComponentData<gz::sim::components::Pose>(engine.thrust_visual, gz::math::Pose3d(center, thrust_cue_rotation));
			_ecm.SetComponentData<gz::sim::components::Geometry>(engine.thrust_visual, geometry);
			_ecm.SetComponentData<gz::sim::components::Transparency>(engine.thrust_visual, clamped > 1e-6 ? 0.35f : 1.0f);
		}
	}
}

void Tv3RocketSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
				gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused || _link.Entity() == gz::sim::kNullEntity || _engines.empty()) {
		return;
	}

	const double sim_time_s = std::chrono::duration<double>(_info.simTime).count();

	if (_reference_start_time_s < 0.0 || sim_time_s < _reference_start_time_s) {
		_reference_start_time_s = sim_time_s;
	}

		const double elapsed_s = sim_time_s - _reference_start_time_s;
		std::vector<double> commanded_thrust_n;
		std::vector<double> commanded_pitch_deg;
		std::vector<double> commanded_yaw_deg;
		std::vector<double> commanded_splay_deg;
		bool command_fresh = false;

		if (_commanded_thrust) {
			std::lock_guard<std::mutex> lock(_command_mutex);
			commanded_thrust_n = _commanded_thrust_n;
			commanded_pitch_deg = _commanded_pitch_deg;
			commanded_yaw_deg = _commanded_yaw_deg;
			commanded_splay_deg = _commanded_splay_deg;
			command_fresh = std::isnan(_last_command_time_s) || (_last_command_time_s >= 0.0
					&& (sim_time_s - _last_command_time_s) <= _command_timeout_s);

			if (!command_fresh) {
				std::vector<double> zero_thrust(_engines.size(), 0.0);
				UpdateEngineVisuals(_ecm, zero_thrust, commanded_pitch_deg, commanded_yaw_deg, commanded_splay_deg);
				return;
			}

		} else {
			commanded_pitch_deg.assign(_engines.size(), 0.0);
			commanded_yaw_deg.assign(_engines.size(), 0.0);
			commanded_splay_deg.assign(_engines.size(), 0.0);
			if (elapsed_s < _ignition_delay_s) {
				std::vector<double> zero_thrust(_engines.size(), 0.0);
				UpdateEngineVisuals(_ecm, zero_thrust, commanded_pitch_deg, commanded_yaw_deg, commanded_splay_deg);
				return;
			}

			const double burn_time_s = elapsed_s - _ignition_delay_s;

			if (_burn_duration_s > 0.0 && burn_time_s > _burn_duration_s) {
				std::vector<double> zero_thrust(_engines.size(), 0.0);
				UpdateEngineVisuals(_ecm, zero_thrust, commanded_pitch_deg, commanded_yaw_deg, commanded_splay_deg);
				return;
			}
		}

	const auto pose = _link.WorldPose(_ecm);

	if (!pose) {
		return;
	}

	gz::math::Vector3d total_force{0.0, 0.0, 0.0};
	gz::math::Vector3d total_torque{0.0, 0.0, 0.0};
	std::vector<double> visual_thrust_n(_engines.size(), 0.0);

		for (size_t slot = 0; slot < _ignition_sequence.size(); ++slot) {
			const int engine_index = _ignition_sequence[slot];
			auto it = std::find_if(_engines.begin(), _engines.end(), [&](const Engine &engine) {
				return engine.index == engine_index;
			});

		if (it == _engines.end()) {
			continue;
		}

			const Engine &engine = *it;
			const double splay_rad = engine.splay_max_deg * M_PI / 180.0;
			double thrust_n = 0.0;

			if (_commanded_thrust) {
				if (engine.index >= 0 && engine.index < static_cast<int>(commanded_thrust_n.size())) {
					thrust_n = commanded_thrust_n[engine.index] * std::cos(splay_rad);
				}

			} else {
				const double burn_time_s = elapsed_s - _ignition_delay_s;

				if (burn_time_s < static_cast<double>(slot) * _ignition_dwell_s) {
					continue;
				}

				thrust_n = _reference_thrust_n * engine.thrust_fraction * std::cos(splay_rad);
			}

			if (engine.index >= 0 && engine.index < static_cast<int>(visual_thrust_n.size())) {
				visual_thrust_n[engine.index] = thrust_n;
			}

			const gz::math::Vector3d force_world = _thrust_axes_in_world
				? engine.thrust_axis * thrust_n
				: pose->Rot().RotateVector(engine.thrust_axis * thrust_n);

		total_force += force_world;

		if (_apply_engine_torques) {
			const gz::math::Vector3d lever_world = pose->Rot().RotateVector(engine.position_m);
			total_torque += lever_world.Cross(force_world);
		} else {
			const gz::math::Vector3d application_world = pose->Rot().RotateVector(_force_application_m);
			total_torque += application_world.Cross(force_world);
		}
	}

	if (total_force.SquaredLength() > 1e-9 || total_torque.SquaredLength() > 1e-9) {
		_link.AddWorldWrench(_ecm, total_force, total_torque);
	}

	UpdateEngineVisuals(_ecm, visual_thrust_n, commanded_pitch_deg, commanded_yaw_deg, commanded_splay_deg);
}
