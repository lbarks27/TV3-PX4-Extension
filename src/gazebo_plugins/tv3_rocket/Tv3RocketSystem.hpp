#pragma once

#include <gz/msgs/actuators.pb.h>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include <mutex>
#include <string>
#include <vector>

namespace tv3
{
class Tv3RocketSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_event_mgr) final;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

	private:
		struct Engine {
		std::string id;
		int index{0};
		gz::math::Vector3d position_m{0.0, 0.0, 0.0};
		gz::math::Vector3d thrust_axis{1.0, 0.0, 0.0};
		gz::math::Vector3d pitch_axis{0.0, -1.0, 0.0};
		gz::math::Vector3d yaw_axis{0.0, 0.0, -1.0};
		double thrust_fraction{1.0};
		double pitch_max_deg{5.0};
		double yaw_max_deg{5.0};
		double splay_max_deg{5.0};
		double slew_dps{250.0};
		gz::sim::Entity nozzle_visual{gz::sim::kNullEntity};
		gz::sim::Entity thrust_visual{gz::sim::kNullEntity};
	};

		static gz::math::Vector3d ParseVector(const std::string &value,
						      const gz::math::Vector3d &fallback);
		static gz::math::Quaterniond RotationFromXAxis(const gz::math::Vector3d &axis);
		static gz::math::Quaterniond RotationFromZAxis(const gz::math::Vector3d &axis);
		void CommandCallback(const gz::msgs::Actuators &msg);
		void ResolveEngineVisuals(gz::sim::EntityComponentManager &_ecm);
		void UpdateEngineVisuals(gz::sim::EntityComponentManager &_ecm,
					  const std::vector<double> &thrust_n,
					  const std::vector<double> &pitch_deg,
					  const std::vector<double> &yaw_deg,
					  const std::vector<double> &splay_deg);

		gz::sim::Model _model{gz::sim::kNullEntity};
		gz::sim::Link _link{gz::sim::kNullEntity};
		gz::transport::Node _node;
		std::string _base_link_name{"base_link"};
		std::string _command_topic;
		std::vector<Engine> _engines;
		std::vector<int> _ignition_sequence;
		std::vector<double> _commanded_thrust_n;
		std::vector<double> _commanded_pitch_deg;
		std::vector<double> _commanded_yaw_deg;
		std::vector<double> _commanded_splay_deg;
		std::mutex _command_mutex;
		double _reference_thrust_n{0.0};
		double _ignition_delay_s{0.0};
		double _ignition_dwell_s{0.0};
		double _burn_duration_s{0.0};
		double _splay_max_deg{0.0};
		double _reference_start_time_s{-1.0};
		double _last_command_time_s{-1.0};
		double _command_timeout_s{0.25};
		double _command_scale{1.0};
		double _thrust_visual_radius_m{0.016};
		double _thrust_visual_min_length_m{0.01};
		double _thrust_visual_max_length_m{0.55};
		gz::math::Vector3d _force_application_m{0.0, 0.0, 0.0};
		bool _thrust_axes_in_world{false};
		bool _apply_engine_torques{true};
		bool _commanded_thrust{true};
	};
	} // namespace tv3
