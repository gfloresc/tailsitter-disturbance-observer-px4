/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file IjcVideoSystem.cpp
 *
 * World system for the ijc_control videos (worlds ijc_*_video):
 *  - moves a static camera model so that it keeps a fixed offset (world frame) from the
 *    vehicle and looks at it (the camera does not rotate with the vehicle, so its tilt is
 *    visible),
 *  - places a reference marker model at the hover reference (vehicle position before takeoff
 *    + <reference_height>), once,
 *  - optionally keeps a wind-arrow model at a fixed offset from the vehicle, oriented along
 *    <arrow_yaw> (ENU, rad).
 *
 * SDF: <model_name>, <link_name>, <camera_model>, <camera_offset> (x y z, ENU),
 *      <reference_model>, <reference_height>, <arrow_model>, <arrow_offset>, <arrow_yaw>.
 *
 * Fixed-camera mode (<follow>false</follow>): once the vehicle has settled on the ground (p_init),
 * the camera is placed at p_init + <look_at_offset> + <camera_offset>, looking at
 * p_init + <look_at_offset>, and does not move afterwards.
 * Trail: <trail_prefix> models <trail_prefix>0 .. N-1 (<trail_count>) placed at the vehicle
 * positions of the last N * <trail_period> seconds.
 * Wind arrow (instantaneous wind): <arrow_segment_prefix> models (<arrow_segment_count>, each
 * <arrow_segment_length> long along its x axis) and <arrow_head_model>; centered at
 * p_init + <arrow_anchor>, horizontal direction of the world wind, length = speed * <arrow_scale>.
 * Anchored models: <attach model="" offset="x y z"/> (repeated) are placed once at p_init + offset
 * (e.g. the desired circle drawn at the flight altitude).
 * Desired position (circle scenario, IJC_TRAJ = 1): <desired_model> is kept at the instantaneous
 * reference of ijc_control's circle, p_hover + R (sin th, cos th - 1, 0) (ENU; p_hover = p_init +
 * <reference_height>), th(t) with the angular-rate ramp <desired_ramp> and period <desired_period>,
 * radius <desired_radius>. The scenario time is t - t_msg - <desired_lead>, t_msg being the arrival
 * of the message on <desired_topic> (the wind trigger of ijc_flight.py, sent <desired_lead> s of
 * simulated time before the scenario start).
 */

#include <atomic>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace custom
{
class IjcVideoSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &, gz::sim::EventManager &) final
	{
		_model_name = sdf->Get<std::string>("model_name", _model_name).first;
		_link_name = sdf->Get<std::string>("link_name", _link_name).first;
		_camera_model = sdf->Get<std::string>("camera_model", _camera_model).first;
		_camera_offset = sdf->Get<gz::math::Vector3d>("camera_offset", _camera_offset).first;
		_reference_model = sdf->Get<std::string>("reference_model", _reference_model).first;
		_reference_height = sdf->Get<double>("reference_height", _reference_height).first;
		_arrow_model = sdf->Get<std::string>("arrow_model", _arrow_model).first;
		_arrow_offset = sdf->Get<gz::math::Vector3d>("arrow_offset", _arrow_offset).first;
		_arrow_yaw = sdf->Get<double>("arrow_yaw", _arrow_yaw).first;
		_follow = sdf->Get<bool>("follow", _follow).first;
		_look_at_offset = sdf->Get<gz::math::Vector3d>("look_at_offset", _look_at_offset).first;
		_trail_prefix = sdf->Get<std::string>("trail_prefix", "").first;
		_trail_count = sdf->Get<int>("trail_count", 0).first;
		_trail_period = sdf->Get<double>("trail_period", 0.1).first;
		_seg_prefix = sdf->Get<std::string>("arrow_segment_prefix", "").first;
		_seg_count = sdf->Get<int>("arrow_segment_count", 0).first;
		_seg_len = sdf->Get<double>("arrow_segment_length", 0.1).first;
		_head_model = sdf->Get<std::string>("arrow_head_model", "").first;
		_arrow_anchor = sdf->Get<gz::math::Vector3d>("arrow_anchor", _arrow_anchor).first;
		_arrow_scale = sdf->Get<double>("arrow_scale", 0.1).first;
		auto e = std::const_pointer_cast<sdf::Element>(sdf);

		for (auto a = e->FindElement("attach"); a; a = a->GetNextElement("attach")) {
			_attach_names.push_back(a->Get<std::string>("model"));
			_attach_offsets.push_back(a->Get<gz::math::Vector3d>("offset"));
		}

		_desired_model = sdf->Get<std::string>("desired_model", "").first;
		_desired_radius = sdf->Get<double>("desired_radius", 2.0).first;
		_desired_period = sdf->Get<double>("desired_period", 20.0).first;
		_desired_ramp = sdf->Get<double>("desired_ramp", 5.0).first;
		_desired_lead = sdf->Get<double>("desired_lead", 8.0).first;
		const std::string topic = sdf->Get<std::string>("desired_topic", "").first;

		if (!_desired_model.empty() && !topic.empty()) {
			_node.Subscribe(topic, &IjcVideoSystem::onTrigger, this);
		}

		// camera orientation: look from the offset towards the vehicle (x forward, no roll)
		const gz::math::Vector3d d = -_camera_offset;
		const double yaw = std::atan2(d.Y(), d.X());
		const double pitch = std::atan2(-d.Z(), std::hypot(d.X(), d.Y()));   // positive: looking down
		_camera_rot = gz::math::Quaterniond(0.0, pitch, yaw);
	}

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) final
	{
		if (_link == gz::sim::kNullEntity) {
			const auto model = find(ecm, _model_name);

			if (model == gz::sim::kNullEntity) {
				return;
			}

			_link = gz::sim::Model(model).LinkByName(ecm, _link_name);
			gz::sim::Link(_link).EnableVelocityChecks(ecm, true);
			_camera = find(ecm, _camera_model);
			_reference = find(ecm, _reference_model);
			_arrow = find(ecm, _arrow_model);

			for (int i = 0; i < _trail_count; i++) {
				_trail.push_back(find(ecm, _trail_prefix + std::to_string(i)));
			}

			for (int i = 0; i < _seg_count; i++) {
				_segments.push_back(find(ecm, _seg_prefix + std::to_string(i)));
			}

			_head = find(ecm, _head_model);

			for (const auto &n : _attach_names) {
				_attached.push_back(find(ecm, n));
			}

			_desired = find(ecm, _desired_model);
			_wind = ecm.EntityByComponents(gz::sim::components::Wind());
			return;
		}

		const auto pose = gz::sim::Link(_link).WorldPose(ecm);

		if (!pose) {
			return;
		}

		const gz::math::Vector3d p = pose->Pos();

		// reference marker: vehicle position on the ground (after it settled) + takeoff height
		if (!_reference_set && info.simTime > std::chrono::seconds(3)) {
			_p_init = p;

			if (_reference != gz::sim::kNullEntity) {
				setPose(ecm, _reference, gz::math::Pose3d(p + gz::math::Vector3d(0, 0, _reference_height), gz::math::Quaterniond::Identity));
			}

			if (!_follow && _camera != gz::sim::kNullEntity) {
				setPose(ecm, _camera, gz::math::Pose3d(p + _look_at_offset + _camera_offset, _camera_rot));
			}

			for (size_t i = 0; i < _attached.size(); i++) {
				if (_attached[i] != gz::sim::kNullEntity) {
					setPose(ecm, _attached[i], gz::math::Pose3d(p + _attach_offsets[i], gz::math::Quaterniond::Identity));
				}
			}

			_reference_set = true;
		}

		if (_follow && _camera != gz::sim::kNullEntity) {
			setPose(ecm, _camera, gz::math::Pose3d(p + _camera_offset, _camera_rot));
		}

		const double t = std::chrono::duration<double>(info.simTime).count();

		// trail of the last trail_count * trail_period seconds
		if (!_trail.empty() && t - _last_trail >= _trail_period) {
			_last_trail = t;
			_history.push_front(p);

			while (_history.size() > _trail.size()) {
				_history.pop_back();
			}

			for (size_t i = 0; i < _trail.size(); i++) {
				if (_trail[i] != gz::sim::kNullEntity) {
					const gz::math::Vector3d q = (i < _history.size()) ? _history[i] : gz::math::Vector3d(0, 0, -20);
					setPose(ecm, _trail[i], gz::math::Pose3d(q, gz::math::Quaterniond::Identity));
				}
			}
		}

		// instantaneous wind arrow (30 Hz is enough for the camera)
		if (!_segments.empty() && _reference_set && _wind != gz::sim::kNullEntity && t - _last_arrow >= 1.0 / 30.0) {
			_last_arrow = t;
			const auto wc = ecm.Component<gz::sim::components::WorldLinearVelocity>(_wind);
			const gz::math::Vector3d w = wc ? wc->Data() : gz::math::Vector3d::Zero;
			const double speed = std::hypot(w.X(), w.Y());
			const double yaw = std::atan2(w.Y(), w.X());
			const gz::math::Vector3d u(std::cos(yaw), std::sin(yaw), 0.0);
			const gz::math::Quaterniond rot(0, 0, yaw);
			const int n = std::min<int>(_segments.size(), std::max(1, static_cast<int>(std::round(speed * _arrow_scale / _seg_len))));
			const double L = n * _seg_len;
			const gz::math::Vector3d tail = _p_init + _arrow_anchor - u * (0.5 * L);

			for (int i = 0; i < static_cast<int>(_segments.size()); i++) {
				const gz::math::Vector3d q = (i < n) ? tail + u * (i * _seg_len) : gz::math::Vector3d(0, 0, -20);
				setPose(ecm, _segments[i], gz::math::Pose3d(q, rot));
			}

			if (_head != gz::sim::kNullEntity) {
				setPose(ecm, _head, gz::math::Pose3d(tail + u * L, rot));
			}
		}

		// desired position on the circle
		if (_desired != gz::sim::kNullEntity && _reference_set) {
			if (_triggered && _t_trigger < 0.0) {
				_t_trigger = t;
			}

			const double ts = (_t_trigger < 0.0) ? 0.0 : std::max(0.0, t - _t_trigger - _desired_lead);
			const double w = 2.0 * M_PI / _desired_period;
			const double th = (ts < _desired_ramp) ? 0.5 * w * ts * ts / _desired_ramp : w * (ts - 0.5 * _desired_ramp);
			const gz::math::Vector3d pd = _p_init + gz::math::Vector3d(_desired_radius * std::sin(th),
						      _desired_radius * (std::cos(th) - 1.0), _reference_height);
			setPose(ecm, _desired, gz::math::Pose3d(pd, gz::math::Quaterniond::Identity));
		}

		if (_arrow != gz::sim::kNullEntity) {
			setPose(ecm, _arrow, gz::math::Pose3d(p + _arrow_offset, gz::math::Quaterniond(0, 0, _arrow_yaw)));
		}
	}

private:
	void onTrigger(const gz::msgs::Empty &) { _triggered = true; }

	static gz::sim::Entity find(gz::sim::EntityComponentManager &ecm, const std::string &name)
	{
		if (name.empty()) {
			return gz::sim::kNullEntity;
		}

		return ecm.EntityByComponents(gz::sim::components::Name(name), gz::sim::components::Model());
	}

	static void setPose(gz::sim::EntityComponentManager &ecm, gz::sim::Entity e, const gz::math::Pose3d &pose)
	{
		// static models: update the Pose component directly (read by the rendering/sensors systems)
		auto comp = ecm.Component<gz::sim::components::Pose>(e);

		if (comp) {
			*comp = gz::sim::components::Pose(pose);
			ecm.SetChanged(e, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);
		}
	}

	std::string _model_name{"quadtailsitter_ijc_0"};
	std::string _link_name{"base_link"};
	std::string _camera_model{"ijc_cam"};
	std::string _reference_model{"ijc_ref"};
	std::string _arrow_model{};
	gz::math::Vector3d _camera_offset{3.2, -1.2, 0.8};
	gz::math::Vector3d _arrow_offset{0.0, 0.0, 1.0};
	double _reference_height{2.5};
	double _arrow_yaw{0.0};
	gz::math::Quaterniond _camera_rot{gz::math::Quaterniond::Identity};
	gz::sim::Entity _link{gz::sim::kNullEntity};
	gz::sim::Entity _camera{gz::sim::kNullEntity};
	gz::sim::Entity _reference{gz::sim::kNullEntity};
	gz::sim::Entity _arrow{gz::sim::kNullEntity};
	bool _reference_set{false};
	bool _follow{true};
	gz::math::Vector3d _look_at_offset{0, 0, 2.5};
	gz::math::Vector3d _p_init{0, 0, 0};
	std::string _trail_prefix, _seg_prefix, _head_model;
	int _trail_count{0}, _seg_count{0};
	double _trail_period{0.1}, _seg_len{0.1}, _arrow_scale{0.1};
	gz::math::Vector3d _arrow_anchor{0, 0, 3.5};
	std::vector<gz::sim::Entity> _trail, _segments;
	gz::sim::Entity _head{gz::sim::kNullEntity};
	gz::sim::Entity _wind{gz::sim::kNullEntity};
	std::deque<gz::math::Vector3d> _history;
	double _last_trail{-1.0}, _last_arrow{-1.0};
	std::vector<std::string> _attach_names;
	std::vector<gz::math::Vector3d> _attach_offsets;
	std::vector<gz::sim::Entity> _attached;
	std::string _desired_model;
	gz::sim::Entity _desired{gz::sim::kNullEntity};
	double _desired_radius{2.0}, _desired_period{20.0}, _desired_ramp{5.0}, _desired_lead{8.0};
	double _t_trigger{-1.0};
	std::atomic<bool> _triggered{false};
	gz::transport::Node _node;
};
} // namespace custom

GZ_ADD_PLUGIN(custom::IjcVideoSystem, gz::sim::System, custom::IjcVideoSystem::ISystemConfigure,
	      custom::IjcVideoSystem::ISystemPreUpdate)
