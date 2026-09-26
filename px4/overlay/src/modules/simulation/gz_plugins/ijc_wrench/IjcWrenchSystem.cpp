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

#include "IjcWrenchSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	IjcWrenchSystem,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate
)

void IjcWrenchSystem::Configure(const gz::sim::Entity &entity,
				const std::shared_ptr<const sdf::Element> &sdf,
				gz::sim::EntityComponentManager &ecm,
				gz::sim::EventManager &)
{
	if (sdf->HasElement("link_name")) {
		_link_name = sdf->Get<std::string>("link_name");
	}

	const auto world_name = gz::sim::World(entity).Name(ecm).value_or("default");
	const std::string topic = "/world/" + world_name + "/ijc_wrench";

	if (!_node.Subscribe(topic, &IjcWrenchSystem::onWrench, this)) {
		gzerr << "[IjcWrenchSystem] failed to subscribe to " << topic << std::endl;

	} else {
		gzmsg << "[IjcWrenchSystem] listening on " << topic << std::endl;
	}
}

void IjcWrenchSystem::onWrench(const gz::msgs::EntityWrench &msg)
{
	std::lock_guard<std::mutex> lock(_mutex);
	_model_name = msg.entity().name();
	_force_world.Set(msg.wrench().force().x(), msg.wrench().force().y(), msg.wrench().force().z());
	_torque_body.Set(msg.wrench().torque().x(), msg.wrench().torque().y(), msg.wrench().torque().z());
	_new_msg = true;
}

void IjcWrenchSystem::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (info.paused) {
		return;
	}

	std::lock_guard<std::mutex> lock(_mutex);

	if (_new_msg) {
		_new_msg = false;
		_have_msg = true;
		_last_msg_sim_time = info.simTime;
	}

	if (!_have_msg || info.simTime - _last_msg_sim_time > std::chrono::milliseconds(100)) {
		return;
	}

	if (_link == gz::sim::kNullEntity || _link_model_name != _model_name) {
		const gz::sim::Entity model = ecm.EntityByComponents(gz::sim::components::Name(_model_name),
					      gz::sim::components::Model());

		if (model == gz::sim::kNullEntity) {
			return;
		}

		_link = gz::sim::Model(model).LinkByName(ecm, _link_name);
		_link_model_name = _model_name;

		if (_link == gz::sim::kNullEntity) {
			gzerr << "[IjcWrenchSystem] link " << _link_name << " not found in " << _model_name << std::endl;
			return;
		}

		gz::sim::Link(_link).EnableVelocityChecks(ecm, true); // creates the WorldPose component
		gzmsg << "[IjcWrenchSystem] applying wrench to " << _model_name << "::" << _link_name << std::endl;
	}

	gz::sim::Link link(_link);
	const auto pose = link.WorldPose(ecm);
	const auto inertial = ecm.Component<gz::sim::components::Inertial>(_link);

	if (!pose || !inertial) {
		return;
	}

	// torque from the body (link) frame to the world frame; force applied at the CoM
	const gz::math::Vector3d torque_world = pose->Rot().RotateVector(_torque_body);
	const gz::math::Vector3d com_offset = inertial->Data().Pose().Pos();
	link.AddWorldWrench(ecm, _force_world, torque_world, com_offset);
}
