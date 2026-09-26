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
 * @file IjcAeroSystem.cpp
 *
 * World system that measures the aerodynamic wrench acting on a multirotor model
 * (ground truth for the disturbance observers of ijc_control in wind).
 *
 * Every step, after the model systems have queued their forces (it must run with a
 * higher <gz:system_priority> than AdvancedLiftDrag / MulticopterMotorModel), it reads the
 * ExternalWorldWrenchCmd of the base link and of the rotor links, i.e. everything that is
 * applied to the vehicle in that step except gravity, contacts and joint forces, and
 * removes the actuation:
 *   - rotor thrust: component of each rotor-link force along the rotor axis (the in-plane
 *     component is the aerodynamic rotor drag and is kept),
 *   - rotor drag torque on the base link: -dir_i * km * T_i * a_i (MulticopterMotorModel).
 * What remains (wing lift/drag/moments, rotor drag, rotor rolling moments, and any wrench of
 * IjcWrenchSystem) is published on /world/<world>/ijc_aero as gz.msgs.Wrench:
 *   force: world frame (ENU) [N];  torque: base-link frame (FLU), about the CoM [N m];
 *   header stamp: simulation time.
 *
 * SDF parameters: <model_name>, <base_link>, <rotor> (repeated: name, turning direction
 * attribute "ccw"/"cw"), <moment_constant>.
 */

#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace custom
{
class IjcAeroSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) final
	{
		_model_name = sdf->Get<std::string>("model_name", _model_name).first;
		_base_name = sdf->Get<std::string>("base_link", _base_name).first;
		_km = sdf->Get<double>("moment_constant", _km).first;

		auto elem = std::const_pointer_cast<sdf::Element>(sdf);

		for (auto r = elem->FindElement("rotor"); r; r = r->GetNextElement("rotor")) {
			Rotor rotor;
			rotor.name = r->Get<std::string>();
			rotor.dir = (r->Get<std::string>("direction") == "cw") ? -1.0 : 1.0;
			_rotors.push_back(rotor);
		}

		const auto world_name = gz::sim::World(entity).Name(ecm).value_or("default");
		_pub = _node.Advertise<gz::msgs::Wrench>("/world/" + world_name + "/ijc_aero");
		gzmsg << "[IjcAeroSystem] " << _model_name << ": " << _rotors.size() << " rotors, km = " << _km << std::endl;
	}

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) final
	{
		if (info.paused) {
			return;
		}

		if (_base == gz::sim::kNullEntity) {
			const auto model = ecm.EntityByComponents(gz::sim::components::Name(_model_name), gz::sim::components::Model());

			if (model == gz::sim::kNullEntity) {
				return;
			}

			gz::sim::Model m(model);
			_base = m.LinkByName(ecm, _base_name);
			gz::sim::Link(_base).EnableVelocityChecks(ecm, true);

			for (auto &r : _rotors) {
				r.link = m.LinkByName(ecm, r.name);
				gz::sim::Link(r.link).EnableVelocityChecks(ecm, true);
			}

			return;
		}

		gz::sim::Link base(_base);
		const auto base_pose = base.WorldPose(ecm);
		const auto com_pose = base.WorldInertialPose(ecm);

		if (!base_pose || !com_pose) {
			return;
		}

		const gz::math::Vector3d c = com_pose->Pos();
		gz::math::Vector3d force, moment;

		auto accumulate = [&](gz::sim::Entity link, gz::math::Vector3d &F_out) {
			F_out = gz::math::Vector3d::Zero;
			const auto cmd = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(link);
			const auto pose = gz::sim::Link(link).WorldPose(ecm);

			if (!cmd || !pose) {
				return;
			}

			// the queued wrench is referred to the link origin (force + torque about the origin)
			const gz::math::Vector3d F = gz::msgs::Convert(cmd->Data().force());
			const gz::math::Vector3d tau = gz::msgs::Convert(cmd->Data().torque());
			force += F;
			moment += tau + (pose->Pos() - c).Cross(F);
			F_out = F;
		};

		gz::math::Vector3d unused;
		accumulate(_base, unused);

		for (auto &r : _rotors) {
			gz::math::Vector3d F;
			accumulate(r.link, F);
			const auto pose = gz::sim::Link(r.link).WorldPose(ecm);

			if (!pose) {
				continue;
			}

			const gz::math::Vector3d a = pose->Rot().RotateVector(gz::math::Vector3d::UnitZ);
			const double T = F.Dot(a);
			const gz::math::Vector3d F_thrust = a * T;
			force -= F_thrust;
			moment -= (pose->Pos() - c).Cross(F_thrust);
			moment -= a * (-r.dir * _km * T);   // rotor drag torque applied to the base link
		}

		gz::msgs::Wrench msg;
		const auto sec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
		msg.mutable_header()->mutable_stamp()->set_sec(sec.count());
		msg.mutable_header()->mutable_stamp()->set_nsec(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - sec).count());
		gz::msgs::Set(msg.mutable_force(), force);
		gz::msgs::Set(msg.mutable_torque(), base_pose->Rot().RotateVectorReverse(moment));
		_pub.Publish(msg);
	}

private:
	struct Rotor {
		std::string name;
		double dir{1.0};
		gz::sim::Entity link{gz::sim::kNullEntity};
	};

	std::string _model_name{"quadtailsitter_ijc_0"};
	std::string _base_name{"base_link"};
	double _km{0.06};
	std::vector<Rotor> _rotors;
	gz::sim::Entity _base{gz::sim::kNullEntity};
	gz::transport::Node _node;
	gz::transport::Node::Publisher _pub;
};
} // namespace custom

GZ_ADD_PLUGIN(custom::IjcAeroSystem, gz::sim::System, custom::IjcAeroSystem::ISystemConfigure,
	      custom::IjcAeroSystem::ISystemPreUpdate)
