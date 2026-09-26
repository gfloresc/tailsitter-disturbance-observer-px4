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
 * @file IjcWrenchSystem.hpp
 *
 * World system that applies an external wrench to a model link (disturbance
 * injection for the ijc_control experiments). The wrench is received on
 * /world/<world>/ijc_wrench (gz.msgs.EntityWrench) from the PX4 gz_bridge:
 *   entity.name : model name (the wrench goes to its "base_link")
 *   wrench.force  : force in the world frame (ENU) [N], applied at the link CoM
 *   wrench.torque : torque in the link frame (FLU) [N m]
 * The last received wrench is applied at every physics step (zero-order hold);
 * it is dropped if no message is received for more than 100 ms of sim time.
 */

#pragma once

#include <mutex>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace custom
{
class IjcWrenchSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) final;

	void PreUpdate(const gz::sim::UpdateInfo &info,
		       gz::sim::EntityComponentManager &ecm) final;

private:
	void onWrench(const gz::msgs::EntityWrench &msg);

	gz::transport::Node _node;
	std::mutex _mutex;

	std::string _link_name{"base_link"};
	std::string _model_name;
	gz::math::Vector3d _force_world{};
	gz::math::Vector3d _torque_body{};
	bool _new_msg{false};
	bool _have_msg{false};
	std::chrono::steady_clock::duration _last_msg_sim_time{};

	gz::sim::Entity _link{gz::sim::kNullEntity};
	std::string _link_model_name;
};
} // end namespace custom
