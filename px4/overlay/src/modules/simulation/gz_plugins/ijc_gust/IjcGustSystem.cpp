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
 * @file IjcGustSystem.cpp
 *
 * Aggressive, multi-frequency wind for the ijc_control demonstration videos. Every physics step
 * (before the model systems: use a negative <gz:system_priority>) it sets the world wind velocity
 * (WorldLinearVelocity of the wind entity), which the aerodynamic systems of the model use:
 *
 *   speed(t) = r(t) * [ U0 + sum_i a_i sin(2 pi t / T_i + phi_i) ] + turbulence
 *   dir(t)   = D0 + sum_j b_j sin(2 pi t / P_j + psi_j) + direction noise
 *
 * t is the time since the wind was triggered: at simulation start; with <trigger_topic>, when the
 * first message arrives on that gz-transport topic (ijc_flight.py --gust-topic sends it a fixed simulated
 * time before the scenario start, so every run sees the same wind relative to its scenario); or, with
 * <trigger_model> and <trigger_height>, when that model first climbs above the height. The vehicle takes
 * off in calm air; the random turbulence also starts at the trigger.
 * r(t): linear rise over <rise_time>; turbulence: first-order Gauss-Markov per horizontal axis
 * (sigma, tau) and vertical (sigma_z, tau). Components are given as <speed_sine amp="" period="" phase=""/>
 * and <dir_sine amp_deg="" period="" phase=""/> elements. The wind is also logged to <log_file>
 * (sim time, wind ENU) every 10 ms when given; a relative <log_file> is resolved against the
 * environment variable IJC_ROOT (repository root, exported by scripts/paths.sh), and its directory
 * is created if needed.
 *
 * Gust fronts: <front t="" rise="" hold="" fall="" dspeed="" ddir_deg=""/> add a trapezoid (linear rise,
 * hold, linear fall; times in wind time) of dspeed [m/s] to the speed and ddir_deg to the direction:
 * step-like gust fronts and sudden, sustained direction changes.
 * External pulses: <pulse t="" dur="" fx="" fy="" fz="" tx="" ty="" tz=""/> apply a rectangular wrench
 * to <pulse_model>::<pulse_link> (force in the world ENU frame at the CoM [N], torque in the link frame
 * [N m]); they are added to the other external wrenches (e.g. IjcWrenchSystem).
 */

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/transport/Node.hh>

namespace custom
{
class IjcGustSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &, gz::sim::EventManager &) final
	{
		auto e = std::const_pointer_cast<sdf::Element>(sdf);
		_U0 = sdf->Get<double>("mean_speed", _U0).first;
		_D0 = sdf->Get<double>("mean_direction_deg", _D0).first * M_PI / 180.0;
		_rise = sdf->Get<double>("rise_time", _rise).first;
		_sigma = sdf->Get<double>("turbulence_sigma", _sigma).first;
		_sigma_z = sdf->Get<double>("turbulence_sigma_z", _sigma_z).first;
		_tau = sdf->Get<double>("turbulence_tau", _tau).first;
		_dir_sigma = sdf->Get<double>("direction_noise_deg", _dir_sigma).first * M_PI / 180.0;
		_scale = sdf->Get<double>("scale", _scale).first;
		_rng.seed(sdf->Get<unsigned>("seed", 1u).first);
		_log_file = sdf->Get<std::string>("log_file", "").first;
		_trigger_model = sdf->Get<std::string>("trigger_model", "").first;
		_trigger_height = sdf->Get<double>("trigger_height", 0.0).first;
		_trigger_topic = sdf->Get<std::string>("trigger_topic", "").first;

		if (!_trigger_topic.empty()) {
			if (_node.Subscribe(_trigger_topic, &IjcGustSystem::onTrigger, this)) {
				gzmsg << "IjcGustSystem: wind starts on a message on " << _trigger_topic << std::endl;

			} else {
				gzerr << "IjcGustSystem: failed to subscribe to " << _trigger_topic << std::endl;
			}
		}

		for (auto s = e->FindElement("speed_sine"); s; s = s->GetNextElement("speed_sine")) {
			_speed_sines.push_back({s->Get<double>("amp"), s->Get<double>("period"), s->Get<double>("phase")});
		}

		for (auto s = e->FindElement("dir_sine"); s; s = s->GetNextElement("dir_sine")) {
			_dir_sines.push_back({s->Get<double>("amp_deg") * M_PI / 180.0, s->Get<double>("period"), s->Get<double>("phase")});
		}

		for (auto s = e->FindElement("front"); s; s = s->GetNextElement("front")) {
			_fronts.push_back({s->Get<double>("t"), s->Get<double>("rise"), s->Get<double>("hold"), s->Get<double>("fall"),
					   s->Get<double>("dspeed"), s->Get<double>("ddir_deg") * M_PI / 180.0});
		}

		for (auto s = e->FindElement("pulse"); s; s = s->GetNextElement("pulse")) {
			_pulses.push_back({s->Get<double>("t"), s->Get<double>("dur"),
					   gz::math::Vector3d(s->Get<double>("fx"), s->Get<double>("fy"), s->Get<double>("fz")),
					   gz::math::Vector3d(s->Get<double>("tx"), s->Get<double>("ty"), s->Get<double>("tz"))});
		}

		_pulse_model = sdf->Get<std::string>("pulse_model", "").first;
		_pulse_link = sdf->Get<std::string>("pulse_link", "base_link").first;

		if (!_log_file.empty() && _log_file.front() != '/' && std::getenv("IJC_ROOT") != nullptr) {
			_log_file = std::string(std::getenv("IJC_ROOT")) + "/" + _log_file;
		}

		if (!_log_file.empty()) {
			std::error_code ec;
			std::filesystem::create_directories(std::filesystem::path(_log_file).parent_path(), ec);
			_log.open(_log_file);
			_log << "sim_time_s,wind_time_s,wind_e,wind_n,wind_u\n";
		}
	}

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) final
	{
		if (info.paused) {
			return;
		}

		if (_wind == gz::sim::kNullEntity) {
			_wind = ecm.EntityByComponents(gz::sim::components::Wind());

			if (_wind == gz::sim::kNullEntity) {
				return;
			}
		}

		const double t_sim = std::chrono::duration<double>(info.simTime).count();
		const double dt = std::chrono::duration<double>(info.dt).count();

		if (_t_trigger < 0.0) {
			if (!_trigger_topic.empty()) {
				if (_triggered) {
					_t_trigger = t_sim;
					gzmsg << "IjcGustSystem: wind triggered at sim time " << t_sim << " s" << std::endl;
				}

			} else if (_trigger_model.empty()) {
				_t_trigger = 0.0;

			} else {
				if (_model == gz::sim::kNullEntity) {
					_model = ecm.EntityByComponents(gz::sim::components::Name(_trigger_model), gz::sim::components::Model());
				}

				auto pose = (_model != gz::sim::kNullEntity) ? ecm.Component<gz::sim::components::Pose>(_model) : nullptr;

				if (pose && pose->Data().Pos().Z() > _trigger_height) {
					_t_trigger = t_sim;
					gzmsg << "IjcGustSystem: wind triggered at sim time " << t_sim << " s" << std::endl;
				}
			}

			if (_t_trigger < 0.0) {
				return;
			}
		}

		const double t = t_sim - _t_trigger;

		// Gauss-Markov turbulence (exact discretization)
		if (dt > 0) {
			const double a = std::exp(-dt / _tau);
			const double b = std::sqrt(1.0 - a * a);
			_turb.X() = a * _turb.X() + b * _sigma * _n(_rng);
			_turb.Y() = a * _turb.Y() + b * _sigma * _n(_rng);
			_turb.Z() = a * _turb.Z() + b * _sigma_z * _n(_rng);
			_dir_noise = a * _dir_noise + b * _dir_sigma * _n(_rng);
		}

		double speed = _U0;

		for (const auto &s : _speed_sines) {
			speed += s.amp * std::sin(2.0 * M_PI * t / s.period + s.phase);
		}

		double dir = _D0 + _dir_noise;

		for (const auto &s : _dir_sines) {
			dir += s.amp * std::sin(2.0 * M_PI * t / s.period + s.phase);
		}

		for (const auto &f : _fronts) {
			const double env = f.envelope(t);
			speed += f.dspeed * env;
			dir += f.ddir * env;
		}

		const double r = (_rise > 0.0) ? std::min(1.0, t / _rise) : 1.0;
		speed = std::max(0.0, speed) * r * _scale;
		gz::math::Vector3d w(speed * std::cos(dir), speed * std::sin(dir), 0.0);
		w += _turb * (r * _scale);

		auto comp = ecm.Component<gz::sim::components::WorldLinearVelocity>(_wind);

		if (comp) {
			*comp = gz::sim::components::WorldLinearVelocity(w);

		} else {
			ecm.CreateComponent(_wind, gz::sim::components::WorldLinearVelocity(w));
		}

		ecm.SetChanged(_wind, gz::sim::components::WorldLinearVelocity::typeId, gz::sim::ComponentState::PeriodicChange);

		applyPulses(t, ecm);

		if (_log.is_open() && t - _last_log >= 0.01) {
			_log << t_sim << ',' << t << ',' << w.X() << ',' << w.Y() << ',' << w.Z() << '\n';
			_last_log = t;
		}
	}

private:
	void onTrigger(const gz::msgs::Empty &) { _triggered = true; }

	struct Sine {
		double amp, period, phase;
	};

	struct Front {
		double t, rise, hold, fall, dspeed, ddir;

		double envelope(double tw) const
		{
			const double a = tw - t;

			if (a <= 0.0) { return 0.0; }

			if (a < rise) { return a / rise; }

			if (a < rise + hold) { return 1.0; }

			if (a < rise + hold + fall) { return 1.0 - (a - rise - hold) / fall; }

			return 0.0;
		}
	};

	struct Pulse {
		double t, dur;
		gz::math::Vector3d force, torque;
	};

	void applyPulses(double t, gz::sim::EntityComponentManager &ecm)
	{
		if (_pulses.empty() || _pulse_model.empty()) {
			return;
		}

		gz::math::Vector3d force, torque_body;
		bool active = false;

		for (const auto &p : _pulses) {
			if (t >= p.t && t < p.t + p.dur) {
				force += p.force;
				torque_body += p.torque;
				active = true;
			}
		}

		if (!active) {
			return;
		}

		if (_pulse_link_entity == gz::sim::kNullEntity) {
			const auto model = ecm.EntityByComponents(gz::sim::components::Name(_pulse_model), gz::sim::components::Model());

			if (model == gz::sim::kNullEntity) {
				return;
			}

			_pulse_link_entity = gz::sim::Model(model).LinkByName(ecm, _pulse_link);

			if (_pulse_link_entity == gz::sim::kNullEntity) {
				return;
			}

			gz::sim::Link(_pulse_link_entity).EnableVelocityChecks(ecm, true);   // creates the WorldPose component
		}

		gz::sim::Link link(_pulse_link_entity);
		const auto pose = link.WorldPose(ecm);
		const auto inertial = ecm.Component<gz::sim::components::Inertial>(_pulse_link_entity);

		if (!pose || !inertial) {
			return;
		}

		link.AddWorldWrench(ecm, force, pose->Rot().RotateVector(torque_body), inertial->Data().Pose().Pos());
	}

	double _U0{5.0}, _D0{0.0}, _rise{5.0}, _sigma{0.5}, _sigma_z{0.2}, _tau{0.5}, _dir_sigma{0.0}, _scale{1.0};
	std::vector<Sine> _speed_sines, _dir_sines;
	gz::math::Vector3d _turb{0, 0, 0};
	double _dir_noise{0.0};
	std::mt19937 _rng;
	std::normal_distribution<double> _n{0.0, 1.0};
	std::string _log_file;
	std::ofstream _log;
	double _last_log{-1.0};
	gz::sim::Entity _wind{gz::sim::kNullEntity};
	std::string _trigger_model;
	std::vector<Front> _fronts;
	std::vector<Pulse> _pulses;
	std::string _pulse_model, _pulse_link;
	gz::sim::Entity _pulse_link_entity{gz::sim::kNullEntity};
	std::string _trigger_topic;
	std::atomic<bool> _triggered{false};
	gz::transport::Node _node;
	double _trigger_height{0.0};
	double _t_trigger{-1.0};
	gz::sim::Entity _model{gz::sim::kNullEntity};
};
} // namespace custom

GZ_ADD_PLUGIN(custom::IjcGustSystem, gz::sim::System, custom::IjcGustSystem::ISystemConfigure,
	      custom::IjcGustSystem::ISystemPreUpdate)
