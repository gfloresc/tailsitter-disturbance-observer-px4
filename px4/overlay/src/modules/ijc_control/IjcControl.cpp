/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include "IjcControl.hpp"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace matrix;

static constexpr float kGravity = CONSTANTS_ONE_G;
static constexpr float kAirborneHeight = 0.05f;       ///< height above arming point that activates the observers [m]
static constexpr float kLandBelowGround = 0.3f;       ///< landing reference goes this far below the arming point [m]
static constexpr float kTouchdownHeight = 0.05f;      ///< touchdown if closer to the arming height than this [m]
static constexpr float kTouchdownRampTime = 1.0f;     ///< thrust ramp-down after touchdown [s]
static constexpr hrt_abstime kAllocatorStatusTimeout = 50_ms;

IjcControl::IjcControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_thrust_setpoint_pub.advertise();
	_vehicle_torque_setpoint_pub.advertise();
	_ijc_debug_pub.advertise();
	parametersUpdated();
	setCommand(0);
}

IjcControl::~IjcControl()
{
	perf_free(_loop_perf);
}

bool IjcControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void IjcControl::parametersUpdated()
{
	_mass = _param_mass.get();
	_J = Vector3f(_param_jxx.get(), _param_jyy.get(), _param_jzz.get());
	_tau_max = Vector3f(_param_tmax_x.get(), _param_tmax_y.get(), _param_tmax_z.get());
	_f_max = _param_fmax.get();

	_obs_rot.setGains(_param_k1.get(), _param_l1.get());
	_obs_trans.setGains(_param_k2.get(), _param_l2.get());
}

void IjcControl::setCommand(int32_t cmd)
{
	if (_param_cmd.get() != cmd) {
		_param_cmd.set(cmd);
		_param_cmd.commit_no_notification();
	}

	_last_cmd = cmd;
}

void IjcControl::resetObservers()
{
	_obs_rot.reset(_omega);
	_obs_trans.reset(_v);
	_thrust_acc_integral.zero();
	_thrust_acc_time = 0.f;
}

void IjcControl::computeDisturbance(float t, Vector3f &F, Vector3f &T) const
{
	// S1 signals (Section 6 of the spec / Section 5.2 of the paper), t = scenario time
	F.zero();
	T.zero();

	if (_param_dist.get() == 2) {
		computeRandomDisturbance(t, F, T);
		return;
	}

	if (_param_dist.get() != 1 || t < 0.f) {
		return;
	}

	const float A_F = _param_af.get();
	const Vector3f A_T(_param_atx.get(), _param_aty.get(), _param_atz.get());

	static constexpr float pulse_start[] {5.f, 10.f, 15.f};
	bool pulse = false;

	for (float n : pulse_start) {
		if (t > n && t < n + 1.5f) {
			pulse = true;
		}
	}

	F(0) = pulse ? A_F : 0.f;
	F(1) = (t > 5.f) ? 0.4f * A_F * sinf(t) : 0.f;
	F(2) = (t >= 5.f) ? 0.2f * A_F * (t / 3.f - floorf(t / 3.f)) : 0.f;

	T(0) = pulse ? A_T(0) : 0.f;
	T(1) = (t > 7.f) ? A_T(1) * sinf(t) : 0.f;
	T(2) = (t >= 7.f) ? A_T(2) * (t / 2.f - floorf(t / 2.f)) : 0.f;
}

void IjcControl::computeRandomDisturbance(float t, Vector3f &F, Vector3f &T) const
{
	// S1r: same structure and amplitude class as S1 (pulses, sinusoids, sawtooths on each axis),
	// with pulse times/durations/signs, frequencies, phases and periods drawn from IJC_DSEED.
	F.zero();
	T.zero();

	if (t < 0.f) {
		return;
	}

	uint32_t state = static_cast<uint32_t>(_param_dseed.get()) * 2654435761u + 12345u;
	auto uniform = [&state](float lo, float hi) {
		// xorshift32
		state ^= state << 13;
		state ^= state >> 17;
		state ^= state << 5;
		return lo + (hi - lo) * static_cast<float>(state & 0xFFFFFFu) / 16777216.f;
	};

	const float A_F = _param_af.get();
	const Vector3f A_T(_param_atx.get(), _param_aty.get(), _param_atz.get());

	// pulses on F_x and T_x: three each, independent times, 1-2 s long, amplitude 70-100 %, random sign
	for (int axis = 0; axis < 2; axis++) {
		for (int k = 0; k < 3; k++) {
			const float t0 = 3.5f + 5.f * k + uniform(0.f, 2.5f);
			const float dur = uniform(1.f, 2.f);
			const float amp = uniform(0.7f, 1.f) * (uniform(0.f, 1.f) < 0.5f ? -1.f : 1.f);

			if (t > t0 && t < t0 + dur) {
				if (axis == 0) { F(0) = amp * A_F; }

				else { T(0) = amp * A_T(0); }
			}
		}
	}

	// sinusoids on F_y and T_y
	const float wy_f = uniform(0.7f, 1.3f), ph_f = uniform(0.f, 6.2832f), ty_f = uniform(3.f, 6.f);
	const float wy_t = uniform(0.7f, 1.3f), ph_t = uniform(0.f, 6.2832f), ty_t = uniform(5.f, 8.f);
	F(1) = (t > ty_f) ? 0.4f * A_F * sinf(wy_f * t + ph_f) : 0.f;
	T(1) = (t > ty_t) ? A_T(1) * sinf(wy_t * t + ph_t) : 0.f;

	// sawtooths on F_z and T_z (random period, onset and sign)
	const float pz_f = uniform(2.f, 4.f), tz_f = uniform(3.f, 6.f), sz_f = uniform(0.f, 1.f) < 0.5f ? -1.f : 1.f;
	const float pz_t = uniform(1.5f, 3.f), tz_t = uniform(5.f, 8.f), sz_t = uniform(0.f, 1.f) < 0.5f ? -1.f : 1.f;
	F(2) = (t >= tz_f) ? sz_f * 0.2f * A_F * (t / pz_f - floorf(t / pz_f)) : 0.f;
	T(2) = (t >= tz_t) ? sz_t * A_T(2) * (t / pz_t - floorf(t / pz_t)) : 0.f;
}

void IjcControl::circleReference(float t)
{
	// circle through the hover point, constant altitude; angular rate ramps from 0 to w in T_r
	const float R = _param_trj_r.get();
	const float w = 2.f * M_PI_F / _param_trj_t.get();
	const float Tr = _param_trj_tr.get();
	float th, dth, ddth;

	if (t < Tr) {
		th = 0.5f * w * t * t / Tr;
		dth = w * t / Tr;
		ddth = w / Tr;

	} else {
		th = w * (t - 0.5f * Tr);
		dth = w;
		ddth = 0.f;
	}

	const float c = cosf(th);
	const float s = sinf(th);
	_p_d = _p_hover + Vector3f(R * (c - 1.f), R * s, 0.f);
	_v_d = Vector3f(-R * s * dth, R * c * dth, 0.f);
	_a_d = Vector3f(-R * s * ddth - R * c * dth * dth, R * c * ddth - R * s * dth * dth, 0.f);
}

void IjcControl::updateReference(hrt_abstime now)
{
	const bool armed_offboard = _control_mode.flag_armed && _control_mode.flag_control_offboard_enabled;

	if (!armed_offboard) {
		if (_state != State::Idle) {
			PX4_INFO("disengaged (armed: %d, offboard: %d)", _control_mode.flag_armed,
				 _control_mode.flag_control_offboard_enabled);
		}

		_state = State::Idle;
		_engaged = false;
		_observers_active = false;
		_scenario_running = false;
		return;
	}

	const float t_state = (now > _t_state_start) ? (now - _t_state_start) * 1e-6f : 0.f;

	switch (_state) {
	case State::Idle:
		if (_local_position_valid) {
			_p0 = _p;
			_p_hover = _p0 - Vector3f(0.f, 0.f, _param_tko_alt.get());
			_p_d = _p0;
			_v_d.zero();
			_a_d.zero();
			_t_state_start = now;
			_observers_active = false;
			_scenario_running = false;
			setCommand(0);
			resetObservers();
			_f_model = 0.f;
			_tau_model.zero();
			_f_lat_ff.zero();
			_engaged = true;
			_state = State::Takeoff;
			PX4_INFO("takeoff to %.2f m", (double)_param_tko_alt.get());
		}

		break;

	case State::Takeoff: {
			// quintic (minimum-jerk) reference from p0 to p_hover
			const float T = _param_tko_t.get();
			const float s = math::constrain(t_state / T, 0.f, 1.f);
			const float sigma = s * s * s * (10.f - 15.f * s + 6.f * s * s);
			const float dsigma = 30.f * s * s * (1.f - s) * (1.f - s) / T;
			const float ddsigma = 60.f * s * (1.f - s) * (1.f - 2.f * s) / (T * T);
			const Vector3f delta = _p_hover - _p0;
			_p_d = _p0 + delta * sigma;
			_v_d = delta * dsigma;
			_a_d = delta * ddsigma;

			if (!_observers_active && (_p0(2) - _p(2)) > kAirborneHeight) {
				resetObservers();
				_observers_active = true;
			}

			if (s >= 1.f) {
				_p_d = _p_hover;
				_v_d.zero();
				_a_d.zero();

				if (!_observers_active) {
					resetObservers();
					_observers_active = true;
				}

				_state = State::Hover;
				PX4_INFO("hover");
			}
		}
		break;

	case State::Hover: {
			_p_d = _p_hover;
			_v_d.zero();
			_a_d.zero();

			const int32_t cmd = _param_cmd.get();

			if (cmd == 1 && !_scenario_running) {
				_scenario_running = true;
				_t_scen_start = now;
				PX4_INFO("scenario started");

			} else if (cmd == 0 && _scenario_running) {
				_scenario_running = false;
				PX4_INFO("scenario stopped");

			} else if (cmd == 2) {
				_scenario_running = false;
				_land_z_start = _p_d(2);
				_land_xy = Vector2f(_p_d(0), _p_d(1));
				_t_state_start = now;
				_state = State::Land;
				PX4_INFO("landing");
			}

			_last_cmd = cmd;

			if (_scenario_running && _state == State::Hover && _param_traj.get() == 1) {
				circleReference((now - _t_scen_start) * 1e-6f);
			}
		}
		break;

	case State::Land: {
			const float vel = _param_lnd_vel.get();
			const float z_end = _p0(2) + kLandBelowGround;
			const float z_d = _land_z_start + vel * t_state;

			_p_d = Vector3f(_land_xy(0), _land_xy(1), math::min(z_d, z_end));
			_v_d = Vector3f(0.f, 0.f, (z_d < z_end) ? vel : 0.f);
			_a_d.zero();

			const bool near_ground = (_p0(2) - _p(2)) < kTouchdownHeight && fabsf(_v(2)) < 0.2f;

			if (near_ground || _land_detected.ground_contact || _land_detected.maybe_landed || _land_detected.landed) {
				if (_touchdown_candidate == 0) {
					_touchdown_candidate = now;
				}

			} else {
				_touchdown_candidate = 0;
			}

			if (_touchdown_candidate != 0 && now - _touchdown_candidate > 300_ms) {
				_touchdown_candidate = 0;
				_touchdown_f0 = _f;
				_observers_active = false;
				_t_state_start = now;
				_state = State::Touchdown;
				PX4_INFO("touchdown");
			}
		}
		break;

	case State::Touchdown:
		break;
	}
}

void IjcControl::publishTrajectorySetpoint()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_p_d.copyTo(trajectory_setpoint.position);
	_v_d.copyTo(trajectory_setpoint.velocity);
	_a_d.copyTo(trajectory_setpoint.acceleration);
	trajectory_setpoint.jerk[0] = trajectory_setpoint.jerk[1] = trajectory_setpoint.jerk[2] = NAN;
	trajectory_setpoint.yaw = math::radians(_param_psi_d.get());
	trajectory_setpoint.yawspeed = 0.f;

	if (_state == State::Touchdown) {
		// PX4 controllers: keep descending on the ground so that the land detector triggers
		trajectory_setpoint.position[2] = NAN;
		trajectory_setpoint.velocity[0] = trajectory_setpoint.velocity[1] = NAN;
		trajectory_setpoint.velocity[2] = _param_lnd_vel.get();
		trajectory_setpoint.acceleration[0] = trajectory_setpoint.acceleration[1] = trajectory_setpoint.acceleration[2] = NAN;
	}

	trajectory_setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint_pub.publish(trajectory_setpoint);
}

void IjcControl::positionControl()
{
	const Vector3f e3(0.f, 0.f, 1.f);
	const Vector3f e_p = _p - _p_d;
	const Vector3f e_v = _v - _v_d;
	const bool comp = _param_comp.get() == 1;
	const Vector3f F_hat = (comp && _observers_active) ? _obs_trans.d_hat() : Vector3f{};

	// Proposition 4.1
	Vector3f mu = (e3 * kGravity + e_p * _param_kp.get() + e_v * _param_kv.get() - _a_d + F_hat) * _mass;

	// canted rotors: known lateral force produced with the torques (excluded from F_hat), as low-pass
	// filtered feedforward
	const int32_t ff_mode = _param_flat_ff.get();
	Vector3f f_lat{};

	if (_observers_active && (ff_mode == 2 || (ff_mode == 1 && comp))) {
		const Vector3f tau_ff = (ff_mode == 1) ? Vector3f(-_J.emult(_obs_rot.d_hat())) : _tau_model;
		const Vector3f tau_ff_norm = tau_ff.edivide(_tau_max);
		const Vector3f F_lat_body(_param_fx_ty.get() * tau_ff_norm(1), _param_fy_tx.get() * tau_ff_norm(0), 0.f);
		f_lat = Dcmf(_q) * F_lat_body;
	}

	const float tc = _param_flat_tc.get();

	if (tc > 0.f && _dt_pos > 0.f) {
		const float a = expf(-_dt_pos / tc);
		_f_lat_ff = _f_lat_ff * a + f_lat * (1.f - a);

	} else {
		_f_lat_ff = f_lat;
	}

	mu += _f_lat_ff;

	// practical safeguards: minimum vertical component and maximum tilt of mu
	mu(2) = math::max(mu(2), _param_muz_min.get() * _mass * kGravity);
	const float mu_xy = Vector2f(mu(0), mu(1)).norm();
	const float mu_xy_max = mu(2) * tanf(math::radians(_param_tilt_max.get()));

	if (mu_xy > mu_xy_max) {
		mu(0) *= mu_xy_max / mu_xy;
		mu(1) *= mu_xy_max / mu_xy;
	}

	_mu = mu;
	_f = mu.norm();

	// desired attitude (eqs. 20-24)
	const float psi_d = math::radians(_param_psi_d.get());
	const float cp = cosf(psi_d);
	const float sp = sinf(psi_d);
	const Vector3f z(( cp * mu(0) + sp * mu(1)) / _f,
			 (-sp * mu(0) + cp * mu(1)) / _f,
			 mu(2) / _f);
	const float s = sqrtf(2.f * (1.f + z(2)));
	const Quatf Q_r((1.f + z(2)) / s, -z(1) / s, z(0) / s, 0.f);
	const Quatf Q_a(cosf(psi_d / 2.f), 0.f, 0.f, sinf(psi_d / 2.f));
	_q_d = Q_a * Q_r;
	_q_d.normalize();
}

void IjcControl::attitudeControl()
{
	// Proposition 4.2 (no Omega_d feedforward)
	Quatf q_bar = _q_d.inversed() * _q;

	if (q_bar(0) < 0.f) {
		q_bar = -q_bar;
	}

	_q_bar = q_bar;

	const bool comp = _param_comp.get() == 1;
	const Vector3f T_hat = (comp && _observers_active) ? _obs_rot.d_hat() : Vector3f{};
	const Vector3f q_v(q_bar(1), q_bar(2), q_bar(3));
	const Vector3f J_omega = _J.emult(_omega);

	_tau = _omega.cross(J_omega) + _J.emult(-T_hat - q_v * _param_kq.get() - _omega * _param_kom.get());
}

void IjcControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parametersUpdated();
	}

	vehicle_angular_velocity_s angular_velocity;

	if (!_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		perf_end(_loop_perf);
		return;
	}

	const hrt_abstime now = angular_velocity.timestamp_sample;
	_dt_att = (_last_att_sample != 0) ? math::constrain((now - _last_att_sample) * 1e-6f, 1e-4f, 0.02f) : 0.f;
	_last_att_sample = now;
	_omega = Vector3f(angular_velocity.xyz);

	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.update(&attitude)) {
		_q = Quatf(attitude.q);

	} else if (_dt_att > 0.f) {
		// EKF attitude is published at a lower rate than the gyro: propagate it with Omega in between
		_q = _q * Quatf(AxisAnglef(_omega * _dt_att));
		_q.normalize();
	}

	_vehicle_control_mode_sub.update(&_control_mode);
	_vehicle_land_detected_sub.update(&_land_detected);
	_control_allocator_status_sub.update(&_allocator_status);

	bool new_position = false;
	vehicle_local_position_s local_position;

	if (_vehicle_local_position_sub.update(&local_position)) {
		_local_position_valid = local_position.xy_valid && local_position.z_valid
					&& local_position.v_xy_valid && local_position.v_z_valid;
		_p = Vector3f(local_position.x, local_position.y, local_position.z);
		_v = Vector3f(local_position.vx, local_position.vy, local_position.vz);

		if (local_position.timestamp_sample > _last_pos_sample) {
			_dt_pos = (_last_pos_sample != 0) ? math::constrain((local_position.timestamp_sample - _last_pos_sample) * 1e-6f,
					1e-4f, 0.1f) : 0.f;
			_last_pos_sample = local_position.timestamp_sample;
			new_position = true;
		}
	}

	// ---- applied inputs over the step that just ended (setpoint minus what the allocator could not produce)
	if (_engaged) {
		if (!ijcInTheLoop()) {
			// the standard PX4 controllers command the allocator
			vehicle_thrust_setpoint_s thrust_sp;
			vehicle_torque_setpoint_s torque_sp;

			if (_vehicle_thrust_setpoint_sub.update(&thrust_sp)) {
				_thrust_norm_sp = -thrust_sp.xyz[2];
			}

			if (_vehicle_torque_setpoint_sub.update(&torque_sp)) {
				_torque_norm_sp = Vector3f(torque_sp.xyz);
			}
		}

		Vector3f unallocated_torque{};
		float unallocated_thrust_z = 0.f;

		if (_allocator_status.timestamp != 0 && hrt_elapsed_time(&_allocator_status.timestamp) < kAllocatorStatusTimeout) {
			unallocated_torque = Vector3f(_allocator_status.unallocated_torque);
			unallocated_thrust_z = _allocator_status.unallocated_thrust[2];
		}

		_tau_applied = (_torque_norm_sp - unallocated_torque).emult(_tau_max);
		_f_applied = math::max(_thrust_norm_sp + unallocated_thrust_z, 0.f) * _f_max;

	} else {
		_tau_applied.zero();
		_f_applied = 0.f;
	}

	// actuator (rotor speed) dynamics: first-order lag, exact discretization
	const float tau_mot = _param_tau_mot.get();

	if (tau_mot > 0.f && _dt_att > 0.f && _engaged) {
		const float a = expf(-_dt_att / tau_mot);
		_tau_model = _tau_model * a + _tau_applied * (1.f - a);
		_f_model = _f_model * a + _f_applied * (1.f - a);

	} else {
		_tau_model = _tau_applied;
		_f_model = _f_applied;
	}

	// ---- observers (Lemma 3.1)
	if (_observers_active && _dt_att > 0.f) {
		const Vector3f phi_omega = (_tau_model - _omega.cross(_J.emult(_omega))).edivide(_J);
		_obs_rot.update(phi_omega, _omega, _dt_att);

		// applied force in the body frame: thrust along -z plus the lateral force that canted rotors
		// produce together with the roll/pitch torques
		const Vector3f torque_norm_model = _tau_model.edivide(_tau_max);
		const Vector3f F_body(_param_fx_ty.get() * torque_norm_model(1), _param_fy_tx.get() * torque_norm_model(0), -_f_model);
		_thrust_acc_integral -= Dcmf(_q) * F_body * (_dt_att / _mass);
		_thrust_acc_time += _dt_att;
	}

	if (new_position) {
		if (_observers_active && _thrust_acc_time > 0.f && _dt_pos > 0.f) {
			// phi_v = g e3 - (f/m) R e3 (+ lateral rotor force), averaged over the position sample interval
			const Vector3f phi_v = Vector3f(0.f, 0.f, kGravity) - _thrust_acc_integral / _thrust_acc_time;
			_obs_trans.update(phi_v, _v, _dt_pos);
		}

		_thrust_acc_integral.zero();
		_thrust_acc_time = 0.f;
	}

	if (!_observers_active) {
		resetObservers();
	}

	// ---- state machine and reference
	const State prev_state = _state;
	updateReference(now);

	// ---- disturbance injection (S1)
	if (_scenario_running && _state == State::Hover) {
		computeDisturbance((now - _t_scen_start) * 1e-6f, _F_dist, _T_dist);

	} else {
		_F_dist.zero();
		_T_dist.zero();
	}

	ijc_wrench_s wrench{};
	wrench.timestamp = hrt_absolute_time();
	(_F_dist * _mass).copyTo(wrench.force_ned);
	_J.emult(_T_dist).copyTo(wrench.torque_frd);
	_ijc_wrench_pub.publish(wrench);

	// ---- reference for the loop (also consumed by mc_pos_control when IJC_MODE = 1)
	if (_engaged && (new_position || prev_state != _state)) {
		publishTrajectorySetpoint();
	}

	// ---- control laws
	if (_engaged && !ijcInTheLoop()) {
		// standard PX4 controllers in the loop: log their commands in physical units
		_f = _thrust_norm_sp * _f_max;
		_tau = _torque_norm_sp.emult(_tau_max);
		vehicle_attitude_setpoint_s att_sp;

		if (_vehicle_attitude_setpoint_sub.update(&att_sp)) {
			_q_d = Quatf(att_sp.q_d);
		}

		_q_bar = _q_d.inversed() * _q;

		if (_q_bar(0) < 0.f) {
			_q_bar = -_q_bar;
		}

		_mu = Dcmf(_q_d).col(2) * _f;

	} else if (_engaged) {
		if (_state == State::Touchdown) {
			const float t_td = (now > _t_state_start) ? (now - _t_state_start) * 1e-6f : 0.f;
			_f = _touchdown_f0 * math::max(1.f - t_td / kTouchdownRampTime, 0.f);
			_tau.zero();

		} else {
			if (new_position || prev_state != _state) {
				positionControl();
			}

			attitudeControl();
		}

		_thrust_norm_sp = math::constrain(_f / _f_max, 0.f, 1.f);

		for (int i = 0; i < 3; i++) {
			_torque_norm_sp(i) = math::constrain(_tau(i) / _tau_max(i), -1.f, 1.f);
		}

		vehicle_thrust_setpoint_s thrust_setpoint{};
		thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		thrust_setpoint.xyz[2] = -_thrust_norm_sp;
		thrust_setpoint.timestamp = hrt_absolute_time();
		_vehicle_thrust_setpoint_pub.publish(thrust_setpoint);

		vehicle_torque_setpoint_s torque_setpoint{};
		torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		_torque_norm_sp.copyTo(torque_setpoint.xyz);
		torque_setpoint.timestamp = hrt_absolute_time();
		_vehicle_torque_setpoint_pub.publish(torque_setpoint);

	} else {
		_thrust_norm_sp = 0.f;
		_torque_norm_sp.zero();
		_f = 0.f;
		_tau.zero();
		_q_d = _q;
		_q_bar = Quatf();
		_mu.zero();
	}

	// ---- offboard heartbeat: thrust and torque from this module, or position setpoints for the PX4 controllers
	if (hrt_elapsed_time(&_last_offboard_pub) > 20_ms) {
		offboard_control_mode_s ocm{};
		ocm.thrust_and_torque = ijcInTheLoop();
		ocm.position = !ijcInTheLoop();
		ocm.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(ocm);
		_last_offboard_pub = ocm.timestamp;
	}

	// ---- debug / log
	ijc_debug_s dbg{};
	dbg.timestamp_sample = angular_velocity.timestamp_sample;
	dbg.state = static_cast<uint8_t>(_state);
	dbg.comp = _param_comp.get() == 1;
	dbg.observers_active = _observers_active;
	dbg.t_scen = _scenario_running ? (now - _t_scen_start) * 1e-6f : -1.f;
	_p.copyTo(dbg.p);
	_v.copyTo(dbg.v);
	_p_d.copyTo(dbg.p_d);
	_v_d.copyTo(dbg.v_d);
	_a_d.copyTo(dbg.a_d);
	(_p - _p_d).copyTo(dbg.e_p);
	(_v - _v_d).copyTo(dbg.e_v);
	_obs_trans.y_hat().copyTo(dbg.v_hat);
	_obs_trans.d_hat().copyTo(dbg.f_hat);
	_omega.copyTo(dbg.omega);
	_obs_rot.y_hat().copyTo(dbg.omega_hat);
	_obs_rot.d_hat().copyTo(dbg.t_hat);
	_q.copyTo(dbg.q);
	_q_d.copyTo(dbg.q_d);
	_q_bar.copyTo(dbg.q_bar);
	_mu.copyTo(dbg.mu);
	dbg.f = _f;
	_tau.copyTo(dbg.tau);
	dbg.f_applied = _f_applied;
	_tau_applied.copyTo(dbg.tau_applied);
	dbg.f_model = _f_model;
	_f_lat_ff.copyTo(dbg.f_lat_ff);
	dbg.mode = static_cast<uint8_t>(_param_mode.get());
	_tau_model.copyTo(dbg.tau_model);
	dbg.thrust_norm = _thrust_norm_sp;
	_torque_norm_sp.copyTo(dbg.torque_norm);
	_F_dist.copyTo(dbg.f_applied_dist);
	_T_dist.copyTo(dbg.t_applied_dist);
	dbg.dt_att = _dt_att;
	dbg.dt_pos = _dt_pos;
	dbg.timestamp = hrt_absolute_time();
	_ijc_debug_pub.publish(dbg);

	// compact copy at 20 Hz for live monitoring over MAVLink (DEBUG_FLOAT_ARRAY "IJC", scripts/demo.sh):
	// [t_scen, p_d(3), p(3), e_p(3), state, IJC_MODE, IJC_COMP]
	if (now - _last_debug_array_pub >= 50_ms) {
		debug_array_s arr{};
		arr.id = 1;
		strncpy(arr.name, "IJC", sizeof(arr.name));
		arr.data[0] = dbg.t_scen;

		for (int i = 0; i < 3; i++) {
			arr.data[1 + i] = _p_d(i);
			arr.data[4 + i] = _p(i);
			arr.data[7 + i] = _p(i) - _p_d(i);
		}

		arr.data[10] = static_cast<float>(_state);
		arr.data[11] = static_cast<float>(_param_mode.get());
		arr.data[12] = static_cast<float>(_param_comp.get());
		arr.timestamp = hrt_absolute_time();
		_debug_array_pub.publish(arr);
		_last_debug_array_pub = now;
	}

	perf_end(_loop_perf);
}

int IjcControl::task_spawn(int argc, char *argv[])
{
	IjcControl *instance = new IjcControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int IjcControl::print_status()
{
	static constexpr const char *state_names[] = {"idle", "takeoff", "hover", "land", "touchdown"};
	PX4_INFO("state: %s, observers: %s, compensation: %s, scenario: %s",
		 state_names[static_cast<int>(_state)], _observers_active ? "on" : "off",
		 _param_comp.get() == 1 ? "on" : "off", _scenario_running ? "running" : "off");
	PX4_INFO("F_hat: [%.3f %.3f %.3f] m/s^2", (double)_obs_trans.d_hat()(0), (double)_obs_trans.d_hat()(1),
		 (double)_obs_trans.d_hat()(2));
	PX4_INFO("T_hat: [%.3f %.3f %.3f] rad/s^2", (double)_obs_rot.d_hat()(0), (double)_obs_rot.d_hat()(1),
		 (double)_obs_rot.d_hat()(2));
	PX4_INFO("f: %.3f N (norm %.3f), tau: [%.3f %.3f %.3f] N m", (double)_f, (double)_thrust_norm_sp,
		 (double)_tau(0), (double)_tau(1), (double)_tau(2));
	PX4_INFO("e_p: [%.3f %.3f %.3f] m, dt_att %.4f s, dt_pos %.4f s", (double)(_p - _p_d)(0), (double)(_p - _p_d)(1),
		 (double)(_p - _p_d)(2), (double)_dt_att, (double)_dt_pos);
	perf_print_counter(_loop_perf);
	return 0;
}

int IjcControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int IjcControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Observer-based disturbance-compensated position and attitude controller for
tail-sitter UAVs in hover (IJC revision). Replaces mc_pos_control,
mc_att_control and mc_rate_control; publishes normalized thrust and torque
setpoints to the control allocator while the vehicle is armed in Offboard mode
(thrust_and_torque), including the takeoff and landing references.

Flight commands are given with the IJC_CMD parameter (0 hover, 1 start
scenario, 2 land). IJC_COMP switches the disturbance compensation.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ijc_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ijc_control_main(int argc, char *argv[])
{
	return IjcControl::main(argc, argv);
}
