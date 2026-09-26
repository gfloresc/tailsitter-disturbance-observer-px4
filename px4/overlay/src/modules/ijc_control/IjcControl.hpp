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

/**
 * @file IjcControl.hpp
 *
 * Observer-based disturbance-compensated position and attitude control of
 * tail-sitter UAVs in hover (IJC revision). Replaces the whole multicopter
 * control cascade: position law (Prop. 4.1), desired attitude (eqs. 20-24),
 * attitude law (Prop. 4.2) and the two disturbance observers (Lemma 3.1).
 * Outputs normalized thrust/torque to the control allocator; the vehicle is
 * flown in Offboard mode (thrust_and_torque), including takeoff and landing.
 */

#pragma once

#include "DisturbanceObserver.hpp"

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/ijc_debug.h>
#include <uORB/topics/ijc_wrench.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class IjcControl : public ModuleBase<IjcControl>, public ModuleParams, public px4::WorkItem
{
public:
	IjcControl();
	~IjcControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	void parametersUpdated();
	void resetObservers();
	void updateReference(hrt_abstime now);
	void positionControl();
	void attitudeControl();
	void computeDisturbance(float t, matrix::Vector3f &F, matrix::Vector3f &T) const;
	void computeRandomDisturbance(float t, matrix::Vector3f &F, matrix::Vector3f &T) const;
	void setCommand(int32_t cmd);
	void publishTrajectorySetpoint();
	void circleReference(float t);
	bool ijcInTheLoop() const { return _param_mode.get() == 0; }

	// uORB
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	// IJC_MODE 1: setpoints of the standard PX4 controllers
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 100_ms};

	uORB::Publication<ijc_debug_s>                _ijc_debug_pub{ORB_ID(ijc_debug)};
	uORB::Publication<debug_array_s>              _debug_array_pub{ORB_ID(debug_array)};
	uORB::Publication<ijc_wrench_s>               _ijc_wrench_pub{ORB_ID(ijc_wrench)};
	uORB::Publication<offboard_control_mode_s>    _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s>      _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>  _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>  _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	vehicle_control_mode_s _control_mode{};
	vehicle_land_detected_s _land_detected{};
	control_allocator_status_s _allocator_status{};

	enum class State : uint8_t {
		Idle = ijc_debug_s::STATE_IDLE,
		Takeoff = ijc_debug_s::STATE_TAKEOFF,
		Hover = ijc_debug_s::STATE_HOVER,
		Land = ijc_debug_s::STATE_LAND,
		Touchdown = ijc_debug_s::STATE_TOUCHDOWN,
	} _state{State::Idle};

	// measurements
	matrix::Vector3f _p{};
	matrix::Vector3f _v{};
	matrix::Quatf _q{};
	matrix::Vector3f _omega{};
	bool _local_position_valid{false};
	hrt_abstime _last_att_sample{0};
	hrt_abstime _last_pos_sample{0};
	float _dt_att{0.f};
	float _dt_pos{0.f};

	// reference
	matrix::Vector3f _p0{};        ///< position at arming (ground)
	matrix::Vector3f _p_hover{};   ///< hover point
	matrix::Vector3f _p_d{};
	matrix::Vector3f _v_d{};
	matrix::Vector3f _a_d{};
	hrt_abstime _t_state_start{0};
	hrt_abstime _t_scen_start{0};
	bool _scenario_running{false};
	float _land_z_start{0.f};
	matrix::Vector2f _land_xy{};    ///< horizontal reference during landing
	float _touchdown_f0{0.f};
	hrt_abstime _touchdown_candidate{0};

	// observers
	DisturbanceObserver _obs_rot;   ///< y = Omega, d = T
	DisturbanceObserver _obs_trans; ///< y = v, d = F
	bool _observers_active{false};
	matrix::Vector3f _thrust_acc_integral{}; ///< int R (f e3 - F_lat) / m dt since last position sample
	float _thrust_acc_time{0.f};

	// control
	matrix::Vector3f _mu{};
	float _f{0.f};
	matrix::Quatf _q_d{};
	matrix::Quatf _q_bar{};
	matrix::Vector3f _tau{};
	bool _engaged{false};           ///< armed in Offboard and flying the ijc reference (any IJC_MODE)
	matrix::Vector3f _f_lat_ff{};      ///< lateral rotor force feedforward in mu [N] (inertial, filtered)
	matrix::Vector3f _torque_norm_sp{};
	float _thrust_norm_sp{0.f};
	float _f_applied{0.f};
	matrix::Vector3f _tau_applied{};
	float _f_model{0.f};              ///< applied thrust through the actuator model (used in phi)
	matrix::Vector3f _tau_model{};    ///< applied torque through the actuator model (used in phi)

	// disturbance injection
	matrix::Vector3f _F_dist{};
	matrix::Vector3f _T_dist{};

	// cached parameters
	float _mass{1.f};
	matrix::Vector3f _J{1.f, 1.f, 1.f};
	matrix::Vector3f _tau_max{1.f, 1.f, 1.f};
	float _f_max{1.f};
	int32_t _last_cmd{0};

	hrt_abstime _last_offboard_pub{0};
	hrt_abstime _last_debug_array_pub{0};

	perf_counter_t _loop_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IJC_KP>) _param_kp,
		(ParamFloat<px4::params::IJC_KV>) _param_kv,
		(ParamFloat<px4::params::IJC_KQ>) _param_kq,
		(ParamFloat<px4::params::IJC_KOM>) _param_kom,
		(ParamFloat<px4::params::IJC_K1>) _param_k1,
		(ParamFloat<px4::params::IJC_L1>) _param_l1,
		(ParamFloat<px4::params::IJC_K2>) _param_k2,
		(ParamFloat<px4::params::IJC_L2>) _param_l2,
		(ParamFloat<px4::params::IJC_TAU_MOT>) _param_tau_mot,
		(ParamInt<px4::params::IJC_MODE>) _param_mode,
		(ParamInt<px4::params::IJC_FLAT_FF>) _param_flat_ff,
		(ParamFloat<px4::params::IJC_FLAT_TC>) _param_flat_tc,
		(ParamInt<px4::params::IJC_COMP>) _param_comp,
		(ParamFloat<px4::params::IJC_PSI_D>) _param_psi_d,
		(ParamFloat<px4::params::IJC_MASS>) _param_mass,
		(ParamFloat<px4::params::IJC_JXX>) _param_jxx,
		(ParamFloat<px4::params::IJC_JYY>) _param_jyy,
		(ParamFloat<px4::params::IJC_JZZ>) _param_jzz,
		(ParamFloat<px4::params::IJC_FMAX>) _param_fmax,
		(ParamFloat<px4::params::IJC_TMAX_X>) _param_tmax_x,
		(ParamFloat<px4::params::IJC_TMAX_Y>) _param_tmax_y,
		(ParamFloat<px4::params::IJC_TMAX_Z>) _param_tmax_z,
		(ParamFloat<px4::params::IJC_FX_TY>) _param_fx_ty,
		(ParamFloat<px4::params::IJC_FY_TX>) _param_fy_tx,
		(ParamFloat<px4::params::IJC_TILT_MAX>) _param_tilt_max,
		(ParamFloat<px4::params::IJC_MUZ_MIN>) _param_muz_min,
		(ParamFloat<px4::params::IJC_TKO_ALT>) _param_tko_alt,
		(ParamFloat<px4::params::IJC_TKO_T>) _param_tko_t,
		(ParamFloat<px4::params::IJC_LND_VEL>) _param_lnd_vel,
		(ParamInt<px4::params::IJC_CMD>) _param_cmd,
		(ParamInt<px4::params::IJC_DIST>) _param_dist,
		(ParamInt<px4::params::IJC_TRAJ>) _param_traj,
		(ParamFloat<px4::params::IJC_TRJ_R>) _param_trj_r,
		(ParamFloat<px4::params::IJC_TRJ_T>) _param_trj_t,
		(ParamFloat<px4::params::IJC_TRJ_TR>) _param_trj_tr,
		(ParamInt<px4::params::IJC_DSEED>) _param_dseed,
		(ParamFloat<px4::params::IJC_AF>) _param_af,
		(ParamFloat<px4::params::IJC_ATX>) _param_atx,
		(ParamFloat<px4::params::IJC_ATY>) _param_aty,
		(ParamFloat<px4::params::IJC_ATZ>) _param_atz
	)
};
