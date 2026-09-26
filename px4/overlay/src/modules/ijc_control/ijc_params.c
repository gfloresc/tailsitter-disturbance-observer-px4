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
 * @file ijc_params.c
 *
 * Parameters of the observer-based disturbance-compensated tail-sitter
 * controller (ijc_control). Default gains are those of the paper
 * (Tables 2-3); default vehicle constants are those of the Gazebo
 * quadtailsitter model (see scripts/normalization.py).
 */

/**
 * Position gain k_p
 *
 * @min 0.0
 * @decimal 2
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_KP, 4.0f);

/**
 * Velocity gain k_v
 *
 * @min 0.0
 * @decimal 2
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_KV, 4.0f);

/**
 * Attitude gain k_q
 *
 * @min 0.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_KQ, 1600.0f);

/**
 * Angular velocity gain K_Omega (same on the three axes)
 *
 * @min 0.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_KOM, 60.0f);

/**
 * Rotational observer gain K1
 *
 * @min 0.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_K1, 300.0f);

/**
 * Rotational observer gain L1
 *
 * @min 0.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_L1, 22500.0f);

/**
 * Translational observer gain K2
 *
 * @min 0.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_K2, 40.0f);

/**
 * Translational observer gain L2
 *
 * @min 0.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_L2, 400.0f);

/**
 * Actuator time constant used for the applied thrust/torque in the observers
 *
 * The observers' known terms phi use the applied thrust and torque. With a
 * value > 0 the applied (post-saturation) setpoints are passed through a
 * first-order lag with this time constant, a model of the rotor speed
 * dynamics, before entering phi. 0 reproduces the paper (instantaneous
 * actuators).
 *
 * @unit s
 * @min 0.0
 * @max 0.2
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TAU_MOT, 0.0f);

/**
 * Controller in the loop
 *
 * 0: ijc_control computes thrust and torques (the paper's laws).
 * 1: comparison with the standard PX4 multicopter controllers: ijc_control only
 * provides the same reference (trajectory_setpoint, Offboard position), injects the
 * same disturbances and runs the observers for logging; thrust and torques come from
 * mc_pos_control / mc_att_control / mc_rate_control (airframe 4051).
 *
 * @value 0 ijc_control
 * @value 1 PX4 standard controllers
 * @group IJC Control
 */
PARAM_DEFINE_INT32(IJC_MODE, 0);

/**
 * Feedforward of the canted-rotor lateral force in mu
 *
 * The lateral force that canted rotors produce together with the control torques
 * is excluded from F_hat (it is a known part of the applied force, see IJC_FX_TY).
 * This term adds it as a feedforward in the position law, mu += R F_lat(tau_ff),
 * low-pass filtered with IJC_FLAT_TC: unfiltered, it closes a fast positive loop
 * q_d -> tau -> F_lat -> mu -> q_d (roll limit cycle on the canted-rotor vehicle).
 *
 * @value 0 Off
 * @value 1 From the disturbance-compensation torque tau_ff = -J T_hat (compensation on only)
 * @value 2 From the total applied torque (known model term, also without compensation)
 * @group IJC Control
 */
PARAM_DEFINE_INT32(IJC_FLAT_FF, 0);

/**
 * Time constant of the low-pass filter of the lateral-force feedforward
 *
 * 0 disables the filter.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_FLAT_TC, 0.5f);

/**
 * Disturbance compensation switch
 *
 * 1: F_hat and T_hat are used in the control laws. 0: they are forced to zero in
 * the control laws only (the observers keep running and are logged).
 *
 * @boolean
 * @group IJC Control
 */
PARAM_DEFINE_INT32(IJC_COMP, 1);

/**
 * Desired (constant) heading psi_d
 *
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_PSI_D, 0.0f);

/**
 * Vehicle mass m
 *
 * @unit kg
 * @min 0.01
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_MASS, 1.635f);

/**
 * Vehicle inertia J_xx (body FRD, about the CoM)
 *
 * @unit kg m^2
 * @min 0.00001
 * @decimal 5
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_JXX, 0.11535f);

/**
 * Vehicle inertia J_yy (body FRD, about the CoM)
 *
 * @unit kg m^2
 * @min 0.00001
 * @decimal 5
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_JYY, 0.03224f);

/**
 * Vehicle inertia J_zz (body FRD, about the CoM)
 *
 * @unit kg m^2
 * @min 0.00001
 * @decimal 5
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_JZZ, 0.08568f);

/**
 * Collective thrust at normalized thrust 1
 *
 * thrust_norm = f / IJC_FMAX. Derived from the rotor model and the allocator
 * normalization (scripts/normalization.py).
 *
 * @unit N
 * @min 0.1
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_FMAX, 47.8305f);

/**
 * Roll torque at normalized torque 1
 *
 * @unit Nm
 * @min 0.001
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TMAX_X, 4.8169f);

/**
 * Pitch torque at normalized torque 1
 *
 * @unit Nm
 * @min 0.001
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TMAX_Y, 4.5947f);

/**
 * Yaw torque at normalized torque 1
 *
 * @unit Nm
 * @min 0.001
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TMAX_Z, 5.9662f);

/**
 * Body x force per unit normalized pitch torque
 *
 * Rotors with canted axes produce a lateral force together with a control
 * torque. This known part of the applied force is included in the
 * translational observer (phi_v), so that F_hat estimates only external
 * disturbances. 0 for rotors parallel to the body z axis.
 *
 * @unit N
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_FX_TY, 0.0f);

/**
 * Body y force per unit normalized roll torque
 *
 * See IJC_FX_TY.
 *
 * @unit N
 * @decimal 4
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_FY_TX, 0.0f);

/**
 * Maximum tilt of mu (safeguard on the horizontal part of mu)
 *
 * @unit deg
 * @min 1.0
 * @max 89.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TILT_MAX, 45.0f);

/**
 * Minimum vertical component of mu, as a fraction of m g
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_MUZ_MIN, 0.3f);

/**
 * Takeoff height above the arming position
 *
 * @unit m
 * @min 0.5
 * @max 20.0
 * @decimal 2
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TKO_ALT, 2.5f);

/**
 * Takeoff duration (quintic reference)
 *
 * @unit s
 * @min 1.0
 * @max 30.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TKO_T, 5.0f);

/**
 * Landing descent speed
 *
 * @unit m/s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_LND_VEL, 0.5f);

/**
 * Flight command
 *
 * Set by the experiment script over MAVLink while hovering.
 * The module resets it to 0 at startup and when armed.
 *
 * @value 0 Hover
 * @value 1 Start scenario (scenario timer t = 0)
 * @value 2 Land
 * @group IJC Control
 */
PARAM_DEFINE_INT32(IJC_CMD, 0);

/**
 * Disturbance injection
 *
 * @value 0 None
 * @value 1 S1 synthetic force/torque signals (physical wrench applied in Gazebo)
 * @value 2 S1r: randomized S1-like signals (IJC_DSEED), same amplitude class, for validation
 * @group IJC Control
 */
PARAM_DEFINE_INT32(IJC_DIST, 0);

/**
 * Seed of the randomized disturbance (IJC_DIST = 2)
 *
 * @min 1
 * @max 1000000
 * @group IJC Control
 */
PARAM_DEFINE_INT32(IJC_DSEED, 1);

/**
 * Scenario reference
 *
 * @value 0 Hover at the takeoff point
 * @value 1 Circle (S3): radius IJC_TRJ_R, period IJC_TRJ_T, constant altitude, starting at the hover point
 * @group IJC Control
 */
PARAM_DEFINE_INT32(IJC_TRAJ, 0);

/**
 * Circle radius
 *
 * @unit m
 * @min 0.1
 * @max 20.0
 * @decimal 2
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TRJ_R, 2.0f);

/**
 * Circle period
 *
 * @unit s
 * @min 2.0
 * @max 120.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TRJ_T, 20.0f);

/**
 * Circle start ramp (angular velocity ramps linearly from 0 during this time)
 *
 * @unit s
 * @min 0.1
 * @max 30.0
 * @decimal 1
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_TRJ_TR, 5.0f);

/**
 * Force disturbance amplitude A_F (as acceleration)
 *
 * @unit m/s^2
 * @min 0.0
 * @decimal 3
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_AF, 2.942f);

/**
 * Torque disturbance amplitude A_T, roll axis (as angular acceleration)
 *
 * @unit rad/s^2
 * @min 0.0
 * @decimal 3
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_ATX, 9.0f);

/**
 * Torque disturbance amplitude A_T, pitch axis (as angular acceleration)
 *
 * @unit rad/s^2
 * @min 0.0
 * @decimal 3
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_ATY, 20.0f);

/**
 * Torque disturbance amplitude A_T, yaw axis (as angular acceleration)
 *
 * @unit rad/s^2
 * @min 0.0
 * @decimal 3
 * @group IJC Control
 */
PARAM_DEFINE_FLOAT(IJC_ATZ, 7.0f);
