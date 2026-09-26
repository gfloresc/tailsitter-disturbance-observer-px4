# Technical notes: implementation, gain selection, methodology, results and deviations

Detailed notes of the PX4 SITL implementation. Installation and reproduction steps are in the
[README](../README.md); paths below are relative to the repository root.

## Overview

SITL implementation, in PX4 + Gazebo, of the control scheme of *"Observer-Based
Disturbance-Compensated Position and Attitude Control of Tail-Sitter UAVs"*
(spec: `spec/SPEC_px4_ijc.md`).

Only the paper's laws are in the loop: disturbance observers without leakage (Lemma 3.1),
position law with `F_hat` (Prop. 4.1), desired attitude `Q_d = Q_a ⊗ Q_r` with constant
`psi_d` (eqs. 20-24) and attitude law with `T_hat` (Prop. 4.2, no `Omega_d` feedforward).
No standard PX4 control law (position PID, attitude P, rate PID) runs when `ijc_control`
flies, not even for takeoff or landing. PX4 provides the infrastructure: uORB, the `matrix`
library, EKF2, commander, control allocator, land detector and logger.

## What was added to PX4

| Item | Files |
|---|---|
| Module `ijc_control` (1 kHz, `rate_ctrl` work queue, on `vehicle_angular_velocity`) | `src/modules/ijc_control/` (`IjcControl.*`, `DisturbanceObserver.hpp`, `ijc_params.c`) |
| uORB topics `ijc_debug` (all controller signals) and `ijc_wrench` (disturbance request) | `msg/IjcDebug.msg`, `msg/IjcWrench.msg` |
| Airframe 4050 (`ijc_control` in the loop) and apps script without standard controllers | `ROMFS/.../init.d-posix/airframes/4050_gz_quadtailsitter_ijc`, `ROMFS/.../init.d/rc.mc_ijc_apps`, `rc.vehicle_setup` (`VEHICLE_TYPE mc_ijc`) |
| Airframe 4051 (same vehicle, standard PX4 controllers, comparison case) | `.../airframes/4051_gz_quadtailsitter_px4{,.post}` |
| Disturbance injection: `gz_bridge` forwards `ijc_wrench` to a Gazebo system that applies the wrench at the CoM | `src/modules/simulation/gz_bridge/GZBridge.*`, `src/modules/simulation/gz_plugins/ijc_wrench/` |
| World (1 kHz physics, g = 9.80665, `IjcWrenchSystem`) and model (quadtailsitter with 1 kHz IMU) | `Tools/simulation/gz/worlds/ijc_calm.sdf`, `Tools/simulation/gz/models/quadtailsitter_ijc/` |
| Logging: `ijc_debug` and controller topics at full rate; logger triggered by `ijc_debug` | `src/modules/logger/logged_topics.cpp`, `init.d-posix/rcS` (`LOGGER_POLL_TOPIC`) |
| Measured aerodynamic wrench: Gazebo system `IjcAeroSystem` -> `gz_bridge` -> uORB `ijc_aero` | `src/modules/simulation/gz_plugins/ijc_wrench/IjcAeroSystem.cpp`, `msg/IjcAero.msg` |
| Wind world (`WindEffects`) | `Tools/simulation/gz/worlds/ijc_wind.sdf` |
| Magnetometer from `sensor_mag_sim` instead of the gz magnetometer (D15) | `quadtailsitter_ijc/model.sdf`, airframe 4050 (`SENS_EN_MAGSIM`) |

`ijc_control` flies the whole flight in Offboard mode (`thrust_and_torque`): after arming
it follows a quintic takeoff reference (2.5 m in 5 s), hovers, runs the scenario when the
parameter `IJC_CMD` is set to 1 (scenario time t = 0) and lands when `IJC_CMD = 2`.
The references (`p_d, v_d, a_d, psi_d`) are generated in the module and published on
`trajectory_setpoint`. `IJC_COMP` switches the compensation: with 0, `F_hat` and `T_hat` are
forced to zero in the control laws only; the observers keep running and are logged.

## Normalization (thrust/torque to the control allocator)

`scripts/normalization.py` reproduces the allocator (effectiveness from the `CA_*`
parameters, pseudo-inverse, per-axis normalization of `ControlAllocationPseudoInverse`)
and maps its rotor commands through the physical rotor model of the SDF (positions, canted
axes, `motorConstant`, `momentConstant`, `maxRotVelocity`, spin directions). With
`THR_MDL_FAC = 1` and `SIM_GZ_EC_MIN/MAX = 0/1200` (= `maxRotVelocity`), rotor thrust is
linear in the allocator output and the map is exact:

```
f = IJC_FMAX * thrust_norm                IJC_FMAX = 47.8305 N
tau_i = IJC_TMAX_i * torque_norm_i        IJC_TMAX = [4.8169, 4.5947, 5.9662] N m
```

The `[tau_x, tau_y, tau_z, f]` block of the map is exactly diagonal. Verified in flight:
hover thrust 16.03 N = m g (m = 1.635 kg) and `thrust_norm = 0.335 = MPC_THR_HOVER`.
Inertia (composite, about the CoM, FRD): J = diag(0.11535, 0.03224, 0.08568) kg m².

## Running the experiments

```bash
cd scripts
./run_S1.sh          # S1, A_T = 30 %: 4 flights (compensated, non compensated, PX4 tuned, PX4 shipped) + figures + metrics
./run_S1.sh high     # S1h, A_T = 50 %
python3 tune.py search ijc; python3 tune.py search px4    # gain searches (about 1.5 h at 4x real time)
./run_validation.sh  # S1r validation of the selected gains
./run_S2S3_reps.sh S2 5 4   # hover under wind: 4 cases x 5 repetitions at 4x real time
./run_S2S3_reps.sh S3 5 4   # circle under wind
python3 postprocess.py S2 --reps ../logs/S2 --tend 60    # mean +- std, figures of repetition 1
python3 postprocess.py S3 --reps ../logs/S3 --tend 60    # (+ top view sitl_S3_xy.pdf)
./run_all_final.sh          # everything above plus S1/S1h/S1r and the real-time equivalence runs
```

Single flights: `./sitl.sh start [world] [4050|4051]`, then
`python3 ijc_flight.py --name <run> --param IJC_COMP=1 --param IJC_DIST=1 ... --settle 10 --duration 25`,
`./sitl.sh stop`. Each run starts from a fresh SITL with the airframe default parameters
(parameters saved by previous runs are deleted). Logs go to `logs/<run>.ulg`.

Figures and metrics: `python3 postprocess.py S1 ../logs/S1_comp.ulg ../logs/S1_nocomp.ulg --px4 ../logs/S1_px4.ulg --px4-shipped ../logs/S1_px4ship.ulg`
(writes `results/figures/`, `results/metrics.csv`, `results/metrics_table.tex`);
`python3 write_params.py ../logs/S1_comp.ulg ...` writes `results/params.txt`.
The reference simulation is in `spec/sim_python/` (`python3 simnb.py && python3 plots.py`; its figures are in `reference/`).
Time series of the figures as CSV (one file per scenario and case, 1 kHz): `python3 export_plotdata.py`
-> `results/plotdata/` (columns described in `results/plotdata/README.md`; not versioned because of
their size, ~185 MB). `results/` (with these CSV files) is also distributed as `results_data.zip` in the Release.

## Gain selection

**Criterion** (lower is better), evaluated on S1 (A_T = 30 %), t = 0-20 s, by `scripts/tune.py`:

```
J = RMS|e_p| / 0.10 m  +  RMS_att(a) / 1 deg  +  0.5 * (% of time the allocator could not realize the setpoint)
```

`RMS|e_p|` is the 3-D RMS position error; `RMS_att(a)` the RMS over the three axes of the attitude
error while each torque disturbance acts, excluding 0.5 s after the `F_x` jumps (metric (a), D12).
A candidate is rejected (J = inf) if the flight does not complete the scenario, if |e_p| > 3 m or if
the hover before the scenario is oscillatory (RMS attitude error > 1 deg in its last 5 s).

**Constraints for `ijc_control`** (time-scale hierarchy of Theorem 4.3 and stability). Parameterization:
attitude `theta'' + 2 zeta_a omega_a theta' + omega_a^2 theta = 0` (`k_q = 2 omega_a^2`, since
`q_bar ~ theta/2`; `K_Omega = 2 zeta_a omega_a`), observers with double poles at `-lambda_1`, `-lambda_2`
(`K = 2 lambda`, `L = lambda^2`), position `k_p = omega_p^2`, `k_v = 2 zeta_p omega_p`. With `p_a` the
slowest attitude pole: attitude and rotational observer faster than the position loop
(`p_a >= 3 omega_p`, `lambda_1 >= 3 omega_p`, `lambda_1 >= p_a`); translational observer slower than
the attitude and faster than the position loop (`1.5 omega_p <= lambda_2 <= p_a`); linear single-axis
model (attitude law + rotational observer + rotor lag 12.5-25 ms + delay 2-8 ms + gyro filter,
`scripts/linear_model.py`) stable with worst max Re(pole) < -1 1/s, with and without compensation.
The linear filter rejects `omega_a >= 18 rad/s`: the rotor dynamics limit the attitude bandwidth.

Both searches were run before the magnetometer fix (D15); in S1 the vehicle stays nearly level, so the
EKF yaw error was small (~5 deg), and the final runs with the fix confirm the selection (S1, S1r below).

**Search** (simulation at 4x real time, verified equivalent: baseline J = 2.66 +- 0.06 at 4x vs 2.73
in real time; run-to-run noise of J ~ +-0.1-0.2). Stages: (0) baseline x3; (1) `omega_a` x `zeta_a` x
`lambda_1`; (2) `lambda_2` x `omega_p`; (3) one-at-a-time refinement of `omega_a`, `lambda_1`,
`zeta_p`; (4) the best three repeated (5 evaluations each for the leaders). 44 flights for
`ijc_control`, 56 for PX4 (below). Full tables: `results/tuning/tuning_ijc.csv`, `tuning_px4.csv`.

| `ijc_control` candidate | J (mean +- std, n) | worst linear max Re (1/s) |
|---|---|---|
| `omega_a` = 14.14, `zeta_a` = 1.06, `lambda_1` = 80, `lambda_2` = 10, `omega_p` = 2 (previous S1) | 2.63 +- 0.06 (5) | -2.03 |
| same, `lambda_1` = 50 (**selected**) | 2.60 +- 0.11 (5) | -2.01 |
| `omega_a` = 15.56, `lambda_1` = 40 | 2.59 +- 0.19 (5) | -1.33 |
| `omega_a` = 10 (best of 6) | 3.54 (1) | |
| `zeta_a` = 0.8, `omega_a` = 14.14 | oscillatory hover | |
| `lambda_2` = 5-7, `omega_p` = 1.5-3 | 2.92-3.66 (1) | |

The J surface is flat around the previous gains: the three leaders are within the run-to-run
noise, i.e. the previous tuning was already at the actuator-limited optimum. `lambda_1 = 50` is
selected (lowest mean J among the candidates with worst linear max Re <= -2); the difference with
the previous gains is not statistically significant.

**Final gains of `ijc_control`** (defaults of airframe 4050; paper values for reference):

| Gain | Paper (m = 1, J = I) | Final | Poles / meaning |
|---|---|---|---|
| `k_p`, `k_v` | 4, 4 | 4, 4 | position: double pole at -2 |
| `k_q`, `K_Omega` | 1600, 60 | 400, 30 | attitude: `omega_a` = 14.1 rad/s, `zeta_a` = 1.06 (poles -10, -20) |
| `K1`, `L1` | 300, 22500 | 100, 2500 | rotational observer: double pole at -50 |
| `K2`, `L2` | 40, 400 | 20, 100 | translational observer: double pole at -10 |
| `IJC_TAU_MOT` | (0) | 25 ms | actuator model in `phi` (D2) |
| `IJC_FLAT_FF`, `IJC_FLAT_TC` | -- | 2, 0.5 s | lateral-force feedforward (D3) |

**Standard PX4 controllers.** Starting point: the tuning PX4 ships for the quadtailsitter
(`rc.vtol_defaults` + `4018_gz_quadtailsitter`: `MC_ROLL/PITCH_P = 3`, `MC_ROLL/PITCHRATE_P = 0.3`,
`MC_AIRMODE = 2`, `MC_YAW_P = 2`, rate limits 120 deg/s, `MPC_XY_VEL_MAX = 8`, ...). The previous S1
had used the generic multicopter defaults by mistake (`MC_ROLL_P = 4`, `MC_ROLLRATE_P = 0.15`,
`MC_AIRMODE = 0`); airframe 4051 now sets the shipped tuning, including `MAN_ARM_GESTURE = 0`, without
which PX4 resets `MC_AIRMODE` from 2 to 1 (as 4018 does). The search itself ran with `MC_AIRMODE = 1` (before
this fix); the final S1/S1h/S1r runs of both PX4 cases use `MC_AIRMODE = 2` (tuned: J 2.88 on S1, 3.24 on S1r). Search with the same criterion
(56 flights): (1) attitude P x roll/pitch rate gains (P, I, D scaled together); (2) `MPC_XY_P` x
`MPC_XY_VEL_P_ACC` x `MPC_XY_VEL_I_ACC`; (3) yaw and vertical loops, local refinement, yaw scale
extended past the edge of the grid; (4) repeats. Note that the shipped tuning was made for the stock
actuator mapping (quadratic thrust, ~1.4x larger incremental gain than the linear mapping used
here), which the search compensates.

| PX4 | J (mean +- std, n) |
|---|---|
| shipped quadtailsitter tuning | 18.12 +- 0.03 (3) |
| tuned: `MC_ROLL/PITCH_P = 9`, roll/pitch rate gains x2 (P 0.6, I 0.4, D 0.006), `MPC_XY_P = 2.5`, `MPC_XY_VEL_P_ACC = 3`, `MPC_XY_VEL_I_ACC = 3`, yaw x4 (`MC_YAW_P = 8`, `MC_YAWRATE_P = 0.8`, `MC_YAWRATE_I = 0.4`) | 3.03 +- 0.01 (3) |
| `MC_ROLL/PITCH_P = 11.25` or rate x3 or `MPC_XY_VEL_P_ACC = 4.5` | unstable / oscillatory |

**Validation on a different case, without retuning.** S1r (`IJC_DIST = 2`): same structure and
amplitude class as S1 (pulses, sinusoids, sawtooths on each axis) with pulse times, durations, signs,
frequencies, phases and periods drawn from a seed not used in the search (and a different sensor-noise
realization). `scripts/run_validation.sh`, `results/tuning/validation.csv` (runs after the magnetometer
fix, D15):

| Controller | J (seeds 11 / 23 / 37) | mean J | mean RMS\|e_p\| (m) | mean RMS att. (a) (deg) |
|---|---|---|---|---|
| ijc_control, compensated | 2.57 / 2.52 / 2.49 | 2.53 | 0.111 | 1.42 |
| ijc_control, non compensated | 6.05 / 6.19 / 5.87 | 6.04 | 0.280 | 3.23 |
| PX4 tuned | 3.12 / 3.25 / 3.35 | 3.24 | 0.161 | 1.61 |
| PX4 shipped tuning | 14.70 / 16.42 / 17.33 | 16.15 | 0.902 | 7.13 |

The ranking and the margins of S1 hold on S1r, so the selected gains are not overfitted to S1.

## Methodology of the result tables

- **Position error**: the main metric is the tracking error of the real vehicle, from Gazebo's ground
  truth (`vehicle_local_position_groundtruth`, ~60 Hz, origin aligned with the EKF at t = 0); the error
  seen by the controller (EKF) is also reported. Both are in `metrics.csv` (`gt_*` and plain columns).
- **S1, S1h**: one real-time flight per case. **S2, S3**: five flights per case, mean ± standard
  deviation (`metrics.csv`: `<metric>` and `<metric>_std`, `n`; single runs in `metrics_runs.csv`);
  the repetitions differ in the sensor-noise and wind-noise realizations. Figures show repetition 1.
- **Faster-than-real-time simulation (S2/S3 repetitions)**: flights at 4x real time
  (`PX4_SIM_SPEED_FACTOR = 4`; PX4 runs on the simulation clock). Checked against five real-time
  flights of S2 for the compensated and PX4-tuned cases (`results/tuning/speed_equivalence.csv`):
  the metrics seen by the controller (EKF position, attitude) differ by at most 15 % with |t| < 2.1;
  the ground-truth x error differs by -11 % (compensated) and +3 % (PX4 tuned) with |t| ~ 3, in
  opposite directions for the two cases (EKF-drift variability, not a systematic effect of the speed).
  The ranking and the ratios between controllers are the same.

<details><summary>Real time vs 4x (S2)</summary>

| Case | metric | real time (n = 5) | 4x (n = 5) | difference | Welch t |
|---|---|---|---|---|---|
| Compensated | `gt_rms_ep_x` | 0.100 ± 0.007 | 0.113 ± 0.006 | -11.2 % | -3.12 |
| Compensated | `gt_rms_ep_y` | 0.038 ± 0.010 | 0.033 ± 0.004 | +13.8 % | +0.95 |
| Compensated | `gt_rms_ep_z` | 0.164 ± 0.059 | 0.154 ± 0.068 | +6.1 % | +0.23 |
| Compensated | `rms_ep_x` | 0.055 ± 0.002 | 0.053 ± 0.004 | +3.7 % | +0.95 |
| Compensated | `rms_ep_y` | 0.018 ± 0.004 | 0.016 ± 0.003 | +9.7 % | +0.78 |
| Compensated | `rms_ep_z` | 0.076 ± 0.020 | 0.071 ± 0.023 | +7.3 % | +0.38 |
| Compensated | `rms_eatt_theta` | 0.832 ± 0.010 | 0.818 ± 0.012 | +1.7 % | +1.99 |
| Compensated | `rms_eatt_phi` | 0.236 ± 0.028 | 0.241 ± 0.040 | -2.0 % | -0.22 |
| Compensated | `rms_eatt_psi` | 0.073 ± 0.007 | 0.086 ± 0.012 | -14.6 % | -2.04 |
| PX4 tuned | `gt_rms_ep_x` | 0.199 ± 0.005 | 0.192 ± 0.002 | +3.3 % | +2.79 |
| PX4 tuned | `gt_rms_ep_y` | 0.028 ± 0.004 | 0.030 ± 0.004 | -6.6 % | -0.76 |
| PX4 tuned | `gt_rms_ep_z` | 0.227 ± 0.030 | 0.267 ± 0.049 | -15.2 % | -1.58 |
| PX4 tuned | `rms_ep_x` | 0.168 ± 0.003 | 0.168 ± 0.002 | +0.3 % | +0.31 |
| PX4 tuned | `rms_ep_y` | 0.015 ± 0.002 | 0.016 ± 0.001 | -5.7 % | -0.88 |
| PX4 tuned | `rms_ep_z` | 0.183 ± 0.003 | 0.192 ± 0.009 | -4.7 % | -2.08 |
| PX4 tuned | `rms_eatt_theta` | 0.552 ± 0.005 | 0.554 ± 0.002 | -0.4 % | -0.87 |
| PX4 tuned | `rms_eatt_phi` | 0.274 ± 0.010 | 0.269 ± 0.012 | +2.0 % | +0.79 |
| PX4 tuned | `rms_eatt_psi` | 0.089 ± 0.006 | 0.095 ± 0.004 | -5.5 % | -1.63 |

</details>

## Results: S1 (synthetic force/torque disturbances in hover)

Four cases with the same vehicle, reference and disturbances: `ijc_control` with compensation,
`ijc_control` without compensation (same controller and gains, `F_hat = T_hat = 0` in the laws), the
standard PX4 controllers tuned with the same criterion, and the standard PX4 controllers with the
tuning PX4 ships for the quadtailsitter. Window t = 0-20 s (disturbances start at t = 5 s), real time.
The figures show the first three cases (PX4 tuned in black dash-dot):
`results/figures/sitl_S1_{pos,att,inputs,inputs_cmp}.pdf`.

**S1, A_T = 30 % of the authority (main case).**

| Case | J | ground truth: max \|e_p\| x / y / z (m) | ground truth: RMS e_p x / y / z (m) | EKF: RMS e_p x / y / z (m) | att. error (a) θ / φ / ψ (deg) | bias (b) θ / φ / ψ (deg) | unallocated (%) |
|---|---|---|---|---|---|---|---|
| Compensated | 2.55 | 0.40 / 0.15 / 0.32 | 0.138 / 0.057 / 0.165 | 0.097 / 0.040 / 0.061 | 1.68 / 1.20 / 0.21 | 0.19 / 0.05 / 0.01 | 0.3 |
| Non compensated | 6.17 | 0.75 / 0.41 / 0.13 | 0.285 / 0.190 / 0.040 | 0.287 / 0.138 / 0.035 | 4.27 / 2.73 / 0.83 | 0.33 / 2.48 / 0.76 | 0.0 |
| PX4 tuned | 2.88 | 0.44 / 0.20 / 0.14 | 0.178 / 0.077 / 0.053 | 0.139 / 0.060 / 0.037 | 1.55 / 1.54 / 0.36 | -0.12 / 1.20 / 0.10 | 0.1 |
| PX4 shipped tuning | 17.96 | 1.76 / 1.38 / 0.16 | 0.703 / 0.513 / 0.073 | 0.700 / 0.519 / 0.034 | 7.58 / 13.78 / 3.00 | 0.84 / 14.78 / 1.23 | 0.0 |

J: tuning criterion (EKF position, attitude (a), saturation). Attitude error (a): RMS while each torque
disturbance acts, excluding 0.5 s after the `F_x` jumps; bias (b): mean error over the same intervals,
excluding 0.5 s after every disturbance discontinuity (D12).

- Compensation vs the same controller without it: ground-truth position error 2.1x (x) and
  3.3x (y) smaller in RMS; attitude error during the torque disturbances
  2.5x (θ), 2.3x (φ),
  4.1x (ψ) smaller, and its bias removed (roll: 0.05 vs 2.48 deg).
- Compensated vs PX4 tuned: J 2.55 vs 2.88 (S1) and 2.53 vs 3.24 (S1r, 3 seeds).
  `ijc_control` has the smaller horizontal error (ground truth RMS x / y: 0.138 / 0.057 m vs
  0.178 / 0.077 m), a smaller roll error and no attitude bias (PX4 tuned: 1.2 deg in roll).
  In z the ground-truth error of this single compensated run (0.165 m) is dominated by an EKF altitude drift
  (its EKF error is 0.061 m); see "Estimator drift" below.
- PX4 with its shipped quadtailsitter tuning: 1.4-1.8 m errors and a 15 deg roll bias.
- Estimation: `F_hat` and `T_hat` track the injected disturbances except right after the
  discontinuities (`F_hat` settles in ~0.5 s with the observer poles at -10).
- Sign check (Section 9.2): during the +`F_x` pulses the vehicle pitches nose-up (θ ≈ +15-30 deg),
  tilting the thrust towards -x, i.e. against the disturbance.
- Hover without disturbances (Section 9.1, last 5 s before the scenario): `|F_hat| <= 0.06 m/s^2`,
  `|T_hat| <= 0.05 rad/s^2`, `f = 16.03 N = m g`. No NaNs; allocation saturation 0.3 % of the time
  (transients) with compensation.

**S1h, A_T = 50 % of the authority (supplementary).**

| Case | J | ground truth: max \|e_p\| x / y / z (m) | ground truth: RMS e_p x / y / z (m) | EKF: RMS e_p x / y / z (m) | att. error (a) θ / φ / ψ (deg) | bias (b) θ / φ / ψ (deg) | unallocated (%) |
|---|---|---|---|---|---|---|---|
| Compensated | 22.33 | 0.48 / 1.40 / 0.66 | 0.137 / 0.247 / 0.152 | 0.103 / 0.237 / 0.194 | 4.64 / 18.82 / 8.68 | -0.70 / 9.96 / 3.05 | 13.7 |
| Non compensated | 21.10 | 0.86 / 0.45 / 0.10 | 0.334 / 0.178 / 0.046 | 0.322 / 0.152 / 0.044 | 7.42 / 6.76 / 17.68 | 0.59 / 5.86 / 3.52 | 11.6 |
| PX4 tuned | 9.43 | 0.49 / 0.29 / 0.85 | 0.180 / 0.101 / 0.221 | 0.140 / 0.081 / 0.184 | 1.86 / 2.26 / 0.51 | -0.13 / 2.02 / 0.18 | 10.5 |
| PX4 shipped tuning | 32.54 | 1.87 / 1.93 / 0.90 | 0.795 / 0.805 / 0.194 | 0.810 / 0.828 / 0.193 | 10.83 / 22.08 / 5.28 | 1.78 / 23.65 / 2.61 | 12.6 |

At 50 % the simultaneous peaks of `T_x`, `T_y` and `T_z` exceed the rotor authority: the allocator
cannot realize the setpoint 10-14 % of the time in all four cases. With compensation the controller
asks for the full counter-torque, saturates in roll during the `T_x` pulses and loses roll tracking
(φ error 19 deg RMS, transient peaks of 86 deg); without compensation the yaw drifts
(18 deg). The tuned PX4 cascade degrades much less: its yaw airmode (`MC_AIRMODE = 2`) gives
roll/pitch priority over yaw and its rate integrators are bounded, whereas the paper's laws have no
saturation handling (a natural extension: anti-windup of the compensation and roll/pitch priority in
`tau`). This amplitude measures behaviour under actuator saturation; S1 at 30 % is the representative case.

## Results: S2 (hover under Gazebo wind)

World `ijc_wind`: mean wind 4 m/s (mostly from the north, i.e. normal to the wing of the tail-sitter
heading north), gusts 3-5 m/s (magnitude +-25 %, period 8 s; direction +-15 deg, period 13 s) and
gaussian turbulence (D13). Hover at the takeoff point, `psi_d = 0`, 60 s, four cases x 5 repetitions.
Figures (repetition 1): `results/figures/sitl_S2_{pos,att,inputs,inputs_cmp}.pdf`. The perturbation rows
compare the aerodynamic wrench measured in Gazebo (D16; "Actual") with the observers' estimates
`m F_hat`, `J T_hat` of the compensated run ("Observed"), as in S1.

Position:

| Case (n = 5) | ground truth: max \|e_p\| x / y / z (m) | ground truth: RMS e_p x / y / z (m) | EKF: RMS e_p x / y / z (m) |
|---|---|---|---|
| Compensated | 0.24 ± 0.02 / 0.08 ± 0.01 / 0.31 ± 0.11 | 0.113 ± 0.006 / 0.033 ± 0.004 / 0.154 ± 0.068 | 0.053 ± 0.004 / 0.016 ± 0.003 / 0.071 ± 0.023 |
| Non compensated | 0.85 ± 0.04 / 0.18 ± 0.03 / 0.66 ± 0.17 | 0.576 ± 0.035 / 0.093 ± 0.018 / 0.332 ± 0.098 | 0.607 ± 0.018 / 0.083 ± 0.004 / 0.330 ± 0.037 |
| PX4 tuned | 0.42 ± 0.02 / 0.07 ± 0.01 / 0.56 ± 0.10 | 0.192 ± 0.002 / 0.030 ± 0.004 / 0.267 ± 0.049 | 0.168 ± 0.002 / 0.016 ± 0.002 / 0.192 ± 0.009 |
| PX4 shipped tuning | 1.23 ± 0.03 / 0.35 ± 0.02 / 0.38 ± 0.10 | 0.574 ± 0.010 / 0.160 ± 0.005 / 0.174 ± 0.033 | 0.537 ± 0.008 / 0.155 ± 0.006 / 0.125 ± 0.007 |

Attitude and disturbance estimation:

| Case (n = 5) | RMS att. error θ / φ / ψ (deg) | max att. error θ / φ / ψ (deg) | RMS \|m F_hat - F_aero\| x / y / z (N) | RMS \|J T_hat - τ_aero\| x / y / z (N·m) |
|---|---|---|---|---|
| Compensated | 0.82 ± 0.01 / 0.24 ± 0.04 / 0.09 ± 0.01 | 2.3 ± 0.1 / 1.5 ± 0.4 / 0.4 ± 0.1 | 0.22 ± 0.02 / 0.20 ± 0.08 / 0.22 ± 0.00 | 0.014 ± 0.000 / 0.017 ± 0.000 / 0.017 ± 0.003 |
| Non compensated | 4.17 ± 0.03 / 1.39 ± 0.01 / 0.72 ± 0.03 | 7.0 ± 0.1 / 2.5 ± 0.1 / 1.3 ± 0.1 | 0.23 ± 0.04 / 0.15 ± 0.05 / 0.20 ± 0.01 | 0.013 ± 0.000 / 0.017 ± 0.000 / 0.007 ± 0.000 |
| PX4 tuned | 0.55 ± 0.00 / 0.27 ± 0.01 / 0.09 ± 0.00 | 1.3 ± 0.0 / 0.7 ± 0.1 / 0.3 ± 0.1 | -- / -- / -- | -- / -- / -- |
| PX4 shipped tuning | 1.07 ± 0.01 / 2.05 ± 0.07 / 1.29 ± 0.04 | 2.1 ± 0.1 / 5.2 ± 0.3 / 2.7 ± 0.7 | -- / -- / -- | -- / -- / -- |

- The wing turns the wind into large, gusty disturbances: measured aerodynamic force about 2-7 N in x
  and 0.5-5 N of vertical lift, and moments of up to 0.8 N m; the vehicle hovers pitched 10-37 deg
  into the wind.
- Compensated vs PX4 tuned (ground truth RMS): 1.7x smaller in x, 1.7x in z, similar in y;
  vs non compensated: 5.1x (x) and 2.2x (z) smaller (without compensation the vehicle keeps
  a steady offset of ~0.6 m downwind and ~0.3 m in altitude). With the EKF error (what the controller
  sees) the ratios vs PX4 tuned are 3.2x (x) and 2.7x (z).
- Attitude: the compensated case tracks its desired attitude with 0.82 / 0.24 / 0.09 deg RMS
  (without compensation 4.17 / 1.39 / 0.72 deg: the aerodynamic moments leave a steady error); the tuned
  PX4 attitude loop (stiffer, with integrators) is slightly better in pitch (0.55 deg).
- Estimation: the observers reproduce the measured aerodynamic wrench with RMS errors of
  ~0.2 N (force, vs 1.3 N standard deviation of the force itself) and ~0.015 N m (moments). The
  remaining offset of ~0.3 N in y comes from the residual EKF yaw error (~2.6 deg) projected on the
  horizontal thrust component (~7 N when pitched ~25 deg).
- No saturation in any case.

## Results: S3 (circle under Gazebo wind)

Same wind; circle of radius 2 m and period 20 s at constant altitude, `psi_d = 0`, `p_d`, `v_d`, `a_d`
analytic (published on `trajectory_setpoint`, also to the PX4 cases); the angular rate ramps up in
5 s (D14). 60 s (three laps), four cases x 5 repetitions. Figures (repetition 1):
`results/figures/sitl_S3_{pos,att,inputs,inputs_cmp}.pdf` and the top view `sitl_S3_xy.pdf`
(four cases; PX4 shipped tuning dotted).

Position:

| Case (n = 5) | ground truth: max \|e_p\| x / y / z (m) | ground truth: RMS e_p x / y / z (m) | EKF: RMS e_p x / y / z (m) |
|---|---|---|---|
| Compensated | 0.30 ± 0.01 / 0.12 ± 0.01 / 0.36 ± 0.14 | 0.129 ± 0.010 / 0.060 ± 0.004 / 0.167 ± 0.071 | 0.054 ± 0.002 / 0.017 ± 0.003 / 0.075 ± 0.028 |
| Non compensated | 0.93 ± 0.05 / 0.23 ± 0.02 / 0.61 ± 0.08 | 0.606 ± 0.007 / 0.116 ± 0.011 / 0.338 ± 0.058 | 0.612 ± 0.010 / 0.086 ± 0.004 / 0.342 ± 0.025 |
| PX4 tuned | 0.53 ± 0.11 / 0.15 ± 0.02 / 0.58 ± 0.13 | 0.234 ± 0.027 / 0.072 ± 0.001 / 0.246 ± 0.040 | 0.178 ± 0.027 / 0.024 ± 0.001 / 0.200 ± 0.019 |
| PX4 shipped tuning | 1.54 ± 0.41 / 0.46 ± 0.02 / 0.39 ± 0.09 | 0.732 ± 0.054 / 0.199 ± 0.026 / 0.182 ± 0.030 | 0.655 ± 0.064 / 0.182 ± 0.025 / 0.139 ± 0.010 |

Attitude and disturbance estimation:

| Case (n = 5) | RMS att. error θ / φ / ψ (deg) | max att. error θ / φ / ψ (deg) | RMS \|m F_hat - F_aero\| x / y / z (N) | RMS \|J T_hat - τ_aero\| x / y / z (N·m) |
|---|---|---|---|---|
| Compensated | 0.80 ± 0.02 / 0.35 ± 0.07 / 0.11 ± 0.01 | 2.3 ± 0.1 / 2.0 ± 0.3 / 0.6 ± 0.1 | 0.23 ± 0.01 / 0.31 ± 0.22 / 0.22 ± 0.01 | 0.014 ± 0.000 / 0.017 ± 0.000 / 0.025 ± 0.006 |
| Non compensated | 4.20 ± 0.04 / 1.52 ± 0.02 / 0.80 ± 0.03 | 7.2 ± 0.1 / 3.5 ± 0.2 / 1.8 ± 0.2 | 0.21 ± 0.01 / 0.24 ± 0.09 / 0.18 ± 0.00 | 0.014 ± 0.000 / 0.017 ± 0.000 / 0.009 ± 0.000 |
| PX4 tuned | 0.56 ± 0.01 / 0.34 ± 0.01 / 0.11 ± 0.01 | 1.6 ± 0.1 / 1.2 ± 0.2 / 0.3 ± 0.0 | -- / -- / -- | -- / -- / -- |
| PX4 shipped tuning | 1.08 ± 0.01 / 2.40 ± 0.20 / 1.47 ± 0.15 | 2.3 ± 0.2 / 6.3 ± 0.5 / 3.5 ± 0.2 | -- / -- / -- | -- / -- / -- |

- Compensated vs PX4 tuned (ground truth RMS): 1.8x (x), 1.2x (y), 1.5x (z) smaller;
  vs non compensated 4.7x (x), 1.9x (y), 2.0x (z).
- Same attitude and estimation behaviour as in S2.

**Estimator drift and ground truth (all scenarios).** The EKF position drifts with respect to the ground
truth by 0.05-0.35 m during a run (GPS and barometer noise models of the simulator), mostly in
altitude and independently of the controller. The ground-truth error is roughly the error seen by the
controller plus that drift, so the drift dominates when the controller error is small: this is why the
compensated case shows the largest gap between its EKF and ground-truth altitude errors, and why the
standard deviation of the ground-truth z error over the repetitions is large (0.07 m).

## Videos

Not versioned; distributed in the Release (`videos.zip`: the official final versions `hover_final.mp4` and
`circle_final.mp4`, with the corrected title screen, plus the generated versions listed below). How they are
made and all numbers: `results/videos/README.md`.

- `hover_aggressive_comparison.mp4` and `circle_aggressive_comparison.mp4` (current): hover and circle
  (R = 2 m, T = 20 s, 2.5 m) under step-like wind gust fronts (+2.4 m/s every 6.5 s on a 2.4 m/s base,
  sudden +-45 deg direction changes, slow/fast gusts, turbulence), compensated / non compensated / PX4
  tuned in three synchronized columns with fixed cameras (close view of the hover reference; near-overhead
  view of the circle, drawn at the flight altitude with a sphere at the desired position and a one-lap
  trail), live wind arrow and a live plot of ||e_p||. The disturbance was chosen among eight profiles
  as the one with the largest RMS advantage over PX4 tuned with at most 5 % saturated time in every
  case (gust fronts help, direction changes alone barely, external pulses make the compensated case
  saturate). RMS ||e_p|| (ground truth): hover 0.21 / 0.68 / 0.40 m, circle 0.21 / 0.78 / 0.40 m
  (compensated / non compensated / PX4 tuned); compensated saturation 2.7 % / 1.7 %.
- `S1_comparison.mp4`, `S2_comparison.mp4` (first version, follow camera).

## Deviations from the spec (and why)

Each deviation was needed to make the scheme work on the Gazebo vehicle or to follow
PX4 conventions; all of them are parameters, so the paper's exact scheme is still available.
Evidence logs are in `logs/` (not versioned, large ULogs); the analyses are reproducible with
the scripts. The final parameter values are in `results/params.txt`.

**D1. Gains (attitude, rotational and translational observers).**
Final values (selected by the gain search, see "Gain selection"): `k_p = k_v = 4` (paper),
`k_q = 400`, `K_Omega = 30 I`, `K1 = 100 I`, `L1 = 2500 I` (rotational observer poles at -50) and
`K2 = 20 I`, `L2 = 100 I` (translational observer poles at -10 instead of -20).
*Why.* The Gazebo rotors are first-order systems (12.5 ms up / 25 ms down) plus ~2-8 ms of
delay (gyro filter, sampling). With the paper's gains the SITL diverged ~1 s after takeoff
(`V1_hover_comp_paper`), and a single-axis linear model (attitude law + observer + rotor lag +
delay + gyro filter) reproduces it: dominant pole at +23 1/s (paper gains, observer at -150),
+3 1/s (`k_q = 400`, observer at -80). The fast rotational observer is faster than the actuators,
which breaks the time-scale structure of Theorem 4.3. Besides, with `J_x = 0.115 kg m^2` and
3.5 N m of roll authority at hover, `k_q = 1600` saturates the roll torque at ~1 deg of error.
The translational observer was slowed down for the same reason: with the attitude poles at
-10/-20 1/s, an observer at -20 is as fast as the attitude loop; in S1 this produced a 2.4 Hz
oscillation of the desired attitude through `F_hat` (1.36 deg amplitude); with -10 it drops to
0.43 deg and the attitude error during the torque disturbances becomes clearly smaller than
without compensation on all three axes (A/B runs `AB_comp_ff0`, `AB_comp_tm25`, `AB_comp_obs10`
against the previous configuration). The price is a slower `F_hat` (it settles in ~0.5 s after
a step instead of ~0.25 s), which increases the position error during the `F_x` pulses.

**D2. Applied thrust/torque in the observers through an actuator model** (`IJC_TAU_MOT = 25 ms`).
The spec asks for the *applied* `f` and `tau` in `phi`. The allocator's post-saturation
setpoints reach the rotors through their speed dynamics, so they are passed through a
first-order lag before entering `phi_Omega` and `phi_v`. Without it the compensation is
unstable (`V1_hover_comp_red`: `T_hat_y` oscillating at +-50 rad/s^2, roll limit cycle).
With the linear model above, the worst case over rotor lags 12.5-25 ms and delays 2-8 ms is
stable for `IJC_TAU_MOT >= 22 ms` (worst max Re: +0.99 for 18 ms, -2.0 for 25 ms), so the
upper bound of the motor time constants is used. `IJC_TAU_MOT = 0` gives the paper's observer.

**D3. Lateral force of the canted rotors: in the observer and as feedforward in `mu`.**
The quadtailsitter rotors are canted by 0.17 rad in roll and pitch, so a control torque also
produces a lateral force (derived from the SDF: `F_y = -3.84 N` per unit normalized roll torque,
`F_x = -5.81 N` per unit pitch torque; `IJC_FY_TX`, `IJC_FX_TY`). For roll it points against
the motion the torque is producing (non-minimum-phase-like coupling).
(a) *In `phi_v`.* This known part of the applied force is included in the translational
observer, so `F_hat` estimates only external disturbances. Without it, `F_hat` fed the lateral
force back into `mu` (loop gain ~1.15) and the roll/y channel had a limit cycle
(`V1_hover_comp_tm`).
(b) *Feedforward in `mu`.* The lateral force is not negligible: in S1 its RMS is ~0.5 N
(injected disturbances: 1.2-2.3 N RMS) and its quasi-static part reaches 0.87 N, i.e. ~0.13 m of
position offset with `k_p = 4`. It is therefore added to the position law,
`mu += R F_lat(tau_applied)`, low-pass filtered with `IJC_FLAT_TC = 0.5 s` (`IJC_FLAT_FF = 2`).
It is a known model term, not a disturbance estimate, so it is active with and without
compensation (the two cases still differ only by `F_hat`, `T_hat`). The filter is necessary:
unfiltered, the feedforward closes the same fast positive loop `q_d -> tau -> F_lat -> mu -> q_d`
(gain ~1.14) and produces a roll limit cycle, both with the total torque (`V3_hover_flatff2`)
and with the compensation torque `-J T_hat` (first S1h attempt, which lost altitude); at the
limit-cycle frequency (~2-3 Hz) the filter reduces the loop gain below 0.3 while the
quasi-static part, which causes the position offset, passes. In A/B runs (previous observer
tuning) the feedforward reduced the maximum `y` error from 0.17 to 0.12 m and its RMS from 0.058
to 0.038 m.

**D4. Disturbance amplitudes.** `A_F = 0.3 g = 2.942 m/s^2` (4.81 N). `A_T` is set per axis, to
30 % of the angular acceleration the allocator can produce on that axis at hover thrust
(`scripts/normalization.py`): `A_T = [9.106, 20.272, 7.003] rad/s^2` (`J A_T = [1.05, 0.65, 0.60] N m`).
A single `A_T` is not possible: yaw authority at hover is only 2.0 N m (23 rad/s^2) vs 67.6 rad/s^2
in pitch. With these amplitudes the difference with/without compensation is clear on all three
axes, so this is the main S1. S1h repeats it with 50 % (`A_T = [15.177, 33.786, 11.671] rad/s^2`);
there, simultaneous peaks of `T_x`, `T_y`, `T_z` saturate the rotors in all three cases, so
the comparison measures actuator limits more than the scheme (see results).

**D5. Vehicle and simulation.** The spec prefers a quad tail-sitter model: PX4's `quadtailsitter`
(Swan K1) is used in hover, as a multirotor airframe (`CA_AIRFRAME 0`, no VTOL stack, no
transitions). `quadtailsitter_ijc` is an identical copy with the IMU at 1 kHz; the world
`ijc_calm` runs the physics at 1 kHz with g = 9.80665 m/s^2 (= PX4 `CONSTANTS_ONE_G`), and the
vehicle spawns heading north (`psi_d = 0`). Actuator mapping: `THR_MDL_FAC = 1`,
`SIM_GZ_EC_MIN/MAX = 0/1200` (stock: 10/1500 with rotor speed clamped at 1200 rad/s), which
makes the normalization exact; `FD_ESCS_EN = 0` because idle rotor speed is 0 with this mapping.

**D6. Flight management without standard control laws.** `ijc_control` flies from arming to
disarming in Offboard `thrust_and_torque`: quintic takeoff reference (2.5 m in 5 s), hover,
landing at 0.5 m/s, thrust ramp-down at touchdown. The observers are held in reset until the
vehicle is 5 cm above the ground (no leakage: ground contact would otherwise wind up the
estimates). The references are generated in the module and published on `trajectory_setpoint`
(for logging and for the PX4 comparison case) instead of being subscribed; the experiment
script commands the flight through the parameter `IJC_CMD`.

**D7. Attitude at 1 kHz.** EKF2 publishes `vehicle_attitude` at ~200 Hz; between samples the
module propagates it with the 1 kHz filtered gyro (`Q <- Q (x) exp(Omega dt / 2)`).

**D8. Disturbance injection.** The Gazebo `ApplyLinkWrench` topics apply wrenches for one step or
persistently without replacement, so a small world system (`IjcWrenchSystem`) applies the last
requested wrench at every physics step (zero-order hold), at the CoM of `base_link` (force in
the world frame, torque in the body frame). `ijc_control` computes the S1 signals from the
scenario time and publishes `F_ext = m F`, `tau_ext = J T` on `ijc_wrench`; `gz_bridge` forwards
them (NED/FRD -> ENU/FLU). The applied wrench lags the logged one by at most one physics step (1 ms).

**D9. PX4 infrastructure.** Logger: `ijc_debug` and the controller topics at full rate, logger
triggered by `ijc_debug` (`LOGGER_POLL_TOPIC` in `rcS`), one log per flight (`SDLOG_MODE 0`),
ULog format buffer 1600 -> 2400 bytes (the long `ijc_debug` format plus the reader's leftover
did not fit; no change to the ULog format).

**D10. Standard PX4 comparison cases (added after the S1 review).** Airframe 4051: same model,
actuator mapping, EKF2 and allocator; `mc_pos_control`, `mc_att_control`, `mc_rate_control` with
the multicopter tuning PX4 ships for the quadtailsitter (`rc.vtol_defaults` + 4018; plus
`MPC_THR_HOVER = 0.335`, a vehicle property); the "PX4 tuned" case overrides it with the gains found
by the same search as `ijc_control` (see "Gain selection"). `ijc_control` runs
with `IJC_MODE = 1`: it publishes the same reference as Offboard position setpoints
(`p_d`, `v_d`, `a_d`), injects the same disturbances and logs the observers (not used by the
controller). PX4's takeoff logic spools up more slowly, so every case hovers 10 s before the
scenario starts.

**D11. Figures.** `plots.py` settings are reused verbatim. The third case is drawn with a black
dash-dot line; the attitude figure has one row per case (compensated, non compensated, PX4
standard), so it is 524 x 409 pt instead of 524 x 310 pt (same row height); an extra
`sitl_S1_inputs_cmp.pdf` compares `f` and `||tau||` of the three cases (`sitl_S1_inputs.pdf`
shows the compensated case, as in `plots.py`); y-limits are automatic; the `tau` legend uses
three columns so that it does not cover the curves.

**D12. Metrics.** Computed over the scenario window t = 0-20 s from the controller signals
(EKF estimates; EKF vs Gazebo ground truth: bias < 1.2 cm in x, y, std <= 6 cm). Attitude error:
Euler angles of `Q` minus those of `Q_d`. S1 adds the RMS attitude error while each torque
disturbance acts (`theta` while `T_y` acts, t > 7; `phi` during the `T_x` pulses; `psi` while `T_z`
acts, t >= 7), (a) excluding 0.5 s after each `F_x` jump (as requested) and (b) excluding 0.5 s
after every disturbance discontinuity (also the `T_z` sawtooth resets). The estimation error is
not reported for the PX4 case (its observers are not part of the controller). Ground-truth tracking
errors (`gt_*` in `metrics.csv`): Gazebo pose (~60 Hz) minus `p_d`, origin aligned with the EKF at t = 0;
they are the main position metric of the tables. S2/S3 add the maximum attitude error and the estimation
error of the observers against the measured aerodynamic wrench (D16); their values are mean +- std over five
repetitions (see "Methodology of the result tables").

**D13. Wind.** Gazebo `WindEffects` provides the world wind (mean + sinusoidal gusts in magnitude and
direction + gaussian noise); the vehicle's own aerodynamic models use it (`AdvancedLiftDrag` of the
wing, rotor drag of `MulticopterMotorModel`), which is what makes wind relevant for a tail-sitter.
The generic `WindEffects` body force (links with `enable_wind`) is not enabled, since it would add a
second, non-physical drag on top of the wing model. `WindEffects` has no Dryden-type turbulence; the
gusts are its sinusoidal modulation (8 s / 13 s) plus white gaussian noise.

**D15. Magnetometer (found while validating S2/S3 against the ground truth).** The gz-sim 8
magnetometer publishes in a left-handed frame; `gz_bridge` remaps it with a fixed reflection that is only
right for a level vehicle, so the EKF yaw was off by ~5 deg in S1 and by ~37 deg in S2 (vehicle pitched
into the wind), for every controller. `quadtailsitter_ijc` therefore has no gz magnetometer and PX4's
`sensor_mag_sim` (`SENS_EN_MAGSIM = 1`: field from the WMM at the simulated GPS position and the
ground-truth attitude) replaces it; the EKF yaw error is now 2.6 +- 1.9 deg in wind. All results in this
README (S1, S1h, S1r, S2, S3) were re-run after the fix; the gain searches were not (see "Gain selection").

**D16. Measured aerodynamic wrench (ground truth of the disturbance in S2/S3).** Gazebo does not publish
the forces of its aerodynamic systems, so a world system (`IjcAeroSystem`, `gz:system_priority` 100, i.e.
after the model systems) reads, every physics step, the wrench that the model systems queued on the base
link and on the rotor links (`ExternalWorldWrenchCmd`), removes the actuation (rotor thrust = component of
each rotor force along its axis, and the rotor drag torque `-dir km T a`) and publishes the rest about the
CoM: wing lift/drag/moments, rotor in-plane drag and rolling moments, and any injected wrench. `gz_bridge`
forwards it to the uORB topic `ijc_aero` (logged). Validation on S1 (injected wrench, calm air): measured
minus injected = 0.14 / 0.03 / 0.05 N and 0.02-0.03 N m RMS (steps and the vehicle's own drag), zero in
hover. For plots and estimation errors it is averaged over 50 ms (centered), which removes the white,
per-physics-step part of the wind noise that no observer can follow and keeps the gusts.

**D14. Trajectory (S3).** The circle starts at the hover point with zero velocity: the angular rate
ramps linearly from 0 to `2 pi / 20 s` in 5 s (`IJC_TRJ_TR`), with `p_d`, `v_d`, `a_d` analytic
throughout, to avoid a velocity step at t = 0. The landing starts from the reference at the end of the
scenario.

