# SPEC — PX4 SITL implementation of the observer-based tail-sitter controller (IJC revision)

This document is the implementation brief for the SITL implementation work on a Linux machine.
It is self-contained: everything needed to implement, run, and plot the SITL experiments is here.

Companion files (copy them next to this spec, e.g. `~/ijc_tailsitter/spec/`):

- `sim_python/simnb.py` — reference numerical simulation of the exact control scheme (Python + numba, RK4, 1e-4 s). **Ground truth for signs and conventions.**
- `sim_python/plots.py` — plotting script whose style must be reproduced exactly.
- `anonimo.tex` (optional) — the paper; Sections 2–5 contain the equations below.

---

## 0. Ground rules for the session

1. Work **only** inside `~/ijc_tailsitter/`. Do not touch any other PX4 installation or folder.
2. Use a dedicated git branch (`ijc-observers`) and commit after each working milestone.
3. Before writing controller code, run `sim_python/simnb.py` once and keep its outputs as the reference behavior.
4. Ask the user before any `sudo` command; the user types the password.
5. When something in this spec conflicts with the PX4 version installed, prefer the PX4 convention and report the change.

---

## 1. Goal

Implement in PX4 (SITL, Gazebo) the control scheme of the paper *"Observer-Based Disturbance-Compensated Position and Attitude Control of Tail-Sitter UAVs"* and produce:

- **S1.** Hover under synthetic force/torque disturbances (reproduces Section 5.2 of the paper with the new laws).
- **S2.** Hover under Gazebo wind (steady wind + gusts) — answers Reviewer 2.2 ("additional virtual environment scenarios").
- **S3.** Trajectory tracking (circle or lemniscate) under wind — answers Reviewer 2.2 and complements Remark "Trajectory tracking".

Each scenario is run **with** and **without** disturbance compensation (same controller, estimates forced to zero in the control laws; observers keep running and are still logged).

Deliverables: figures (PDF, paper style), a metrics table (CSV + LaTeX), the parameter file, and a short README describing how to reproduce.

---

## 2. Conventions (must match the paper and PX4)

- Inertial frame **NED**, `e3 = [0,0,1]` points **down**. Body frame **FRD**.
- Quaternion Hamilton `Q = [w, x, y, z]`; `R(Q)` maps body → inertial (PX4 `vehicle_attitude.q` convention).
- Model used by the paper (hover regime, multirotor-like):

  ```
  p_dot = v
  v_dot = g e3 - (f/m) R e3 + F(t)          # F: disturbance as ACCELERATION (F = F_ext/m)
  Q_dot = 0.5 Q ⊗ (0, Ω)
  Ω_dot = -J^{-1}(Ω × JΩ) + J^{-1} τ + T(t)  # T: disturbance as ANGULAR ACCELERATION (T = J^{-1} τ_ext)
  ```

- Thrust `f > 0` in newtons acts along `-z_B` (upwards in hover). Torque `τ` in N·m, body frame.

---

## 3. Control scheme (implement exactly)

### 3.1 Disturbance observers (Lemma 3.1 of the paper — NO leakage term)

Generic form for `y_dot = φ(t) + d(t)` with measured `y`:

```
y_hat_dot = φ + d_hat - K (y_hat - y)
d_hat_dot = -L (y_hat - y)
```

- **Translational observer** (`y = v`, `d = F`): `φ_v = g e3 - (f/m) R e3`, gains `K2, L2` (diagonal).
- **Rotational observer** (`y = Ω`, `d = T`): `φ_Ω = -J^{-1}(Ω×JΩ) + J^{-1} τ`, gains `K1, L1` (diagonal).
- Use the **applied** `f` and `τ` (after any saturation) in `φ`, and PX4's estimated `v` (`vehicle_local_position`) and filtered `Ω` (`vehicle_angular_velocity`).
- Discretize with the actual `dt` from timestamps. Forward Euler is acceptable if `dt·max(K) < 0.2` and `dt²·max(L) < 0.05`; otherwise use the exact discretization of the 2×2 linear error system per axis or sub-step.

### 3.2 Position control (Proposition 4.1)

```
e_p = p - p_d,   e_v = v - v_d
μ   = m ( g e3 + k_p e_p + k_v e_v - a_d + F_hat )     # a_d = feedforward acceleration
f   = ||μ||
```

With compensation OFF: `F_hat := 0` inside `μ` only.

Practical safeguards (report them in the README): saturate the horizontal part of `μ` to a max tilt (e.g. 45°), and keep `μ_z ≥ 0.3 m g`.

### 3.3 Desired attitude (Section 4.1, eqs. 20–24)

```
R_psi = Rz(psi_d)
z     = R_psi^T μ / ||μ||                               # thrust direction in heading frame
Q_r   = [1 + z_z, (e3 × z)] / sqrt(2 (1 + z_z))         # = (1+z_z, -z_y, z_x, 0)/sqrt(2(1+z_z))
Q_a   = [cos(psi_d/2), 0, 0, sin(psi_d/2)]
Q_d   = Q_a ⊗ Q_r
```

By construction `R_d e3 = μ/||μ||`. Use constant `psi_d` (0 for hover; along-track or 0 for S3).

### 3.4 Attitude control (Proposition 4.2) — NO Ω_d feedforward

```
Q_bar = Q_d^{-1} ⊗ Q ;   if Q_bar.w < 0:  Q_bar = -Q_bar
τ     = Ω × JΩ + J ( -T_hat - k_q q_bar - K_Ω Ω )        # q_bar = vector part of Q_bar
```

With compensation OFF: `T_hat := 0` inside `τ` only.
Optional (Remark "Feedforward of Ω_d"): add feedforward with a filtered `Ω_d` from differentiating `R_d`; keep it **off** for the main results.

### 3.5 Time-scale structure (Theorem 4.3)

Attitude and rotational-observer gains are the "fast" ones: `k_q = k̄_q/ε²`, `K_Ω = K̄_Ω/ε`, `K1 = K̄1/ε`, `L1 = L̄1/ε²`. In the Python reference: `ε = 0.01`, `k̄_q = 0.16`, `K̄_Ω = 0.6 I`, `K̄1 = 3 I`, `L̄1 = 2.25 I`.

---

## 4. Gains

Reference values (paper Tables 2–3, for `m = 1 kg`, `J = I`):

| Block | Gains |
|---|---|
| Position | `k_p = 4`, `k_v = 4` |
| Attitude | `k_q = 1600`, `K_Ω = 60 I` |
| Rotational observer | `K1 = 300 I`, `L1 = 22500 I` (poles at −150 rad/s, ζ = 1) |
| Translational observer | `K2 = 40 I`, `L2 = 400 I` (poles at −20 rad/s, ζ = 1) |

For the Gazebo vehicle (real `m`, `J`): keep **the same pole locations** — position and attitude laws already contain `m` and `J`, so `k_p, k_v, k_q, K_Ω, K1, L1, K2, L2` can be kept as they are. If the attitude loop is too aggressive for the sim rate/actuators, reduce `k_q, K_Ω` together (e.g. `k_q = 400`, `K_Ω = 30 I`) and the rotational observer to poles at −80 rad/s (`K1 = 160 I`, `L1 = 6400 I`). Report the final values.

---

## 5. PX4 integration

Target PX4 ≥ v1.14 (control allocator architecture). Suggested design (adapt to the installed version):

1. **New module** `src/modules/ijc_control/` (C++, `ModuleBase` + `WorkItem`), scheduled on `vehicle_angular_velocity` updates (~400 Hz–1 kHz in SITL):
   - Subscribes: `vehicle_local_position`, `vehicle_attitude`, `vehicle_angular_velocity`, `trajectory_setpoint`, `vehicle_control_mode`.
   - Runs every cycle: rotational observer + attitude law. Runs position law + translational observer when a new `vehicle_local_position` sample arrives (or every cycle with the latest sample).
   - Publishes: `vehicle_thrust_setpoint` and `vehicle_torque_setpoint` (normalized, see below), plus a custom debug topic.
2. **Disable the stock multicopter controllers** (`mc_pos_control`, `mc_att_control`, `mc_rate_control`) for this airframe only, e.g. with a custom SITL airframe/startup script that starts `ijc_control` instead. Keep estimator (EKF2), commander, navigator, control allocator, and logger unchanged.
3. **Normalization.** The control allocator expects normalized thrust/torque. Derive the conversion from the vehicle model:
   - thrust: `thrust_norm = f / f_max` (use `f_max = n_rotors · CT · ω_max²` from the Gazebo model / `CA_*` params, or identify: hover thrust param `MPC_THR_HOVER` ↔ `m g`);
   - torque: `τ_norm = τ ./ τ_max` with `τ_max` per axis from rotor geometry and coefficients (`CA_ROTORx_PX/PY`, `CT`, `KM`).
   Document the constants and verify at hover (`f ≈ m g`, `thrust_norm ≈ MPC_THR_HOVER`).
4. **Parameters** (`ijc_params.c`): all gains, `IJC_COMP` (0/1 compensation switch), `IJC_PSI_D`, `IJC_MASS`, `IJC_JXX/JYY/JZZ`, disturbance-injection switch.
5. **Custom uORB topic** `ijc_debug` (add to `msg/` and to the logger's default topics): `e_p[3], e_v[3], F_hat[3], T_hat[3], v_hat[3], Omega_hat[3], q_d[4], q_bar[4], f, tau[3], F_applied[3], T_applied[3]`.

**Vehicle model.** List `Tools/simulation/gz/models/`. If a quad tail-sitter model exists (e.g. `quadtailsitter`), use it in hover; otherwise use `x500` — the paper's hover model is multirotor-like, so this is acceptable but must be stated in the README. Read `m` and `J` from the model SDF.

---

## 6. Disturbances and scenarios

### S1 — synthetic disturbances (hover at p_d = 0 relative to takeoff hover point, ψ_d = 0)

Apply **physical** wrenches to the vehicle base link in Gazebo (preferred: the Gazebo `ApplyLinkWrench` system / `/world/<world>/wrench` topic if available in the installed Gazebo; otherwise inject them as additive terms in the simulated dynamics or, as last resort, in the module, and document it). Time origin = start of the hover segment.

| Force (as acceleration × m) | Torque (as angular acceleration × J) |
|---|---|
| `F_x`: amplitude `A_F` on `n < t < n + 1.5`, `n = 5, 10, 15` s | `T_x`: amplitude `A_T` on the same intervals |
| `F_y = (0.4 A_F) sin t`, `t > 5` s | `T_y = A_T sin t`, `t > 7` s |
| `F_z = 0.2 A_F (t/3 − ⌊t/3⌋)`, `t ≥ 5` s | `T_z = A_T (t/2 − ⌊t/2⌋)`, `t ≥ 7` s |

The paper's numerical case used `A_F = 5 m/s²` and `A_T = 100 rad/s²` with `m = 1`, `J = I`. For the Gazebo vehicle **scale to actuator capability**: choose `A_F ≈ 0.3 g` and `A_T ≈ 30 %` of the max angular acceleration the allocator can produce; report the values. Apply `F_ext = m·F`, `τ_ext = J·T`.

### S2 — Gazebo wind (hover)

World with the Gazebo wind system (e.g. `WindEffects`): steady wind 3–5 m/s plus gusts/turbulence. Duration ≥ 60 s. Compensation ON vs OFF.

### S3 — trajectory tracking under wind

Circle radius 2 m, period 20 s (or lemniscate), constant altitude, `ψ_d = 0`, with the S2 wind. Provide `p_d, v_d, a_d` analytically in `trajectory_setpoint`. Compensation ON vs OFF.

Flight procedure for every run: arm → takeoff/hover with stock mode or with `ijc_control` at hover → settle 5 s → start scenario timer → log ≥ 20 s (S1) / ≥ 60 s (S2, S3) → land. Automate with a Python script over MAVLink (`pymavlink` or MAVSDK).

---

## 7. Logging and post-processing

- Logger: enable `ijc_debug`, `vehicle_local_position`, `vehicle_attitude`, `vehicle_angular_velocity`, `trajectory_setpoint`, `vehicle_thrust_setpoint`, `vehicle_torque_setpoint` at full rate.
- Convert `.ulg` with `pyulog` (`ulog2csv`) and align all signals to the scenario time origin.
- Metrics per scenario and per case (ON/OFF), computed after the settling period: max and RMS of `e_p` per axis, RMS attitude tracking error (deg) per Euler angle, RMS disturbance-estimation error (S1 only), max `f` and `||τ||`. Save as CSV and as a LaTeX table (`booktabs`).

---

## 8. Figures (must look identical in style to `sim_python/plots.py`)

Reuse `plots.py` settings verbatim: Computer Modern (`font.serif = cmr10`, `mathtext.fontset = cm`), font sizes 7/6.5, ticks inward on all four sides, full box, light grid, boxed legends above the rows, row titles ("Position states", "Attitude states", "Perturbations"), dashed pattern `[4.5, 2.5]`.

- Colors/lines: desired **solid black**; compensated **solid gray (0.55)**; non-compensated **dashed black**; in disturbance rows actual **solid black**, observed **dashed gray**.
- Layout and sizes: position figure 3×3, 533×310 pt; attitude figure 3×3, 524×310 pt (spacer row between rows 2 and 3 so the legend never overlaps tick labels); inputs figure 1×2, 612×111 pt.
- Units: forces in N, torques in N·m, angles in deg, time in s.
- One set of figures per scenario: `sitl_S1_pos.pdf`, `sitl_S1_att.pdf`, `sitl_S1_inputs.pdf`, `sitl_S2_*.pdf`, `sitl_S3_*.pdf` (for S2/S3 the "Perturbations" row shows the estimated `F_hat`, `T_hat` since true wind forces are not directly available; label the legend accordingly).
- Check every figure for overlapping text before finishing.

---

## 9. Validation checklist

1. With no disturbances and compensation ON, `F_hat` and `T_hat` converge to small values (model mismatch only) and hover is stable.
2. Hover thrust ≈ `m g`; sign check: a positive `F_x` step must make the vehicle pitch so that it resists motion in `+x`.
3. S1 with the paper's synthetic signals: estimates track the applied disturbances (except at discontinuities), and position error with compensation is clearly smaller than without.
4. Compare qualitatively with `sim_python` outputs (same trends).
5. No NaNs, no saturation for more than short transients; report any saturation.

---

## 10. Final deliverables (put in `~/ijc_tailsitter/results/`)

- `figures/` — all PDFs listed in Section 8.
- `metrics.csv`, `metrics_table.tex`.
- `params.txt` — all gains, disturbance amplitudes, vehicle `m`, `J`, normalization constants, PX4 and Gazebo versions, model used.
- `README.md` — how to build, run each scenario, and regenerate figures; list of deviations from this spec.
- The git branch with the module, airframe/startup changes, world files, and scripts.
