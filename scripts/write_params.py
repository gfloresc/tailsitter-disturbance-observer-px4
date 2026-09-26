#!/usr/bin/env python3
"""Write results/params.txt from the parameters stored in a ULog (the values actually used).

usage: write_params.py <log.ulg> [more logs, for the list of runs]
"""
import os
import subprocess
import sys

from pyulog import ULog

from paths import PX4, ROOT  # noqa: E402  (repository root, PX4 tree)


def sh(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, text=True, stderr=subprocess.STDOUT).strip()
    except subprocess.CalledProcessError:
        return 'n/a'


logs = sys.argv[1:]
P = ULog(logs[0], []).initial_parameters

groups = [
    ('Controller in the loop (0: ijc_control, 1: standard PX4 controllers, airframe 4051)', ['IJC_MODE', 'IJC_COMP']),
    ('Gains (Section 4; attitude and observers reduced for this vehicle, README D1)',
     ['IJC_KP', 'IJC_KV', 'IJC_KQ', 'IJC_KOM', 'IJC_K1', 'IJC_L1', 'IJC_K2', 'IJC_L2']),
    ('Vehicle model (quadtailsitter SDF, scripts/normalization.py)',
     ['IJC_MASS', 'IJC_JXX', 'IJC_JYY', 'IJC_JZZ']),
    ('Normalization constants (thrust_norm = f/IJC_FMAX, torque_norm_i = tau_i/IJC_TMAX_i)',
     ['IJC_FMAX', 'IJC_TMAX_X', 'IJC_TMAX_Y', 'IJC_TMAX_Z', 'MPC_THR_HOVER', 'THR_MDL_FAC',
      'SIM_GZ_EC_MIN1', 'SIM_GZ_EC_MAX1', 'CA_ROTOR0_PX', 'CA_ROTOR0_PY', 'CA_ROTOR0_KM', 'CA_ROTOR0_CT']),
    ('Applied-wrench model in the observers (deviation from the paper, see README)',
     ['IJC_TAU_MOT', 'IJC_FX_TY', 'IJC_FY_TX', 'IJC_FLAT_FF', 'IJC_FLAT_TC']),
    ('Safeguards / references',
     ['IJC_PSI_D', 'IJC_TILT_MAX', 'IJC_MUZ_MIN', 'IJC_TKO_ALT', 'IJC_TKO_T', 'IJC_LND_VEL']),
    ('S1 disturbance amplitudes (F = A_F..., T = A_T...; F_ext = m F, tau_ext = J T)',
     ['IJC_DIST', 'IJC_AF', 'IJC_ATX', 'IJC_ATY', 'IJC_ATZ']),
    ('Sensors / logging / checks', ['IMU_GYRO_RATEMAX', 'IMU_GYRO_CUTOFF', 'SDLOG_PROFILE', 'SDLOG_MODE', 'FD_ESCS_EN', 'SENS_EN_MAGSIM']),
]
PX4_KEYS = ['MC_AIRMODE', 'MC_ROLL_P', 'MC_PITCH_P', 'MC_YAW_P', 'MC_ROLLRATE_P', 'MC_ROLLRATE_I', 'MC_ROLLRATE_D',
            'MC_PITCHRATE_P', 'MC_PITCHRATE_I', 'MC_PITCHRATE_D', 'MC_YAWRATE_P', 'MC_YAWRATE_I', 'MC_ROLLRATE_MAX',
            'MC_YAWRATE_MAX', 'MPC_XY_P', 'MPC_XY_VEL_P_ACC', 'MPC_XY_VEL_I_ACC', 'MPC_XY_VEL_D_ACC', 'MPC_Z_P',
            'MPC_Z_VEL_P_ACC', 'MPC_Z_VEL_I_ACC', 'MPC_XY_VEL_MAX', 'MPC_ACC_HOR_MAX', 'MPC_THR_HOVER']

lines = ['# ijc_control SITL experiments: parameters and versions', '']
lines.append(f"PX4-Autopilot: {sh(f'git -C {PX4} describe --tags --always')} (branch {sh(f'git -C {PX4} rev-parse --abbrev-ref HEAD')}, based on v1.17.0)")
lines.append(f"Gazebo: {sh('gz sim --version | head -1')} (gz-harmonic)")
lines.append('Vehicle model: quadtailsitter_ijc (= PX4 quadtailsitter, Swan K1; IMU at 1 kHz) in hover, world ijc_calm (1 kHz physics)')
lines.append('Airframe: 4050_gz_quadtailsitter_ijc (no standard PX4 controller running)')
lines.append('')
for title, names in groups:
    lines.append(f'[{title}]')
    for n in names:
        if n in P:
            v = P[n]
            lines.append(f'{n:16s} = {v:.6g}' if isinstance(v, float) else f'{n:16s} = {v}')
    lines.append('')
for lg in logs:
    Q = ULog(lg, []).initial_parameters
    if Q.get('IJC_MODE') == 1 and os.path.basename(lg).startswith('S1_'):
        lines.append(f'[Standard PX4 controllers used in {os.path.basename(lg)}]')
        lines += [f'{n:16s} = {Q[n]:.6g}' if isinstance(Q[n], float) else f'{n:16s} = {Q[n]}' for n in PX4_KEYS if n in Q]
        lines.append('')
lines.append('Derived quantities:')
m, g = P['IJC_MASS'], 9.80665
J = [P['IJC_JXX'], P['IJC_JYY'], P['IJC_JZZ']]
lines.append(f"  hover thrust m g = {m * g:.4f} N  -> thrust_norm = {m * g / P['IJC_FMAX']:.4f}")
lines.append(f"  m A_F = {m * P['IJC_AF']:.4f} N;  J A_T = [{J[0] * P['IJC_ATX']:.4f}, {J[1] * P['IJC_ATY']:.4f}, {J[2] * P['IJC_ATZ']:.4f}] N m")
lines.append(f"  (A_T = 30% of the max angular acceleration the allocator can produce per axis at hover thrust;"
                 "\n   S1h runs: 50%)")
lines.append('')
lines.append('Runs:')
for lg in logs:
    Q = ULog(lg, []).initial_parameters
    lines.append(f"  {os.path.basename(lg)}: SYS_AUTOSTART={Q.get('SYS_AUTOSTART')} IJC_MODE={Q.get('IJC_MODE')} IJC_COMP={Q.get('IJC_COMP')} "
                 f"IJC_DIST={Q.get('IJC_DIST')} IJC_TRAJ={Q.get('IJC_TRAJ')} "
                 + (f"A_T=[{Q.get('IJC_ATX'):.3f}, {Q.get('IJC_ATY'):.3f}, {Q.get('IJC_ATZ'):.3f}]" if Q.get('IJC_DIST') else 'world ijc_wind (no synthetic disturbance)'))
with open(f'{ROOT}/results/params.txt', 'w') as f:
    f.write('\n'.join(lines) + '\n')
print('\n'.join(lines))
