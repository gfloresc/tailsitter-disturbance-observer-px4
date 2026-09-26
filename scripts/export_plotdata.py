#!/usr/bin/env python3
"""Export the time series used by the figures to results/plotdata/<S>_<case>.csv.

Runs: the ones plotted in the figures (S1: logs/S1_<case>.ulg; S2/S3: repetition 1,
logs/<S>/<S>_<case>_r1.ulg); the four cases compensated, non_compensated, px4_tuned, px4_shipped.
Signals come from postprocess.load_run (same windowing, alignment and processing as the figures).
"""
import os

import numpy as np

import postprocess as pp
from ulog_tools import load

from paths import ROOT  # noqa: E402  (repository root)
OUT = f'{ROOT}/results/plotdata'
CASES = {'C': ('comp', 'compensated'), 'N': ('nocomp', 'non_compensated'), 'P': ('px4', 'px4_tuned'),
         'Q': ('px4ship', 'px4_shipped')}
SCEN = {'S1': (20.0, lambda suf: f'{ROOT}/logs/S1_{suf}.ulg'),
        'S2': (60.0, lambda suf: f'{ROOT}/logs/S2/S2_{suf}_r1.ulg'),
        'S3': (60.0, lambda suf: f'{ROOT}/logs/S3/S3_{suf}_r1.ulg')}


def ground_truth_velocity(path, R, tend):
    """Gazebo ground-truth velocity (~60 Hz) interpolated on the controller time base."""
    L = load(path, ['ijc_debug', 'vehicle_local_position_groundtruth'])
    d, g = L['ijc_debug'], L['vehicle_local_position_groundtruth']
    k = (d['t_scen'] >= 0) & (d['t_scen'] <= tend + 1e-9)
    return np.column_stack([np.interp(d['t'][k], g['t'], g[f'v{a}']) for a in 'xyz'])


def export(S, c):
    tend, path_of = SCEN[S]
    suf, cid = CASES[c]
    path = path_of(suf)
    R = pp.load_run(path, tend)
    p0 = R['p0']
    cols = {'t_s': R['t']}
    for i, a in enumerate('xyz'):
        cols[f'p_d_{a}_m'] = R['p_d'][:, i] - p0[i]
    for i, a in enumerate('xyz'):
        cols[f'p_ekf_{a}_m'] = R['p'][:, i] - p0[i]
    for i, a in enumerate('xyz'):
        # ground truth: e_p_gt = p_gt - p_d (aligned with the EKF at t = 0)  ->  p_gt relative to the hover point
        cols[f'p_gt_{a}_m'] = R['e_p_gt'][:, i] + R['p_d'][:, i] - p0[i]
    for i, a in enumerate('xyz'):
        cols[f'v_d_{a}_mps'] = R['v_d'][:, i]
    for i, a in enumerate('xyz'):
        cols[f'v_ekf_{a}_mps'] = R['v'][:, i]
    vgt = ground_truth_velocity(path, R, tend)
    for i, a in enumerate('xyz'):
        cols[f'v_gt_{a}_mps'] = vgt[:, i]
    ED, E = pp.eul_deg(R['q_d']), pp.eul_deg(R['q'])
    for i, a in enumerate(('theta', 'phi', 'psi')):
        cols[f'{a}_d_deg'] = ED[i]
    for i, a in enumerate(('theta', 'phi', 'psi')):
        cols[f'{a}_deg'] = E[i]
    if S == 'S1':   # injected disturbance
        F_act, T_act = R['m'] * R['f_applied_dist'], R['J'] * R['t_applied_dist']
    else:           # aerodynamic wrench measured in Gazebo (50 ms moving average)
        F_act, T_act = R['aero_F'], R['aero_T']
    for i, a in enumerate('xyz'):
        cols[f'F_actual_{a}_N'] = F_act[:, i]
    for i, a in enumerate('xyz'):
        cols[f'F_hat_{a}_N'] = R['m'] * R['f_hat'][:, i]
    for i, a in enumerate('xyz'):
        cols[f'T_actual_{a}_Nm'] = T_act[:, i]
    for i, a in enumerate('xyz'):
        cols[f'T_hat_{a}_Nm'] = R['J'][i] * R['t_hat'][:, i]
    cols['f_N'] = R['f']
    for i, a in enumerate('xyz'):
        cols[f'tau_{a}_Nm'] = R['tau'][:, i]
    names = list(cols)
    data = np.column_stack([np.asarray(cols[n], dtype=float) for n in names])
    assert np.isfinite(data).all(), f'non-finite values in {S} {cid}'
    out = f'{OUT}/{S}_{cid}.csv'
    np.savetxt(out, data, delimiter=',', header=','.join(names), comments='', fmt='%.6g')
    return out, os.path.basename(path), data.shape


def main():
    os.makedirs(OUT, exist_ok=True)
    for S in SCEN:
        for c in CASES:
            out, src, shape = export(S, c)
            print(f'{os.path.basename(out)}: {shape[0]} rows x {shape[1]} columns (from {src})')


if __name__ == '__main__':
    main()
