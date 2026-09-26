#!/usr/bin/env python3
"""Figures (paper style, sim_python/plots.py) and metrics of an ijc_control SITL scenario.

usage: postprocess.py <S1|S2|S3> <compensated.ulg> <non_compensated.ulg> [--px4 <px4_standard.ulg>] [--tend 20]

Writes results/figures/sitl_<S>_{pos,att,inputs,inputs_cmp}.pdf (+ .png previews) and
updates results/metrics.csv and results/metrics_table.tex (one row per scenario and case).
All signals come from the ijc_debug topic (controller rate, 1 kHz), aligned to the
scenario time origin (IJC_CMD=1, t = 0). Positions are relative to the hover point p_d(0).
For the standard PX4 case (IJC_MODE=1) the reference is the same trajectory_setpoint,
the desired attitude is PX4's vehicle_attitude_setpoint, and f, tau are its thrust and
torque setpoints converted with the same normalization constants.
"""
import argparse
import csv
import glob
import os
import re

import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from ulog_tools import load, vec, quat_to_euler

from paths import ROOT  # noqa: E402  (repository root)
RES = f'{ROOT}/results'
FIG = f'{RES}/figures'

# ---- settings of sim_python/plots.py (verbatim)
plt.rcParams.update({'font.family':'serif','font.serif':['cmr10'],'mathtext.fontset':'cm',
 'axes.unicode_minus':False,'axes.formatter.use_mathtext':True,'font.size':7,
 'axes.labelsize':7,'xtick.labelsize':6.5,'ytick.labelsize':6.5,'legend.fontsize':7,
 'xtick.direction':'in','ytick.direction':'in','xtick.top':True,'ytick.right':True,
 'axes.linewidth':0.5,'xtick.major.width':0.5,'ytick.major.width':0.5,'xtick.major.size':2.5,'ytick.major.size':2.5,
 'grid.color':'0.85','grid.linewidth':0.5,'axes.grid':True,'legend.frameon':True,
 'legend.fancybox':False,'legend.edgecolor':'k','legend.framealpha':1,'pdf.fonttype':42,'lines.dashed_pattern':[4.5,2.5]})
GR = '0.55'
DASHDOT = (0, (6.0, 1.8, 1.2, 1.8))   # standard PX4 controllers: black dash-dot

# line styles of the three cases in the state rows
STY = {'C': dict(color=GR, lw=0.7), 'N': dict(color='k', ls='--', lw=0.7), 'P': dict(color='k', ls=DASHDOT, lw=0.7)}
CASE_LABEL = {'C': 'Compensated', 'N': 'Non compensated', 'P': 'PX4 (tuned)', 'Q': 'PX4 (shipped tuning)'}
CASE_ID = {'C': 'compensated', 'N': 'non_compensated', 'P': 'px4_tuned', 'Q': 'px4_shipped'}

# S1-type scenarios: intervals with torque disturbances and discontinuities, detected from the
# logged injected disturbance (works for S1 and for the randomized S1r)
EXCL = 0.5   # s excluded after each discontinuity (transients)
AERO_AVG = 50   # samples (1 kHz) of the moving average of the measured aerodynamic wrench


def load_run(path, tend):
    L = load(path, ['ijc_debug', 'control_allocator_status', 'vehicle_local_position_groundtruth', 'ijc_aero'])
    d = L['ijc_debug']
    k = (d['t_scen'] >= 0) & (d['t_scen'] <= tend + 1e-9)
    P = L['_params']
    r = dict(t=d['t_scen'][k].astype(float))
    for f in ('p', 'v', 'p_d', 'v_d', 'e_p', 'e_v', 'f_hat', 't_hat', 'tau', 'tau_applied',
              'f_applied_dist', 't_applied_dist', 'torque_norm', 'f_lat_ff'):
        r[f] = vec(d, f)[k]
    for f in ('q', 'q_d'):
        r[f] = vec(d, f, 4)[k]
    for f in ('f', 'f_applied', 'thrust_norm'):
        r[f] = d[f][k]
    r['m'] = P['IJC_MASS']
    r['J'] = np.array([P['IJC_JXX'], P['IJC_JYY'], P['IJC_JZZ']])
    r['p0'] = r['p_d'][0].copy()
    r['params'] = P
    # aerodynamic wrench measured in Gazebo (IjcAeroSystem), aligned by simulation time
    aero = L.get('ijc_aero')
    if aero is not None and len(aero['t']) > 100:
        ts, td = aero['timestamp_sample'] * 1e-6, d['timestamp_sample'] * 1e-6   # full log, windowed below
        # 50 ms centered moving average: removes the white, per-physics-step part of the Gazebo wind
        # noise (not trackable by any observer), keeps the gusts
        smooth = lambda x: np.convolve(x, np.ones(AERO_AVG) / AERO_AVG, mode='same')  # noqa
        r['aero_F'] = np.column_stack([smooth(np.interp(td, ts, aero[f'force_ned[{i}]'])) for i in range(3)])[k]
        r['aero_T'] = np.column_stack([smooth(np.interp(td, ts, aero[f'torque_frd[{i}]'])) for i in range(3)])[k]
    # Gazebo ground truth (~60 Hz), origin aligned with the EKF at the scenario start
    g = L.get('vehicle_local_position_groundtruth')
    if g is not None:
        tt = d['t'][k]
        gt = np.column_stack([np.interp(tt, g['t'], g[a]) for a in 'xyz'])
        r['e_p_gt'] = gt - r['p_d'] + (r['p'][0] - gt[0])
    cas = L.get('control_allocator_status')
    if cas is not None:
        t0 = d['t'][k][0]
        kc = (cas['t'] >= t0) & (cas['t'] <= t0 + tend)
        r['torque_achieved'] = cas['torque_setpoint_achieved'][kc]
        r['thrust_achieved'] = cas['thrust_setpoint_achieved'][kc]
    return r


def eul_deg(Q):
    roll, pitch, yaw = quat_to_euler(Q)
    return np.degrees(pitch), np.degrees(roll), np.degrees(yaw)   # plots.py order: theta, phi, psi


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def fmt(ax, last, tend):
    ax.set_xlim(0, tend)
    ax.set_xticks(np.arange(0, tend + 1e-9, 5 if tend <= 30 else 10))
    if last:
        ax.set_xlabel('time (s)')


def pad_ylim(ax, *series, frac=0.12, min_span=None):
    lo = min(np.nanmin(s) for s in series)
    hi = max(np.nanmax(s) for s in series)
    span = hi - lo
    if min_span is not None and span < min_span:
        c = 0.5 * (hi + lo)
        lo, hi, span = c - min_span / 2, c + min_span / 2, min_span
    ax.set_ylim(lo - frac * span, hi + frac * span)


def save(fig, name):
    fig.savefig(f'{FIG}/{name}.pdf')
    fig.savefig(f'{FIG}/{name}.png', dpi=200)
    plt.close(fig)


def figures(S, runs, tend, true_dist):
    """runs: dict with keys 'C' (compensated), 'N' (non compensated) and optionally 'P' (PX4 standard)."""
    os.makedirs(FIG, exist_ok=True)
    C, N = runs['C'], runs['N']
    cases = [c for c in ('C', 'N', 'P') if c in runs]
    measured = (not true_dist) and 'aero_F' in C
    lbl_act = 'Actual' if true_dist else ('Actual (Gazebo aerodynamics)' if measured else 'Estimated (comp.)')
    lbl_obs = 'Observed' if (true_dist or measured) else 'Estimated (non comp.)'
    h2 = [Line2D([], [], color='k', lw=0.8), Line2D([], [], color=GR, ls='--', lw=0.8)]

    # ---------------- position figure (533 x 310 pt)
    fig = plt.figure(figsize=(533 / 72, 310 / 72))
    gs = fig.add_gridspec(4, 3, height_ratios=[1, 1, 0.12, 1], hspace=0.38, wspace=0.30, left=0.065, right=0.98, bottom=0.1, top=0.9)
    axs = np.array([[fig.add_subplot(gs[r, c]) for c in range(3)] for r in (0, 1, 3)])
    labs = [r'$x$ (m)', r'$y$ (m)', r'$z$ (m)', r'$\dot{x}$ (m/s)', r'$\dot{y}$ (m/s)', r'$\dot{z}$ (m/s)']
    for i in range(3):
        for r, (key, keyd, off) in enumerate([('p', 'p_d', True), ('v', 'v_d', False)]):
            ax = axs[r, i]
            yd = C[keyd][:, i] - (C['p0'][i] if off else 0)
            ax.plot(C['t'], yd, 'k-', lw=0.9)
            ys = [yd]
            for c in cases:
                R = runs[c]
                y = R[key][:, i] - (R['p0'][i] if off else 0)
                ax.plot(R['t'], y, **STY[c])
                ys.append(y)
            ax.set_ylabel(labs[3 * r + i]); fmt(ax, False, tend)
            pad_ylim(ax, *ys, min_span=0.1)
        ax = axs[2, i]
        if true_dist:
            ya, yo, to = C['m'] * C['f_applied_dist'][:, i], C['m'] * C['f_hat'][:, i], C['t']
        elif measured:
            ya, yo, to = C['aero_F'][:, i], C['m'] * C['f_hat'][:, i], C['t']
        else:
            ya, yo, to = C['m'] * C['f_hat'][:, i], N['m'] * N['f_hat'][:, i], N['t']
        ax.plot(C['t'], ya, 'k-', lw=0.8)
        ax.plot(to, yo, '--', color=GR, lw=0.8)
        ax.set_ylabel([r'$F_x$ (N)', r'$F_y$ (N)', r'$F_z$ (N)'][i]); fmt(ax, True, tend)
        pad_ylim(ax, ya, yo, min_span=0.5)
    h1 = [Line2D([], [], color='k', lw=0.9)] + [Line2D([], [], **STY[c]) for c in cases]
    b0 = axs[0, 0].get_position(); b2 = axs[2, 0].get_position()
    fig.legend(h1, ['Desired'] + [CASE_LABEL[c] for c in cases], loc='lower center', ncol=len(h1),
               bbox_to_anchor=(0.64, b0.y1 + 0.005), handlelength=3, borderpad=0.3, columnspacing=1.2)
    fig.text(0.18, b0.y1 + 0.02, 'Position states', ha='center', fontsize=8)
    fig.legend(h2, [lbl_act, lbl_obs], loc='lower center', ncol=2, bbox_to_anchor=(0.62, b2.y1 + 0.012), handlelength=3, borderpad=0.3)
    fig.text(0.24, b2.y1 + 0.024, 'Perturbations', ha='center', fontsize=8)
    save(fig, f'sitl_{S}_pos')

    # ---------------- attitude figure: one row per case (desired vs actual) + perturbations
    nrow = len(cases)
    height = 310 * (nrow + 1.12) / 3.12       # same row height as the 2-case figure of plots.py
    fig = plt.figure(figsize=(524 / 72, height / 72))
    gs = fig.add_gridspec(nrow + 2, 3, height_ratios=[1] * nrow + [0.12, 1], hspace=0.45, wspace=0.30,
                          left=0.065, right=0.98, bottom=0.1 * 310 / height, top=1 - 0.1 * 310 / height)
    axs = np.array([[fig.add_subplot(gs[r, c]) for c in range(3)] for r in list(range(nrow)) + [nrow + 1]])
    names = [r'$\theta$ (deg)', r'$\phi$ (deg)', r'$\psi$ (deg)']
    for r, c in enumerate(cases):
        R = runs[c]
        E, ED = eul_deg(R['q']), eul_deg(R['q_d'])
        for i in range(3):
            ax = axs[r, i]
            ax.plot(R['t'], ED[i], 'k-', lw=0.9); ax.plot(R['t'], E[i], '--', color=GR, lw=0.8)
            ax.set_ylabel(names[i]); fmt(ax, False, tend)
            pad_ylim(ax, ED[i], E[i], min_span=1.0)
        bb = axs[r, 2].get_position()
        fig.text(bb.x1, bb.y1 + 0.004, CASE_LABEL[c], ha='right', va='bottom', fontsize=6.5, style='italic')
    for i in range(3):
        j = [1, 0, 2][i]   # theta<->T_y, phi<->T_x, psi<->T_z
        ax = axs[nrow, i]
        if true_dist:
            ya, yo, to = C['J'][j] * C['t_applied_dist'][:, j], C['J'][j] * C['t_hat'][:, j], C['t']
        elif measured:
            ya, yo, to = C['aero_T'][:, j], C['J'][j] * C['t_hat'][:, j], C['t']
        else:
            ya, yo, to = C['J'][j] * C['t_hat'][:, j], N['J'][j] * N['t_hat'][:, j], N['t']
        ax.plot(C['t'], ya, 'k-', lw=0.8); ax.plot(to, yo, '--', color=GR, lw=0.8)
        ax.set_ylabel([r'$T_y$ (N$\cdot$m)', r'$T_x$ (N$\cdot$m)', r'$T_z$ (N$\cdot$m)'][i]); fmt(ax, True, tend)
        pad_ylim(ax, ya, yo, min_span=0.1)
    b0 = axs[0, 0].get_position(); b2 = axs[nrow, 0].get_position()
    h1 = [Line2D([], [], color='k', lw=0.9), Line2D([], [], color=GR, ls='--', lw=0.8)]
    fig.legend(h1, ['Desired', 'Actual'], loc='lower center', ncol=2, bbox_to_anchor=(0.62, b0.y1 + 0.005 * 310 / height), handlelength=3, borderpad=0.3)
    fig.text(0.24, b0.y1 + 0.02 * 310 / height, 'Attitude states', ha='center', fontsize=8)
    fig.legend(h2, [lbl_act, lbl_obs], loc='lower center', ncol=2, bbox_to_anchor=(0.62, b2.y1 + 0.012 * 310 / height), handlelength=3, borderpad=0.3)
    fig.text(0.24, b2.y1 + 0.024 * 310 / height, 'Perturbations', ha='center', fontsize=8)
    save(fig, f'sitl_{S}_att')

    # ---------------- inputs figure (612 x 111 pt), compensated case as in plots.py
    fig, axs = plt.subplots(1, 2, figsize=(612 / 72, 111 / 72))
    axs[0].plot(C['t'], C['f'], 'k-', lw=0.8); axs[0].set_ylabel(r'$f$ (N)'); fmt(axs[0], True, tend)
    pad_ylim(axs[0], C['f'], min_span=1.0)
    axs[1].plot(C['t'], C['tau'][:, 0], 'k-', lw=0.8, label=r'$\tau_x$')
    axs[1].plot(C['t'], C['tau'][:, 1], color=GR, lw=0.8, label=r'$\tau_y$')
    axs[1].plot(C['t'], C['tau'][:, 2], '--', color=GR, lw=0.8, label=r'$\tau_z$')
    axs[1].set_ylabel(r'$\tau$ (N$\cdot$m)'); fmt(axs[1], True, tend)
    pad_ylim(axs[1], C['tau'], frac=0.12)
    lo, hi = axs[1].get_ylim(); axs[1].set_ylim(lo, hi + 0.45 * (hi - lo))   # room for the legend
    axs[1].legend(loc='upper right', borderpad=0.3, handlelength=2.5, ncol=3)
    fig.tight_layout(w_pad=3)
    save(fig, f'sitl_{S}_inputs')

    # ---------------- inputs of the three cases (f and ||tau||), same size
    fig, axs = plt.subplots(1, 2, figsize=(612 / 72, 111 / 72))
    for c in cases:
        R = runs[c]
        axs[0].plot(R['t'], R['f'], label=CASE_LABEL[c], **STY[c])
        axs[1].plot(R['t'], np.linalg.norm(R['tau'], axis=1), label=CASE_LABEL[c], **STY[c])
    axs[0].set_ylabel(r'$f$ (N)'); fmt(axs[0], True, tend)
    axs[1].set_ylabel(r'$\|\tau\|$ (N$\cdot$m)'); fmt(axs[1], True, tend)
    pad_ylim(axs[0], *[runs[c]['f'] for c in cases], min_span=1.0)
    pad_ylim(axs[1], *[np.linalg.norm(runs[c]['tau'], axis=1) for c in cases])
    for ax in axs:
        lo, hi = ax.get_ylim(); ax.set_ylim(lo, hi + 0.45 * (hi - lo))
    axs[1].legend(loc='upper right', borderpad=0.3, handlelength=3, ncol=len(cases))
    fig.tight_layout(w_pad=3)
    save(fig, f'sitl_{S}_inputs_cmp')


def jump_times(t, x):
    """Times of the discontinuities of a logged disturbance signal (step > 5 % of its range in one sample)."""
    rng = np.ptp(x)
    if rng <= 0:
        return np.array([])
    return t[1:][np.abs(np.diff(x)) > 0.05 * rng]


def topview(S, runs):
    """x-y view of the trajectory (north up, east right), four cases, relative to the hover point."""
    fig, ax = plt.subplots(figsize=(260 / 72, 262 / 72))
    C = runs['C']
    ax.plot(C['p_d'][:, 1] - C['p0'][1], C['p_d'][:, 0] - C['p0'][0], 'k-', lw=0.9)
    sty = dict(STY, Q=dict(color='k', ls=':', lw=0.9))
    for c in ('C', 'N', 'P', 'Q'):
        if c in runs:
            R = runs[c]
            ax.plot(R['p'][:, 1] - R['p0'][1], R['p'][:, 0] - R['p0'][0], **sty[c])
    ax.set_xlabel(r'$y$ (m)'); ax.set_ylabel(r'$x$ (m)')
    ax.set_aspect('equal', adjustable='datalim')
    h = [Line2D([], [], color='k', lw=0.9)] + [Line2D([], [], **sty[c]) for c in ('C', 'N', 'P', 'Q') if c in runs]
    fig.legend(h, ['Desired'] + [CASE_LABEL[c] for c in ('C', 'N', 'P', 'Q') if c in runs], loc='upper center', ncol=2,
               handlelength=3, borderpad=0.3, columnspacing=1.0, bbox_to_anchor=(0.54, 0.995))
    fig.subplots_adjust(left=0.16, right=0.97, bottom=0.13, top=0.80)
    save(fig, f'sitl_{S}_xy')


def s1_masks(R, all_jumps=False):
    """Per-axis masks (theta, phi, psi) of the samples where the corresponding torque disturbance
    acts (T_y, T_x, T_z nonzero), excluding EXCL s after each jump of F_x (default, metric (a)) or
    after each jump of any disturbance component (all_jumps, metric (b))."""
    t, F, T = R['t'], R['f_applied_dist'], R['t_applied_dist']
    comps = [F[:, 0]] if not all_jumps else [F[:, i] for i in range(3)] + [T[:, i] for i in range(3)]
    ex = np.zeros_like(t, bool)
    for x in comps:
        for j in jump_times(t, x):
            ex |= (t >= j) & (t < j + EXCL)
    return [(np.abs(T[:, 1]) > 1e-6) & ~ex, (np.abs(T[:, 0]) > 1e-6) & ~ex, (np.abs(T[:, 2]) > 1e-6) & ~ex]


def metrics(S, case, R, true_dist):
    ep = R['e_p']
    EA, ED = np.array(eul_deg(R['q'])).T, np.array(eul_deg(R['q_d'])).T
    ea = wrap180(EA - ED)   # theta, phi, psi
    row = dict(scenario=S, case=CASE_ID[case])
    for i, a in enumerate('xyz'):
        row[f'max_ep_{a}'] = np.max(np.abs(ep[:, i]))
        row[f'rms_ep_{a}'] = np.sqrt(np.mean(ep[:, i] ** 2))
    for i, a in enumerate(['theta', 'phi', 'psi']):
        row[f'rms_eatt_{a}'] = np.sqrt(np.mean(ea[:, i] ** 2))
        row[f'max_eatt_{a}'] = np.max(np.abs(ea[:, i]))
    if 'e_p_gt' in R:
        for i, a in enumerate('xyz'):
            row[f'gt_max_ep_{a}'] = np.max(np.abs(R['e_p_gt'][:, i]))
            row[f'gt_rms_ep_{a}'] = np.sqrt(np.mean(R['e_p_gt'][:, i] ** 2))
    if S.startswith('S1'):
        for i, (a, mk) in enumerate(zip(['theta', 'phi', 'psi'], s1_masks(R))):
            row[f'rms_eatt_dist_{a}'] = np.sqrt(np.mean(ea[mk, i] ** 2))
        for i, (a, mk) in enumerate(zip(['theta', 'phi', 'psi'], s1_masks(R, all_jumps=True))):
            row[f'rms_eatt_dist2_{a}'] = np.sqrt(np.mean(ea[mk, i] ** 2))
            row[f'mean_eatt_dist2_{a}'] = np.mean(ea[mk, i])
    if case not in ('P', 'Q') and (true_dist or 'aero_F' in R):
        if true_dist:
            eF = R['m'] * (R['f_hat'] - R['f_applied_dist'])
            eT = R['J'] * (R['t_hat'] - R['t_applied_dist'])
        else:   # against the aerodynamic wrench measured in Gazebo
            eF = R['m'] * R['f_hat'] - R['aero_F']
            eT = R['J'] * R['t_hat'] - R['aero_T']
        for i, a in enumerate('xyz'):
            row[f'rms_eF_{a}'] = np.sqrt(np.mean(eF[:, i] ** 2))
        for i, a in enumerate('xyz'):
            row[f'rms_eT_{a}'] = np.sqrt(np.mean(eT[:, i] ** 2))
    row['max_f'] = np.max(R['f'])
    row['max_tau'] = np.max(np.linalg.norm(R['tau'], axis=1))
    row['sat_torque_pct'] = 100.0 * np.mean(np.any(np.abs(R['torque_norm']) >= 0.999, axis=1))
    if 'torque_achieved' in R:
        row['unalloc_pct'] = 100.0 * np.mean(~(R['torque_achieved'].astype(bool) & R['thrust_achieved'].astype(bool)))
    return {k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in row.items()}


COLS = ['scenario', 'case', 'n', 'max_ep_x', 'max_ep_y', 'max_ep_z', 'rms_ep_x', 'rms_ep_y', 'rms_ep_z',
        'rms_eatt_theta', 'rms_eatt_phi', 'rms_eatt_psi', 'max_eatt_theta', 'max_eatt_phi', 'max_eatt_psi',
        'rms_eatt_dist_theta', 'rms_eatt_dist_phi', 'rms_eatt_dist_psi',
        'rms_eatt_dist2_theta', 'rms_eatt_dist2_phi', 'rms_eatt_dist2_psi',
        'mean_eatt_dist2_theta', 'mean_eatt_dist2_phi', 'mean_eatt_dist2_psi',
        'rms_eF_x', 'rms_eF_y', 'rms_eF_z', 'rms_eT_x', 'rms_eT_y', 'rms_eT_z',
        'gt_max_ep_x', 'gt_max_ep_y', 'gt_max_ep_z', 'gt_rms_ep_x', 'gt_rms_ep_y', 'gt_rms_ep_z',
        'max_f', 'max_tau', 'sat_torque_pct', 'unalloc_pct']
NUM = COLS[3:]
COLS_ALL = COLS + [c + '_std' for c in NUM]
ORDER = {'compensated': 0, 'non_compensated': 1, 'px4_tuned': 2, 'px4_shipped': 3}
SHORT = {'compensated': 'Comp.', 'non_compensated': 'Non comp.', 'px4_tuned': 'PX4 tuned', 'px4_shipped': 'PX4 shipped'}


def update_tables(rows):
    os.makedirs(RES, exist_ok=True)
    path = f'{RES}/metrics.csv'
    old = []
    if os.path.exists(path):
        with open(path) as f:
            old = [r for r in csv.DictReader(f) if r['scenario'] not in {x['scenario'] for x in rows} and r['case'] in SHORT]
    allrows = old + [{k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()} for r in rows]
    for r in allrows:
        r.setdefault('n', '1')
    allrows.sort(key=lambda r: (r['scenario'], ORDER.get(r['case'], 9)))
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLS_ALL, restval='', extrasaction='ignore')
        w.writeheader(); w.writerows(allrows)

    def cell(r, k, nd=3):
        v = r.get(k, '')
        if v in ('', None):
            return '--'
        sd = r.get(k + '_std', '')
        return f'{float(v):.{nd}f}' if sd in ('', None) else f'{float(v):.{nd}f}$\\pm${float(sd):.{nd}f}'
    lines = [r'% generated by scripts/postprocess.py -- requires \usepackage{booktabs}',
             r'% position error against the Gazebo ground truth; S2/S3: mean $\pm$ std over the repetitions (n in metrics.csv)',
             r'\begin{tabular}{llrrrrrrrrrrr}', r'\toprule',
             r' & & \multicolumn{3}{c}{max $|e_p|$, ground truth (m)} & \multicolumn{3}{c}{RMS $e_p$, ground truth (m)} & \multicolumn{3}{c}{RMS att.\ error (deg)} & max $f$ & max $\|\tau\|$ \\',
             r'\cmidrule(lr){3-5}\cmidrule(lr){6-8}\cmidrule(lr){9-11}',
             r'Scen. & Case & $x$ & $y$ & $z$ & $x$ & $y$ & $z$ & $\theta$ & $\phi$ & $\psi$ & (N) & (N$\cdot$m) \\', r'\midrule']
    for r in allrows:
        lines.append(f"{r['scenario']} & {SHORT[r['case']]} & "
                     + ' & '.join(cell(r, k, 2 if k.startswith('gt_max') else 3) for k in ['gt_max_ep_x', 'gt_max_ep_y', 'gt_max_ep_z', 'gt_rms_ep_x', 'gt_rms_ep_y', 'gt_rms_ep_z'])
                     + ' & ' + ' & '.join(cell(r, k, 2) for k in ['rms_eatt_theta', 'rms_eatt_phi', 'rms_eatt_psi'])
                     + f" & {cell(r, 'max_f', 2)} & {cell(r, 'max_tau', 2)} \\\\")
    lines += [r'\bottomrule', r'\end{tabular}']
    s1 = [r for r in allrows if r['scenario'].startswith('S1')]
    if s1:
        lines += ['', r'% S1: RMS attitude error while each torque disturbance acts (T_y: t>7, T_x: pulses, T_z: t>=7),',
                  r'% (a) excluding 0.5 s after the F_x jumps, (b) excluding 0.5 s after every disturbance discontinuity;',
                  r'% and disturbance-estimation error of the observers',
                  r'\begin{tabular}{llrrrrrrrrrrrr}', r'\toprule',
                  r' & & \multicolumn{3}{c}{att.\ error (a) (deg)} & \multicolumn{3}{c}{att.\ error (b) (deg)} & \multicolumn{3}{c}{RMS $m(\hat F - F)$ (N)} & \multicolumn{3}{c}{RMS $J(\hat T - T)$ (N$\cdot$m)} \\',
                  r'\cmidrule(lr){3-5}\cmidrule(lr){6-8}\cmidrule(lr){9-11}\cmidrule(lr){12-14}',
                  r'Scen. & Case & $\theta$ & $\phi$ & $\psi$ & $\theta$ & $\phi$ & $\psi$ & $x$ & $y$ & $z$ & $x$ & $y$ & $z$ \\', r'\midrule']
        for r in s1:
            lines.append(f"{r['scenario']} & {SHORT[r['case']]} & "
                         + ' & '.join(cell(r, k, 2) for k in ['rms_eatt_dist_theta', 'rms_eatt_dist_phi', 'rms_eatt_dist_psi',
                                                              'rms_eatt_dist2_theta', 'rms_eatt_dist2_phi', 'rms_eatt_dist2_psi'])
                         + ' & ' + ' & '.join(cell(r, k) for k in ['rms_eF_x', 'rms_eF_y', 'rms_eF_z', 'rms_eT_x', 'rms_eT_y', 'rms_eT_z']) + r' \\')
        lines += [r'\bottomrule', r'\end{tabular}']
    s23 = [r for r in allrows if not r['scenario'].startswith('S1')]
    if s23:
        lines += ['', r'% S2/S3: position error seen by the controller (EKF), maximum attitude error, and estimation error of the',
                  r'% observers against the aerodynamic wrench measured in Gazebo (50 ms moving average); mean $\pm$ std',
                  r'\begin{tabular}{llrrrrrrrrrrrr}', r'\toprule',
                  r' & & \multicolumn{3}{c}{RMS $e_p$, EKF (m)} & \multicolumn{3}{c}{max att.\ error (deg)} & \multicolumn{3}{c}{RMS $m\hat F - F_a$ (N)} & \multicolumn{3}{c}{RMS $J\hat T - \tau_a$ (N$\cdot$m)} \\',
                  r'\cmidrule(lr){3-5}\cmidrule(lr){6-8}\cmidrule(lr){9-11}\cmidrule(lr){12-14}',
                  r'Scen. & Case & $x$ & $y$ & $z$ & $\theta$ & $\phi$ & $\psi$ & $x$ & $y$ & $z$ & $x$ & $y$ & $z$ \\', r'\midrule']
        for r in s23:
            lines.append(f"{r['scenario']} & {SHORT[r['case']]} & "
                         + ' & '.join(cell(r, k) for k in ['rms_ep_x', 'rms_ep_y', 'rms_ep_z'])
                         + ' & ' + ' & '.join(cell(r, k, 2) for k in ['max_eatt_theta', 'max_eatt_phi', 'max_eatt_psi'])
                         + ' & ' + ' & '.join(cell(r, k) for k in ['rms_eF_x', 'rms_eF_y', 'rms_eF_z', 'rms_eT_x', 'rms_eT_y', 'rms_eT_z']) + r' \\')
        lines += [r'\bottomrule', r'\end{tabular}']
    with open(f'{RES}/metrics_table.tex', 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenario')
    ap.add_argument('comp', nargs='?')
    ap.add_argument('nocomp', nargs='?')
    ap.add_argument('--reps', help='directory with <S>_<comp|nocomp|px4|px4ship>_r<i>.ulg: mean and std over the repetitions, figures of r1')
    ap.add_argument('--px4', help='ULog of the standard PX4 controllers, tuned (IJC_MODE=1); plotted')
    ap.add_argument('--px4-shipped', help='ULog of the standard PX4 controllers with the shipped tuning; metrics only')
    ap.add_argument('--tend', type=float, default=20.0)
    a = ap.parse_args()
    true_dist = a.scenario.startswith('S1')
    if a.reps:
        return main_reps(a, true_dist)
    runs = {'C': load_run(a.comp, a.tend), 'N': load_run(a.nocomp, a.tend)}
    assert runs['C']['params']['IJC_COMP'] == 1 and runs['N']['params']['IJC_COMP'] == 0, 'wrong compensation flags'
    assert runs['C']['params'].get('IJC_MODE', 0) == 0 and runs['N']['params'].get('IJC_MODE', 0) == 0
    if a.px4:
        runs['P'] = load_run(a.px4, a.tend)
        assert runs['P']['params']['IJC_MODE'] == 1, 'the PX4 log must have IJC_MODE = 1'
    figures(a.scenario, runs, a.tend, true_dist)
    if a.px4_shipped:
        runs['Q'] = load_run(a.px4_shipped, a.tend)
        assert runs['Q']['params']['IJC_MODE'] == 1
    rows = [metrics(a.scenario, c, runs[c], true_dist) for c in runs]
    update_tables(rows)
    for r in rows:
        print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})


SUFFIX = {'C': 'comp', 'N': 'nocomp', 'P': 'px4', 'Q': 'px4ship'}


def main_reps(a, true_dist):
    S = a.scenario
    files = {}
    for c, suf in SUFFIX.items():
        fs = glob.glob(f'{a.reps}/{S}_{suf}_r*.ulg')
        files[c] = sorted(fs, key=lambda p: int(re.search(r'_r(\d+)\.ulg$', p).group(1)))
    first = {c: load_run(files[c][0], a.tend) for c in files if files[c]}
    figures(S, {c: first[c] for c in ('C', 'N', 'P') if c in first}, a.tend, true_dist)
    if S == 'S3':
        topview(S, first)
    runs_rows, agg = [], []
    for c in files:
        rows = []
        for i, p in enumerate(files[c]):
            R = first[c] if i == 0 else load_run(p, a.tend)
            m = metrics(S, c, R, true_dist)
            m['run'] = os.path.basename(p)
            rows.append(m)
        runs_rows += rows
        keys = [k for k in rows[0] if isinstance(rows[0][k], float)]
        row = dict(scenario=S, case=CASE_ID[c], n=len(rows))
        for k in keys:
            v = np.array([r[k] for r in rows if k in r])
            row[k] = float(np.mean(v))
            row[k + '_std'] = float(np.std(v, ddof=1)) if len(v) > 1 else 0.0
        agg.append(row)
    # per-run metrics
    path = f'{RES}/metrics_runs.csv'
    old = []
    if os.path.exists(path):
        with open(path) as f:
            old = [r for r in csv.DictReader(f) if r['scenario'] != S]
    fields = ['run'] + [c for c in COLS if c != 'n']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, restval='', extrasaction='ignore')
        w.writeheader(); w.writerows(old)
        w.writerows([{k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()} for r in runs_rows])
    update_tables(agg)
    for r in agg:
        print(r['case'], r['n'], {k: round(r[k], 3) for k in ('gt_rms_ep_x', 'gt_rms_ep_y', 'gt_rms_ep_z', 'rms_eatt_theta', 'rms_eatt_phi', 'rms_eatt_psi') if k in r})


if __name__ == '__main__':
    main()
