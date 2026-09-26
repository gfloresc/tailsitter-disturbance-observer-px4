#!/usr/bin/env python3
"""Gain search for ijc_control and for the standard PX4 controllers on scenario S1.

usage: tune.py search <ijc|px4>      run all stages (resumable: evaluated candidates are cached)
       tune.py eval <ijc|px4> <name> PARAM=VALUE ...   evaluate one candidate
       tune.py validate <name> <ijc|px4> <seed> PARAM=VALUE ...   evaluate on S1r (IJC_DIST=2)

Criterion (lower is better), on S1 (A_T = 30 %), window t = 0-20 s:
    J = RMS|e_p| / 0.10 m + RMS_att(a) / 1 deg + 0.5 * unallocated_pct
  RMS|e_p|: 3-D RMS position error; RMS_att(a): RMS over the three axes of the attitude error
  while each torque disturbance acts, excluding 0.5 s after the F_x jumps (metric (a) of the
  README); unallocated_pct: % of time the allocator could not realize the thrust/torque.
  A candidate fails (J = inf) if the flight does not complete the scenario, if |e_p| > 3 m,
  or if the hover before the scenario is oscillatory (RMS attitude error > 1 deg in the last 5 s).
ijc_control candidates must also satisfy the time-scale hierarchy of Theorem 4.3 and be stable
in the linear model over the actuator uncertainty (worst max Re(pole) < -1 1/s).

Simulation runs faster than real time (SIM_SPEED = 4); results go to results/tuning/.
"""
import csv
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

import postprocess as pp
from linear_model import worst_case

from paths import ROOT  # noqa: E402  (repository root)
S = f'{ROOT}/scripts'
OUT = f'{ROOT}/results/tuning'
LOGS = f'{ROOT}/logs/tune'
SPEED = 4
S1 = {'IJC_DIST': 1, 'IJC_AF': 2.942, 'IJC_ATX': 9.106, 'IJC_ATY': 20.272, 'IJC_ATZ': 7.003}
AIRFRAME = {'ijc': '4050', 'px4': '4051'}
FIELDS = ['name', 'controller', 'stage', 'scenario', 'J', 'ok', 'reason', 'rms_pos3d', 'rms_att3d_a', 'unalloc_pct',
          'max_ep_x', 'max_ep_y', 'max_ep_z', 'rms_ep_x', 'rms_ep_y', 'rms_ep_z',
          'rms_eatt_dist_theta', 'rms_eatt_dist_phi', 'rms_eatt_dist_psi', 'hover_att_rms', 'params']


# ---------------------------------------------------------------- evaluation
def run_flight(ctrl, name, params, extra=None):
    env = dict(os.environ, SIM_SPEED=str(SPEED))
    subprocess.run([f'{S}/sitl.sh', 'stop'], capture_output=True)
    r = subprocess.run([f'{S}/sitl.sh', 'start', 'ijc_calm', AIRFRAME[ctrl]], env=env, capture_output=True, text=True)
    if r.returncode != 0:
        return None, 'sitl start failed'
    args = ['python3', f'{S}/ijc_flight.py', '--name', name, '--outdir', LOGS, '--speed', str(SPEED),
            '--settle', '10', '--duration', '22']
    for k, v in {**params, **(extra or {})}.items():
        args += ['--param', f'{k}={v}']
    try:
        f = subprocess.run(args, capture_output=True, text=True, timeout=240)
    except subprocess.TimeoutExpired:
        subprocess.run([f'{S}/sitl.sh', 'stop'], capture_output=True)
        return None, 'flight timeout'
    subprocess.run([f'{S}/sitl.sh', 'stop'], capture_output=True)
    if f.returncode != 0 or 'vehicle disarmed' in f.stdout:
        return None, 'flight failed: ' + (f.stdout + f.stderr).strip().splitlines()[-1][:80]
    return f'{LOGS}/{name}.ulg', ''


def evaluate(ctrl, name, stage, params, scenario='S1', extra=None):
    row = dict(name=name, controller=ctrl, stage=stage, scenario=scenario, params=json.dumps(params), ok=0, J=math.inf)
    log, reason = run_flight(ctrl, name, params, {**S1, **(extra or {})})
    if log is None:
        row['reason'] = reason
        return row
    return score_log(log, row)


def score_log(log, row=None):
    row = row if row is not None else dict(ok=0, J=math.inf)
    try:
        R = pp.load_run(log, 20.0)
        full = pp.load(log, ['ijc_debug'])['ijc_debug']
    except Exception as e:  # noqa
        row['reason'] = f'log: {e}'
        return row
    if R['t'][-1] < 19.9:
        row['reason'] = 'scenario incomplete'
        return row
    # hover before the scenario (last 5 s): must not be oscillatory
    k = (full['t_scen'] < 0) & (full['state'] == 2)
    th = full['t'][k]
    k2 = th > th[-1] - 5.0
    qa, qd = pp.vec(full, 'q', 4)[k][k2], pp.vec(full, 'q_d', 4)[k][k2]
    ea = pp.wrap180(np.array(pp.eul_deg(qa)).T - np.array(pp.eul_deg(qd)).T)
    row['hover_att_rms'] = float(np.sqrt(np.mean(ea ** 2)))
    m = pp.metrics('S1', 'C', R, True)
    row.update({k: m[k] for k in m if k in FIELDS})
    row['rms_pos3d'] = math.sqrt(m['rms_ep_x'] ** 2 + m['rms_ep_y'] ** 2 + m['rms_ep_z'] ** 2)
    row['rms_att3d_a'] = math.sqrt((m['rms_eatt_dist_theta'] ** 2 + m['rms_eatt_dist_phi'] ** 2 + m['rms_eatt_dist_psi'] ** 2) / 3)
    row['unalloc_pct'] = m.get('unalloc_pct', 0.0)
    if max(m['max_ep_x'], m['max_ep_y'], m['max_ep_z']) > 3.0:
        row['reason'] = 'position error > 3 m'
    elif row['hover_att_rms'] > 1.0:
        row['reason'] = 'oscillatory hover'
    else:
        row['ok'] = 1
        row['J'] = row['rms_pos3d'] / 0.10 + row['rms_att3d_a'] / 1.0 + 0.5 * row['unalloc_pct']
    return row


def table_path(ctrl):
    return f'{OUT}/tuning_{ctrl}.csv'


def load_table(ctrl):
    p = table_path(ctrl)
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def append(ctrl, row):
    os.makedirs(OUT, exist_ok=True)
    p = table_path(ctrl)
    new = not os.path.exists(p)
    with open(p, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow({k: (f'{v:.5g}' if isinstance(v, float) else v) for k, v in row.items()})


def run_candidates(ctrl, stage, cands):
    """cands: list of (name, params). Evaluates the ones not in the table; returns all rows of the stage."""
    done = {r['name'] for r in load_table(ctrl)}
    for name, params in cands:
        if name in done:
            continue
        row = evaluate(ctrl, name, stage, params)
        append(ctrl, row)
        print(f"[{ctrl}/{stage}] {name}: J = {row['J']:.3f} {row.get('reason', '')}", flush=True)
    names = {n for n, _ in cands}
    return [r for r in load_table(ctrl) if r['name'] in names]


def best(rows, n=1):
    ok = [r for r in rows if r['ok'] == '1']
    ok.sort(key=lambda r: float(r['J']))
    return ok[:n]


# ---------------------------------------------------------------- ijc_control parameterization
def ijc_params(wa, za, lam1, lam2, wp, zp=1.0):
    """attitude: s^2 + 2 za wa s + wa^2 (k_q/2 = wa^2, q_bar ~ theta/2); observers: double poles at -lam;
    position: s^2 + 2 zp wp s + wp^2."""
    return {'IJC_KQ': round(2 * wa ** 2, 3), 'IJC_KOM': round(2 * za * wa, 4),
            'IJC_K1': round(2 * lam1, 3), 'IJC_L1': round(lam1 ** 2, 3),
            'IJC_K2': round(2 * lam2, 3), 'IJC_L2': round(lam2 ** 2, 3),
            'IJC_KP': round(wp ** 2, 4), 'IJC_KV': round(2 * zp * wp, 4)}


def slow_att_pole(wa, za):
    return wa * (za - math.sqrt(za ** 2 - 1)) if za >= 1 else wa * za


def ijc_feasible(wa, za, lam1, lam2, wp, zp=1.0):
    """Theorem 4.3 hierarchy + linear stability. Returns (ok, reason)."""
    ps = slow_att_pole(wa, za)
    if ps < 3 * wp * 0.999:
        return False, 'attitude not faster than position'
    if lam1 < ps * 0.999 or lam1 < 3 * wp:
        return False, 'rotational observer too slow'
    if lam2 > ps * 1.001:
        return False, 'translational observer not slower than attitude'
    if lam2 < 1.5 * wp:
        return False, 'translational observer slower than position loop'
    p = ijc_params(wa, za, lam1, lam2, wp, zp)
    wc = worst_case(p['IJC_KQ'], p['IJC_KOM'], lam1)
    if wc > -1.0:
        return False, f'linear worst case max Re = {wc:+.2f}'
    return True, ''


def ijc_name(wa, za, lam1, lam2, wp, zp=1.0):
    return f'ijc_wa{wa:g}_za{za:g}_l1{lam1:g}_l2{lam2:g}_wp{wp:g}_zp{zp:g}'


def ijc_cand(wa, za, lam1, lam2, wp, zp=1.0):
    return ijc_name(wa, za, lam1, lam2, wp, zp), ijc_params(wa, za, lam1, lam2, wp, zp), (wa, za, lam1, lam2, wp, zp)


def search_ijc():
    BASE = (14.1421, 1.0607, 80.0, 10.0, 2.0, 1.0)     # gains of the previous S1 (k_q 400, K_Om 30, ...)

    def filt(items):
        out = []
        for it in items:
            ok, why = ijc_feasible(*it)
            if ok:
                out.append(ijc_cand(*it)[:2])
            else:
                print(f'  skip {ijc_name(*it)}: {why}')
        return out

    # stage 0: baseline, repeated (noise level of J)
    run_candidates('ijc', 'S0', [(ijc_name(*BASE) + f'_r{i}', ijc_params(*BASE)) for i in range(3)])
    # stage 1: attitude and rotational observer
    s1 = []
    for wa in (10.0, 14.1421, 18.0, 22.0, 26.0):
        for za in (0.8, 1.0607):
            for lam1 in (40.0, 80.0, 120.0):
                s1.append((wa, za, lam1, min(10.0, slow_att_pole(wa, za)), 2.0, 1.0))
    rows = run_candidates('ijc', 'S1', filt(s1))
    b = best(rows)[0]
    wa, za, lam1 = parse_ijc(b['name'])[:3]
    print('stage 1 best:', b['name'], b['J'])
    # stage 2: translational observer and position loop
    ps = slow_att_pole(wa, za)
    s2 = [(wa, za, lam1, lam2, wp, 1.0) for lam2 in sorted({5.0, 7.0, 10.0, 14.0, 18.0, round(ps, 2)})
          for wp in (1.5, 2.0, 2.5, 3.0)]
    rows = run_candidates('ijc', 'S2', filt(s2))
    b = best(rows)[0]
    print('stage 2 best:', b['name'], b['J'])
    t = parse_ijc(b['name'])
    # stage 3: local refinement (one parameter at a time)
    wa, za, lam1, lam2, wp, zp = t
    s3 = [(wa, za, lam1, lam2, wp, 0.8), (wa, za, lam1, lam2, wp, 1.25),
          (wa * 0.9, za, lam1, min(lam2, slow_att_pole(wa * 0.9, za)), wp, zp), (wa * 1.1, za, lam1, lam2, wp, zp),
          (wa, za, lam1 * 0.75, lam2, wp, zp), (wa, za, lam1 * 1.25, lam2, wp, zp)]
    s3 = [tuple(round(x, 4) for x in c) for c in s3]
    run_candidates('ijc', 'S3', filt(s3))
    final_repeats('ijc', parse_ijc, lambda c: ijc_params(*c), ijc_name)


def strip_rep(name):
    return re.sub(r'_r\d+$', '', name)


def parse_ijc(name):
    v = strip_rep(name).replace('ijc_', '').split('_')
    return tuple(float(x[2:]) for x in v)


# ---------------------------------------------------------------- PX4 parameterization
PX4_SHIPPED = dict(att=3.0, rate=1.0, xy_p=0.95, vel_p=1.8, vel_i=0.4, yaw=1.0, z=1.0)


def px4_params(att, rate, xy_p, vel_p, vel_i, yaw=1.0, z=1.0):
    """Scaled from the tuning PX4 ships for the quadtailsitter (rc.vtol_defaults + 4018)."""
    return {'MC_ROLL_P': att, 'MC_PITCH_P': att,
            'MC_ROLLRATE_P': round(0.3 * rate, 4), 'MC_PITCHRATE_P': round(0.3 * rate, 4),
            'MC_ROLLRATE_I': round(0.2 * rate, 4), 'MC_PITCHRATE_I': round(0.2 * rate, 4),
            'MC_ROLLRATE_D': round(0.003 * rate, 5), 'MC_PITCHRATE_D': round(0.003 * rate, 5),
            'MC_YAW_P': round(2.0 * yaw, 3), 'MC_YAWRATE_P': round(0.2 * yaw, 4), 'MC_YAWRATE_I': round(0.1 * yaw, 4),
            'MPC_XY_P': xy_p, 'MPC_XY_VEL_P_ACC': vel_p, 'MPC_XY_VEL_I_ACC': vel_i, 'MPC_XY_VEL_D_ACC': 0.2,
            'MPC_Z_P': round(1.0 * z, 3), 'MPC_Z_VEL_P_ACC': round(4.0 * z, 3), 'MPC_Z_VEL_I_ACC': round(2.0 * z, 3)}


def px4_name(att, rate, xy_p, vel_p, vel_i, yaw=1.0, z=1.0):
    return f'px4_att{att:g}_rate{rate:g}_xyp{xy_p:g}_velp{vel_p:g}_veli{vel_i:g}_yaw{yaw:g}_z{z:g}'


def parse_px4(name):
    v = strip_rep(name).replace('px4_', '').split('_')
    keys = ('att', 'rate', 'xyp', 'velp', 'veli', 'yaw', 'z')
    out = []
    for x in v:
        for k in keys:
            if x.startswith(k):
                out.append(float(x[len(k):]))
                break
    return tuple(out)


def search_px4():
    B = PX4_SHIPPED
    base = (B['att'], B['rate'], B['xy_p'], B['vel_p'], B['vel_i'], B['yaw'], B['z'])
    cand = lambda c: (px4_name(*c), px4_params(*c))  # noqa
    run_candidates('px4', 'S0', [(px4_name(*base) + f'_r{i}', px4_params(*base)) for i in range(3)])
    # stage 1: inner loops (attitude P, rate gains scaled together)
    s1 = [(att, rate, B['xy_p'], B['vel_p'], B['vel_i'], 1.0, 1.0) for att in (3.0, 5.0, 7.0, 9.0) for rate in (0.5, 1.0, 1.5, 2.0, 3.0)]
    rows = run_candidates('px4', 'S1', [cand(c) for c in s1])
    att, rate = parse_px4(best(rows)[0]['name'])[:2]
    print('stage 1 best:', best(rows)[0]['name'])
    # stage 2: horizontal position / velocity loops (velocity integrator scaled with the P gain)
    s2 = [(att, rate, xy_p, vel_p, round(vel_p * vi, 3), 1.0, 1.0) for xy_p in (0.95, 1.5, 2.5) for vel_p in (1.8, 3.0, 4.5)
          for vi in (0.4 / 1.8, 1.0)]
    rows = run_candidates('px4', 'S2', [cand(c) for c in s2])
    b = parse_px4(best(rows)[0]['name'])
    print('stage 2 best:', best(rows)[0]['name'])
    # stage 3: yaw and vertical loops, local refinement
    att, rate, xy_p, vel_p, vel_i, yaw, z = b
    s3 = [(att, rate, xy_p, vel_p, vel_i, 2.0, 1.0), (att, rate, xy_p, vel_p, vel_i, 3.0, 1.0),
          (att, rate, xy_p, vel_p, vel_i, 1.0, 1.5), (att, rate * 1.25, xy_p, vel_p, vel_i, 1.0, 1.0),
          (att * 1.25, rate, xy_p, vel_p, vel_i, 1.0, 1.0), (att, rate, xy_p * 1.25, vel_p, vel_i, 1.0, 1.0)]
    run_candidates('px4', 'S3', [cand(tuple(round(x, 4) for x in c)) for c in s3])
    final_repeats('px4', parse_px4, lambda c: px4_params(*c), px4_name)


# ---------------------------------------------------------------- final selection
def final_repeats(ctrl, parse, mk, name_of, n_top=3, n_rep=3):
    """Re-evaluate the n_top candidates until each has n_rep evaluations; select the lowest mean J."""
    rows = [r for r in load_table(ctrl) if r['stage'] != 'S4']
    tops, seen = [], set()
    for r in best(rows, 50):
        key = parse(r['name'])
        if key not in seen:
            seen.add(key)
            tops.append(key)
        if len(tops) == n_top:
            break
    for key in tops:
        base = name_of(*key)
        have = [r for r in load_table(ctrl) if parse(r['name']) == key]
        reps = [(f'{base}_r{i}', mk(key)) for i in range(len(have), n_rep)]
        run_candidates(ctrl, 'S4', reps)
    summary(ctrl, parse)


def summary(ctrl, parse):
    groups = {}
    for r in load_table(ctrl):
        groups.setdefault(parse(r['name']), []).append(r)
    res = []
    for key, rs in groups.items():
        Js = [float(r['J']) for r in rs]
        res.append((np.mean(Js) if all(np.isfinite(Js)) else math.inf, np.std(Js), len(rs), key, rs[0]['name']))
    res.sort(key=lambda x: x[0])
    print(f'\n{ctrl}: best candidates (mean J over repeats)')
    for m, s, n, key, nm in res[:8]:
        print(f'  J = {m:.3f} +- {s:.3f} (n={n})  {strip_rep(nm)}')
    return res


def main():
    cmd = sys.argv[1]
    if cmd == 'search':
        {'ijc': search_ijc, 'px4': search_px4}[sys.argv[2]]()
    elif cmd == 'eval':
        ctrl, name = sys.argv[2], sys.argv[3]
        params = {k: float(v) if '.' in v else int(v) for k, v in (a.split('=') for a in sys.argv[4:])}
        row = evaluate(ctrl, name, 'manual', params)
        append(ctrl, row)
        print(row)
    elif cmd == 'validate':
        # tune.py validate <ijc|px4> <label> <seed> PARAM=VALUE ...  -> S1r (IJC_DIST = 2), results/tuning/validation.csv
        ctrl, label, seed = sys.argv[2], sys.argv[3], int(sys.argv[4])
        params = {k: float(v) if '.' in v else int(v) for k, v in (a.split('=') for a in sys.argv[5:])}
        name = f'val_{label}_seed{seed}'
        row = evaluate(ctrl, name, 'validation', params, scenario=f'S1r seed {seed}', extra={'IJC_DIST': 2, 'IJC_DSEED': seed})
        os.makedirs(OUT, exist_ok=True)
        p = f'{OUT}/validation.csv'
        new = not os.path.exists(p)
        with open(p, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction='ignore')
            if new:
                w.writeheader()
            w.writerow({k: (f'{v:.5g}' if isinstance(v, float) else v) for k, v in row.items()})
        print(name, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()
                     if k in ('J', 'ok', 'reason', 'rms_pos3d', 'rms_att3d_a', 'unalloc_pct')})
    elif cmd == 'score':
        for lg in sys.argv[2:]:
            r = score_log(lg)
            print(os.path.basename(lg), {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()
                                         if k in ('J', 'ok', 'reason', 'rms_pos3d', 'rms_att3d_a', 'unalloc_pct', 'hover_att_rms')})
    elif cmd == 'summary':
        summary(sys.argv[2], parse_ijc if sys.argv[2] == 'ijc' else parse_px4)


if __name__ == '__main__':
    main()
