#!/usr/bin/env python3
"""Summary of one calib_gust.sh round: RMS / max ||e_p|| (ground truth) and saturated time per case.

usage: calib_summary.py <out_dir> <profile> <scale> <kind>...
Prints the metrics and appends one line per kind to <out_dir>/summary.csv.
"""
import os
import sys

import numpy as np

import postprocess as pp

CASES = ('comp', 'nocomp', 'px4')


def main():
    out, prof, scale, kinds = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:]
    summary = f'{out}/summary.csv'
    new = not os.path.exists(summary)
    with open(summary, 'a') as fs:
        if new:
            fs.write('profile,scale,kind,rms_comp,rms_nocomp,rms_px4,max_comp,max_nocomp,max_px4,'
                     'unalloc_comp,unalloc_nocomp,unalloc_px4,px4_minus_comp\n')
        for k in kinds:
            r = {}
            for c in CASES:
                try:
                    R = pp.load_run(f'{out}/{k}_{c}_{prof}_s{scale}.ulg', 40.0)
                except Exception as e:  # noqa: BLE001  (crashed / missing run)
                    print(f'{k:6s} {c:6s} {prof} x{scale}: FAILED ({e})')
                    r[c] = (np.nan, np.nan, 100.0)
                    continue
                m = pp.metrics('S2', 'C', R, False)
                e = np.linalg.norm(R['e_p_gt'], axis=1)
                r[c] = (float(np.sqrt(np.mean(e ** 2))), float(e.max()), m['unalloc_pct'])
                print(f'{k:6s} {c:6s} {prof} x{scale}: unalloc {m["unalloc_pct"]:5.2f} %  '
                      f'rms|e_p| {r[c][0]:.3f} m  max|e_p| {r[c][1]:.3f} m  max f {m["max_f"]:.1f} N')
            d = r['px4'][0] - r['comp'][0]
            print(f'{k:6s} {prof} x{scale}: RMS px4 - comp = {d:+.3f} m, '
                  f'worst unalloc {max(v[2] for v in r.values()):.2f} %')
            fs.write(f'{prof},{scale},{k},' + ','.join(f'{r[c][j]:.4f}' for j in range(3) for c in CASES)
                     + f',{d:.4f}\n')


if __name__ == '__main__':
    main()
