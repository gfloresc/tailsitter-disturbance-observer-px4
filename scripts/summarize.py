#!/usr/bin/env python3
"""Print a condensed time series of an ijc_control ULog (debugging aid).

usage: summarize.py <log.ulg> [step_s] [t_from] [t_to]
"""
import sys
import numpy as np
from ulog_tools import load, vec, quat_to_euler

path = sys.argv[1]
step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
L = load(path, ['ijc_debug'])
d = L['ijc_debug']
t = d['t'] - d['t'][0]
t_from = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
t_to = float(sys.argv[4]) if len(sys.argv) > 4 else t[-1]
for ts, m in L['_messages']:
    print('%7.2f %s' % (ts - d['t'][0], m.strip()))
print('samples %d, dt median %.4f max %.4f' % (len(t), np.median(np.diff(t)), np.max(np.diff(t))))

ep, fh, th = vec(d, 'e_p'), vec(d, 'f_hat'), vec(d, 't_hat')
tn = vec(d, 'torque_norm')
r, p, y = quat_to_euler(vec(d, 'q', 4))
rd, pd, yd = quat_to_euler(vec(d, 'q_d', 4))
print('    t st  e_p [m]              F_hat [m/s2]         T_hat [rad/s2]          f[N]  thr   tau_norm             rpy [deg]             rpy_d [deg]')
for i in np.searchsorted(t, np.arange(t_from, t_to, step)):
    print('%5.1f %d [%6.2f %6.2f %6.2f] [%6.2f %6.2f %6.2f] [%7.2f %7.2f %7.2f] %6.2f %5.3f [%5.2f %5.2f %5.2f] [%6.1f %6.1f %6.1f] [%6.1f %6.1f %6.1f]' % (
        t[i], d['state'][i], *ep[i], *fh[i], *th[i], d['f'][i], d['thrust_norm'][i], *tn[i],
        *np.degrees([r[i], p[i], y[i]]), *np.degrees([rd[i], pd[i], yd[i]])))
