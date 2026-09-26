"""Single-axis linear model of the ijc_control attitude loop with the rotational observer.

Loop (per axis, normalized by J): u = -(k_q/2) theta_m - K_Omega Omega_m - T_hat, with
  actuator      D(s) = exp(-s Td) / (Ta s + 1)     (rotor speed lag + delay, Pade(3))
  gyro filter   F(s) = wf^2 / (s^2 + 1.414 wf s + wf^2)   (IMU_GYRO_CUTOFF = 40 Hz)
  observer      T_hat = H(s) (F D - M) u,  H = lam^2 / (s + lam)^2,  M = 1 / (Tm s + 1)
(q_bar ~ theta/2 for small angles; Tm = IJC_TAU_MOT is the observer's actuator model).
"""
from math import factorial

import numpy as np
from numpy.polynomial import polynomial as P


def _pade(Td, n=3):
    num, den = [], []
    for k in range(n + 1):
        c = factorial(2 * n - k) * factorial(n) / (factorial(2 * n) * factorial(k) * factorial(n - k))
        num.append(c * (-Td) ** k)
        den.append(c * Td ** k)
    return np.array(num), np.array(den)


def max_real_pole(kq, kom, lam, Ta, Td, Tm, comp=True, wf=251.0):
    dn, dd = _pade(Td)
    Dn, Dd = dn, P.polymul(dd, [1, Ta])
    Fn, Fd = np.array([wf ** 2]), np.array([wf ** 2, 1.414 * wf, 1])
    FDn, FDd = P.polymul(Fn, Dn), P.polymul(Fd, Dd)
    Hn, Hd = np.array([lam ** 2]), P.polymul([lam, 1], [lam, 1])
    Mn, Md = np.array([1.0]), np.array([1.0, Tm])
    s2, s1 = np.array([0, 0, 1.0]), np.array([0, 1.0])
    c = 1.0 if comp else 0.0
    t0 = P.polymul(P.polymul(P.polymul(s2, FDd), Hd), Md)
    t1 = (kq / 2) * P.polymul(P.polymul(FDn, Hd), Md)
    t2 = kom * P.polymul(P.polymul(P.polymul(s1, FDn), Hd), Md)
    t3 = c * P.polymul(P.polymul(s2, Hn), P.polysub(P.polymul(FDn, Md), P.polymul(FDd, Mn)))
    ch = P.polyadd(P.polyadd(t0, t1), P.polyadd(t2, t3))
    return float(np.max(P.polyroots(ch).real))


def worst_case(kq, kom, lam, Tm=0.025):
    """Worst max Re(pole) over rotor lag 12.5-25 ms and delay 2-8 ms, with and without compensation."""
    return max(max_real_pole(kq, kom, lam, Ta, Td, Tm, comp)
               for Ta in np.linspace(0.0125, 0.025, 4) for Td in np.linspace(0.002, 0.008, 3) for comp in (True, False))
