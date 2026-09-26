#!/usr/bin/env python3
"""Derive the thrust/torque normalization constants of ijc_control.

The PX4 control allocator receives *normalized* thrust/torque setpoints.  This
script reproduces, in numpy, exactly what the allocator does with them
(effectiveness matrix from the CA_* params, pseudo-inverse, per-axis
normalization of ControlAllocationPseudoInverse) and maps the resulting rotor
commands through the *physical* rotor model of the Gazebo SDF (rotor
positions and canted axes, motorConstant, momentConstant, maxRotVelocity,
spin directions).  The product gives the physical wrench produced per unit
normalized command, from which

    f = F_MAX * thrust_norm,   tau_i = TAU_MAX_i * torque_norm_i

It also reports mass, CoM and composite inertia (FRD, about the CoM).

Assumptions (set in the ijc airframe): THR_MDL_FAC = 1 (rotor thrust linear
in the allocator output), SIM_GZ_EC_MIN = 0, SIM_GZ_EC_MAX = maxRotVelocity.

Usage: normalization.py [path/to/model.sdf] [airframe file]
"""
import sys
import re
import xml.etree.ElementTree as ET
import numpy as np

from paths import PX4  # noqa: E402
SDF = sys.argv[1] if len(sys.argv) > 1 else f'{PX4}/Tools/simulation/gz/models/quadtailsitter/model.sdf'
AIRFRAME = sys.argv[2] if len(sys.argv) > 2 else \
    f'{PX4}/ROMFS/px4fmu_common/init.d-posix/airframes/4050_gz_quadtailsitter_ijc'
G = 9.80665

FLU2FRD = np.diag([1.0, -1.0, -1.0])


def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def pose(el):
    p = el.find('pose')
    v = np.zeros(6) if p is None else np.array([float(x) for x in p.text.split()])
    return v[:3], rpy2R(*v[3:])


def read_params(path):
    params = {}
    try:
        for line in open(path):
            m = re.match(r'\s*param set-default\s+(\S+)\s+(\S+)', line)
            if m:
                params[m.group(1)] = float(m.group(2))
    except FileNotFoundError:
        pass
    return params


root = ET.parse(SDF).getroot()
model = root.find('model')

# ---------------------------------------------------------------- mass properties
links = {}
for link in model.findall('link'):
    p_link, R_link = pose(link)
    inert = link.find('inertial')
    p_i, R_i = pose(inert)
    m = float(inert.find('mass').text)
    I = inert.find('inertia')
    Il = np.diag([float(I.find(k).text) for k in ('ixx', 'iyy', 'izz')])
    links[link.get('name')] = dict(p=p_link, R=R_link, m=m, c=p_link + R_link @ p_i, I=R_link @ R_i @ Il @ (R_link @ R_i).T)

# merged include of the airspeed sensor (model://airspeed at pose 0 0 0.7)
inc = model.find('include')
if inc is not None and 'airspeed' in inc.find('uri').text:
    asdf = ET.parse(f'{PX4}/Tools/simulation/gz/models/airspeed/model.sdf').getroot().find('model')
    p_inc, R_inc = pose(inc)
    for link in asdf.findall('link'):
        inert = link.find('inertial')
        m = float(inert.find('mass').text)
        I = inert.find('inertia')
        Il = np.diag([float(I.find(k).text) for k in ('ixx', 'iyy', 'izz')])
        links['airspeed_' + link.get('name')] = dict(p=p_inc, R=R_inc, m=m, c=p_inc, I=R_inc @ Il @ R_inc.T)

M = sum(l['m'] for l in links.values())
com = sum(l['m'] * l['c'] for l in links.values()) / M
J_flu = np.zeros((3, 3))
for l in links.values():
    d = l['c'] - com
    J_flu += l['I'] + l['m'] * (d @ d * np.eye(3) - np.outer(d, d))
J = FLU2FRD @ J_flu @ FLU2FRD

# ---------------------------------------------------------------- physical rotor model
rotors = []
for plg in model.findall('plugin'):
    if 'multicopter-motor-model' not in plg.get('filename'):
        continue
    link = links[plg.find('linkName').text]
    n = int(plg.find('motorNumber').text)
    kT = float(plg.find('motorConstant').text)
    km = float(plg.find('momentConstant').text)
    wmax = float(plg.find('maxRotVelocity').text)
    spin = 1.0 if plg.find('turningDirection').text == 'ccw' else -1.0
    axis = link['R'] @ np.array([0, 0, 1.0])
    rotors.append(dict(n=n, pos=link['p'] - com, axis=axis, Tmax=kT * wmax ** 2, km=km, spin=spin, wmax=wmax))
rotors.sort(key=lambda r: r['n'])

# wrench (FRD, about CoM) per unit of normalized rotor thrust t_i in [0,1]
B_phys = np.zeros((6, len(rotors)))
for i, r in enumerate(rotors):
    F = r['Tmax'] * r['axis']
    tau = np.cross(r['pos'], F) - r['spin'] * r['km'] * F     # gz MulticopterMotorModel drag torque
    B_phys[:3, i] = FLU2FRD @ tau
    B_phys[3:, i] = FLU2FRD @ F

# ---------------------------------------------------------------- PX4 allocator model
P = read_params(AIRFRAME)
B_alloc = np.zeros((6, len(rotors)))
for i in range(len(rotors)):
    pos = np.array([P.get(f'CA_ROTOR{i}_PX', 0.0), P.get(f'CA_ROTOR{i}_PY', 0.0), P.get(f'CA_ROTOR{i}_PZ', 0.0)])
    ax = np.array([P.get(f'CA_ROTOR{i}_AX', 0.0), P.get(f'CA_ROTOR{i}_AY', 0.0), P.get(f'CA_ROTOR{i}_AZ', -1.0)])
    ax /= np.linalg.norm(ax)
    ct = P.get(f'CA_ROTOR{i}_CT', 6.5)
    km = P.get(f'CA_ROTOR{i}_KM', 0.05)
    B_alloc[:3, i] = ct * np.cross(pos, ax) - ct * km * ax
    B_alloc[3:, i] = ct * ax

mix = np.linalg.pinv(B_alloc)                       # (n_rot x 6)
scale = np.ones(6)
nz = lambda c: np.count_nonzero(np.abs(c) > 1e-3)
r_s = np.sqrt(np.sum(mix[:, 0] ** 2) / (nz(mix[:, 0]) / 2.0))
p_s = np.sqrt(np.sum(mix[:, 1] ** 2) / (nz(mix[:, 1]) / 2.0))
scale[0] = scale[1] = max(r_s, p_s)
scale[2] = mix[:, 2].max()
for ax in (2, 1, 0):
    col = np.abs(mix[:, 3 + ax])
    k = np.count_nonzero(col > np.finfo(np.float32).eps)
    scale[3 + ax] = col.sum() / k if k else scale[5]
mix_n = mix / scale
mix_n[np.abs(mix_n) < 1e-3] = 0.0

W = B_phys @ mix_n       # physical wrench per unit normalized command (FRD)

np.set_printoptions(precision=5, suppress=True, linewidth=140)
print(f'SDF: {SDF}\nairframe: {AIRFRAME}\n')
print(f'mass m = {M:.5f} kg,  CoM (model FLU) = {com}')
print('J (FRD, about CoM) =\n', J)
for r in rotors:
    print(f"rotor {r['n']}: pos(FRD)={FLU2FRD @ r['pos']}, axis(FRD)={FLU2FRD @ r['axis']}, Tmax={r['Tmax']:.4f} N, spin={'ccw' if r['spin'] > 0 else 'cw'}")
print('\nB_phys (rows: Mx My Mz Fx Fy Fz) =\n', B_phys)
print('\nB_alloc =\n', B_alloc)
print('\nallocator scale =', scale)
print('\nW = B_phys * mix_normalized  (columns: tau_n x,y,z, thrust_n x,y,z) =\n', W)

F_MAX = W[5, 5]   # thrust_sp z = -thrust_norm  ->  Fz = -F_MAX * thrust_norm (upwards)
TAU_MAX = np.diag(W)[:3]
print(f'\nF_MAX (thrust_norm=1)       = {F_MAX:.5f} N')
print(f'TAU_MAX x,y,z (torque_norm=1) = {TAU_MAX} N m')
off = W[np.ix_([0, 1, 2, 5], [0, 1, 2, 5])] - np.diag(np.r_[TAU_MAX, W[5, 5]])
print(f'max |off-diagonal| in [Mx My Mz Fz] block = {np.abs(off).max():.5f}')
print(f'lateral force per unit torque cmd (Fx,Fy rows) =\n{W[3:5, :3]}')
print(f'\nhover: f = m g = {M * G:.4f} N  ->  thrust_norm = {M * G / F_MAX:.4f}')
alpha = TAU_MAX / np.diag(J)
print(f'max angular acceleration per axis (torque_norm=1) = {alpha} rad/s^2')
print(f'30% of it = {0.3 * alpha} rad/s^2  -> torque {0.3 * TAU_MAX} N m')
print(f'0.3 g = {0.3 * G:.4f} m/s^2 -> force {0.3 * G * M:.4f} N')

# ---------------------------------------------------------------- authority at hover (for the S1 amplitudes)
# single-axis normalized torque that saturates the first rotor (u in [0,1]) at hover thrust
t_hover = M * G / F_MAX
u_hover = mix_n[:, 5] * (-t_hover)
print(f'\nrotor commands at hover: {u_hover}')
c_max = np.zeros(3)
for ax in range(3):
    col = mix_n[:, ax]
    lim = [((1 - u) / c) if c > 0 else (u / -c) for u, c in zip(u_hover, col) if abs(c) > 1e-9]
    c_max[ax] = min(lim)
tau_hover = c_max * TAU_MAX
alpha_hover = tau_hover / np.diag(J)
print(f'max normalized torque per axis at hover = {c_max}')
print(f'max torque per axis at hover            = {tau_hover} N m')
print(f'max angular acceleration at hover       = {alpha_hover} rad/s^2')
print(f'A_T = 30% of it                         = {0.3 * alpha_hover} rad/s^2 -> J*A_T = {0.3 * tau_hover} N m')
print(f'A_F = 0.3 g = {0.3 * G:.4f} m/s^2 -> m*A_F = {0.3 * G * M:.4f} N')
print(f'\nIJC_FX_TY = {W[3, 1]:.4f} N (body x force per unit normalized pitch torque)')
print(f'IJC_FY_TX = {W[4, 0]:.4f} N (body y force per unit normalized roll torque)')
