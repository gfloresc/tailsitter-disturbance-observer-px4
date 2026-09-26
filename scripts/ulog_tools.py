"""Helpers to load ijc_control ULogs (pyulog) as numpy arrays."""
import numpy as np
from pyulog import ULog


def load(path, topics=None):
    """Return {topic_name[_instance]: {field: array, 't': seconds}} for the requested topics."""
    ulog = ULog(path, topics)
    out = {}
    for d in ulog.data_list:
        name = d.name if d.multi_id == 0 else f'{d.name}_{d.multi_id}'
        data = {k: np.asarray(v) for k, v in d.data.items()}
        data['t'] = data['timestamp'] * 1e-6
        out[name] = data
    out['_params'] = ulog.initial_parameters
    out['_changed_params'] = ulog.changed_parameters
    out['_messages'] = [(m.timestamp * 1e-6, m.message) for m in ulog.logged_messages]
    return out


def vec(d, field, n=3):
    """Stack field[0..n-1] columns into an (N, n) array."""
    return np.column_stack([d[f'{field}[{i}]'] for i in range(n)])


def quat_to_euler(q):
    """(N,4) [w,x,y,z] -> roll, pitch, yaw [rad] (ZYX)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def quat_rotate(q, v):
    """Rotate body vectors v (N,3) to the world frame with quaternions q (N,4) [w,x,y,z]."""
    w, x, y, z = q[:, 0:1], q[:, 1:2], q[:, 2:3], q[:, 3:4]
    u = np.hstack([x, y, z])
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)
