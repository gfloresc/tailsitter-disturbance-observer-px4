#!/usr/bin/env python3
"""Automated ijc_control flight over MAVLink (PX4 SITL, airframe 4050_gz_quadtailsitter_ijc).

Procedure: set parameters -> Offboard mode -> arm (ijc_control takes off with its
own reference) -> settle in hover -> IJC_CMD=1 (scenario timer t=0) -> run for
--duration s -> IJC_CMD=2 (ijc_control lands) -> disarm -> copy the ULog.

No PX4 standard controller is involved at any stage.

Example:
  ijc_flight.py --name S1_comp --param IJC_COMP=1 --param IJC_DIST=1 --duration 25
"""
import argparse
import glob
import json
import os
import shutil
import struct
import sys
import time

from pymavlink import mavutil

from paths import PX4, ROOT  # noqa: E402
LOG_ROOT = f'{PX4}/build/px4_sitl_default/rootfs/log'
PX4_MAIN_MODE_OFFBOARD = 6


class Vehicle:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.m.wait_heartbeat(timeout=60)
        self.last_hb = 0.0
        self.armed = False
        self.custom_mode = 0
        self.z = float('nan')
        self.vz = float('nan')
        self.boot_ms = None   # vehicle time (= simulation time in SITL)
        self.statustexts = []
        self.dbg = None       # latest DEBUG_FLOAT_ARRAY 'IJC' of ijc_control (see LiveMonitor)
        self.tick = None      # optional callback, called for every received message

    def _gcs_heartbeat(self):
        if time.time() - self.last_hb > 0.5:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_hb = time.time()

    def spin(self, seconds, cond=None):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self._gcs_heartbeat()
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            t = msg.get_type()
            if hasattr(msg, 'time_boot_ms') and msg.get_srcSystem() == self.m.target_system:
                self.boot_ms = msg.time_boot_ms
            if t == 'HEARTBEAT' and msg.get_srcSystem() == self.m.target_system and msg.get_srcComponent() == 1:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.custom_mode = msg.custom_mode
            elif t == 'LOCAL_POSITION_NED':
                self.z, self.vz = msg.z, msg.vz
            elif t == 'DEBUG_FLOAT_ARRAY' and msg.name == 'IJC':
                self.dbg = list(msg.data)
            elif t == 'STATUSTEXT':
                print(f'  [px4] {msg.text}', flush=True)
                self.statustexts.append(msg.text)
            if self.tick is not None:
                self.tick()
            if cond is not None and cond(msg):
                return msg
        return None

    def spin_sim(self, seconds, cond=None, wall_timeout=None):
        """Like spin, but waits for `seconds` of vehicle (simulation) time."""
        while self.boot_ms is None:
            self.spin(0.1)
        t0 = self.boot_ms
        w0 = time.time()
        while self.boot_ms - t0 < seconds * 1000.0:
            if self.boot_ms < t0 - 1000:
                sys.exit('simulation time went backwards (world reset in Gazebo?): stopping')
            msg = self.spin(0.01, cond)   # fine steps: the waits resolve to the vehicle-time message rate
            if msg is not None:
                return msg
            if wall_timeout is not None and time.time() - w0 > wall_timeout:
                break
        return None

    def command(self, cmd, *params, timeout=5.0):
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
        ack = self.spin(timeout, lambda x: x.get_type() == 'COMMAND_ACK' and x.command == cmd)
        return None if ack is None else ack.result

    def set_param(self, name, value, retries=5):
        is_int = isinstance(value, int)
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        if is_int:  # PX4 uses bytewise encoding for integers
            wire = struct.unpack('<f', struct.pack('<i', value))[0]
        else:
            wire = float(value)
        for _ in range(retries):
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), wire, ptype)
            msg = self.spin(2.0, lambda x: x.get_type() == 'PARAM_VALUE' and x.param_id == name)
            if msg is not None:
                if is_int:
                    got = struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                    ok = got == value
                else:
                    ok = abs(msg.param_value - value) <= 1e-6 * max(1.0, abs(value))
                if ok:
                    return True
        raise RuntimeError(f'could not set {name}={value}')


def parse_value(s):
    try:
        return int(s)
    except ValueError:
        return float(s)


def newest_log(after):
    logs = [f for f in glob.glob(f'{LOG_ROOT}/*/*.ulg') if os.path.getmtime(f) >= after]
    return max(logs, key=os.path.getmtime) if logs else None


class LiveMonitor:
    """Live position error in the terminal (--live): ground truth from Gazebo (/world/<w>/pose/info) and
    the reference/EKF position of ijc_control (DEBUG_FLOAT_ARRAY 'IJC', 20 Hz: t_scen, p_d, p, e_p, state).
    The ground truth is expressed in the EKF frame by aligning both positions at the start of the takeoff
    (as the metrics align them at the scenario start)."""
    STATES = {0: 'idle', 1: 'takeoff', 2: 'hover', 3: 'landing', 4: 'touchdown'}

    def __init__(self, vehicle, world, model='quadtailsitter_ijc_0', period=0.5):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
        self.v, self.model, self.period = vehicle, model, period
        self.gt = None
        self.offset = None
        self.last_print = 0.0
        self.last_t = None
        self.last_state = None
        self.samples = []    # (|e_p| ground truth, |e_p| EKF) during the scenario
        self.node = Node()
        self.node.subscribe(Pose_V, f'/world/{world}/pose/info', self._on_pose)
        vehicle.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 350, 50000)   # DEBUG_FLOAT_ARRAY, 20 Hz
        vehicle.tick = self.tick

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                self.gt = (p.position.y, p.position.x, -p.position.z)   # ENU -> NED
                return

    def tick(self):
        d = self.v.dbg
        if d is None:
            return
        t_scen, p_d, p, e_ekf, state = d[0], d[1:4], d[4:7], d[7:10], int(d[10])
        gt = self.gt
        if gt is not None and self.offset is None and state == 1:
            self.offset = [p[i] - gt[i] for i in range(3)]
        e_gt = None
        if gt is not None and self.offset is not None:
            e_gt = [gt[i] + self.offset[i] - p_d[i] for i in range(3)]
        n_ekf = sum(x * x for x in e_ekf) ** 0.5
        n_gt = sum(x * x for x in e_gt) ** 0.5 if e_gt else float('nan')
        if t_scen >= 0 and t_scen != self.last_t and state == 2:
            self.last_t = t_scen
            self.samples.append((n_gt, n_ekf))
        now = time.time()
        if state == 0 or (state >= 3 and state == self.last_state):   # landing phases: print once
            return
        if now - self.last_print < self.period and state == self.last_state:
            return
        self.last_print = now
        self.last_state = state
        e = e_gt if e_gt else e_ekf
        phase = 'scenario' if (t_scen >= 0 and state == 2) else self.STATES.get(state, '?')
        ts = f't = {t_scen:6.2f} s' if t_scen >= 0 else 't =    --  '
        print(f'  {ts}  {phase:9s} |e_p| = {n_gt:5.3f} m (ground truth)  {n_ekf:5.3f} m (EKF)   '
              f'e_p NED = ({e[0]:+5.2f}, {e[1]:+5.2f}, {e[2]:+5.2f}) m', flush=True)

    def summary(self):
        if not self.samples:
            return 'no scenario samples'
        n = len(self.samples)
        rms = [(sum(s[j] ** 2 for s in self.samples) / n) ** 0.5 for j in range(2)]
        mx = max(s[0] for s in self.samples)
        return (f'scenario: RMS |e_p| = {rms[0]:.3f} m (ground truth), {rms[1]:.3f} m (EKF); '
                f'max {mx:.3f} m (ground truth); {n} samples at 20 Hz')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True, help='run name (ULog copied to logs/<name>.ulg)')
    ap.add_argument('--param', action='append', default=[], help='NAME=VALUE, set before arming')
    ap.add_argument('--settle', type=float, default=5.0, help='hover time before the scenario [s]')
    ap.add_argument('--duration', type=float, default=25.0, help='scenario duration [s]')
    ap.add_argument('--speed', type=float, default=1.0, help='simulation speed factor (SIM_SPEED of sitl.sh)')
    ap.add_argument('--simtime', action='store_true', help='wait in vehicle (simulation) time instead of wall time')
    ap.add_argument('--gust-topic', default='', help='gz topic that starts the IjcGustSystem wind (e.g. /ijc_gust_start)')
    ap.add_argument('--gust-lead', type=float, default=8.0, help='wind start before the scenario start [s, simulated]')
    ap.add_argument('--live', action='store_true', help='print the position error live (needs --world)')
    ap.add_argument('--world', default='', help='Gazebo world name (for --live ground truth)')
    ap.add_argument('--url', default='udpin:127.0.0.1:14540')
    ap.add_argument('--outdir', default=os.path.join(ROOT, 'logs'))
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    v = Vehicle(args.url)
    print(f'connected to system {v.m.target_system}', flush=True)
    live = LiveMonitor(v, args.world) if args.live else None

    params = {'IJC_CMD': 0}
    for kv in args.param:
        k, val = kv.split('=')
        params[k] = parse_value(val)
    for k, val in params.items():
        v.set_param(k, val)
        print(f'  {k} = {val}', flush=True)

    # pre-arm checks
    ok = v.spin(90, lambda x: x.get_type() == 'SYS_STATUS' and
                x.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
    print('pre-arm checks ok' if ok else 'WARNING: pre-arm flag not seen', flush=True)

    t_start = time.time()
    res = v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    PX4_MAIN_MODE_OFFBOARD)
    print(f'offboard mode: result {res}', flush=True)
    res = v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    print(f'arm: result {res}', flush=True)
    if res != 0:
        sys.exit('arming failed')

    gust = None
    if args.gust_topic:   # advertise early so that gz-transport discovery is done when the trigger is sent
        from gz.msgs10.empty_pb2 import Empty
        from gz.transport13 import Node
        gz_node = Node()
        gust = gz_node.advertise(args.gust_topic, Empty)

    sp = args.speed
    wait = (lambda s, cond=None: v.spin_sim(s, cond, wall_timeout=20 * s + 60)) if args.simtime else \
        (lambda s, cond=None: v.spin(s / sp, cond))
    # takeoff (IJC_TKO_T, default 5 s) + settle (simulated time)
    if gust is not None:
        wait(5.0 + args.settle - args.gust_lead)
        t_wall = time.time()
        while not gust.has_connections() and time.time() - t_wall < 3:
            time.sleep(0.01)
        gust.publish(Empty())
        print(f'wind started (subscriber connected: {gust.has_connections()}), vehicle time {v.boot_ms} ms', flush=True)
        wait(args.gust_lead)
    else:
        wait(5.0 + args.settle)
    print(f'hover: z = {v.z:.3f} m, vz = {v.vz:.3f} m/s', flush=True)
    if not v.armed:
        sys.exit('vehicle disarmed during takeoff')

    v.set_param('IJC_CMD', 1)
    print('scenario started', flush=True)
    wait(args.duration)
    print(f'scenario done: z = {v.z:.3f} m', flush=True)
    if live is not None:
        print(live.summary(), flush=True)
        live.period = 2.0   # slower during the landing

    v.set_param('IJC_CMD', 2)
    print('landing', flush=True)
    if live is not None:   # ijc_control touchdown state for 5 s is enough (PX4's land detector can be slow)
        t_td = [None]

        def landed(_):
            if v.dbg is not None and int(v.dbg[10]) == 4:
                t_td[0] = t_td[0] or time.time()
                return time.time() - t_td[0] > 5.0
            return not v.armed
        wait(60 + 5 * sp, lambda x: not v.armed or landed(x))
    else:
        wait(60 + 5 * sp, lambda x: not v.armed)
    if v.armed:
        print('not disarmed automatically: forcing disarm', flush=True)
        v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
        v.spin(3)
    print(f'disarmed: {not v.armed}', flush=True)

    time.sleep(2.0)  # let the logger close the file
    log = newest_log(t_start - 5)
    if log is None:
        sys.exit('no ULog found')
    dst = os.path.join(args.outdir, f'{args.name}.ulg')
    shutil.copy2(log, dst)
    meta = dict(name=args.name, params=params, settle=args.settle, duration=args.duration,
                source_log=os.path.relpath(log, PX4), statustexts=v.statustexts)
    with open(os.path.join(args.outdir, f'{args.name}.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    print(f'ULog: {dst}', flush=True)


if __name__ == '__main__':
    main()
