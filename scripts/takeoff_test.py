"""Smoke test: arm, takeoff, check altitude, land via MAVLink (PX4 SITL)."""
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('udpin:127.0.0.1:14540', source_system=255)
m.wait_heartbeat(timeout=30)
print(f"heartbeat from sys {m.target_system} comp {m.target_component}")

last_hb = 0.0
def gcs_hb():
    global last_hb
    if time.time() - last_hb > 0.5:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_hb = time.time()

def cmd(c, *p):
    p = list(p) + [0] * (7 - len(p))
    m.mav.command_long_send(m.target_system, m.target_component, c, 0, *p)

def spin(seconds, cond=None):
    t0 = time.time()
    while time.time() - t0 < seconds:
        gcs_hb()
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'STATUSTEXT':
            print('  [px4]', msg.text)
        if cond and cond(msg):
            return msg
    return None

# wait until the EKF has a position and pre-arm checks pass
ok = spin(60, lambda x: x.get_type() == 'SYS_STATUS' and
          x.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
print('prearm checks ok' if ok else 'WARNING: prearm flag not seen, trying anyway')

target = 5.0
cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float('nan'), float('nan'), float('nan'), float('nan'))
spin(1)
cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
ack = spin(5, lambda x: x.get_type() == 'COMMAND_ACK' and x.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
print('arm ack:', ack.result if ack else None)

max_alt = 0.0
t0 = time.time()
while time.time() - t0 < 25:
    msg = spin(1, lambda x: x.get_type() == 'LOCAL_POSITION_NED')
    if msg:
        max_alt = max(max_alt, -msg.z)
        print(f"t={time.time()-t0:5.1f}s  alt={-msg.z:5.2f} m  vz={msg.vz:5.2f}")
print(f"max altitude: {max_alt:.2f} m")

cmd(mavutil.mavlink.MAV_CMD_NAV_LAND)
t0 = time.time()
landed = None
while time.time() - t0 < 40:
    msg = spin(1, lambda x: x.get_type() == 'EXTENDED_SYS_STATE')
    if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
        landed = time.time() - t0
        break
hb = spin(3, lambda x: x.get_type() == 'HEARTBEAT' and x.get_srcSystem() == m.target_system)
armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None
print(f"landed after {landed} s, armed={armed}")
print('RESULT:', 'PASS' if max_alt > 2.0 and landed else 'FAIL')
