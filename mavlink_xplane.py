#!/usr/bin/env python3
"""
mavlink_xplane.py — ArduPilot fmuv3-hil / SITL ↔ X-Plane bridge.

Mirrors SIM_XPlane.cpp behaviour completely via MAVLink:

  X-Plane DATA@ rows → HIL_SENSOR + GPS_INPUT + engine DREFs
  SERVO_OUTPUT_RAW   → X-Plane yoke DREFs (elevon de-mix) + throttle
  HEARTBEAT arm bit  → engine start / stop DREFs
  RC_CHANNELS_OVERRIDE from joystick (elevon mixed, CH1-5)

Data injected into the flight controller
─────────────────────────────────────────
  HIL_SENSOR              gyro, accel, baro, mag   — on every X-Plane packet
  GPS_INPUT               lat, lon, alt, vel, yaw  — at --gps-rate Hz
  RC_CHANNELS_OVERRIDE    joystick → CH1-5         — on change (--joystick)

Data received from the flight controller
─────────────────────────────────────────
  SERVO_OUTPUT_RAW → X-Plane yoke DREFs (elevon de-mix) + throttle DREF

X-Plane wire protocol
─────────────────────
  DSEL / USEL / DREF / RREF / DATA@ — same as SIM_XPlane.cpp

Engine management
─────────────────
  On arm:   battery on, mixture full rich, ignition start (4) → magnetos (3)
  On disarm: ignition off, mixture cutoff, battery off

Setup
─────
  HITL: Flash ArduPlane built for fmuv3-hil.  Parameters set via defaults.parm.
  SITL: python3 Tools/autotest/sim_vehicle.py --vehicle Plane --location WICC

Usage:
    python3 mavlink_xplane.py --pixhawk /dev/tty.usbmodem1101
    python3 mavlink_xplane.py --pixhawk tcp:127.0.0.1:5760
    python3 mavlink_xplane.py --joystick --joy-index 0 --debug
"""

import argparse
import math
import select
import socket
import struct
import time

from pymavlink import mavutil

# Optional joystick support via pygame
try:
    import pygame
    _PYGAME = True
except ImportError:
    _PYGAME = False

# ── constants ────────────────────────────────────────────────────────────────
KNOTS_TO_MS  = 0.514444
FEET_TO_M    = 0.3048
GRAVITY_MSS  = 9.80665
DEG_TO_RAD   = math.pi / 180.0

# Standard atmosphere — for baro synthesis from altitude
SEA_PRESSURE_HPA = 1013.25
LAPSE_RATE       = 0.0065   # K/m
T0_K             = 288.15   # sea-level temperature K

UINT16_MAX = 65535  # MAVLink "no override" sentinel for RC_CHANNELS_OVERRIDE


def alt_to_pressure(alt_m: float) -> float:
    """Return absolute pressure (hPa) from altitude (m) via ISA model."""
    return SEA_PRESSURE_HPA * ((1.0 - LAPSE_RATE * alt_m / T0_K) ** 5.2561)


def airspeed_to_diff_pressure(airspeed_ms: float) -> float:
    """Return differential pressure (hPa) from airspeed (m/s)."""
    rho = 1.225  # kg/m³ at sea level
    return 0.5 * rho * airspeed_ms * airspeed_ms / 100.0   # Pa → hPa


# ── HIL_SENSOR fields_updated bitmask (MAVLink spec) ────────────────────────
HIL_SENSOR_ACCEL = 0x007
HIL_SENSOR_GYRO  = 0x038
HIL_SENSOR_MAG   = 0x1C0
HIL_SENSOR_BARO  = 0xE00
HIL_SENSOR_TEMP  = 0x1000
HIL_FIELDS = (HIL_SENSOR_ACCEL | HIL_SENSOR_GYRO | HIL_SENSOR_MAG |
              HIL_SENSOR_BARO  | HIL_SENSOR_TEMP)

# ── GPS time ─────────────────────────────────────────────────────────────────
_GPS_EPOCH_UNIX  = 315964800
_GPS_LEAP_SECONDS = 18

def gps_time() -> tuple:
    """Return (time_week, time_week_ms) from current UTC system time."""
    t = time.time() + _GPS_LEAP_SECONDS - _GPS_EPOCH_UNIX
    week   = int(t / 604800)
    tow_ms = int((t % 604800) * 1000)
    return week, tow_ms


# ── Magnetic field helper ────────────────────────────────────────────────────
# Horizontal field intensity used when building the NED mag vector from
# X-Plane's magnetic heading.
_MAG_H_GAUSS = 0.22

def ned_to_body(vec_ned: list, roll_r: float, pitch_r: float, yaw_r: float) -> list:
    """Rotate a NED vector to body frame using ZYX Euler angles (rad)."""
    sr, cr = math.sin(roll_r),  math.cos(roll_r)
    sp, cp = math.sin(pitch_r), math.cos(pitch_r)
    sy, cy = math.sin(yaw_r),   math.cos(yaw_r)
    n, e, d = vec_ned
    bx = cp * cy * n + cp * sy * e - sp * d
    by = (sr * sp * cy - cr * sy) * n + (sr * sp * sy + cr * cy) * e + sr * cp * d
    bz = (cr * sp * cy + sr * sy) * n + (cr * sp * sy - sr * cy) * e + cr * cp * d
    return [bx, by, bz]


# ── X-Plane DATA@ row codes — mirrors SIM_XPlane.cpp enum ───────────────────
ROW_TIMES          = 1
ROW_SPEED          = 3
ROW_GLOAD          = 4
ROW_TRIM           = 13
ROW_ANG_VEL        = 16
ROW_PITCH_ROLL_HDG = 17
ROW_LAT_LON_ALT    = 20
ROW_LOC_VEL_DIST   = 21
ROW_ENGINE_RPM     = 37
ROW_PROP_RPM       = 38
ROW_PROP_PITCH     = 39

# All rows we subscribe to — matches required_data[] in SIM_XPlane.cpp.
# Sent via DSEL; we track seen_mask and only re-request unseen rows.
REQUIRED_ROWS = [
    ROW_TIMES, ROW_LAT_LON_ALT, ROW_SPEED, ROW_PITCH_ROLL_HDG,
    ROW_LOC_VEL_DIST, ROW_ANG_VEL, ROW_GLOAD,
    ROW_TRIM,
    ROW_PROP_PITCH, ROW_ENGINE_RPM, ROW_PROP_RPM,
]

# RREF codes
RREF_VERSION = 1
RREF_MAG_PSI = 2   # sim/flightmodel/position/mag_psi — magnetic heading (deg)


# ── X-Plane UDP wire protocol ────────────────────────────────────────────────

def xp_dsel(sock, addr, rows):
    """Subscribe to DATA@ rows (DSEL packet)."""
    padded = (list(rows) + [0] * 8)[:8]
    sock.sendto(b'DSEL\x00' + struct.pack('<8I', *padded), addr)


def xp_usel(sock, addr, code: int):
    """Unsubscribe from a DATA@ row (USEL packet)."""
    buf = struct.pack('<8I', code, 0, 0, 0, 0, 0, 0, 0)
    sock.sendto(b'USEL\x00' + buf, addr)


def xp_dref(sock, addr, name: str, value: float):
    """Send a DREF packet to X-Plane to set a dataref."""
    name_b = name.encode() + b'\x00' * (500 - len(name))
    sock.sendto(b'DREF\x00' + struct.pack('<f', value) + name_b, addr)


def xp_rref(sock, addr, name: str, code: int, rate_hz: int):
    """Subscribe to an RREF dataref at rate_hz (0 = cancel)."""
    name_b = name.encode() + b'\x00' * (400 - len(name))
    sock.sendto(b'RREF\x00' + struct.pack('<II', rate_hz, code) + name_b, addr)


def xp_parse(pkt: bytes) -> dict:
    """Parse DATA@ packet → {row_code: (v0…v7)}.

    values[N] corresponds to C++ data[N+1] (row code is at data[0]).
    """
    if len(pkt) < 5 or pkt[:4] != b'DATA':
        return {}
    rows = {}
    off = 5
    while off + 36 <= len(pkt):
        code   = struct.unpack_from('<I', pkt, off)[0]
        values = struct.unpack_from('<8f', pkt, off + 4)
        rows[code] = values
        off += 36
    return rows


# ── DREF / servo map ──────────────────────────────────────────────────────────
def _range(pwm, r=1.0): return r * (pwm - 1000) / 1000.0

# Yoke DREFs for elevon de-mixed output
YOKE_ROLL_DREF  = 'sim/joystick/yoke_roll_ratio'
YOKE_PITCH_DREF = 'sim/joystick/yoke_pitch_ratio'

# Additional servo DREFs beyond the elevon channels.
# (channel index is 0-based into the servos[] list from SERVO_OUTPUT_RAW)
# CH2 = servo3 = throttle, CH4 = servo5 = flap
SERVO_DREFS = [
    ('sim/flightmodel/engine/ENGN_thro_use[0]', 2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[1]', 2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[2]', 2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[3]', 2, _range),
    ('sim/cockpit2/controls/flap_ratio',         4, _range),
]

# Number of engines to manage on arm/disarm (matches ENGN_thro_use[0..3])
NUM_ENGINES = 4

# Engine ignition key values (X-Plane)
IGNITION_OFF   = 0.0   # engine off
IGNITION_START = 4.0   # starter cranking
IGNITION_MAGS  = 3.0   # both magnetos running


# ── engine state management — mirrors XPlane::handle_engine_state() ─────────

def engine_arm(sock, addr):
    """Send arm DREFs: battery on, mixture full rich, ignition start."""
    for i in range(NUM_ENGINES):
        xp_dref(sock, addr, f'sim/cockpit2/electrical/battery_on[{i}]', 1.0)
        xp_dref(sock, addr,
                f'sim/cockpit2/engine/actuators/mixture_ratio[{i}]', 1.0)


def engine_disarm(sock, addr):
    """Send disarm DREFs: ignition off, mixture cutoff, battery off."""
    for i in range(NUM_ENGINES):
        xp_dref(sock, addr,
                f'sim/cockpit2/engine/actuators/ignition_key[{i}]',
                IGNITION_OFF)
        xp_dref(sock, addr,
                f'sim/cockpit2/engine/actuators/mixture_ratio[{i}]', 0.0)
        xp_dref(sock, addr, f'sim/cockpit2/electrical/battery_on[{i}]', 0.0)


def engine_set_ignition(sock, addr, value: float):
    """Set ignition key for all engines."""
    for i in range(NUM_ENGINES):
        xp_dref(sock, addr,
                f'sim/cockpit2/engine/actuators/ignition_key[{i}]', value)


# ── joystick helpers ─────────────────────────────────────────────────────────

def _axis_pwm(v: float, invert: bool = False) -> int:
    """Map joystick axis [-1, 1] → PWM [1000, 2000], centre 1500."""
    if invert:
        v = -v
    return int(max(1000, min(2000, 1500 + v * 500)))


def _thr_pwm(v: float, invert: bool = False) -> int:
    """Map throttle axis [-1=min, +1=max] → PWM [1000, 2000]."""
    if invert:
        v = -v
    return int(max(1000, min(2000, 1000 + (v + 1.0) * 500)))


# ArduPilot CH5 flight-mode band centres (µs).
_FLTMODE_CENTRES = [1165, 1295, 1425, 1555, 1685, 1815]

def _fltmode_pwm(v: float) -> int:
    """Map axis [-1, 1] → nearest ArduPilot flight-mode band centre (µs)."""
    raw = int(max(1000, min(2000, 1500 + v * 500)))
    return min(_FLTMODE_CENTRES, key=lambda c: abs(c - raw))


# ── dynamic DSEL helpers ─────────────────────────────────────────────────────

def select_data(sock, addr, seen_mask: int, last_dsel_time: float,
                now: float) -> float:
    """Re-subscribe to any rows not yet seen.  Throttled to 1 Hz.
    Returns updated last_dsel_time."""
    all_mask = (1 << len(REQUIRED_ROWS)) - 1
    if (seen_mask & all_mask) == all_mask:
        return last_dsel_time   # got everything
    if now - last_dsel_time < 1.0:
        return last_dsel_time   # throttle to 1 Hz
    missing = [REQUIRED_ROWS[i] for i in range(len(REQUIRED_ROWS))
               if not (seen_mask & (1 << i))]
    # DSEL takes up to 8 rows per packet
    for start in range(0, len(missing), 8):
        xp_dsel(sock, addr, missing[start:start + 8])
    return now


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='fmuv3-hil ↔ X-Plane bridge (complete SIM_XPlane replacement)'
    )
    ap.add_argument('--pixhawk',     default='udpin:0.0.0.0:14560',
                    help='MAVLink connection  (default: udpin:0.0.0.0:14560)')
    ap.add_argument('--xplane-host', default='127.0.0.1')
    ap.add_argument('--xplane-port', type=int, default=49000)
    ap.add_argument('--bind-port',   type=int, default=49005,
                    help='Local port for X-Plane DATA@ (default: 49005)')
    ap.add_argument('--gps-rate',    type=int, default=5,
                    help='GPS_INPUT rate Hz (default: 5)')
    ap.add_argument('--hil-rate',    type=int, default=100,
                    help='Maximum HIL_SENSOR rate Hz (default: 100)')
    # ── joystick ──────────────────────────────────────────────────────────────
    ap.add_argument('--joystick',       action='store_true',
                    help='Enable joystick → RC_CHANNELS_OVERRIDE (requires pygame)')
    ap.add_argument('--joy-index',      type=int, default=0)
    ap.add_argument('--joy-roll-axis',  type=int, default=0)
    ap.add_argument('--joy-pitch-axis', type=int, default=1,
                    help='Pitch axis (auto-inverted)')
    ap.add_argument('--joy-thr-axis',   type=int, default=2)
    ap.add_argument('--joy-yaw-axis',   type=int, default=3)
    ap.add_argument('--joy-fltmode-axis', type=int, default=4,
                    help='CH5 / mode switch axis (-1 = disabled)')
    ap.add_argument('--joy-thr-invert', action='store_true',
                    help='Invert throttle axis')
    ap.add_argument('--joy-rate',       type=int, default=50,
                    help='RC_CHANNELS_OVERRIDE rate Hz (default: 50)')
    ap.add_argument('--debug',          action='store_true')
    args = ap.parse_args()

    xp_addr = (args.xplane_host, args.xplane_port)

    # ── joystick init ─────────────────────────────────────────────────────────
    joystick = None
    if args.joystick:
        if not _PYGAME:
            print('[JOY] pygame not installed — joystick disabled. '
                  'Run: pip install pygame')
        else:
            pygame.init()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            if count == 0:
                print('[JOY] No joystick detected')
            elif args.joy_index >= count:
                print(f'[JOY] Index {args.joy_index} out of range '
                      f'({count} device(s) found)')
            else:
                joystick = pygame.joystick.Joystick(args.joy_index)
                joystick.init()
                print(f'[JOY] {joystick.get_name()}  '
                      f'axes={joystick.get_numaxes()}  '
                      f'buttons={joystick.get_numbuttons()}')
                print(f'[JOY] roll=axis{args.joy_roll_axis}  '
                      f'pitch=axis{args.joy_pitch_axis}(inv)  '
                      f'thr=axis{args.joy_thr_axis}  '
                      f'yaw=axis{args.joy_yaw_axis}')

    # ── MAVLink ──────────────────────────────────────────────────────────────
    print(f'[MAV] Connecting to {args.pixhawk} …')
    mav = mavutil.mavlink_connection(args.pixhawk, source_system=255)
    mav.wait_heartbeat()
    print(f'[MAV] Heartbeat  sysid={mav.target_system}  '
          f'compid={mav.target_component}')

    # ── Enable HIL mode ──────────────────────────────────────────────────────
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED |
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
    )
    print('[MAV] HIL mode enabled')

    # Request SERVO_OUTPUT_RAW at 50 Hz
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        20000,   # interval µs → 50 Hz
        0, 0, 0, 0, 0,
    )

    # ── X-Plane socket ───────────────────────────────────────────────────────
    xp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    xp_sock.bind(('0.0.0.0', args.bind_port))
    xp_sock.setblocking(False)
    print(f'[XP]  Listening on :{args.bind_port}  →  {xp_addr}')

    # Initial full DSEL
    xp_dsel(xp_sock, xp_addr, REQUIRED_ROWS)

    # RREF subscriptions
    xp_rref(xp_sock, xp_addr, 'sim/version/xplane_internal_version', RREF_VERSION, 1)
    xp_rref(xp_sock, xp_addr, 'sim/flightmodel/position/mag_psi',    RREF_MAG_PSI, 10)

    # Override joystick and throttles so X-Plane responds to our DREFs
    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_joystick',  1.0)
    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_throttles', 1.0)

    # Parking brake on until armed
    xp_dref(xp_sock, xp_addr, 'sim/flightmodel/controls/parkbrake', 1.0)
    print('[XP]  Parking brake SET')
    print(f'[XP]  DSEL rows {REQUIRED_ROWS}')

    # ── state ────────────────────────────────────────────────────────────────
    xp_time     = 0.0
    lat         = 0.0
    lon         = 0.0
    alt_m       = 0.0
    agl_m       = 0.0
    ground_elev = 0.0
    roll_r      = 0.0
    pitch_r     = 0.0
    yaw_r       = 0.0
    gyro        = [0.0, 0.0, 0.0]
    accel_body  = [0.0, 0.0, -GRAVITY_MSS]
    vel_ned     = [0.0, 0.0, 0.0]
    airspeed    = 0.0       # equivalent airspeed m/s (matches C++ data[2])
    engine_rpm  = [0.0] * 4
    prop_rpm    = [0.0] * 4

    xp_version    = 0
    mag_psi_r     = 0.0
    mag_psi_valid = False
    pos_valid     = False
    att_valid     = False
    home_set      = False
    home_set_time = 0.0
    xp_sensor_updated = False

    # seen_mask: bit i set when REQUIRED_ROWS[i] has been received
    seen_mask = 0

    # Engine state — mirrors XPlane::handle_engine_state()
    armed            = False     # current arm state from HEARTBEAT
    engine_cranking  = False     # True during the 2 s starter crank window
    arm_time         = 0.0       # monotonic time when arm transition happened

    gps_ivl      = 1.0 / args.gps_rate
    hil_min_ivl  = 1.0 / args.hil_rate
    joy_ivl      = 1.0 / args.joy_rate
    last_gps     = 0.0
    last_hil     = 0.0
    prev_joy     = [None, None, None, None, None]
    last_dsel    = time.monotonic()
    last_debug   = time.monotonic()
    last_srv     = time.monotonic()
    srv_count    = 0
    servos       = [0] * 8
    dref_sent    = False

    # X-Plane connected flag (set when first DATA@ arrives from X-Plane)
    xp_connected = False

    print('\nRunning — Ctrl+C to stop\n')

    while True:
        now = time.monotonic()

        # ── dynamic DSEL — re-request rows not yet seen (1 Hz throttle) ──────
        last_dsel = select_data(xp_sock, xp_addr, seen_mask, last_dsel, now)

        # Refresh override flags every 5 s (X-Plane resets them on scene reload)
        if now - last_dsel >= 5.0:
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_joystick',  1.0)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_throttles', 1.0)

        # ── receive X-Plane DATA@ ────────────────────────────────────────────
        timeout = max(0.0, min(
            (last_gps + gps_ivl) - now,
            hil_min_ivl,
            joy_ivl if joystick else hil_min_ivl,
        ))
        readable, _, _ = select.select([xp_sock], [], [], timeout)

        xp_sensor_updated = False
        try:
            while readable:
                pkt, xp_src = xp_sock.recvfrom(65536)

                # ── RREF replies ─────────────────────────────────────────────
                if pkt[:4] == b'RREF' and len(pkt) >= 13:
                    code, val = struct.unpack_from('<If', pkt, 5)
                    if code == RREF_VERSION and xp_version == 0:
                        xp_version = int(val)
                        xp_connected = True
                        print(f'[XP]  X-Plane {xp_version // 10000} '
                              f'(build {xp_version})')
                    elif code == RREF_MAG_PSI:
                        mag_psi_r     = math.radians(val % 360.0)
                        mag_psi_valid = True
                    continue

                rows = xp_parse(pkt)
                if not rows:
                    continue

                if not xp_connected:
                    xp_connected = True
                    print(f'[XP]  Connected — first DATA@ from {xp_src}')

                is_xp12 = (xp_version // 10000) >= 12

                for code, values in rows.items():
                    # Track seen_mask for dynamic DSEL
                    if code in REQUIRED_ROWS:
                        seen_mask |= (1 << REQUIRED_ROWS.index(code))
                    else:
                        # Deselect rows we did not ask for (mirrors deselect_code())
                        xp_usel(xp_sock, xp_addr, code)
                        continue

                    # ── Row 1: Times ─────────────────────────────────────────
                    # values[2] = data[3] = elapsed sim time (s) — matches C++
                    if code == ROW_TIMES:
                        xp_time = values[2]

                    # ── Row 3: Speed ─────────────────────────────────────────
                    # values[1] = data[2] = equivalent airspeed (kts)
                    # mirrors SIM_XPlane.cpp: airspeed = data[2] * kts_to_ms
                    elif code == ROW_SPEED:
                        airspeed = values[1] * KNOTS_TO_MS

                    # ── Row 4: Gload → body accel ────────────────────────────
                    # accel_body.z = -data[5]·g  values[4]
                    # accel_body.x =  data[6]·g  values[5]
                    # accel_body.y =  data[7]·g  values[6]
                    elif code == ROW_GLOAD:
                        accel_body = [
                             values[5] * GRAVITY_MSS,
                             values[6] * GRAVITY_MSS,
                            -values[4] * GRAVITY_MSS,
                        ]
                        xp_sensor_updated = True

                    # ── Row 13: Trim — parsed, not currently forwarded ────────
                    elif code == ROW_TRIM:
                        pass   # trim state available if needed

                    # ── Row 16: AngularVelocities ────────────────────────────
                    # XP12: deg/s  values[0..2] = data[1..3]
                    # XP11: rad/s  axes swapped: gyro.x=data[2], gyro.y=data[1]
                    elif code == ROW_ANG_VEL:
                        if is_xp12:
                            gyro = [values[0] * DEG_TO_RAD,
                                    values[1] * DEG_TO_RAD,
                                    values[2] * DEG_TO_RAD]
                        else:
                            gyro = [values[1], values[0], values[2]]
                        xp_sensor_updated = True

                    # ── Row 17: PitchRollHeading ─────────────────────────────
                    # values[0]=pitch°  values[1]=roll°  values[2]=true hdg°
                    elif code == ROW_PITCH_ROLL_HDG:
                        pitch_r = values[0] * DEG_TO_RAD
                        roll_r  = values[1] * DEG_TO_RAD
                        yaw_r   = values[2] * DEG_TO_RAD
                        if not att_valid:
                            att_valid = True
                            print(f'[ATT] First attitude  '
                                  f'hdg={math.degrees(yaw_r):.1f}°')
                        xp_sensor_updated = True

                    # ── Row 20: LatLonAlt ────────────────────────────────────
                    # values[0]=lat°  values[1]=lon°  values[2]=alt_ft_MSL
                    # values[3]=alt_ft_AGL
                    elif code == ROW_LAT_LON_ALT:
                        lat         = values[0]
                        lon         = values[1]
                        alt_m       = values[2] * FEET_TO_M
                        agl_m       = values[3] * FEET_TO_M
                        ground_elev = alt_m - agl_m
                        pos_valid   = True

                    # ── Row 21: LocVelDistTraveled ───────────────────────────
                    # velocity_ef: y=data[4] E, z=−data[5] D, x=−data[6] N
                    # values indices: E=values[3], D=values[4], N=values[5]
                    elif code == ROW_LOC_VEL_DIST:
                        vel_ned = [-values[5], values[3], -values[4]]

                    # ── Row 37: EngineRPM ─────────────────────────────────────
                    elif code == ROW_ENGINE_RPM:
                        for i in range(min(NUM_ENGINES, 4)):
                            engine_rpm[i] = values[i]

                    # ── Row 38: PropRPM ───────────────────────────────────────
                    elif code == ROW_PROP_RPM:
                        for i in range(min(NUM_ENGINES, 4)):
                            prop_rpm[i] = values[i]

                    # ── Row 39: PropPitch — received, not forwarded ──────────
                    elif code == ROW_PROP_PITCH:
                        pass

        except BlockingIOError:
            pass

        # ── RC_CHANNELS_OVERRIDE (joystick) ───────────────────────────────────
        if joystick:
            pygame.event.pump()

            # Elevon mixing: left = elev + ail, right = elev - ail
            ail  =  joystick.get_axis(args.joy_roll_axis)
            elev = -joystick.get_axis(args.joy_pitch_axis)
            ch1 = int(max(1000, min(2000, 1500 + (elev + ail) * 500)))
            ch2 = int(max(1000, min(2000, 1500 + (elev - ail) * 500)))
            ch3 = _thr_pwm( joystick.get_axis(args.joy_thr_axis),
                            invert=args.joy_thr_invert)
            ch4 = _axis_pwm(joystick.get_axis(args.joy_yaw_axis))
            ch5 = (_fltmode_pwm(joystick.get_axis(args.joy_fltmode_axis))
                   if args.joy_fltmode_axis >= 0 else UINT16_MAX)

            cur = [ch1, ch2, ch3, ch4, ch5]
            if cur != prev_joy:
                if args.debug:
                    dm_ail  = (ch1 - ch2) / 1000.0
                    dm_elev = (ch1 + ch2 - 3000) / 1000.0
                    print(f'[JOY] elv_L={ch1} elv_R={ch2} thr={ch3} '
                          f'rud={ch4} mode={ch5}'
                          f'  (ail={dm_ail:+.2f} elev={dm_elev:+.2f})')
                prev_joy = cur
                mav.mav.rc_channels_override_send(
                    mav.target_system, mav.target_component,
                    ch1, ch2, ch3, ch4,
                    ch5, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                )

        if not pos_valid:
            # Drain MAVLink (need HEARTBEAT for arm state even before GPS)
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
            if args.debug and now - last_debug >= 5.0:
                last_debug = now
                print('[wait] No X-Plane DATA@ — is X-Plane running and unpaused?')
            continue

        t_us = int(now * 1e6)

        # ── Set home ──────────────────────────────────────────────────────────
        if not home_set and home_set_time == 0.0:
            home_set_time = now + 2.0
        if not home_set and now >= home_set_time:
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0, 0, 0, 0, 0,
                lat, lon, ground_elev,
            )
            home_set = True
            print(f'[MAV] Set home  lat={lat:.6f}  lon={lon:.6f}  '
                  f'terrain={ground_elev:.1f} m  AGL={agl_m:.1f} m')

        # ── HIL_SENSOR ────────────────────────────────────────────────────────
        if xp_sensor_updated and (now - last_hil) >= hil_min_ivl:
            last_hil = now

            abs_p  = alt_to_pressure(alt_m)
            diff_p = airspeed_to_diff_pressure(airspeed)
            temp_c = 15.0 - 0.0065 * alt_m

            # Magnetometer: NED field from X-Plane mag_psi RREF.
            # declination = true_heading − mag_heading
            # dip angle   = arctan(2·sin(lat) / cos(lat))  — dipole model
            decl_r   = yaw_r - (mag_psi_r if mag_psi_valid else yaw_r)
            lat_r    = math.radians(lat)
            dip_r    = math.atan2(2.0 * math.sin(lat_r), math.cos(lat_r))
            D        = _MAG_H_GAUSS * math.tan(dip_r)
            mag_ned  = [_MAG_H_GAUSS * math.cos(decl_r),
                        _MAG_H_GAUSS * math.sin(decl_r),
                        D]
            mag_body = ned_to_body(mag_ned, roll_r, pitch_r, yaw_r)

            mav.mav.hil_sensor_send(
                t_us,
                accel_body[0], accel_body[1], accel_body[2],
                gyro[0],       gyro[1],       gyro[2],
                mag_body[0],   mag_body[1],   mag_body[2],
                abs_p,
                diff_p,
                agl_m,          # pressure_alt = AGL — baro reads 0 on ground
                temp_c,
                HIL_FIELDS,
                0,
            )

        # ── GPS_INPUT ─────────────────────────────────────────────────────────
        if now - last_gps >= gps_ivl:
            last_gps = now

            if att_valid:
                raw    = int((yaw_r % (2 * math.pi)) * 18000 / math.pi)
                hdg_cd = raw if raw != 0 else 36000
            else:
                hdg_cd = 0

            gps_week, gps_tow_ms = gps_time()
            mav.mav.gps_input_send(
                t_us,
                0,                       # gps_id
                0,                       # ignore_flags — use all fields
                gps_tow_ms, gps_week,
                3,                       # fix_type: 3D
                int(lat * 1e7),
                int(lon * 1e7),
                alt_m,
                1.0, 1.0,
                vel_ned[0], vel_ned[1], vel_ned[2],
                0.2,                     # speed_accuracy m/s
                0.3,                     # horiz_accuracy m
                0.5,                     # vert_accuracy m
                10,                      # satellites_visible
                hdg_cd,                  # yaw cdeg (0=unknown)
            )

        # ── SERVO_OUTPUT_RAW → X-Plane DREFs ─────────────────────────────────
        # ── MAVLink receive loop (SERVO_OUTPUT_RAW + HEARTBEAT) ──────────────
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break

            # ── HEARTBEAT: track arm state for engine management ──────────────
            if msg.get_type() == 'HEARTBEAT':
                mav_armed = bool(msg.base_mode &
                                 mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if mav_armed != armed:
                    armed = mav_armed
                    if armed:
                        print('[ENG] Armed — starting engines')
                        engine_arm(xp_sock, xp_addr)
                        engine_set_ignition(xp_sock, xp_addr, IGNITION_START)
                        arm_time        = now
                        engine_cranking = True
                        xp_dref(xp_sock, xp_addr,
                                'sim/flightmodel/controls/parkbrake', 0.0)
                    else:
                        print('[ENG] Disarmed — stopping engines')
                        engine_disarm(xp_sock, xp_addr)
                        engine_cranking = False
                        xp_dref(xp_sock, xp_addr,
                                'sim/flightmodel/controls/parkbrake', 1.0)
                continue

            if msg.get_type() != 'SERVO_OUTPUT_RAW':
                continue

            servos = [
                msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                msg.servo4_raw, msg.servo5_raw,
                msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
            ]
            srv_count += 1
            if args.debug:
                labels = ['elv_L', 'elv_R', 'thr', 'rud', 'ch5',
                          'ch6', 'ch7', 'ch8']
                parts  = [f'{l}={v}' for l, v in zip(labels, servos) if v > 0]
                print(f'[SRV] {" ".join(parts)}')

            if not dref_sent:
                dref_sent = True
                print(f'[XP]  First DREF → {xp_addr}')

            # Elevon de-mix: inverse of (L = elev+ail, R = elev-ail)
            # ail  = (L - R) / 1000   elev = (L + R - 3000) / 1000
            ail  = max(-1.0, min(1.0, (servos[0] - servos[1]) / 1000.0))
            elev = max(-1.0, min(1.0, (servos[0] + servos[1] - 3000) / 1000.0))
            xp_dref(xp_sock, xp_addr, YOKE_ROLL_DREF,  ail)
            xp_dref(xp_sock, xp_addr, YOKE_PITCH_DREF, elev)
            if args.debug:
                print(f'  [demix] ail={ail:+.3f}  elev={elev:+.3f}')

            for dref_name, ch, conv in SERVO_DREFS:
                val = conv(servos[ch])
                xp_dref(xp_sock, xp_addr, dref_name, val)
                if args.debug:
                    print(f'  → {dref_name} = {val:.3f}')

        # ── Engine ignition transition: crank 2 s then switch to magnetos ─────
        # Mirrors XPlane::handle_engine_state() armed branch.
        if armed and engine_cranking and (now - arm_time) >= 2.0:
            engine_cranking = False
            engine_set_ignition(xp_sock, xp_addr, IGNITION_MAGS)
            print('[ENG] Engine started — magnetos on')

        # ── periodic 1 s status ────────────────────────────────────────────────
        if now - last_srv >= 1.0:
            last_srv = now
            hdg = math.degrees(yaw_r) % 360.0
            print(f'[GPS] lat={lat:.6f}  lon={lon:.6f}  alt={alt_m:.1f} m  '
                  f'agl={agl_m:.1f} m  hdg={hdg:.1f}°  IAS={airspeed:.1f} m/s')
            if not args.debug:
                s_ail  = (servos[0] - servos[1]) / 1000.0
                s_elev = (servos[0] + servos[1] - 3000) / 1000.0
                thr    = _range(servos[2])
                rpm0   = engine_rpm[0]
                print(f'[SRV] ail={s_ail:+.3f}  elev={s_elev:+.3f}  '
                      f'thr={thr:.2f}  rpm={rpm0:.0f}'
                      f'  ({srv_count} srv/s)'
                      f'  armed={armed}')
            srv_count = 0

        # ── debug ─────────────────────────────────────────────────────────────
        if args.debug and now - last_debug >= 5.0:
            last_debug = now
            hdg = math.degrees(yaw_r) % 360
            seen_rows = [REQUIRED_ROWS[i] for i in range(len(REQUIRED_ROWS))
                         if seen_mask & (1 << i)]
            print(f'[xp]   lat={lat:.5f}  lon={lon:.5f}  '
                  f'alt={alt_m:.1f} m  agl={agl_m:.1f} m  '
                  f'terrain={ground_elev:.1f} m  t={xp_time:.1f} s')
            print(f'[att]  pitch={math.degrees(pitch_r):+.1f}°  '
                  f'roll={math.degrees(roll_r):+.1f}°  hdg={hdg:.1f}°  '
                  f'att_valid={att_valid}')
            print(f'[vel]  N={vel_ned[0]:.1f}  E={vel_ned[1]:.1f}  '
                  f'D={vel_ned[2]:.1f} m/s  airspeed={airspeed:.1f} m/s')
            print(f'[gyro] {[round(g, 3) for g in gyro]} rad/s')
            print(f'[acc]  {[round(a, 2) for a in accel_body]} m/s²')
            p = alt_to_pressure(alt_m)
            print(f'[baro] abs={p:.2f} hPa  pressure_alt(AGL)={agl_m:.1f} m  '
                  f'diff={airspeed_to_diff_pressure(airspeed):.4f} hPa')
            print(f'[eng]  rpm={engine_rpm[:2]}  armed={armed}  '
                  f'cranking={engine_cranking}')
            print(f'[seen] rows={seen_rows}  mask=0x{seen_mask:03x}')
            print()


if __name__ == '__main__':
    main()
