#!/usr/bin/env python3
"""
mavlink_xplane.py — ArduPilot fmuv3-hil / SITL ↔ X-Plane bridge.

Predictor-corrector HIL loop
─────────────────────────────
  SITL's own aerodynamic model (SIM_Plane) integrates sitl->state
  (position, velocity, attitude) continuously at its own rate.

  When X-Plane DATA@ packets arrive, HIL_SENSOR overrides the raw sensor
  backends (gyro, accel, baro, mag) with X-Plane ground-truth readings.
  The EKF then fuses these corrected sensors and converges toward X-Plane
  truth, while SIM_Plane keeps sitl->state smooth between corrections.

  Between X-Plane packets: SIM_Plane holds aerodynamics, sensors hold
  their last injected values, EKF propagates.
  When X-Plane data arrives: sensor backends are corrected via HIL_SENSOR.

Data injected into the flight controller
─────────────────────────────────────────
  HIL_SENSOR              gyro, accel, baro, mag   — on every X-Plane packet
  GPS_INPUT               lat, lon, alt, vel, yaw  — at --gps-rate Hz
  RC_CHANNELS_OVERRIDE    joystick → CH1-5         — on change (--joystick)

Data received from the flight controller
─────────────────────────────────────────
  SERVO_OUTPUT_RAW        → X-Plane DREFs (control surfaces)

X-Plane wire protocol
─────────────────────
  DSEL / DREF / RREF / DATA@ rows — same as SIM_XPlane.cpp

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
    """Return differential pressure (hPa) from true airspeed (m/s)."""
    rho = 1.225  # kg/m³ at sea level
    return 0.5 * rho * airspeed_ms * airspeed_ms / 100.0   # Pa → hPa


# ── HIL_SENSOR fields_updated bitmask (MAVLink spec) ────────────────────────
HIL_SENSOR_ACCEL    = 0x007   # xacc | yacc | zacc
HIL_SENSOR_GYRO     = 0x038   # xgyro | ygyro | zgyro
HIL_SENSOR_MAG      = 0x1C0   # xmag | ymag | zmag
HIL_SENSOR_BARO     = 0xE00   # abs_pressure | diff_pressure | pressure_alt
HIL_SENSOR_TEMP     = 0x1000
HIL_FIELDS = (HIL_SENSOR_ACCEL | HIL_SENSOR_GYRO | HIL_SENSOR_MAG |
              HIL_SENSOR_BARO  | HIL_SENSOR_TEMP)

# ── GPS time ─────────────────────────────────────────────────────────────────
# GPS epoch: 6 Jan 1980. Leap seconds as of 2024: 18.
_GPS_EPOCH_UNIX = 315964800
_GPS_LEAP_SECONDS = 18

def gps_time() -> tuple:
    """Return (time_week, time_week_ms) from current UTC system time."""
    t = time.time() + _GPS_LEAP_SECONDS - _GPS_EPOCH_UNIX
    week    = int(t / 604800)
    tow_ms  = int((t % 604800) * 1000)
    return week, tow_ms


# ── Magnetic field helpers (mirrors SIM_Aircraft::update_mag_field_bf) ───────

def igrf_dipole_ned_gauss(lat_deg: float, lon_deg: float) -> list:
    """
    IGRF-14 (2025) degree-1 dipole approximation of Earth magnetic field in NED,
    returned in Gauss (HIL_SENSOR units).  Matches AP_Declination intensity to
    within ~20% globally; direction error <5° — sufficient for HIL compass.

    Mirrors the field model used by Aircraft::update_mag_field_bf() in
    SIM_Aircraft.cpp, without requiring the binary AP_Declination table.
    """
    # IGRF-14 epoch 2025.0 degree-1 coefficients, nT
    g10, g11, h11 = -29351.0, -1410.0, 4545.0
    lat   = math.radians(lat_deg)
    lon   = math.radians(lon_deg)
    colat = math.pi / 2.0 - lat          # geocentric colatitude ≈ geographic
    sc, cc = math.sin(colat), math.cos(colat)
    sl, cl = math.sin(lon),   math.cos(lon)
    gsl    = g11 * cl + h11 * sl         # g11·cosλ + h11·sinλ
    # NED components, nT → Gauss (* 1e-5)
    bn = -(g10 * sc - gsl * cc) * 1e-5
    be =  (g11 * sl - h11 * cl) * 1e-5
    bd = -2.0 * (g10 * cc + gsl * sc) * 1e-5
    return [bn, be, bd]



def ned_to_body(vec_ned: list, roll_r: float, pitch_r: float, yaw_r: float) -> list:
    """
    Rotate a NED vector to body frame using ZYX Euler angles (rad).
    Mirrors dcm.transposed() * mag_ef in SIM_Aircraft::update_mag_field_bf().
    """
    sr, cr = math.sin(roll_r),  math.cos(roll_r)
    sp, cp = math.sin(pitch_r), math.cos(pitch_r)
    sy, cy = math.sin(yaw_r),   math.cos(yaw_r)
    n, e, d = vec_ned
    bx = cp * cy * n + cp * sy * e - sp * d
    by = (sr * sp * cy - cr * sy) * n + (sr * sp * sy + cr * cy) * e + sr * cp * d
    bz = (cr * sp * cy + sr * sy) * n + (cr * sp * sy - sr * cy) * e + cr * cp * d
    return [bx, by, bz]

# ── X-Plane DATA@ row codes (SIM_XPlane.cpp enum) ───────────────────────────
ROW_TIMES          = 1
ROW_SPEED          = 3
ROW_GLOAD          = 4
ROW_ANG_VEL        = 16
ROW_PITCH_ROLL_HDG = 17
ROW_LAT_LON_ALT    = 20
ROW_LOC_VEL_DIST   = 21
REQUIRED_ROWS = [
    ROW_TIMES, ROW_SPEED, ROW_GLOAD,
    ROW_ANG_VEL, ROW_PITCH_ROLL_HDG,
    ROW_LAT_LON_ALT, ROW_LOC_VEL_DIST,
]

RREF_VERSION = 1


# ── X-Plane UDP wire protocol ────────────────────────────────────────────────

def xp_dsel(sock, addr, rows):
    padded = (list(rows) + [0] * 8)[:8]
    sock.sendto(b'DSEL\x00' + struct.pack('<8I', *padded), addr)


def xp_dref(sock, addr, name: str, value: float):
    name_b = name.encode() + b'\x00' * (500 - len(name))
    sock.sendto(b'DREF\x00' + struct.pack('<f', value) + name_b, addr)


def xp_rref(sock, addr, name: str, code: int, rate_hz: int):
    name_b = name.encode() + b'\x00' * (400 - len(name))
    sock.sendto(b'RREF\x00' + struct.pack('<II', rate_hz, code) + name_b, addr)


def xp_parse(pkt: bytes) -> dict:
    """Parse DATA@ packet → {row_code: (v0…v7)}.  values[N] == C++ data[N+1]."""
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
def _angle(pwm, r=1.0): return r * (pwm - 1500) / 500.0
def _range(pwm, r=1.0): return r * (pwm - 1000) / 1000.0
def _deg(pwm, max_deg=20.0): return max_deg * (pwm - 1500) / 500.0

# Flying-wing elevon mixing: ArduPilot outputs per-elevon PWM on CH1/CH2.
# Write directly to wing surface deflection DREFs (degrees) instead of joystick
# yoke ratios, so X-Plane applies no additional internal mixing on top.
# CH0 = SERVO1 = ELEVON_LEFT  → wing1l (left elevon, degrees)
# CH1 = SERVO2 = ELEVON_RIGHT → wing1r (right elevon, degrees)
SERVO_DREFS = [
    ('sim/flightmodel/controls/wing1l_ail1def',  0, _deg),
    ('sim/flightmodel/controls/wing1r_ail1def',  1, _deg),
    ('sim/flightmodel/engine/ENGN_thro_use[0]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[1]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[2]',  2, _range),
    ('sim/flightmodel/engine/ENGN_thro_use[3]',  2, _range),
    ('sim/cockpit2/controls/flap_ratio',          4, _range),
]


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


# ArduPilot CH5 flight-mode band centres (µs).  Any raw PWM is snapped to the
# nearest centre, matching ArduPilot's FLTMODE_CH thresholds:
#   <1231 → 1165, 1231-1360 → 1295, 1361-1490 → 1425,
#   1491-1620 → 1555, 1621-1749 → 1685, ≥1750 → 1815
_FLTMODE_CENTRES = [1165, 1295, 1425, 1555, 1685, 1815]

def _fltmode_pwm(v: float) -> int:
    """Map axis [-1, 1] → nearest ArduPilot flight-mode band centre (µs)."""
    raw = int(max(1000, min(2000, 1500 + v * 500)))
    return min(_FLTMODE_CENTRES, key=lambda c: abs(c - raw))


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='fmuv3-hil ↔ X-Plane bridge (HIL_SENSOR + GPS_INPUT + RC override)'
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
    ap.add_argument('--joystick',      action='store_true',
                    help='Enable joystick → RC_CHANNELS_OVERRIDE (requires pygame)')
    ap.add_argument('--joy-index',     type=int, default=0,
                    help='Joystick device index (default: 0)')
    ap.add_argument('--joy-roll-axis', type=int, default=0,
                    help='Axis index for roll/aileron (default: 0)')
    ap.add_argument('--joy-pitch-axis',type=int, default=1,
                    help='Axis index for pitch/elevator (default: 1, auto-inverted)')
    ap.add_argument('--joy-thr-axis',  type=int, default=2,
                    help='Axis index for throttle (default: 2)')
    ap.add_argument('--joy-yaw-axis',  type=int, default=3,
                    help='Axis index for yaw/rudder (default: 3)')
    ap.add_argument('--joy-fltmode-axis',  type=int, default=4,
                    help='Axis index for CH5 / mode switch (default: -1 = disabled)')
    ap.add_argument('--joy-thr-invert', action='store_true',
                    help='Invert throttle axis (use if full-up gives 1000 µs)')
    ap.add_argument('--joy-rate',      type=int, default=50,
                    help='RC_CHANNELS_OVERRIDE rate Hz (default: 50)')
    ap.add_argument('--debug',         action='store_true')
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
                print(f'[JOY] Mapping: roll=axis{args.joy_roll_axis}  '
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
    # With HIL enabled ArduPilot discards its own SITL physics and uses
    # incoming HIL_SENSOR / GPS_INPUT messages as the sole sensor source.
    # This synchronises SITL attitude with X-Plane via mavlink_xplane.py.
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED |
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,  # custom_mode 0 = MANUAL
    )
    print('[MAV] HIL mode enabled')

    # request SERVO_OUTPUT_RAW at 50 Hz via SET_MESSAGE_INTERVAL
    # (request_data_stream is deprecated and ignored by SITL)
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,  # message id
        20000,   # interval µs → 50 Hz
        0, 0, 0, 0, 0,
    )

    # ── X-Plane socket ───────────────────────────────────────────────────────
    xp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    xp_sock.bind(('0.0.0.0', args.bind_port))
    xp_sock.setblocking(False)
    print(f'[XP]  Listening on :{args.bind_port}  →  {xp_addr}')

    xp_dsel(xp_sock, xp_addr, REQUIRED_ROWS)
    xp_rref(xp_sock, xp_addr, 'sim/version/xplane_internal_version', RREF_VERSION, 1)

    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_joystick',         1.0)
    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_throttles',        1.0)
    xp_dref(xp_sock, xp_addr, 'sim/operation/override/override_control_surfaces', 1.0)

    xp_dref(xp_sock, xp_addr, 'sim/flightmodel/controls/parkbrake', 1.0)
    print('[XP]  Parking brake SET')
    print(f'[XP]  DSEL rows {REQUIRED_ROWS}')

    # ── state ────────────────────────────────────────────────────────────────
    xp_time    = 0.0
    lat        = 0.0
    lon        = 0.0
    alt_m      = 0.0
    agl_m      = 0.0        # altitude above ground (m) — from Row 20 data[4]
    ground_elev = 0.0       # terrain elevation MSL (m) = alt_m - agl_m
    roll_r     = 0.0
    pitch_r    = 0.0
    yaw_r      = 0.0
    # gyro  — ArduPilot body frame [x, y, z] rad/s
    gyro       = [0.0, 0.0, 0.0]
    # accel — ArduPilot body frame [x, y, z] m/s²
    accel_body = [0.0, 0.0, -GRAVITY_MSS]
    # velocity NED m/s
    vel_ned    = [0.0, 0.0, 0.0]
    airspeed   = 0.0

    xp_version      = 0
    pos_valid       = False   # True once Row 20 data received
    att_valid       = False   # True once Row 17 data received (yaw_r is valid)
    home_set        = False   # True once MAV_CMD_DO_SET_HOME has been sent
    home_set_time   = 0.0     # monotonic time to send SET_HOME (2 s after first fix)
    xp_sensor_updated = False # True when fresh IMU/attitude rows arrive from XP

    gps_ivl     = 1.0 / args.gps_rate
    hil_min_ivl = 1.0 / args.hil_rate
    joy_ivl     = 1.0 / args.joy_rate
    last_gps   = 0.0
    last_hil   = 0.0
    last_joy   = 0.0
    prev_joy   = [None, None, None, None, None]  # previous CH1-5 PWM values
    last_dsel  = time.monotonic()
    last_debug = time.monotonic()
    last_srv   = time.monotonic()
    srv_count  = 0     # SERVO_OUTPUT_RAW messages received since last print
    servos     = [0] * 8  # last received servo values (PWM µs)
    dref_sent  = False # True after first DREF batch is sent to X-Plane

    print('\nRunning — Ctrl+C to stop\n')

    while True:
        now = time.monotonic()

        # Re-subscribe DSEL + refresh overrides every 5 s.
        # X-Plane resets override flags on scene reload / unpause.
        if now - last_dsel >= 5.0:
            xp_dsel(xp_sock, xp_addr, REQUIRED_ROWS)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_joystick',         1.0)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_throttles',        1.0)
            xp_dref(xp_sock, xp_addr,
                    'sim/operation/override/override_control_surfaces', 1.0)
            last_dsel = now

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
                pkt, _ = xp_sock.recvfrom(65536)

                if pkt[:4] == b'RREF' and len(pkt) >= 13:
                    code, val = struct.unpack_from('<If', pkt, 5)
                    if code == RREF_VERSION and xp_version == 0:
                        xp_version = int(val)
                        print(f'[XP]  X-Plane {xp_version // 10000} '
                              f'(build {xp_version})')
                    continue

                rows = xp_parse(pkt)
                if not rows:
                    continue

                is_xp12 = (xp_version // 10000) >= 12

                # Row 1 — Times: values[2] = data[3] = elapsed sim time (s)
                if ROW_TIMES in rows:
                    xp_time = rows[ROW_TIMES][2]

                # Row 3 — Speed: values[0] = data[1] = IAS kts
                if ROW_SPEED in rows:
                    airspeed = rows[ROW_SPEED][0] * KNOTS_TO_MS

                # Row 4 — Gload → body-frame accel  (SIM_XPlane.cpp)
                # accel_body.z = -data[5]*g  values[4]
                # accel_body.x =  data[6]*g  values[5]
                # accel_body.y =  data[7]*g  values[6]
                if ROW_GLOAD in rows:
                    d = rows[ROW_GLOAD]
                    accel_body = [
                         d[5] * GRAVITY_MSS,    # x
                         d[6] * GRAVITY_MSS,    # y
                        -d[4] * GRAVITY_MSS,    # z
                    ]
                    xp_sensor_updated = True

                # Row 16 — AngularVelocities
                # XP12 deg/s  data[1..3] → values[0..2]
                # XP11 rad/s  axes swapped: gyro.x=data[2], gyro.y=data[1]
                if ROW_ANG_VEL in rows:
                    d = rows[ROW_ANG_VEL]
                    if is_xp12:
                        gyro = [d[0] * DEG_TO_RAD,
                                d[1] * DEG_TO_RAD,
                                d[2] * DEG_TO_RAD]
                    else:
                        gyro = [d[1], d[0], d[2]]
                    xp_sensor_updated = True

                # Row 17 — PitchRollHeading
                # data[1]=pitch°  data[2]=roll°  data[3]=TRUE heading°
                if ROW_PITCH_ROLL_HDG in rows:
                    d = rows[ROW_PITCH_ROLL_HDG]
                    pitch_r = d[0] * DEG_TO_RAD
                    roll_r  = d[1] * DEG_TO_RAD
                    yaw_r   = d[2] * DEG_TO_RAD
                    if not att_valid:
                        att_valid = True
                        print(f'[ATT] First attitude received  hdg={math.degrees(yaw_r):.1f}°')
                    xp_sensor_updated = True

                # Row 20 — LatLonAlt  (mirrors SIM_XPlane.cpp LatLonAlt case)
                # data[1]=lat°  data[2]=lon°  data[3]=alt_ft_MSL  data[4]=alt_ft_AGL
                if ROW_LAT_LON_ALT in rows:
                    d = rows[ROW_LAT_LON_ALT]
                    lat          = d[0]
                    lon          = d[1]
                    alt_m        = d[2] * FEET_TO_M          # MSL altitude (m)
                    agl_m        = d[3] * FEET_TO_M          # altitude above ground (m)
                    ground_elev  = alt_m - agl_m             # terrain elevation MSL (m)
                    pos_valid    = True

                # Row 21 — LocVelDistTraveled  (SIM_XPlane.cpp)
                # velocity_ef: y=data[4] E, z=−data[5] D, x=−data[6] N
                if ROW_LOC_VEL_DIST in rows:
                    d = rows[ROW_LOC_VEL_DIST]
                    vel_ned = [-d[5], d[3], -d[4]]   # [N, E, D]

        except BlockingIOError:
            pass

        # ── RC_CHANNELS_OVERRIDE (joystick) ───────────────────────────────────
        # Runs unconditionally — does not require X-Plane pos_valid.
        # Sends immediately when any axis changes; repeats at joy_rate Hz
        # as a heartbeat so ArduPilot does not trigger RC failsafe.
        if joystick:
            pygame.event.pump()

            ch1 = _axis_pwm(joystick.get_axis(args.joy_roll_axis))
            ch2 = _axis_pwm(joystick.get_axis(args.joy_pitch_axis), invert=True)
            ch3 = _thr_pwm( joystick.get_axis(args.joy_thr_axis),
                            invert=args.joy_thr_invert)
            ch4 = _axis_pwm(joystick.get_axis(args.joy_yaw_axis))
            ch5 = (_fltmode_pwm(joystick.get_axis(args.joy_fltmode_axis))
                   if args.joy_fltmode_axis >= 0 else UINT16_MAX)

            cur = [ch1, ch2, ch3, ch4, ch5]
            changed = (cur != prev_joy)

            if changed:
                if args.debug:
                    print(f'[JOY] CH1={ch1} CH2={ch2} CH3={ch3} CH4={ch4} CH5={ch5}')
                last_joy = now
                prev_joy = cur

                mav.mav.rc_channels_override_send(
                    mav.target_system, mav.target_component,
                    ch1, ch2, ch3, ch4,
                    ch5, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                )

        if not pos_valid:
            mav.recv_match(blocking=False)
            if args.debug and now - last_debug >= 5.0:
                last_debug = now
                print('[wait] No X-Plane DATA@ — is X-Plane running and unpaused?')
            continue

        t_us = int(now * 1e6)

        # ── Set home ──────────────────────────────────────────────────────────
        # Home altitude must match the terrain elevation so that relative_alt
        # = baro_alt(AGL) - 0 = 0 on the ground.  We set home 2 s after the
        # first GPS fix to let the EKF settle, using ground_elev from Row 20.
        if not home_set and home_set_time == 0.0:
            home_set_time = now + 2.0
        if not home_set and now >= home_set_time:
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,            # confirmation
                0,            # param1: 0 = use specified coordinates
                0, 0, 0,      # param2-4 unused
                lat,          # param5: latitude  (deg)
                lon,          # param6: longitude (deg)
                ground_elev,  # param7: terrain elevation MSL (m)
            )
            home_set = True
            print(f'[MAV] Set home  lat={lat:.6f}  lon={lon:.6f}  '
                  f'terrain={ground_elev:.1f} m  AGL={agl_m:.1f} m')

        # ── HIL_SENSOR ────────────────────────────────────────────────────────
        if xp_sensor_updated and (now - last_hil) >= hil_min_ivl:
            last_hil = now

            # Baro: pressure_alt = AGL altitude (m).
            # handle_hil_sensor() uses pressure_alt directly as sitl->state.altitude,
            # which AP_Baro_SITL reads.  Sending AGL (not MSL) means the baro always
            # reads 0 on the ground regardless of airport elevation — no calibration
            # needed and the EKF relative altitude is correct from the first sample.
            abs_p  = alt_to_pressure(alt_m)          # absolute pressure hPa (MSL)
            diff_p = airspeed_to_diff_pressure(airspeed)
            temp_c = 15.0 - 0.0065 * alt_m           # ISA temperature

            # Magnetometer: mirrors SIM_Aircraft::update_mag_field_bf().
            # IGRF-14 dipole NED field rotated to body frame using true heading
            # (yaw_r from Row 17), exactly as SIM_XPlane.cpp does via DCM.
            mag_ned  = igrf_dipole_ned_gauss(lat, lon)
            mag_body = ned_to_body(mag_ned, roll_r, pitch_r, yaw_r)

            mav.mav.hil_sensor_send(
                t_us,
                accel_body[0], accel_body[1], accel_body[2],  # m/s²
                gyro[0],       gyro[1],       gyro[2],         # rad/s
                mag_body[0],   mag_body[1],   mag_body[2],     # Gauss
                abs_p,                                          # hPa
                diff_p,                                         # hPa
                agl_m,                                          # pressure_alt = AGL m
                temp_c,                                         # °C
                HIL_FIELDS,                                     # fields_updated
                0,                                              # sensor id
            )

        # ── JSON state → SITL flight model ───────────────────────────────────
        # When --json-port is set, SITL runs with --model JSON.
        # Sending position/velocity/attitude directly into sitl->state keeps
        # AP_AHRS_SIM and all sitl->state readers in sync with X-Plane at the
        # full sensor rate, complementing HIL_SENSOR raw injection.
        # ── GPS_INPUT ─────────────────────────────────────────────────────────
        if now - last_gps >= gps_ivl:
            last_gps = now

            # GPS_INPUT yaw: true heading from X-Plane Row 17.
            # Send 0 (unknown) until att_valid to avoid initialising the EKF
            # with a stale yaw_r=0 (north) before we have real attitude data.
            # Once att_valid, mirrors SIM_XPlane.cpp: yawDeg = true heading.
            if att_valid:
                raw    = int((yaw_r % (2 * math.pi)) * 18000 / math.pi)
                hdg_cd = raw if raw != 0 else 36000
            else:
                hdg_cd = 0   # unknown — EKF will not fuse GPS yaw yet

            gps_week, gps_tow_ms = gps_time()
            mav.mav.gps_input_send(
                t_us,
                0,                       # gps_id
                0,                       # ignore_flags — use all fields
                gps_tow_ms, gps_week,    # time_week_ms, time_week
                3,                       # fix_type: 3D
                int(lat * 1e7),          # lat  degE7
                int(lon * 1e7),          # lon  degE7
                alt_m,                   # alt  m MSL  (GPS always uses MSL)
                1.0, 1.0,                # hdop, vdop
                vel_ned[0], vel_ned[1], vel_ned[2],   # vn, ve, vd  m/s
                0.2,                     # speed_accuracy  m/s
                0.3,                     # horiz_accuracy  m
                0.5,                     # vert_accuracy   m
                10,                      # satellites_visible
                hdg_cd,                  # yaw  cdeg (0=unknown, 36000=north)
            )

        # ── SERVO_OUTPUT_RAW → X-Plane DREFs ─────────────────────────────────
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() != 'SERVO_OUTPUT_RAW':
                continue
            servos = [
                msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                msg.servo4_raw, msg.servo5_raw,
                msg.servo6_raw, msg.servo7_raw, msg.servo8_raw,
            ]
            srv_count += 1
            if args.debug:
                labels = ['ail', 'elev', 'thr', 'rud', 'ch5', 'ch6', 'ch7', 'ch8']
                parts  = [f'{l}={v}' for l, v in zip(labels, servos) if v > 0]
                print(f'[SRV] {" ".join(parts)}')

            if not dref_sent:
                dref_sent = True
                print(f'[XP]  First DREF → {xp_addr}')
            for dref_name, ch, conv in SERVO_DREFS:
                val = conv(servos[ch])
                xp_dref(xp_sock, xp_addr, dref_name, val)
                if args.debug:
                    print(f'  → {dref_name} = {val:.3f}')

        # ── periodic 1 s status ────────────────────────────────────────────────
        if now - last_srv >= 1.0:
            last_srv  = now
            hdg = math.degrees(yaw_r) % 360.0
            print(f'[GPS] lat={lat:.6f}  lon={lon:.6f}  alt={alt_m:.1f} m  '
                  f'agl={agl_m:.1f} m  hdg={hdg:.1f}°  IAS={airspeed:.1f} m/s')
            if not args.debug:
                labels = ['ail', 'elev', 'thr', 'rud', 'ch5', 'ch6', 'ch7', 'ch8']
                parts  = [f'{l}={v}' for l, v in zip(labels, servos) if v > 0]
                print(f'[SRV] {" ".join(parts) or "(none)"}  '
                      f'({srv_count} msg/s)')
            srv_count = 0

        # ── debug ─────────────────────────────────────────────────────────────
        if args.debug and now - last_debug >= 5.0:
            last_debug = now
            hdg = math.degrees(yaw_r) % 360
            print(f'[xp]   lat={lat:.5f}  lon={lon:.5f}  '
                  f'alt={alt_m:.1f} m  agl={agl_m:.1f} m  terrain={ground_elev:.1f} m  '
                  f't={xp_time:.1f} s')
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
            print()


if __name__ == '__main__':
    main()
