#!/usr/bin/env python3
"""
test_joystick_xplane.py — Joystick → X-Plane direct DREF control with live
per-channel bar visualization in the console.

Sends joystick axes as X-Plane UDP DREF packets straight to X-Plane's input
port (default 49000).  No ArduPilot / MAVLink involved — useful for testing
aircraft model response without any HIL/SITL setup.

DREF mapping:
  Axis 0  → roll     sim/joystick/yoke_roll_ratio           (−1 … +1)
  Axis 1  → pitch    sim/joystick/yoke_pitch_ratio          (inverted; push-fwd = nose-down)
  Axis 2  → throttle sim/cockpit2/engine/actuators/throttle_ratio[0]  (0 … 1, full-range axis)
  Axis 3  → yaw      sim/joystick/yoke_heading_ratio        (−1 … +1)

DREF packet format (509 bytes, UDP to X-Plane port 49000):
  b'DREF\\x00'  — 5-byte header
  <float32>    — 4-byte value (little-endian)
  <bytes×500>  — DREF name, null-padded to 500 bytes

Usage:
    python3 test_joystick_xplane.py
    python3 test_joystick_xplane.py --ip 192.168.1.5
    python3 test_joystick_xplane.py --ip 192.168.1.5 --port 49000
    python3 test_joystick_xplane.py --joy 1 --rate 50
    python3 test_joystick_xplane.py --no-send          # visualization only, no UDP
"""

import argparse
import signal
import socket
import struct
import sys
import time

try:
    import pygame
except ImportError:
    print("pygame not installed — run: pip3 install pygame")
    sys.exit(1)

# ── Channel definitions ───────────────────────────────────────────────────────

# (dref_name, display_label, range_mode)
# range_mode "centred" → bar centred at 0, range −1…+1
# range_mode "full"    → bar from left,   range  0…+1
_CHANNELS = [
    ("sim/joystick/yoke_roll_ratio",              "Roll    ", "centred"),
    ("sim/joystick/yoke_pitch_ratio",             "Pitch   ", "centred"),
    ("sim/cockpit2/engine/actuators/throttle_ratio[0]", "Throttle", "full"),
    ("sim/joystick/yoke_heading_ratio",           "Yaw     ", "centred"),
]

BAR_WIDTH = 28

# ── DREF UDP helpers ──────────────────────────────────────────────────────────

def send_dref(sock: socket.socket, ip: str, port: int,
              dref: str, value: float) -> None:
    """Send one DREF packet (509 bytes) to X-Plane."""
    name_b = dref.encode("ascii").ljust(500, b"\x00")
    sock.sendto(b"DREF\x00" + struct.pack("<f", value) + name_b, (ip, port))


def release_controls(sock: socket.socket, ip: str, port: int) -> None:
    """Zero all channel DREFs so X-Plane regains control."""
    for dref, _, _ in _CHANNELS:
        send_dref(sock, ip, port, dref, 0.0)


# ── Joystick → DREF values ────────────────────────────────────────────────────

def _safe_axis(joy: "pygame.joystick.Joystick", i: int) -> float:
    return joy.get_axis(i) if i < joy.get_numaxes() else 0.0


def read_values(joy: "pygame.joystick.Joystick") -> dict[str, float]:
    """Return {dref: value} for the four mapped channels."""
    roll     =  _safe_axis(joy, 0)
    pitch    = -_safe_axis(joy, 1)         # invert: push-fwd axis+ → nose-down DREF−
    thr_raw  =  _safe_axis(joy, 2)
    throttle = (thr_raw + 1.0) * 0.5      # full-range axis −1…+1 → DREF 0…1
    yaw      =  _safe_axis(joy, 3)

    return {
        _CHANNELS[0][0]: roll,
        _CHANNELS[1][0]: pitch,
        _CHANNELS[2][0]: throttle,
        _CHANNELS[3][0]: yaw,
    }


# ── Console visualization ─────────────────────────────────────────────────────

def _bar(v: float, mode: str) -> str:
    if mode == "centred":
        frac   = (v + 1.0) / 2.0
        filled = max(0, min(BAR_WIDTH, int(frac * BAR_WIDTH)))
        bar    = [" "] * BAR_WIDTH
        for i in range(filled):
            bar[i] = "█"
        bar[BAR_WIDTH // 2] = "│"          # centre marker
        return "".join(bar)
    else:                                   # "full" 0…1
        filled = max(0, min(BAR_WIDTH, int(v * BAR_WIDTH)))
        return "█" * filled + " " * (BAR_WIDTH - filled)


def render(values: dict, joy_name: str, target: str, n_sent: int) -> None:
    lines = ["\033[H\033[J"]
    lines.append(f" Joystick : {joy_name}")
    lines.append(f" X-Plane  : {target}   sent={n_sent}")
    lines.append(f" {'─' * 56}")
    lines.append(f"  {'Channel':<10}  −1{'':>{BAR_WIDTH // 2 - 2}}0"
                 f"{'':>{BAR_WIDTH // 2 - 1}}+1    val")
    lines.append(f"  {'─' * 56}")
    for dref, label, mode in _CHANNELS:
        v   = values.get(dref, 0.0)
        bar = _bar(v, mode)
        val = f"{v:+.3f}" if mode == "centred" else f" {v:.3f}"
        lines.append(f"  {label}  [{bar}]  {val}")
    lines.append(f"  {'─' * 56}")
    lines.append("  Ctrl-C to quit")
    print("\n".join(lines), end="", flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Joystick → X-Plane DREF UDP with console bar visualization"
    )
    parser.add_argument("--ip",      default="127.0.0.1",
                        help="X-Plane host IP (default: 127.0.0.1)")
    parser.add_argument("--port",    type=int, default=49000,
                        help="X-Plane UDP port (default: 49000)")
    parser.add_argument("--joy",     type=int, default=0,
                        help="Joystick device index (default: 0)")
    parser.add_argument("--rate",    type=float, default=20.0,
                        help="Send rate in Hz (default: 20)")
    parser.add_argument("--no-send", action="store_true",
                        help="Visualise only — do not send DREFs to X-Plane")
    args = parser.parse_args()

    # ── UDP socket ────────────────────────────────────────────────────────────
    sock = None
    if not args.no_send:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[XP] Sending DREFs to {args.ip}:{args.port}")
    else:
        print("[XP] --no-send: visualization only")

    # ── Joystick ──────────────────────────────────────────────────────────────
    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        print("[JOY] No joystick detected")
        sys.exit(1)
    if args.joy >= count:
        print(f"[JOY] Index {args.joy} out of range — found {count} device(s):")
        for i in range(count):
            j = pygame.joystick.Joystick(i)
            j.init()
            print(f"  [{i}] {j.get_name()}")
        sys.exit(1)

    joy = pygame.joystick.Joystick(args.joy)
    joy.init()
    print(f"[JOY] [{args.joy}] {joy.get_name()}  "
          f"axes={joy.get_numaxes()}  buttons={joy.get_numbuttons()}")

    interval = 1.0 / args.rate
    n_sent   = 0
    running  = True

    def _shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    target = f"{args.ip}:{args.port}" if sock else "(no send)"

    try:
        while running:
            t0 = time.monotonic()

            pygame.event.pump()
            values = read_values(joy)

            if sock:
                for dref, value in values.items():
                    send_dref(sock, args.ip, args.port, dref, value)
                n_sent += 1

            render(values, joy.get_name(), target, n_sent)

            elapsed = time.monotonic() - t0
            rem     = interval - elapsed
            if rem > 0:
                time.sleep(rem)

    finally:
        if sock:
            release_controls(sock, args.ip, args.port)
            print("\n[XP] Controls released (zeroed)")
            sock.close()
        joy.quit()
        pygame.quit()
        print("[JOY] Closed")


if __name__ == "__main__":
    main()
