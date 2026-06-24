#!/usr/bin/env python3
"""
Arm controller — ONBOARD  (runs on the Raspberry Pi)
=====================================================
Receives joint PWM commands from arm_sender.py over UDP and forwards
them to the Pixhawk 6 AUX outputs via MAVLink RC_CHANNELS_OVERRIDE
through MAVProxy.

Confirmed AUX wiring:
    AUX1 (RC ch 9)  → J4
    AUX2 (RC ch 10) → J1
    AUX3 (RC ch 11) → J3
    AUX4 (RC ch 12) → J6  (continuous rotation)
    AUX5 (RC ch 13) → J5
    AUX6 (RC ch 14) → J2
    AUX7 (RC ch 15) → Claw
    AUX8 (RC ch 16) → spare (always 1500)

Incoming UDP packet (from arm_sender.py), comma-separated:
    J1, J2, J3, J4, J5, J6_PWM, Claw, J6_TARGET_ANGLE
    index: 0    1    2    3    4     5      6         7
    PWM range 500-2500 µs.  J6_TARGET_ANGLE in degrees (BNO055 auto-level).

Optional hardware (degrades gracefully if absent):
    BNO055 IMU  — J6 auto-level stabilization when stick is centered
    lgpio/GPIO17 MOSFET — servo power rail switch controlled from web UI
"""

import json
import math
import socket
import threading
import time

from pymavlink import mavutil

from mavlink_rc import MAVLINK_ONBOARD, connect_mavlink, send_rc_channels_override, wait_for_heartbeat

# ── Optional: BNO055 IMU ──────────────────────────────────────────────────────
try:
    import board
    import busio
    import adafruit_bno055
    _i2c = busio.I2C(board.SCL, board.SDA)
    _bno = adafruit_bno055.BNO055_I2C(_i2c, address=0x29)
    HAVE_BNO = True
    print("[arm] BNO055 IMU ready — J6 auto-level enabled")
except Exception as _e:
    HAVE_BNO = False
    _bno     = None
    print(f"[arm] BNO055 not available ({_e}) — J6 manual-only")

# ── Optional: MOSFET via lgpio ────────────────────────────────────────────────
MOSFET_GPIO = 17
MOSFET_PORT = 5007

try:
    import lgpio as _lgpio
    _gpio_h = _lgpio.gpiochip_open(0)
    _lgpio.gpio_claim_output(_gpio_h, MOSFET_GPIO, 0)
    HAVE_GPIO = True
    print(f"[arm] lgpio ready — MOSFET on GPIO{MOSFET_GPIO}")
except Exception as _e:
    HAVE_GPIO = False
    _lgpio   = None
    _gpio_h  = None
    print(f"[arm] lgpio not available ({_e}) — MOSFET control disabled")

# ── Config ────────────────────────────────────────────────────────────────────
UDP_PORT    = 5006
MAVLINK_URL = MAVLINK_ONBOARD
CENTER_US   = 1500
MIN_US      = 500
MAX_US      = 2500
IGNORE      = 65535
OVERRIDE_HZ = 20
PRINT_HZ    = 2
TIMEOUT_SEC = 0.75    # center all joints if no UDP packet received for this long

# Maps incoming CSV joint index → RC channel number (AUX1=ch9, AUX2=ch10 …)
# Incoming order: J1(0), J2(1), J3(2), J4(3), J5(4), J6_PWM(5), Claw(6)
JOINT_TO_RC_CH = {
    0: 10,   # J1   → AUX2
    1: 14,   # J2   → AUX6
    2: 11,   # J3   → AUX3
    3:  9,   # J4   → AUX1
    4: 13,   # J5   → AUX5
    5: 12,   # J6   → AUX4  (continuous rotation — computed separately)
    6: 15,   # Claw → AUX7
}
J6_RC_CH    = 12   # AUX4
SPARE_RC_CH = 16   # AUX8 — always CENTER_US

# J6 BNO055 stabilization (same constants as original new_ar.py)
J6_IN_DEADBAND  = 10      # ±µs from 1500 that counts as "centered"
J6_OUT_MIN      = 1350
J6_OUT_CENTER   = 1500
J6_OUT_MAX      = 1650
J6_KP           = -2.0
J6_DEADBAND_DEG = 3.0
_LEVEL_NORMAL_RAW = [0.0180, -0.9993, 0.0337]
# ─────────────────────────────────────────────────────────────────────────────


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _clamp_us(x):
    return int(_clamp(float(x), MIN_US, MAX_US))


def _normalize(v):
    m = math.sqrt(sum(x * x for x in v))
    return [x / m for x in v] if m > 0.001 else None


_LEVEL_NORMAL = _normalize(_LEVEL_NORMAL_RAW)


def _read_j6_angle_deg():
    """Read BNO055 gravity vector and return tilt angle (degrees). None on error."""
    if not HAVE_BNO:
        return None
    try:
        g = _bno.gravity
        if g is None or any(v is None for v in g):
            return None
        gravity = _normalize(g)
        if gravity is None:
            return None
        dot = _clamp(sum(gravity[i] * _LEVEL_NORMAL[i] for i in range(3)), -1.0, 1.0)
        return math.degrees(math.asin(dot))
    except Exception:
        return None


def _compute_j6_pwm(j6_input_us, j6_target_angle_deg):
    """
    Compute J6 continuous-rotation servo PWM.
    Stick outside deadband → direct manual mapping.
    Stick centered + BNO055 available → auto-level to target angle.
    Stick centered + no BNO055 → hold 1500 (stopped).
    """
    if j6_input_us > CENTER_US + J6_IN_DEADBAND:
        scale = (j6_input_us - (CENTER_US + J6_IN_DEADBAND)) / \
                (MAX_US - (CENTER_US + J6_IN_DEADBAND))
        return int(round(_clamp(
            J6_OUT_CENTER + scale * (J6_OUT_MAX - J6_OUT_CENTER),
            J6_OUT_CENTER, J6_OUT_MAX
        )))

    if j6_input_us < CENTER_US - J6_IN_DEADBAND:
        scale = ((CENTER_US - J6_IN_DEADBAND) - j6_input_us) / \
                ((CENTER_US - J6_IN_DEADBAND) - MIN_US)
        return int(round(_clamp(
            J6_OUT_CENTER - scale * (J6_OUT_CENTER - J6_OUT_MIN),
            J6_OUT_MIN, J6_OUT_CENTER
        )))

    # Stick centered — auto-level if BNO055 is available
    if HAVE_BNO:
        angle = _read_j6_angle_deg()
        if angle is not None:
            err = angle + j6_target_angle_deg
            if abs(err) < J6_DEADBAND_DEG:
                return J6_OUT_CENTER
            return int(round(_clamp(J6_OUT_CENTER + J6_KP * err, J6_OUT_MIN, J6_OUT_MAX)))

    return J6_OUT_CENTER


# ── Shared state (protected by _lock) ────────────────────────────────────────
_lock = threading.Lock()
_joint_us      = [CENTER_US] * 7   # [J1, J2, J3, J4, J5, J6_manual, Claw]
_j6_target_deg = 0.0
_last_pkt_time = 0.0
_rx_count      = 0
_mosfet_on     = False


# ── MOSFET control ────────────────────────────────────────────────────────────
def _set_mosfet(on: bool):
    global _mosfet_on
    _mosfet_on = on
    if HAVE_GPIO:
        _lgpio.gpio_write(_gpio_h, MOSFET_GPIO, 1 if on else 0)


def _mosfet_listener():
    """Background thread: accept MOSFET on/off JSON commands from the web UI."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", MOSFET_PORT))
        s.settimeout(1.0)
    except Exception as e:
        print(f"[arm] MOSFET listener bind failed: {e}")
        return
    while True:
        try:
            data, _ = s.recvfrom(256)
            cmd = json.loads(data.decode())
            if cmd.get("cmd") == "mosfet":
                _set_mosfet(bool(cmd.get("state", False)))
                print(f"[arm] MOSFET {'ON' if _mosfet_on else 'OFF'} (web UI)")
        except socket.timeout:
            pass
        except Exception:
            pass


# ── MAVLink helpers ───────────────────────────────────────────────────────────
def _send_rc_override(master, rc):
    send_rc_channels_override(master, rc, ignore=IGNORE)


def _build_rc_array():
    """Build the 18-element RC array to send, computing J6 fresh each call."""
    rc = [IGNORE] * 18
    with _lock:
        joint_us_snap  = list(_joint_us)
        j6_target_snap = _j6_target_deg
        last_pkt_snap  = _last_pkt_time

    timed_out = (time.time() - last_pkt_snap > TIMEOUT_SEC) and (last_pkt_snap > 0)

    if timed_out:
        # Safety: stop all joints
        for ch in list(JOINT_TO_RC_CH.values()) + [SPARE_RC_CH]:
            rc[ch - 1] = CENTER_US
        return rc

    for joint_idx, rc_ch in JOINT_TO_RC_CH.items():
        if rc_ch == J6_RC_CH:
            # J6 computed separately below
            continue
        rc[rc_ch - 1] = _clamp_us(joint_us_snap[joint_idx])

    # J6 continuous rotation (index 5 = J6_manual input)
    rc[J6_RC_CH - 1] = _compute_j6_pwm(
        _clamp_us(joint_us_snap[5]), j6_target_snap
    )

    rc[SPARE_RC_CH - 1] = CENTER_US

    return rc


def _send_override(master):
    _send_rc_override(master, _build_rc_array())


def _send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global _joint_us, _j6_target_deg, _last_pkt_time, _rx_count

    print(f"[arm] Connecting to MAVProxy at {MAVLINK_URL} ...")
    master = connect_mavlink(MAVLINK_URL)

    print("[arm] Waiting for heartbeat from Pix6 ...")
    hb = wait_for_heartbeat(master, timeout=20)
    if hb:
        print(f"[arm] Heartbeat OK "
              f"(system={master.target_system} component={master.target_component})")
    else:
        print("[arm] *** No heartbeat in 20 s — continuing anyway ***")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.001)

    threading.Thread(target=_mosfet_listener, daemon=True).start()

    print(f"[arm] Listening on UDP {UDP_PORT}")
    print(f"[arm] AUX1=J4  AUX2=J1  AUX3=J3  AUX4=J6  AUX5=J5  AUX6=J2  AUX7=Claw")
    print(f"[arm] BNO055={'yes' if HAVE_BNO else 'no'}  MOSFET={'yes' if HAVE_GPIO else 'no'}")

    last_send      = 0.0
    last_heartbeat = 0.0
    last_print     = 0.0

    try:
        while True:
            now = time.time()

            # ── Receive UDP arm commands ──────────────────────────────────────
            try:
                data, _ = sock.recvfrom(1024)
                line = data.decode(errors="ignore").strip()
                if line.startswith("PWM:"):
                    line = line[4:]
                parts = line.split(",")
                if len(parts) >= 7:
                    vals = [float(x) for x in parts]
                    with _lock:
                        _joint_us      = [_clamp_us(vals[i]) for i in range(7)]
                        _j6_target_deg = float(vals[7]) if len(vals) >= 8 else 0.0
                        _last_pkt_time = now
                        _rx_count     += 1
            except socket.timeout:
                pass
            except (ValueError, IndexError):
                pass

            # ── Send RC_CHANNELS_OVERRIDE at OVERRIDE_HZ ─────────────────────
            if now - last_send >= 1.0 / OVERRIDE_HZ:
                last_send = now
                _send_override(master)

            # ── GCS heartbeat every second ────────────────────────────────────
            if now - last_heartbeat >= 1.0:
                last_heartbeat = now
                _send_heartbeat(master)

            # ── Status print at PRINT_HZ ──────────────────────────────────────
            if now - last_print >= 1.0 / PRINT_HZ:
                last_print = now
                with _lock:
                    rx   = _rx_count
                    j6t  = _j6_target_deg
                    lpt  = _last_pkt_time
                    jus  = list(_joint_us)
                timed_out = (now - lpt > TIMEOUT_SEC) and (lpt > 0)
                j6_pwm = _compute_j6_pwm(_clamp_us(jus[5]), j6t) if not timed_out else CENTER_US
                print(
                    f"[arm] rx={rx} timeout={timed_out} mosfet={_mosfet_on} | "
                    f"J1={jus[0]} J2={jus[1]} J3={jus[2]} J4={jus[3]} "
                    f"J5={jus[4]} J6={j6_pwm}(in={jus[5]},tgt={j6t:.1f}) "
                    f"Claw={jus[6]}"
                )

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n[arm] Stopping — centering all AUX channels.")

    finally:
        # Send CENTER on all AUX channels before exiting
        rc = [IGNORE] * 18
        for ch in list(JOINT_TO_RC_CH.values()) + [SPARE_RC_CH]:
            rc[ch - 1] = CENTER_US
        _send_rc_override(master, rc)
        time.sleep(0.2)

        if HAVE_GPIO and _gpio_h is not None:
            _lgpio.gpio_write(_gpio_h, MOSFET_GPIO, 0)
            _lgpio.gpiochip_close(_gpio_h)

        print("[arm] Done.")


if __name__ == "__main__":
    main()
