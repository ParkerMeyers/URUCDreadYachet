#!/usr/bin/env python3
"""
Arm controller — ONBOARD  (runs on the Raspberry Pi)
=====================================================
Receives joint PWM commands from arm_sender.py over UDP and forwards
them to the Pixhawk 6 AUX outputs via MAVLink RC_CHANNELS_OVERRIDE
through MAVProxy.

Confirmed AUX wiring:
    AUX1 (RC ch 9)  → J5
    AUX2 (RC ch 10) → J2
    AUX3 (RC ch 11) → J6  (continuous rotation, center 1500 µs)
    AUX4 (RC ch 12) → J1
    AUX5 (RC ch 13) → J3
    AUX6 (RC ch 14) → J4
    AUX7 (RC ch 15) → Claw (continuous rotation, center 1515 µs)
    AUX8 (RC ch 16) → spare (always 1500)

Incoming UDP packet (from arm_sender.py), comma-separated:
    J1, J2, J3, J4, J5, J6_PWM, Claw, J6_TARGET_ANGLE
    index: 0    1    2    3    4     5      6         7
    PWM range 500-2500 µs.  J6_TARGET_ANGLE in degrees (BNO055 auto-level).

Optional hardware (degrades gracefully if absent):
    BNO055 IMU  — J6 auto-level stabilization when stick is centered
    lgpio/GPIO17 MOSFET — servo power rail switch controlled from web UI

Manual AUX PWM (web UI, JSON on UDP port 5007):
    {"cmd": "manual_pwm", "enabled": true}   — override arm_sender UDP
    {"cmd": "manual_pwm", "enabled": false}
    {"cmd": "manual_pwm", "aux": 6, "pwm": 1500}  — AUX1–7, 500–2500 µs
    {"cmd": "manual_pwm", "center": true}
"""

import json
import math
import socket
import threading
import time

from pymavlink import mavutil

from mavlink_rc import MAVLINK_ONBOARD, connect_mavlink, send_rc_channels_override, wait_for_heartbeat

# ── Optional hardware (initialized lazily — I2C/GPIO can block at import) ─────
_bno = None
HAVE_BNO = False
_gpio_h = None
HAVE_GPIO = False
_lgpio = None

# ── Config ────────────────────────────────────────────────────────────────────
UDP_PORT    = 5006
MAVLINK_URL = MAVLINK_ONBOARD
MOSFET_GPIO = 17
MOSFET_PORT = 5007
CENTER_US   = 1500
MIN_US      = 500
MAX_US      = 2500
IGNORE      = 65535
OVERRIDE_HZ = 20
PRINT_HZ    = 2
ARM_TELEM_HZ = 5
ARM_TELEM_PORT = 5008   # topside rov_ui listener (must match arm_telemetry_port)
TIMEOUT_SEC = 0.75    # center all joints if no UDP packet received for this long
IMU_READ_STALE_SEC = 2.0

# Maps incoming CSV joint index → RC channel number (AUX1=ch9, AUX2=ch10 …)
# Incoming order: J1(0), J2(1), J3(2), J4(3), J5(4), J6_PWM(5), Claw(6)
JOINT_TO_RC_CH = {
    0: 12,   # J1   → AUX4
    1: 10,   # J2   → AUX2
    2: 13,   # J3   → AUX5
    3: 14,   # J4   → AUX6
    4:  9,   # J5   → AUX1
    5: 11,   # J6   → AUX3  (continuous rotation — computed separately)
    6: 15,   # Claw → AUX7
}
J6_RC_CH      = 11   # AUX3
CLAW_RC_CH    = 15   # AUX7
CLAW_JOINT_IDX = 6
SPARE_RC_CH   = 16   # AUX8 — always CENTER_US

# J6 continuous rotation (center 1500 µs)
J6_IN_DEADBAND  = 10      # ±µs from 1500 that counts as "centered"
J6_OUT_MIN      = 1350
J6_OUT_CENTER   = 1500
J6_OUT_MAX      = 1650
J6_KP           = -2.0
J6_DEADBAND_DEG = 3.0

# Claw continuous rotation (center 1515 µs)
CLAW_CENTER_US  = 1515
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


def _init_optional_hardware() -> None:
    """Probe BNO055 and MOSFET GPIO without blocking process startup."""
    global _bno, HAVE_BNO, _gpio_h, HAVE_GPIO, _lgpio

    try:
        import board
        import busio
        import adafruit_bno055
        _i2c = busio.I2C(board.SCL, board.SDA)
        _bno = adafruit_bno055.BNO055_I2C(_i2c, address=0x29)
        HAVE_BNO = True
        print("[arm] BNO055 IMU ready — J6 auto-level enabled", flush=True)
    except Exception as _e:
        HAVE_BNO = False
        _bno = None
        print(f"[arm] BNO055 not available ({_e}) — J6 manual-only", flush=True)

    try:
        import lgpio as _lgpio_mod
        _lgpio = _lgpio_mod
        _gpio_h = _lgpio.gpiochip_open(0)
        _lgpio.gpio_claim_output(_gpio_h, MOSFET_GPIO, 0)
        HAVE_GPIO = True
        print(f"[arm] lgpio ready — MOSFET on GPIO{MOSFET_GPIO}", flush=True)
    except Exception as _e:
        HAVE_GPIO = False
        _lgpio = None
        _gpio_h = None
        print(f"[arm] lgpio not available ({_e}) — MOSFET control disabled", flush=True)


def _read_j6_angle_deg():
    """Read BNO055 gravity vector and return tilt angle (degrees). None on error."""
    global _last_imu_angle_deg, _last_imu_read_time
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
        angle = math.degrees(math.asin(dot))
        _last_imu_angle_deg = angle
        _last_imu_read_time = time.time()
        return angle
    except Exception:
        return None


def _cached_j6_angle_deg():
    """Return fresh IMU angle, or last good reading if BNO glitched briefly."""
    angle = _read_j6_angle_deg()
    if angle is not None:
        return angle, False
    if (
        _last_imu_angle_deg is not None
        and (time.time() - _last_imu_read_time) <= IMU_READ_STALE_SEC
    ):
        return _last_imu_angle_deg, True
    return None, False


def _compute_j6_pwm(j6_input_us, j6_target_angle_deg):
    """
    Compute J6 continuous-rotation servo PWM.
    Stick outside deadband → direct manual mapping.
    Stick centered + claw hold ON + BNO055 available → auto-level to target angle.
    Stick centered + manual mode (hold OFF or IMU unavailable) → hold stop PWM.
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

    # Stick centered — IMU auto-level only when claw hold is ON and BNO is healthy
    if not _j6_manual_mode():
        angle = _read_j6_angle_deg()
        if angle is not None:
            err = angle + j6_target_angle_deg
            if abs(err) < J6_DEADBAND_DEG:
                return J6_OUT_CENTER
            return int(round(_clamp(J6_OUT_CENTER + J6_KP * err, J6_OUT_MIN, J6_OUT_MAX)))

    return J6_OUT_CENTER


def _imu_available_for_autolevel() -> bool:
    """True when BNO055 is present and returning a usable angle."""
    if not HAVE_BNO:
        return False
    angle, _stale = _cached_j6_angle_deg()
    return angle is not None


def _j6_manual_mode(claw_hold_snap=None) -> bool:
    """Manual J6 when claw hold is OFF or the arm IMU is unavailable."""
    if claw_hold_snap is None:
        with _lock:
            claw_hold_snap = _claw_hold_enabled
    if not claw_hold_snap:
        return True
    return not _imu_available_for_autolevel()


def _claw_output_pwm(claw_input_us):
    """Claw continuous rotation — centered stick/input → 1515 µs stop."""
    us = _clamp_us(claw_input_us)
    if abs(us - CENTER_US) <= J6_IN_DEADBAND:
        return CLAW_CENTER_US
    return us


def _default_joint_us():
    vals = [CENTER_US] * 7
    vals[CLAW_JOINT_IDX] = CLAW_CENTER_US
    return vals


def _default_manual_aux_pwm():
    """Neutral PWM for AUX1–7 in manual mode (AUX7 claw = 1515)."""
    return [CENTER_US, CENTER_US, J6_OUT_CENTER, CENTER_US, CENTER_US, CENTER_US, CLAW_CENTER_US]


def _neutral_pwm_for_rc_ch(rc_ch: int) -> int:
    if rc_ch == J6_RC_CH:
        return J6_OUT_CENTER
    if rc_ch == CLAW_RC_CH:
        return CLAW_CENTER_US
    return CENTER_US


def _fill_rc_neutral(rc: list) -> None:
    for rc_ch in JOINT_TO_RC_CH.values():
        rc[rc_ch - 1] = _neutral_pwm_for_rc_ch(rc_ch)
    rc[SPARE_RC_CH - 1] = CENTER_US


def _should_hold_neutral(last_pkt_time: float) -> bool:
    """Hold stop PWM until arm_sender is live, or after command timeout."""
    if last_pkt_time <= 0:
        return True
    return time.time() - last_pkt_time > TIMEOUT_SEC


# AUX1–AUX7 labels for manual override (web UI types AUX channel number)
AUX_LABELS = ("J5", "J2", "J6", "J1", "J3", "J4", "Claw")

# Joint name / index → AUX port (for "J1 1500" style commands)
JOINT_TO_AUX = {
    1: 4,   # J1 → AUX4
    2: 2,   # J2 → AUX2
    3: 5,   # J3 → AUX5
    4: 6,   # J4 → AUX6
    5: 1,   # J5 → AUX1
    6: 3,   # J6 → AUX3
    7: 7,   # Claw → AUX7
}

# ── Shared state (protected by _lock) ────────────────────────────────────────
_lock = threading.Lock()
_joint_us      = _default_joint_us()
_j6_target_deg = 0.0
_last_pkt_time = 0.0
_rx_count      = 0
_mosfet_on     = False
_manual_mode   = False
_manual_aux_pwm = _default_manual_aux_pwm()
_claw_hold_enabled = True
_telem_sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_telem_subscribers: set = set()
_telem_sub_lock = threading.Lock()
_telem_send_failures: dict = {}
_last_imu_angle_deg = None
_last_imu_read_time = 0.0


# ── MOSFET control ────────────────────────────────────────────────────────────
def _set_mosfet(on: bool):
    global _mosfet_on
    _mosfet_on = on
    if HAVE_GPIO:
        _lgpio.gpio_write(_gpio_h, MOSFET_GPIO, 1 if on else 0)


def _note_telemetry_subscriber(host: str, port: int) -> None:
    with _telem_sub_lock:
        _telem_subscribers.add((host, int(port)))


def _send_arm_telemetry(j6_pwm_out: int) -> None:
    """Push arm BNO055 gripper angle to subscribed topside UI clients."""
    imu_angle, imu_stale = _cached_j6_angle_deg()
    with _lock:
        j6_target = _j6_target_deg
        claw_hold = _claw_hold_enabled
    imu_ok = _imu_available_for_autolevel()
    manual = _j6_manual_mode(claw_hold)

    payload = json.dumps({
        "type": "arm",
        "arm_bno_ready": HAVE_BNO,
        "arm_imu_ok": bool(HAVE_BNO and imu_angle is not None),
        "arm_imu_stale": bool(imu_stale),
        "arm_imu_angle_deg": round(imu_angle, 2) if imu_angle is not None else None,
        "arm_j6_target_deg": round(j6_target, 2),
        "arm_j6_pwm_out": int(j6_pwm_out),
        "arm_claw_hold_request": bool(claw_hold),
        "arm_claw_hold_active": bool(claw_hold and imu_ok),
        "arm_j6_manual": bool(manual),
    }).encode("utf-8")

    with _telem_sub_lock:
        subscribers = list(_telem_subscribers)
    if not subscribers:
        return

    for dest in subscribers:
        try:
            _telem_sock.sendto(payload, dest)
            _telem_send_failures.pop(dest, None)
        except OSError:
            fails = _telem_send_failures.get(dest, 0) + 1
            _telem_send_failures[dest] = fails
            if fails >= 30:
                with _telem_sub_lock:
                    _telem_subscribers.discard(dest)
                _telem_send_failures.pop(dest, None)
                print(f"[arm] Dropped arm telemetry subscriber {dest} after repeated send failures",
                      flush=True)


def _apply_claw_hold_cmd(cmd: dict) -> None:
    """Enable/disable J6 IMU auto-level (claw hold)."""
    global _claw_hold_enabled
    enabled = bool(cmd.get("enabled", True))
    with _lock:
        _claw_hold_enabled = enabled
    mode = "MANUAL" if _j6_manual_mode(enabled) else "IMU HOLD"
    print(f"[arm] Claw hold {'ON' if enabled else 'OFF'} → J6 {mode}", flush=True)


def _apply_manual_pwm_cmd(cmd: dict) -> None:
    """Handle manual AUX PWM commands from the web UI (overrides arm_sender UDP)."""
    global _manual_mode, _manual_aux_pwm

    if cmd.get("center"):
        with _lock:
            _manual_mode = True
            _manual_aux_pwm = _default_manual_aux_pwm()
        print("[arm] Manual AUX: all centered (AUX7 claw → 1515 µs)", flush=True)
        return

    if "enabled" in cmd and cmd.get("aux") is None and cmd.get("joint") is None:
        enabled = bool(cmd.get("enabled"))
        with _lock:
            _manual_mode = enabled
            if enabled:
                _manual_aux_pwm = _default_manual_aux_pwm()
        print(f"[arm] Manual AUX mode {'ON — ignoring arm_sender UDP' if enabled else 'OFF'}",
              flush=True)
        return

    aux = cmd.get("aux")
    if aux is None and cmd.get("joint") is not None:
        try:
            aux = JOINT_TO_AUX.get(int(cmd.get("joint")))
        except (TypeError, ValueError):
            aux = None
    pwm = cmd.get("pwm")
    if aux is None or pwm is None:
        return
    try:
        aux_i = int(aux)
        pwm_i = _clamp_us(pwm)
    except (TypeError, ValueError):
        return
    if not (1 <= aux_i <= 7):
        return
    with _lock:
        _manual_mode = True
        _manual_aux_pwm[aux_i - 1] = pwm_i
    label = AUX_LABELS[aux_i - 1]
    print(f"[arm] Manual AUX{aux_i} ({label}) → {pwm_i} µs [override ON]", flush=True)


def _arm_control_listener():
    """Background thread: MOSFET + manual AUX PWM JSON commands from the web UI."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", MOSFET_PORT))
        s.settimeout(1.0)
    except Exception as e:
        print(f"[arm] Control listener bind failed: {e}")
        return
    while True:
        try:
            data, addr = s.recvfrom(256)
            cmd = json.loads(data.decode())
            if cmd.get("cmd") == "mosfet":
                _set_mosfet(bool(cmd.get("state", False)))
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
                print(f"[arm] MOSFET {'ON' if _mosfet_on else 'OFF'} (web UI)")
            elif cmd.get("cmd") == "manual_pwm":
                _apply_manual_pwm_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "arm_telemetry" and cmd.get("subscribe"):
                port = int(cmd.get("port", ARM_TELEM_PORT))
                _note_telemetry_subscriber(addr[0], port)
                print(f"[arm] Arm telemetry → {addr[0]}:{port}", flush=True)
            elif cmd.get("cmd") == "claw_hold":
                _apply_claw_hold_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
        except socket.timeout:
            pass
        except json.JSONDecodeError as e:
            print(f"[arm] Control listener bad JSON: {e}", flush=True)
        except Exception as e:
            print(f"[arm] Control listener error: {e}", flush=True)


# ── MAVLink helpers ───────────────────────────────────────────────────────────
def _send_rc_override(master, rc):
    send_rc_channels_override(master, rc, ignore=IGNORE)


def _build_rc_manual(aux_vals=None):
    """Build RC override from manual AUX1–7 PWM values (web UI manual mode)."""
    rc = [IGNORE] * 18
    if aux_vals is None:
        with _lock:
            aux_vals = list(_manual_aux_pwm)
    for aux_i in range(1, 8):
        rc[8 + aux_i - 1] = _clamp_us(aux_vals[aux_i - 1])
    rc[SPARE_RC_CH - 1] = CENTER_US
    return rc


def _build_rc_array():
    """Build the 18-element RC array to send, computing J6 fresh each call."""
    with _lock:
        manual = _manual_mode
        if manual:
            aux_vals = list(_manual_aux_pwm)
    if manual:
        return _build_rc_manual(aux_vals)

    rc = [IGNORE] * 18
    with _lock:
        joint_us_snap  = list(_joint_us)
        j6_target_snap = _j6_target_deg
        last_pkt_snap  = _last_pkt_time

    if _should_hold_neutral(last_pkt_snap):
        # Safety: stop all joints at neutral PWM (no BNO auto-level before arm_sender)
        _fill_rc_neutral(rc)
        return rc

    for joint_idx, rc_ch in JOINT_TO_RC_CH.items():
        if rc_ch == J6_RC_CH:
            continue
        if joint_idx == CLAW_JOINT_IDX:
            rc[rc_ch - 1] = _claw_output_pwm(joint_us_snap[joint_idx])
        else:
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


def _try_connect_mavlink():
    """Connect to MAVProxy; return master or None (never blocks startup indefinitely)."""
    try:
        print(f"[arm] Connecting to MAVProxy at {MAVLINK_URL} ...", flush=True)
        master = connect_mavlink(MAVLINK_URL, timeout=12.0)
        print("[arm] Waiting for heartbeat from Pix6 ...", flush=True)
        hb = wait_for_heartbeat(master, timeout=8.0)
        if hb:
            print(
                f"[arm] Heartbeat OK "
                f"(system={master.target_system} component={master.target_component})",
                flush=True,
            )
        else:
            print("[arm] *** No heartbeat in 8 s — continuing anyway ***", flush=True)
        return master
    except Exception as e:
        print(f"[arm] MAVLink unavailable ({e}) — will retry", flush=True)
        return None


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global _joint_us, _j6_target_deg, _last_pkt_time, _rx_count

    print("[arm] Arm controller starting...", flush=True)
    threading.Thread(
        target=_init_optional_hardware, daemon=True, name="arm-hw-init",
    ).start()

    # Bind UDP first so arm_sender / UI can reach us while MAVLink connects.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", UDP_PORT))
    except OSError as e:
        print(f"[arm] FATAL: cannot bind UDP {UDP_PORT}: {e}", flush=True)
        raise
    sock.settimeout(0.001)

    threading.Thread(target=_arm_control_listener, daemon=True).start()

    print(f"[arm] Listening on UDP {UDP_PORT}", flush=True)
    print(f"[arm] Manual AUX PWM on UDP {MOSFET_PORT} (cmd=manual_pwm)", flush=True)
    print(f"[arm] AUX1=J5  AUX2=J2  AUX3=J6  AUX4=J1  AUX5=J3  AUX6=J4  AUX7=Claw", flush=True)

    master = _try_connect_mavlink()
    print(f"[arm] BNO055={'yes' if HAVE_BNO else 'no'}  MOSFET={'yes' if HAVE_GPIO else 'no'}  "
          f"MAVLink={'yes' if master else 'retrying'}", flush=True)

    last_send      = 0.0
    last_heartbeat = 0.0
    last_print     = 0.0
    last_telem     = 0.0
    last_j6_pwm    = J6_OUT_CENTER
    last_mav_try   = time.time()
    MAV_RETRY_SEC  = 5.0

    try:
        while True:
            now = time.time()

            if master is None and (now - last_mav_try) >= MAV_RETRY_SEC:
                last_mav_try = now
                master = _try_connect_mavlink()
                if master:
                    print("[arm] MAVLink link up — RC override active", flush=True)

            # ── Receive UDP arm commands ──────────────────────────────────────
            try:
                data, addr = sock.recvfrom(1024)
                line = data.decode(errors="ignore").strip()
                if line.startswith("PWM:"):
                    line = line[4:]
                parts = line.split(",")
                if len(parts) >= 7:
                    with _lock:
                        if _manual_mode:
                            pass  # web manual override active — ignore arm_sender
                        else:
                            vals = [float(x) for x in parts]
                            _joint_us      = [_clamp_us(vals[i]) for i in range(7)]
                            _j6_target_deg = float(vals[7]) if len(vals) >= 8 else 0.0
                            _last_pkt_time = now
                            _rx_count     += 1
                    _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            except socket.timeout:
                pass
            except (ValueError, IndexError):
                pass

            # ── Send RC_CHANNELS_OVERRIDE at OVERRIDE_HZ ─────────────────────
            if master is not None and now - last_send >= 1.0 / OVERRIDE_HZ:
                last_send = now
                rc = _build_rc_array()
                _send_rc_override(master, rc)
                if rc[J6_RC_CH - 1] != IGNORE:
                    last_j6_pwm = int(rc[J6_RC_CH - 1])

            if now - last_telem >= 1.0 / ARM_TELEM_HZ:
                last_telem = now
                _send_arm_telemetry(last_j6_pwm)

            # ── GCS heartbeat every second ────────────────────────────────────
            if master is not None and now - last_heartbeat >= 1.0:
                last_heartbeat = now
                _send_heartbeat(master)

            # ── Status print at PRINT_HZ ──────────────────────────────────────
            if now - last_print >= 1.0 / PRINT_HZ:
                last_print = now
                with _lock:
                    rx      = _rx_count
                    j6t     = _j6_target_deg
                    lpt     = _last_pkt_time
                    jus     = list(_joint_us)
                    manual  = _manual_mode
                    aux_pwm = list(_manual_aux_pwm)
                if manual:
                    aux_str = " ".join(
                        f"A{i+1}={aux_pwm[i]}" for i in range(7)
                    )
                    print(
                        f"[arm] MANUAL mosfet={_mosfet_on} | {aux_str}",
                        flush=True,
                    )
                else:
                    hold_neutral = _should_hold_neutral(lpt)
                    with _lock:
                        claw_hold = _claw_hold_enabled
                    j6_manual = _j6_manual_mode(claw_hold) if not hold_neutral else True
                    j6_pwm = J6_OUT_CENTER if hold_neutral else _compute_j6_pwm(_clamp_us(jus[5]), j6t)
                    print(
                        f"[arm] rx={rx} hold={hold_neutral} claw={claw_hold} j6={'MAN' if j6_manual else 'IMU'} "
                        f"mosfet={_mosfet_on} | "
                        f"J1={jus[0]} J2={jus[1]} J3={jus[2]} J4={jus[3]} "
                        f"J5={jus[4]} J6={j6_pwm}(in={jus[5]},tgt={j6t:.1f}) "
                        f"Claw={jus[6]}"
                    )

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n[arm] Stopping — centering all AUX channels.")

    finally:
        # Send neutral on all AUX channels before exiting
        if master is not None:
            rc = [IGNORE] * 18
            _fill_rc_neutral(rc)
            _send_rc_override(master, rc)
            time.sleep(0.2)

        if HAVE_GPIO and _gpio_h is not None:
            _lgpio.gpio_write(_gpio_h, MOSFET_GPIO, 0)
            _lgpio.gpiochip_close(_gpio_h)

        print("[arm] Done.")


if __name__ == "__main__":
    main()
