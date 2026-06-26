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
    AUX7 (RC ch 15) → Claw (continuous rotation; stop PWM configurable, default 1515 µs)
    AUX8 (RC ch 16) → spare (always 1500)

Incoming UDP packet (from arm_sender.py), comma-separated:
    J1, J2, J3, J4, J5, J6_PWM, Claw, J6_TARGET_ANGLE
    index: 0    1    2    3    4     5      6         7
    PWM range 500-2500 µs.  J6_TARGET_ANGLE in degrees (BNO055 auto-level).

Optional hardware (degrades gracefully if absent):
    BNO055 IMU  — J6 auto-level stabilization when stick is centered
    lgpio/GPIO17 MOSFET — driven by onboard/mosfet_service.py (UDP 5007), not this process

Manual AUX PWM (web UI, JSON on UDP port 5006 or 5009):
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

from mavlink_rc import MAVLINK_ONBOARD_ARM, connect_mavlink, send_rc_channels_override, wait_for_heartbeat

# ── Optional hardware (initialized lazily — I2C can block at import) ─────
_bno = None
_i2c = None
HAVE_BNO = False
BNO_INIT_RETRY_SEC = 5.0
BNO_FAIL_LOG_INTERVAL_SEC = 30.0
_bno_last_fail_log = 0.0
_bno_fail_hint_shown = False

# ── Config ────────────────────────────────────────────────────────────────────
UDP_PORT    = 5006
MAVLINK_URL = MAVLINK_ONBOARD_ARM
# MOSFET is handled by onboard/mosfet_service.py on UDP 5007 (same as test script).
ARM_CONTROL_PORT = 5009
CENTER_US   = 1500
MIN_US      = 500
MAX_US      = 2500
IGNORE      = 65535
OVERRIDE_HZ = 20
PRINT_HZ    = 2
ARM_TELEM_HZ = 5
ARM_TELEM_PORT = 5008   # topside rov_ui listener (must match arm_telemetry_port)
TIMEOUT_SEC = 0.75    # center all joints if no UDP packet received for this long
IMU_READ_STALE_SEC = 5.0       # drop cached angle after this long without a good read
IMU_STALE_WARN_SEC = 2.0       # UI/control "stale" after this long without a good read
IMU_MIN_READ_INTERVAL_SEC = 0.05  # cap BNO055 I2C reads (~20 Hz)
IMU_REINIT_AFTER_FAILURES = 50    # re-open BNO055 after this many consecutive bad reads

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
J6_IN_DEADBAND      = 10    # ±µs from 1500 — stick centered for IMU hold (hold ON)
J6_MANUAL_DEADBAND  = 50    # ±µs from 1500 — snap to stop when rotation hold is OFF
J6_OUT_MIN      = 1350
J6_OUT_CENTER   = 1500
J6_OUT_MAX      = 1650
J6_KP           = -2.0
J6_DEADBAND_DEG = 3.0

# Claw continuous rotation — stop PWM configurable (synced from topside rov_config.json)
CLAW_STOP_US_DEFAULT = 1515
CLAW_IN_DEADBAND = 10     # ±µs from 1500 that counts as "centered" stick input
_claw_stop_pwm   = CLAW_STOP_US_DEFAULT
_LEVEL_NORMAL_RAW = [0.0180, -0.9993, 0.0337]
# Wrist J6 angle: atan2 of two gravity components (rotation about Y, not Y tilt).
# Previous asin(dot) tracked alignment with -Y — wrong DOF for wrist roll.
# Swap NUM/DEN to (2, 1) for rotation about X if needed on your mount.
J6_IMU_GRAV_NUM = 2   # Z
J6_IMU_GRAV_DEN = 0   # X
J6_IMU_SIGN = -1.0      # flip: physical CW matches UI CW
J6_IMU_ZERO_OFFSET_DEG = -154.0  # raw gravity angle at flat/stow (0° after cal)
_j6_imu_sign = J6_IMU_SIGN
_j6_imu_zero_offset = J6_IMU_ZERO_OFFSET_DEG
# ─────────────────────────────────────────────────────────────────────────────


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _clamp_us(x):
    return int(_clamp(float(x), MIN_US, MAX_US))


def _claw_stop_us() -> int:
    with _lock:
        return int(_claw_stop_pwm)


def _normalize(v):
    m = math.sqrt(sum(x * x for x in v))
    return [x / m for x in v] if m > 0.001 else None


_LEVEL_NORMAL = _normalize(_LEVEL_NORMAL_RAW)
_J6_IMU_REF_DEG = (
    math.degrees(math.atan2(_LEVEL_NORMAL[J6_IMU_GRAV_NUM], _LEVEL_NORMAL[J6_IMU_GRAV_DEN]))
    if _LEVEL_NORMAL is not None else 0.0
)


def _wrap_deg180(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def _read_j6_grav_deg():
    """Gravity atan2 angle minus factory ref, before user zero/sign."""
    if not HAVE_BNO or _bno is None:
        return None

    gravity = None
    with _bno_lock:
        for attempt in range(4):
            try:
                g = _bno.gravity
                if g is not None and not any(v is None for v in g):
                    gravity = g
                    break
            except Exception:
                pass
            if attempt < 3:
                time.sleep(0.003)

        # Fusion gravity can briefly return None — accel ≈ gravity when stationary.
        if gravity is None:
            for attempt in range(3):
                try:
                    a = _bno.acceleration
                    if a is not None and not any(v is None for v in a):
                        gravity = a
                        break
                except Exception:
                    pass
                if attempt < 2:
                    time.sleep(0.003)

    if gravity is None:
        return None

    vec = _normalize(gravity)
    if vec is None:
        return None
    raw_deg = math.degrees(math.atan2(
        vec[J6_IMU_GRAV_NUM],
        vec[J6_IMU_GRAV_DEN],
    ))
    return _wrap_deg180(raw_deg - _J6_IMU_REF_DEG)


def _apply_j6_imu_calibration(raw_deg: float) -> float:
    with _lock:
        sign = _j6_imu_sign
        zero = _j6_imu_zero_offset
    return _wrap_deg180(sign * (float(raw_deg) - zero))


def _reset_imu_cache() -> None:
    """Clear IMU angle cache after a fresh BNO055 init."""
    global _last_imu_angle_deg, _last_imu_read_time
    global _imu_snap_angle, _imu_snap_stale, _imu_last_poll_time
    global _imu_consecutive_failures
    _last_imu_angle_deg = None
    _last_imu_read_time = 0.0
    _imu_snap_angle = None
    _imu_snap_stale = False
    _imu_last_poll_time = 0.0
    _imu_consecutive_failures = 0


def _release_bno() -> None:
    """Release the I2C bus so the next process start can reopen the BNO055."""
    global _bno, _i2c, HAVE_BNO
    with _bno_lock:
        _bno = None
        HAVE_BNO = False
        if _i2c is not None:
            try:
                if hasattr(_i2c, "deinit"):
                    _i2c.deinit()
            except Exception:
                pass
            _i2c = None


def _try_init_bno_once() -> bool:
    """Try once to bring up the BNO055. Returns True on success."""
    global _bno, _i2c, HAVE_BNO, _bno_last_fail_log, _bno_fail_hint_shown

    _release_bno()
    time.sleep(0.15)

    try:
        import board
        import busio
        import adafruit_bno055

        i2c = busio.I2C(board.SCL, board.SDA)
        bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
        bno.mode = adafruit_bno055.NDOF_MODE
        time.sleep(0.05)
        # Confirm the chip responds before marking it ready.
        with _bno_lock:
            for _ in range(15):
                g = bno.gravity
                if g is not None and not any(v is None for v in g):
                    _i2c = i2c
                    _bno = bno
                    HAVE_BNO = True
                    _reset_imu_cache()
                    print("[arm] BNO055 IMU ready — J6 auto-level enabled", flush=True)
                    return True
                time.sleep(0.05)
        _release_bno()
        return False
    except Exception as _e:
        _release_bno()
        now = time.time()
        if now - _bno_last_fail_log >= BNO_FAIL_LOG_INTERVAL_SEC:
            _bno_last_fail_log = now
            print(f"[arm] BNO055 init failed ({_e})", flush=True)
            if not _bno_fail_hint_shown:
                _bno_fail_hint_shown = True
                print(
                    "[arm] BNO055 hint: enable I2C (raspi-config), check wiring, "
                    "run: i2cdetect -y 1  (expect 0x29). MOSFET is on UDP 5007 "
                    "via mosfet_service.py — independent of IMU.",
                    flush=True,
                )
        return False


def _init_optional_hardware() -> None:
    """Probe BNO055 in background — does not block arm UDP / MAVLink startup."""
    attempt = 0
    while not HAVE_BNO:
        if _try_init_bno_once():
            break
        attempt += 1
        if attempt == 1:
            print(
                "[arm] BNO055 not ready — retrying (release I2C after prior stop)...",
                flush=True,
            )
        time.sleep(BNO_INIT_RETRY_SEC)


def _start_imu_poll_thread() -> None:
    """Dedicated BNO055 reader — avoids stale reads when the main loop is busy."""
    def _loop():
        global _imu_consecutive_failures
        while True:
            if HAVE_BNO:
                _poll_imu_cache(force=True)
                if _imu_consecutive_failures >= IMU_REINIT_AFTER_FAILURES:
                    _imu_consecutive_failures = 0
                    print(
                        "[arm] BNO055 reads failing — reinitializing I2C...",
                        flush=True,
                    )
                    _try_init_bno_once()
            else:
                _imu_snap_angle = None
                _imu_snap_stale = False
            time.sleep(IMU_MIN_READ_INTERVAL_SEC)

    threading.Thread(target=_loop, daemon=True, name="arm-imu").start()


def _read_j6_angle_deg_raw():
    """Single BNO055 gravity read. None on error; does not touch the IMU cache."""
    raw = _read_j6_grav_deg()
    if raw is None:
        return None
    return _apply_j6_imu_calibration(raw)


def _poll_imu_cache(force=False):
    """BNO055 read; normally called from the dedicated arm-imu thread."""
    global _last_imu_angle_deg, _last_imu_read_time
    global _imu_snap_angle, _imu_snap_stale, _imu_last_poll_time
    global _imu_consecutive_failures

    if not HAVE_BNO:
        _imu_snap_angle = None
        _imu_snap_stale = False
        return

    now = time.time()
    if not force and (now - _imu_last_poll_time) < IMU_MIN_READ_INTERVAL_SEC:
        if _last_imu_read_time > 0.0:
            age = now - _last_imu_read_time
            _imu_snap_stale = age > IMU_STALE_WARN_SEC
            if age > IMU_READ_STALE_SEC:
                _imu_snap_angle = None
            elif _imu_snap_angle is None and _last_imu_angle_deg is not None:
                _imu_snap_angle = _last_imu_angle_deg
        return

    _imu_last_poll_time = now
    angle = _read_j6_angle_deg_raw()
    now = time.time()

    if angle is not None:
        _last_imu_angle_deg = angle
        _last_imu_read_time = now
        _imu_snap_angle = angle
        _imu_snap_stale = False
        _imu_consecutive_failures = 0
        return

    _imu_consecutive_failures += 1

    if _last_imu_angle_deg is not None and (now - _last_imu_read_time) <= IMU_READ_STALE_SEC:
        _imu_snap_angle = _last_imu_angle_deg
        _imu_snap_stale = (now - _last_imu_read_time) > IMU_STALE_WARN_SEC
        return

    _imu_snap_angle = None
    _imu_snap_stale = True


def _imu_read_age_sec() -> float | None:
    if _last_imu_read_time <= 0.0:
        return None
    return round(time.time() - _last_imu_read_time, 3)


def _cached_j6_angle_deg():
    """Return the latest polled IMU angle and whether it is getting old."""
    return _imu_snap_angle, _imu_snap_stale


def _j6_stick_centered(j6_input_us: int) -> bool:
    return abs(int(j6_input_us) - CENTER_US) <= J6_IN_DEADBAND


def _compute_j6_pwm(j6_input_us, j6_target_angle_deg, claw_hold_enabled: bool):
    """
    Compute J6 continuous-rotation servo PWM.

    Rotation hold OFF → never run IMU; snap to stop PWM inside manual deadband.
    Rotation hold ON  → stick deflection passthrough; centered stick → IMU hold.
    """
    us = _clamp_us(j6_input_us)

    # Hold OFF — manual only; tolerate RC noise near center (matches arm test stop).
    if not claw_hold_enabled:
        if abs(us - CENTER_US) <= J6_MANUAL_DEADBAND:
            return J6_OUT_CENTER
        return us

    # Hold ON — operator stick overrides IMU for manual wrist trim.
    if not _j6_stick_centered(us):
        return us

    # Hold ON + centered stick → IMU rotation hold to target angle.
    angle, stale = _cached_j6_angle_deg()
    if angle is not None and not stale:
        err = _wrap_deg180(angle + j6_target_angle_deg)
        if abs(err) < J6_DEADBAND_DEG:
            return J6_OUT_CENTER
        return int(round(_clamp(
            J6_OUT_CENTER + J6_KP * err, J6_OUT_MIN, J6_OUT_MAX
        )))

    return J6_OUT_CENTER


def _imu_available_for_autolevel() -> bool:
    """True when BNO055 is present and returning a fresh angle."""
    if not HAVE_BNO:
        return False
    angle, stale = _cached_j6_angle_deg()
    return angle is not None and not stale


def _j6_manual_mode(claw_hold_snap=None) -> bool:
    """Manual J6 when claw hold is OFF or the arm IMU is unavailable."""
    if claw_hold_snap is None:
        with _lock:
            claw_hold_snap = _claw_hold_enabled
    if not claw_hold_snap:
        return True
    return not _imu_available_for_autolevel()


def _claw_output_pwm(claw_input_us):
    """Claw continuous rotation — centered stick/input → configured stop PWM."""
    us = _clamp_us(claw_input_us)
    if abs(us - CENTER_US) <= CLAW_IN_DEADBAND:
        return _claw_stop_us()
    return us


def _default_joint_us():
    vals = [CENTER_US] * 7
    vals[CLAW_JOINT_IDX] = _claw_stop_us()
    return vals


def _default_manual_aux_pwm():
    """Neutral PWM for AUX1–7 in manual mode (AUX7 claw uses configured stop)."""
    return [CENTER_US, CENTER_US, J6_OUT_CENTER, CENTER_US, CENTER_US, CENTER_US, _claw_stop_us()]


def _neutral_pwm_for_rc_ch(rc_ch: int) -> int:
    if rc_ch == J6_RC_CH:
        return J6_OUT_CENTER
    if rc_ch == CLAW_RC_CH:
        return _claw_stop_us()
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
_manual_mode   = False
_manual_aux_pwm = _default_manual_aux_pwm()
_claw_hold_enabled = False
_arm_enabled       = True   # topside sends arm_enable=false when DISARMED
_mavlink_up        = False
_mav_master        = None
_mav_lock          = threading.Lock()
_mav_connecting    = False
_preset_motion     = False
_preset_motion_since = 0.0
PRESET_MOTION_TIMEOUT_SEC = 45.0
_telem_sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_telem_subscribers: set = set()
_telem_sub_lock = threading.Lock()
_telem_send_failures: dict = {}
_last_imu_angle_deg = None
_last_imu_read_time = 0.0
_imu_snap_angle = None
_imu_snap_stale = False
_imu_last_poll_time = 0.0
_imu_consecutive_failures = 0
_bno_lock = threading.Lock()


def _note_telemetry_subscriber(host: str, port: int) -> None:
    with _telem_sub_lock:
        _telem_subscribers.add((host, int(port)))


def _send_arm_telemetry(j6_pwm_out: int) -> None:
    """Push arm BNO055 gripper angle to subscribed topside UI clients.

    IMU readout is always included when the BNO055 is reading — independent of
    rotation/claw hold (hold only affects J6 PWM in _compute_j6_pwm).
    """
    imu_angle, imu_stale = _cached_j6_angle_deg()
    with _lock:
        j6_target = _j6_target_deg
        claw_hold = _claw_hold_enabled
        imu_zero = _j6_imu_zero_offset
        imu_sign = _j6_imu_sign
        rx_count = _rx_count
        last_pkt = _last_pkt_time
        joint_us = list(_joint_us)
        armed = _arm_enabled
        manual = _manual_mode
        preset = _preset_motion
    hold_neutral = _should_hold_neutral(last_pkt)
    imu_ok = _imu_available_for_autolevel()
    j6_manual = _j6_manual_mode(claw_hold)

    payload = json.dumps({
        "type": "arm",
        "arm_bno_ready": HAVE_BNO,
        "arm_imu_ok": bool(HAVE_BNO and imu_angle is not None),
        "arm_imu_stale": bool(imu_stale),
        "arm_imu_read_age_sec": _imu_read_age_sec(),
        "arm_imu_angle_deg": round(imu_angle, 2) if imu_angle is not None else None,
        "arm_j6_target_deg": round(j6_target, 2),
        "arm_j6_pwm_out": int(j6_pwm_out),
        "arm_claw_hold_request": bool(claw_hold),
        "arm_claw_hold_active": bool(claw_hold and imu_ok),
        "arm_j6_manual": bool(j6_manual),
        "arm_enabled": bool(armed),
        "arm_imu_zero_offset": round(imu_zero, 2),
        "arm_imu_sign": round(imu_sign, 3),
        "arm_claw_stop_us": int(_claw_stop_us()),
        "arm_rx_count": int(rx_count),
        "arm_hold_neutral": bool(hold_neutral),
        "arm_mavlink_ok": bool(_mavlink_up),
        "arm_manual_mode": bool(manual),
        "arm_preset_motion": bool(preset),
        "arm_joint_us": joint_us,
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


def _apply_arm_enable_cmd(cmd: dict) -> None:
    """Enable/disable all arm motion (tied to ROV disarmed mode on topside)."""
    global _arm_enabled, _manual_mode, _preset_motion, _last_pkt_time
    enabled = bool(cmd.get("enabled", False))
    with _lock:
        was_enabled = _arm_enabled
        _arm_enabled = enabled
        if not enabled:
            _manual_mode = False
            _preset_motion = False
        else:
            _preset_motion = False
            # Avoid hold-neutral stall after disarm or Pi restart handshake.
            if not was_enabled and (_rx_count > 0 or _last_pkt_time > 0):
                _last_pkt_time = time.time()
    print(f"[arm] Arm {'ENABLED' if enabled else 'DISABLED (disarmed)'}", flush=True)


def _apply_claw_hold_cmd(cmd: dict) -> None:
    """Enable/disable J6 IMU auto-level (rotation / claw hold)."""
    global _claw_hold_enabled
    if "enabled" not in cmd:
        return
    enabled = bool(cmd["enabled"])
    with _lock:
        _claw_hold_enabled = enabled
    mode = "MANUAL" if _j6_manual_mode(enabled) else "IMU HOLD"
    print(f"[arm] Claw hold {'ON' if enabled else 'OFF'} → J6 {mode}", flush=True)


def _apply_arm_imu_cal_cmd(cmd: dict) -> None:
    """Apply persisted sign/zero from topside config."""
    global _j6_imu_sign, _j6_imu_zero_offset
    with _lock:
        if "sign" in cmd:
            _j6_imu_sign = float(cmd["sign"])
        if "zero_offset_deg" in cmd:
            _j6_imu_zero_offset = float(cmd["zero_offset_deg"])
        sign = _j6_imu_sign
        zero = _j6_imu_zero_offset
    print(f"[arm] IMU cal → sign={sign:+.0f} zero={zero:.1f}°", flush=True)


def _apply_arm_claw_stop_cmd(cmd: dict) -> None:
    """Apply persisted claw stop PWM from topside config."""
    global _claw_stop_pwm
    stop = _clamp_us(cmd.get("stop_us", CLAW_STOP_US_DEFAULT))
    with _lock:
        _claw_stop_pwm = stop
    print(f"[arm] Claw stop PWM → {stop} µs", flush=True)


def _apply_arm_imu_zero_cmd(cmd: dict) -> None:
    """Set current wrist pose as 0° (updates zero offset)."""
    global _j6_imu_zero_offset
    raw = _read_j6_grav_deg()
    if raw is None:
        print("[arm] IMU zero failed — no BNO055 reading", flush=True)
        return
    with _lock:
        _j6_imu_zero_offset = raw
        zero = _j6_imu_zero_offset
    _poll_imu_cache(force=True)
    angle = _cached_j6_angle_deg()[0]
    print(
        f"[arm] IMU zero set → offset={zero:.1f}° (now {angle:.1f}°)",
        flush=True,
    )


def _apply_preset_motion_cmd(cmd: dict) -> None:
    """While enabled, ignore arm_sender UDP so preset steps are not overwritten."""
    global _preset_motion, _manual_mode, _preset_motion_since
    enabled = bool(cmd.get("enabled", False))
    with _lock:
        if not _arm_enabled and enabled:
            return
        _preset_motion = enabled
        _preset_motion_since = time.time() if enabled else 0.0
        if enabled:
            _manual_mode = False
    print(
        f"[arm] Preset motion {'ON — ignoring arm_sender UDP' if enabled else 'OFF'}",
        flush=True,
    )


def _maybe_clear_stale_preset_motion(now: float) -> None:
    """Drop preset lock if topside never finished the sequence."""
    global _preset_motion, _preset_motion_since
    with _lock:
        if not _preset_motion:
            return
        if _preset_motion_since <= 0:
            return
        if (now - _preset_motion_since) < PRESET_MOTION_TIMEOUT_SEC:
            return
        _preset_motion = False
        _preset_motion_since = 0.0
    print("[arm] Preset motion timeout — resuming arm_sender", flush=True)


def _apply_preset_step_cmd(cmd: dict) -> None:
    """Apply one preset step (single joint moved in working pose)."""
    global _joint_us, _j6_target_deg, _last_pkt_time, _preset_motion_since
    pwms = cmd.get("pwm")
    if not isinstance(pwms, list) or len(pwms) < 7:
        return
    with _lock:
        if not _arm_enabled:
            return
        _joint_us = [_clamp_us(pwms[i]) for i in range(7)]
        if "j6_angle" in cmd:
            _j6_target_deg = float(cmd["j6_angle"])
        _last_pkt_time = time.time()
        _preset_motion_since = time.time()

def _apply_manual_pwm_cmd(cmd: dict) -> None:
    """Handle manual AUX PWM commands from the web UI (overrides arm_sender UDP)."""
    global _manual_mode, _manual_aux_pwm

    with _lock:
        if not _arm_enabled:
            print("[arm] Manual AUX ignored — arm DISABLED (switch to DRIVE/ARMED)", flush=True)
            return

    if cmd.get("center"):
        with _lock:
            _manual_mode = True
            _manual_aux_pwm = _default_manual_aux_pwm()
        print(f"[arm] Manual AUX: all centered (AUX7 claw → {_claw_stop_us()} µs)", flush=True)
        return

    if "enabled" in cmd and cmd.get("aux") is None and cmd.get("joint") is None:
        enabled = bool(cmd.get("enabled"))
        with _lock:
            _manual_mode = enabled
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


def _handle_arm_control_json(cmd: dict, addr) -> None:
    """Handle JSON control commands from the web UI (manual AUX, presets, arm_enable)."""
    if not isinstance(cmd, dict) or not cmd.get("cmd"):
        return
    if cmd.get("cmd") == "preset_motion":
        _apply_preset_motion_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "preset_step":
        _apply_preset_step_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "manual_pwm":
        _apply_manual_pwm_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "arm_telemetry" and cmd.get("subscribe"):
        port = int(cmd.get("port", ARM_TELEM_PORT))
        _note_telemetry_subscriber(addr[0], port)
        print(f"[arm] Arm telemetry → {addr[0]}:{port}", flush=True)
        _send_arm_telemetry(J6_OUT_CENTER)
    elif cmd.get("cmd") == "claw_hold":
        _apply_claw_hold_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "arm_imu_cal":
        _apply_arm_imu_cal_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "arm_claw_stop":
        _apply_arm_claw_stop_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "arm_imu_zero":
        _apply_arm_imu_zero_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "arm_enable":
        _apply_arm_enable_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)


def _arm_control_listener():
    """Background thread: manual AUX PWM / preset / IMU JSON from the web UI."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", ARM_CONTROL_PORT))
        s.settimeout(1.0)
    except Exception as e:
        print(f"[arm] Control listener bind failed on UDP {ARM_CONTROL_PORT}: {e}", flush=True)
        print(
            "[arm] FATAL: manual AUX / presets unavailable — restart arm after mosfet_service "
            f"(port {ARM_CONTROL_PORT} must be free)",
            flush=True,
        )
        return
    print(f"[arm] Control JSON on UDP {ARM_CONTROL_PORT}", flush=True)
    while True:
        try:
            data, addr = s.recvfrom(512)
            cmd = json.loads(data.decode())
            if cmd.get("cmd") == "preset_motion":
                _apply_preset_motion_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "preset_step":
                _apply_preset_step_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "manual_pwm":
                _apply_manual_pwm_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "arm_telemetry" and cmd.get("subscribe"):
                port = int(cmd.get("port", ARM_TELEM_PORT))
                _note_telemetry_subscriber(addr[0], port)
                print(f"[arm] Arm telemetry → {addr[0]}:{port}", flush=True)
                _send_arm_telemetry(J6_OUT_CENTER)
            elif cmd.get("cmd") == "claw_hold":
                _apply_claw_hold_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "arm_imu_cal":
                _apply_arm_imu_cal_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "arm_claw_stop":
                _apply_arm_claw_stop_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "arm_imu_zero":
                _apply_arm_imu_zero_cmd(cmd)
                _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            elif cmd.get("cmd") == "arm_enable":
                _apply_arm_enable_cmd(cmd)
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
        if not _arm_enabled:
            rc = [IGNORE] * 18
            _fill_rc_neutral(rc)
            return rc
        manual = _manual_mode
        if manual:
            aux_vals = list(_manual_aux_pwm)
    if manual:
        return _build_rc_manual(aux_vals)

    rc = [IGNORE] * 18
    with _lock:
        joint_us_snap   = list(_joint_us)
        j6_target_snap  = _j6_target_deg
        last_pkt_snap   = _last_pkt_time
        claw_hold_snap  = _claw_hold_enabled

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

    # J6 continuous rotation (index 5 = J6 stick input from arm_sender)
    rc[J6_RC_CH - 1] = _compute_j6_pwm(
        _clamp_us(joint_us_snap[5]), j6_target_snap, claw_hold_snap
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
        _request_aux_servo_stream(master)
        return master
    except Exception as e:
        print(f"[arm] MAVLink unavailable ({e}) — will retry", flush=True)
        return None


def _request_aux_servo_stream(master) -> None:
    """Ask FC for AUX RC + PWM output so we can detect override pipeline stalls."""
    try:
        ts = master.target_system or 1
        tc = master.target_component or 1
        for msg_id, interval_us in ((36, 500_000), (65, 500_000)):
            master.mav.command_long_send(
                ts, tc,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval_us, 0, 0, 0, 0, 0,
            )
    except Exception:
        pass


def _mavlink_connect_loop() -> None:
    """Background MAVLink connect/reconnect — keeps the main loop responsive."""
    global _mav_master, _mavlink_up, _mav_connecting
    while True:
        with _mav_lock:
            connected = _mav_master is not None
        if connected:
            time.sleep(2.0)
            continue
        if _mav_connecting:
            time.sleep(0.5)
            continue
        _mav_connecting = True
        try:
            master = _try_connect_mavlink()
            if master is not None:
                with _mav_lock:
                    _mav_master = master
                print("[arm] MAVLink link up — RC override active", flush=True)
            else:
                time.sleep(5.0)
        finally:
            _mav_connecting = False


def _get_mavlink_master():
    with _mav_lock:
        return _mav_master


def _drop_mavlink_master(reason: str = "") -> None:
    global _mav_master, _mavlink_up
    with _mav_lock:
        master = _mav_master
        _mav_master = None
    _mavlink_up = False
    if master is not None:
        try:
            master.close()
        except Exception:
            pass
    if reason:
        print(f"[arm] MAVLink dropped ({reason}) — will retry", flush=True)


def _drain_mavlink(master) -> dict:
    """Drain inbound MAVLink (required so TCP writes do not stall)."""
    fc_rc: dict = {}
    fc_srv: dict = {}
    if master is None:
        return {"fc_rc": fc_rc, "fc_srv": fc_srv}
    for _ in range(80):
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "RC_CHANNELS":
            for rc_ch in range(9, 17):
                fc_rc[rc_ch] = getattr(msg, f"chan{rc_ch}_raw", 0)
        elif t == "SERVO_OUTPUT_RAW":
            for rc_ch in range(9, 17):
                fc_srv[rc_ch] = getattr(msg, f"servo{rc_ch}_raw", 0)
    return {"fc_rc": fc_rc, "fc_srv": fc_srv}


def _maybe_warn_servo_mismatch(rc: list, fc_rc: dict, fc_srv: dict, hold_neutral: bool) -> None:
    """Log when FC is not applying our AUX RC override (Mission Planner / MAVLink)."""
    if hold_neutral:
        return
    targets = []
    for joint_idx, rc_ch in JOINT_TO_RC_CH.items():
        val = rc[rc_ch - 1]
        if val == IGNORE or abs(val - _neutral_pwm_for_rc_ch(rc_ch)) <= J6_IN_DEADBAND:
            continue
        targets.append((rc_ch, int(val)))
    if not targets or not fc_rc:
        return
    rc_ok = all(abs(fc_rc.get(ch, 0) - us) < 25 for ch, us in targets)
    if not rc_ok:
        print(
            "[arm] *** AUX RC override not reaching FC — check MAVProxy tcp:5763 ***",
            flush=True,
        )
        return
    if not fc_srv:
        return
    srv_ok = all(abs(fc_srv.get(ch, 0) - us) < 25 for ch, us in targets)
    if not srv_ok:
        print(
            "[arm] *** FC RC changed but AUX servo output did not — "
            "set SERVO9-16_FUNCTION=1 (RCPassThru), BRD_SAFETYENABLE=0 ***",
            flush=True,
        )


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global _joint_us, _j6_target_deg, _last_pkt_time, _rx_count, _mavlink_up

    print("[arm] Arm controller starting...", flush=True)
    threading.Thread(
        target=_init_optional_hardware, daemon=True, name="arm-hw-init",
    ).start()
    _start_imu_poll_thread()

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
    threading.Thread(
        target=_mavlink_connect_loop, daemon=True, name="arm-mavlink",
    ).start()

    print(f"[arm] Listening on UDP {UDP_PORT}", flush=True)
    print(f"[arm] AUX1=J5  AUX2=J2  AUX3=J6  AUX4=J1  AUX5=J3  AUX6=J4  AUX7=Claw", flush=True)
    print(f"[arm] BNO055={'yes' if HAVE_BNO else 'pending'}  MAVLink=connecting", flush=True)

    last_send      = 0.0
    last_heartbeat = 0.0
    last_print     = 0.0
    last_telem     = 0.0
    last_j6_pwm    = J6_OUT_CENTER
    last_servo_diag = 0.0
    SERVO_DIAG_SEC = 3.0
    last_fc_rc: dict = {}
    last_fc_srv: dict = {}

    try:
        while True:
            now = time.time()
            _maybe_clear_stale_preset_motion(now)

            master = _get_mavlink_master()
            _mavlink_up = master is not None
            if master is not None:
                drained = _drain_mavlink(master)
                if drained["fc_rc"]:
                    last_fc_rc = drained["fc_rc"]
                if drained["fc_srv"]:
                    last_fc_srv = drained["fc_srv"]

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
                        elif _preset_motion:
                            pass  # preset sequence active — ignore arm_sender
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

            rc = None

            # ── Send RC_CHANNELS_OVERRIDE at OVERRIDE_HZ ─────────────────────
            if master is not None and now - last_send >= 1.0 / OVERRIDE_HZ:
                last_send = now
                rc = _build_rc_array()
                try:
                    _send_rc_override(master, rc)
                except OSError as e:
                    _drop_mavlink_master(str(e))
                    master = None
                else:
                    if rc[J6_RC_CH - 1] != IGNORE:
                        last_j6_pwm = int(rc[J6_RC_CH - 1])

            if master is not None and now - last_heartbeat >= 1.0:
                last_heartbeat = now
                try:
                    _send_heartbeat(master)
                except OSError as e:
                    _drop_mavlink_master(str(e))
                    master = None

            if (
                master is not None
                and rc is not None
                and now - last_servo_diag >= SERVO_DIAG_SEC
            ):
                last_servo_diag = now
                with _lock:
                    lpt = _last_pkt_time
                _maybe_warn_servo_mismatch(
                    rc, last_fc_rc, last_fc_srv, _should_hold_neutral(lpt)
                )

            if now - last_telem >= 1.0 / ARM_TELEM_HZ:
                last_telem = now
                _send_arm_telemetry(last_j6_pwm)

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
                        f"[arm] MANUAL | {aux_str}",
                        flush=True,
                    )
                else:
                    hold_neutral = _should_hold_neutral(lpt)
                    with _lock:
                        claw_hold = _claw_hold_enabled
                        armed = _arm_enabled
                    j6_manual = _j6_manual_mode(claw_hold) if not hold_neutral else True
                    j6_pwm = (
                        J6_OUT_CENTER
                        if hold_neutral
                        else _compute_j6_pwm(_clamp_us(jus[5]), j6t, claw_hold)
                    )
                    print(
                        f"[arm] rx={rx} armed={armed} mav={'OK' if _mavlink_up else 'DOWN'} "
                        f"hold={hold_neutral} claw={claw_hold} "
                        f"j6={'MAN' if j6_manual else 'IMU'} | "
                        f"J1={jus[0]} J2={jus[1]} J3={jus[2]} J4={jus[3]} "
                        f"J5={jus[4]} J6={j6_pwm}(in={jus[5]},tgt={j6t:.1f}) "
                        f"Claw={jus[6]}",
                        flush=True,
                    )

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n[arm] Stopping — centering all AUX channels.", flush=True)

    finally:
        master = _get_mavlink_master()
        if master is not None:
            rc = [IGNORE] * 18
            _fill_rc_neutral(rc)
            try:
                _send_rc_override(master, rc)
            except Exception:
                pass
            time.sleep(0.2)

        _drop_mavlink_master()

        _release_bno()
        print("[arm] Done.", flush=True)


if __name__ == "__main__":
    main()
