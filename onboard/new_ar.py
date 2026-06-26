#!/usr/bin/env python3
"""
Arm controller — ONBOARD  (runs on the Raspberry Pi)
=====================================================
Receives joint PWM commands from arm_sender.py over UDP and forwards
them to the Pixhawk 6 AUX outputs via MAVLink RC_CHANNELS_OVERRIDE
through MAVProxy.

Physical arm (4 DOF): J1, J2, J3, Claw
    AUX4 (RC ch 12) → J1
    AUX1 (RC ch  9) → J2
    AUX3 (RC ch 11) → J3
    AUX7 (RC ch 15) → Claw (continuous rotation; stop PWM configurable)
    AUX2, AUX5, AUX6 — removed joints, held at neutral
    AUX8 (RC ch 16) → spare (always 1500)

arm_sender still transmits 7 joint PWM values (+ optional angle field).
Indices 1–3 (old J2–J4) and the angle field are ignored.

Incoming UDP packet (comma-separated):
    J1, J2, J3, J4, J5, J6_PWM, Claw [, angle]
    index: 0    1    2    3    4     5      6      7 (ignored)
    Active CSV indices: 0→J1, 4→J2, 5→J3, 6→Claw

Manual AUX PWM (web UI, JSON on UDP port 5006 or 5009):
    {"cmd": "manual_pwm", "enabled": true}
    {"cmd": "manual_pwm", "aux": 4, "pwm": 1500}
    {"cmd": "manual_pwm", "center": true}

Thruster manual PWM is handled by stabilization.py (UDP 5005).
"""

import json
import socket
import threading
import time

from pymavlink import mavutil

from mavlink_rc import MAVLINK_ONBOARD_ARM, connect_mavlink, send_rc_channels_override, wait_for_heartbeat

# ── Config ────────────────────────────────────────────────────────────────────
UDP_PORT    = 5006
MAVLINK_URL = MAVLINK_ONBOARD_ARM
ARM_CONTROL_PORT = 5009
CENTER_US   = 1500
MIN_US      = 500
MAX_US      = 2500
IGNORE      = 65535
OVERRIDE_HZ = 20
PRINT_HZ    = 2
ARM_TELEM_HZ = 5
ARM_TELEM_PORT = 5008
TIMEOUT_SEC = 0.75

# arm_sender CSV index → RC channel (AUX1=ch9 …)
CSV_TO_RC_CH = {
    0: 12,   # J1  → AUX4
    4:  9,   # J2  → AUX1 (was J5)
    5: 11,   # J3  → AUX3 (was J6)
    6: 15,   # Claw → AUX7
}
CLAW_CSV_IDX = 6
CLAW_RC_CH   = 15
SPARE_RC_CH  = 16
REMOVED_RC_CHS = (10, 13, 14)  # AUX2, AUX5, AUX6 — no hardware

CLAW_MIN_US = 1325
CLAW_MAX_US = 1525
CLAW_STOP_US_DEFAULT = 1425
CLAW_IN_DEADBAND = 10
_claw_stop_pwm = CLAW_STOP_US_DEFAULT

AUX_LABELS = ("J2", "—", "J3", "J1", "—", "—", "Claw")
JOINT_TO_AUX = {1: 4, 2: 1, 3: 3, 4: 7}  # physical J1–J3, Claw → AUX port


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _clamp_us(x):
    return int(_clamp(float(x), MIN_US, MAX_US))


def _clamp_claw_us(x):
    return int(_clamp(float(x), CLAW_MIN_US, CLAW_MAX_US))


def _claw_stop_us() -> int:
    with _lock:
        return int(_claw_stop_pwm)


def _claw_output_pwm(claw_input_us):
    """Claw continuous rotation — centered stick/input → configured stop PWM."""
    us = _clamp_claw_us(claw_input_us)
    if abs(us - CENTER_US) <= CLAW_IN_DEADBAND:
        return _claw_stop_us()
    return us


def _default_joint_us():
    vals = [CENTER_US] * 7
    vals[CLAW_CSV_IDX] = _claw_stop_us()
    return vals


def _default_manual_aux_pwm():
    """Neutral PWM for AUX1–7 in manual mode."""
    return [CENTER_US, CENTER_US, CENTER_US, CENTER_US, CENTER_US, CENTER_US, _claw_stop_us()]


def _neutral_pwm_for_rc_ch(rc_ch: int) -> int:
    if rc_ch == CLAW_RC_CH:
        return _claw_stop_us()
    return CENTER_US


def _fill_rc_neutral(rc: list) -> None:
    for rc_ch in CSV_TO_RC_CH.values():
        rc[rc_ch - 1] = _neutral_pwm_for_rc_ch(rc_ch)
    for rc_ch in REMOVED_RC_CHS:
        rc[rc_ch - 1] = CENTER_US
    rc[SPARE_RC_CH - 1] = CENTER_US


def _should_hold_neutral(last_pkt_time: float) -> bool:
    if last_pkt_time <= 0:
        return True
    return time.time() - last_pkt_time > TIMEOUT_SEC


def _pwm_for_csv_index(joint_us: list, csv_idx: int) -> int:
    if csv_idx == CLAW_CSV_IDX:
        return _claw_output_pwm(joint_us[csv_idx])
    return _clamp_us(joint_us[csv_idx])


# ── Shared state ──────────────────────────────────────────────────────────────
_lock = threading.Lock()
_joint_us = _default_joint_us()
_last_pkt_time = 0.0
_rx_count = 0
_manual_mode = False
_manual_aux_pwm = _default_manual_aux_pwm()
_arm_enabled = True
_mavlink_up = False
_mav_master = None
_mav_lock = threading.Lock()
_mav_connecting = False
_preset_motion = False
_preset_motion_since = 0.0
PRESET_MOTION_TIMEOUT_SEC = 45.0
_telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_telem_subscribers: set = set()
_telem_sub_lock = threading.Lock()
_telem_send_failures: dict = {}


def _note_telemetry_subscriber(host: str, port: int) -> None:
    with _telem_sub_lock:
        _telem_subscribers.add((host, int(port)))


def _send_arm_telemetry() -> None:
    with _lock:
        rx_count = _rx_count
        last_pkt = _last_pkt_time
        joint_us = list(_joint_us)
        armed = _arm_enabled
        manual = _manual_mode
        preset = _preset_motion
    hold_neutral = _should_hold_neutral(last_pkt)

    payload = json.dumps({
        "type": "arm",
        "arm_enabled": bool(armed),
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
    global _arm_enabled, _manual_mode, _preset_motion, _last_pkt_time
    enabled = bool(cmd.get("enabled", False))
    with _lock:
        _arm_enabled = enabled
        if not enabled:
            _manual_mode = False
            _preset_motion = False
        else:
            _preset_motion = False
            _last_pkt_time = time.time()
    print(f"[arm] Arm {'ENABLED' if enabled else 'DISABLED (disarmed)'}", flush=True)


def _apply_arm_claw_stop_cmd(cmd: dict) -> None:
    global _claw_stop_pwm
    stop = _clamp_claw_us(cmd.get("stop_us", CLAW_STOP_US_DEFAULT))
    with _lock:
        _claw_stop_pwm = stop
    print(f"[arm] Claw stop PWM → {stop} µs", flush=True)


def _apply_preset_motion_cmd(cmd: dict) -> None:
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
    global _joint_us, _last_pkt_time, _preset_motion_since
    pwms = cmd.get("pwm")
    if not isinstance(pwms, list) or len(pwms) < 7:
        return
    with _lock:
        if not _arm_enabled:
            return
        _joint_us = [_clamp_us(pwms[i]) for i in range(7)]
        _last_pkt_time = time.time()
        _preset_motion_since = time.time()


def _apply_manual_pwm_cmd(cmd: dict) -> None:
    global _manual_mode, _manual_aux_pwm, _arm_enabled

    turning_off = (
        "enabled" in cmd
        and not cmd.get("enabled")
        and cmd.get("aux") is None
        and cmd.get("joint") is None
        and not cmd.get("center")
    )
    has_pwm = (
        cmd.get("center")
        or cmd.get("aux") is not None
        or cmd.get("joint") is not None
    )

    # Topside only sends manual PWM while DRIVE/ARMED — recover if arm_enable UDP was lost.
    if has_pwm or (cmd.get("enabled") and not turning_off):
        with _lock:
            if not _arm_enabled:
                _arm_enabled = True
                print("[arm] Arm auto-enabled for manual AUX command", flush=True)

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
        pwm_i = _clamp_claw_us(pwm) if aux_i == 7 else _clamp_us(pwm)
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
        _flush_rc_override_now()
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "arm_telemetry" and cmd.get("subscribe"):
        port = int(cmd.get("port", ARM_TELEM_PORT))
        _note_telemetry_subscriber(addr[0], port)
        print(f"[arm] Arm telemetry → {addr[0]}:{port}", flush=True)
        _send_arm_telemetry()
    elif cmd.get("cmd") == "arm_claw_stop":
        _apply_arm_claw_stop_cmd(cmd)
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
    elif cmd.get("cmd") == "arm_enable":
        _apply_arm_enable_cmd(cmd)
        _flush_rc_override_now()
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)


def _arm_control_listener():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", ARM_CONTROL_PORT))
        s.settimeout(1.0)
    except Exception as e:
        print(f"[arm] Control listener bind failed on UDP {ARM_CONTROL_PORT}: {e}", flush=True)
        return
    print(f"[arm] Control JSON on UDP {ARM_CONTROL_PORT}", flush=True)
    while True:
        try:
            data, addr = s.recvfrom(512)
            cmd = json.loads(data.decode())
            _handle_arm_control_json(cmd, addr)
        except socket.timeout:
            pass
        except json.JSONDecodeError as e:
            print(f"[arm] Control listener bad JSON: {e}", flush=True)
        except Exception as e:
            print(f"[arm] Control listener error: {e}", flush=True)


def _send_rc_override(master, rc):
    send_rc_channels_override(master, rc, ignore=IGNORE)


def _build_rc_manual(aux_vals=None):
    rc = [IGNORE] * 18
    if aux_vals is None:
        with _lock:
            aux_vals = list(_manual_aux_pwm)
    for aux_i in range(1, 8):
        val = aux_vals[aux_i - 1]
        rc[8 + aux_i - 1] = _clamp_claw_us(val) if aux_i == 7 else _clamp_us(val)
    rc[SPARE_RC_CH - 1] = CENTER_US
    return rc


def _build_rc_array():
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
        joint_us_snap = list(_joint_us)
        last_pkt_snap = _last_pkt_time

    if _should_hold_neutral(last_pkt_snap):
        _fill_rc_neutral(rc)
        return rc

    for csv_idx, rc_ch in CSV_TO_RC_CH.items():
        rc[rc_ch - 1] = _pwm_for_csv_index(joint_us_snap, csv_idx)
    for rc_ch in REMOVED_RC_CHS:
        rc[rc_ch - 1] = CENTER_US
    rc[SPARE_RC_CH - 1] = CENTER_US
    return rc


def _send_override(master):
    _send_rc_override(master, _build_rc_array())


def _flush_rc_override_now() -> None:
    """Push RC override immediately after a manual/control command."""
    master = _get_mavlink_master()
    if master is None:
        return
    try:
        _send_override(master)
    except OSError as e:
        _drop_mavlink_master(str(e))


def _send_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def _try_connect_mavlink():
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
    if hold_neutral:
        return
    targets = []
    for rc_ch in CSV_TO_RC_CH.values():
        val = rc[rc_ch - 1]
        if val == IGNORE or abs(val - _neutral_pwm_for_rc_ch(rc_ch)) <= CLAW_IN_DEADBAND:
            continue
        targets.append((rc_ch, int(val)))
    if not targets or not fc_rc:
        return
    rc_ok = all(abs(fc_rc.get(ch, 0) - us) < 25 for ch, us in targets)
    if not rc_ok:
        print("[arm] *** AUX RC override not reaching FC — check MAVProxy tcp:5763 ***", flush=True)
        return
    if not fc_srv:
        return
    srv_ok = all(abs(fc_srv.get(ch, 0) - us) < 25 for ch, us in targets)
    if not srv_ok:
        print(
            "[arm] *** FC RC changed but AUX servo output did not — "
            "set SERVO9=59 SERVO11=61 SERVO12=62 SERVO15=65 (RCPassThru), BRD_SAFETYENABLE=0 ***",
            flush=True,
        )


def main():
    global _joint_us, _last_pkt_time, _rx_count, _mavlink_up

    print("[arm] Arm controller starting...", flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", UDP_PORT))
    except OSError as e:
        print(f"[arm] FATAL: cannot bind UDP {UDP_PORT}: {e}", flush=True)
        raise
    sock.settimeout(0.001)

    threading.Thread(target=_arm_control_listener, daemon=True).start()
    threading.Thread(target=_mavlink_connect_loop, daemon=True, name="arm-mavlink").start()

    print(f"[arm] Listening on UDP {UDP_PORT} (CSV arm + JSON control)", flush=True)
    print("[arm] AUX4=J1  AUX1=J2  AUX3=J3  AUX7=Claw  (AUX2/5/6 removed)", flush=True)
    print("[arm] MAVLink=connecting", flush=True)

    last_send = 0.0
    last_heartbeat = 0.0
    last_print = 0.0
    last_telem = 0.0
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

            try:
                data, addr = sock.recvfrom(1024)
                text = data.decode(errors="ignore").strip()
                if not text:
                    continue
                if text.startswith("{"):
                    try:
                        cmd = json.loads(text)
                        _handle_arm_control_json(cmd, addr)
                        continue
                    except json.JSONDecodeError:
                        pass
                line = text
                if line.startswith("PWM:"):
                    line = line[4:]
                parts = line.split(",")
                if len(parts) >= 7:
                    with _lock:
                        if _manual_mode:
                            pass
                        elif _preset_motion:
                            pass
                        else:
                            vals = [float(x) for x in parts]
                            _joint_us = [_clamp_us(vals[i]) for i in range(7)]
                            _last_pkt_time = now
                            _rx_count += 1
                    _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            except socket.timeout:
                pass
            except (ValueError, IndexError):
                pass

            rc = None

            if master is not None and now - last_send >= 1.0 / OVERRIDE_HZ:
                last_send = now
                rc = _build_rc_array()
                try:
                    _send_rc_override(master, rc)
                except OSError as e:
                    _drop_mavlink_master(str(e))
                    master = None

            if master is not None and now - last_heartbeat >= 1.0:
                last_heartbeat = now
                try:
                    _send_heartbeat(master)
                except OSError as e:
                    _drop_mavlink_master(str(e))
                    master = None

            if master is not None and rc is not None and now - last_servo_diag >= SERVO_DIAG_SEC:
                last_servo_diag = now
                with _lock:
                    lpt = _last_pkt_time
                _maybe_warn_servo_mismatch(rc, last_fc_rc, last_fc_srv, _should_hold_neutral(lpt))

            if now - last_telem >= 1.0 / ARM_TELEM_HZ:
                last_telem = now
                _send_arm_telemetry()

            if now - last_print >= 1.0 / PRINT_HZ:
                last_print = now
                with _lock:
                    rx = _rx_count
                    lpt = _last_pkt_time
                    jus = list(_joint_us)
                    manual = _manual_mode
                    aux_pwm = list(_manual_aux_pwm)
                    armed = _arm_enabled
                if manual:
                    aux_str = " ".join(f"A{i+1}={aux_pwm[i]}" for i in range(7))
                    print(f"[arm] MANUAL | {aux_str}", flush=True)
                else:
                    hold_neutral = _should_hold_neutral(lpt)
                    j1 = _pwm_for_csv_index(jus, 0) if not hold_neutral else _neutral_pwm_for_rc_ch(12)
                    j2 = _pwm_for_csv_index(jus, 4) if not hold_neutral else _neutral_pwm_for_rc_ch(9)
                    j3 = _pwm_for_csv_index(jus, 5) if not hold_neutral else _neutral_pwm_for_rc_ch(11)
                    claw = _pwm_for_csv_index(jus, 6) if not hold_neutral else _claw_stop_us()
                    print(
                        f"[arm] rx={rx} armed={armed} mav={'OK' if _mavlink_up else 'DOWN'} "
                        f"hold={hold_neutral} | "
                        f"J1={j1} J2={j2} J3={j3} Claw={claw} "
                        f"(in J1={jus[0]} J2={jus[4]} J3={jus[5]} Claw={jus[6]})",
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
        print("[arm] Done.", flush=True)


if __name__ == "__main__":
    main()
