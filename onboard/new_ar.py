#!/usr/bin/env python3
"""
Arm controller — ONBOARD  (runs on the Raspberry Pi)
=====================================================
Receives joint PWM over UDP and forwards to Pix6 AUX outputs via
MAVLink RC_CHANNELS_OVERRIDE (same path as test/arm_test_onboard.py).

Physical arm (4 DOF):
    J1 / M13  AUX5  RC ch 13   500–2350 µs, neutral 1400
    J2 / M9   AUX1  RC ch  9   950–2200 µs, neutral 1600
    J3 / M11  AUX3  RC ch 11   1300–1700 µs, stop 1500 (continuous)
    Claw/M15  AUX7  RC ch 15   open 1325, close 1525, stop 1425 (continuous)

UDP inputs (port 5006, also 5009 for legacy control):
    Test / direct:  {"joint": 1, "pwm": 1400}  |  {"center_all": true}
    arm_sender CSV: 1400,1500,1500,1500,1600,1500,1425
    Web UI JSON:    {"cmd": "manual_pwm", "joint": 2, "pwm": 1600}
                    {"cmd": "preset_step", "pwm": [7 values]}
                    {"cmd": "arm_enable", "enabled": true}
"""

from __future__ import annotations

import json
import socket
import threading
import time

from pymavlink import mavutil

from arm_joints import (
    CLAW_STOP_US_DEFAULT,
    NUM_JOINTS,
    RC_IGNORE,
    build_rc_override,
    clamp_joint_pwm,
    csv_list_to_joint_pwm,
    default_joint_pwm,
    joint_pwm_to_csv_list,
)
from mavlink_rc import MAVLINK_ONBOARD_ARM, connect_mavlink, send_rc_channels_override, wait_for_heartbeat

UDP_PORT = 5006
ARM_CONTROL_PORT = 5009
MAVLINK_URL = MAVLINK_ONBOARD_ARM
OVERRIDE_HZ = 20
PRINT_HZ = 2
ARM_TELEM_HZ = 5
ARM_TELEM_PORT = 5006
TIMEOUT_SEC = 0.75
PRESET_MOTION_TIMEOUT_SEC = 45.0
SERVO_DIAG_SEC = 3.0
NEUTRAL_DEADBAND_US = 10

_lock = threading.Lock()
_joint_pwm = default_joint_pwm()
_claw_stop_pwm = CLAW_STOP_US_DEFAULT
_last_pkt_time = 0.0
_rx_count = 0
_arm_enabled = True
_manual_mode = False
_preset_motion = False
_preset_motion_since = 0.0
_mavlink_up = False
_mav_master = None
_mav_lock = threading.Lock()
_mav_connecting = False

_telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_telem_subscribers: set[tuple[str, int]] = set()
_telem_sub_lock = threading.Lock()
_telem_send_failures: dict[tuple[str, int], int] = {}


def _should_hold_neutral(last_pkt_time: float) -> bool:
    if last_pkt_time <= 0:
        return True
    return time.time() - last_pkt_time > TIMEOUT_SEC


def _neutral_pwm() -> dict[int, int]:
    with _lock:
        claw = int(_claw_stop_pwm)
    return default_joint_pwm(claw_stop=claw)


def _apply_joint_pwm(updates: dict[int, int]) -> None:
    global _joint_pwm, _last_pkt_time
    with _lock:
        for joint, us in updates.items():
            if 1 <= joint <= NUM_JOINTS:
                _joint_pwm[joint] = clamp_joint_pwm(joint, us, claw_stop=_claw_stop_pwm)
        _last_pkt_time = time.time()


def _center_all_joints() -> None:
    global _joint_pwm, _last_pkt_time
    with _lock:
        _joint_pwm = default_joint_pwm(claw_stop=int(_claw_stop_pwm))
        _last_pkt_time = time.time()


def _build_rc_array() -> list[int]:
    with _lock:
        if not _arm_enabled:
            return build_rc_override(_neutral_pwm(), ignore=RC_IGNORE)
        manual = _manual_mode
        preset = _preset_motion
        pwm = dict(_joint_pwm)
        last_pkt = _last_pkt_time
        claw = int(_claw_stop_pwm)

    if manual or preset:
        return build_rc_override(pwm, ignore=RC_IGNORE)

    if _should_hold_neutral(last_pkt):
        return build_rc_override(default_joint_pwm(claw_stop=claw), ignore=RC_IGNORE)

    return build_rc_override(pwm, ignore=RC_IGNORE)


def _note_telemetry_subscriber(host: str, port: int) -> None:
    with _telem_sub_lock:
        _telem_subscribers.add((host, int(port)))


def _send_arm_telemetry() -> None:
    with _lock:
        rx_count = _rx_count
        last_pkt = _last_pkt_time
        pwm = dict(_joint_pwm)
        armed = _arm_enabled
        manual = _manual_mode
        preset = _preset_motion
        claw = int(_claw_stop_pwm)

    hold_neutral = False if manual or preset else _should_hold_neutral(last_pkt)
    payload = json.dumps({
        "type": "arm",
        "arm_enabled": bool(armed),
        "arm_claw_stop_us": claw,
        "arm_rx_count": int(rx_count),
        "arm_hold_neutral": bool(hold_neutral),
        "arm_mavlink_ok": bool(_mavlink_up),
        "arm_manual_mode": bool(manual),
        "arm_preset_motion": bool(preset),
        "arm_joint_us": joint_pwm_to_csv_list(pwm),
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


def _flush_rc_override_now() -> None:
    master = _get_mavlink_master()
    if master is None:
        return
    try:
        send_rc_channels_override(master, _build_rc_array(), ignore=RC_IGNORE)
    except OSError as e:
        _drop_mavlink_master(str(e))


def _handle_direct_joint_cmd(cmd: dict) -> bool:
    """Test-script format: {joint, pwm} or {center_all: true}."""
    if cmd.get("center_all"):
        _center_all_joints()
        print("[arm] ALL CENTER", flush=True)
        return True

    if "joint" not in cmd:
        return False

    try:
        joint = int(cmd["joint"])
        pwm = int(cmd.get("pwm", 1500))
    except (TypeError, ValueError):
        return False

    if not (1 <= joint <= NUM_JOINTS):
        return False

    _apply_joint_pwm({joint: pwm})
    print(f"[arm] joint {joint} → {pwm} µs", flush=True)
    return True


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
    print(f"[arm] Arm {'ENABLED' if enabled else 'DISABLED'}", flush=True)


def _apply_arm_claw_stop_cmd(cmd: dict) -> None:
    global _claw_stop_pwm
    stop = clamp_joint_pwm(4, cmd.get("stop_us", CLAW_STOP_US_DEFAULT))
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
    print(f"[arm] Preset motion {'ON' if enabled else 'OFF'}", flush=True)


def _apply_preset_step_cmd(cmd: dict) -> None:
    global _joint_pwm, _last_pkt_time, _preset_motion_since
    pwms = cmd.get("pwm")
    if not isinstance(pwms, list) or len(pwms) < 7:
        return
    with _lock:
        if not _arm_enabled:
            return
        _joint_pwm = csv_list_to_joint_pwm(pwms, claw_stop=int(_claw_stop_pwm))
        _last_pkt_time = time.time()
        _preset_motion_since = time.time()


def _apply_manual_pwm_cmd(cmd: dict) -> None:
    global _manual_mode, _arm_enabled

    turning_off = (
        "enabled" in cmd
        and not cmd.get("enabled")
        and cmd.get("aux") is None
        and cmd.get("joint") is None
        and not cmd.get("center")
    )
    has_pwm = cmd.get("center") or cmd.get("aux") is not None or cmd.get("joint") is not None

    if has_pwm or (cmd.get("enabled") and not turning_off):
        with _lock:
            if not _arm_enabled:
                _arm_enabled = True
                print("[arm] Arm auto-enabled for manual command", flush=True)

    with _lock:
        if not _arm_enabled:
            print("[arm] Manual ignored — arm DISABLED", flush=True)
            return

    if cmd.get("center"):
        with _lock:
            _manual_mode = True
        _center_all_joints()
        print("[arm] Manual: all joints centered", flush=True)
        return

    if "enabled" in cmd and cmd.get("aux") is None and cmd.get("joint") is None:
        with _lock:
            _manual_mode = bool(cmd.get("enabled"))
            manual_on = _manual_mode
        print(f"[arm] Manual mode {'ON' if manual_on else 'OFF'}", flush=True)
        return

    joint = cmd.get("joint")
    if joint is None and cmd.get("aux") is not None:
        from arm_joints import AUX_TO_JOINT
        try:
            joint = AUX_TO_JOINT.get(int(cmd["aux"]))
        except (TypeError, ValueError):
            joint = None

    pwm = cmd.get("pwm")
    if joint is None or pwm is None:
        return

    try:
        joint_i = int(joint)
        pwm_i = int(pwm)
    except (TypeError, ValueError):
        return

    if not (1 <= joint_i <= NUM_JOINTS):
        return

    with _lock:
        _manual_mode = True
    _apply_joint_pwm({joint_i: pwm_i})
    print(f"[arm] Manual joint {joint_i} → {pwm_i} µs", flush=True)


def _handle_arm_control_json(cmd: dict, addr) -> None:
    if not isinstance(cmd, dict):
        return

    if _handle_direct_joint_cmd(cmd):
        _flush_rc_override_now()
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
        return

    name = cmd.get("cmd")
    if name == "preset_motion":
        _apply_preset_motion_cmd(cmd)
    elif name == "preset_step":
        _apply_preset_step_cmd(cmd)
    elif name == "manual_pwm":
        _apply_manual_pwm_cmd(cmd)
        _flush_rc_override_now()
    elif name == "arm_telemetry" and cmd.get("subscribe"):
        _note_telemetry_subscriber(str(addr[0]).strip(), int(cmd.get("port", ARM_TELEM_PORT)))
        _send_arm_telemetry()
    elif name == "arm_claw_stop":
        _apply_arm_claw_stop_cmd(cmd)
    elif name == "arm_enable":
        _apply_arm_enable_cmd(cmd)
        _flush_rc_override_now()
    else:
        return

    _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)


def _arm_control_listener() -> None:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", ARM_CONTROL_PORT))
        s.settimeout(1.0)
    except OSError as e:
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
            print(f"[arm] Control bad JSON: {e}", flush=True)
        except Exception as e:
            print(f"[arm] Control error: {e}", flush=True)


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
        for msg_id, interval_us in ((36, 500_000), (65, 500_000)):
            try:
                master.mav.command_long_send(
                    master.target_system or 1,
                    master.target_component or 1,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, msg_id, interval_us, 0, 0, 0, 0, 0,
                )
            except Exception:
                pass
        return master
    except Exception as e:
        print(f"[arm] MAVLink unavailable ({e}) — will retry", flush=True)
        return None


def _mavlink_connect_loop() -> None:
    global _mav_master, _mav_connecting
    while True:
        with _mav_lock:
            if _mav_master is not None:
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
                print("[arm] MAVLink link up", flush=True)
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


def _drain_mavlink(master) -> tuple[dict, dict]:
    fc_rc: dict[int, int] = {}
    fc_srv: dict[int, int] = {}
    for _ in range(80):
        msg = master.recv_match(type=["RC_CHANNELS", "SERVO_OUTPUT_RAW"], blocking=False)
        if msg is None:
            break
        if msg.get_type() == "RC_CHANNELS":
            for rc_ch in range(9, 17):
                fc_rc[rc_ch] = getattr(msg, f"chan{rc_ch}_raw", 0)
        else:
            for rc_ch in range(9, 17):
                fc_srv[rc_ch] = getattr(msg, f"servo{rc_ch}_raw", 0)
    return fc_rc, fc_srv


def _maybe_warn_servo_mismatch(rc: list[int], fc_rc: dict, fc_srv: dict, hold_neutral: bool) -> None:
    if hold_neutral:
        return
    from arm_joints import joint_center_us, joint_to_rc_ch

    with _lock:
        claw = int(_claw_stop_pwm)

    targets = []
    for joint in range(1, NUM_JOINTS + 1):
        rc_ch = joint_to_rc_ch(joint)
        val = rc[rc_ch - 1]
        neutral = joint_center_us(joint, claw_stop=claw if joint == 4 else None)
        if val == RC_IGNORE or abs(val - neutral) <= NEUTRAL_DEADBAND_US:
            continue
        targets.append((rc_ch, int(val)))

    if not targets or not fc_rc:
        return
    if not all(abs(fc_rc.get(ch, 0) - us) < 25 for ch, us in targets):
        print("[arm] *** RC override not reaching FC — check MAVProxy tcp:5763 ***", flush=True)
        return
    if fc_srv and not all(abs(fc_srv.get(ch, 0) - us) < 25 for ch, us in targets):
        print(
            "[arm] *** FC RC changed but servo output did not — "
            "SERVO9=59 SERVO11=61 SERVO13=63 SERVO15=65, BRD_SAFETYENABLE=0 ***",
            flush=True,
        )


def _maybe_clear_stale_preset_motion(now: float) -> None:
    global _preset_motion, _preset_motion_since
    with _lock:
        if not _preset_motion or _preset_motion_since <= 0:
            return
        if (now - _preset_motion_since) < PRESET_MOTION_TIMEOUT_SEC:
            return
        _preset_motion = False
        _preset_motion_since = 0.0
    print("[arm] Preset motion timeout — resuming arm_sender", flush=True)


def _handle_csv_line(line: str, now: float) -> None:
    global _joint_pwm, _last_pkt_time, _rx_count
    if line.startswith("PWM:"):
        line = line[4:]
    parts = line.split(",")
    if len(parts) < 7:
        return
    try:
        vals = [float(x) for x in parts[:7]]
    except ValueError:
        return

    with _lock:
        if _manual_mode or _preset_motion:
            return
        _joint_pwm = csv_list_to_joint_pwm(vals, claw_stop=int(_claw_stop_pwm))
        _last_pkt_time = now
        _rx_count += 1


def main() -> None:
    global _mavlink_up, _rx_count

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

    print(f"[arm] Listening on UDP {UDP_PORT} (joint CSV + JSON)", flush=True)
    print("[arm] J1/M13  J2/M9  J3/M11  Claw/M15", flush=True)

    last_send = 0.0
    last_heartbeat = 0.0
    last_print = 0.0
    last_telem = 0.0
    last_servo_diag = 0.0
    last_fc_rc: dict = {}
    last_fc_srv: dict = {}

    try:
        while True:
            now = time.time()
            _maybe_clear_stale_preset_motion(now)

            master = _get_mavlink_master()
            _mavlink_up = master is not None
            if master is not None:
                fc_rc, fc_srv = _drain_mavlink(master)
                if fc_rc:
                    last_fc_rc = fc_rc
                if fc_srv:
                    last_fc_srv = fc_srv

            try:
                data, addr = sock.recvfrom(1024)
                text = data.decode(errors="ignore").strip()
                if text.startswith("{"):
                    try:
                        _handle_arm_control_json(json.loads(text), addr)
                    except json.JSONDecodeError:
                        pass
                elif text:
                    _handle_csv_line(text, now)
                    _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            except socket.timeout:
                pass

            rc = None
            if master is not None and now - last_send >= 1.0 / OVERRIDE_HZ:
                last_send = now
                rc = _build_rc_array()
                try:
                    send_rc_channels_override(master, rc, ignore=RC_IGNORE)
                except OSError as e:
                    _drop_mavlink_master(str(e))
                    master = None

            if master is not None and now - last_heartbeat >= 1.0:
                last_heartbeat = now
                try:
                    master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )
                except OSError as e:
                    _drop_mavlink_master(str(e))
                    master = None

            if master is not None and rc is not None and now - last_servo_diag >= SERVO_DIAG_SEC:
                last_servo_diag = now
                with _lock:
                    lpt = _last_pkt_time
                    manual = _manual_mode
                    preset = _preset_motion
                hold = False if manual or preset else _should_hold_neutral(lpt)
                _maybe_warn_servo_mismatch(rc, last_fc_rc, last_fc_srv, hold)

            if now - last_telem >= 1.0 / ARM_TELEM_HZ:
                last_telem = now
                _send_arm_telemetry()

            if now - last_print >= 1.0 / PRINT_HZ:
                last_print = now
                with _lock:
                    pwm = dict(_joint_pwm)
                    rx = _rx_count
                    lpt = _last_pkt_time
                    manual = _manual_mode
                    armed = _arm_enabled
                hold = _should_hold_neutral(lpt) if not manual else False
                tag = "MANUAL" if manual else ("HOLD" if hold else "LIVE")
                print(
                    f"[arm] {tag} rx={rx} armed={armed} mav={'OK' if _mavlink_up else 'DOWN'} | "
                    f"J1={pwm[1]} J2={pwm[2]} J3={pwm[3]} Claw={pwm[4]}",
                    flush=True,
                )

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n[arm] Stopping — centering all joints.", flush=True)
    finally:
        _center_all_joints()
        master = _get_mavlink_master()
        if master is not None:
            try:
                send_rc_channels_override(master, _build_rc_array(), ignore=RC_IGNORE)
            except Exception:
                pass
            time.sleep(0.2)
        _drop_mavlink_master()
        print("[arm] Done.", flush=True)


if __name__ == "__main__":
    main()
