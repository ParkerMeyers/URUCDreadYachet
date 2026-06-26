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

UDP (port 5006 — CSV + JSON; port 5009 — legacy JSON control):
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
    AUX_TO_JOINT,
    CLAW_STOP_US_DEFAULT,
    JOINT_NAMES,
    JOINT_TO_AUX,
    JOINT_TO_MOTOR,
    NUM_JOINTS,
    RC_IGNORE,
    build_rc_override,
    clamp_joint_pwm,
    csv_list_to_joint_pwm,
    default_joint_pwm,
    joint_pwm_to_csv_list,
    joint_to_rc_ch,
)
from mavlink_rc import MAVLINK_ONBOARD_ARM, connect_mavlink, send_rc_channels_override

UDP_PORT = 5006
ARM_CONTROL_PORT = 5009
MAVLINK_URL = MAVLINK_ONBOARD_ARM
OVERRIDE_HZ = 20
DIAG_INTERVAL = 2.0
ARM_TELEM_HZ = 5
ARM_TELEM_PORT = 5006
TIMEOUT_SEC = 0.75
PRESET_MOTION_TIMEOUT_SEC = 45.0

_lock = threading.Lock()
_pwm = default_joint_pwm()
_claw_stop = CLAW_STOP_US_DEFAULT
_last_pkt_time = 0.0
_rx_count = 0
_arm_enabled = False
_manual_mode = False
_preset_motion = False
_preset_motion_since = 0.0
_fc_rc: dict[int, int] = {}
_fc_srv: dict[int, int] = {}
_mavlink_ok = False
_mav_master = None

_telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_telem_subscribers: set[tuple[str, int]] = set()
_telem_sub_lock = threading.Lock()
_telem_send_failures: dict[tuple[str, int], int] = {}


def _hold_neutral(last_pkt: float) -> bool:
    if last_pkt <= 0:
        return True
    return time.time() - last_pkt > TIMEOUT_SEC


def _snapshot() -> tuple[dict[int, int], float, int, bool, bool, bool, bool, int]:
    with _lock:
        return (
            dict(_pwm),
            _last_pkt_time,
            _rx_count,
            _arm_enabled,
            _manual_mode,
            _preset_motion,
            _mavlink_ok,
            int(_claw_stop),
        )


def _effective_pwm() -> dict[int, int]:
    pwm, last_pkt, _, armed, manual, preset, _, claw = _snapshot()
    if not armed:
        return default_joint_pwm(claw_stop=claw)
    if manual or preset:
        return pwm
    if _hold_neutral(last_pkt):
        return default_joint_pwm(claw_stop=claw)
    return pwm


def _apply_pwm(updates: dict[int, int], *, from_csv: bool = False) -> None:
    global _pwm, _last_pkt_time, _rx_count
    with _lock:
        for joint, us in updates.items():
            if 1 <= joint <= NUM_JOINTS:
                _pwm[joint] = clamp_joint_pwm(joint, us, claw_stop=_claw_stop)
        _last_pkt_time = time.time()
        if from_csv:
            _rx_count += 1


def _center_all() -> None:
    global _pwm, _last_pkt_time
    with _lock:
        _pwm = default_joint_pwm(claw_stop=int(_claw_stop))
        _last_pkt_time = time.time()


def _poll_mavlink(master) -> None:
    global _fc_rc, _fc_srv
    while True:
        msg = master.recv_match(type=["RC_CHANNELS", "SERVO_OUTPUT_RAW"], blocking=False)
        if msg is None:
            break
        if msg.get_type() == "RC_CHANNELS":
            for joint in range(1, NUM_JOINTS + 1):
                rc_ch = joint_to_rc_ch(joint)
                _fc_rc[rc_ch] = getattr(msg, f"chan{rc_ch}_raw", 0)
        else:
            for joint in range(1, NUM_JOINTS + 1):
                rc_ch = joint_to_rc_ch(joint)
                _fc_srv[rc_ch] = getattr(msg, f"servo{rc_ch}_raw", 0)


def _send_override(master) -> None:
    send_rc_channels_override(master, build_rc_override(_effective_pwm()), ignore=RC_IGNORE)


def _send_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def _joint_telemetry_rows(pwm: dict[int, int]) -> list[dict]:
    rows = []
    for joint in range(1, NUM_JOINTS + 1):
        rc_ch = joint_to_rc_ch(joint)
        rows.append({
            "joint": joint,
            "name": JOINT_NAMES[joint],
            "aux": JOINT_TO_AUX[joint],
            "motor": JOINT_TO_MOTOR[joint],
            "rc_ch": rc_ch,
            "sending_us": int(pwm.get(joint, 0)),
            "fc_rc_us": _fc_rc.get(rc_ch),
            "fc_srv_us": _fc_srv.get(rc_ch),
        })
    return rows


def _telemetry_payload() -> dict:
    pwm, last_pkt, rx_count, armed, manual, preset, mav_ok, claw = _snapshot()
    hold = False if (manual or preset or not armed) else _hold_neutral(last_pkt)
    return {
        "type": "arm",
        "arm_enabled": bool(armed),
        "arm_claw_stop_us": claw,
        "arm_rx_count": int(rx_count),
        "arm_hold_neutral": bool(hold),
        "arm_mavlink_ok": bool(mav_ok),
        "arm_manual_mode": bool(manual),
        "arm_preset_motion": bool(preset),
        "arm_joint_us": joint_pwm_to_csv_list(pwm),
        "arm_joints": _joint_telemetry_rows(pwm),
    }


def _note_telemetry_subscriber(host: str, port: int) -> None:
    with _telem_sub_lock:
        _telem_subscribers.add((host, int(port)))


def _send_arm_telemetry() -> None:
    payload = json.dumps(_telemetry_payload()).encode("utf-8")
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


def _print_diag() -> None:
    pwm, _, _, armed, manual, preset, mav_ok, _ = _snapshot()
    print()
    if _fc_rc:
        rc_str = "  ".join(
            f"{JOINT_NAMES[j]}(AUX{JOINT_TO_AUX[j]},ch{joint_to_rc_ch(j)})="
            f"{_fc_rc.get(joint_to_rc_ch(j), '?')}"
            for j in range(1, NUM_JOINTS + 1)
        )
        print(f"[arm] RC_CHANNELS (FC RC input):     {rc_str}")
    else:
        print("[arm] RC_CHANNELS: no data yet")

    if _fc_srv:
        srv_str = "  ".join(
            f"{JOINT_NAMES[j]}(AUX{JOINT_TO_AUX[j]},ch{joint_to_rc_ch(j)})="
            f"{_fc_srv.get(joint_to_rc_ch(j), '?')}"
            for j in range(1, NUM_JOINTS + 1)
        )
        print(f"[arm] SERVO_OUTPUT_RAW (FC PWM out): {srv_str}")
    else:
        print("[arm] SERVO_OUTPUT_RAW: no data yet")

    sending = "  ".join(
        f"{JOINT_NAMES[j]}(AUX{JOINT_TO_AUX[j]})={pwm[j]}"
        for j in sorted(pwm)
    )
    tag = "MANUAL" if manual else ("PRESET" if preset else ("ARMED" if armed else "LOCKED"))
    print(f"[arm] We are sending ({tag}, mav={'OK' if mav_ok else 'DOWN'}): {sending}")
    print(flush=True)


def _handle_direct(cmd: dict) -> bool:
    if cmd.get("center_all"):
        _center_all()
        print("[arm] ALL CENTER", flush=True)
        return True
    if "joint" not in cmd:
        return False
    try:
        joint = int(cmd["joint"])
        us = int(cmd.get("pwm", 1500))
    except (TypeError, ValueError):
        return False
    if not (1 <= joint <= NUM_JOINTS):
        return False
    _apply_pwm({joint: us})
    name = JOINT_NAMES[joint]
    aux = JOINT_TO_AUX[joint]
    rc_ch = joint_to_rc_ch(joint)
    print(f"[arm] {name}/M{JOINT_TO_MOTOR[joint]} (AUX{aux}) → ch{rc_ch}  {us} µs", flush=True)
    return True


def _handle_csv_line(line: str) -> None:
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
        claw = int(_claw_stop)
    _apply_pwm(csv_list_to_joint_pwm(vals, claw_stop=claw), from_csv=True)


def _apply_arm_enable(cmd: dict) -> None:
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


def _apply_arm_claw_stop(cmd: dict) -> None:
    global _claw_stop
    stop = clamp_joint_pwm(4, cmd.get("stop_us", CLAW_STOP_US_DEFAULT))
    with _lock:
        _claw_stop = stop
    print(f"[arm] Claw stop PWM → {stop} µs", flush=True)


def _apply_preset_motion(cmd: dict) -> None:
    global _preset_motion, _manual_mode, _preset_motion_since, _last_pkt_time
    enabled = bool(cmd.get("enabled", False))
    with _lock:
        if not _arm_enabled and enabled:
            return
        _preset_motion = enabled
        _preset_motion_since = time.time() if enabled else 0.0
        if enabled:
            _manual_mode = False
            _last_pkt_time = time.time()
    print(f"[arm] Preset motion {'ON' if enabled else 'OFF'}", flush=True)


def _apply_preset_step(cmd: dict) -> None:
    global _preset_motion_since, _last_pkt_time
    pwms = cmd.get("pwm")
    if not isinstance(pwms, list) or len(pwms) < 7:
        return
    with _lock:
        if not _arm_enabled:
            return
        claw = int(_claw_stop)
    _apply_pwm(csv_list_to_joint_pwm(pwms, claw_stop=claw))
    with _lock:
        _preset_motion_since = time.time()


def _apply_manual_pwm(cmd: dict) -> None:
    global _manual_mode, _arm_enabled, _last_pkt_time

    if cmd.get("center"):
        with _lock:
            _manual_mode = True
        _center_all()
        print("[arm] Manual: all joints centered", flush=True)
        return

    if "enabled" in cmd and cmd.get("aux") is None and cmd.get("joint") is None:
        with _lock:
            _manual_mode = bool(cmd.get("enabled"))
        print(f"[arm] Manual mode {'ON' if cmd.get('enabled') else 'OFF'}", flush=True)
        return

    joint = cmd.get("joint")
    if joint is None and cmd.get("aux") is not None:
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
        if not _arm_enabled:
            _arm_enabled = True
            print("[arm] Arm auto-enabled for manual command", flush=True)
        _manual_mode = True
    _apply_pwm({joint_i: pwm_i})
    print(f"[arm] Manual {JOINT_NAMES[joint_i]} → {pwm_i} µs", flush=True)


def _handle_json(cmd: dict, addr: tuple[str, int] | None) -> None:
    if not isinstance(cmd, dict):
        return

    if _handle_direct(cmd):
        if addr:
            _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
        return

    name = cmd.get("cmd")
    if name == "preset_motion":
        _apply_preset_motion(cmd)
    elif name == "preset_step":
        _apply_preset_step(cmd)
    elif name == "manual_pwm":
        _apply_manual_pwm(cmd)
    elif name == "arm_telemetry" and cmd.get("subscribe"):
        host = str(cmd.get("host") or (addr[0] if addr else "")).strip()
        port = int(cmd.get("port", ARM_TELEM_PORT))
        if host:
            _note_telemetry_subscriber(host, port)
        _send_arm_telemetry()
    elif name == "arm_claw_stop":
        _apply_arm_claw_stop(cmd)
    elif name == "arm_enable":
        _apply_arm_enable(cmd)
    else:
        return

    if addr:
        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)


def _maybe_clear_stale_preset(now: float) -> None:
    global _preset_motion, _preset_motion_since
    with _lock:
        if not _preset_motion or _preset_motion_since <= 0:
            return
        if (now - _preset_motion_since) < PRESET_MOTION_TIMEOUT_SEC:
            return
        _preset_motion = False
        _preset_motion_since = 0.0
    print("[arm] Preset motion timeout — resuming arm_sender", flush=True)


def _control_listener() -> None:
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
            _handle_json(json.loads(data.decode()), addr)
        except socket.timeout:
            pass
        except json.JSONDecodeError as e:
            print(f"[arm] Control bad JSON: {e}", flush=True)
        except Exception as e:
            print(f"[arm] Control error: {e}", flush=True)


def _connect_mavlink():
    print(f"[arm] Connecting to MAVProxy at {MAVLINK_URL} ...", flush=True)
    master = connect_mavlink(MAVLINK_URL, timeout=12.0)
    print("[arm] Waiting for heartbeat from Pix6 ...", flush=True)
    hb = master.wait_heartbeat(timeout=15)
    if hb:
        print(
            f"[arm] Heartbeat OK "
            f"(system={master.target_system}  component={master.target_component})",
            flush=True,
        )
    else:
        print("[arm] *** NO HEARTBEAT in 15 s ***", flush=True)
        print("[arm]     Check: MAVProxy running?  Pix6 USB plugged in?", flush=True)

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


def main() -> None:
    global _mav_master, _mavlink_ok

    print("[arm] Arm controller starting...", flush=True)
    print("[arm] J1/M13  J2/M9  J3/M11  Claw/M15", flush=True)

    try:
        _mav_master = _connect_mavlink()
    except Exception as e:
        print(f"[arm] MAVLink connect failed ({e}) — will retry in loop", flush=True)
        _mav_master = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)

    threading.Thread(target=_control_listener, daemon=True).start()

    print(f"[arm] Listening on UDP {UDP_PORT} (joint CSV + JSON)", flush=True)

    last_send = 0.0
    last_heartbeat = 0.0
    last_diag = 0.0
    last_telem = 0.0
    last_mav_retry = 0.0

    if _mav_master is not None:
        _send_override(_mav_master)

    try:
        while True:
            now = time.time()
            _maybe_clear_stale_preset(now)

            master = _mav_master
            _mavlink_ok = master is not None
            if master is not None:
                _poll_mavlink(master)

            try:
                while True:
                    data, addr = sock.recvfrom(4096)
                    text = data.decode(errors="ignore").strip()
                    if text.startswith("{"):
                        try:
                            _handle_json(json.loads(text), addr)
                        except json.JSONDecodeError:
                            pass
                    elif text:
                        _handle_csv_line(text)
                        _note_telemetry_subscriber(addr[0], ARM_TELEM_PORT)
            except BlockingIOError:
                pass

            if master is None and now - last_mav_retry >= 5.0:
                last_mav_retry = now
                try:
                    _mav_master = _connect_mavlink()
                    master = _mav_master
                    _send_override(master)
                except Exception as e:
                    print(f"[arm] MAVLink retry failed: {e}", flush=True)
                    _mav_master = None

            if master is not None:
                if now - last_heartbeat >= 1.0:
                    last_heartbeat = now
                    try:
                        _send_heartbeat(master)
                    except OSError as e:
                        print(f"[arm] MAVLink heartbeat failed: {e}", flush=True)
                        try:
                            master.close()
                        except Exception:
                            pass
                        _mav_master = None

                if now - last_send >= 1.0 / OVERRIDE_HZ:
                    last_send = now
                    try:
                        _send_override(master)
                    except OSError as e:
                        print(f"[arm] MAVLink override failed: {e}", flush=True)
                        try:
                            master.close()
                        except Exception:
                            pass
                        _mav_master = None

            if now - last_telem >= 1.0 / ARM_TELEM_HZ:
                last_telem = now
                _send_arm_telemetry()

            if now - last_diag >= DIAG_INTERVAL:
                last_diag = now
                _print_diag()

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[arm] Stopping — centering all joints.", flush=True)
    finally:
        _center_all()
        master = _mav_master
        if master is not None:
            try:
                _send_override(master)
            except Exception:
                pass
            time.sleep(0.2)
            try:
                master.close()
            except Exception:
                pass
        print("[arm] Done.", flush=True)


if __name__ == "__main__":
    main()
