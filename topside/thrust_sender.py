#!/usr/bin/env python3
"""
ROV joystick sender with stabilization, depth hold, yaw hold, gain scaling, and telemetry.

Controls:
- S = pitch/roll stabilization toggle
- D = depth hold toggle
- Y = yaw hold toggle
- D-pad up/down = gain +/- 10%
- ESC = quit

Controller:
- Button 9 toggles stabilization.
- Optional button constants exist for depth/yaw/gain if you want them.

Control layout:
Set CONTROL_LAYOUT below:
- "original":
    Left stick X  = yaw
    Right stick X = strafe
- "swapped":
    Left stick X  = strafe
    Right stick X = yaw

Both layouts:
- Left stick Y  = vertical / heave
- Right stick Y = forward/back

Thrust limiting:
- Gain scales all commands first.
- Horizontal group demand = max(abs(forward), abs(lateral), abs(yaw)).
- Vertical group demand = abs(vertical).
- If horizontal + vertical > 1.50, all commands are scaled down together.
"""

import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

import json
import socket
import sys
import time

import pygame


try:
    from pynput import keyboard as pynput_keyboard
    HAVE_PYNPUT = True
except Exception:
    HAVE_PYNPUT = False


# ============================================================
# USER CONFIG
# ============================================================

PI_IP_DEFAULT = "10.42.0.181"
UDP_PORT = 5005
TELEMETRY_PORT = 5006
SEND_HZ = 50

UI_TELEMETRY_FORWARD_PORT = os.getenv("ROV_TELEMETRY_UI_PORT")
UI_MODE_FILE = os.getenv("ROV_UI_MODE_FILE", "")
UI_MANAGED = os.getenv("ROV_UI_MANAGED", "").strip().lower() in ("1", "true", "yes")

if UI_MANAGED:
    USE_DISPLAY = False

USE_DISPLAY = True

# Choose:
#   "original" = left X yaw, right X strafe
#   "swapped"  = left X strafe, right X yaw
CONTROL_LAYOUT = "original"

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4

SIGN_YAW = 1.0
SIGN_VERTICAL = -1.0
SIGN_LATERAL = 1.0
SIGN_FORWARD = -1.0

DEADZONE = 0.05

GAIN_MIN_PERCENT = 10
GAIN_MAX_PERCENT = 100
GAIN_STEP_PERCENT = 10
GAIN_DEFAULT_PERCENT = 100

COMBINED_GROUP_TOTAL_LIMIT = 1.50

BUTTON_STABILIZE = 9
BUTTON_DEPTH_HOLD = None
BUTTON_YAW_HOLD = None
BUTTON_GAIN_UP = None
BUTTON_GAIN_DOWN = None
BUTTON_QUIT = None

PRINT_BUTTON_DEBUG = True


# ============================================================
# HELPERS
# ============================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def apply_deadzone(x, dz):
    if abs(x) < dz:
        return 0.0
    return x


def get_axis_safe(joy, axis_index):
    if axis_index is None:
        return 0.0

    if axis_index < 0 or axis_index >= joy.get_numaxes():
        return 0.0

    return joy.get_axis(axis_index)


def adjust_gain(gain_percent, delta_percent):
    gain_percent += delta_percent
    gain_percent = int(clamp(gain_percent, GAIN_MIN_PERCENT, GAIN_MAX_PERCENT))
    return gain_percent


def apply_combined_group_limit(forward, lateral, yaw, vertical):
    horizontal_group = max(abs(forward), abs(lateral), abs(yaw))
    vertical_group = abs(vertical)
    total_group = horizontal_group + vertical_group

    if total_group <= COMBINED_GROUP_TOTAL_LIMIT:
        return forward, lateral, yaw, vertical, 1.0, horizontal_group, vertical_group, total_group

    if total_group <= 0.000001:
        return forward, lateral, yaw, vertical, 1.0, horizontal_group, vertical_group, total_group

    scale = COMBINED_GROUP_TOTAL_LIMIT / total_group

    forward *= scale
    lateral *= scale
    yaw *= scale
    vertical *= scale

    horizontal_group *= scale
    vertical_group *= scale
    total_group = horizontal_group + vertical_group

    return forward, lateral, yaw, vertical, scale, horizontal_group, vertical_group, total_group


def make_neutral_packet(seq):
    return {
        "seq": seq,
        "time": time.time(),
        "forward": 0.0,
        "lateral": 0.0,
        "yaw": 0.0,
        "vertical": 0.0,
        "stabilize": False,
        "depth_hold": False,
        "yaw_hold": False,
        "gain_percent": GAIN_DEFAULT_PERCENT,
        "telemetry_port": TELEMETRY_PORT,
    }


def read_ui_mode():
    if not UI_MODE_FILE:
        return "Drive/Armed"

    try:
        with open(UI_MODE_FILE, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except Exception:
        return "Drive/Armed"

    mode = str(payload.get("mode", "Drive/Armed")).strip()
    if mode in {"Stabilization", "Drive/Armed", "Disarmed"}:
        return mode
    return "Drive/Armed"


def format_depth(value):
    if value is None:
        return "N/A"
    try:
        return f"{float(value):+.2f} m"
    except Exception:
        return "N/A"


def format_deg(value):
    if value is None:
        return "N/A"
    try:
        return f"{float(value):+.1f} deg"
    except Exception:
        return "N/A"


def get_layout_axes():
    if CONTROL_LAYOUT.lower() == "original":
        return {
            "name": "ORIGINAL: Left X = Yaw, Right X = Strafe",
            "axis_yaw": AXIS_LEFT_X,
            "axis_lateral": AXIS_RIGHT_X,
        }

    if CONTROL_LAYOUT.lower() == "swapped":
        return {
            "name": "SWAPPED: Left X = Strafe, Right X = Yaw",
            "axis_yaw": AXIS_RIGHT_X,
            "axis_lateral": AXIS_LEFT_X,
        }

    raise ValueError('CONTROL_LAYOUT must be "original" or "swapped"')


# ============================================================
# MAIN
# ============================================================

def main():
    layout = get_layout_axes()
    axis_yaw = layout["axis_yaw"]
    axis_lateral = layout["axis_lateral"]

    if len(sys.argv) >= 2:
        pi_ip = sys.argv[1]
    else:
        pi_ip = PI_IP_DEFAULT
        print(f"No IP given. Using default Pi IP: {pi_ip}")

    pygame.init()
    pygame.joystick.init()

    screen = None
    font = None

    if USE_DISPLAY:
        try:
            screen = pygame.display.set_mode((920, 660))
            pygame.display.set_caption(
                f"ROV Sender - {layout['name']}"
            )
            font = pygame.font.SysFont(None, 25)
        except Exception as e:
            print(f"Could not open pygame display: {e}")
            print("Continuing headless.")
            screen = None
            font = None

    if pygame.joystick.get_count() == 0:
        if UI_MANAGED:
            print("No joystick found. UI-managed mode: sending neutral thrust until a controller is connected.")
            joy = None
        else:
            print("No joystick found.")
            print("Plug in controller, then run again.")
            sys.exit(1)
    else:
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"Using joystick: {joy.get_name()}")
        print(f"Axes: {joy.get_numaxes()}, Buttons: {joy.get_numbuttons()}, Hats: {joy.get_numhats()}")
    print(f"Sending UDP to {pi_ip}:{UDP_PORT}")
    print(f"Listening for telemetry on UDP port {TELEMETRY_PORT}")
    print()
    print(layout["name"])
    print("  Left stick Y           = vertical")
    print("  Right stick Y          = forward/back")
    print()
    print("Controls:")
    print(f"  Button {BUTTON_STABILIZE} = pitch/roll stabilization toggle")
    print("  S = pitch/roll stabilization toggle")
    print("  D = depth hold toggle")
    print("  Y = yaw hold toggle")
    print("  D-pad up/down = gain +/- 10%")
    print("  ESC = quit")
    print()
    print("Combined thrust limit: H + V <= 150%")
    print()

    if HAVE_PYNPUT:
        print("Global keyboard enabled with pynput: S/D/Y/ESC/up/down work without pygame focus.")
    else:
        print("Global keyboard not enabled. To enable it:")
        print("  pip3 install pynput")

    print()

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    telemetry_sock.bind(("0.0.0.0", TELEMETRY_PORT))
    telemetry_sock.setblocking(False)

    telemetry_forward_sock = None
    telemetry_forward_addr = None
    if UI_TELEMETRY_FORWARD_PORT:
        try:
            telemetry_forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            telemetry_forward_addr = ("127.0.0.1", int(UI_TELEMETRY_FORWARD_PORT))
            print(f"Forwarding telemetry to UI on 127.0.0.1:{UI_TELEMETRY_FORWARD_PORT}")
        except Exception as exc:
            print(f"Could not configure UI telemetry forward port: {exc}")
            telemetry_forward_sock = None
            telemetry_forward_addr = None

    if UI_MODE_FILE:
        print(f"Reading UI mode from: {UI_MODE_FILE}")

    stabilize = False
    depth_hold = False
    yaw_hold = False
    gain_percent = GAIN_DEFAULT_PERCENT

    seq = 0
    running = True
    clock = pygame.time.Clock()
    last_print = 0.0

    button_previous = [False] * (joy.get_numbuttons() if joy is not None else 0)
    hat_previous = [(0, 0)] * (joy.get_numhats() if joy is not None else 0)

    telemetry = {
        "depth_m": None,
        "hold_depth_m": None,
        "depth_hold_request": False,
        "depth_hold_active": False,
        "depth_recapture_pending": False,
        "depth_recapture_time_remaining": 0.0,
        "manual_vertical_active": False,

        "yaw_deg": None,
        "hold_yaw_deg": None,
        "yaw_hold_request": False,
        "yaw_hold_active": False,
        "yaw_recapture_pending": False,
        "yaw_recapture_time_remaining": 0.0,
        "manual_yaw_active": False,
        "yaw_error_deg": 0.0,
        "yaw_correction": 0.0,

        "depth_source": "none",
        "state": "NO_TELEMETRY",
        "depth_correction": 0.0,
        "roll_deg": None,
        "pitch_deg": None,
        "pcorr": 0.0,
        "rcorr": 0.0,
    }
    last_telemetry_time = 0.0

    global_stabilize_toggle_requested = False
    global_depth_toggle_requested = False
    global_yaw_toggle_requested = False
    global_quit_requested = False
    global_gain_delta_requested = 0

    def on_global_key_press(key):
        nonlocal global_stabilize_toggle_requested
        nonlocal global_depth_toggle_requested
        nonlocal global_yaw_toggle_requested
        nonlocal global_quit_requested
        nonlocal global_gain_delta_requested

        try:
            if key == pynput_keyboard.Key.esc:
                global_quit_requested = True
                return False

            if key == pynput_keyboard.Key.up:
                global_gain_delta_requested += GAIN_STEP_PERCENT

            elif key == pynput_keyboard.Key.down:
                global_gain_delta_requested -= GAIN_STEP_PERCENT

            elif hasattr(key, "char") and key.char is not None:
                ch = key.char.lower()

                if ch == "s":
                    global_stabilize_toggle_requested = True

                elif ch == "d":
                    global_depth_toggle_requested = True

                elif ch == "y":
                    global_yaw_toggle_requested = True

        except Exception:
            pass

        return True

    keyboard_listener = None

    if HAVE_PYNPUT:
        try:
            keyboard_listener = pynput_keyboard.Listener(
                on_press=on_global_key_press
            )
            keyboard_listener.daemon = True
            keyboard_listener.start()
        except Exception as e:
            print(f"Could not start global keyboard listener: {e}")
            keyboard_listener = None

    try:
        while running:
            # ----------------------------------------------------
            # Read telemetry from Pi.
            # ----------------------------------------------------
            while True:
                try:
                    data, _addr = telemetry_sock.recvfrom(4096)
                except BlockingIOError:
                    break

                if telemetry_forward_sock is not None and telemetry_forward_addr is not None:
                    try:
                        telemetry_forward_sock.sendto(data, telemetry_forward_addr)
                    except Exception:
                        pass

                try:
                    telemetry = json.loads(data.decode("utf-8"))
                    last_telemetry_time = time.time()
                except Exception:
                    pass

            telemetry_age = time.time() - last_telemetry_time if last_telemetry_time > 0 else 999.0
            telemetry_online = telemetry_age < 1.0

            # ----------------------------------------------------
            # Pygame events.
            # ----------------------------------------------------
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

                    elif event.key == pygame.K_s:
                        stabilize = not stabilize
                        print(f"Stabilization toggled by pygame keyboard: {stabilize}")

                    elif event.key == pygame.K_d:
                        depth_hold = not depth_hold
                        print(f"Depth hold toggled by pygame keyboard: {depth_hold}")

                    elif event.key == pygame.K_y:
                        yaw_hold = not yaw_hold
                        print(f"Yaw hold toggled by pygame keyboard: {yaw_hold}")

                    elif event.key == pygame.K_UP:
                        gain_percent = adjust_gain(gain_percent, GAIN_STEP_PERCENT)
                        print(f"Gain: {gain_percent}%")

                    elif event.key == pygame.K_DOWN:
                        gain_percent = adjust_gain(gain_percent, -GAIN_STEP_PERCENT)
                        print(f"Gain: {gain_percent}%")

            pygame.event.pump()

            # ----------------------------------------------------
            # Global keyboard requests.
            # ----------------------------------------------------
            if global_quit_requested:
                running = False

            if global_stabilize_toggle_requested:
                global_stabilize_toggle_requested = False
                stabilize = not stabilize
                print(f"Stabilization toggled by global keyboard: {stabilize}")

            if global_depth_toggle_requested:
                global_depth_toggle_requested = False
                depth_hold = not depth_hold
                print(f"Depth hold toggled by global keyboard: {depth_hold}")

            if global_yaw_toggle_requested:
                global_yaw_toggle_requested = False
                yaw_hold = not yaw_hold
                print(f"Yaw hold toggled by global keyboard: {yaw_hold}")

            if global_gain_delta_requested != 0:
                gain_percent = adjust_gain(gain_percent, global_gain_delta_requested)
                global_gain_delta_requested = 0
                print(f"Gain: {gain_percent}%")

            # ----------------------------------------------------
            # Controller buttons / axes (optional when UI-managed).
            # ----------------------------------------------------
            if joy is None and pygame.joystick.get_count() > 0:
                joy = pygame.joystick.Joystick(0)
                joy.init()
                button_previous = [False] * joy.get_numbuttons()
                hat_previous = [(0, 0)] * joy.get_numhats()
                print(f"Joystick connected: {joy.get_name()}")

            if joy is not None:
                num_buttons = joy.get_numbuttons()

                if len(button_previous) != num_buttons:
                    button_previous = [False] * num_buttons

                for b in range(num_buttons):
                    pressed = bool(joy.get_button(b))

                    if pressed and not button_previous[b]:
                        if PRINT_BUTTON_DEBUG:
                            print(f"Button pressed: {b}")

                        if b == BUTTON_STABILIZE:
                            stabilize = not stabilize
                            print(f"Stabilization toggled by controller button {b}: {stabilize}")

                        if BUTTON_DEPTH_HOLD is not None and b == BUTTON_DEPTH_HOLD:
                            depth_hold = not depth_hold
                            print(f"Depth hold toggled by controller button {b}: {depth_hold}")

                        if BUTTON_YAW_HOLD is not None and b == BUTTON_YAW_HOLD:
                            yaw_hold = not yaw_hold
                            print(f"Yaw hold toggled by controller button {b}: {yaw_hold}")

                        if BUTTON_GAIN_UP is not None and b == BUTTON_GAIN_UP:
                            gain_percent = adjust_gain(gain_percent, GAIN_STEP_PERCENT)
                            print(f"Gain: {gain_percent}%")

                        if BUTTON_GAIN_DOWN is not None and b == BUTTON_GAIN_DOWN:
                            gain_percent = adjust_gain(gain_percent, -GAIN_STEP_PERCENT)
                            print(f"Gain: {gain_percent}%")

                        if BUTTON_QUIT is not None and b == BUTTON_QUIT:
                            running = False

                    button_previous[b] = pressed

                num_hats = joy.get_numhats()

                if len(hat_previous) != num_hats:
                    hat_previous = [(0, 0)] * num_hats

                for h in range(num_hats):
                    hat_x, hat_y = joy.get_hat(h)
                    prev_x, prev_y = hat_previous[h]

                    if hat_y == 1 and prev_y != 1:
                        gain_percent = adjust_gain(gain_percent, GAIN_STEP_PERCENT)
                        print(f"Gain: {gain_percent}%")

                    elif hat_y == -1 and prev_y != -1:
                        gain_percent = adjust_gain(gain_percent, -GAIN_STEP_PERCENT)
                        print(f"Gain: {gain_percent}%")

                    hat_previous[h] = (hat_x, hat_y)

                left_x = get_axis_safe(joy, AXIS_LEFT_X)
                left_y = get_axis_safe(joy, AXIS_LEFT_Y)
                right_x = get_axis_safe(joy, AXIS_RIGHT_X)
                right_y = get_axis_safe(joy, AXIS_RIGHT_Y)
            else:
                left_x = left_y = right_x = right_y = 0.0

            # ----------------------------------------------------
            # Mapped controls.
            # ----------------------------------------------------
            yaw_raw = SIGN_YAW * get_axis_safe(joy, axis_yaw)
            vertical_raw = SIGN_VERTICAL * get_axis_safe(joy, AXIS_LEFT_Y)
            lateral_raw = SIGN_LATERAL * get_axis_safe(joy, axis_lateral)
            forward_raw = SIGN_FORWARD * get_axis_safe(joy, AXIS_RIGHT_Y)

            yaw_raw = clamp(apply_deadzone(yaw_raw, DEADZONE), -1.0, 1.0)
            vertical_raw = clamp(apply_deadzone(vertical_raw, DEADZONE), -1.0, 1.0)
            lateral_raw = clamp(apply_deadzone(lateral_raw, DEADZONE), -1.0, 1.0)
            forward_raw = clamp(apply_deadzone(forward_raw, DEADZONE), -1.0, 1.0)

            gain = gain_percent / 100.0

            forward = forward_raw * gain
            lateral = lateral_raw * gain
            yaw = yaw_raw * gain
            vertical = vertical_raw * gain

            (
                forward,
                lateral,
                yaw,
                vertical,
                total_limit_scale,
                horizontal_group,
                vertical_group,
                total_group,
            ) = apply_combined_group_limit(
                forward,
                lateral,
                yaw,
                vertical,
            )

            ui_mode = read_ui_mode()
            if ui_mode == "Disarmed":
                packet = make_neutral_packet(seq)
            else:
                packet_stabilize = True if ui_mode == "Stabilization" else stabilize
                packet = {
                    "seq": seq,
                    "time": time.time(),
                    "forward": forward,
                    "lateral": lateral,
                    "yaw": yaw,
                    "vertical": vertical,
                    "stabilize": packet_stabilize,
                    "depth_hold": depth_hold,
                    "yaw_hold": yaw_hold,
                    "gain_percent": gain_percent,
                    "telemetry_port": TELEMETRY_PORT,
                }

            send_sock.sendto(json.dumps(packet).encode("utf-8"), (pi_ip, UDP_PORT))
            seq += 1

            # ----------------------------------------------------
            # Display.
            # ----------------------------------------------------
            if screen is not None and font is not None:
                screen.fill((20, 20, 20))

                receiver_state = telemetry.get("state", "NO_TELEMETRY") if telemetry_online else "NO_TELEMETRY"

                depth_m = telemetry.get("depth_m", None) if telemetry_online else None
                hold_depth_m = telemetry.get("hold_depth_m", None) if telemetry_online else None
                depth_source = telemetry.get("depth_source", "none") if telemetry_online else "none"
                depth_hold_active = bool(telemetry.get("depth_hold_active", False)) if telemetry_online else False
                depth_recapture_pending = bool(telemetry.get("depth_recapture_pending", False)) if telemetry_online else False
                depth_wait = float(telemetry.get("depth_recapture_time_remaining", 0.0)) if telemetry_online else 0.0
                manual_vertical_active = bool(telemetry.get("manual_vertical_active", False)) if telemetry_online else False
                depth_correction = float(telemetry.get("depth_correction", 0.0)) if telemetry_online else 0.0

                yaw_deg = telemetry.get("yaw_deg", None) if telemetry_online else None
                hold_yaw_deg = telemetry.get("hold_yaw_deg", None) if telemetry_online else None
                yaw_hold_active = bool(telemetry.get("yaw_hold_active", False)) if telemetry_online else False
                yaw_recapture_pending = bool(telemetry.get("yaw_recapture_pending", False)) if telemetry_online else False
                yaw_wait = float(telemetry.get("yaw_recapture_time_remaining", 0.0)) if telemetry_online else 0.0
                manual_yaw_active = bool(telemetry.get("manual_yaw_active", False)) if telemetry_online else False
                yaw_error_deg = float(telemetry.get("yaw_error_deg", 0.0)) if telemetry_online else 0.0
                yaw_correction = float(telemetry.get("yaw_correction", 0.0)) if telemetry_online else 0.0

                roll_deg = telemetry.get("roll_deg", None) if telemetry_online else None
                pitch_deg = telemetry.get("pitch_deg", None) if telemetry_online else None

                lines = [
                    layout["name"],
                    f"Sending to {pi_ip}:{UDP_PORT}     Telemetry: {receiver_state}",
                    "",
                    f"Left stick raw:  X={left_x:+.2f} Y={left_y:+.2f}",
                    f"Right stick raw: X={right_x:+.2f} Y={right_y:+.2f}",
                    "",
                    f"CMD: F={forward:+.2f} L={lateral:+.2f} Yaw={yaw:+.2f} V={vertical:+.2f}",
                    f"Gain: {gain_percent}%     Total limit scale: {total_limit_scale:.2f}",
                    f"H group: {horizontal_group:.2f}     V group: {vertical_group:.2f}     H+V: {total_group:.2f}/1.50",
                    "",
                    f"Stabilization: {'ON' if stabilize else 'OFF'}     press S / controller button {BUTTON_STABILIZE}",
                    "",
                    f"Depth hold request: {'ON' if depth_hold else 'OFF'}     press D",
                    f"Depth active: {'YES' if depth_hold_active else 'NO'}     recapture: {'YES' if depth_recapture_pending else 'NO'} wait={depth_wait:.2f}s     manual V: {'YES' if manual_vertical_active else 'NO'}",
                    f"Depth: {format_depth(depth_m)}     Hold depth: {format_depth(hold_depth_m)}",
                    f"Depth correction: {depth_correction:+.3f}     Source: {depth_source}",
                    "",
                    f"Yaw hold request: {'ON' if yaw_hold else 'OFF'}     press Y",
                    f"Yaw active: {'YES' if yaw_hold_active else 'NO'}     recapture: {'YES' if yaw_recapture_pending else 'NO'} wait={yaw_wait:.2f}s     manual yaw: {'YES' if manual_yaw_active else 'NO'}",
                    f"Yaw: {format_deg(yaw_deg)}     Hold yaw: {format_deg(hold_yaw_deg)}     Error: {yaw_error_deg:+.1f} deg",
                    f"Yaw correction: {yaw_correction:+.3f}",
                    "",
                    f"Roll: {format_deg(roll_deg)}     Pitch: {format_deg(pitch_deg)}",
                    "",
                    "D-pad up/down changes gain by 10%",
                    "Joystick background input enabled",
                ]

                y_pos = 14

                for line in lines:
                    text = font.render(line, True, (230, 230, 230))
                    screen.blit(text, (20, y_pos))
                    y_pos += 25

                pygame.display.flip()

            # ----------------------------------------------------
            # Terminal print.
            # ----------------------------------------------------
            now = time.time()

            if now - last_print > 0.5:
                last_print = now

                if telemetry_online:
                    depth_text = format_depth(telemetry.get("depth_m", None))
                    hold_text = format_depth(telemetry.get("hold_depth_m", None))
                    yaw_text = format_deg(telemetry.get("yaw_deg", None))
                    hold_yaw_text = format_deg(telemetry.get("hold_yaw_deg", None))
                    rx_state = telemetry.get("state", "OK")
                    dh_active_text = telemetry.get("depth_hold_active", False)
                    yh_active_text = telemetry.get("yaw_hold_active", False)
                else:
                    depth_text = "N/A"
                    hold_text = "N/A"
                    yaw_text = "N/A"
                    hold_yaw_text = "N/A"
                    rx_state = "NO_TELEMETRY"
                    dh_active_text = False
                    yh_active_text = False

                print(
                    f"RX={rx_state} | "
                    f"GAIN={gain_percent}% scale={total_limit_scale:.2f} "
                    f"H={horizontal_group:.2f} Vgrp={vertical_group:.2f} sum={total_group:.2f} | "
                    f"CMD F={forward:+.2f} L={lateral:+.2f} "
                    f"Y={yaw:+.2f} V={vertical:+.2f} | "
                    f"stab={stabilize} "
                    f"dh_req={depth_hold} dh_act={dh_active_text} "
                    f"yh_req={yaw_hold} yh_act={yh_active_text} | "
                    f"depth={depth_text} holdD={hold_text} "
                    f"yaw={yaw_text} holdY={hold_yaw_text}"
                )

            clock.tick(SEND_HZ)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt. Quitting.")

    finally:
        neutral_packet = make_neutral_packet(seq)

        for _ in range(10):
            send_sock.sendto(json.dumps(neutral_packet).encode("utf-8"), (pi_ip, UDP_PORT))
            time.sleep(0.02)

        try:
            if keyboard_listener is not None:
                keyboard_listener.stop()
        except Exception:
            pass

        pygame.quit()


if __name__ == "__main__":
    main()
