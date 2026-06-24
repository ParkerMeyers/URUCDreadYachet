#!/usr/bin/env python3

import math
import socket
import time

import adafruit_bno055
import board
import busio
from adafruit_servokit import ServoKit

UDP_IP = "0.0.0.0"
UDP_PORT = 5006
BNO055_ADDR = 0x29

MOSFET_GPIO = 17  # BCM GPIO17, HIGH = servo power ON
MOSFET_STATE_FILE = "/tmp/uru_mosfet_state"

MIN_US, MAX_US, CENTER_US = 500, 2500, 1500

UPDATE_HZ = 200
PRINT_HZ = 10
TIMEOUT_SEC = 0.75

MAX_STEP_US = 100
DEADBAND_US = 3

J4_CH = 10
J4_DEADBAND_US = 25
J4_MAX_STEP_US = 100

J6_IN_LOW = 1490
J6_IN_HIGH = 1510

J6_OUT_MIN = 1350
J6_OUT_CENTER = 1500
J6_OUT_MAX = 1650

J6_KP = -2.0
J6_DEADBAND_DEG = 3.0

LEVEL_NORMAL = [0.0180, -0.9993, 0.0337]

CLAW_CH = 8
CLAW_DEFAULT_US = 1460

# Incoming order:
# J1,J2,J3,J4,J5,J6_PWM,Claw,J6_TARGET_ANGLE
JOINT_TO_CHANNEL = {
    1: 10,  # J1
    2: 14,  # J2 shoulder
    3: 11,  # J3
    4: 9,   # J4
    5: 15,  # J5
    6: 12,  # J6 stabilization/manual
    7: 8,   # Claw
}

CHANNEL_TO_JOINT_INDEX = {
    10: 0,  # J1
    13: 1,  # J2
    11: 2,  # J3
    9: 3,   # J4
    15: 4,  # J5
    8: 6,   # Claw
}

J6_CH = 12
J5_CH = 15

kit = ServoKit(channels=16)
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c, address=BNO055_ADDR)

servo_power_enabled = False
controller_enabled = True
j6_enabled = False

target_us = {
    ch: CLAW_DEFAULT_US if ch == CLAW_CH else CENTER_US
    for ch in CHANNEL_TO_JOINT_INDEX
}

current_us = {
    ch: CLAW_DEFAULT_US if ch == CLAW_CH else CENTER_US
    for ch in CHANNEL_TO_JOINT_INDEX
}

j6_input_pwm = CENTER_US
j6_target_angle_deg = 0.0

last_packet_time = 0.0
rx_count = 0
last_raw = ""

last_pwm_sent = {}
j6_update_count = 0
last_j6_rate_time = time.time()
j6_actual_hz = 0.0


def servo_power(on):
    global servo_power_enabled
    servo_power_enabled = bool(on)


def read_mosfet_state():
    try:
        with open(MOSFET_STATE_FILE, "r", encoding="utf-8") as handle:
            return handle.read().strip().lower() in {"1", "true", "on", "yes", "y"}
    except FileNotFoundError:
        return False
    except Exception:
        return False


def refresh_mosfet_state():
    global servo_power_enabled
    servo_power_enabled = read_mosfet_state()


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_us(x):
    return int(clamp(float(x), MIN_US, MAX_US))


def normalize(v):
    magnitude = math.sqrt(sum(x * x for x in v))
    if magnitude < 0.001:
        return None
    return [x / magnitude for x in v]


LEVEL_NORMAL = normalize(LEVEL_NORMAL)


def us_to_angle(us):
    us = clamp_us(us)
    return (us - MIN_US) * 180.0 / (MAX_US - MIN_US)


def set_pwm(ch, us):
    us = clamp_us(us)

    if last_pwm_sent.get(ch) == us:
        return

    try:
        kit.servo[ch].angle = us_to_angle(us)
        last_pwm_sent[ch] = us
    except OSError as exc:
        print(f"I2C write failed PCA{ch}: {exc}")


def set_off(ch):
    try:
        kit.servo[ch].angle = None
        last_pwm_sent[ch] = None
    except OSError as exc:
        print(f"I2C off failed PCA{ch}: {exc}")


def map_forward_pwm(input_pwm):
    return J6_OUT_CENTER + (input_pwm - J6_IN_HIGH) * (J6_OUT_MAX - J6_OUT_CENTER) / (MAX_US - J6_IN_HIGH)


def map_reverse_pwm(input_pwm):
    return J6_OUT_CENTER - (J6_IN_LOW - input_pwm) * (J6_OUT_CENTER - J6_OUT_MIN) / (J6_IN_LOW - MIN_US)


def default_for_channel(ch):
    if ch == CLAW_CH:
        return CLAW_DEFAULT_US
    if ch == J6_CH:
        return J6_OUT_CENTER
    return CENTER_US


def enable_power():
    global servo_power_enabled
    servo_power_enabled = True
    servo_power(True)
    time.sleep(0.15)

    for ch in list(CHANNEL_TO_JOINT_INDEX.keys()) + [J6_CH]:
        default_us = default_for_channel(ch)
        current_us[ch] = default_us
        target_us[ch] = default_us
        set_pwm(ch, default_us)


def disable_power():
    global servo_power_enabled, controller_enabled, j6_enabled
    servo_power_enabled = False
    controller_enabled = False
    j6_enabled = False

    for ch in list(CHANNEL_TO_JOINT_INDEX.keys()) + [J6_CH]:
        set_off(ch)

    time.sleep(0.05)
    servo_power(False)


def smooth_regular_servos():
    refresh_mosfet_state()
    if not servo_power_enabled:
        return

    for ch in CHANNEL_TO_JOINT_INDEX:
        deadband = J4_DEADBAND_US if ch == J4_CH else DEADBAND_US
        max_step = J4_MAX_STEP_US if ch == J4_CH else MAX_STEP_US

        error = target_us[ch] - current_us[ch]

        if abs(error) <= deadband:
            new_us = current_us[ch]
        elif error > 0:
            new_us = current_us[ch] + min(max_step, error)
        else:
            new_us = current_us[ch] - min(max_step, -error)

        if new_us != current_us[ch]:
            current_us[ch] = new_us
            set_pwm(ch, new_us)


def read_j6_angle_deg():
    try:
        gravity_vec = bno.gravity
        if gravity_vec is None or any(value is None for value in gravity_vec):
            return None

        gravity = normalize(gravity_vec)
        if gravity is None:
            return None

        dot = (
            gravity[0] * LEVEL_NORMAL[0]
            + gravity[1] * LEVEL_NORMAL[1]
            + gravity[2] * LEVEL_NORMAL[2]
        )
        dot = clamp(dot, -1.0, 1.0)
        return math.degrees(math.asin(dot))
    except Exception:
        return None


def update_j6():
    refresh_mosfet_state()
    if not servo_power_enabled or not j6_enabled:
        return None, "off", J6_OUT_CENTER

    if j6_input_pwm > J6_IN_HIGH:
        pwm = int(round(clamp(map_forward_pwm(j6_input_pwm), J6_OUT_CENTER, J6_OUT_MAX)))
        set_pwm(J6_CH, pwm)
        return None, "manual+", pwm

    if j6_input_pwm < J6_IN_LOW:
        pwm = int(round(clamp(map_reverse_pwm(j6_input_pwm), J6_OUT_MIN, J6_OUT_CENTER)))
        set_pwm(J6_CH, pwm)
        return None, "manual-", pwm

    measured_angle = read_j6_angle_deg()
    if measured_angle is None:
        set_pwm(J6_CH, J6_OUT_CENTER)
        return None, "bad_imu", J6_OUT_CENTER

    error = measured_angle + j6_target_angle_deg

    if abs(error) < J6_DEADBAND_DEG:
        pwm = J6_OUT_CENTER
    else:
        pwm = J6_OUT_CENTER + J6_KP * error

    pwm = int(round(clamp(pwm, J6_OUT_MIN, J6_OUT_MAX)))
    set_pwm(J6_CH, pwm)
    return error, "hold", pwm


for ch in list(CHANNEL_TO_JOINT_INDEX.keys()) + [J6_CH]:
    kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)
    set_off(ch)

servo_power(False)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.001)

print(f"Listening on UDP {UDP_PORT}")
print(f"MOSFET GPIO {MOSFET_GPIO}: controlled from main UI")
print("Startup defaults: servo power OFF / controller input active")
print("Expected: J1,J2,J3,J4,J5,J6_PWM,Claw,J6_TARGET_ANGLE")
print("All joint PWM values expected from 500 to 2500.")
print()

last_update = time.time()
last_print = time.time()
j6_err = None
j6_pwm = CENTER_US
j6_status = "off"

try:
    while True:
        now = time.time()

        try:
            data, _addr = sock.recvfrom(1024)
            line = data.decode(errors="ignore").strip()
            last_raw = line

            if line.startswith("PWM:"):
                line = line[4:]

            parts = line.split(",")
            if len(parts) >= 8:
                values = [float(value) for value in parts]
                incoming = values[:7]

                rx_count += 1
                last_packet_time = now

                j6_input_pwm = clamp_us(incoming[5])
                j6_target_angle_deg = round(float(values[7]), 2)

                if controller_enabled:
                    for ch, joint_idx in CHANNEL_TO_JOINT_INDEX.items():
                        target_us[ch] = clamp_us(incoming[joint_idx])

        except socket.timeout:
            pass
        except ValueError:
            pass

        if j6_enabled and now - last_packet_time > TIMEOUT_SEC:
            j6_input_pwm = CENTER_US
            set_pwm(J6_CH, J6_OUT_CENTER)
            j6_status = "timeout"

        if now - last_update >= 1.0 / UPDATE_HZ:
            last_update = now
            smooth_regular_servos()
            j6_err, j6_status, j6_pwm = update_j6()

            j6_update_count += 1
            if now - last_j6_rate_time >= 1.0:
                j6_actual_hz = j6_update_count / (now - last_j6_rate_time)
                j6_update_count = 0
                last_j6_rate_time = now

        if now - last_print >= 1.0 / PRINT_HZ:
            last_print = now
            print(
                f"rx={rx_count} j6_hz={j6_actual_hz:6.1f} "
                f"power={servo_power_enabled} controller={controller_enabled} "
                f"j6={j6_enabled} | "
                f"j6_in={j6_input_pwm} mode={j6_status} "
                f"target={j6_target_angle_deg:+7.2f} "
                f"err={(j6_err if j6_err is not None else 0):+7.2f} pwm={j6_pwm} "
                f"raw='{last_raw[:80]}'"
            )

except KeyboardInterrupt:
    pass

finally:
    print("\nStopping.")
    set_pwm(J6_CH, J6_OUT_CENTER)
    time.sleep(0.05)

    for ch in list(CHANNEL_TO_JOINT_INDEX.keys()) + [J6_CH]:
        set_off(ch)

    servo_power(False)
    print("Servo power OFF.")
