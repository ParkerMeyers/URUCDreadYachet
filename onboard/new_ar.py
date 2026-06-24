#!/usr/bin/env python3

import socket, time, sys, select, termios, tty, math, signal
import board, busio, adafruit_bno055
from adafruit_servokit import ServoKit
import lgpio

UDP_IP = "0.0.0.0"
UDP_PORT = 5006
BNO055_ADDR = 0x29

MOSFET_GPIO = 17  # BCM GPIO17, HIGH = servo power ON

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
    4: 9,  # J4
    5: 15,  # J5
    6: 12,  # J6 stabilization/manual
    7: 8,   # Claw
}

CHANNEL_TO_JOINT_INDEX = {
    10: 0,  # J1
    13: 1,  # J2
    11: 2,  # J3
    9: 3,  # J4
    15: 4,  # J5
    8: 6,   # Claw
}

J6_CH = 12
J5_CH = 15



gpio_handle = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(gpio_handle, MOSFET_GPIO, 0)

kit = ServoKit(channels=16)
i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c, address=BNO055_ADDR)

servo_power_enabled = False
controller_enabled = False
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
    lgpio.gpio_write(gpio_handle, MOSFET_GPIO, 1 if on else 0)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def clamp_us(x):
    return int(clamp(float(x), MIN_US, MAX_US))

def normalize(v):
    m = math.sqrt(sum(x * x for x in v))
    if m < 0.001:
        return None
    return [x / m for x in v]

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
    except OSError as e:
        print(f"I2C write failed PCA{ch}: {e}")

def set_off(ch):
    try:
        kit.servo[ch].angle = None
        last_pwm_sent[ch] = None
    except OSError as e:
        print(f"I2C off failed PCA{ch}: {e}")

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
    if not servo_power_enabled:
        return

    for ch in CHANNEL_TO_JOINT_INDEX:
        deadband = J4_DEADBAND_US if ch == J4_CH else DEADBAND_US
        max_step = J4_MAX_STEP_US if ch == J4_CH else MAX_STEP_US

        err = target_us[ch] - current_us[ch]

        if abs(err) <= deadband:
            new_us = current_us[ch]
        elif err > 0:
            new_us = current_us[ch] + min(max_step, err)
        else:
            new_us = current_us[ch] - min(max_step, -err)

        if new_us != current_us[ch]:
            current_us[ch] = new_us
            set_pwm(ch, new_us)

def read_j6_angle_deg():
    try:
        g = bno.gravity
        if g is None or any(v is None for v in g):
            return None

        gravity = normalize(g)
        if gravity is None:
            return None

        dot = (
            gravity[0] * LEVEL_NORMAL[0] +
            gravity[1] * LEVEL_NORMAL[1] +
            gravity[2] * LEVEL_NORMAL[2]
        )

        dot = clamp(dot, -1.0, 1.0)
        return math.degrees(math.asin(dot))

    except Exception:
        return None

def update_j6():
    if not servo_power_enabled or not j6_enabled:
        return None, "off", J6_OUT_CENTER

    # Manual override still has priority.
    if j6_input_pwm > J6_IN_HIGH:
        pwm = int(round(clamp(map_forward_pwm(j6_input_pwm), J6_OUT_CENTER, J6_OUT_MAX)))
        set_pwm(J6_CH, pwm)
        return None, "manual+", pwm

    if j6_input_pwm < J6_IN_LOW:
        pwm = int(round(clamp(map_reverse_pwm(j6_input_pwm), J6_OUT_MIN, J6_OUT_CENTER)))
        set_pwm(J6_CH, pwm)
        return None, "manual-", pwm

    # Centered J6 input means stabilization resumes.
    measured_angle = read_j6_angle_deg()
    if measured_angle is None:
        set_pwm(J6_CH, J6_OUT_CENTER)
        return None, "bad_imu", J6_OUT_CENTER

    err = measured_angle + j6_target_angle_deg

    if abs(err) < J6_DEADBAND_DEG:
        pwm = J6_OUT_CENTER
    else:
        pwm = J6_OUT_CENTER + J6_KP * err

    pwm = int(round(clamp(pwm, J6_OUT_MIN, J6_OUT_MAX)))
    set_pwm(J6_CH, pwm)
    return err, "hold", pwm

def handle_keyboard():
    global controller_enabled, j6_enabled, j6_input_pwm

    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return None

    c = sys.stdin.read(1)

    if c == " ":
        if servo_power_enabled:
            disable_power()
            print("\nSERVO POWER OFF / CONTROLLER OFF / J6 OFF")
        else:
            enable_power()
            print("\nSERVO POWER ON / ALL CHANNELS DEFAULTED")
        return None

    if c == "j":
        if not servo_power_enabled:
            print("\nJ6 cannot enable: servo power is OFF. Press SPACE first.")
            return None

        j6_enabled = not j6_enabled

        if j6_enabled:
            set_pwm(J6_CH, J6_OUT_CENTER)
            print("\nJ6 STABILIZATION ON")
        else:
            set_pwm(J6_CH, J6_OUT_CENTER)
            print("\nJ6 STABILIZATION OFF / J6 1460")
        return None

    if c == "g":
        if not servo_power_enabled:
            print("\nCannot listen to controller: servo power is OFF. Press SPACE first.")
            return None

        controller_enabled = True
        print("\nCONTROLLER INPUT ENABLED")
        return None

    if c in ("\n", "\r"):
        return None

    line = c + sys.stdin.readline()
    cmd = line.strip().lower()

    if cmd in ("stop", "hold", "safe"):
        controller_enabled = False

        for ch in CHANNEL_TO_JOINT_INDEX:
            target_us[ch] = default_for_channel(ch)

        j6_input_pwm = CENTER_US
        print("\nCONTROLLER INPUT OFF - TARGETS DEFAULTED")

    elif cmd in ("q", "quit", "exit"):
        return "quit"

    return None

for ch in list(CHANNEL_TO_JOINT_INDEX.keys()) + [J6_CH]:
    kit.servo[ch].set_pulse_width_range(MIN_US, MAX_US)
    set_off(ch)

servo_power(False)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.001)

print(f"Listening on UDP {UDP_PORT}")
print(f"MOSFET GPIO {MOSFET_GPIO}: LOW at startup")
print("space = servo power on/off")
print("g = start listening to controller")
print("j = toggle J6 stabilization")
print("Expected: J1,J2,J3,J4,J5,J6_PWM,Claw,J6_TARGET_ANGLE")
print("All joint PWM values expected from 500 to 2500.")
print("Startup defaults: claw=700, regular servos=1500, J6 continuous rotation neutral=1460.")
print("Also accepts PWM: prefix.")
print()

old_terminal = termios.tcgetattr(sys.stdin)

last_update = time.time()
last_print = time.time()
j6_err = None
j6_pwm = CENTER_US
j6_status = "off"


def _handle_shutdown_signal(signum, frame):
    raise KeyboardInterrupt()


signal.signal(signal.SIGTERM, _handle_shutdown_signal)

try:
    tty.setcbreak(sys.stdin.fileno())

    while True:
        now = time.time()

        if handle_keyboard() == "quit":
            break

        try:
            data, addr = sock.recvfrom(1024)
            line = data.decode(errors="ignore").strip()
            last_raw = line

            if line.startswith("PWM:"):
                line = line[4:]

            parts = line.split(",")

            if len(parts) >= 8:
                vals = [float(x) for x in parts]

                incoming = vals[:7]

                rx_count += 1
                last_packet_time = now

                # 6th value is still J6 manual override PWM.
                j6_input_pwm = clamp_us(incoming[5])

                # 8th value is the live J6 stabilization target angle.
                # No angle limits are applied.
                j6_target_angle_deg = round(float(vals[7]), 2)

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
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
    print("\nStopping.")

    set_pwm(J6_CH, J6_OUT_CENTER)
    time.sleep(0.05)

    for ch in list(CHANNEL_TO_JOINT_INDEX.keys()) + [J6_CH]:
        set_off(ch)

    servo_power(False)
    lgpio.gpiochip_close(gpio_handle)
    print("Servo power OFF.")
