#!/usr/bin/env python3
"""
ROV receiver + 8-thruster mixer + fast pitch/roll stabilization + direct depth hold + direct yaw hold.

This version:
- BAKES IN measured pitch/roll attitude trim.
- Does NOT capture pitch/roll when stabilization is toggled.
- Keeps fixed SCALED_PRESSURE2 depth zero.
- Keeps direct depth hold.
- Keeps 0.75 s depth recapture delay after vertical stick release.
- Keeps direct yaw hold.
- Keeps 0.75 s yaw recapture delay after yaw stick release.
- Keeps combined horizontal + vertical thrust limit of 150%.
- Pitch/roll stabilization corrections use VERTICAL_BIAS.
"""

import json
import math
import socket
import time
from dataclasses import dataclass

from pymavlink import mavutil

from mavlink_rc import (
    MAVLINK_ONBOARD,
    connect_mavlink,
    send_rc_channels_override,
    wait_for_heartbeat,
)


# ============================================================
# USER CONFIG
# ============================================================

MAVLINK_UDP = MAVLINK_ONBOARD
UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 5005

DEFAULT_TELEMETRY_PORT = 5006
TELEMETRY_HZ = 10

# ── Pix6 RC output ──────────────────────────────────────────────────────────
# Pix6 PWM outputs must be set to RCPassThru in QGroundControl (Advanced Params):
#   SERVO1_FUNCTION = 51   → front_left_h   (MOTORS index 0)
#   SERVO2_FUNCTION = 52   → back_left_h    (MOTORS index 1)
#   SERVO3_FUNCTION = 53   → front_left_v   (MOTORS index 2)
#   SERVO4_FUNCTION = 54   → front_right_v  (MOTORS index 3)
#   SERVO5_FUNCTION = 55   → front_right_h  (MOTORS index 4)
#   SERVO6_FUNCTION = 56   → back_right_h   (MOTORS index 5)
#   SERVO7_FUNCTION = 57   → back_right_v   (MOTORS index 6)
#   SERVO8_FUNCTION = 58   → back_left_v    (MOTORS index 7)
# Also set RC_OVERRIDE_TIME to e.g. 3 (seconds) so overrides persist briefly
# if the loop hiccups, but fail-safe to neutral on actual loss of comms.
# ─────────────────────────────────────────────────────────────────────────────

NEUTRAL_US = 1500
MIN_US = 1100
MAX_US = 1900
MAX_PWM_DELTA_US = 400

CONTROL_TIMEOUT_SEC = 0.80
IMU_TIMEOUT_SEC = 2.00
DEPTH_TIMEOUT_SEC = 2.50
LOOP_HZ = 100

MAVLINK_LINK_TIMEOUT_SEC = 3.0
MAVLINK_RECONNECT_COOLDOWN_SEC = 5.0
MAVLINK_MAX_MSGS_PER_POLL = 500
MAVLINK_SENSOR_RESYNC_SEC = 2.0
MAVLINK_SENSOR_RECONNECT_SEC = 5.0

INPUT_DEADZONE = 0.08

DEPTH_HOLD_VERTICAL_DEADZONE = 0.08
DEPTH_RECAPTURE_DELAY_SEC = 0.75

YAW_HOLD_INPUT_DEADZONE = 0.08
YAW_RECAPTURE_DELAY_SEC = 0.75

MOTOR_COMMAND_DEADBAND = 0.004


# ============================================================
# COMBINED THRUST LIMIT
# ============================================================

HORIZONTAL_GROUP_MAX = 1.00
VERTICAL_GROUP_MAX = 1.00
COMBINED_GROUP_TOTAL_LIMIT = 1.50


# ============================================================
# BAKED-IN ATTITUDE ZERO DATA
# ============================================================

AUTO_CAPTURE_ATTITUDE_ON_STABILIZE = False

ROLL_TARGET_DEG = 177.3791
PITCH_TARGET_DEG = -0.4991

ATTITUDE_ZERO_SAMPLE_COUNT = 661
ATTITUDE_ZERO_ROLL_STDEV_DEG = 0.023034
ATTITUDE_ZERO_PITCH_STDEV_DEG = 0.006171
ATTITUDE_ZERO_YAW_MEAN_DEG = -45.8601


# ============================================================
# ATTITUDE HOLD CONFIG
# ============================================================

PITCH_MIX_SIGN = 1.0
ROLL_MIX_SIGN = 1.0

IMU_ROLL_SIGN = 1.0
IMU_PITCH_SIGN = 1.0
IMU_YAW_SIGN = 1.0

ATTITUDE_FILTER_ALPHA = 0.55

PITCH_ERROR_DEADBAND_DEG = 0.5
ROLL_ERROR_DEADBAND_DEG = 0.5

PITCH_KP = 0.030
PITCH_KI = 0.000
PITCH_KD = 0.0015
PITCH_OUTPUT_LIMIT = 0.30

ROLL_KP = 0.030
ROLL_KI = 0.000
ROLL_KD = 0.0015
ROLL_OUTPUT_LIMIT = 0.30

DERIVATIVE_FILTER_ALPHA = 0.65
PID_OUTPUT_FILTER_ALPHA = 0.15
PID_OUTPUT_SLEW_RATE = 8.00
INTEGRAL_DECAY_IN_DEADBAND = 0.90


# ============================================================
# DIRECT YAW HOLD CONFIG
# ============================================================

YAW_KP = 0.030
YAW_OUTPUT_LIMIT = 0.60
YAW_ERROR_DEADBAND_DEG = 1.0

# Flip to -1.0 if yaw hold corrects the wrong direction.
YAW_CORRECTION_SIGN = 1.0


# ============================================================
# BAKED-IN DEPTH ZERO DATA
# ============================================================

PREFERRED_PRESSURE_SOURCE = "SCALED_PRESSURE2"
FIXED_SURFACE_PRESSURE_HPA = 1046.1335
AUTO_ZERO_PRESSURE_AT_STARTUP = False

PRESSURE_ZERO_SAMPLE_COUNT = 364
PRESSURE_ZERO_STDEV_HPA = 0.259105
PRESSURE_ZERO_MIN_HPA = 1045.4000
PRESSURE_ZERO_MAX_HPA = 1046.7000

DEPTH_SIGN = 1.0

WATER_DENSITY_KG_M3 = 997.0
GRAVITY_M_S2 = 9.80665

CLAMP_DEPTH_TO_ZERO = True


# ============================================================
# DIRECT DEPTH HOLD CONFIG
# ============================================================

DEPTH_KP = 2.00
DEPTH_KI = 0.000
DEPTH_KD = 0.000

DEPTH_OUTPUT_LIMIT = 0.85
DEPTH_ERROR_DEADBAND_M = 0.005

# Positive is upward / ascend.
DEPTH_HOLD_UPWARD_FEEDFORWARD = 0.10

# Flip to -1.0 only if depth hold strongly drives the wrong way.
DEPTH_CORRECTION_SIGN = 1.0


# ============================================================
# MANUAL COMMAND SCALE
# ============================================================

MAX_FORWARD_CMD = 1.00
MAX_LATERAL_CMD = 1.00
MAX_YAW_CMD = 1.00
MAX_VERTICAL_CMD = 1.00


# ============================================================
# MOTOR MAP
# ============================================================

MOTORS = {
    "front_left_h":  0,
    "back_left_h":   1,
    "front_left_v":  2,
    "front_right_v": 3,
    "front_right_h": 4,
    "back_right_h":  5,
    "back_right_v":  6,
    "back_left_v":   7,
}

DIR = {
    "front_left_h":   -1,
    "back_left_h":     1,
    "front_left_v":    1,
    "front_right_v":   1,
    "front_right_h":   1,
    "back_right_h":   -1,
    "back_right_v":   -1,
    "back_left_v":    -1,
}

# This affects:
# - manual vertical
# - depth hold
# - pitch stabilization
# - roll stabilization
VERTICAL_BIAS = {
    "front_right_v": 1.20,
    "front_left_v":  1.25,
    "back_right_v":  0.825,
    "back_left_v":   0.825,
}

HORIZONTAL_MOTORS = [
    "front_left_h",
    "back_left_h",
    "front_right_h",
    "back_right_h",
]

VERTICAL_MOTORS = [
    "front_left_v",
    "front_right_v",
    "back_left_v",
    "back_right_v",
]


# ============================================================
# HELPERS
# ============================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_depth(depth_m):
    if CLAMP_DEPTH_TO_ZERO:
        return max(0.0, depth_m)
    return depth_m


def deadzone(x, dz):
    if abs(x) < dz:
        return 0.0
    return x


def apply_center_deadband_command(x, dz):
    if abs(x) < dz:
        return 0.0
    return x


def wrap_180(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def angle_error_deg(target_deg, current_deg):
    return wrap_180(target_deg - current_deg)


def angle_lowpass_deg(previous_deg, new_deg, alpha):
    diff = angle_error_deg(new_deg, previous_deg)
    return wrap_180(previous_deg + alpha * diff)


def smooth_deadband_error(error, deadband):
    if abs(error) <= deadband:
        return 0.0
    return math.copysign(abs(error) - deadband, error)


def pressure_hpa_to_depth_m(pressure_hpa, surface_pressure_hpa):
    pressure_delta_pa = (pressure_hpa - surface_pressure_hpa) * 100.0
    depth_m = pressure_delta_pa / (WATER_DENSITY_KG_M3 * GRAVITY_M_S2)
    depth_m *= DEPTH_SIGN
    return clamp_depth(depth_m)


def group_max(cmds, names):
    return max(abs(cmds[name]) for name in names)


def scale_group(cmds, names, scale):
    for name in names:
        cmds[name] *= scale


def limit_group_to_max(cmds, names, max_allowed):
    max_mag = group_max(cmds, names)

    if max_mag <= max_allowed:
        return

    if max_mag <= 0.000001:
        return

    scale = max_allowed / max_mag
    scale_group(cmds, names, scale)


def limit_motor_groups(cmds):
    limit_group_to_max(cmds, HORIZONTAL_MOTORS, HORIZONTAL_GROUP_MAX)
    limit_group_to_max(cmds, VERTICAL_MOTORS, VERTICAL_GROUP_MAX)

    h = group_max(cmds, HORIZONTAL_MOTORS)
    v = group_max(cmds, VERTICAL_MOTORS)
    total = h + v

    if total > COMBINED_GROUP_TOTAL_LIMIT and total > 0.000001:
        scale = COMBINED_GROUP_TOTAL_LIMIT / total
        scale_group(cmds, HORIZONTAL_MOTORS, scale)
        scale_group(cmds, VERTICAL_MOTORS, scale)

    return cmds


def command_to_pwm_us(motor_name, command):
    command = clamp(command, -1.0, 1.0)
    command = apply_center_deadband_command(command, MOTOR_COMMAND_DEADBAND)

    pwm = NEUTRAL_US + DIR[motor_name] * command * MAX_PWM_DELTA_US
    return int(clamp(pwm, MIN_US, MAX_US))


def msg_id(name, fallback):
    return getattr(mavutil.mavlink, name, fallback)


def fmt_float_or_none(value, digits=2):
    if value is None:
        return "NA"
    return f"{value:.{digits}f}"


# ============================================================
# SMOOTH PID FOR PITCH/ROLL ONLY
# ============================================================

@dataclass
class SmoothPID:
    kp: float
    ki: float
    kd: float
    output_limit: float
    error_deadband: float

    integral: float = 0.0
    previous_error: float | None = None
    derivative_filtered: float = 0.0
    previous_output: float = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.derivative_filtered = 0.0
        self.previous_output = 0.0

    def update(self, raw_error, dt):
        if dt <= 0.0:
            return self.previous_output

        error = smooth_deadband_error(raw_error, self.error_deadband)

        if error == 0.0:
            self.integral *= INTEGRAL_DECAY_IN_DEADBAND
            self.derivative_filtered *= DERIVATIVE_FILTER_ALPHA
            desired_output = 0.0
            self.previous_error = 0.0
        else:
            self.integral += error * dt

            if self.ki > 0.0:
                max_integral = self.output_limit / self.ki
                self.integral = clamp(self.integral, -max_integral, max_integral)
            else:
                self.integral = 0.0

            if self.previous_error is None:
                raw_derivative = 0.0
            else:
                raw_derivative = (error - self.previous_error) / dt

            self.derivative_filtered = (
                DERIVATIVE_FILTER_ALPHA * self.derivative_filtered
                + (1.0 - DERIVATIVE_FILTER_ALPHA) * raw_derivative
            )

            self.previous_error = error

            desired_output = (
                self.kp * error
                + self.ki * self.integral
                + self.kd * self.derivative_filtered
            )

        desired_output = clamp(desired_output, -self.output_limit, self.output_limit)

        filtered_output = (
            PID_OUTPUT_FILTER_ALPHA * self.previous_output
            + (1.0 - PID_OUTPUT_FILTER_ALPHA) * desired_output
        )

        max_step = PID_OUTPUT_SLEW_RATE * dt
        output_step = clamp(
            filtered_output - self.previous_output,
            -max_step,
            max_step,
        )

        output = self.previous_output + output_step
        output = clamp(output, -self.output_limit, self.output_limit)

        self.previous_output = output
        return output


# ============================================================
# MAVLINK ATTITUDE + DEPTH READER
# ============================================================

class MavlinkReader:
    def __init__(self, mavlink_url):
        self.mavlink_url = mavlink_url
        self.master = None
        self.last_any_message = 0.0
        self.last_reconnect_attempt = 0.0
        self.last_sensor_resync = 0.0

        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        self.filtered_roll_deg = 0.0
        self.filtered_pitch_deg = 0.0
        self.filtered_yaw_deg = 0.0

        self.have_attitude = False
        self.last_attitude_update = 0.0

        self.depth_m = 0.0
        self.have_depth = False
        self.last_depth_update = 0.0
        self.depth_source = "none"

        self.pressure_hpa = None
        self.surface_pressure_hpa = FIXED_SURFACE_PRESSURE_HPA
        self.pressure_temperature_c = None

        self.pressure_sources = {
            "SCALED_PRESSURE": {
                "pressure_hpa": None,
                "surface_hpa": None,
                "depth_m": None,
                "last_update": 0.0,
                "temperature_c": None,
            },
            "SCALED_PRESSURE2": {
                "pressure_hpa": None,
                "surface_hpa": FIXED_SURFACE_PRESSURE_HPA,
                "depth_m": None,
                "last_update": 0.0,
                "temperature_c": None,
            },
            "SCALED_PRESSURE3": {
                "pressure_hpa": None,
                "surface_hpa": None,
                "depth_m": None,
                "last_update": 0.0,
                "temperature_c": None,
            },
        }

        self.last_rate_request = 0.0
        self.last_pressure_debug_print = 0.0

        self.battery_voltage_v = None
        self.battery_current_a = None
        self.battery_remaining_pct = None
        self.battery_consumed_mah = None
        self.last_battery_update = 0.0

        self._connect()

        print(
            f"Depth fixed zero: {PREFERRED_PRESSURE_SOURCE} "
            f"surface={FIXED_SURFACE_PRESSURE_HPA:.4f} hPa"
        )

    def _connect(self):
        print(f"Connecting MAVLink: {self.mavlink_url}")
        self.master = connect_mavlink(self.mavlink_url)
        self.last_rate_request = 0.0

    def link_dead(self):
        if self.last_any_message <= 0.0:
            return False
        return (time.time() - self.last_any_message) > MAVLINK_LINK_TIMEOUT_SEC

    def attitude_age_sec(self):
        if self.last_attitude_update <= 0.0:
            return None
        return time.time() - self.last_attitude_update

    def sensor_stream_stalled(self):
        """MAVLink traffic alive but ATTITUDE + depth both missing for a while."""
        if self.last_any_message <= 0.0:
            return False
        if (time.time() - self.last_any_message) > MAVLINK_LINK_TIMEOUT_SEC:
            return False
        att_age = self.attitude_age_sec()
        if att_age is None or att_age < MAVLINK_SENSOR_RECONNECT_SEC:
            return False
        if not self.have_depth:
            return att_age >= MAVLINK_SENSOR_RECONNECT_SEC
        depth_age = time.time() - self.last_depth_update
        return depth_age >= MAVLINK_SENSOR_RECONNECT_SEC

    def sensors_stale(self):
        return self.attitude_stale() and self.depth_stale()

    def try_reconnect(self):
        now = time.time()
        if now - self.last_reconnect_attempt < MAVLINK_RECONNECT_COOLDOWN_SEC:
            return False

        self.last_reconnect_attempt = now
        print("[mavlink] Link lost — reconnecting to MAVProxy...")

        try:
            if self.master is not None:
                close = getattr(self.master, "close", None)
                if callable(close):
                    close()
        except Exception:
            pass

        try:
            self._connect()
            hb = wait_for_heartbeat(self.master, timeout=5.0)
            if hb:
                print(
                    f"[mavlink] Reconnected — system {self.master.target_system}, "
                    f"component {self.master.target_component}"
                )
            else:
                print("[mavlink] Reconnected (no heartbeat yet)")
            self.last_rate_request = 0.0
            return True
        except Exception as e:
            print(f"[mavlink] Reconnect failed: {e}")
            return False

    def resync_sensor_streams(self):
        now = time.time()
        if now - self.last_sensor_resync < MAVLINK_SENSOR_RESYNC_SEC:
            return
        self.last_sensor_resync = now
        self.last_rate_request = 0.0
        print("[mavlink] Re-requesting ATTITUDE / pressure message rates")

    def request_message_rates(self):
        now = time.time()

        if now - self.last_rate_request < 2.0:
            return

        self.last_rate_request = now

        requests = [
            (msg_id("MAVLINK_MSG_ID_ATTITUDE", 30), 20000),
            (msg_id("MAVLINK_MSG_ID_SCALED_PRESSURE", 29), 50000),
            (msg_id("MAVLINK_MSG_ID_SCALED_PRESSURE2", 137), 50000),
            (msg_id("MAVLINK_MSG_ID_SCALED_PRESSURE3", 143), 50000),
            (msg_id("MAVLINK_MSG_ID_SYS_STATUS", 1), 1000000),
            (msg_id("MAVLINK_MSG_ID_BATTERY_STATUS", 147), 1000000),
        ]

        target_system = self.master.target_system or 1
        target_component = self.master.target_component or 1

        for message_id, interval_us in requests:
            try:
                self.master.mav.command_long_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    message_id,
                    interval_us,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception as e:
                print(f"Could not request MAVLink message {message_id}: {e}")

    def update_depth(self, depth_m, source):
        depth_m = clamp_depth(float(depth_m))

        self.depth_m = depth_m
        self.have_depth = True
        self.last_depth_update = time.time()
        self.depth_source = source

    def update_pressure_source(self, source_name, press_abs_hpa, temperature_raw=None):
        now = time.time()

        if source_name not in self.pressure_sources:
            return

        data = self.pressure_sources[source_name]
        press_abs_hpa = float(press_abs_hpa)

        data["pressure_hpa"] = press_abs_hpa
        data["last_update"] = now

        if temperature_raw is not None:
            try:
                data["temperature_c"] = float(temperature_raw) / 100.0
            except Exception:
                pass

        if source_name == PREFERRED_PRESSURE_SOURCE:
            data["surface_hpa"] = FIXED_SURFACE_PRESSURE_HPA
            data["depth_m"] = pressure_hpa_to_depth_m(
                press_abs_hpa,
                FIXED_SURFACE_PRESSURE_HPA,
            )

            self.pressure_hpa = press_abs_hpa
            self.surface_pressure_hpa = FIXED_SURFACE_PRESSURE_HPA
            self.pressure_temperature_c = data["temperature_c"]

            self.update_depth(
                data["depth_m"],
                f"{PREFERRED_PRESSURE_SOURCE}_FIXED_ZERO",
            )
        else:
            data["surface_hpa"] = None
            data["depth_m"] = None

    def print_pressure_debug(self):
        now = time.time()

        if now - self.last_pressure_debug_print < 2.0:
            return

        self.last_pressure_debug_print = now

        parts = []

        for source_name in ["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3"]:
            data = self.pressure_sources[source_name]

            pressure = data["pressure_hpa"]
            surface = data["surface_hpa"]
            depth = data["depth_m"]

            if pressure is None:
                parts.append(f"{source_name}=none")
            else:
                parts.append(
                    f"{source_name}: "
                    f"P={pressure:.1f}hPa "
                    f"zero={fmt_float_or_none(surface, 1)} "
                    f"D={fmt_float_or_none(depth, 2)}m"
                )

        print("PRESSURE DEBUG | " + " | ".join(parts))

    def _process_message(self, msg):
        """Handle one MAVLink message; return True if it updated sensor data."""
        self.last_any_message = time.time()
        msg_type = msg.get_type()

        if msg_type == "ATTITUDE":
            raw_roll = wrap_180(math.degrees(msg.roll) * IMU_ROLL_SIGN)
            raw_pitch = wrap_180(math.degrees(msg.pitch) * IMU_PITCH_SIGN)
            raw_yaw = wrap_180(math.degrees(msg.yaw) * IMU_YAW_SIGN)

            self.roll_deg = raw_roll
            self.pitch_deg = raw_pitch
            self.yaw_deg = raw_yaw

            if not self.have_attitude:
                self.filtered_roll_deg = raw_roll
                self.filtered_pitch_deg = raw_pitch
                self.filtered_yaw_deg = raw_yaw
                self.have_attitude = True
            else:
                self.filtered_roll_deg = angle_lowpass_deg(
                    self.filtered_roll_deg,
                    raw_roll,
                    ATTITUDE_FILTER_ALPHA,
                )
                self.filtered_pitch_deg = angle_lowpass_deg(
                    self.filtered_pitch_deg,
                    raw_pitch,
                    ATTITUDE_FILTER_ALPHA,
                )
                self.filtered_yaw_deg = angle_lowpass_deg(
                    self.filtered_yaw_deg,
                    raw_yaw,
                    ATTITUDE_FILTER_ALPHA,
                )

            self.last_attitude_update = time.time()
            return True

        if msg_type in ["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3"]:
            temperature_raw = getattr(msg, "temperature", None)
            self.update_pressure_source(
                msg_type,
                float(msg.press_abs),
                temperature_raw,
            )
            return True

        if msg_type == "SYS_STATUS":
            voltage_mv = int(getattr(msg, "voltage_battery", -1))
            current_ca = int(getattr(msg, "current_battery", -1))
            remaining = int(getattr(msg, "battery_remaining", -1))
            if voltage_mv > 0:
                self.battery_voltage_v = voltage_mv / 1000.0
            if current_ca != -1:
                self.battery_current_a = current_ca / 100.0
            if 0 <= remaining <= 100:
                self.battery_remaining_pct = float(remaining)
            self.last_battery_update = time.time()
            return True

        if msg_type == "BATTERY_STATUS":
            voltages = getattr(msg, "voltages", None)
            if voltages and len(voltages) > 0 and int(voltages[0]) != 65535:
                total_mv = sum(int(v) for v in voltages if int(v) not in (-1, 65535))
                if total_mv > 0:
                    self.battery_voltage_v = total_mv / 1000.0
            current_ca = int(getattr(msg, "current_battery", -1))
            if current_ca != -1:
                self.battery_current_a = current_ca / 100.0
            consumed = int(getattr(msg, "current_consumed", -1))
            if consumed != -1:
                self.battery_consumed_mah = float(consumed)
            remaining = int(getattr(msg, "battery_remaining", -1))
            if 0 <= remaining <= 100:
                self.battery_remaining_pct = float(remaining)
            self.last_battery_update = time.time()
            return True

        return False

    def poll(self):
        got_new = False

        self.request_message_rates()

        # Priority pass — drain ATTITUDE / pressure before RC/heartbeat flood.
        for _ in range(100):
            msg = self.master.recv_match(
                type=[
                    "ATTITUDE",
                    "SCALED_PRESSURE",
                    "SCALED_PRESSURE2",
                    "SCALED_PRESSURE3",
                ],
                blocking=False,
            )
            if msg is None:
                break
            if self._process_message(msg):
                got_new = True

        msgs_read = 0
        while msgs_read < MAVLINK_MAX_MSGS_PER_POLL:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            msgs_read += 1
            if self._process_message(msg):
                got_new = True

        self.print_pressure_debug()
        return got_new

    def attitude_stale(self):
        if not self.have_attitude:
            return True
        return (time.time() - self.last_attitude_update) > IMU_TIMEOUT_SEC

    def depth_stale(self):
        if not self.have_depth:
            return True
        return (time.time() - self.last_depth_update) > DEPTH_TIMEOUT_SEC


# ============================================================
# PIXHAWK RC OUTPUT
# ============================================================

class PixhawkOutput:
    """
    Drives the Pix6's PWM outputs via MAVLink RC_CHANNELS_OVERRIDE, bypassing
    ArduSub's built-in motor mixer entirely.  All stabilization, mixing, and
    gain logic stays in this Python process; the Pix6 is a dumb PWM output
    board + IMU/depth sensor.

    Requires SERVO1..8_FUNCTION = 51..58 (RCPassThru) in QGroundControl.
    MOTORS index 0-7 maps directly to Pix6 outputs 1-8 (RC channels 1-8).
    """

    IGNORE = 65535  # MAVLink sentinel: leave this RC channel unchanged

    def __init__(self, master):
        self.master = master
        self._last_heartbeat = 0.0

    def set_master(self, master):
        self.master = master

    def _target(self):
        ts = self.master.target_system or 1
        tc = self.master.target_component or 1
        return ts, tc

    def _send_heartbeat_if_due(self):
        """
        Periodically send a GCS heartbeat so ArduSub accepts RC overrides.
        MAVProxy also heartbeats, but this ensures our system ID is known.
        """
        now = time.time()
        if now - self._last_heartbeat < 1.0:
            return
        self._last_heartbeat = now
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception:
            pass

    def send_pwm(self, pwm_by_name: dict):
        """
        Send one RC_CHANNELS_OVERRIDE frame covering all 8 thruster channels.
        pwm_by_name maps motor name → PWM microseconds (1100-1900).
        Channels not in pwm_by_name are set to IGNORE (unchanged on the FC).
        """
        self._send_heartbeat_if_due()

        rc = [self.IGNORE] * 8      # all 8 motors live on channels 1-8 (indices 0-7)
        for motor_name, pwm in pwm_by_name.items():
            idx = MOTORS[motor_name]          # 0-based → rc[0..7] = RC ch 1..8
            rc[idx] = int(clamp(pwm, MIN_US, MAX_US))

        try:
            send_rc_channels_override(self.master, rc, ignore=self.IGNORE)
        except Exception as e:
            print(f"[WARN] RC_CHANNELS_OVERRIDE send failed: {e}")

    def neutral_all(self):
        """Set all thruster channels to neutral (1500 µs)."""
        self.send_pwm({name: NEUTRAL_US for name in MOTORS})

    def stop_all(self):
        """Alias for neutral_all — stops all thrusters without disarming."""
        self.neutral_all()


# ============================================================
# UDP CONTROL RECEIVER
# ============================================================

class ControlReceiver:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((ip, port))
        self.sock.setblocking(False)

        self.forward = 0.0
        self.lateral = 0.0
        self.yaw = 0.0
        self.vertical = 0.0

        self.stabilize = False
        self.depth_hold = False
        self.yaw_hold = False
        self.calibrate_imu = False

        self.gain_percent = 100

        self.telemetry_port = DEFAULT_TELEMETRY_PORT
        self.telemetry_addr = None

        self.last_packet_time = 0.0
        self.last_sender = None
        self.seq = 0

        print(f"Listening for controller UDP on {ip}:{port}")

    def poll(self):
        got_new = False

        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break

            try:
                packet = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            self.forward = deadzone(float(packet.get("forward", 0.0)), INPUT_DEADZONE)
            self.lateral = deadzone(float(packet.get("lateral", 0.0)), INPUT_DEADZONE)
            self.yaw = deadzone(float(packet.get("yaw", 0.0)), INPUT_DEADZONE)
            self.vertical = deadzone(float(packet.get("vertical", 0.0)), INPUT_DEADZONE)

            self.stabilize = bool(packet.get("stabilize", False))
            self.depth_hold = bool(packet.get("depth_hold", False))
            self.yaw_hold = bool(packet.get("yaw_hold", False))
            self.calibrate_imu = bool(packet.get("calibrate_imu", False))

            self.gain_percent = int(packet.get("gain_percent", self.gain_percent))
            self.seq = int(packet.get("seq", self.seq))

            self.forward = clamp(self.forward, -1.0, 1.0)
            self.lateral = clamp(self.lateral, -1.0, 1.0)
            self.yaw = clamp(self.yaw, -1.0, 1.0)
            self.vertical = clamp(self.vertical, -1.0, 1.0)

            self.telemetry_port = int(packet.get("telemetry_port", DEFAULT_TELEMETRY_PORT))
            self.telemetry_addr = (addr[0], self.telemetry_port)

            self.last_packet_time = time.time()
            self.last_sender = addr
            got_new = True

        return got_new

    def timed_out(self):
        return (time.time() - self.last_packet_time) > CONTROL_TIMEOUT_SEC

    def zero_controls(self):
        self.forward = 0.0
        self.lateral = 0.0
        self.yaw = 0.0
        self.vertical = 0.0
        self.stabilize = False
        self.depth_hold = False
        self.yaw_hold = False


# ============================================================
# MIXER
# ============================================================

def mix_thrusters(forward, lateral, yaw, vertical, pitch_correction, roll_correction):
    f = forward * MAX_FORWARD_CMD
    l = lateral * MAX_LATERAL_CMD
    y = yaw * MAX_YAW_CMD
    v = vertical * MAX_VERTICAL_CMD

    cmds = {name: 0.0 for name in MOTORS.keys()}

    # Horizontal thrusters.
    cmds["front_left_h"]  = f + l + y
    cmds["back_left_h"]   = f - l + y
    cmds["front_right_h"] = f - l - y
    cmds["back_right_h"]  = f + l - y

    # Vertical thrust with bias.
    # This includes manual vertical and depth hold.
    cmds["front_left_v"]  = v * VERTICAL_BIAS["front_left_v"]
    cmds["front_right_v"] = v * VERTICAL_BIAS["front_right_v"]
    cmds["back_left_v"]   = v * VERTICAL_BIAS["back_left_v"]
    cmds["back_right_v"]  = v * VERTICAL_BIAS["back_right_v"]

    # Pitch stabilization WITH vertical bias.
    p = pitch_correction * PITCH_MIX_SIGN

    cmds["front_left_v"]  += p * VERTICAL_BIAS["front_left_v"]
    cmds["front_right_v"] += p * VERTICAL_BIAS["front_right_v"]
    cmds["back_left_v"]   -= p * VERTICAL_BIAS["back_left_v"]
    cmds["back_right_v"]  -= p * VERTICAL_BIAS["back_right_v"]

    # Roll stabilization WITH vertical bias.
    r = roll_correction * ROLL_MIX_SIGN

    cmds["front_left_v"]  -= r * VERTICAL_BIAS["front_left_v"]
    cmds["back_left_v"]   -= r * VERTICAL_BIAS["back_left_v"]
    cmds["front_right_v"] += r * VERTICAL_BIAS["front_right_v"]
    cmds["back_right_v"]  += r * VERTICAL_BIAS["back_right_v"]

    for name in cmds:
        cmds[name] = apply_center_deadband_command(cmds[name], MOTOR_COMMAND_DEADBAND)

    return limit_motor_groups(cmds)


# ============================================================
# TELEMETRY
# ============================================================

def send_telemetry(control, payload):
    if control.telemetry_addr is None:
        return

    try:
        control.sock.sendto(
            json.dumps(payload).encode("utf-8"),
            control.telemetry_addr,
        )
    except Exception:
        pass


# ============================================================
# DIRECT DEPTH HOLD
# ============================================================

def calculate_direct_depth_correction(depth_hold_active, current_depth_m, hold_depth_m):
    depth_error = clamp_depth(current_depth_m) - clamp_depth(hold_depth_m)

    if not depth_hold_active:
        return depth_error, 0.0, 0.0, 0.0

    if abs(depth_error) < DEPTH_ERROR_DEADBAND_M:
        raw_correction = 0.0
    else:
        raw_correction = DEPTH_CORRECTION_SIGN * DEPTH_KP * depth_error

    feedforward = DEPTH_HOLD_UPWARD_FEEDFORWARD

    correction = raw_correction + feedforward
    correction = clamp(correction, -DEPTH_OUTPUT_LIMIT, DEPTH_OUTPUT_LIMIT)

    return depth_error, raw_correction, feedforward, correction


# ============================================================
# DIRECT YAW HOLD
# ============================================================

def calculate_direct_yaw_correction(yaw_hold_active, current_yaw_deg, hold_yaw_deg):
    yaw_error = angle_error_deg(hold_yaw_deg, current_yaw_deg)

    if not yaw_hold_active:
        return yaw_error, 0.0, 0.0

    if abs(yaw_error) < YAW_ERROR_DEADBAND_DEG:
        raw_correction = 0.0
    else:
        raw_correction = YAW_CORRECTION_SIGN * YAW_KP * yaw_error

    correction = clamp(raw_correction, -YAW_OUTPUT_LIMIT, YAW_OUTPUT_LIMIT)

    return yaw_error, raw_correction, correction


# ============================================================
# MAIN
# ============================================================

def main():
    control = ControlReceiver(UDP_LISTEN_IP, UDP_LISTEN_PORT)
    mav = MavlinkReader(MAVLINK_UDP)

    print("Waiting for MAVLink heartbeat from Pix6 via MAVProxy...")
    hb = wait_for_heartbeat(mav.master, timeout=20)
    if hb:
        print(f"Heartbeat received — system {mav.master.target_system}, "
              f"component {mav.master.target_component}.")
    else:
        print("[INFO] No heartbeat yet — continuing. Will retry in main loop.")

    pixhawk = PixhawkOutput(mav.master)
    pixhawk.neutral_all()
    print("Pix6 RC output ready. All channels set to neutral (1500 µs).")

    pitch_pid = SmoothPID(
        PITCH_KP,
        PITCH_KI,
        PITCH_KD,
        PITCH_OUTPUT_LIMIT,
        PITCH_ERROR_DEADBAND_DEG,
    )

    roll_pid = SmoothPID(
        ROLL_KP,
        ROLL_KI,
        ROLL_KD,
        ROLL_OUTPUT_LIMIT,
        ROLL_ERROR_DEADBAND_DEG,
    )

    last_time = time.time()
    last_print = 0.0
    last_telemetry = 0.0

    previous_stabilize = False
    previous_calibrate_imu = False
    previous_depth_hold_request = False
    previous_manual_vertical_active = False

    previous_yaw_hold_request = False
    previous_manual_yaw_active = False

    depth_recapture_pending = False
    depth_recapture_release_time = 0.0
    depth_recapture_time_remaining = 0.0

    yaw_recapture_pending = False
    yaw_recapture_release_time = 0.0
    yaw_recapture_time_remaining = 0.0

    pitch_hold_target_deg = PITCH_TARGET_DEG
    roll_hold_target_deg = ROLL_TARGET_DEG
    imu_targets_calibrated = False

    hold_depth_m = 0.0
    hold_yaw_deg = 0.0

    print("Receiver running.")
    print("Pitch/roll stabilization uses vertical thrusters only.")
    print("Pitch/roll stabilization uses VERTICAL_BIAS.")
    print("Pitch/roll targets are BAKED IN, not captured on stabilize toggle.")
    print(f"  ROLL_TARGET_DEG  = {ROLL_TARGET_DEG:.4f}")
    print(f"  PITCH_TARGET_DEG = {PITCH_TARGET_DEG:.4f}")
    print(f"  attitude samples = {ATTITUDE_ZERO_SAMPLE_COUNT}")
    print(f"  roll stdev       = {ATTITUDE_ZERO_ROLL_STDEV_DEG:.6f} deg")
    print(f"  pitch stdev      = {ATTITUDE_ZERO_PITCH_STDEV_DEG:.6f} deg")
    print("Yaw hold uses horizontal yaw mixer.")
    print(f"Depth source locked to: {PREFERRED_PRESSURE_SOURCE}")
    print(f"Fixed surface pressure: {FIXED_SURFACE_PRESSURE_HPA:.4f} hPa")
    print("Depth hold is DIRECT P, no depth PID/filtering.")
    print("Yaw hold is DIRECT P, no yaw PID/filtering.")
    print(f"Depth recapture delay: {DEPTH_RECAPTURE_DELAY_SEC:.2f} s")
    print(f"Yaw recapture delay:   {YAW_RECAPTURE_DELAY_SEC:.2f} s")
    print(f"DEPTH_KP={DEPTH_KP:.2f}, DEPTH_FF={DEPTH_HOLD_UPWARD_FEEDFORWARD:.2f}, DEPTH_LIMIT={DEPTH_OUTPUT_LIMIT:.2f}")
    print(f"YAW_KP={YAW_KP:.3f}, YAW_LIMIT={YAW_OUTPUT_LIMIT:.2f}, YAW_SIGN={YAW_CORRECTION_SIGN:+.1f}")
    print("Vertical bias:")
    for name, bias in VERTICAL_BIAS.items():
        print(f"  {name}: {bias:.3f}")
    print("Combined thrust limit:")
    print(f"  horizontal group max = {HORIZONTAL_GROUP_MAX:.2f}")
    print(f"  vertical group max   = {VERTICAL_GROUP_MAX:.2f}")
    print(f"  combined H + V max   = {COMBINED_GROUP_TOTAL_LIMIT:.2f}")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            mav.poll()
            control.poll()

            if mav.link_dead() or mav.sensor_stream_stalled():
                if mav.try_reconnect():
                    pixhawk.set_master(mav.master)
            elif mav.attitude_stale() or mav.depth_stale():
                mav.resync_sensor_streams()

            control_timed_out = control.timed_out()
            attitude_stale = mav.attitude_stale()
            depth_stale = mav.depth_stale()

            if control_timed_out:
                control.zero_controls()
                stabilize_active = False
                depth_hold_request = False
                yaw_hold_request = False
            elif attitude_stale:
                stabilize_active = False
                depth_hold_request = control.depth_hold
                yaw_hold_request = False
            else:
                stabilize_active = control.stabilize
                depth_hold_request = control.depth_hold
                yaw_hold_request = control.yaw_hold

            # ----------------------------------------------------
            # IMU zero / calibration (rising edge on calibrate_imu).
            # ----------------------------------------------------
            calibrate_imu_request = control.calibrate_imu
            if calibrate_imu_request and not previous_calibrate_imu:
                if attitude_stale:
                    print("IMU CALIBRATE SKIPPED — attitude stale")
                else:
                    pitch_hold_target_deg = mav.filtered_pitch_deg
                    roll_hold_target_deg = mav.filtered_roll_deg
                    imu_targets_calibrated = True
                    pitch_pid.reset()
                    roll_pid.reset()
                    print(
                        f"IMU CALIBRATED | "
                        f"pitch target={pitch_hold_target_deg:.1f} deg | "
                        f"roll target={roll_hold_target_deg:.1f} deg"
                    )

            previous_calibrate_imu = calibrate_imu_request

            # ----------------------------------------------------
            # Pitch / roll stabilization target handling.
            # ----------------------------------------------------
            if stabilize_active and not previous_stabilize:
                pitch_pid.reset()
                roll_pid.reset()

                if AUTO_CAPTURE_ATTITUDE_ON_STABILIZE:
                    pitch_hold_target_deg = mav.filtered_pitch_deg
                    roll_hold_target_deg = mav.filtered_roll_deg
                    imu_targets_calibrated = True
                    print(
                        f"STABILIZATION ON | captured "
                        f"pitch target={pitch_hold_target_deg:.1f} deg | "
                        f"roll target={roll_hold_target_deg:.1f} deg"
                    )
                elif imu_targets_calibrated:
                    print(
                        f"STABILIZATION ON | calibrated "
                        f"pitch target={pitch_hold_target_deg:.1f} deg | "
                        f"roll target={roll_hold_target_deg:.1f} deg"
                    )
                else:
                    pitch_hold_target_deg = PITCH_TARGET_DEG
                    roll_hold_target_deg = ROLL_TARGET_DEG
                    print(
                        f"STABILIZATION ON | baked "
                        f"pitch target={pitch_hold_target_deg:.1f} deg | "
                        f"roll target={roll_hold_target_deg:.1f} deg"
                    )

            if not stabilize_active and previous_stabilize:
                pitch_pid.reset()
                roll_pid.reset()
                print("STABILIZATION OFF")

            previous_stabilize = stabilize_active

            # ----------------------------------------------------
            # Pitch / roll correction.
            # ----------------------------------------------------
            pitch_correction = 0.0
            roll_correction = 0.0
            pitch_error = 0.0
            roll_error = 0.0

            if stabilize_active:
                pitch_error = angle_error_deg(
                    pitch_hold_target_deg,
                    mav.filtered_pitch_deg,
                )
                pitch_correction = pitch_pid.update(pitch_error, dt)

                roll_error = angle_error_deg(
                    mav.filtered_roll_deg,
                    roll_hold_target_deg,
                )
                roll_correction = roll_pid.update(roll_error, dt)

            # ----------------------------------------------------
            # Direct depth hold with release delay.
            # ----------------------------------------------------
            manual_vertical_active = abs(control.vertical) > DEPTH_HOLD_VERTICAL_DEADZONE

            depth_hold_active = False
            depth_error = 0.0
            depth_raw_correction = 0.0
            depth_feedforward = 0.0
            depth_correction = 0.0
            depth_recapture_time_remaining = 0.0

            if not depth_stale:
                current_depth_m = clamp_depth(mav.depth_m)
            else:
                current_depth_m = clamp_depth(hold_depth_m)

            if not depth_hold_request:
                depth_hold_active = False
                depth_correction = 0.0
                depth_recapture_pending = False
                depth_recapture_release_time = 0.0

                if not depth_stale:
                    hold_depth_m = clamp_depth(current_depth_m)

            elif depth_stale:
                depth_hold_active = False
                depth_correction = 0.0
                depth_recapture_pending = False
                depth_recapture_release_time = 0.0

            else:
                if depth_hold_request and not previous_depth_hold_request:
                    hold_depth_m = clamp_depth(current_depth_m)
                    depth_recapture_pending = False
                    depth_recapture_release_time = 0.0
                    print(f"DEPTH HOLD ON | hold_depth={hold_depth_m:.2f} m")

                if manual_vertical_active:
                    depth_hold_active = False
                    depth_correction = 0.0
                    depth_recapture_pending = True
                    depth_recapture_release_time = 0.0

                else:
                    if previous_manual_vertical_active:
                        depth_recapture_pending = True
                        depth_recapture_release_time = now
                        print(
                            f"VERTICAL RELEASED | waiting "
                            f"{DEPTH_RECAPTURE_DELAY_SEC:.2f}s before depth recapture"
                        )

                    if depth_recapture_pending:
                        if depth_recapture_release_time <= 0.0:
                            depth_recapture_release_time = now

                        elapsed = now - depth_recapture_release_time
                        depth_recapture_time_remaining = max(
                            0.0,
                            DEPTH_RECAPTURE_DELAY_SEC - elapsed,
                        )

                        if elapsed < DEPTH_RECAPTURE_DELAY_SEC:
                            depth_hold_active = False
                            depth_correction = 0.0
                        else:
                            hold_depth_m = clamp_depth(current_depth_m)
                            depth_recapture_pending = False
                            depth_recapture_release_time = 0.0
                            depth_recapture_time_remaining = 0.0
                            depth_hold_active = True

                            print(f"DEPTH HOLD RE-CAPTURE AFTER DELAY | hold_depth={hold_depth_m:.2f} m")

                            (
                                depth_error,
                                depth_raw_correction,
                                depth_feedforward,
                                depth_correction,
                            ) = calculate_direct_depth_correction(
                                depth_hold_active=True,
                                current_depth_m=current_depth_m,
                                hold_depth_m=hold_depth_m,
                            )

                    else:
                        depth_hold_active = True

                        (
                            depth_error,
                            depth_raw_correction,
                            depth_feedforward,
                            depth_correction,
                        ) = calculate_direct_depth_correction(
                            depth_hold_active=True,
                            current_depth_m=current_depth_m,
                            hold_depth_m=hold_depth_m,
                        )

            if not depth_hold_request and previous_depth_hold_request:
                print("DEPTH HOLD OFF")

            previous_depth_hold_request = depth_hold_request
            previous_manual_vertical_active = manual_vertical_active

            # ----------------------------------------------------
            # Direct yaw hold with release delay.
            # ----------------------------------------------------
            manual_yaw_active = abs(control.yaw) > YAW_HOLD_INPUT_DEADZONE

            yaw_hold_active = False
            yaw_error = 0.0
            yaw_raw_correction = 0.0
            yaw_correction = 0.0
            yaw_recapture_time_remaining = 0.0

            current_yaw_deg = mav.filtered_yaw_deg

            if not yaw_hold_request:
                yaw_hold_active = False
                yaw_correction = 0.0
                yaw_recapture_pending = False
                yaw_recapture_release_time = 0.0
                hold_yaw_deg = current_yaw_deg

            else:
                if yaw_hold_request and not previous_yaw_hold_request:
                    hold_yaw_deg = current_yaw_deg
                    yaw_recapture_pending = False
                    yaw_recapture_release_time = 0.0
                    print(f"YAW HOLD ON | hold_yaw={hold_yaw_deg:.1f} deg")

                if manual_yaw_active:
                    yaw_hold_active = False
                    yaw_correction = 0.0
                    yaw_recapture_pending = True
                    yaw_recapture_release_time = 0.0

                else:
                    if previous_manual_yaw_active:
                        yaw_recapture_pending = True
                        yaw_recapture_release_time = now
                        print(
                            f"YAW RELEASED | waiting "
                            f"{YAW_RECAPTURE_DELAY_SEC:.2f}s before yaw recapture"
                        )

                    if yaw_recapture_pending:
                        if yaw_recapture_release_time <= 0.0:
                            yaw_recapture_release_time = now

                        elapsed = now - yaw_recapture_release_time
                        yaw_recapture_time_remaining = max(
                            0.0,
                            YAW_RECAPTURE_DELAY_SEC - elapsed,
                        )

                        if elapsed < YAW_RECAPTURE_DELAY_SEC:
                            yaw_hold_active = False
                            yaw_correction = 0.0
                        else:
                            hold_yaw_deg = current_yaw_deg
                            yaw_recapture_pending = False
                            yaw_recapture_release_time = 0.0
                            yaw_recapture_time_remaining = 0.0
                            yaw_hold_active = True

                            print(f"YAW HOLD RE-CAPTURE AFTER DELAY | hold_yaw={hold_yaw_deg:.1f} deg")

                            (
                                yaw_error,
                                yaw_raw_correction,
                                yaw_correction,
                            ) = calculate_direct_yaw_correction(
                                yaw_hold_active=True,
                                current_yaw_deg=current_yaw_deg,
                                hold_yaw_deg=hold_yaw_deg,
                            )

                    else:
                        yaw_hold_active = True

                        (
                            yaw_error,
                            yaw_raw_correction,
                            yaw_correction,
                        ) = calculate_direct_yaw_correction(
                            yaw_hold_active=True,
                            current_yaw_deg=current_yaw_deg,
                            hold_yaw_deg=hold_yaw_deg,
                        )

            if not yaw_hold_request and previous_yaw_hold_request:
                print("YAW HOLD OFF")

            previous_yaw_hold_request = yaw_hold_request
            previous_manual_yaw_active = manual_yaw_active

            # ----------------------------------------------------
            # Final mixer inputs.
            # ----------------------------------------------------
            vertical_for_mixer = clamp(
                control.vertical + depth_correction,
                -1.0,
                1.0,
            )

            yaw_for_mixer = clamp(
                control.yaw + yaw_correction,
                -1.0,
                1.0,
            )

            cmds = mix_thrusters(
                forward=control.forward,
                lateral=control.lateral,
                yaw=yaw_for_mixer,
                vertical=vertical_for_mixer,
                pitch_correction=pitch_correction,
                roll_correction=roll_correction,
            )

            h_group = group_max(cmds, HORIZONTAL_MOTORS)
            v_group = group_max(cmds, VERTICAL_MOTORS)

            pwm_out = {
                motor_name: command_to_pwm_us(motor_name, cmd)
                for motor_name, cmd in cmds.items()
            }
            pixhawk.send_pwm(pwm_out)

            # ----------------------------------------------------
            # Telemetry back to sender.
            # ----------------------------------------------------
            if now - last_telemetry >= 1.0 / TELEMETRY_HZ:
                last_telemetry = now

                telemetry = {
                    "time": now,
                    "state": "OK",
                    "control_timeout": control_timed_out,
                    "attitude_stale": attitude_stale,
                    "depth_stale": depth_stale,
                    "mavlink_link_dead": mav.link_dead(),
                    "mavlink_last_rx_age_sec": (
                        None if mav.last_any_message <= 0.0
                        else round(time.time() - mav.last_any_message, 2)
                    ),
                    "attitude_age_sec": (
                        None if mav.attitude_age_sec() is None
                        else round(mav.attitude_age_sec(), 2)
                    ),

                    "stabilize": stabilize_active,

                    "pitch_target_deg": pitch_hold_target_deg,
                    "roll_target_deg": roll_hold_target_deg,
                    "pitch_error_deg": pitch_error,
                    "roll_error_deg": roll_error,

                    "depth_hold_request": depth_hold_request,
                    "depth_hold_active": depth_hold_active,
                    "depth_recapture_pending": depth_recapture_pending,
                    "depth_recapture_time_remaining": depth_recapture_time_remaining,
                    "manual_vertical_active": manual_vertical_active,
                    "depth_m": None if depth_stale else clamp_depth(mav.depth_m),
                    "hold_depth_m": clamp_depth(hold_depth_m),
                    "depth_error_m": depth_error,
                    "depth_raw_correction": depth_raw_correction,
                    "depth_feedforward": depth_feedforward,
                    "depth_correction": depth_correction,
                    "depth_source": mav.depth_source,

                    "yaw_hold_request": yaw_hold_request,
                    "yaw_hold_active": yaw_hold_active,
                    "yaw_recapture_pending": yaw_recapture_pending,
                    "yaw_recapture_time_remaining": yaw_recapture_time_remaining,
                    "manual_yaw_active": manual_yaw_active,
                    "yaw_deg": mav.filtered_yaw_deg,
                    "hold_yaw_deg": hold_yaw_deg,
                    "yaw_error_deg": yaw_error,
                    "yaw_raw_correction": yaw_raw_correction,
                    "yaw_correction": yaw_correction,
                    "yaw_for_mixer": yaw_for_mixer,

                    "pressure_hpa": mav.pressure_hpa,
                    "surface_pressure_hpa": mav.surface_pressure_hpa,
                    "pressure_temperature_c": mav.pressure_temperature_c,

                    "battery_voltage_v": mav.battery_voltage_v,
                    "battery_current_a": mav.battery_current_a,
                    "battery_remaining_pct": mav.battery_remaining_pct,
                    "battery_consumed_mah": mav.battery_consumed_mah,

                    "roll_deg": mav.filtered_roll_deg,
                    "pitch_deg": mav.filtered_pitch_deg,
                    "pcorr": pitch_correction,
                    "rcorr": roll_correction,
                    "gain_percent": control.gain_percent,
                    "horizontal_group": h_group,
                    "vertical_group": v_group,
                    "combined_group": h_group + v_group,
                    "vertical_for_mixer": vertical_for_mixer,
                    "vFL": cmds["front_left_v"],
                    "vFR": cmds["front_right_v"],
                    "vBL": cmds["back_left_v"],
                    "vBR": cmds["back_right_v"],
                }

                if control_timed_out:
                    telemetry["state"] = "CONTROL_TIMEOUT"
                elif attitude_stale:
                    telemetry["state"] = "IMU_STALE"
                elif depth_stale:
                    telemetry["state"] = "DEPTH_STALE"

                send_telemetry(control, telemetry)

            # ----------------------------------------------------
            # Terminal print.
            # ----------------------------------------------------
            if now - last_print > 0.5:
                last_print = now

                if control_timed_out:
                    state_text = "CONTROL_TIMEOUT"
                elif attitude_stale:
                    state_text = "IMU_STALE"
                elif depth_stale:
                    state_text = "DEPTH_STALE"
                else:
                    state_text = "OK"

                depth_text = "NA" if depth_stale else f"{clamp_depth(mav.depth_m):+.2f}"
                pressure_text = "NA" if mav.pressure_hpa is None else f"{mav.pressure_hpa:.1f}"
                zero_text = "NA" if mav.surface_pressure_hpa is None else f"{mav.surface_pressure_hpa:.1f}"

                print(
                    f"{state_text} | "
                    f"stab={stabilize_active} | "
                    f"targetR={roll_hold_target_deg:+.1f} "
                    f"targetP={pitch_hold_target_deg:+.1f} | "
                    f"dh_req={depth_hold_request} "
                    f"dh_act={depth_hold_active} "
                    f"drecap={depth_recapture_pending} "
                    f"dwait={depth_recapture_time_remaining:.2f}s "
                    f"manV={manual_vertical_active} | "
                    f"yh_req={yaw_hold_request} "
                    f"yh_act={yaw_hold_active} "
                    f"yrecap={yaw_recapture_pending} "
                    f"ywait={yaw_recapture_time_remaining:.2f}s "
                    f"manY={manual_yaw_active} | "
                    f"F={control.forward:+.2f} "
                    f"L={control.lateral:+.2f} "
                    f"Y={control.yaw:+.2f} "
                    f"Ymix={yaw_for_mixer:+.2f} "
                    f"V={control.vertical:+.2f} "
                    f"Vmix={vertical_for_mixer:+.2f} | "
                    f"Hgrp={h_group:.2f} "
                    f"Vgrp={v_group:.2f} "
                    f"sum={h_group + v_group:.2f} | "
                    f"depth={depth_text} "
                    f"holdD={clamp_depth(hold_depth_m):+.2f} "
                    f"derr={depth_error:+.2f} "
                    f"dcorr={depth_correction:+.3f} | "
                    f"yaw={mav.filtered_yaw_deg:+.1f} "
                    f"holdY={hold_yaw_deg:+.1f} "
                    f"yerr={yaw_error:+.1f} "
                    f"ycorr={yaw_correction:+.3f} | "
                    f"src={mav.depth_source} "
                    f"P={pressure_text}hPa "
                    f"zero={zero_text}hPa | "
                    f"roll={mav.filtered_roll_deg:+.1f} "
                    f"pitch={mav.filtered_pitch_deg:+.1f} "
                    f"perr={pitch_error:+.1f} "
                    f"rerr={roll_error:+.1f} | "
                    f"pcorr={pitch_correction:+.3f} "
                    f"rcorr={roll_correction:+.3f} | "
                    f"vFL={cmds['front_left_v']:+.3f} "
                    f"vFR={cmds['front_right_v']:+.3f} "
                    f"vBL={cmds['back_left_v']:+.3f} "
                    f"vBR={cmds['back_right_v']:+.3f}"
                )

            sleep_time = max(0.0, (1.0 / LOOP_HZ) - (time.time() - now))
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping. Setting all Pix6 channels to neutral.")
        pixhawk.neutral_all()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
