#!/usr/bin/env python3
"""
MOSFET GPIO toggle — run on the Raspberry Pi
=============================================
Press a key to drive GPIO17 (servo power rail MOSFET) on or off.
Standalone script — only needs lgpio (sudo apt install python3-lgpio).

Usage:
    python3 mosfet_key_pi.py
    python3 mosfet_key_pi.py --gpio 17

Keys:
    SPACE or M   toggle ON ↔ OFF
    O            turn ON
    F            turn OFF
    Q            turn OFF and exit
"""

import argparse
import select
import sys
import termios
import time
import tty

try:
    import lgpio
except ImportError:
    print("lgpio not found — install with:  sudo apt install python3-lgpio", flush=True)
    sys.exit(1)

GPIO = 17


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def show_state(on: bool, gpio: int):
    label = "ON " if on else "OFF"
    bar = "████" if on else "····"
    print(
        f"\r  GPIO{gpio} MOSFET {label}  [{bar}]  "
        f"(SPACE/M=toggle  O=on  F=off  Q=quit)  ",
        end="",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Toggle MOSFET on GPIO17 from the Pi keyboard")
    parser.add_argument("--gpio", type=int, default=GPIO, help=f"GPIO pin (default {GPIO})")
    args = parser.parse_args()
    gpio = args.gpio

    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_output(h, gpio, 0)
    on = False

    print(f"MOSFET toggle on GPIO{gpio} — keyboard must be attached to the Pi.")
    print("Keys:  SPACE/M = toggle   O = on   F = off   Q = quit\n")
    show_state(on, gpio)

    def set_mosfet(state: bool):
        nonlocal on
        on = bool(state)
        lgpio.gpio_write(h, gpio, 1 if on else 0)
        show_state(on, gpio)

    try:
        while True:
            key = read_key()
            if key is None:
                time.sleep(0.05)
                continue

            if key == "q":
                break
            if key in (" ", "m"):
                set_mosfet(not on)
            elif key == "o":
                set_mosfet(True)
            elif key == "f":
                set_mosfet(False)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nExiting — MOSFET OFF.")
        lgpio.gpio_write(h, gpio, 0)
        lgpio.gpio_free(h, gpio)
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
