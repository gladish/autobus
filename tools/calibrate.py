#!/usr/bin/env python3
"""
autobus calibration tool
Calibrates steering servo center/limits and ESC neutral/creep values.
Saves results to config.json for use by the C++ autobus binary.

Hardware:
  - Steering servo: PCA9685 channel 0
  - ESC:            PCA9685 channel 8

Controls during each step:
  A / LEFT  : decrease pulse width
  D / RIGHT : increase pulse width
  W         : increase step size
  S         : decrease step size
  ENTER     : confirm and move to next step
  Q         : quit without saving
"""

import sys
import os
import json
import time
import termios
import tty

# ---------------------------------------------------------------------------
# Dependency check
# ---------------------------------------------------------------------------
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
except ImportError:
    print("\n[ERROR] Required libraries not found.")
    print("Install them with:\n")
    print("  pip3 install adafruit-circuitpython-pca9685 --break-system-packages\n")
    print("Then re-run this script.")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SERVO_CHANNEL   = 0
ESC_CHANNEL     = 8
PWM_FREQ_HZ     = 50
PERIOD_US       = 1_000_000 // PWM_FREQ_HZ   # 20,000 µs
PCA9685_COUNTS  = 4096

SERVO_CENTER_US = 1500
SERVO_MIN_US    = 900
SERVO_MAX_US    = 2100

ESC_NEUTRAL_US  = 1500
ESC_MIN_US      = 1000
ESC_MAX_US      = 2000

DEFAULT_STEP_US = 10
MIN_STEP_US     = 1
MAX_STEP_US     = 50

# ---------------------------------------------------------------------------
# ANSI colours
# ---------------------------------------------------------------------------
RESET  = "\033[0m"
BOLD   = "\033[1m"
CYAN   = "\033[96m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
DIM    = "\033[2m"

def c(colour: str, text: str) -> str:
    return f"{colour}{text}{RESET}"

# ---------------------------------------------------------------------------
# PCA9685 helpers
# ---------------------------------------------------------------------------
def us_to_counts(pulse_us: int) -> int:
    """Convert microseconds to PCA9685 12-bit counts."""
    return round(pulse_us * PCA9685_COUNTS / PERIOD_US)

def set_pulse(pca: PCA9685, channel: int, pulse_us: int) -> None:
    counts = us_to_counts(pulse_us)
    pca.channels[channel].duty_cycle = counts << 4  # 12-bit -> 16-bit

def clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))

# ---------------------------------------------------------------------------
# Raw keypress
# ---------------------------------------------------------------------------
def get_key() -> str:
    """Read a single keypress without waiting for Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch + ch2 + ch3
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ---------------------------------------------------------------------------
# UI helpers
# ---------------------------------------------------------------------------
def clear_line() -> None:
    print("\r\033[K", end="", flush=True)

def print_header(title: str) -> None:
    width = 60
    print()
    print(c(CYAN, "=" * width))
    print(c(CYAN + BOLD, f"  {title}"))
    print(c(CYAN, "=" * width))

def print_controls() -> None:
    print(c(DIM, "  A/← decrease   D/→ increase   W bigger step   S smaller step"))
    print(c(DIM, "  ENTER confirm   Q quit"))

def print_pulse(label: str, pulse_us: int, step_us: int) -> None:
    bar_width = 30
    lo, hi = SERVO_MIN_US, SERVO_MAX_US
    pos = int((pulse_us - lo) / (hi - lo) * bar_width)
    bar = "[" + "-" * pos + c(GREEN, "█") + "-" * (bar_width - pos) + "]"
    clear_line()
    print(f"\r  {label}: {c(BOLD, f'{pulse_us:4d} µs')}  step={step_us}µs  {bar}", end="", flush=True)

# ---------------------------------------------------------------------------
# Interactive nudge loop
# ---------------------------------------------------------------------------
def nudge_loop(
    pca: PCA9685,
    channel: int,
    label: str,
    initial_us: int,
    lo_us: int,
    hi_us: int,
) -> int | None:
    """
    Let the user nudge a pulse width with A/D keys.
    Returns the confirmed value, or None if the user quits.
    """
    pulse = initial_us
    step  = DEFAULT_STEP_US
    set_pulse(pca, channel, pulse)
    print_controls()
    print()

    while True:
        print_pulse(label, pulse, step)
        key = get_key()

        if key in ('a', 'A', '\x1b[D'):   # left arrow
            pulse = clamp(pulse - step, lo_us, hi_us)
            set_pulse(pca, channel, pulse)

        elif key in ('d', 'D', '\x1b[C'): # right arrow
            pulse = clamp(pulse + step, lo_us, hi_us)
            set_pulse(pca, channel, pulse)

        elif key in ('w', 'W'):
            step = clamp(step + 1, MIN_STEP_US, MAX_STEP_US)

        elif key in ('s', 'S'):
            step = clamp(step - 1, MIN_STEP_US, MAX_STEP_US)

        elif key in ('\r', '\n'):
            print()
            return pulse

        elif key in ('q', 'Q', '\x03'):   # q or Ctrl-C
            print()
            return None

# ---------------------------------------------------------------------------
# Calibration steps
# ---------------------------------------------------------------------------
def step_arm_esc(pca: PCA9685) -> bool:
    """Send neutral to ESC and wait for user to confirm arming beep."""
    print_header("Step 1 of 4 — Arm the ESC")
    print()
    print(f"  Sending neutral ({c(BOLD, '1500 µs')}) to ESC on channel {ESC_CHANNEL}.")
    print()
    print(c(YELLOW, "  ► Power ON your ESC now."))
    print(c(YELLOW, "  ► Wait for the arming beep sequence to finish."))
    print()

    set_pulse(pca, ESC_CHANNEL, ESC_NEUTRAL_US)

    while True:
        print(f"  Did you hear the arming beep? [{c(GREEN,'Y')}es / {c(RED,'N')}o / {c(DIM,'Q')}uit] ", end="", flush=True)
        key = input().strip().lower()
        if key == 'y':
            print(c(GREEN, "\n  ✓ ESC armed.\n"))
            return True
        elif key == 'n':
            print(c(YELLOW, "  Waiting — make sure the ESC power switch is on.\n"))
        elif key in ('q', ''):
            return False


def step_servo_center(pca: PCA9685) -> int | None:
    """Trim steering servo center."""
    print_header("Step 2 of 4 — Steering Center")
    print()
    print("  Nudge until the front wheels point " + c(BOLD, "dead straight") + ".")
    print()
    result = nudge_loop(pca, SERVO_CHANNEL, "center", SERVO_CENTER_US, SERVO_MIN_US, SERVO_MAX_US)
    if result is not None:
        print(c(GREEN, f"  ✓ Center locked at {result} µs\n"))
    return result


def step_servo_limits(pca: PCA9685, center_us: int) -> tuple[int, int] | None:
    """Find left and right steering limits."""
    # Left limit
    print_header("Step 3 of 4 — Steering Limits")
    print()
    print("  Nudge to " + c(BOLD, "maximum LEFT") + " — stop " + c(YELLOW, "before") + " the linkage binds.")
    print()
    left_us = nudge_loop(pca, SERVO_CHANNEL, "left ", center_us, SERVO_MIN_US, center_us)
    if left_us is None:
        return None
    print(c(GREEN, f"  ✓ Left limit locked at {left_us} µs\n"))

    # Right limit
    print()
    print("  Nudge to " + c(BOLD, "maximum RIGHT") + " — stop " + c(YELLOW, "before") + " the linkage binds.")
    print()
    right_us = nudge_loop(pca, SERVO_CHANNEL, "right", center_us, center_us, SERVO_MAX_US)
    if right_us is None:
        return None
    print(c(GREEN, f"  ✓ Right limit locked at {right_us} µs\n"))

    # Return to center
    set_pulse(pca, SERVO_CHANNEL, center_us)
    return left_us, right_us


def step_esc_creep(pca: PCA9685) -> int | None:
    """Find the minimum forward throttle that actually moves the bus."""
    print_header("Step 4 of 4 — Minimum Creep Speed")
    print()
    print("  Nudge throttle " + c(BOLD, "up from neutral") + " until the wheels " + c(BOLD, "just start turning") + ".")
    print(c(YELLOW, "  ⚠  Wheels are off the ground — keep throttle low!"))
    print()
    result = nudge_loop(pca, ESC_CHANNEL, "throttle", ESC_NEUTRAL_US, ESC_NEUTRAL_US, ESC_NEUTRAL_US + 200)
    if result is not None:
        # Return ESC to neutral
        set_pulse(pca, ESC_CHANNEL, ESC_NEUTRAL_US)
        print(c(GREEN, f"  ✓ Creep speed locked at {result} µs\n"))
    return result

# ---------------------------------------------------------------------------
# Save config
# ---------------------------------------------------------------------------
def save_config(center_us: int, left_us: int, right_us: int, creep_us: int) -> None:
    config = {
        "servo": {
            "channel":   SERVO_CHANNEL,
            "center_us": center_us,
            "left_us":   left_us,
            "right_us":  right_us,
        },
        "esc": {
            "channel":        ESC_CHANNEL,
            "neutral_us":     ESC_NEUTRAL_US,
            "creep_us":       creep_us,
            "max_forward_us": ESC_MAX_US,
        },
    }
    path = os.path.join(os.path.dirname(__file__), "config.json")
    with open(path, "w") as f:
        json.dump(config, f, indent=2)
    print(c(GREEN, f"  ✓ Saved to {path}\n"))
    print(json.dumps(config, indent=2))
    print()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    print(c(CYAN + BOLD, "\n  autobus — hardware calibration tool"))
    print(c(DIM,         "  Steering: channel 0 | ESC: channel 8 | 50 Hz\n"))

    # Init PCA9685
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c)
        pca.frequency = PWM_FREQ_HZ
        print(c(GREEN, "  ✓ PCA9685 initialised at 50 Hz\n"))
    except Exception as e:
        print(c(RED, f"\n  [ERROR] Could not init PCA9685: {e}"))
        print("  Check I2C is enabled (sudo raspi-config) and wiring is correct.")
        sys.exit(1)

    try:
        # Step 1 — Arm ESC
        if not step_arm_esc(pca):
            print(c(RED, "  Aborted.\n"))
            return

        # Step 2 — Servo center
        center_us = step_servo_center(pca)
        if center_us is None:
            print(c(RED, "  Aborted.\n"))
            return

        # Step 3 — Servo limits
        limits = step_servo_limits(pca, center_us)
        if limits is None:
            print(c(RED, "  Aborted.\n"))
            return
        left_us, right_us = limits

        # Step 4 — ESC creep
        creep_us = step_esc_creep(pca)
        if creep_us is None:
            print(c(RED, "  Aborted.\n"))
            return

        # Save
        print_header("Calibration Complete")
        print()
        save_config(center_us, left_us, right_us, creep_us)
        print(c(GREEN + BOLD, "  All done! You can now build and run autobus.\n"))

    finally:
        # Always return ESC to neutral and servo to center on exit
        try:
            set_pulse(pca, ESC_CHANNEL,   ESC_NEUTRAL_US)
            set_pulse(pca, SERVO_CHANNEL, SERVO_CENTER_US)
            time.sleep(0.1)
            pca.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()