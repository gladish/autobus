#!/usr/bin/env python3
"""
Live sidewalk-following steering loop -- runs ON THE PI (hulk), not on
recorded video. Captures frames from the camera, runs the same detection
used in sidewalk_detect.py, and drives the steering servo via the PCA9685.

This is deliberately kept separate from sidewalk_detect.py: that script is
for offline tuning against recorded footage with no hardware dependency,
this one is for the actual closed loop on the robot.

Requires on the Pi:
    pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor picamera2

SAFETY NOTES -- read before running with the drive motor connected:
    - Test with the vehicle up on blocks (wheels off the ground) first.
    - Keep a hand near a physical kill switch / be ready to cut power.
    - This script does NOT include drive-motor / ESC control -- it only
      commands steering. Wire your own throttle logic and make sure it
      respects `should_stop` below before adding motion.
    - lost_frame_limit in SteeringConfig determines how many consecutive
      "lost the sidewalk" frames are tolerated before should_stop fires.
      Tune this down (more cautious) before any real outdoor test.
"""

import time

from picamera2 import Picamera2
from adafruit_pca9685 import PCA9685
import board
import busio

from sidewalk_detect import analyze_frame, DEFAULT_SIDEWALK_HSV_LOW, DEFAULT_SIDEWALK_HSV_HIGH
from steering_controller import SteeringController, SteeringConfig

STEERING_CHANNEL = 0        # PCA9685 channel the steering servo is wired to
SERVO_PWM_FREQ = 50         # standard analog servo frequency (Hz)

# Pulse width range for your specific servo -- these are common defaults,
# but confirm against your servo's datasheet before trusting them.
SERVO_MIN_PULSE_US = 1000   # pulse width at 0 degrees
SERVO_MAX_PULSE_US = 2000   # pulse width at 180 degrees


def angle_to_duty_cycle(angle_deg, pca_freq=SERVO_PWM_FREQ):
    """Convert a servo angle (0-180) to a PCA9685 16-bit duty cycle value."""
    pulse_us = SERVO_MIN_PULSE_US + (angle_deg / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
    period_us = 1_000_000 / pca_freq
    duty_fraction = pulse_us / period_us
    return int(duty_fraction * 0xFFFF)


def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = SERVO_PWM_FREQ

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    controller = SteeringController(SteeringConfig())
    sidewalk_low, sidewalk_high = DEFAULT_SIDEWALK_HSV_LOW, DEFAULT_SIDEWALK_HSV_HIGH

    print("Starting steering loop. Ctrl+C to stop.")
    try:
        while True:
            frame = picam2.capture_array()
            _, result = analyze_frame(frame, sidewalk_low, sidewalk_high)
            angle, should_stop = controller.update(
                result["center_offset_px"], result["confidence"]
            )

            if should_stop:
                # Hold steering centered-ish and, importantly, your throttle
                # logic (not included here) should cut drive power on this
                # condition rather than continuing to move blind.
                print(f"SIDEWALK LOST -- holding angle {angle:.1f}, stop drive motor here")
            else:
                pca.channels[STEERING_CHANNEL].duty_cycle = angle_to_duty_cycle(angle)

            # Print at a readable rate rather than flooding the console.
            print(f"offset={result['center_offset_px']} conf={result['confidence']:.2f} angle={angle:.1f}")

            time.sleep(0.03)  # ~30fps loop pace; adjust to match actual camera throughput

    except KeyboardInterrupt:
        pass
    finally:
        # Return steering to center and release the camera cleanly on exit.
        pca.channels[STEERING_CHANNEL].duty_cycle = angle_to_duty_cycle(SteeringConfig().center_angle)
        picam2.stop()
        print("Stopped, steering centered.")


if __name__ == "__main__":
    main()