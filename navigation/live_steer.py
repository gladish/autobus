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
import argparse
import math
import sys
import time

import cv2
from picamera2 import Picamera2
from adafruit_pca9685 import PCA9685
import board
import busio

from sidewalk_detect import analyze_frame, DEFAULT_SIDEWALK_HSV_LOW, DEFAULT_SIDEWALK_HSV_HIGH
from steering_controller import SteeringController, SteeringConfig
from led_control import update_leds

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
    parser = argparse.ArgumentParser(description="Live sidewalk-following steering loop")
    parser.add_argument(
        "--record",
        default=None,
        help="path to write an annotated mp4 of what the camera/controller saw (e.g. run1.mp4)",
    )
    args = parser.parse_args()

    TARGET_FPS = 30

    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = SERVO_PWM_FREQ

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={
          "size": (640, 480),
          "format": "RGB888"
        },
        controls={
          "FrameRate": TARGET_FPS
        },
      )
    picam2.configure(config)
    picam2.start()

    writer = None
    if args.record:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.record, fourcc, TARGET_FPS, (640, 480))
        print(f"Recording annotated video to {args.record}")

    steering_config = SteeringConfig()
    controller = SteeringController(steering_config)
    sidewalk_low, sidewalk_high = DEFAULT_SIDEWALK_HSV_LOW, DEFAULT_SIDEWALK_HSV_HIGH
    blink_state = False

    print("Starting steering loop. Ctrl+C to stop.")


    TARGET_LOOP_DT = 1.0 / TARGET_FPS

    try:
        prev_t = time.monotonic()
        loop_start = time.monotonic()
        frame_count = 0
        while True:
            frame = picam2.capture_array()
            now = time.monotonic()
            measured_fps = 1.0 / (now - prev_t)
            prev_t = now

            overlay, result = analyze_frame(frame, sidewalk_low, sidewalk_high)
            angle, should_stop = controller.update(result["center_offset_px"], result["confidence"])

            if should_stop:
                # Hold steering centered-ish and, importantly, your throttle
                # logic (not included here) should cut drive power on this
                # condition rather than continuing to move blind.
                print(f"SIDEWALK LOST -- holding angle {angle:.1f}, stop drive motor here")
                # TODO: we should beep here
                blink_state = not blink_state
                update_leds(pca, angle, steering_config.center_angle, blink_state)
            else:
                pca.channels[STEERING_CHANNEL].duty_cycle = angle_to_duty_cycle(angle)
                blink_state = False
                update_leds(pca, angle, steering_config.center_angle, blink_state)

            if writer is not None:
                status = "STOP" if should_stop else "DRIVE"
                cv2.putText(
                    overlay,
                    f"servo angle={angle:.1f} [{status}]",
                    (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 0),
                    2,
                )
                writer.write(overlay)

            # Print at a readable rate rather than flooding the console.
            # print(f"offset={result['center_offset_px']} conf={result['confidence']:.2f} angle={angle:.1f}")

            elapsed = time.monotonic() - now
            remainder = TARGET_LOOP_DT - elapsed
            if remainder > 0:
              time.sleep(remainder)

            frame_count += 1
            if frame_count % (TARGET_FPS * 5) == 0:
              print(f"measured_fps={measured_fps:.1f} target={TARGET_FPS}")
    except KeyboardInterrupt:
        pass
    finally:
        # Return steering to center and release the camera cleanly on exit.
        pca.channels[STEERING_CHANNEL].duty_cycle = angle_to_duty_cycle(SteeringConfig().center_angle)
        picam2.stop()
        if writer is not None:
            writer.release()
        print("Stopped, steering centered.")


if __name__ == "__main__":
    main()

