#!/usr/bin/env python3
"""
Hardware demo (not a unit test) -- commands the steering servo left, then
right, calling update_leds() at each position so you can visually confirm
on the actual robot that left is left and right is right:

    servo at min_angle (right) -> RIGHT rear LED should light
    servo at max_angle (left)  -> LEFT rear LED should light

(min_angle/max_angle -> right/left confirmed by hardware test, opposite of
what the angle names might suggest -- see SteeringConfig comments.)

Run on the Pi with the servo and LEDs wired up. Put the vehicle on blocks
first -- this drives the real servo.
"""
import time

from adafruit_pca9685 import PCA9685
import board
import busio

from steering_controller import SteeringConfig
from led_control import update_leds

STEERING_CHANNEL = 0
SERVO_PWM_FREQ = 50
SERVO_MIN_PULSE_US = 1000
SERVO_MAX_PULSE_US = 2000
HOLD_SECONDS = 3


def angle_to_duty_cycle(angle_deg, pca_freq=SERVO_PWM_FREQ):
    pulse_us = SERVO_MIN_PULSE_US + (angle_deg / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
    period_us = 1_000_000 / pca_freq
    duty_fraction = pulse_us / period_us
    return int(duty_fraction * 0xFFFF)


def go_to(pca, cfg, angle, label):
    print(f"--- {label}: commanding servo angle {angle:.1f} ---")
    pca.channels[STEERING_CHANNEL].duty_cycle = angle_to_duty_cycle(angle)
    update_leds(pca, angle, cfg.center_angle, blink_state=False)
    time.sleep(HOLD_SECONDS)


def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = SERVO_PWM_FREQ
    cfg = SteeringConfig()

    try:
        go_to(pca, cfg, cfg.min_angle, "RIGHT (expect RIGHT led on)")
        go_to(pca, cfg, cfg.max_angle, "LEFT (expect LEFT led on)")
    finally:
        print("--- returning to center, leds off ---")
        pca.channels[STEERING_CHANNEL].duty_cycle = angle_to_duty_cycle(cfg.center_angle)
        update_leds(pca, cfg.center_angle, cfg.center_angle, blink_state=False)


if __name__ == "__main__":
    main()
