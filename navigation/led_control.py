from adafruit_pca9685 import PCA9685

import board
import busio
import math
import time

SERVO_PWM_FREQ = 50         # standard analog servo frequency (Hz)

LEDID_REAR_RIGHT = 4
LEDID_REAR_LEFT = 5

def led_on(pca: PCA9685, channel: int):
  pca.channels[channel].duty_cycle = 0xffff

def led_off(pca: PCA9685, channel: int):
  pca.channels[channel].duty_cycle = 0

def update_leds(pca: PCA9685, angle, center_angle, blink_state):
    if math.isclose(angle, center_angle, abs_tol=0.01):
        val = 0xffff if blink_state else 0
        pca.channels[LEDID_REAR_LEFT].duty_cycle = val
        pca.channels[LEDID_REAR_RIGHT].duty_cycle = val
    elif angle > center_angle:
        # angle > center heads toward max_angle, which is physically LEFT
        # on this vehicle's servo mounting (confirmed on hardware) -- see
        # SteeringConfig.min_angle/max_angle comments in steering_controller.py.
        pca.channels[LEDID_REAR_LEFT].duty_cycle = 0xffff
        pca.channels[LEDID_REAR_RIGHT].duty_cycle = 0
    else:
        pca.channels[LEDID_REAR_LEFT].duty_cycle = 0
        pca.channels[LEDID_REAR_RIGHT].duty_cycle = 0xffff


def main():
  i2c = busio.I2C(board.SCL, board.SDA)
  pca = PCA9685(i2c)
  pca.frequency = SERVO_PWM_FREQ
  state = True

  while True:
    if state:
      print("left on, right off")
      led_on(pca, LEDID_REAR_LEFT)
      led_off(pca, LEDID_REAR_RIGHT)
    else:
      print("righ on, left off")
      led_on(pca, LEDID_REAR_RIGHT)
      led_off(pca, LEDID_REAR_LEFT)
    state = not state
    time.sleep(2)



if __name__ == "__main__":
  main()
