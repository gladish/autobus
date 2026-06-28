#pragma once

#include <expected>
#include <string>

#include "config.h"
#include "pca9685_servo_driver.h"


// ---------------------------------------------------------------------------
// Servo — steering abstraction
//
// steer(t):
//   t = -1.0  →  full left  (left_us)
//   t =  0.0  →  center     (center_us)
//   t = +1.0  →  full right (right_us)
//
// Left and right throws are mapped independently so asymmetric linkages
// (like the King Yellow's) are handled correctly.
// ---------------------------------------------------------------------------
class Servo
{
public:
  explicit Servo(PCA9685ServoDriver& servo_driver, const ServoConfig& cfg)
    : servo_driver_(servo_driver)
    , cfg_(cfg)
  { }

  // Steer in [-1.0, +1.0]. Clamps silently.
  std::expected<void, std::string> steer(float t);

  // Return to mechanical center
  std::expected<void, std::string> center();

private:
  PCA9685ServoDriver& servo_driver_;
  ServoConfig cfg_;
};