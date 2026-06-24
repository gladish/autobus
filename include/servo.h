#pragma once

#include <expected>
#include <string>

#include "config.h"

class PwmController;

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
  explicit Servo(PwmController& pwm, const ServoConfig& cfg)
    : pwm_(pwm)
    , cfg_(cfg)
  { }

  // Steer in [-1.0, +1.0]. Clamps silently.
  [[nodiscard]] std::expected<void, std::string> steer(float t);

  // Return to mechanical center
  [[nodiscard]] std::expected<void, std::string> center();

private:
  PwmController& pwm_;
  ServoConfig cfg_;
};