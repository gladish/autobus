#include <algorithm>
#include <cmath>

#include "servo.h"
#include "pwm_controller.h"


std::expected<void, std::string> Servo::steer(float t)
{
  t = std::clamp(t, -1.0f, 1.0f);
  if (cfg_.reverse) {
    t = -t;
  }

  int const left_us = std::min(cfg_.left_us, cfg_.right_us);
  int const right_us = std::max(cfg_.left_us, cfg_.right_us);

  int pulse_us{};
  if (t < 0.0f) {
    // Left side always maps toward the minimum configured pulse.
    pulse_us = static_cast<int>(std::lerp(
        static_cast<float>(cfg_.center_us),
        static_cast<float>(left_us),
        -t));
  }
  else {
    // Right side always maps toward the maximum configured pulse.
    pulse_us = static_cast<int>(std::lerp(
        static_cast<float>(cfg_.center_us),
        static_cast<float>(right_us),
        t));
  }

  return pwm_.set_pulse_us(cfg_.channel, pulse_us);
}

std::expected<void, std::string> Servo::center()
{
  return pwm_.set_pulse_us(cfg_.channel, cfg_.center_us);
}
