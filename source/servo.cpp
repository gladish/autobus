#include <algorithm>
#include <cmath>

#include "servo.h"
#include "pwm_controller.h"


std::expected<void, std::string> Servo::steer(float t)
{
  t = std::clamp(t, -1.0f, 1.0f);

  int pulse_us{};
  if (t < 0.0f) {
    // Left side: center → left_us
    pulse_us = static_cast<int>(std::lerp(
        static_cast<float>(cfg_.center_us),
        static_cast<float>(cfg_.left_us),
        -t));
  }
  else {
    // Right side: center → right_us
    pulse_us = static_cast<int>(std::lerp(
        static_cast<float>(cfg_.center_us),
        static_cast<float>(cfg_.right_us),
        t));
  }

  return pwm_.set_pulse_us(cfg_.channel, pulse_us);
}

std::expected<void, std::string> Servo::center()
{
  return pwm_.set_pulse_us(cfg_.channel, cfg_.center_us);
}
