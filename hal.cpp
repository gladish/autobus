#include "hal.h"

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {
float clampf(float v, float lo, float hi)
{
  return std::clamp(v, lo, hi);
}

// Linear interpolation between two int endpoints
int lerp_us(float t, int a, int b)
{
  return static_cast<int>(std::round(static_cast<float>(a) + t * static_cast<float>(b - a)));
}
}  // namespace

// ---------------------------------------------------------------------------
// Servo
// ---------------------------------------------------------------------------
std::expected<void, std::string> Servo::steer(float t)
{
  t = clampf(t, -1.0f, 1.0f);

  int pulse_us{};
  if (t < 0.0f)
    // Left side: center → left_us
    pulse_us = lerp_us(-t, cfg_.center_us, cfg_.left_us);
  else
    // Right side: center → right_us
    pulse_us = lerp_us(t, cfg_.center_us, cfg_.right_us);

  return pwm_.set_pulse_us(cfg_.channel, pulse_us);
}

std::expected<void, std::string> Servo::center()
{
  return pwm_.set_pulse_us(cfg_.channel, cfg_.center_us);
}

// ---------------------------------------------------------------------------
// Esc
// ---------------------------------------------------------------------------
std::expected<void, std::string> Esc::throttle(float t)
{
  t = clampf(t, -1.0f, 1.0f);
  const auto now = std::chrono::steady_clock::now();

  int pulse_us{};
  if (t == 0.0f) {
    pulse_us = cfg_.neutral_us;
    if (reverse_state_ == ReverseState::NeutralPause) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - reverse_state_since_);
      if (elapsed >= std::chrono::milliseconds(cfg_.reverse_neutral_ms)) {
        reverse_state_ = ReverseState::Engaged;
      }
    } else if (reverse_state_ != ReverseState::Engaged) {
      reverse_state_ = ReverseState::Ready;
    }
  } else if (t > 0.0f) {
    reverse_state_ = ReverseState::Ready;
    // Map (0, 1] → [creep_us, max_forward_us]
    pulse_us = lerp_us(t, cfg_.creep_us, cfg_.max_forward_us);
  } else {
    if (reverse_state_ == ReverseState::Ready) {
      reverse_state_ = ReverseState::BrakePulse;
      reverse_state_since_ = now;
    }

    if (reverse_state_ == ReverseState::BrakePulse) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - reverse_state_since_);
      if (elapsed < std::chrono::milliseconds(cfg_.reverse_brake_ms)) {
        pulse_us = cfg_.reverse_creep_us;
        return pwm_.set_pulse_us(cfg_.channel, pulse_us);
      }

      reverse_state_ = ReverseState::NeutralPause;
      reverse_state_since_ = now;
    }

    if (reverse_state_ == ReverseState::NeutralPause) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - reverse_state_since_);
      if (elapsed < std::chrono::milliseconds(cfg_.reverse_neutral_ms)) {
        pulse_us = cfg_.neutral_us;
        return pwm_.set_pulse_us(cfg_.channel, pulse_us);
      }

      reverse_state_ = ReverseState::Engaged;
    }

    // Map [-1, 0) → [max_reverse_us, reverse_creep_us]
    pulse_us = lerp_us(-t, cfg_.reverse_creep_us, cfg_.max_reverse_us);
  }

  return pwm_.set_pulse_us(cfg_.channel, pulse_us);
}

std::expected<void, std::string> Esc::stop()
{
  reverse_state_ = ReverseState::Ready;
  return pwm_.set_pulse_us(cfg_.channel, cfg_.neutral_us);
}