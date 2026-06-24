#include <algorithm>
#include <cmath>

#include "speed_controller.h"
#include "pwm_controller.h"

// ---------------------------------------------------------------------------
// Esc
// ---------------------------------------------------------------------------
std::expected<void, std::string> Esc::throttle(float t)
{
  t = std::clamp(t, -1.0f, 1.0f);
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
    pulse_us = static_cast<int>(std::lerp(
        static_cast<float>(cfg_.creep_us),
        static_cast<float>(cfg_.max_forward_us),
        t));
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
    pulse_us = static_cast<int>(std::lerp(
      static_cast<float>(cfg_.reverse_creep_us),
      static_cast<float>(cfg_.max_reverse_us),
      -t));
  }

  return pwm_.set_pulse_us(cfg_.channel, pulse_us);
}

std::expected<void, std::string> Esc::stop()
{
  reverse_state_ = ReverseState::Ready;
  return pwm_.set_pulse_us(cfg_.channel, cfg_.neutral_us);
}