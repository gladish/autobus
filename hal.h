#pragma once

#include <chrono>
#include <expected>
#include <string>

#include "config.h"
#include "pwm.h"

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

// ---------------------------------------------------------------------------
// Esc — throttle abstraction
//
// throttle(t):
//   t = -1.0 → max_reverse_us
//   t =  0.0 → neutral (stopped)
//   t = +1.0 → max_forward_us
//
//   Forward t in (0, 1] maps to [creep_us, max_forward_us].
//   Reverse t in [-1, 0) maps to [reverse_creep_us, max_reverse_us].
//   t=0.0 always sends neutral.
//
// stop() — sends neutral immediately
// ---------------------------------------------------------------------------
class Esc
{
public:
  explicit Esc(PwmController& pwm, const EscConfig& cfg)
    : pwm_(pwm)
    , cfg_(cfg)
  { }

  // Throttle in [-1.0, 1.0]. Clamps silently.
  [[nodiscard]] std::expected<void, std::string> throttle(float t);

  // Send neutral immediately
  [[nodiscard]] std::expected<void, std::string> stop();

private:
  enum class ReverseState {
    Ready,
    BrakePulse,
    NeutralPause,
    Engaged,
  };

  PwmController& pwm_;
  EscConfig cfg_;
  ReverseState reverse_state_ = ReverseState::Ready;
  std::chrono::steady_clock::time_point reverse_state_since_{};
};