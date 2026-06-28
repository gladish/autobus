#include "config.h"

class PCA9685ServoDriver;

#include <expected>
#include <string>

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
  explicit Esc(PCA9685ServoDriver& servo_driver, const EscConfig& cfg)
    : servo_driver_(servo_driver)
    , cfg_(cfg)
  { }

  // Throttle in [-1.0, 1.0]. Clamps silently.
  std::expected<void, std::string> throttle(float t);

  // Send neutral immediately
  std::expected<void, std::string> stop();

private:
  enum class ReverseState {
    Ready,
    BrakePulse,
    NeutralPause,
    Engaged,
  };

  PCA9685ServoDriver& servo_driver_;
  EscConfig cfg_;
  ReverseState reverse_state_ = ReverseState::Ready;
  std::chrono::steady_clock::time_point reverse_state_since_{};
};