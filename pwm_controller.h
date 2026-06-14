#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>

// ---------------------------------------------------------------------------
// PwmController — thin wrapper around the PCA9685 over I2C
//
// All public methods return std::expected<void, std::string> so the caller
// can handle errors without exceptions.
// ---------------------------------------------------------------------------
class PwmController
{
public:
  PwmController() = default;
  ~PwmController();

  // Non-copyable, movable
  PwmController(const PwmController&) = delete;
  PwmController& operator=(const PwmController&) = delete;
  PwmController(PwmController&&) = default;
  PwmController& operator=(PwmController&&) = default;

  [[nodiscard]] std::expected<void, std::string> open(std::string_view i2c_device);
  [[nodiscard]] std::expected<void, std::string> set_pwm_freq(float freq_hz);

  // Set pulse width in microseconds on a given channel (assumes 50 Hz / 20 ms period)
  [[nodiscard]] std::expected<void, std::string> set_pulse_us(int channel, int pulse_us);

private:
  [[nodiscard]] std::expected<void, std::string> write_reg(uint8_t reg, uint8_t value);
  [[nodiscard]] std::expected<void, std::string> set_pwm(int channel, uint16_t on, uint16_t off);

  int fd_ = -1;
};