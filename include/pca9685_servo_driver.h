#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>

/**
 * Thin wrapper around the PCA9685 connected over I2C.
 *
 * All public methods return std::expected<void, std::string> so callers
 * can handle failures without exceptions.
 */
class PCA9685ServoDriver
{
public:
  PCA9685ServoDriver() = default;

  ~PCA9685ServoDriver();

  // Non-copyable
  PCA9685ServoDriver(const PCA9685ServoDriver&) = delete;

  PCA9685ServoDriver& operator=(const PCA9685ServoDriver&) = delete;

  PCA9685ServoDriver(PCA9685ServoDriver&& other) noexcept;

  PCA9685ServoDriver& operator=(PCA9685ServoDriver&& other) noexcept;

  /**
   * Opens the Linux I2C device node used to communicate with the PCA9685.
   *
   * @param i2cDevice Path to the I2C bus device (for example, "/dev/i2c-1").
   * @return Empty success value or an error string describing what failed.
   */
  std::expected<void, std::string> open(std::string_view i2cDevice);

  /**
   * Configures the PWM frequency for the PCA9685 channels.
   *
   * @param freqHz Target PWM frequency in hertz.
   * @return Empty success value or an error string describing what failed.
   */
  std::expected<void, std::string> setPwmFreq(float freqHz);

  /**
   * Sets a channel pulse width in microseconds (assumes 50 Hz / 20 ms period).
   *
   * @param channel PCA9685 output channel index.
   * @param pulseUs Pulse width in microseconds.
   * @return Empty success value or an error string describing what failed.
   */
  std::expected<void, std::string> setPulseUs(int channel, int pulseUs);

private:
  /**
   * Writes one PCA9685 register over I2C.
   *
   * @param reg Register address to write.
   * @param value Byte value to write.
   * @return Empty success value or an error string describing what failed.
   */
  std::expected<void, std::string> writeReg(uint8_t reg, uint8_t value);

  /**
   * Programs ON/OFF tick counts for a PWM channel.
   *
   * @param channel PCA9685 output channel index.
   * @param on Tick at which output turns on.
   * @param off Tick at which output turns off.
   * @return Empty success value or an error string describing what failed.
   */
  std::expected<void, std::string> setPwm(int channel, uint16_t on, uint16_t off);

private:
  int i2cDevice_ = -1;
};