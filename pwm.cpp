#include "pwm.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fmt/core.h>
#include <thread>

// ---------------------------------------------------------------------------
// PCA9685 register map
// ---------------------------------------------------------------------------
namespace {
constexpr uint8_t kAddr = 0x40;
constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kPrescale = 0xFE;
constexpr uint8_t kLed0OnL = 0x06;

constexpr uint8_t kMode1Sleep = 0x10;
constexpr uint8_t kMode1AutoInc = 0x20;
constexpr uint8_t kMode1Restart = 0x80;

constexpr int kPeriodUs = 20'000;  // 50 Hz
constexpr int kCounts = 4096;
constexpr double kOscillatorHz = 25'000'000.0;
}  // namespace

// ---------------------------------------------------------------------------
PwmController::~PwmController()
{
  if (fd_ >= 0)
    ::close(fd_);
}

// ---------------------------------------------------------------------------
std::expected<void, std::string> PwmController::open(std::string_view i2c_device)
{
  fd_ = ::open(i2c_device.data(), O_RDWR);
  if (fd_ < 0)
    return std::unexpected(fmt::format("Failed to open {}: {}", i2c_device, std::strerror(errno)));

  if (::ioctl(fd_, I2C_SLAVE, kAddr) < 0) {
    ::close(fd_);
    fd_ = -1;
    return std::unexpected(
        fmt::format("Failed to set I2C slave 0x{:02x}: {}", kAddr, std::strerror(errno)));
  }

  return {};
}

// ---------------------------------------------------------------------------
std::expected<void, std::string> PwmController::set_pwm_freq(float freq_hz)
{
  // Datasheet section 7.3.5
  auto prescale = static_cast<uint8_t>(
      std::round(kOscillatorHz / (kCounts * static_cast<double>(freq_hz))) - 1.0);

  // Must sleep before changing prescale
  if (auto r = write_reg(kMode1, kMode1Sleep); !r)
    return r;

  if (auto r = write_reg(kPrescale, prescale); !r)
    return r;

  // Wake, enable auto-increment
  if (auto r = write_reg(kMode1, kMode1AutoInc); !r)
    return r;

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // Restart PWM channels (datasheet section 7.3.1.1)
  if (auto r = write_reg(kMode1, kMode1AutoInc | kMode1Restart); !r)
    return r;

  return {};
}

// ---------------------------------------------------------------------------
std::expected<void, std::string> PwmController::set_pulse_us(int channel, int pulse_us)
{
  int counts = (pulse_us * kCounts) / kPeriodUs;
  return set_pwm(channel, 0, static_cast<uint16_t>(counts));
}

// ---------------------------------------------------------------------------
std::expected<void, std::string> PwmController::write_reg(uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};
  if (::write(fd_, buf, sizeof(buf)) != sizeof(buf))
    return std::unexpected(fmt::format("write_reg 0x{:02x} failed: {}", reg, std::strerror(errno)));
  return {};
}

// ---------------------------------------------------------------------------
std::expected<void, std::string> PwmController::set_pwm(int channel, uint16_t on, uint16_t off)
{
  uint8_t reg = static_cast<uint8_t>(kLed0OnL + 4 * channel);
  uint8_t buf[5] = {
      reg,
      static_cast<uint8_t>(on & 0xff),
      static_cast<uint8_t>(on >> 8),
      static_cast<uint8_t>(off & 0xff),
      static_cast<uint8_t>(off >> 8),
  };
  if (::write(fd_, buf, sizeof(buf)) != sizeof(buf))
    return std::unexpected(fmt::format("set_pwm ch{} failed: {}", channel, std::strerror(errno)));
  return {};
}