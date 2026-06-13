#include <fmt/core.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string_view>

#include "config.h"
#include "hal.h"
#include "pwm.h"

// ---------------------------------------------------------------------------
// Signal handling — Ctrl-C sets a flag so we can clean up gracefully
// ---------------------------------------------------------------------------
namespace {
std::atomic<bool> g_running{true};

void on_signal(int)
{
  g_running = false;
}

// ---------------------------------------------------------------------------
// Raw terminal — no echo, no line buffering, read one byte at a time
// ---------------------------------------------------------------------------
struct RawTerminal {
  termios original{};

  RawTerminal()
  {
    tcgetattr(STDIN_FILENO, &original);
    termios raw = original;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;   // non-blocking
    raw.c_cc[VTIME] = 1;  // 100 ms timeout
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~RawTerminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &original);
  }

  // Non-copyable
  RawTerminal(const RawTerminal&) = delete;
  RawTerminal& operator=(const RawTerminal&) = delete;
};

// ---------------------------------------------------------------------------
// Read a single keypress; returns '\0' if nothing available
// ---------------------------------------------------------------------------
char read_key()
{
  char ch = '\0';
  // Swallow escape sequences (arrow keys) so they don't bleed into input
  char buf[4]{};
  ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
  if (n == 1)
    ch = buf[0];
  // Ignore multi-byte sequences (arrow keys etc.)
  return ch;
}

// ---------------------------------------------------------------------------
// Print status on a single line, overwriting previous output
// ---------------------------------------------------------------------------
void print_status(float throttle, float steer, int throttle_us, int steer_us)
{
  // Simple ASCII bar for steering: ← center →
  constexpr int kWidth = 20;
  int pos = static_cast<int>((steer + 1.0f) / 2.0f * static_cast<float>(kWidth));
  pos = std::clamp(pos, 0, kWidth);

  std::string bar(static_cast<std::string::size_type>(kWidth), '-');
  bar[static_cast<std::string::size_type>(pos)] = '|';

  fmt::print("\r  throttle: {:.2f} ({:4d}µs)   steer: [{:s}] {:.2f} ({:4d}µs)   ", throttle,
             throttle_us, bar, steer, steer_us);
  std::fflush(stdout);
}

// ---------------------------------------------------------------------------
// Map throttle/steer floats back to µs for display (mirrors HAL logic)
// ---------------------------------------------------------------------------
int throttle_to_us(float t, const EscConfig& cfg)
{
  if (t == 0.0f)
    return cfg.neutral_us;

  if (t > 0.0f) {
    return static_cast<int>(static_cast<float>(cfg.creep_us) +
                            t * static_cast<float>(cfg.max_forward_us - cfg.creep_us));
  }

  return static_cast<int>(static_cast<float>(cfg.reverse_creep_us) +
                          (-t) * static_cast<float>(cfg.max_reverse_us - cfg.reverse_creep_us));
}

int steer_to_us(float t, const ServoConfig& cfg)
{
  if (t < 0.0f)
    return static_cast<int>(static_cast<float>(cfg.center_us) +
                            (-t) * static_cast<float>(cfg.left_us - cfg.center_us));
  return static_cast<int>(static_cast<float>(cfg.center_us) +
                          t * static_cast<float>(cfg.right_us - cfg.center_us));
}
}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  // Accept an optional config path; default to config.json next to the binary
  std::string_view config_path = (argc > 1) ? argv[1] : "config.json";

  // Load config
  auto cfg_result = load_config(config_path);
  if (!cfg_result) {
    fmt::print(stderr, "[error] {}\n", cfg_result.error());
    return 1;
  }
  const Config cfg = *cfg_result;

  fmt::print("\n  pwmcli — manual control\n");
  fmt::print("  Config: {}\n\n", config_path);
  fmt::print("  W / S / X  forward up / stop / reverse request\n");
  fmt::print("            reverse may engage after brake+neutral timing\n");
  fmt::print("  A / D   steer left / right\n");
  fmt::print("  0       steer center\n");
  fmt::print("  Q       quit\n\n");

  // Init PWM
  PwmController pwm;
  if (auto r = pwm.open("/dev/i2c-1"); !r) {
    fmt::print(stderr, "[error] {}\n", r.error());
    return 1;
  }
  if (auto r = pwm.set_pwm_freq(50.0f); !r) {
    fmt::print(stderr, "[error] {}\n", r.error());
    return 1;
  }

  Servo servo{pwm, cfg.servo};
  Esc esc{pwm, cfg.esc};

  // Arm ESC — send neutral and wait for user confirmation
  fmt::print("  Sending neutral to ESC. Power it on now and wait for the arming beep.\n");
  fmt::print("  Press ENTER when armed... ");
  std::fflush(stdout);

  if (auto r = esc.stop(); !r) {
    fmt::print(stderr, "\n[error] {}\n", r.error());
    return 1;
  }
  if (auto r = servo.center(); !r) {
    fmt::print(stderr, "\n[error] {}\n", r.error());
    return 1;
  }

  // Wait for Enter (in normal terminal mode before we go raw)
  while (std::getchar() != '\n') { }
  fmt::print("\n  Armed. Use WASD to drive. Q to quit.\n\n");

  // Install signal handler
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // Go raw
  RawTerminal raw_term;

  float throttle = 0.0f;
  float steer = 0.0f;

  constexpr float kThrottleStep = 0.05f;
  constexpr float kSteerStep = 0.05f;

  while (g_running) {
    char key = read_key();

    switch (key) {
      case 'w':
      case 'W':
        throttle = std::clamp(throttle + kThrottleStep, -1.0f, 1.0f);
        break;

      case 'x':
      case 'X':
        throttle = std::clamp(throttle - kThrottleStep, -1.0f, 1.0f);
        break;

      case 's':
      case 'S':
        throttle = 0.0f;
        break;

      case 'a':
      case 'A':
        steer = std::clamp(steer - kSteerStep, -1.0f, 1.0f);
        break;

      case 'd':
      case 'D':
        steer = std::clamp(steer + kSteerStep, -1.0f, 1.0f);
        break;

      case '0':
        steer = 0.0f;
        break;

      case 'q':
      case 'Q':
      case '\x03':  // q or Ctrl-C
        g_running = false;
        break;

      default:
        break;
    }

    // Send to hardware on every iteration (key or timeout)
    if (auto r = esc.throttle(throttle); !r)
      fmt::print(stderr, "\n[error] {}\n", r.error());

    if (auto r = servo.steer(steer); !r)
      fmt::print(stderr, "\n[error] {}\n", r.error());

    print_status(throttle, steer, throttle_to_us(throttle, cfg.esc), steer_to_us(steer, cfg.servo));
  }

  // Clean shutdown
  fmt::print("\n\n  Stopping...\n");
  if (auto r = esc.stop(); !r)
    fmt::print(stderr, "[error] {}\n", r.error());

  if (auto r = servo.center(); !r)
    fmt::print(stderr, "[error] {}\n", r.error());

  return 0;
}