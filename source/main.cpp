/*
 * drive_listener.cpp
 *
 * Safety-hardened drive controller:
 * - Boots into a safe state (neutral throttle + centered steering).
 * - Requires explicit arming before applying steer/throttle.
 * - Enforces a watchdog timeout: if commands stop arriving, force neutral.
 * - Provides a dedicated emergency-stop topic that always forces neutral.
 * - Makes MQTT host/port/topics configurable via config, env, or CLI.
 *
 * Build notes (dependencies already used elsewhere in this repo):
 *   - libmosquitto-dev
 *   - nlohmann-json
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include <mosquitto.h>
#include <nlohmann/json.hpp>

#include "config.h"
#include "logger.h"
#include "pca9685_servo_driver.h"
#include "servo.h"
#include "speed_controller.h"

// ------------------------------
// Small utilities
// ------------------------------
namespace {

constexpr int kQosAtLeastOnce = 1;

[[nodiscard]] std::optional<std::string> get_env(std::string_view key)
{
  // Why: we support env overrides so operators can quickly swap brokers/topics
  // without editing JSON on-device.
  if (key.empty()) return std::nullopt;
  auto* v = std::getenv(std::string(key).c_str());
  if (!v || std::strlen(v) == 0) return std::nullopt;
  return std::string(v);
}

[[nodiscard]] std::optional<int> parse_int(std::string_view s)
{
  if (s.empty()) return std::nullopt;
  try {
    return std::stoi(std::string(s));
  } catch (...) {
    return std::nullopt;
  }
}

[[nodiscard]] std::string env_key(std::string_view suffix)
{
  return std::string("AUTOBUS_") + std::string(suffix);
}

} // namespace

struct RuntimeMqttConfig
{
  std::string host;
  int port = 1883;
  int keepalive_secs = 30;

  std::string topic_drive;
  std::string topic_arm;
  std::string topic_estop;
};

struct RuntimeSafetyConfig
{
  bool start_armed = false;
  int watchdog_timeout_ms = 500;
  bool neutral_on_boot = true;
};

struct BusContext
{
  Config servo_calibration;

  // Hardware
  PCA9685ServoDriver pca_9685_driver;
  std::unique_ptr<Servo> servo;
  std::unique_ptr<Esc> esc;

  // Runtime config (resolved from config file + env + CLI)
  RuntimeMqttConfig mqtt;
  RuntimeSafetyConfig safety;

  // Safety state
  std::atomic<bool> armed{false};
  std::atomic<bool> estop_latched{false};

  // For telemetry + watchdog
  std::atomic<int64_t> last_drive_cmd_ms{0};

  // last applied values (for logging/observability)
  std::atomic<float> last_steer{0.0f};
  std::atomic<float> last_throttle{0.0f};
};

[[nodiscard]] int64_t now_monotonic_ms()
{
  auto const now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

// PUBLIC_INTERFACE
void force_safe_state(BusContext& ctx, char const* reason)
{
  /** Force safe actuator outputs: steering center + ESC neutral.
   *
   * Params:
   *  - ctx: bus controller context holding servo/esc objects
   *  - reason: short reason string for logs
   *
   * Return:
   *  - (void) logs failures but always attempts both operations
   */
  // Why: e-stop/watchdog must always attempt to neutralize outputs regardless of
  // current state. We log errors but continue so a partial failure is visible.
  if (ctx.esc) {
    if (auto r = ctx.esc->stop(); !r) {
      ALOG_ERROR("force_safe_state esc->stop failed (%s): %s", reason, r.error().c_str());
    }
  }
  if (ctx.servo) {
    if (auto r = ctx.servo->center(); !r) {
      ALOG_ERROR("force_safe_state servo->center failed (%s): %s", reason, r.error().c_str());
    }
  }

  ctx.last_steer.store(0.0f);
  ctx.last_throttle.store(0.0f);
}

[[nodiscard]] RuntimeMqttConfig resolve_mqtt_config(
    const Config& cfg,
    char const* cli_host,
    std::optional<int> cli_port,
    char const* cli_topic_drive,
    char const* cli_topic_arm,
    char const* cli_topic_estop)
{
  RuntimeMqttConfig out{};
  out.host = cfg.mqtt.host;
  out.port = cfg.mqtt.port;
  out.keepalive_secs = cfg.mqtt.keepalive_secs;
  out.topic_drive = cfg.mqtt.topic_drive;
  out.topic_arm = cfg.mqtt.topic_arm;
  out.topic_estop = cfg.mqtt.topic_estop;

  // Env overrides (highest priority after CLI).
  if (auto v = get_env(env_key("MQTT_HOST")); v) out.host = *v;
  if (auto v = get_env(env_key("MQTT_PORT")); v) {
    if (auto p = parse_int(*v); p) out.port = *p;
  }
  if (auto v = get_env(env_key("MQTT_KEEPALIVE_SECS")); v) {
    if (auto k = parse_int(*v); k) out.keepalive_secs = *k;
  }
  if (auto v = get_env(env_key("MQTT_TOPIC_DRIVE")); v) out.topic_drive = *v;
  if (auto v = get_env(env_key("MQTT_TOPIC_ARM")); v) out.topic_arm = *v;
  if (auto v = get_env(env_key("MQTT_TOPIC_ESTOP")); v) out.topic_estop = *v;

  // CLI overrides (highest priority).
  if (cli_host && std::strlen(cli_host) > 0) out.host = cli_host;
  if (cli_port) out.port = *cli_port;
  if (cli_topic_drive && std::strlen(cli_topic_drive) > 0) out.topic_drive = cli_topic_drive;
  if (cli_topic_arm && std::strlen(cli_topic_arm) > 0) out.topic_arm = cli_topic_arm;
  if (cli_topic_estop && std::strlen(cli_topic_estop) > 0) out.topic_estop = cli_topic_estop;

  return out;
}

[[nodiscard]] RuntimeSafetyConfig resolve_safety_config(const Config& cfg)
{
  RuntimeSafetyConfig out{};
  out.start_armed = cfg.safety.start_armed;
  out.watchdog_timeout_ms = cfg.safety.watchdog_timeout_ms;
  out.neutral_on_boot = cfg.safety.neutral_on_boot;

  // Env overrides for quick field tuning.
  if (auto v = get_env(env_key("START_ARMED")); v) out.start_armed = (*v == "1" || *v == "true");
  if (auto v = get_env(env_key("WATCHDOG_TIMEOUT_MS")); v) {
    if (auto ms = parse_int(*v); ms) out.watchdog_timeout_ms = *ms;
  }
  if (auto v = get_env(env_key("NEUTRAL_ON_BOOT")); v) out.neutral_on_boot = (*v == "1" || *v == "true");

  return out;
}

void on_connect(mosquitto* mosq, void* user_data, int status)
{
  auto* ctx = static_cast<BusContext*>(user_data);
  if (!ctx) {
    ALOG_FATAL("missing bus context");
    return;
  }

  if (status != 0) {
    ALOG_ERROR("connect failed: %s", mosquitto_connack_string(status));
    return;
  }

  ALOG_INFO("connected: broker=%s:%d keepalive=%ds", ctx->mqtt.host.c_str(), ctx->mqtt.port, ctx->mqtt.keepalive_secs);

  // Why: subscribe to control channels as separate topics to keep semantics
  // explicit and to allow "stop now" messages to bypass drive JSON parsing.
  mosquitto_subscribe(mosq, nullptr, ctx->mqtt.topic_drive.c_str(), kQosAtLeastOnce);
  mosquitto_subscribe(mosq, nullptr, ctx->mqtt.topic_arm.c_str(), kQosAtLeastOnce);
  mosquitto_subscribe(mosq, nullptr, ctx->mqtt.topic_estop.c_str(), kQosAtLeastOnce);

  ALOG_INFO("subscribed drive=%s arm=%s estop=%s",
            ctx->mqtt.topic_drive.c_str(),
            ctx->mqtt.topic_arm.c_str(),
            ctx->mqtt.topic_estop.c_str());
}

// PUBLIC_INTERFACE
void handle_estop(BusContext& ctx, std::string_view topic, std::string_view payload)
{
  /** Emergency stop handler. Always forces neutral and latches until explicitly cleared.
   *
   * Accepts:
   *  - Any payload (ignored) => triggers e-stop
   *  - JSON {"clear": true}  => clears latch (still requires arming to drive)
   */
  // Why: A latched stop prevents accidental re-application of throttle after a
  // transient stop message; operators must explicitly clear it.
  auto j = nlohmann::json::parse(payload, nullptr, false);

  bool clear = false;
  if (!j.is_discarded() && j.is_object()) {
    clear = j.value("clear", false);
  }

  if (clear) {
    ctx.estop_latched.store(false);
    force_safe_state(ctx, "estop_clear");
    ALOG_WARN("E-STOP CLEARED via topic=%.*s payload=%.*s",
              static_cast<int>(topic.size()), topic.data(),
              static_cast<int>(payload.size()), payload.data());
    return;
  }

  ctx.estop_latched.store(true);
  ctx.armed.store(false); // Why: drop armed on e-stop; must re-arm intentionally.
  force_safe_state(ctx, "estop");
  ALOG_FATAL("E-STOP TRIGGERED via topic=%.*s payload=%.*s",
             static_cast<int>(topic.size()), topic.data(),
             static_cast<int>(payload.size()), payload.data());
}

// PUBLIC_INTERFACE
void handle_arm(BusContext& ctx, std::string_view topic, std::string_view payload)
{
  /** Arming handler: toggles armed state if not e-stopped.
   *
   * Payload accepted:
   *  - JSON {"armed": true|false}
   *  - String "arm" / "disarm"
   */
  if (ctx.estop_latched.load()) {
    // Why: prevent re-arming while e-stop is latched; operator must clear e-stop first.
    ALOG_WARN("arm ignored (e-stop latched): topic=%.*s payload=%.*s",
              static_cast<int>(topic.size()), topic.data(),
              static_cast<int>(payload.size()), payload.data());
    return;
  }

  bool target = false;
  bool parsed = false;

  auto j = nlohmann::json::parse(payload, nullptr, false);
  if (!j.is_discarded() && j.is_object() && j.contains("armed")) {
    target = j.value("armed", false);
    parsed = true;
  } else {
    if (payload == "arm") {
      target = true;
      parsed = true;
    } else if (payload == "disarm") {
      target = false;
      parsed = true;
    }
  }

  if (!parsed) {
    ALOG_WARN("arm: unrecognized payload: %.*s", static_cast<int>(payload.size()), payload.data());
    return;
  }

  ctx.armed.store(target);
  // Why: when disarming, immediately force neutral to prevent lingering throttle.
  if (!target) {
    force_safe_state(ctx, "disarm");
  }
  ALOG_INFO("armed=%s (topic=%.*s payload=%.*s)",
            target ? "true" : "false",
            static_cast<int>(topic.size()), topic.data(),
            static_cast<int>(payload.size()), payload.data());
}

void handle_drive(BusContext& ctx, std::string_view topic, std::string_view payload)
{
  // Always update last command time on valid JSON (even if disarmed) so logs
  // show operator activity, but only apply outputs when armed.
  auto payload_json = nlohmann::json::parse(payload, nullptr, false);
  if (payload_json.is_discarded() || !payload_json.is_object()) {
    ALOG_WARN("%.*s invalid json payload: %.*s",
              static_cast<int>(topic.size()), topic.data(),
              static_cast<int>(payload.size()), payload.data());
    return;
  }

  ctx.last_drive_cmd_ms.store(now_monotonic_ms());

  if (ctx.estop_latched.load()) {
    // Why: drive commands are ignored while e-stop is active; safety first.
    ALOG_WARN("drive ignored (e-stop latched): %s", payload_json.dump().c_str());
    return;
  }

  if (!ctx.armed.load()) {
    // Why: arming gate prevents motion on boot or after an operator stop.
    ALOG_INFO("drive ignored (disarmed): %s", payload_json.dump().c_str());
    return;
  }

  // Apply steering/throttle with clamps.
  if (payload_json.contains("steer")) {
    float const steer = std::clamp(payload_json["steer"].get<float>(), -1.0f, 1.0f);
    if (auto r = ctx.servo->steer(steer); !r) {
      ALOG_ERROR("steer: %s", r.error().c_str());
    } else {
      ctx.last_steer.store(steer);
    }
  }

  if (payload_json.contains("throttle")) {
    float const throttle = std::clamp(payload_json["throttle"].get<float>(), -1.0f, 1.0f);
    auto const result = (throttle == 0.0f)
        ? ctx.esc->stop()
        : ctx.esc->throttle(throttle);
    if (!result) {
      ALOG_ERROR("throttle: %s", result.error().c_str());
    } else {
      ctx.last_throttle.store(throttle);
    }
  }

  ALOG_DEBUG("[drive topic=%.*s] armed=%s steer=%.3f throttle=%.3f json=%s",
             static_cast<int>(topic.size()), topic.data(),
             ctx.armed.load() ? "true" : "false",
             ctx.last_steer.load(),
             ctx.last_throttle.load(),
             payload_json.dump().c_str());
}

void on_message(mosquitto* /*mosq*/, void* user_data, const mosquitto_message* msg)
{
  auto* ctx = static_cast<BusContext*>(user_data);
  if (!ctx) {
    ALOG_FATAL("missing bus context");
    return;
  }
  if (!msg || !msg->topic) {
    return;
  }

  std::string_view topic{msg->topic};

  auto const payload_len = msg->payloadlen > 0 ? static_cast<size_t>(msg->payloadlen) : size_t{0};
  std::string const payload{
      payload_len == 0 ? "" : static_cast<char const*>(msg->payload),
      payload_len};

  if (topic == ctx->mqtt.topic_estop) {
    handle_estop(*ctx, topic, payload);
    return;
  }
  if (topic == ctx->mqtt.topic_arm) {
    handle_arm(*ctx, topic, payload);
    return;
  }
  if (topic == ctx->mqtt.topic_drive) {
    handle_drive(*ctx, topic, payload);
    return;
  }

  // Why: if we got here, we're subscribed to something unexpected; log it.
  ALOG_WARN("message on unexpected topic=%.*s payload=%s",
            static_cast<int>(topic.size()), topic.data(),
            payload.c_str());
}

void watchdog_thread(BusContext* ctx)
{
  if (!ctx) return;

  // Why: initializing the timestamp to "now" prevents an immediate watchdog stop
  // at boot if start_armed=true.
  ctx->last_drive_cmd_ms.store(now_monotonic_ms());

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (ctx->estop_latched.load()) {
      // Keep enforcing safe outputs while e-stop is latched.
      force_safe_state(*ctx, "estop_latched");
      continue;
    }

    if (!ctx->armed.load()) {
      // Disarmed => hold safe outputs.
      force_safe_state(*ctx, "disarmed_hold");
      continue;
    }

    const int watchdog_ms = std::max(0, ctx->safety.watchdog_timeout_ms);
    if (watchdog_ms == 0) {
      // Why: allow disabling watchdog by setting timeout to 0 for bench testing.
      continue;
    }

    const int64_t last_ms = ctx->last_drive_cmd_ms.load();
    const int64_t now_ms = now_monotonic_ms();
    if ((now_ms - last_ms) > watchdog_ms) {
      // Why: fail-safe on loss of drive commands: neutralize throttle and center
      // steering. We keep "armed" true, so when commands resume the operator
      // doesn't need to re-arm; they can still disarm explicitly if desired.
      force_safe_state(*ctx, "watchdog_timeout");
      ALOG_WARN("watchdog timeout: no drive cmd for %lldms (threshold=%dms)",
                static_cast<long long>(now_ms - last_ms), watchdog_ms);
      // Avoid spamming logs at 20Hz once in timeout state.
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
}

int main(int argc, char* argv[])
{
  // Minimal CLI:
  //   autobus [config.json] [--mqtt-host HOST] [--mqtt-port PORT] [--topic-drive T] [--topic-arm T] [--topic-estop T]
  std::string_view config_path = (argc > 1) ? argv[1] : "config.json";

  char const* cli_mqtt_host = nullptr;
  std::optional<int> cli_mqtt_port = std::nullopt;
  char const* cli_topic_drive = nullptr;
  char const* cli_topic_arm = nullptr;
  char const* cli_topic_estop = nullptr;

  for (int i = 2; i < argc; ++i) {
    std::string_view a = argv[i];
    auto next = [&]() -> char const* {
      if (i + 1 >= argc) return nullptr;
      return argv[++i];
    };

    if (a == "--mqtt-host") cli_mqtt_host = next();
    else if (a == "--mqtt-port") {
      if (auto* v = next(); v) {
        if (auto p = parse_int(v); p) cli_mqtt_port = *p;
      }
    }
    else if (a == "--topic-drive") cli_topic_drive = next();
    else if (a == "--topic-arm") cli_topic_arm = next();
    else if (a == "--topic-estop") cli_topic_estop = next();
  }

  // Load config
  auto cfg_result = load_config(config_path);
  if (!cfg_result) {
    ALOG_FATAL("%s", cfg_result.error().c_str());
    return 1;
  }

  auto ctx = std::make_unique<BusContext>();
  ctx->servo_calibration = std::move(*cfg_result);

  // Resolve runtime config
  ctx->mqtt = resolve_mqtt_config(ctx->servo_calibration, cli_mqtt_host, cli_mqtt_port, cli_topic_drive, cli_topic_arm, cli_topic_estop);
  ctx->safety = resolve_safety_config(ctx->servo_calibration);
  ctx->armed.store(ctx->safety.start_armed);

  ALOG_INFO("config: i2c=%s neutral_on_boot=%s start_armed=%s watchdog_timeout_ms=%d",
            ctx->servo_calibration.i2c_device.c_str(),
            ctx->safety.neutral_on_boot ? "true" : "false",
            ctx->safety.start_armed ? "true" : "false",
            ctx->safety.watchdog_timeout_ms);

  if (auto status = ctx->pca_9685_driver.open(ctx->servo_calibration.i2c_device); !status) {
    ALOG_FATAL("%s", status.error().c_str());
    return 1;
  }

  if (auto status = ctx->pca_9685_driver.setPwmFreq(50.0f); !status) {
    ALOG_FATAL("%s", status.error().c_str());
    return 1;
  }

  ctx->servo = std::make_unique<Servo>(ctx->pca_9685_driver, ctx->servo_calibration.servo);
  ctx->esc = std::make_unique<Esc>(ctx->pca_9685_driver, ctx->servo_calibration.esc);

  // Default-safe boot state.
  if (ctx->safety.neutral_on_boot) {
    // Why: outputs should never be left in an undefined state at boot; this
    // ensures throttle is neutral before any MQTT traffic arrives.
    force_safe_state(*ctx, "boot");
  }

  mosquitto_lib_init();

  auto* mosq = mosquitto_new(nullptr, true, ctx.get());
  if (!mosq) {
    ALOG_FATAL("failed to create mosquitto client");
    return 1;
  }

  mosquitto_connect_callback_set(mosq, on_connect);
  mosquitto_message_callback_set(mosq, on_message);

  if (mosquitto_connect(mosq, ctx->mqtt.host.c_str(), ctx->mqtt.port, ctx->mqtt.keepalive_secs) != MOSQ_ERR_SUCCESS) {
    ALOG_FATAL("unable to connect to %s:%d", ctx->mqtt.host.c_str(), ctx->mqtt.port);
    return 1;
  }

  // Start watchdog thread.
  std::thread wd(watchdog_thread, ctx.get());
  wd.detach();

  // Main loop (blocking)
  mosquitto_loop_forever(mosq, -1, 1);

  mosquitto_destroy(mosq);
  mosquitto_lib_cleanup();
  return 0;
}
