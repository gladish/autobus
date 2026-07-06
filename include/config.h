#pragma once

#include <expected>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

// ---------------------------------------------------------------------------
// Config — mirrors the structure produced by calibrate.py
// ---------------------------------------------------------------------------
struct ServoConfig {
  int channel = 0;
  int center_us = 1500;
  int left_us = 1000;
  int right_us = 2000;
  bool reverse = false;
};

struct EscConfig {
  int channel = 8;
  int neutral_us = 1500;
  int creep_us = 1560;
  int max_forward_us = 2000;
  int reverse_creep_us = 1440;
  int max_reverse_us = 1000;
  int reverse_brake_ms = 180;
  int reverse_neutral_ms = 180;
};

struct Config {
  std::string i2c_device = "/dev/i2c-1";
  ServoConfig servo;
  EscConfig esc;
  // Safety/control plane configuration is optional and has sensible defaults so
  // existing calibration-only config files remain valid.
  struct MqttConfig {
    std::string host = "localhost";
    int port = 1883;
    int keepalive_secs = 30;
    std::string topic_drive = "bus/cmd/drive";
    // Dedicated emergency stop channel. This is intentionally separate from the
    // drive topic so "stop now" can bypass any higher-level command structure.
    std::string topic_estop = "bus/cmd/estop";
    // Optional arming channel; messages can toggle armed/disarmed.
    std::string topic_arm = "bus/cmd/arm";
  } mqtt;

  struct SafetyConfig {
    // When false, the process starts disarmed and will ignore throttle/steer
    // commands (except e-stop) until explicitly armed.
    bool start_armed = false;
    // If we haven't received a valid drive command within this window while
    // armed, the vehicle is forced to neutral and remains "armed but stopped".
    // This prevents runaways if the *drive-command stream* stops (e.g. autonomy
    // controller crashes, the MQTT broker/control path dies, or the publisher
    // stops sending). This is intentionally *not* tied to any WAN/Internet
    // monitoring uplink; autonomy can be fully on-vehicle.
    int watchdog_timeout_ms = 500;
    // If true, we apply "neutral throttle + centered steering" immediately
    // after initializing the PCA9685 outputs.
    bool neutral_on_boot = true;
  } safety;

  // Autonomy/navigation config: used by the minimal vision-based autonomy loop.
  // Why: keeping autonomy parameters in the same JSON file makes it easy to
  // tune thresholds in the field while reusing the same MQTT broker/topics.
  struct AutonomyConfig {
    // RTSP URL to consume from MediaMTX (rpiCamera -> RTSP/H264).
    // Example: "rtsp://127.0.0.1:8554/cam"
    std::string camera_rtsp_url = "rtsp://127.0.0.1:8554/cam";
    // Control loop rate for autonomy decisions (publish cadence).
    int control_hz = 10;
    // Base throttle (0..1) used while confidently following.
    float throttle_base = 0.10f;
    // Hard clamp on steering output (safety / stability).
    float max_abs_steer = 0.65f;
    // Confidence thresholds.
    float min_confidence_stop = 0.35f;
    float min_confidence_slow = 0.55f;
    // If confidence stays low for too long, disengage (stop + disarm request).
    int low_confidence_grace_ms = 1200;
    // Simple turning behavior.
    int turn_ms = 650;
    float turn_steer = 0.55f;
    float throttle_turn = 0.08f;
  } autonomy;
};

// ---------------------------------------------------------------------------
// JSON deserialization (nlohmann)
// ---------------------------------------------------------------------------
inline void from_json(const nlohmann::json& j, ServoConfig& s)
{
  j.at("channel").get_to(s.channel);
  j.at("center_us").get_to(s.center_us);
  j.at("left_us").get_to(s.left_us);
  j.at("right_us").get_to(s.right_us);
  // "reverse" is the preferred key; "invert" remains for compatibility.
  s.reverse = j.value("reverse", j.value("invert", s.reverse));
}

inline void from_json(const nlohmann::json& j, EscConfig& e)
{
  j.at("channel").get_to(e.channel);
  j.at("neutral_us").get_to(e.neutral_us);
  j.at("creep_us").get_to(e.creep_us);
  j.at("max_forward_us").get_to(e.max_forward_us);
  e.reverse_creep_us = j.value("reverse_creep_us", e.reverse_creep_us);
  e.max_reverse_us = j.value("max_reverse_us", e.max_reverse_us);
  e.reverse_brake_ms = j.value("reverse_brake_ms", e.reverse_brake_ms);
  e.reverse_neutral_ms = j.value("reverse_neutral_ms", e.reverse_neutral_ms);
}

inline void from_json(const nlohmann::json& j, Config& c)
{
  c.i2c_device = j.value("i2c_device", c.i2c_device);
  j.at("servo").get_to(c.servo);
  j.at("esc").get_to(c.esc);

  // Optional MQTT config (kept flexible for different networks/environments).
  if (j.contains("mqtt")) {
    const auto& jm = j.at("mqtt");
    c.mqtt.host = jm.value("host", c.mqtt.host);
    c.mqtt.port = jm.value("port", c.mqtt.port);
    c.mqtt.keepalive_secs = jm.value("keepalive_secs", c.mqtt.keepalive_secs);
    c.mqtt.topic_drive = jm.value("topic_drive", c.mqtt.topic_drive);
    c.mqtt.topic_estop = jm.value("topic_estop", c.mqtt.topic_estop);
    c.mqtt.topic_arm = jm.value("topic_arm", c.mqtt.topic_arm);
  }

  // Optional safety config (defaults preserve prior behavior except that the
  // new features are available without breaking old configs).
  if (j.contains("safety")) {
    const auto& js = j.at("safety");
    c.safety.start_armed = js.value("start_armed", c.safety.start_armed);
    c.safety.watchdog_timeout_ms = js.value("watchdog_timeout_ms", c.safety.watchdog_timeout_ms);
    c.safety.neutral_on_boot = js.value("neutral_on_boot", c.safety.neutral_on_boot);
  }

  // Optional autonomy config (defaults allow starting without autonomy keys).
  if (j.contains("autonomy")) {
    const auto& ja = j.at("autonomy");
    c.autonomy.camera_rtsp_url = ja.value("camera_rtsp_url", c.autonomy.camera_rtsp_url);
    c.autonomy.control_hz = ja.value("control_hz", c.autonomy.control_hz);
    c.autonomy.throttle_base = ja.value("throttle_base", c.autonomy.throttle_base);
    c.autonomy.max_abs_steer = ja.value("max_abs_steer", c.autonomy.max_abs_steer);
    c.autonomy.min_confidence_stop = ja.value("min_confidence_stop", c.autonomy.min_confidence_stop);
    c.autonomy.min_confidence_slow = ja.value("min_confidence_slow", c.autonomy.min_confidence_slow);
    c.autonomy.low_confidence_grace_ms = ja.value("low_confidence_grace_ms", c.autonomy.low_confidence_grace_ms);
    c.autonomy.turn_ms = ja.value("turn_ms", c.autonomy.turn_ms);
    c.autonomy.turn_steer = ja.value("turn_steer", c.autonomy.turn_steer);
    c.autonomy.throttle_turn = ja.value("throttle_turn", c.autonomy.throttle_turn);
  }
}

// ---------------------------------------------------------------------------
// load_config — returns Config or an error string
// ---------------------------------------------------------------------------
[[nodiscard]] inline std::expected<Config, std::string> load_config(std::string_view path)
{
  std::FILE* f = std::fopen(path.data(), "r");
  if (!f)
    return std::unexpected(std::string("Cannot open config file: ") + path.data());

  try {
    nlohmann::json j = nlohmann::json::parse(f);
    std::fclose(f);
    return j.get<Config>();
  } catch (const nlohmann::json::exception& e) {
    std::fclose(f);
    return std::unexpected(std::string("JSON parse error: ") + e.what());
  }
}