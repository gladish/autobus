/*
 * autonomy_main.cpp
 *
 * Minimal Milestone-1 autonomy loop:
 *  - Consumes camera video via MediaMTX RTSP/H264.
 *  - Uses OpenCV for a lightweight edge/lane-follow heuristic.
 *  - Runs a small finite-state machine (FOLLOW / TURN / STOPPED).
 *  - Publishes steer/throttle to the existing bus drive topic (MQTT).
 *  - Fails safe: on low confidence it stops and can optionally disarm.
 *
 * Why this exists as a separate binary:
 *  - Keeps the safety-hardened busctld (actuation + watchdog + e-stop) minimal.
 *  - Autonomy can crash/restart without destabilizing the actuator process.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include <mosquitto.h>
#include <nlohmann/json.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "config.h"
#include "logger.h"

namespace {

constexpr int kQosAtLeastOnce = 1;

[[nodiscard]] int64_t now_monotonic_ms()
{
  auto const now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

[[nodiscard]] std::optional<std::string> get_env(std::string_view key)
{
  // Why: operators may want to override camera URL/broker quickly without
  // editing JSON on-device during iteration.
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

[[nodiscard]] float clampf(float v, float lo, float hi) { return std::min(hi, std::max(lo, v)); }

struct AutonomyRuntime
{
  Config cfg;

  // MQTT runtime (resolved from config + env + CLI)
  std::string mqtt_host;
  int mqtt_port = 1883;
  int mqtt_keepalive_secs = 30;
  std::string topic_drive;
  std::string topic_arm;
  std::string topic_estop;

  std::string camera_url;

  // publishing / safety state
  std::atomic<bool> stop_requested{false};
};

// PUBLIC_INTERFACE
[[nodiscard]] bool mqtt_publish_json(mosquitto* mosq, std::string const& topic, nlohmann::json const& payload)
{
  /** Publish a JSON payload to MQTT.
   *
   * Params:
   *  - mosq: mosquitto client
   *  - topic: destination topic
   *  - payload: JSON to publish
   *
   * Return:
   *  - true on success, false on error (and logs an error)
   */
  auto const s = payload.dump();
  int rc = mosquitto_publish(
      mosq,
      nullptr,
      topic.c_str(),
      static_cast<int>(s.size()),
      s.c_str(),
      kQosAtLeastOnce,
      false);
  if (rc != MOSQ_ERR_SUCCESS) {
    ALOG_ERROR("mqtt publish failed: topic=%s err=%s", topic.c_str(), mosquitto_strerror(rc));
    return false;
  }
  return true;
}

// PUBLIC_INTERFACE
[[nodiscard]] bool publish_drive(mosquitto* mosq, AutonomyRuntime const& rt, float steer, float throttle)
{
  /** Publish a drive command compatible with busctld's drive listener.
   *
   * Params:
   *  - steer: [-1, 1]
   *  - throttle: [-1, 1]
   */
  nlohmann::json j;
  j["steer"] = clampf(steer, -1.0f, 1.0f);
  j["throttle"] = clampf(throttle, -1.0f, 1.0f);
  return mqtt_publish_json(mosq, rt.topic_drive, j);
}

// PUBLIC_INTERFACE
[[nodiscard]] bool publish_arm(mosquitto* mosq, AutonomyRuntime const& rt, bool armed)
{
  /** Request arm/disarm via the dedicated arm topic.
   *
   * Why: autonomy should not silently cause motion unless explicitly armed; this
   * binary defaults to disarmed and only requests arming if started with --arm.
   */
  nlohmann::json j;
  j["armed"] = armed;
  return mqtt_publish_json(mosq, rt.topic_arm, j);
}

// PUBLIC_INTERFACE
[[nodiscard]] bool publish_estop(mosquitto* mosq, AutonomyRuntime const& rt, bool clear)
{
  /** Publish an e-stop message (or clear) to the dedicated topic. */
  nlohmann::json j;
  if (clear) {
    j["clear"] = true;
  } else {
    j["stop"] = true;
  }
  return mqtt_publish_json(mosq, rt.topic_estop, j);
}

struct VisionResult
{
  float steer = 0.0f;       // suggested steer [-1, 1]
  float confidence = 0.0f;  // 0..1
  bool corner = false;      // heuristic "corner / junction" detection
};

// PUBLIC_INTERFACE
[[nodiscard]] VisionResult compute_steer_from_frame(cv::Mat const& bgr)
{
  /** Compute a simple steering suggestion from a BGR image.
   *
   * Algorithm (minimal + robust-ish for first outdoor iterations):
   *  1) Crop bottom half (road/sidewalk likely occupies lower region).
   *  2) Convert to grayscale + blur.
   *  3) Canny edges.
   *  4) Compute horizontal centroid of edge pixels -> lateral error.
   *  5) Confidence is edge density in ROI, corner is when edges are sparse in
   *     the far half but strong near bottom (a crude "dead-end/turn" signal).
   */
  VisionResult out{};

  if (bgr.empty()) return out;

  int const h = bgr.rows;
  int const w = bgr.cols;
  if (h < 20 || w < 20) return out;

  cv::Rect roi(0, h / 2, w, h - h / 2);
  cv::Mat cropped = bgr(roi);

  cv::Mat gray;
  cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

  cv::Mat edges;
  cv::Canny(gray, edges, 60, 160);

  int edge_count = cv::countNonZero(edges);
  int total = edges.rows * edges.cols;
  if (total <= 0) return out;

  // Edge density as a proxy for confidence.
  float density = static_cast<float>(edge_count) / static_cast<float>(total);
  out.confidence = clampf(density * 8.0f, 0.0f, 1.0f);  // scale to a usable 0..1 band

  // Centroid: sum x of all edge pixels. Use moments for speed/clarity.
  cv::Moments m = cv::moments(edges, true);
  if (m.m00 > 1e-3) {
    float cx = static_cast<float>(m.m10 / m.m00);  // 0..w
    float error = (cx - (static_cast<float>(w) / 2.0f)) / (static_cast<float>(w) / 2.0f);  // -1..1
    // Negative error => edges centroid left => steer left (negative).
    out.steer = clampf(error, -1.0f, 1.0f);
  } else {
    out.steer = 0.0f;
    out.confidence = 0.0f;
  }

  // Corner heuristic:
  // - compare edge density in upper quarter of ROI vs lower quarter.
  // Why: approaching a corner/driveway cut often changes edge distribution;
  // this provides a cheap trigger to switch into a timed turn state.
  int const rh = edges.rows;
  cv::Rect top_q(0, 0, w, std::max(1, rh / 4));
  cv::Rect bot_q(0, std::max(0, rh - rh / 4), w, std::max(1, rh / 4));
  float top_density = static_cast<float>(cv::countNonZero(edges(top_q))) / static_cast<float>(top_q.area());
  float bot_density = static_cast<float>(cv::countNonZero(edges(bot_q))) / static_cast<float>(bot_q.area());
  out.corner = (bot_density > 0.06f) && (top_density < 0.015f);

  return out;
}

enum class State
{
  Follow,
  Turn,
  Stopped,
};

}  // namespace

int main(int argc, char* argv[])
{
  // Minimal CLI:
  //   autonomy [config.json]
  //     [--arm] [--disarm-on-stop]
  //     [--camera-rtsp URL]
  //     [--mqtt-host HOST] [--mqtt-port PORT]
  //     [--topic-drive T] [--topic-arm T] [--topic-estop T]
  //     [--turn-left | --turn-right]
  std::string_view config_path = (argc > 1) ? argv[1] : "config.json";

  bool cli_arm = false;
  bool cli_disarm_on_stop = true;
  char const* cli_camera_url = nullptr;

  char const* cli_mqtt_host = nullptr;
  std::optional<int> cli_mqtt_port = std::nullopt;
  char const* cli_topic_drive = nullptr;
  char const* cli_topic_arm = nullptr;
  char const* cli_topic_estop = nullptr;

  // Turning direction preference (sidewalk loops may prefer consistent turns).
  float turn_dir = 1.0f;  // +1 right, -1 left

  for (int i = 2; i < argc; ++i) {
    std::string_view a = argv[i];
    auto next = [&]() -> char const* {
      if (i + 1 >= argc) return nullptr;
      return argv[++i];
    };

    if (a == "--arm") cli_arm = true;
    else if (a == "--no-disarm-on-stop") cli_disarm_on_stop = false;
    else if (a == "--camera-rtsp") cli_camera_url = next();
    else if (a == "--mqtt-host") cli_mqtt_host = next();
    else if (a == "--mqtt-port") {
      if (auto* v = next(); v) {
        if (auto p = parse_int(v); p) cli_mqtt_port = *p;
      }
    }
    else if (a == "--topic-drive") cli_topic_drive = next();
    else if (a == "--topic-arm") cli_topic_arm = next();
    else if (a == "--topic-estop") cli_topic_estop = next();
    else if (a == "--turn-left") turn_dir = -1.0f;
    else if (a == "--turn-right") turn_dir = 1.0f;
  }

  auto cfg_result = load_config(config_path);
  if (!cfg_result) {
    ALOG_FATAL("%s", cfg_result.error().c_str());
    return 1;
  }

  AutonomyRuntime rt{};
  rt.cfg = std::move(*cfg_result);

  // Resolve MQTT settings: config -> env -> CLI.
  rt.mqtt_host = rt.cfg.mqtt.host;
  rt.mqtt_port = rt.cfg.mqtt.port;
  rt.mqtt_keepalive_secs = rt.cfg.mqtt.keepalive_secs;
  rt.topic_drive = rt.cfg.mqtt.topic_drive;
  rt.topic_arm = rt.cfg.mqtt.topic_arm;
  rt.topic_estop = rt.cfg.mqtt.topic_estop;

  if (auto v = get_env("AUTOBUS_MQTT_HOST"); v) rt.mqtt_host = *v;
  if (auto v = get_env("AUTOBUS_MQTT_PORT"); v) {
    if (auto p = parse_int(*v); p) rt.mqtt_port = *p;
  }
  if (auto v = get_env("AUTOBUS_MQTT_KEEPALIVE_SECS"); v) {
    if (auto k = parse_int(*v); k) rt.mqtt_keepalive_secs = *k;
  }
  if (auto v = get_env("AUTOBUS_MQTT_TOPIC_DRIVE"); v) rt.topic_drive = *v;
  if (auto v = get_env("AUTOBUS_MQTT_TOPIC_ARM"); v) rt.topic_arm = *v;
  if (auto v = get_env("AUTOBUS_MQTT_TOPIC_ESTOP"); v) rt.topic_estop = *v;

  if (cli_mqtt_host && std::strlen(cli_mqtt_host) > 0) rt.mqtt_host = cli_mqtt_host;
  if (cli_mqtt_port) rt.mqtt_port = *cli_mqtt_port;
  if (cli_topic_drive && std::strlen(cli_topic_drive) > 0) rt.topic_drive = cli_topic_drive;
  if (cli_topic_arm && std::strlen(cli_topic_arm) > 0) rt.topic_arm = cli_topic_arm;
  if (cli_topic_estop && std::strlen(cli_topic_estop) > 0) rt.topic_estop = cli_topic_estop;

  // Resolve camera URL.
  rt.camera_url = rt.cfg.autonomy.camera_rtsp_url;
  if (auto v = get_env("AUTOBUS_CAMERA_RTSP_URL"); v) rt.camera_url = *v;
  if (cli_camera_url && std::strlen(cli_camera_url) > 0) rt.camera_url = cli_camera_url;

  ALOG_INFO("autonomy config: camera=%s control_hz=%d throttle_base=%.3f max_abs_steer=%.3f",
            rt.camera_url.c_str(),
            rt.cfg.autonomy.control_hz,
            rt.cfg.autonomy.throttle_base,
            rt.cfg.autonomy.max_abs_steer);
  ALOG_INFO("mqtt: broker=%s:%d drive=%s arm=%s estop=%s",
            rt.mqtt_host.c_str(),
            rt.mqtt_port,
            rt.topic_drive.c_str(),
            rt.topic_arm.c_str(),
            rt.topic_estop.c_str());

  // Init MQTT
  mosquitto_lib_init();
  mosquitto* mosq = mosquitto_new(nullptr, true, nullptr);
  if (!mosq) {
    ALOG_FATAL("failed to create mosquitto client");
    return 1;
  }
  if (mosquitto_connect(mosq, rt.mqtt_host.c_str(), rt.mqtt_port, rt.mqtt_keepalive_secs) != MOSQ_ERR_SUCCESS) {
    ALOG_FATAL("unable to connect to %s:%d", rt.mqtt_host.c_str(), rt.mqtt_port);
    return 1;
  }

  // Always start from a safe output request (stop + center).
  // Why: if busctld is armed from a previous run, we want to immediately request neutral outputs.
  (void)publish_drive(mosq, rt, 0.0f, 0.0f);

  if (cli_arm) {
    // Why: arming should be an explicit operator/autonomy launcher decision.
    (void)publish_arm(mosq, rt, true);
  }

  // OpenCV: open RTSP stream via backend's default (FFmpeg on most Linux builds).
  cv::VideoCapture cap;
  if (!cap.open(rt.camera_url)) {
    ALOG_FATAL("failed to open camera stream: %s", rt.camera_url.c_str());
    return 1;
  }

  // State machine
  State state = State::Follow;
  int64_t state_until_ms = 0;

  int64_t low_conf_start_ms = -1;
  int64_t last_log_ms = 0;

  int const hz = std::max(1, rt.cfg.autonomy.control_hz);
  int const period_ms = 1000 / hz;

  ALOG_INFO("autonomy loop start: period_ms=%d turn_dir=%s disarm_on_stop=%s",
            period_ms,
            (turn_dir < 0.0f) ? "left" : "right",
            cli_disarm_on_stop ? "true" : "false");

  while (true) {
    // Keep mosquitto networking pumping without blocking the control cadence.
    mosquitto_loop(mosq, 0, 1);

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
      // Why: camera outage must immediately lead to safe stop; frame loss is effectively 0 confidence.
      ALOG_ERROR("camera frame read failed -> STOP");
      state = State::Stopped;
    }

    VisionResult vr = compute_steer_from_frame(frame);

    // Low-confidence handling (grace period before hard stop).
    if (vr.confidence < rt.cfg.autonomy.min_confidence_stop) {
      if (low_conf_start_ms < 0) low_conf_start_ms = now_monotonic_ms();
    } else {
      low_conf_start_ms = -1;
    }

    int64_t const now_ms = now_monotonic_ms();
    bool const low_conf_too_long =
        (low_conf_start_ms >= 0) && ((now_ms - low_conf_start_ms) > rt.cfg.autonomy.low_confidence_grace_ms);

    if (low_conf_too_long) {
      // Why: prolonged perception uncertainty is treated as unsafe; stop and require re-arming.
      ALOG_FATAL("low confidence for %lldms -> STOP",
                 static_cast<long long>(now_ms - low_conf_start_ms));
      state = State::Stopped;
    }

    float steer_cmd = 0.0f;
    float throttle_cmd = 0.0f;

    if (state == State::Follow) {
      // If "corner" detected, transition to TURN for a fixed time.
      if (vr.corner && vr.confidence >= rt.cfg.autonomy.min_confidence_slow) {
        state = State::Turn;
        state_until_ms = now_ms + rt.cfg.autonomy.turn_ms;
        ALOG_WARN("corner detected -> TURN for %dms", rt.cfg.autonomy.turn_ms);
      }

      // Steer proportional to lateral error (clamped), throttle reduced if confidence is not great.
      steer_cmd = clampf(vr.steer, -rt.cfg.autonomy.max_abs_steer, rt.cfg.autonomy.max_abs_steer);

      if (vr.confidence >= rt.cfg.autonomy.min_confidence_slow) {
        throttle_cmd = rt.cfg.autonomy.throttle_base;
      } else if (vr.confidence >= rt.cfg.autonomy.min_confidence_stop) {
        // Why: if we see *something* but it's weak, go slower instead of abruptly stopping.
        throttle_cmd = rt.cfg.autonomy.throttle_base * 0.6f;
      } else {
        throttle_cmd = 0.0f;
      }
    } else if (state == State::Turn) {
      if (now_ms >= state_until_ms) {
        state = State::Follow;
        ALOG_INFO("TURN complete -> FOLLOW");
      } else {
        steer_cmd = clampf(turn_dir * rt.cfg.autonomy.turn_steer, -rt.cfg.autonomy.max_abs_steer, rt.cfg.autonomy.max_abs_steer);
        throttle_cmd = rt.cfg.autonomy.throttle_turn;
      }
    } else {  // Stopped
      steer_cmd = 0.0f;
      throttle_cmd = 0.0f;

      // Force a stop command immediately; disarm as an extra guard if configured.
      (void)publish_drive(mosq, rt, 0.0f, 0.0f);
      if (cli_disarm_on_stop) {
        (void)publish_arm(mosq, rt, false);
      }
      // Also publish an e-stop to hard-latch if busctld is listening.
      (void)publish_estop(mosq, rt, false);

      ALOG_FATAL("AUTONOMY STOPPED (confidence=%.3f). Exiting.", vr.confidence);
      break;
    }

    // Publish drive command for this tick.
    (void)publish_drive(mosq, rt, steer_cmd, throttle_cmd);

    // Periodic telemetry log (avoid spamming).
    if ((now_ms - last_log_ms) > 500) {
      last_log_ms = now_ms;
      char const* state_name = (state == State::Follow) ? "FOLLOW" : (state == State::Turn) ? "TURN" : "STOPPED";
      ALOG_INFO("state=%s steer=%.3f throttle=%.3f conf=%.3f corner=%s",
                state_name,
                steer_cmd,
                throttle_cmd,
                vr.confidence,
                vr.corner ? "true" : "false");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  cap.release();
  mosquitto_disconnect(mosq);
  mosquitto_destroy(mosq);
  mosquitto_lib_cleanup();
  return 0;
}
