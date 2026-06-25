/*
 * drive_listener.c
 *
 * Connects to the mosquitto broker and subscribes to bus/cmd/drive,
 * printing each message as it arrives.
 *
 * Build:
 *   sudo apt install libmosquitto-dev
 *   gcc drive_listener.c -o drive_listener -lmosquitto
 *
 * Run:
 *   ./drive_listener
 */

#include <algorithm>

#include <mosquitto.h>
#include <nlohmann/json.hpp>
#include <stdio.h>
#include <string.h>

#include "pwm_controller.h"
#include "speed_controller.h"
#include "servo.h"
#include "config.h"

static constexpr uint16_t kMqttPort = 1883;
static constexpr char const* kMqttHost = "100.113.97.42";
static constexpr int kMqttKeepaliveSecs = 30;
static constexpr char const kTopicCommandDrive[] = "bus/cmd/drive";

struct BusContext
{
  Config servo_calibration;
  PwmController pwm;
  std::unique_ptr<Servo> servo;
  std::unique_ptr<Esc> esc;
};

void on_connect(mosquitto* mosq, void* user_data, int status)
{
  auto* ctx = static_cast<BusContext*>(user_data);
  if (!ctx) {
    printf("missing bus context\n");
    return;
  }

  if (status == 0) {
    printf("connected\n");
    mosquitto_subscribe(mosq, nullptr, kTopicCommandDrive, 1 /* QoS */);
    printf("subscribed to %s\n", kTopicCommandDrive);
  }
  else {
    printf("connect failed: %s\n", mosquitto_connack_string(status));
  }
}

void on_message(mosquitto* /* mosq */, void* user_data, mosquitto_message const* msg)
{
  auto* ctx = static_cast<BusContext*>(user_data);
  if (!ctx) {
    printf("missing bus context\n");
    return;
  }

  if (!msg) {
    return;
  }

  auto const payload_len = msg->payloadlen > 0 ? static_cast<size_t>(msg->payloadlen) : size_t{0};
  std::string const buff{
      payload_len == 0 ? "" : static_cast<char const*>(msg->payload),
      payload_len};

  auto payload_json = nlohmann::json::parse(buff, nullptr, false);
  if (payload_json.is_discarded()) {
    printf("%s invalid json payload: %s\n", msg->topic, buff.c_str());
    return;
  }

  if (payload_json.contains("steer")) {
    float const steer = std::clamp(payload_json["steer"].get<float>(), -1.0f, 1.0f);
    if (auto r = ctx->servo->steer(steer); !r) {
      printf("[error] steer: %s\n", r.error().c_str());
    }
  }

  if (payload_json.contains("throttle")) {
    float const throttle = std::clamp(payload_json["throttle"].get<float>(), -1.0f, 1.0f);
    auto const result = (throttle == 0.0f)
      ? ctx->esc->stop()
      : ctx->esc->throttle(throttle);
    if (!result) {
      printf("[error] throttle: %s\n", result.error().c_str());
    }
  }

  printf("[%s] %s\n", msg->topic, payload_json.dump().c_str());
}

int main(int argc, char* argv[])
{
  std::string_view config_path = (argc > 1) ? argv[1] : "config.json";

  // Load config
  auto cfg_result = load_config(config_path);
  if (!cfg_result) {
    printf("[error] %s\n", cfg_result.error().c_str());
    return 1;
  }

  std::unique_ptr<BusContext> ctx = std::make_unique<BusContext>();
  ctx->servo_calibration = std::move(*cfg_result);;

  if (auto status = ctx->pwm.open("/dev/i2c-1"); !status) {
    printf("[error] %s\n", status.error().c_str());
    return 1;
  }

  if (auto status = ctx->pwm.set_pwm_freq(50.0f); !status) {
    printf("[error] %s\n", status.error().c_str());
    return 1;
  }

  ctx->servo = std::make_unique<Servo>(ctx->pwm, ctx->servo_calibration.servo);
  ctx->esc = std::make_unique<Esc>(ctx->pwm, ctx->servo_calibration.esc);

  mosquitto_lib_init();

  auto mosq = mosquitto_new(nullptr, true, ctx.get());
  if (!mosq) {
    printf("failed to create client\n");
    return 1;
  }

  mosquitto_connect_callback_set(mosq, on_connect);
  mosquitto_message_callback_set(mosq, on_message);

  if (mosquitto_connect(mosq, kMqttHost, kMqttPort, kMqttKeepaliveSecs) != MOSQ_ERR_SUCCESS) {
    printf("unable to connect to %s:%d\n", kMqttHost, kMqttPort);
    return 1;
  }

  mosquitto_loop_forever(mosq, -1, 1);

  mosquitto_destroy(mosq);
  mosquitto_lib_cleanup();
  return 0;
}
