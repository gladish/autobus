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
  ServoConfig servo;
  EscConfig esc;
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
  j.at("servo").get_to(c.servo);
  j.at("esc").get_to(c.esc);
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