#include "logger.h"

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <mutex>

namespace {
constexpr char const* kColorReset = "\033[0m";
constexpr char const* kColorFatal = "\033[1;31m";
constexpr char const* kColorError = "\033[1;31m";
constexpr char const* kColorWarn = "\033[1;33m";
constexpr char const* kColorInfo = kColorReset;
constexpr char const* kColorDebug = kColorReset;

constexpr char const* levelToName(LogLevel level)
{
  switch (level) {
    case LogLevel::Fatal:
      return "FATAL";
    case LogLevel::Error:
      return "ERROR";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Debug:
      return "DEBUG";
  }

  return "UNKNOWN";
}

constexpr char const* levelToColor(LogLevel level)
{
  switch (level) {
    case LogLevel::Fatal:
      return kColorFatal;
    case LogLevel::Error:
      return kColorError;
    case LogLevel::Warn:
      return kColorWarn;
    case LogLevel::Info:
      return kColorInfo;
    case LogLevel::Debug:
      return kColorDebug;
  }

  return kColorReset;
}

constexpr int levelRank(LogLevel level)
{
  switch (level) {
    case LogLevel::Fatal:
      return 0;
    case LogLevel::Error:
      return 1;
    case LogLevel::Warn:
      return 2;
    case LogLevel::Info:
      return 3;
    case LogLevel::Debug:
      return 4;
  }

  return 4;
}

bool equals_ignore_case(const char* a, const char* b)
{
  if (!a || !b) {
    return false;
  }

  while (*a != '\0' && *b != '\0') {
    char ca = *a;
    char cb = *b;
    if (ca >= 'a' && ca <= 'z') {
      ca = static_cast<char>(ca - ('a' - 'A'));
    }
    if (cb >= 'a' && cb <= 'z') {
      cb = static_cast<char>(cb - ('a' - 'A'));
    }
    if (ca != cb) {
      return false;
    }
    ++a;
    ++b;
  }

  return *a == '\0' && *b == '\0';
}

bool tryParseLogLevel(const char* value, LogLevel& level)
{
  if (!value || *value == '\0') {
    return false;
  }

  if (equals_ignore_case(value, "FATAL")) {
    level = LogLevel::Fatal;
    return true;
  }
  if (equals_ignore_case(value, "ERROR")) {
    level = LogLevel::Error;
    return true;
  }
  if (equals_ignore_case(value, "WARN") || equals_ignore_case(value, "WARNING")) {
    level = LogLevel::Warn;
    return true;
  }
  if (equals_ignore_case(value, "INFO")) {
    level = LogLevel::Info;
    return true;
  }
  if (equals_ignore_case(value, "DEBUG")) {
    level = LogLevel::Debug;
    return true;
  }

  return false;
}

std::mutex g_log_mutex;
}  // namespace

Logger& Logger::instance()
{
  static Logger logger;
  return logger;
}

Logger::Logger()
{
  configureFromEnvironment();
}

void Logger::set_min_level(LogLevel level)
{
  min_level_ = level;
}

void Logger::log(LogLevel level, const char* format, ...)
{
  std::va_list args;
  va_start(args, format);
  vLog(level, format, args);
  va_end(args);
}

void Logger::fatal(const char* format, ...)
{
  std::va_list args;
  va_start(args, format);
  vLog(LogLevel::Fatal, format, args);
  va_end(args);
}

void Logger::error(const char* format, ...)
{
  std::va_list args;
  va_start(args, format);
  vLog(LogLevel::Error, format, args);
  va_end(args);
}

void Logger::warn(const char* format, ...)
{
  std::va_list args;
  va_start(args, format);
  vLog(LogLevel::Warn, format, args);
  va_end(args);
}

void Logger::info(const char* format, ...)
{
  std::va_list args;
  va_start(args, format);
  vLog(LogLevel::Info, format, args);
  va_end(args);
}

void Logger::debug(const char* format, ...)
{
  std::va_list args;
  va_start(args, format);
  vLog(LogLevel::Debug, format, args);
  va_end(args);
}

void Logger::configureFromEnvironment()
{
  char const* env = std::getenv("AUTOBUS_LOG_LEVEL");
  LogLevel parsed = LogLevel::Debug;
  if (tryParseLogLevel(env, parsed)) {
    min_level_ = parsed;
  }
}

void Logger::vLog(LogLevel level, const char* format, std::va_list args)
{
  if (levelRank(level) > levelRank(min_level_)) {
    return;
  }

  std::lock_guard<std::mutex> lock(g_log_mutex);
  std::printf("%s[%s]%s ", levelToColor(level), levelToName(level), kColorReset);
  std::vprintf(format, args);
  std::printf("\n");
  std::fflush(stdout);
}