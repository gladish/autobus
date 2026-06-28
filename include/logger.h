#pragma once

#include <cstdarg>

enum class LogLevel
{
  Fatal,
  Error,
  Warn,
  Info,
  Debug,
};

class Logger
{
public:
  static Logger& instance();

  void set_min_level(LogLevel level);

  [[gnu::format(printf, 3, 4)]]
  void log(LogLevel level, const char* format, ...);

  [[gnu::format(printf, 2, 3)]]
  void fatal(const char* format, ...);

  [[gnu::format(printf, 2, 3)]]
  void error(const char* format, ...);

  [[gnu::format(printf, 2, 3)]]
  void warn(const char* format, ...);

  [[gnu::format(printf, 2, 3)]]
  void info(const char* format, ...);

  [[gnu::format(printf, 2, 3)]]
  void debug(const char* format, ...);

private:
  Logger();

  void configureFromEnvironment();
  void vLog(LogLevel level, const char* format, std::va_list args);

  LogLevel min_level_ = LogLevel::Debug;
};

#define ALOG_FATAL(FORMAT, ...) Logger::instance().fatal(FORMAT, ##__VA_ARGS__)
#define ALOG_ERROR(FORMAT, ...) Logger::instance().error(FORMAT, ##__VA_ARGS__)
#define ALOG_WARN(FORMAT, ...) Logger::instance().warn(FORMAT, ##__VA_ARGS__)
#define ALOG_INFO(FORMAT, ...) Logger::instance().info(FORMAT, ##__VA_ARGS__)
#define ALOG_DEBUG(FORMAT, ...) Logger::instance().debug(FORMAT, ##__VA_ARGS__)