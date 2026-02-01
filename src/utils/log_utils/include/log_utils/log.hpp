#pragma once

#include <string>

#include <log_utils/spdlog/sinks/stdout_color_sinks.h>
#include <log_utils/spdlog/spdlog.h>

// =============================================================================
// log_utils public API
// =============================================================================
namespace log_utils {

// Log level used by log_utils.
enum class Level { trace, debug, info, warn, err, critical, off };

// Create or reuse a global default logger with colored console output.
inline void init_console_logger(const std::string &name = "core") {
  auto existing = spdlog::get(name);
  if (existing) {
    spdlog::set_default_logger(existing);
    return;
  }

  auto logger = spdlog::stdout_color_mt(name);
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  logger->set_level(spdlog::level::info);
  spdlog::set_default_logger(logger);
}

// Set the level of the default logger.
inline void set_level(Level lv) {
  auto logger = spdlog::default_logger();
  if (!logger) {
    init_console_logger();
  }
  switch (lv) {
    case Level::trace:
      spdlog::set_level(spdlog::level::trace);
      break;
    case Level::debug:
      spdlog::set_level(spdlog::level::debug);
      break;
    case Level::info:
      spdlog::set_level(spdlog::level::info);
      break;
    case Level::warn:
      spdlog::set_level(spdlog::level::warn);
      break;
    case Level::err:
      spdlog::set_level(spdlog::level::err);
      break;
    case Level::critical:
      spdlog::set_level(spdlog::level::critical);
      break;
    case Level::off:
      spdlog::set_level(spdlog::level::off);
      break;
  }
}

// Get the global default logger (auto-init if needed).
inline spdlog::logger &get() {
  auto logger = spdlog::default_logger();
  if (!logger) {
    init_console_logger();
    logger = spdlog::default_logger();
  }
  return *logger;
}

}  // namespace log_utils

// =============================================================================
// Convenience macros
// =============================================================================
#define LOGT(...) log_utils::get().trace(__VA_ARGS__)
#define LOGD(...) log_utils::get().debug(__VA_ARGS__)
#define LOGI(...) log_utils::get().info(__VA_ARGS__)
#define LOGW(...) log_utils::get().warn(__VA_ARGS__)
#define LOGE(...) log_utils::get().error(__VA_ARGS__)
#define LOGC(...) log_utils::get().critical(__VA_ARGS__)
