#pragma once

#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include "utils/utils.h"

#ifdef __CROSS_COMPILE_ARM__
    #define LOG_PATH "/home/robotronik/LOG_CDFR/"
#else
    #define LOG_PATH "log/"
#endif

enum class Color {
    BLACK = 30,
    RED = 31,
    GREEN = 32,
    YELLOW = 33,
    BLUE = 34,
    MAGENTA = 35,
    CYAN = 36,
    WHITE = 37,
    GRAY = 90
};

enum class LogLevel {
    EXTENDED_DEBUG = 0,
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    GREENINFO
};

// Only messages with a level equal to or above CURRENT_LOG_LEVEL will be printed.
constexpr LogLevel CURRENT_LOG_LEVEL = LogLevel::DEBUG;

// Base logger write function (already formatted message).
void log_main(std::optional<Color> color, const std::string& message);

// Logger state helpers for API/UI usage.
int log_main_get_id();
std::string log_main_get_screen();

// Base case for message appending: when there are no more arguments.
inline void appendMessage(std::ostringstream& /*oss*/) {}

// Recursively append all parts of the message.
template<typename T, typename... Args>
inline void appendMessage(std::ostringstream& oss, const T& value, const Args&... args) {
    oss << value;
    appendMessage(oss, args...);
}

inline std::string getLevelString(LogLevel level) {
    switch (level) {
        case LogLevel::EXTENDED_DEBUG: return "EXT_DEBUG";
        case LogLevel::DEBUG:     return "DEBUG";
        case LogLevel::INFO:      return "INFO";
        case LogLevel::WARNING:   return "WARNING";
        case LogLevel::ERROR:     return "ERROR";
        case LogLevel::GREENINFO: return "GREEN";
        default:                  return "";
    }
}

template<typename... Args>
inline void log_main(std::optional<Color> color,
                     LogLevel level,
                     const std::string& file,
                     const int line,
                     const std::string& message,
                     const Args&... args) {
    if (level < CURRENT_LOG_LEVEL) {
        return;
    }

    std::ostringstream oss;
    appendMessage(oss, message, args...);

    std::ostringstream payload;
    payload << currentTimeFormatted() << " "
            << std::left << std::setw(10) << ("[" + getLevelString(level) + "]")
            << std::left << std::setw(30) << ("[" + file + ":" + std::to_string(line) + "]") << "  "
            << oss.str();

    log_main(color, payload.str());
}

// Convenience macros that automatically pass file and line.
#define LOG_EXTENDED_DEBUG(message, ...)   log_main(Color::GRAY   , LogLevel::EXTENDED_DEBUG    , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_DEBUG(message, ...)      log_main(Color::GRAY   , LogLevel::DEBUG    , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_INFO(message, ...)       log_main(std::nullopt  , LogLevel::INFO     , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_WARNING(message, ...)    log_main(Color::YELLOW , LogLevel::WARNING  , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_ERROR(message, ...)      log_main(Color::RED    , LogLevel::ERROR    , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_GREEN_INFO(message, ...) log_main(Color::GREEN  , LogLevel::GREENINFO, __FILE__, __LINE__, message, ##__VA_ARGS__)
