#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <exception>
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <filesystem>
#include <sys/stat.h>
#include "utils/utils.h"

#include <deque>
#include <vector>
#include <optional>
#include <sstream>
#include <string>
#include <initializer_list>

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

enum class Background {
    BLACK = 40,
    RED = 41,
    GREEN = 42,
    YELLOW = 43,
    BLUE = 44,
    MAGENTA = 45,
    CYAN = 46,
    WHITE = 47
};

enum class Style {
    RESET = 0,
    BOLD = 1,
    FAINT = 2,
    ITALIC = 3,
    UNDERLINE = 4,
    BLINK = 5,
    REVERSE = 7
};


struct AnsiStyle {
    std::optional<Color> color = std::nullopt;
    std::optional<Background> background = std::nullopt;
    std::vector<Style> styles = {};

    AnsiStyle() = default;

    AnsiStyle(std::optional<Color> c,
              std::optional<Background> bg = std::nullopt,
              std::initializer_list<Style> s = {})
        : color(c), background(bg), styles(s) {}
};

class logger
{
private:
    bool stdOutInitValid;
    static int globalLogNum;
    bool logEnable = true;
    std::string baseName;
    int fileDescriptor = -1;
    std::string timeString;
    static constexpr size_t MAX_LINES = 40;
    std::deque<std::string> lines;

    static std::string getExecutablePath();
    static std::string resolveLogPath(const std::string& path);
    static void        ensureLogDirectoryAndNumFile(const std::string& logDir, const std::string& numLogFile);
    static int         readAndIncrementLogNum(const std::string& fullPath);

    static std::string makeAnsiCode(
        std::optional<Color> color = std::nullopt,
        std::optional<Background> bg = std::nullopt,
        const std::vector<Style>& styles = {}
    );

    int getLogNumber();
    int getLogFileDescriptor();
    void addLine(const std::string& line);
    std::string init(const bool withColor = false);

    void logToFile(const std::string& message, bool forceSync);
    void logToOutput(const std::string& message);
public:
    logger(const std::string name, bool defaultlogStatus = true);
    ~logger();

    void log(bool logFile,
         bool forceSync,
         bool logOut,
         const AnsiStyle& style,
         const std::string& message);

    int getLogID();
    std::string getLogToRobotScreen();
    void setLogStatus(bool logS);
};


namespace SimpleLogger {


// Base case for message appending: when there are no more arguments.
inline void appendMessage(std::ostringstream& /*oss*/) {}

// Recursively append all parts of the message.
template<typename T, typename... Args>
inline void appendMessage(std::ostringstream& oss, const T& value, const Args&... args) {
    oss << value;
    appendMessage(oss, args...);
}

template<typename... Args>
inline void log(logger* loggerType, bool logFile, bool forceSync, bool logOut, const AnsiStyle& style,const std::string& typeLevel, const std::string& file, const int line, const std::string& message, const Args&... args) {

    std::ostringstream oss;
    appendMessage(oss, message, args...);

    std::ostringstream playLoad;
            playLoad << currentTimeFormatted() << " "
            << std::left << std::setw(10) << ("[" + typeLevel + "]") // Fixed width for log level
            << std::left << std::setw(30) << ("[" + file + ":" + std::to_string(line) + "]") << "  " // Fixed width for function name
            << oss.str();

    loggerType->log(logFile,forceSync,logOut,style,playLoad.str());
}

} // namespace SimpleLogger

//logger instance
logger* log_main();
logger* log_asserv();

// Convenience macros that automatically pass the calling function's name.
#define LOG_DEBUG(message, ...)      SimpleLogger::log((logger*)log_main(),true,false,true,AnsiStyle(Color::GRAY)  ,"DEBUG"  , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_INFO(message, ...)       SimpleLogger::log((logger*)log_main(),true,false,true,{}                      ,"INFO"   , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_WARNING(message, ...)    SimpleLogger::log((logger*)log_main(),true,false,true,AnsiStyle(Color::YELLOW),"WARNING", __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_ERROR(message, ...)      SimpleLogger::log((logger*)log_main(),true,false,true,AnsiStyle(Color::RED)   ,"ERROR"  , __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_GREEN_INFO(message, ...) SimpleLogger::log((logger*)log_main(),true,false,true,AnsiStyle(Color::GREEN) ,"GREEN"  , __FILE__, __LINE__, message, ##__VA_ARGS__)

#define LOG_ASSERV_SET_INFO(message, ...) SimpleLogger::log((logger*)log_asserv(),true,true,false,AnsiStyle(Color::GREEN),"I2C", __FILE__, __LINE__, message, ##__VA_ARGS__)
#define LOG_ASSERV_GET_INFO(message, ...) SimpleLogger::log((logger*)log_asserv(),true,false,false,AnsiStyle(Color::GREEN),"I2C", __FILE__, __LINE__, message, ##__VA_ARGS__)
