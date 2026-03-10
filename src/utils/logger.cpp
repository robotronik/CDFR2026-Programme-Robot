#include "utils/logger.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <deque>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace {

struct LoggerState {
    bool stdOutInitValid = false;
    std::string baseName = "mainLog";
    int fileDescriptor = -1;
    std::string timeString = currentTimeFormatted();
    std::deque<std::string> lines;
};

// Maximum number of recent log lines kept in the in-memory deque for screen logging.
constexpr size_t MAX_LINES = 200;
// Cached log file number, persisted across calls to avoid re-reading/updating the counter.
int globalLogNum = -1;

LoggerState& state() {
    static LoggerState s;
    return s;
}

std::string initBanner(const std::string& timeString, bool withColor) {
    std::ostringstream initMessage;
    if (withColor) {
        initMessage << "\033[1;31m";
    }

    initMessage << "  _____   ____  ____   ____ _______ _____   ____  _   _ _____ _  __" << std::endl;
    initMessage << " |  __ \\ / __ \\|  _ \\ / __ \\__   __|  __ \\ / __ \\| \\ | |_   _| |/ /" << std::endl;
    initMessage << " | |__) | |  | | |_) | |  | | | |  | |__) | |  | |  \\| | | | | ' / " << std::endl;
    initMessage << " |  _  /| |  | |  _ <| |  | | | |  |  _  /| |  | | .   | | | |  <  " << std::endl;
    initMessage << " | | \\ \\| |__| | |_) | |__| | | |  | | \\ \\| |__| | |\\  |_| |_| . \\ " << std::endl;
    initMessage << " |_|  \\_\\\\____/|____/ \\____/  |_|  |_|  \\_\\\\____/|_| \\_|_____|_|\\_\\" << std::endl;

    if (withColor) {
        initMessage << "\n\033[0m";
    }

    initMessage << "ROBOTRONIK" << std::endl;
    initMessage << "PROGRAM ROBOT CDFR" << std::endl;
    initMessage << "Start Time : " << timeString << std::endl;
    return initMessage.str();
}

std::string getExecutablePath() {
    char result[1000];
    ssize_t count = readlink("/proc/self/exe", result, sizeof(result));
    if (count == -1) {
        throw std::runtime_error("Unable to determine executable path");
    }
    return std::filesystem::path(std::string(result, count)).parent_path().string();
}

std::string resolveLogPath(const std::string& path) {
    std::filesystem::path p(path);
    if (p.is_absolute()) {
        return path;
    }
    return (std::filesystem::path(getExecutablePath()) / p).string();
}

void ensureLogDirectoryAndNumFile(const std::string& logDir, const std::string& numLogFile) {
    struct stat st;
    if (stat(logDir.c_str(), &st) != 0) {
        if (mkdir(logDir.c_str(), 0755) != 0) {
            perror("Failed to create log directory");
            exit(EXIT_FAILURE);
        }
        std::ofstream out(numLogFile);
        if (!out) {
            std::cerr << "Erreur : impossible de creer le fichier " << numLogFile << std::endl;
            exit(EXIT_FAILURE);
        }
        out << 0;
        out.close();
    } else {
        std::ifstream in(numLogFile);
        if (!in) {
            std::cerr << "Erreur : le dossier '" << logDir << "' existe mais pas le fichier 'numLog'. Abandon." << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}

int readAndIncrementLogNum(const std::string& fullPath) {
    std::string numFilePath = fullPath + "/numLog";
    ensureLogDirectoryAndNumFile(fullPath, numFilePath);

    std::ifstream in(numFilePath);
    int num = 0;
    if (in >> num) {
        in.close();
    }

    std::ofstream out(numFilePath, std::ios::trunc);
    out << (num + 1);
    return num;
}

int getLogNumber() {
    if (globalLogNum == -1) {
        globalLogNum = readAndIncrementLogNum(resolveLogPath(LOG_PATH));
    }
    return globalLogNum;
}

std::string makeAnsiCode(std::optional<Color> color) {
    if (!color.has_value()) {
        return "\033[0m";
    }
    return "\033[" + std::to_string(static_cast<int>(*color)) + "m";
}

void addLine(const std::string& line) {
    auto& s = state();
    s.lines.push_back(line);
    if (s.lines.size() > MAX_LINES) {
        s.lines.pop_front();
    }
}

int getLogFileDescriptor() {
    auto& s = state();
    if (s.fileDescriptor != -1) {
        return s.fileDescriptor;
    }

    std::ostringstream oss;
    oss << resolveLogPath(LOG_PATH) << s.baseName << "_" << getLogNumber() << ".log";
    std::string fullName = oss.str();

    s.fileDescriptor = open(fullName.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (s.fileDescriptor == -1) {
        perror(("Failed to open log file: " + fullName).c_str());
        return -1;
    }

    std::string initString = initBanner(s.timeString, false);
    write(s.fileDescriptor, initString.c_str(), initString.size());

    return s.fileDescriptor;
}

void logToFile(const std::string& message) {
    int fd = getLogFileDescriptor();
    if (fd == -1) {
        return;
    }

    std::string fullMessage = message + "\n";
    write(fd, fullMessage.c_str(), fullMessage.size());

}

void logToOutput(const std::string& message) {
    auto& s = state();
    if (!s.stdOutInitValid) {
        s.stdOutInitValid = true;
        logToOutput(initBanner(s.timeString, true));
    }

    std::stringstream ss(message);
    std::string line;
    while (std::getline(ss, line)) {
        addLine(line);
    }

    std::cout << message;
    std::cout.flush();
}

} // namespace

void log_main(std::optional<Color> color, const std::string& message) {
    std::ostringstream formatted;
    if (color.has_value()) {
        formatted << makeAnsiCode(color) << message << "\033[0m" << std::endl;
    } else {
        formatted << message << std::endl;
    }
    logToOutput(formatted.str());
    logToFile(message);
}

int log_main_get_id() {
    return getLogNumber();
}

std::string log_main_get_screen() {
    std::ostringstream result;
    for (const auto& line : state().lines) {
        result << line << '\n';
    }
    return result.str();
}
