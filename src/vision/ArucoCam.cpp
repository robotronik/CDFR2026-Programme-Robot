#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include "utils/httplib.h"
#include "vision/ArucoCam.hpp"
#include "utils/logger.hpp"
#include "utils/json.hpp" // For handling JSON

using json = nlohmann::json;

#define PORT_OFFSET 5000
#define SCAN_FAIL_FRAMES_NUM 10
#define SCAN_DONE_FRAMES_NUM 30

pid_t startPythonProgram(char** args);
void stopPythonProgram(pid_t pid);
bool restAPI_GET(const std::string &url, const std::string &resquest, json &response);

ArucoCam::ArucoCam(int cam_number, const char* calibration_file_path) {
    id = cam_number;
    if (id < 0) {
        pid=-1;
        status = true;
        LOG_INFO("Emulating ArucoCam");
        return;
    }
    // Child process: Split the args string into individual arguments
    char *args[] = {
        (char *)"python3",
        (char *)"detect_aruco.py",
        (char *)calibration_file_path,
        (char *)"--api-port",
        (char *)std::to_string(PORT_OFFSET + id).c_str(),
        (char *)"--cam",
        (char *)std::to_string(id).c_str(),
        (char *)"--headless",
        (char *)"True",
        nullptr
    };

    pid = startPythonProgram(args);
    if (pid == -1) {
        LOG_ERROR("Failed to start ArucoCam ", id);
    }
    else
    {
        LOG_GREEN_INFO("ArucoCam ", id, " started with PID ", pid);
        url = "http://localhost:" + std::to_string(PORT_OFFSET + id);
        status = false;
    }
}
ArucoCam::~ArucoCam(){
    if (pid > 0)
        stopPythonProgram(pid);
}
bool ArucoCam::getPos(int & x, int & y, int & a) {
    if (status == false) {
        LOG_ERROR("ArucoCam ", id, " is not running, will start it now");
        start();
        return false;
    }

    // LOG_DEBUG("Fetching position from ArucoCam ", id);
    // Calls /position rest api endpoint of the ArucoCam API
    // Returns true if the call was successful, false otherwise
    if (id < 0) {
        // TODO change this to return a random position
        return false;
    }
    json response;
    if (restAPI_GET(url, "/position", response) == false) {
        LOG_ERROR("ArucoCam::getPos() - Failed to fetch position");
        return false;
    }
    int failedFrames = response.value("failedFrames", -1);
    int sucessFrames = response.value("sucessFrames", -1);
    // int totalFrames = response.value("totalFrames", -1);
    if (failedFrames > SCAN_FAIL_FRAMES_NUM) {
        LOG_WARNING("Cam has too many failed frames : ", failedFrames);
        return false;
    }
    if (sucessFrames < SCAN_DONE_FRAMES_NUM) {
        LOG_WARNING("Cam has not enough good success frames : ", sucessFrames);
        return false;
    }
    // Extract the values from the JSON object
    json position = response["position"];
    x = position.value("x", 0);
    y = position.value("y", 0);
    a = position.value("a", 0);
    LOG_GREEN_INFO("ArucoCam ", id, " position: { x = ", x, ", y = ", y, ", a = ", a, " }");
    // Return true if the values were successfully extracted
    // TODO Maybe call stop() so the cam is not running anymore
    return true;
}

void ArucoCam::start() {
    json response;
    if (restAPI_GET(url, "/start", response)){
        status = true;
        LOG_GREEN_INFO("ArucoCam ", id, " started");
    }
}

void ArucoCam::stop() {
    json response;
    if (restAPI_GET(url, "/stop", response)){
        status = false;
        LOG_GREEN_INFO("ArucoCam ", id, " stopped");
    }
}

bool restAPI_GET(const std::string &url, const std::string &resquest, json &response) {
    // HTTP
    httplib::Client cli(url);
    auto res = cli.Get(resquest.c_str());
    // Check for nullptr
    if (!res) {
        LOG_ERROR("Failed to fetch response from ", url, resquest);
        return false;
    }
    LOG_GREEN_INFO("HTML Status is ", res->status);
    // LOG_GREEN_INFO("HTML Body is ", res->body);

    // Check if the response code is 200 (OK)
    if (res->status != 200) {
        LOG_ERROR("HTTP error: ", res->status);
        return false;
    }
    try {
        // Parse JSON response
        response = json::parse(res->body);
        LOG_GREEN_INFO("API Response: ", response.dump(4));
        return true;
    } catch (const json::parse_error& e) {
        LOG_ERROR("JSON parse error: ", e.what());
        return false;
    }
}

pid_t startPythonProgram(char ** args) {
    pid_t pid = fork();

    if (pid == -1) {
        LOG_ERROR("startPythonProgram - Failed to fork process");
        return -1;
    } else if (pid == 0) {
        // Execute the Python program with arguments
        execvp(args[0], args);
        LOG_ERROR("startPythonProgram - Failed to execute Python script");
        _exit(1); // Ensure child process exits
    }

    // Parent process: Return child PID
    return pid;
}

void stopPythonProgram(pid_t pid) {
    if (pid <= 0) {
        LOG_ERROR("stopPythonProgram - Invalid PID");
        return;
    }

    if (kill(pid, SIGTERM) == -1) {
        LOG_ERROR("stopPythonProgram - Failed to terminate process with PID ", pid);
    } else {
        LOG_INFO("stopPythonProgram - Process with PID ", pid, " terminated");
    }
}

size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
    size_t totalSize = size * nmemb;
    if (response) {
        response->append(static_cast<char*>(contents), totalSize);
    }
    return totalSize;
}