#include <chrono>
#include <cmath>
#include <csignal>
#include <stdexcept>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "defs/constante.h"
#include "defs/mainState.hpp"
#include "defs/structs.hpp"
#include "utils/httplib.h"
#include "utils/logger.hpp"

namespace {
bool viewHasPose(httplib::Client& http, main_State_t state, position_t expected, bool view = true) {
    const auto reply = view ? http.Post("/simulation/view") : http.Get("/get_global");
    if (!reply || reply->status != 200) return false;
    const auto data = json::parse(reply->body, nullptr, false);
    if (!data.is_object() || !data.value("simulation", false) || data.value("status", -1) != state) return false;
    const auto& path = data.at("navigation");
    if (state == RUN && path.size() > 1 &&
        (path.back().at("x") != expected.x || path.back().at("y") != expected.y)) {
        throw std::runtime_error("Simulation path has an unexpected destination");
    }
    const auto& pos = data.at("pos");
    const position_t actual = {pos.at("x").get<double>(), pos.at("y").get<double>(), pos.at("a").get<double>()};
    if (position_distance(actual, expected) < 20.0 &&
        std::abs(normalize_angle(actual.a - expected.a)) < 2.0 &&
        (state != WAITSTART || (data.at("elapsedMs") == 0 && data.at("score") == 0 &&
                                path.size() == 1))) {
        LOG_INFO("Simulation pose: x=", actual.x, " y=", actual.y, " a=", actual.a);
        return true;
    }
    return false;
}

bool waitForPose(httplib::Client& http, main_State_t state, position_t expected, bool view = true) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline) {
        if (viewHasPose(http, state, expected, view)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    LOG_ERROR("Simulation did not reach x=", expected.x, " y=", expected.y, " a=", expected.a);
    return false;
}
}

int main() {
    httplib::Client http("127.0.0.1", 8080);
    http.set_connection_timeout(0, 250000);
    http.set_read_timeout(0, 500000);
    if (http.Get("/")) {
        LOG_ERROR("Port 8080 is in use. Stop the running simulator before testing.");
        return 1;
    }

    const pid_t child = fork();
    if (child < 0) {
        LOG_ERROR("Could not launch programCDFR");
        return 1;
    }
    if (child == 0) {
        execl("./programCDFR", "programCDFR", "--simulate", nullptr);
        _exit(127);
    }

    bool passed = false;
    const auto post = [&http](const char* route) {
        const auto reply = http.Post(route);
        return reply && reply->status == 204;
    };
    position_t yellowStart = START_POSITION;
    position_robot_flip(yellowStart);
    const auto startWithSettings = [&http](const char* body, int expectedStatus) {
        const auto reply = http.Post("/simulation/start", body, "application/json");
        return reply && reply->status == expectedStatus;
    };
    try {
        if (waitForPose(http, WAITSTART, START_POSITION, false)) {
            const auto page = http.Get("/display");
            const auto simulationPage = http.Get("/simulation/");
            const auto navbar = http.Get("/navbar.html");
            const auto startWithoutViewer = http.Post("/simulation/start");
            passed = page && page->status == 200 && simulationPage && simulationPage->status == 200 &&
                     navbar && navbar->body.find("href=\"/simulation/\"") != std::string::npos &&
                     startWithoutViewer && startWithoutViewer->status == 409 &&
                     waitForPose(http, WAITSTART, START_POSITION) &&
                     post("/simulation/start") && waitForPose(http, RUN, {}) &&
                     post("/simulation/reset") && waitForPose(http, WAITSTART, START_POSITION) &&
                     post("/simulation/start") && waitForPose(http, RUN, {});
            if (passed) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1400));
                // The first frame must already be reset when the viewer returns.
                const bool inStandby = viewHasPose(http, WAITSTART, START_POSITION);
                if (!inStandby) LOG_ERROR("Simulation did not reset without a viewer");
                passed = inStandby &&
                         startWithSettings(R"({"color":2,"strategy":"unknown"})", 400) &&
                         startWithSettings(R"({"color":0,"strategy":"ExempleStrat"})", 400) &&
                         startWithSettings(R"({"color":2,"strategy":"ExempleStrat"})", 204) &&
                         waitForPose(http, RUN, {}) &&
                         post("/simulation/reset") && waitForPose(http, WAITSTART, yellowStart);
            }
        }
    } catch (const std::exception& error) {
        LOG_ERROR("Simulation telemetry: ", error.what());
    }

    kill(child, SIGTERM);
    int status;
    return waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0 && passed ? 0 : 1;
}
