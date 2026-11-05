#include "restAPI/matAPI.hpp"

#include "utils/httplib.h"
#include "utils/logger.hpp"

#include <mutex>

const std::string mat_api::MAT_OBJECTS_PATH = "/objects";

namespace {

// Délais courts : la boucle principale ne doit pas bloquer si le mat est absent.
constexpr int CONNECT_TIMEOUT_US = 50 * 1000;   // 50 ms
constexpr int READ_TIMEOUT_US = 150 * 1000;     // 150 ms

const std::string MAT_URL = std::string(MAT_HOST) + ":" + std::to_string(MAT_PORT);

// Connexion HTTP keep-alive unique, réutilisée par toutes les requêtes.
httplib::Client& client() {
    static httplib::Client cli(MAT_URL);
    static const bool configured = []() {
        cli.set_connection_timeout(0, CONNECT_TIMEOUT_US);
        cli.set_read_timeout(0, READ_TIMEOUT_US);
        cli.set_write_timeout(0, READ_TIMEOUT_US);
        return true;
    }();
    (void)configured;
    return cli;
}

std::mutex g_mutex;
bool g_started = false;

bool get(const std::string& path, json& response) {
    auto res = client().Get(path.c_str());

    if (!res) {
        LOG_ERROR("Failed to fetch response from ", MAT_URL, path);
        return false;
    }
    if (res->status != 200) {
        LOG_ERROR("HTTP error: ", res->status);
        return false;
    }
    try {
        response = json::parse(res->body);
        return true;
    } catch (const json::parse_error& e) {
        LOG_ERROR("JSON parse error: ", e.what());
        return false;
    }
}

}  // namespace

bool mat_api::ready() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_started;
}

bool mat_api::start(bool& connectionOk) {
    std::lock_guard<std::mutex> lock(g_mutex);

    if (g_started) {
        connectionOk = true;
        return true;
    }

    json response;
    if (!get("/start", response) || !response.is_object()) {
        return false;
    }
    g_started = true;
    connectionOk = true;
    return true;
}

void mat_api::stop() {
    std::lock_guard<std::mutex> lock(g_mutex);

    json response;
    if (get("/stop", response) && response.is_object()) {
        LOG_INFO("MAT stopped successfully");
    } else {
        LOG_ERROR("Failed to stop MAT");
    }
    g_started = false;
}

bool mat_api::fetchTable(MatTableData& data) {
    std::lock_guard<std::mutex> lock(g_mutex);

    json response;
    if (!get(MAT_OBJECTS_PATH, response) || !response.is_object()) {
        return false;
    }
    data = parseMatTableData(response);
    return true;
}