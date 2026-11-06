#include "restAPI/matAPI.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "utils/httplib.h"
#include "utils/logger.hpp"

const std::string mat_api::MAT_OBJECTS_PATH = "/objects";

namespace {

// Délais courts : le thread de fond ne doit pas rester bloqué si le mat est absent.
constexpr int CONNECT_TIMEOUT_US = 50 * 1000;   // 50 ms
constexpr int READ_TIMEOUT_US = 150 * 1000;     // 150 ms

const std::string MAT_URL = std::string(MAT_HOST) + ":" + std::to_string(MAT_PORT);
constexpr auto POLL_PERIOD = std::chrono::milliseconds(MAT_POLL_PERIOD_MS);

void configure(httplib::Client& cli) {
    cli.set_connection_timeout(0, CONNECT_TIMEOUT_US);
    cli.set_read_timeout(0, READ_TIMEOUT_US);
    cli.set_write_timeout(0, READ_TIMEOUT_US);
}

struct HttpGet {
    bool ok = false;
    std::string error;
};

HttpGet get(httplib::Client& cli, const std::string& path, json& response) {
    auto res = cli.Get(path.c_str());

    if (!res) {
        return {false, "no response"};
    }
    if (res->status != 200) {
        return {false, "HTTP error " + std::to_string(res->status)};
    }
    try {
        response = json::parse(res->body);
    } catch (const json::parse_error& e) {
        return {false, std::string("JSON parse error: ") + e.what()};
    }
    return {true, {}};
}

// État partagé entre le thread de fond et la boucle principale.
std::mutex g_mutex;
std::condition_variable g_wake;
std::thread g_thread;
bool g_runRequested = false;      // le thread de fond doit continuer
bool g_detectionStarted = false;  // GET /start a répondu
MatTableData g_data;              // dernière table reçue
bool g_hasData = false;
std::chrono::steady_clock::time_point g_stamp;

// Boucle de rafraîchissement : un seul client HTTP, utilisé uniquement ici.
void workerLoop() {
    httplib::Client cli(MAT_URL);
    configure(cli);

    // Le mat peut rester absent longtemps : une ligne de log par seconde au plus.
    auto lastErrorLog = std::chrono::steady_clock::time_point{};
    const auto reportFailure = [&lastErrorLog](const std::string& path, const std::string& error) {
        const auto now = std::chrono::steady_clock::now();
        if (now - lastErrorLog < std::chrono::seconds(1)) {
            return;
        }
        lastErrorLog = now;
        LOG_ERROR("MAT ", MAT_URL, path, ": ", error);
    };

    bool detectionStarted = false;
    while (true) {
        json response;
        if (!detectionStarted) {
            const HttpGet start = get(cli, "/start", response);
            if (!start.ok) {
                reportFailure("/start", start.error);
            } else if (!response.is_object()) {
                reportFailure("/start", "unexpected response");
            } else {
                detectionStarted = true;
                std::lock_guard<std::mutex> lock(g_mutex);
                g_detectionStarted = true;
            }
        } else {
            const HttpGet objects = get(cli, mat_api::MAT_OBJECTS_PATH, response);
            if (objects.ok) {
                const MatTableData data = parseMatTableData(response);
                std::lock_guard<std::mutex> lock(g_mutex);
                g_data = data;
                g_hasData = true;
                g_stamp = std::chrono::steady_clock::now();
            } else {
                reportFailure(mat_api::MAT_OBJECTS_PATH, objects.error);
            }
        }

        std::unique_lock<std::mutex> lock(g_mutex);
        if (g_wake.wait_for(lock, POLL_PERIOD, [] { return !g_runRequested; })) {
            break;
        }
    }

    // Le client HTTP n'appartient qu'à ce thread : /stop est envoyé ici.
    if (detectionStarted) {
        json response;
        const HttpGet stop = get(cli, "/stop", response);
        if (stop.ok && response.is_object()) {
            LOG_INFO("MAT stopped successfully");
        } else {
            LOG_ERROR("Failed to stop MAT: ", stop.ok ? "unexpected response" : stop.error);
        }
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    g_detectionStarted = false;
}

}  // namespace

bool mat_api::ready() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_detectionStarted;
}

bool mat_api::start(bool& connectionOk) {
    std::lock_guard<std::mutex> lock(g_mutex);

    if (g_detectionStarted) {
        connectionOk = true;
        return true;
    }

    if (!g_thread.joinable()) {
        g_runRequested = true;
        g_thread = std::thread(workerLoop);
    }

    connectionOk = false;
    return false;
}

void mat_api::stop() {
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_runRequested = false;
    }
    g_wake.notify_all();

    if (g_thread.joinable()) {
        g_thread.join();
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    g_hasData = false;
}

bool mat_api::latest(MatTableData& data, unsigned long& ageMs) {
    std::lock_guard<std::mutex> lock(g_mutex);

    if (!g_hasData) {
        ageMs = 0;
        return false;
    }

    const auto age = std::chrono::steady_clock::now() - g_stamp;
    ageMs = static_cast<unsigned long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(age).count());

    if (ageMs > MAT_STALE_MS) {
        return false;
    }

    data = g_data;
    return true;
}

bool mat_api::fetchTable(MatTableData& data) {
    httplib::Client cli(MAT_URL);
    configure(cli);

    json response;
    const HttpGet result = get(cli, mat_api::MAT_OBJECTS_PATH, response);
    if (!result.ok) {
        LOG_ERROR("MAT ", MAT_URL, mat_api::MAT_OBJECTS_PATH, ": ", result.error);
        return false;
    }
    if (!response.is_object()) {
        LOG_ERROR("Malformed response from MAT ", mat_api::MAT_OBJECTS_PATH);
        return false;
    }
    data = parseMatTableData(response);
    return true;
}
