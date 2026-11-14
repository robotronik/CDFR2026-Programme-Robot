#include "restAPI/matAPI.hpp"

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
constexpr auto STALE_PERIOD = std::chrono::milliseconds(MAT_STALE_MS);

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

// État partagé entre le thread de fond et la boucle principale (protégé par g_mutex).
std::mutex g_mutex;
std::condition_variable g_wake;   // réveille le thread de fond au stop()
std::thread g_thread;
bool g_runRequested = false;
MatTableData g_data;              // dernière table reçue
bool g_hasData = false;
std::chrono::steady_clock::time_point g_stamp;

// g_mutex doit être verrouillé.
bool isFresh() {
    return g_hasData && std::chrono::steady_clock::now() - g_stamp <= STALE_PERIOD;
}

// Interroge le mat sans fin. Seuls le retour, la perte de la connexion et
// l'échec du tout premier essai sont loggés (jamais les tentatives suivantes).
void workerLoop() {
    httplib::Client cli(MAT_URL);
    configure(cli);

    bool connected = false;
    bool firstAttemptDone = false;
    while (true) {
        json response;
        const HttpGet objects = get(cli, mat_api::MAT_OBJECTS_PATH, response);

        std::unique_lock<std::mutex> lock(g_mutex);
        if (g_runRequested == false) {
            break;
        }
        if (objects.ok) {
            g_data = parseMatTableData(response);
            g_hasData = true;
            g_stamp = std::chrono::steady_clock::now();
        }
        lock.unlock();

        if (objects.ok && !connected) {
            LOG_GREEN_INFO("MAT connected");
            connected = true;
        } else if (!objects.ok && connected) {
            LOG_ERROR("MAT lost: ", objects.error);
            connected = false;
        } else if (!objects.ok && !connected && !firstAttemptDone) {
            LOG_ERROR("MAT not reachable at startup (", MAT_URL, "), retrying in background: ",
                      objects.error);
            firstAttemptDone = true;
        }

        lock.lock();
        g_wake.wait_for(lock, POLL_PERIOD, [] { return !g_runRequested; });
    }

    // Le client HTTP n'appartient qu'à ce thread : /stop est envoyé ici.
    if (connected) {
        json response;
        const HttpGet stop = get(cli, "/stop", response);
        if (stop.ok && response.is_object()) {
            LOG_INFO("MAT stopped successfully");
        } else {
            LOG_ERROR("Failed to stop MAT: ", stop.ok ? "unexpected response" : stop.error);
        }
    }
}

}  // namespace

bool mat_api::start() {
    std::lock_guard<std::mutex> lock(g_mutex);

    if (!g_thread.joinable()) {
        g_runRequested = true;
        g_thread = std::thread(workerLoop);
    }
    return isFresh();
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

bool mat_api::ready() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return isFresh();
}

bool mat_api::latest(MatTableData& data) {
    std::lock_guard<std::mutex> lock(g_mutex);

    if (!isFresh()) {
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
    data = parseMatTableData(response);
    return true;
}
