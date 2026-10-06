#include "mat/mat.hpp"
#include "utils/logger.hpp"
#include "utils/httplib.h"

#include <mutex>

// API REST du mat de vision (HTTP/TCP), port 5000 par défaut.
const std::string MAT_URL = std::string(MAT_HOST) + ":" + std::to_string(MAT_PORT);

namespace {

// Le mat répond en quelques millisecondes sur le réseau local ; les délais
// restent courts pour ne pas bloquer la boucle principale si le mat est absent.
constexpr int CONNECT_TIMEOUT_US = 50 * 1000;   // 50 ms
constexpr int READ_TIMEOUT_US = 150 * 1000;     // 150 ms

std::mutex g_matMutex;
MatTableData g_matTableData;

double jsonNumber(const json& object, const char* key) {
    const auto it = object.find(key);
    return (it != object.end() && it->is_number()) ? it->get<double>() : 0.0;
}

std::string jsonString(const json& object, const char* key) {
    const auto it = object.find(key);
    return (it != object.end() && it->is_string()) ? it->get<std::string>() : std::string();
}

[[maybe_unused]] void setMatTableData(const MatTableData& data) {
    std::lock_guard<std::mutex> lock(g_matMutex);
    g_matTableData = data;
}

}  // namespace

MatTableData getMatTableData() {
    std::lock_guard<std::mutex> lock(g_matMutex);
    return g_matTableData;
}

// Requête GET qui renvoie le corps JSON, ou false si le mat est injoignable.
bool restAPI_GET_(const std::string& url, const std::string& request, json& response) {
    httplib::Client cli(url);
    cli.set_connection_timeout(0, CONNECT_TIMEOUT_US);
    cli.set_read_timeout(0, READ_TIMEOUT_US);
    cli.set_write_timeout(0, READ_TIMEOUT_US);

    auto res = cli.Get(request.c_str());

    if (!res) {
        LOG_ERROR("Failed to fetch response from ", url, request);
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

MatTableData parseMatTableData(const json& response) {
    MatTableData data;
    if (!response.is_object()) {
        return data;
    }

    // Robot adverse (couleur opposée au robot principal), vu par la caméra du mat.
    const auto opponents = response.find("opponents");
    if (opponents != response.end() && opponents->is_array() && !opponents->empty()) {
        const json& opponent = opponents->front();
        if (opponent.is_object()) {
            data.opponentVisible = true;
            data.opponentX = jsonNumber(opponent, "x");
            data.opponentY = jsonNumber(opponent, "y");
            data.opponentA = jsonNumber(opponent, "a");
        }
    }

    // Objets de jeu détectés par le mat.
    const auto objects = response.find("objects");
    if (objects != response.end() && objects->is_array()) {
        for (const json& item : *objects) {
            if (!item.is_object()) {
                continue;
            }
            MatGameElement element;
            element.id = static_cast<int>(jsonNumber(item, "id"));
            element.label = jsonString(item, "label");
            element.x = jsonNumber(item, "x");
            element.y = jsonNumber(item, "y");
            element.a = jsonNumber(item, "a");
            data.elements.push_back(element);
        }
    }

    return data;
}

#ifndef DISABLE_MAT

// Demande au mat de démarrer la détection ; renvoie false tant que le mat n'a
// pas répondu, avec un abandon au bout de 5 s.
bool StartMat(bool& connectionOk) {
    static unsigned long startTime = 0;
    json response;

    const bool status = restAPI_GET_(MAT_URL, "/start", response) && response.is_object();

    if (!status) {
        LOG_ERROR("Failed to start MAT");
    }

    if (status) {
        startTime = 0;
        connectionOk = true;
        return true;
    }

    if (startTime == 0) {
        startTime = _millis();
    } else if (_millis() - startTime > 5000) { // 5 seconds timeout
        LOG_ERROR("MAT failed to start within timeout");
        startTime = 0;
        connectionOk = false;
        return true;
    }
    LOG_EXTENDED_DEBUG("Waiting for MAT to start...");
    return false;
}

void StopMat() {
    json response;

    if (restAPI_GET_(MAT_URL, "/stop", response) && response.is_object()) {
        LOG_INFO("MAT stopped successfully");
    } else {
        LOG_ERROR("Failed to stop MAT");
    }
}

// Récupère l'état de la table (adversaire + objets de jeu) et le mémorise pour
// TableState::updateMapStatus().
bool getMapStatus() {
    json response;

    if (!restAPI_GET_(MAT_URL, "/fleet/live", response) || !response.is_object()) {
        LOG_ERROR("Failed to fetch map status from MAT");
        return false;
    }

    setMatTableData(parseMatTableData(response));
    return true;
}

#else  // DISABLE_MAT

// Local builds have no MAT (vision server) available, so the integration is
// stubbed out and the program never tries to reach it. StartMat reports
// "handled" so callers stop retrying, and the table keeps its locally computed
// state.
bool StartMat(bool& connectionOk) {
    connectionOk = false;
    return true;
}

void StopMat() {}

bool getMapStatus() {
    return false;
}

#endif  // DISABLE_MAT
