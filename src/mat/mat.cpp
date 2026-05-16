#include "mat/mat.hpp"
#include "utils/logger.hpp"



bool restAPI_GET_(const std::string &url, const std::string &resquest, json &response) {
    httplib::Client cli(url);
    
    // --- AJOUT : Définir des timeouts courts ---
    // Les paramètres sont (secondes, microsecondes)
    cli.set_connection_timeout(0, 10000); // Timeout de connexion à 20ms
    cli.set_read_timeout(0, 10000);       // Timeout de lecture à 20ms
    // -------------------------------------------

    auto res = cli.Get(resquest.c_str());
    
    if (!res) {
        LOG_ERROR("Failed to fetch response from ", url, resquest);
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

bool StartMat() {
    static unsigned long startTime = 0;
    json response;
    
    bool status = false; 

    if (restAPI_GET_(MAT_URL, "/start", response) == false) {
        LOG_ERROR("Failed to start MAT");
    } else if (response.is_null()) {
        LOG_ERROR("Response is null, MAT might not be running");
    } else {
        bool success = response.value("success", false);
        if (!success) {
            LOG_ERROR("MAT failed to start");
        } else {
            status = true; 
        }
    }

    if (status) {
        startTime = 0;
        return true;
    } else {
        if (startTime == 0) {
            startTime = _millis();
        } else if (_millis() - startTime > 5000) { // 5 seconds timeout
            LOG_ERROR("MAT failed to start within timeout");
            startTime = 0; 
            return true; 
        }
        LOG_EXTENDED_DEBUG("Waiting for MAT to start...");
        return false;
    }
}

void StopMat() {
    json response;
    
    if (restAPI_GET_(MAT_URL, "/stop", response) == false) {
        LOG_ERROR("Failed to stop MAT");
    } else if (response.is_null()) {
        LOG_ERROR("Response is null, MAT might not be running");
    } else {
        bool success = response.value("success", false);
        if (!success) {
            LOG_ERROR("MAT failed to stop");
        } else {
            LOG_INFO("MAT stopped successfully");
        }
    }
}

bool getMapStatus(std::vector<bool>& stock, std::vector<std::pair<int, int>>& dropzone) {
    json response;
    
    if (restAPI_GET_(MAT_URL, "/map", response) == false) {
        LOG_ERROR("Failed to fetch map status from MAT");
        return false;
    }
    if (response.is_null()) {
        LOG_ERROR("Response is null, MAT might not be running");
        return false;
    }
    
    try {
        std::vector<int> temp_stock = response.value("P", std::vector<int>{});
        
        stock.clear();
        stock.reserve(temp_stock.size()); // Optimisation de la mémoire
        for (int val : temp_stock) {
            stock.push_back(val != 0); 
        }
        dropzone = response.value("D", std::vector<std::pair<int, int>>{});
        
        return true;
        
    } catch (const json::exception& e) { // Utilisez json::exception pour attraper toutes les erreurs JSON
        LOG_ERROR("JSON error: ", e.what());
        return false;
    }
}