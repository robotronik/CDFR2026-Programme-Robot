#include "mat/mat.hpp"
#include "utils/logger.hpp"



bool MatCam::restAPI_GET(const std::string &url, const std::string &resquest, json &response) {
    // HTTP
    httplib::Client cli(url);
    auto res = cli.Get(resquest.c_str());
    // Check for nullptr
    if (!res) {
        LOG_ERROR("Failed to fetch response from ", url, resquest);
        return false;
    }
    // LOG_GREEN_INFO("HTML Status is ", res->status);
    // LOG_GREEN_INFO("HTML Body is ", res->body);

    // Check if the response code is 200 (OK)
    if (res->status != 200) {
        LOG_ERROR("HTTP error: ", res->status);
        return false;
    }
    try {
        // Parse JSON response
        response = json::parse(res->body);
        // LOG_GREEN_INFO("API Response: ", response.dump(4));
        return true;
    } catch (const json::parse_error& e) {
        LOG_ERROR("JSON parse error: ", e.what());
        return false;
    }
}

bool MatCam::Start() {
    json response;
    if (restAPI_GET(MAT_URL, "/start", response) == false) {
        LOG_ERROR("Failed to start MAT");
        return false;
    }
    if (response.is_null()) {
        LOG_ERROR("Response is null, MAT might not be running");
        return false;
    }
    bool success = response.value("success", false);
    if (!success) {
        LOG_ERROR("MAT failed to start");
        return false;
    }
    LOG_GREEN_INFO("MAT started successfully");
    return true;
}

bool MatCam::getMapStatus(std::vector<bool>& stock, std::vector<std::pair<int, int>>& dropzone, int& sucess) {
    json response;
    
    if (restAPI_GET(MAT_URL, "/map_status", response) == false) {
        LOG_ERROR("Failed to fetch map status from MAT");
        return false;
    }
    if (response.is_null()) {
        LOG_ERROR("Response is null, MAT might not be running");
        return false;
    }
    
    try {
        std::vector<int> temp_stock = response.value("stock", std::vector<int>{});
        
        stock.clear();
        stock.reserve(temp_stock.size()); // Optimisation de la mémoire
        for (int val : temp_stock) {
            stock.push_back(val != 0); 
        }

        dropzone = response.value("dropzone", std::vector<std::pair<int, int>>{});
        
        sucess = 1;
        return true;
        
    } catch (const json::exception& e) { // Utilisez json::exception pour attraper toutes les erreurs JSON
        LOG_ERROR("JSON error: ", e.what());
        return false;
    }
}