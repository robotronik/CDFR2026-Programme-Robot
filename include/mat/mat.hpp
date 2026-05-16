#pragma once
#include "utils/json.hpp" // For handling JSON
#include "utils/httplib.h"
#include <vector>


using json = nlohmann::json;

class MatCam {
    private:
        const std::string MAT_URL = "http://mat.local:80";
        bool restAPI_GET(const std::string &url, const std::string &resquest, json &response);
    public:
        bool getMapStatus(std::vector<bool>& stock, std::vector<std::pair<int, int>>& dropzone, int& sucess);
        bool Start();
};