#pragma once
#include "utils/json.hpp" // For handling JSON
#include "utils/httplib.h"
#include <vector>


using json = nlohmann::json;


const std::string MAT_URL = "mat.local:8000";

bool getMapStatus();
bool StartMat(bool& connectionOk);
void StopMat();