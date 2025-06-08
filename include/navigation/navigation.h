#pragma once

#include "defs/structs.hpp"
#include <utils/json.hpp>
using json = nlohmann::json;

// Navigation return type
typedef enum {
    NAV_IN_PROCESS,
    NAV_DONE,
    NAV_PAUSED, // In case the opponent is in front
    NAV_ERROR,  // If locked for too long, for example
} nav_return_t;

// Navigation functions
nav_return_t navigationGoTo(position_t pos, bool turnEnd = false, bool useAStar = false);
nav_return_t navigationPath(position_t path[], int pathLenght, bool turnEnd = false);
void navigation_path_json(json& j);
void navigationOpponentDetection();

// FOR TESTING PURPOSES
void fillCurrentPath(position_t path[], int pathLength);