#pragma once

#include "defs/structs.hpp"
#include <utils/json.hpp>
using json = nlohmann::json;

class RobotControl;
class TableState;
class Cam;

// Navigation return type
enum class NavReturn {
    InProcess,
    Done,
    Paused, // In case the opponent is in front
    Error,  // If locked for too long, for example
};

class Navigation {
    public:
        Navigation(RobotControl* robot, TableState* tableStatus, Cam* cam);
        ~Navigation() = default;

        // Go to a position, returns the navigation state
        NavReturn goTo(position_t pos, bool useAStar = false, bool slow_mode = false, bool complete_stop = true);
        NavReturn go();

        // Serialize the current navigation path
        void pathJson(json& j);

    private:
        NavReturn driveStep();
        void opponentDetection();

        RobotControl* robot;
        TableState* tableStatus;
        Cam* cam;

        bool forced_slow_mode = false;

        position_t current_pos_target = {0, 0, 0};
        bool current_use_astar = false;
        bool current_slow_mode = false;
        bool current_complete_stop = true;

        position_t currentPath[1024];
        int currentPathLength = 0;

        bool driving = true;
};

// Global navigation instance, created in main.cpp
extern Navigation navigation;