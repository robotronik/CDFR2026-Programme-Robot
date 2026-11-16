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

        // Positions recorded during the last camera calibration
        position_t prev_final_pos_cam = {0, 0, 0};
        position_t prev_final_pos_otos = {0, 0, 0};

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

        bool is_robot_stalled = false;  // Because of opponent in direction of movement
        unsigned long robot_stall_start_time = 0;
        bool forced_slow_mode = false;
        unsigned long stuck_start = 0;

        position_t current_pos_target = {0, 0, 0};
        bool current_use_astar = false;
        bool current_slow_mode = false;
        bool current_complete_stop = true;

        position_t currentPath[1024];
        int currentPathLength = 0;

        bool driving = true;
        position_t last_pos = {0, 0, 0};
};

// Global navigation instance, created in main.cpp
extern Navigation navigation;