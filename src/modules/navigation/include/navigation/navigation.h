#pragma once

#include "defs/structs.hpp"
#include <utils/json.hpp>
using json = nlohmann::json;

class DriveControl;
class TableState;
class Cam;

// Navigation return type
typedef enum {
    NAV_IN_PROCESS,
    NAV_DONE,
    NAV_PAUSED, // In case the opponent is in front
    NAV_ERROR,  // If locked for too long, for example
} nav_return_t;

class Navigation {
    public:
        Navigation(DriveControl* drive, TableState* tableStatus, Cam* cam);
        ~Navigation() = default;

        // Positions recorded during the last camera calibration
        position_t prev_final_pos_cam = {0, 0, 0};
        position_t prev_final_pos_otos = {0, 0, 0};

        // Go to a position, returns the navigation state
        nav_return_t goTo(position_t pos, bool useAStar = false, bool slow_mode = false, bool complete_stop = true);
        nav_return_t go();

        // Serialize the current navigation path
        void pathJson(json& j);

    private:
        nav_return_t driveStep();
        void opponentDetection();

        DriveControl* drive;
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