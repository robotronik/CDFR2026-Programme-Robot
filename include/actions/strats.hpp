#pragma once
#include "main.hpp"
#include "utils/logger.hpp"

inline void check(colorTeam_t color, int strategy){
    // Check if the color and strategy are valid
    if (color == NONE || strategy < 1 || strategy > 4)
        LOG_ERROR("Invalid color (", color, ") or strategy (", strategy, ")");
}

// Function to handle the strategy
inline position_t StratStartingPos(){
    // Returns the starting position of the robot
    colorTeam_t color = tableStatus.robot.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);
    position_t pos = {0, 0, 0};

    // First set the position as blue
    switch (strategy)
    {
    case 1:
        pos = {125, -1400, 0}; break;
    case 2:
        pos = {0,0,0}; break;
    case 3:
        pos = {125, -1400, 0}; break;
    case 4:
        pos = {1000, 100, 0}; break;
    }
    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}