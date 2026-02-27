#pragma once
#include "main.hpp"
#include "utils/logger.hpp"

inline void check(colorTeam_t color, int strategy){
inline void check(colorTeam_t color, int strategy){
    // Check if the color and strategy are valid
    if (color == NONE || strategy < 1 || strategy > 4)
        LOG_ERROR("Invalid color (", color, ") or strategy (", strategy, ")");
}

// Function to handle the strategy
inline position_t StratStartingPos(){
inline position_t StratStartingPos(){
    // Returns the starting position of the robot
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);
    position_t pos = {-775, 1125, 0};

    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}