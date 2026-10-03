#include "actions/strats.hpp"
#include "utils/logger.hpp"
#include "defs/structs.hpp"
#include <math.h>

void check(colorTeam_t color, int strategy){
    // Check if the color and strategy are valid
    if (color == NONE || strategy < 1 || strategy > 4)
        LOG_ERROR("Invalid color (", color, ") or strategy (", strategy, ")");
}

// Function to handle the strategy
position_t StratStartingPos(TableState* tableState){
    // Returns the starting position of the robot
    colorTeam_t color = tableState->colorTeam;
    int strategy = tableState->strategy;
    check(color, strategy);
    position_t pos = {-675, 1125, 120};

    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}

// NOTE: calculateClosestArucoPosition a été déplacée dans CalibrationAction.cpp
// (elle est désormais une méthode privée de CalibrationAction, seule utilisatrice
// de cette fonction).