#include "actions/strats.hpp"
#include "actions/functions.h"
#include "utils/logger.hpp"
#include "defs/structs.hpp"
#include "navigation/driveControl.h"
#include <math.h>
#include "main.hpp" // for tableStatus
#include "navigation/pathfind.h"

void check(colorTeam_t color, int strategy){
    // Check if the color and strategy are valid
    if (color == NONE || strategy < 1 || strategy > 4)
        LOG_ERROR("Invalid color (", color, ") or strategy (", strategy, ")");
}

// Function to handle the strategy
position_t StratStartingPos(){
    // Returns the starting position of the robot
    colorTeam_t color = tableStatus.colorTeam;
    int strategy = tableStatus.strategy;
    check(color, strategy);
    position_t pos = {-675, 1125, 120};

    if (color == YELLOW)
        position_robot_flip(pos);
    return pos;
}

// Return the closest position to look at an aruco marker
position_t calculateClosestArucoPosition(position_t currentPos){
    position_t outPos = currentPos;
    position_t closestPos = ARUCO_POSITIONS_TABLE[0];
    double minDistance = position_distance(currentPos, ARUCO_POSITIONS_TABLE[0]);
    for (int i = 1; i < 4; i++){
        double d = position_distance(currentPos, ARUCO_POSITIONS_TABLE[i]);
        if (d < minDistance){
            minDistance = d;
            closestPos = ARUCO_POSITIONS_TABLE[i];
        }
    }
    LOG_ERROR("Distance to closest aruco marker: ", minDistance);
    const double target_distance_min = 350.0; // mm
    const double target_distance_max = 550.0; // mm

    if (minDistance < target_distance_min || minDistance > target_distance_max){
        LOG_WARNING("Not in valid range, moving to preset position");
        // Calculate the closest valid position using predetermined pos
        outPos = ARUCO_CALIB_POSITIONS[0];
        double minTargetDistance = 1e6;
        for (int i = 0; i < ARUCO_CALIB_POSITIONS_COUNT; i++){
            // Blue side
            double d = position_distance(currentPos, ARUCO_CALIB_POSITIONS[i]);
            if (d < minTargetDistance){
                minTargetDistance = d;
                outPos = ARUCO_CALIB_POSITIONS[i];
            }
            // Yellow side (mirrored)
            position_t mirroredPos = ARUCO_CALIB_POSITIONS[i];
            mirroredPos.y = -mirroredPos.y;
            d = position_distance(currentPos, mirroredPos);
            if (d < minTargetDistance){
                minTargetDistance = d;
                outPos = mirroredPos;
            }
        }
    }
    else {
        LOG_DEBUG("Good distance → no movement");
    }
    outPos.a = RAD_TO_DEG * position_angle(outPos, closestPos) + OFFSET_CAM_A;

    return outPos;
}
