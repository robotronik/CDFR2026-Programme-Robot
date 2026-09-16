#pragma once
#include "drive_interface.h" // For position_t
#include "utils/utils.h" // For colorTeam_t

void check(colorTeam_t color, int strategy);

// Function to handle the strategy
position_t StratStartingPos();

position_t calculateClosestArucoPosition(position_t currentPos);

