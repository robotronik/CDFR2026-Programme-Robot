#pragma once
#include "defs/structs.hpp" // For colorTeam_t & position_t

void check(colorTeam_t color, int strategy);

// Function to handle the strategy
position_t StratStartingPos();

position_t calculateClosestArucoPosition(position_t currentPos);

