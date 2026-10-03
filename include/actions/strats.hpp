#pragma once
#include "defs/structs.hpp" // For colorTeam_t & position_t
#include "defs/tableState.hpp"

void check(colorTeam_t color, int strategy);

// Function to handle the strategy
position_t StratStartingPos(TableState* tableState);

position_t calculateClosestArucoPosition(position_t currentPos);

