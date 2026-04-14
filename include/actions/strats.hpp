#pragma once
#include "drive_interface.h" // For position_t
#include "utils/utils.h" // For colorTeam_t

void check(colorTeam_t color, int strategy);

// Function to handle the strategy
position_t StratStartingPos();

int chooseNextStock();

bool chooseStockStrategy(int& stockNum, int& stockOffset);

int GetBestDropZone(position_t fromPos);
int getBestStockPositionOff(int stockNum, position_t fromPos);

position_t getBestDropZonePosition(int dropzoneNum, position_t fromPos);
bool getBestStealZonePosition(position_t fromPos, int& bestDropZone, position_t& bestPos);

position_t calculateClosestArucoPosition(position_t currentPos);
position_t getBestIsolatedPosition(position_t centerPos, position_t fromPos);
position_t toFirstStockPos(position_t targetPos);
