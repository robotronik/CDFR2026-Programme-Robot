#pragma once
#include "drive_interface.h" // For position_t
#include "utils/utils.h" // For colorTeam_t

void check(colorTeam_t color, int strategy);

// Function to handle the strategy
position_t StratStartingPos();

int chooseNextStock();

int toAStarDistStock(int stockNum, int stockOffset);
int toAStarDist(position_t a);
int chooseStockStrategy(int& stockNum, int& stockOffset);

int GetBestDropZone();
int getBestStockPositionOff(int stockNum);

position_t getBestDropZonePosition(int dropzoneNum, bool steal = false);
int getBestStealZonePosition(int& bestDropZone, position_t& bestPos);

position_t calculateClosestArucoPosition(position_t currentPos);
position_t getBestIsolatedPosition(position_t centerPos, position_t fromPos);
position_t toFirstStockPos(position_t targetPos);
