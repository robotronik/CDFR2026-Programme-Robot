#pragma once
#include "drive_interface.h" // For position_t
#include "utils/utils.h" // For colorTeam_t

void check(colorTeam_t color, int strategy);

// Function to handle the strategy
position_t StratStartingPos();

double chooseNextStock(int& closest_stock, int& closest_offset);

double toAStarDistStock(int stockNum, int stockOffset);
double toAStarDist(position_t a);
double chooseStockStrategy(int& stockNum, int& stockOffset);

double getBestDropZonePosition(int& dropzoneNum, position_t& bestPoss, bool steal = false);
double getBestStealZonePosition(int& bestDropZone, position_t& bestPos);
double getBestStockPositionOff(int& stockNum, int& bestOffset);

position_t calculateClosestArucoPosition(position_t currentPos);
position_t getBestIsolatedPosition(position_t centerPos, position_t fromPos);
position_t toFirstStockPos(position_t targetPos);
bool NearestValidZone(position_t* pos);
