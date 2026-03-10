#pragma once
#include "main.hpp"
#include "utils/logger.hpp"

void check(colorTeam_t color, int strategy);

// Function to handle the strategy
position_t StratStartingPos();

int chooseNextStock();

bool chooseStockStrategy(int& stockNum, int& stockOffset);

int GetBestDropZone(position_t fromPos);
int getBestStockPositionOff(int stockNum, position_t fromPos);
position_t getBestDropZonePosition(int dropzoneNum, position_t fromPos);
position_t calculateClosestArucoPosition(position_t currentPos, position_t& outPos);
