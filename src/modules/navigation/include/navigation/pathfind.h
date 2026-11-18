#pragma once


#include "drive_interface.h"
#include <span>

class AStar;
class Lidar;
class TableState;

int pathfind(position_t start, position_t goal, std::span<position_t> path, double& pathLengthMm);
void updateGameObstacles(AStar& planner, const TableState& tableState);
void updateOpponentObstacles(AStar& planner, const Lidar& lidar);
