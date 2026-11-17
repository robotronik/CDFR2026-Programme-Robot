#include "drive_interface.h"
#include "lidar/Lidar.hpp"

class TableState;

// Pathfinding function
// Returns the length number of elements of the path found, or 0 if no path found (<= 0 indicates failure)
int pathfind(position_t start, position_t goal, position_t path[], double& path_length_mm);

void pathfind_setup(const TableState& tableState);
void pathfind_place_border();
void pathfind_fill_lidar(Lidar* lidar, const TableState& tableState);