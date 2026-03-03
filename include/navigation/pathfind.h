#include "defs/structs.hpp"


// Pathfinding function
// Returns the length of the path found, or 0 if no path found (<= 0 indicates failure)
int pathfind(position_t start, position_t goal, position_t path[]);

void pathfind_setup();
void pathfind_fill_lidar();