#include "defs/structs.hpp"


void pathfindInit();
// Pathfinding function
// Returns the length of the path found, or -1 if no path found
int pathfind(position_t start, position_t goal, position_t path[]);

// Place un obstacle rectangulaire centré en (cx, cy), avec largeur et hauteur en mm, obstacle_radius_mm est la marge que tu donnes à la zone (pour un mur, 10cm devrait suffire)
void pathfind_place_obstacle_rect_with_inflation(double cx, double cy, int height_mm, int width_mm, int obstacle_radius_mm);

void pathfind_setup();
void pathfind_fill_lidar();