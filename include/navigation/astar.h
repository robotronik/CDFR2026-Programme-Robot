#pragma once

#include "defs/structs.hpp"

#define SCALE 50
#define AS_HEIGHT (2000 / SCALE)
#define AS_WIDTH  (3000 / SCALE)

#define OBSTACLE_COST 255
#define MARGIN_COST   200
#define FREE_SPACE     0

extern unsigned char costmap[AS_HEIGHT][AS_WIDTH];

void astar_initialize_costmap();
void place_obstacle_with_margin(int x0_mm,int y0_mm,int w_mm,int h_mm,int RayonRobot);
void astar_pathfind(int *sx,int *sy,int *gx,int *gy);
int reconstruct_path(int sx,int sy,int gx,int gy,position_t *path);
int smooth_path(position_t *in,int in_len,position_t *out);
void print_costmap_with_path(position_t *path, int len, position_t start, position_t goal);
json astar_get_costmap_json();