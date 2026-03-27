#pragma once

#include "defs/structs.hpp"

#define SCALE 50
#define AS_HEIGHT ((2000 / SCALE) + 1)
#define AS_WIDTH  ((3000 / SCALE) + 1)

#define OBSTACLE_COST 255
#define MARGIN_COST   200
#define FREE_SPACE     0

// Maximum possible path length on the A* grid (conservative bound).
#define MAX_PATH_LEN (4 * (AS_HEIGHT + AS_WIDTH))

typedef struct {
    int x;
    int y;
} position_int_t;

void astar_initialize_costmap();
void astar_place_obstacle_with_margin(position_int_t c, int w, int h, int margin, bool traversable = true);
int astar_pathfind(position_int_t start, position_int_t goal, position_int_t path[]);
int smooth_path(position_int_t in[], int in_len, position_int_t out[]);
int coarse_smooth_path(position_int_t in[], int in_len, position_int_t out[]);
void print_costmap_with_path(position_int_t path[], int len, position_int_t start, position_int_t goal);
unsigned char get_cost(position_int_t p);
double astar_path_length(position_int_t path[], int len);
json astar_get_costmap_json();