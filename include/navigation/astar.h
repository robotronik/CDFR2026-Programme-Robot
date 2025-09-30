#pragma once

#include "defs/structs.hpp"

#define RESOLUTION 15       // mm par cellule
#define AS_HEIGHT 2000/RESOLUTION +1           // x vertical = lignes
#define AS_WIDTH 3000/RESOLUTION + 1         // y horizontal = colonnes

#define OBSTACLE_COST 30
#define SECURITE_PLANK 20
#define SECURITE_OPPONENT 200

extern unsigned char costmap[AS_HEIGHT][AS_WIDTH];

typedef struct 
{
    int  x;
    int  y;
    int cost;
} astar_pos_t;

void astar_initialize_costmap(int border_size);
void astar_place_obstacle_rect_with_inflation(int x, int y, int width, int height, int inflation_radius);

void astar_pathfind(int start_x, int start_y, int goal_x, int goal_y);
int astar_reconstruct_path_points(int start_x, int start_y, int goal_x, int goal_y, astar_pos_t *points, int max_points);

int astar_smooth_path(astar_pos_t *in_path, int in_length, astar_pos_t *out_path, int max_points);
int astar_smooth_path2(astar_pos_t *in_path, int in_length, position_t *out_path, int max_points, int window_size);

void astar_print_costmap_with_path(astar_pos_t *path, int path_len);
void astar_print_costmap();
void astar_print_costmap_around_point(int x, int y);
json astar_get_costmap_json();