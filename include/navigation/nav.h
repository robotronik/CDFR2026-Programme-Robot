#pragma once

#include "defs/structs.hpp"

#define RESOLUTION 15       // mm par cellule
#define HEIGHT 2000/RESOLUTION +1           // x vertical = lignes
#define WIDTH 3000/RESOLUTION + 1         // y horizontal = colonnes

#define OBSTACLE_COST 30
#define SECURITE_PLANK 20
#define SECURITE_OPPONENT 200


extern unsigned char costmap[HEIGHT][WIDTH];

// navigation/nav.h

void initialize_costmap();
void place_obstacle_rect_with_inflation(int x, int y, int width, int height, int inflation);
void print_costmap();
int reconstruct_path_points(int start_x, int start_y, int goal_x, int goal_y, nav_pos_t *points, int max_points);
void print_costmap_with_path(nav_pos_t *path, int path_len);
int smooth_path(nav_pos_t *in_path, int in_length, nav_pos_t *out_path, int max_points);
int smooth_path2(nav_pos_t *in_path, int in_length, position_t *out_path, int max_points, int window_size);
void a_star(int start_x, int start_y, int goal_x, int goal_y);
int convert_x_to_index(double x);
int convert_y_to_index(double y);
void convert_pos_to_index(position_t pos, int& ix, int& iy);
void convert_path_to_coordinates(nav_pos_t nav_path[], int path_len, position_t path[]);
void print_costmap_around_point(int x, int y);
