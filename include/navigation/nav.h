#ifndef NAV_H
#define NAV

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <utils/json.hpp>
#include "defs/constante.h" 
using json = nlohmann::json;

//#define ROBOT_WIDTH 200     // mm
#define RESOLUTION 40       // mm par cellule
#define OBSTACLE_COST 20
#define FREE_SPACE 0
#define MAX_OPEN_SIZE (HEIGHT * WIDTH)
#define HEIGHT 50
#define WIDTH 75

extern unsigned char costmap[HEIGHT][WIDTH];

typedef struct {
    int x, y;
    int g;  // Coût depuis le départ
    int h;  // Heuristique jusqu’à la cible
    int f;  // f = g + h
    int parent_x, parent_y;
    bool visited;
} Node;

extern Node nodes[HEIGHT][WIDTH];

typedef struct {
    int x, y;
} Point;

// navigation/nav.h

void initialize_costmap();
void place_obstacle_rect_with_inflation(int x, int y, int width, int height, int inflation);
void print_costmap();
int reconstruct_path_points(int start_x, int start_y, int goal_x, int goal_y, Point *points, int max_points);
void print_costmap_with_path(Point *path, int path_len);
int smooth_path(Point *in_path, int in_length, Point *out_path, int max_points);
void a_star(int start_x, int start_y, int goal_x, int goal_y);
#endif