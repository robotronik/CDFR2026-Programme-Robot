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

#define RESOLUTION 40       // mm par cellule
#define OBSTACLE_COST 20
#define FREE_SPACE 0
#define HEIGHT 2000/RESOLUTION +1           // x vertical = lignes
#define WIDTH 3000/RESOLUTION + 1         // y horizontal = colonnes
#define MAX_OPEN_SIZE (HEIGHT * WIDTH)
const int SECURITE_PLANK = 20;
const int SECURITE_OPPONENT = 200;

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


// navigation/nav.h

void initialize_costmap();
void place_obstacle_rect_with_inflation(int x, int y, int width, int height, int inflation);
void print_costmap();
int reconstruct_path_points(int start_x, int start_y, int goal_x, int goal_y, position_int *points, int max_points);
void print_costmap_with_path(position_int *path, int path_len);
int smooth_path(position_int *in_path, int in_length, position_int *out_path, int max_points);
void a_star(int start_x, int start_y, int goal_x, int goal_y);
int convert_x_to_index(int x);
int convert_y_to_index(int y);
void convert_path_to_coordinates(position_int *path, int path_len);
void print_costmap_around_point(int x, int y);
#endif