#include "navigation/pathfind.h"
#include "navigation/astar.h"
#include "defs/constante.h"
#include "main.hpp"

void pathfindInit(){
    int border = ROBOT_WIDTH / 2/RESOLUTION;
    astar_initialize_costmap(border);
}

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

int convert_x_to_index(double x) {
    int offset = 1000; // Décalage pour convertir [-1000:1000] en [0:2000]
    int index = round((x + offset) / RESOLUTION);
    index = CLAMP(index, 0, AS_WIDTH - 1); // Clamp pour éviter les erreurs
    return index;
}
int convert_y_to_index(double y) {
    int offset = 1500; // Décalage pour convertir [-1500:1500] en [0:3000]
    int index = round((y + offset) / RESOLUTION);
    index = CLAMP(index, 0, AS_HEIGHT - 1); // Clamp pour éviter les erreurs
    return index;
}
void convert_pos_to_index(position_t pos, int& ix, int& iy){
    ix = convert_x_to_index(pos.x);
    iy = convert_y_to_index(pos.y);
}

void convert_path_to_coordinates(astar_pos_t nav_path[], int path_len, position_t path[]) {
    for (int i = 0; i < path_len; i++) {
        path[i].x = nav_path[i].x * RESOLUTION - 1000; // Conversion en coordonnées x
        path[i].y = nav_path[i].y * RESOLUTION - 1500; // Conversion en coordonnées y
    }
}

int pathfind(position_t start, position_t goal, position_t path[]) {
    int start_ix, start_iy;
    int goal_ix, goal_iy;
    convert_pos_to_index(start, start_ix, start_iy);
    convert_pos_to_index(goal, goal_ix, goal_iy);

    astar_pathfind(start_ix, start_iy, goal_ix, goal_iy);   
    astar_pos_t nav_path[1024];
    int path_len = astar_reconstruct_path_points(start_ix, start_iy, goal_ix, goal_iy, nav_path, 1024);
    if (path_len <= 0) {
        return -1; // No path found
    }

    astar_pos_t smooth_nav_path[1024];
    int smooth_path_len = astar_smooth_path(nav_path, path_len, smooth_nav_path, 1024);
    if (smooth_path_len <= 0) {
        return -1; // Smoothing failed
    }

    convert_path_to_coordinates(smooth_nav_path, smooth_path_len, path);
    return smooth_path_len; // Return the length of the path
}

// Place un obstacle rectangulaire centré en (cx, cy), avec largeur et hauteur en mm, obstacle_radius_mm est la marge que tu donnes à la zone (pour un mur, 10cm devrait suffire)
void pathfind_place_obstacle_rect_with_inflation(double cx, double cy, int height_mm, int width_mm, int obstacle_radius_mm) {
    cx = round(cx);
    cy = round(cy);
    int ix = convert_x_to_index(cx);
    int iy = convert_y_to_index(cy);
    int w_cells = height_mm / (RESOLUTION);
    int h_cells = width_mm / (RESOLUTION);

    int inflation_radius = (ROBOT_WIDTH / 2 + obstacle_radius_mm) / RESOLUTION;

    astar_place_obstacle_rect_with_inflation(ix, iy, h_cells, w_cells, inflation_radius);
}

void pathfind_setup() {
    pathfind_place_obstacle_rect_with_inflation(-725,  675, STOCK_WIDTH_MM, STOCK_HEIGHT_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation(-325, 1425, STOCK_HEIGHT_MM, STOCK_WIDTH_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation( 600, 1425, STOCK_HEIGHT_MM, STOCK_WIDTH_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation( 750,  725, STOCK_WIDTH_MM, STOCK_HEIGHT_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation(  50,  400, STOCK_WIDTH_MM, STOCK_HEIGHT_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation(-725, -675, STOCK_WIDTH_MM, STOCK_HEIGHT_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation(-325,-1425, STOCK_HEIGHT_MM, STOCK_WIDTH_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation( 600,-1425,  STOCK_HEIGHT_MM, STOCK_WIDTH_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation( 750, -725, STOCK_WIDTH_MM, STOCK_HEIGHT_MM, SECURITE_PLANK);
    pathfind_place_obstacle_rect_with_inflation(  50, -400,  STOCK_WIDTH_MM, STOCK_HEIGHT_MM, SECURITE_PLANK);
}

void pathfind_fill_lidar(){
    // Reset the costmap
    pathfindInit();

    for (int i = 0; i < lidar.count; i++){
        if (!lidar.data[i].onTable) continue;
        pathfind_place_obstacle_rect_with_inflation(
            lidar.data[i].x,
            lidar.data[i].y,
            100, 100,
            20
        );
    }
}