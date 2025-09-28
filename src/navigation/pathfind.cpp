#include "navigation/pathfind.h"
#include "navigation/astar.h"

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

int convert_x_to_index(double x) {
    int offset = 1000; // Décalage pour convertir [-1000:1000] en [0:2000]
    int index = round((x + offset) / RESOLUTION);
    index = CLAMP(index, 0, WIDTH - 1); // Clamp pour éviter les erreurs
    return index;
}
int convert_y_to_index(double y) {
    int offset = 1500; // Décalage pour convertir [-1500:1500] en [0:3000]
    int index = round((y + offset) / RESOLUTION);
    index = CLAMP(index, 0, HEIGHT - 1); // Clamp pour éviter les erreurs
    return index;
}
void convert_pos_to_index(position_t pos, int& ix, int& iy){
    ix = convert_x_to_index(pos.x);
    iy = convert_y_to_index(pos.y);
}

void convert_path_to_coordinates(nav_pos_t nav_path[], int path_len, position_t path[]) {
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

    a_star(start_ix, start_iy, goal_ix, goal_iy);   
    nav_pos_t nav_path[1024];
    int path_len = reconstruct_path_points(start_ix, start_iy, goal_ix, goal_iy, nav_path, 1024);
    if (path_len <= 0) {
        return -1; // No path found
    }

    nav_pos_t smooth_nav_path[1024];
    int smooth_path_len = smooth_path(nav_path, path_len, smooth_nav_path, 1024);
    if (smooth_path_len <= 0) {
        return -1; // Smoothing failed
    }

    convert_path_to_coordinates(smooth_nav_path, smooth_path_len, path);
    return smooth_path_len; // Return the length of the path
}