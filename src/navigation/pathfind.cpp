#include "navigation/pathfind.h"
#include "navigation/astar.h"
#include "defs/constante.h"
#include "utils/logger.hpp"
#include "main.hpp" //lidar

// TODO Should this be round() or floor ?
position_int_t convert_to_astar(position_t p){
    position_int_t k;
    k.x = (int)round(p.x + 1000) / SCALE;
    k.y = (int)round(p.y + 1500) / SCALE;
    if (k.x < 0) k.x = 0;
    else if (k.x >= AS_HEIGHT) k.x = AS_HEIGHT - 1;
    if (k.y < 0) k.y = 0;
    else if (k.y >= AS_WIDTH) k.y = AS_WIDTH - 1;
    return k;
}

position_t convert_from_astar(position_int_t k){
    position_t p;
    p.x = k.x * SCALE - 1000;
    p.y = k.y * SCALE - 1500;
    return p;
}

position_int_t astar_path[AS_HEIGHT + AS_WIDTH];
position_int_t smooth_path_arr[AS_HEIGHT + AS_WIDTH];

void place_obstacle_with_margin(double cx, double cy, int w_mm, int h_mm, int RayonRobot, bool traversable = true)
{
    position_int_t c = convert_to_astar(position_t{cx, cy, 0.0});
    int w = w_mm / SCALE;
    int h = h_mm / SCALE;
    int margin = RayonRobot / SCALE;
    return astar_place_obstacle_with_margin(c, w, h, margin, traversable);
}

int pathfind(position_t start, position_t goal, position_t path[], double& path_lenght_mm){
    //LOG_INFO("Original start : ", start.x, " / ", start.y, " goal : ", goal.x, " / ", goal.y);

    position_int_t k_start = convert_to_astar(start);
    position_int_t k_goal = convert_to_astar(goal);

    if (get_cost(k_goal) > MARGIN_COST){
        LOG_WARNING("Goal unreachable because of goal cost");
        return 0;
    }

    int len = astar_pathfind(k_start, k_goal, astar_path);
    if (len <= 0) {
        LOG_WARNING("Goal unreachable, printing costmap with path:");
        //print_costmap_with_path(astar_path, len, k_start, k_goal);
        return 0;
    }

    int smooth_len = smooth_path(astar_path, len, smooth_path_arr);
    //print_costmap_with_path(smooth_path_arr, smooth_len, k_start, k_goal);
    // Convert path to position_t coordinates
    for(int i = 0; i < smooth_len; i++){
        path[i] = convert_from_astar(smooth_path_arr[i]);
        path[i].a = goal.a;
    }
    path[smooth_len] = goal;
    smooth_len++;

    path_lenght_mm = astart_path_lenght(smooth_path_arr, smooth_len) * SCALE;

    return smooth_len;
}

double pathfind_lenght_mm(position_t start, position_t goal){
    position_int_t k_start = convert_to_astar(start);
    position_int_t k_goal = convert_to_astar(goal);
    int len = astar_pathfind(k_start, k_goal, astar_path);
    if (len <= 0) {
        LOG_WARNING("Unreachable");
        return 1e6; //Big
    }

    int smooth_len = smooth_path(astar_path, len, smooth_path_arr);
    // Compute path lenght
    return(astart_path_lenght(smooth_path_arr, smooth_len) * SCALE);
}

void pathfind_setup() {
    astar_initialize_costmap();
    int RayonRobot=200;

    // ===== DROP ZONES =====
    place_obstacle_with_margin( 200,  1400, 200, 200, RayonRobot);
    place_obstacle_with_margin( 900,   800, 200, 200, RayonRobot);
    place_obstacle_with_margin( 200,   700, 200, 200, RayonRobot);
    place_obstacle_with_margin(-450,   250, 200, 200, RayonRobot);
    place_obstacle_with_margin( 200,     0, 200, 200, RayonRobot);

    place_obstacle_with_margin( 200, -1400, 200, 200, RayonRobot);
    place_obstacle_with_margin( 900,  -800, 200, 200, RayonRobot);
    place_obstacle_with_margin( 200,  -700, 200, 200, RayonRobot);
    place_obstacle_with_margin(-450,  -250, 200, 200, RayonRobot);
    place_obstacle_with_margin( 900,     0, 200, 200, RayonRobot);

    // ===== TABLE BORDER ===== 
    place_obstacle_with_margin(    0,-1500,   50, 2000, RayonRobot, false);
    place_obstacle_with_margin(    0, 1500,   50, 2000, RayonRobot, false);
    place_obstacle_with_margin(-1000,    0, 3000,   50, RayonRobot, false);
    place_obstacle_with_margin( 1000,    0, 3000,   50, RayonRobot, false);
    place_obstacle_with_margin( -775,    0, 1800,  450, RayonRobot, false);
}

void pathfind_fill_lidar(){
    pathfind_setup();

    for (int i = 0; i < lidar.count; i++){
        if (!lidar.data[i].onTable) continue;
        // TODO Dont add all of the lidar points to improve performance
        place_obstacle_with_margin(lidar.data[i].x,lidar.data[i].y,150, 150,150,false);
    }
}