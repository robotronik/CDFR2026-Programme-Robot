#include "navigation/pathfind.h"
#include "navigation/astar.h"
#include "defs/constante.h"
#include "utils/logger.hpp"
#include "main.hpp"

int pathfind(position_t start, position_t goal, position_t path[]) {


    // Toujours réinitialiser la costmap
    astar_initialize_costmap();
    pathfind_setup();

    // Ajouter les obstacles du lidar
    pathfind_fill_lidar();

    //LOG_INFO("Original start : ", start.x, " / ", start.y, " goal : ", goal.x, " / ", goal.y);

    int sx = (start.x + 1000) / SCALE;
    int sy = (start.y + 1500) / SCALE;
    int gx = (goal.x + 1000) / SCALE;
    int gy = (goal.y + 1500) / SCALE;

    // Clamp grid indices to valid costmap bounds to avoid out-of-bounds access.
    if (sx < 0) sx = 0;
    else if (sx >= AS_WIDTH) sx = AS_WIDTH - 1;
    if (gx < 0) gx = 0;
    else if (gx >= AS_WIDTH) gx = AS_WIDTH - 1;
    if (sy < 0) sy = 0;
    else if (sy >= AS_HEIGHT) sy = AS_HEIGHT - 1;
    if (gy < 0) gy = 0;
    else if (gy >= AS_HEIGHT) gy = AS_HEIGHT - 1;
    
    //LOG_INFO("Converted start : ", sx, " / ", sy, " goal : ", gx, " / ", gy);

    astar_pathfind(&sx, &sy, &gx, &gy);

    //LOG_INFO("After escape start : ", sx, " / ", sy, " goal : ", gx, " / ", gy);

    int len = reconstruct_path(sx, sy, gx, gy, path);
    //print_costmap_with_path(path, len, position_int_t{sx, sy}, position_int_t{gx, gy});
    if (len <= 0) {
        LOG_WARNING("Goal unreachable, printing costmap with path:");
        return 0;
    }

    int smooth_len = smooth_path(path, len, path);
    //print_costmap_with_path(path, smooth_len, position_int_t{sx, sy}, position_int_t{gx, gy});
    for(int i = 0; i < smooth_len; i++){
        path[i].x = path[i].x * SCALE - 1000;
        path[i].y = path[i].y * SCALE - 1500;
        path[i].a = goal.a;
    }
    path[smooth_len] = (position_t){goal.x, goal.y, goal.a};
    smooth_len++;
    return smooth_len;
}

void pathfind_setup() {
    astar_initialize_costmap();
    int RayonRobot=150;

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
    place_obstacle_with_margin(0,-1500,50,2000,RayonRobot,false);
    place_obstacle_with_margin(0,1500,50,2000,RayonRobot,false);
    place_obstacle_with_margin(-1000,0,3000,50,RayonRobot,false);
    place_obstacle_with_margin(1000,0,3000,50,RayonRobot,false);
    place_obstacle_with_margin(-775,0,1800,450,RayonRobot,false);
}

void pathfind_fill_lidar(){
    // Reset the costmap, pourquoi ?
    astar_initialize_costmap();
    pathfind_setup();

    for (int i = 0; i < lidar.count; i++){
        if (!lidar.data[i].onTable) continue;
        place_obstacle_with_margin(lidar.data[i].x,lidar.data[i].y,150, 150,150,false);
    }
}