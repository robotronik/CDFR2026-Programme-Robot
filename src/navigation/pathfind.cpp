#include "navigation/pathfind.h"
#include "navigation/astar.h"
#include "defs/constante.h"
#include "main.hpp"

int pathfind(position_t start, position_t goal, position_t path[]) {
    int RayonRobot=150;

    int sx=(start.x+1000)/SCALE;
    int sy=(start.y+1500)/SCALE;
    int gx=(goal.x+1000)/SCALE;
    int gy=(goal.y+1500)/SCALE;

    
    astar_pathfind(&sx,&sy,&gx,&gy);
    int len=reconstruct_path(sx,sy,gx,gy,path);

    //print_costmap_with_path(path,len);
    int smooth_len=smooth_path(path,len,path);
    path[smooth_len] = (position_t){goal.x, goal.y};

    print_costmap_with_path(path,smooth_len,position_t {(start.x+1000)/SCALE,(start.y+1500)/SCALE}, position_t {(goal.x+1000)/SCALE,(goal.y+1500)/SCALE});
    for(int i = 0; i < smooth_len; i++){
        path[i].x = path[i].x * SCALE - 1000;
        path[i].y = path[i].y * SCALE - 1500;
    }
    smooth_len ++;
    return smooth_len;}

void pathfind_setup() {
    astar_initialize_costmap();
    int RayonRobot=150;
    // ===== TABLE BORDER ===== 
    place_obstacle_with_margin(0,-1500,50,2000,RayonRobot);
    place_obstacle_with_margin(0,1500,50,2000,RayonRobot);
    place_obstacle_with_margin(-1000,0,3000,50,RayonRobot);
    place_obstacle_with_margin(1000,0,3000,50,RayonRobot);
    place_obstacle_with_margin(-775,0,1800,450,RayonRobot);

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
}

void pathfind_fill_lidar(){
    // Reset the costmap, pourquoi ?
    pathfind_setup();

    for (int i = 0; i < lidar.count; i++){
        if (!lidar.data[i].onTable) continue;
        place_obstacle_with_margin(lidar.data[i].x,lidar.data[i].y,150, 150,150);
    }
}