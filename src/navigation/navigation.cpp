#include "navigation/navigation.h"
#include "main.hpp"
#include "defs/constante.h" // DISTANCESTOP and DISTANCESTART
#include "utils/logger.hpp"
#include "lidar/lidarAnalize.h"
#include "navigation/pathfind.h"


static bool is_robot_stalled = false;
static unsigned long robot_stall_start_time;

static position_t current_pos_target;
static bool current_use_astar;

static position_t currentPath[1024];
static int currentPathLength = 0;

nav_return_t navigationDrive(){
    // Calculate the path
    if (current_use_astar){
        currentPathLength = pathfind(drive.position, current_pos_target, currentPath);
        if (currentPathLength == -1){
            LOG_ERROR("No path found");
            if (!is_robot_stalled){
                is_robot_stalled = true;
                robot_stall_start_time = _millis();
            }
            return NAV_IN_PROCESS;
        }
        else {
            LOG_GREEN_INFO("Path found!");
        }
    }
    else{
        currentPathLength = 1;
        currentPath[0] = current_pos_target;
    }
    bool done = drive.drive(currentPath, currentPathLength);
    if (done) return NAV_DONE;
    return NAV_IN_PROCESS;
}

nav_return_t navigationGo(){
    // FSM which does drive and calibration
    static bool driving = true;
    if (driving){
        nav_return_t result = navigationDrive();
        if (result == NAV_DONE){
            LOG_INFO("Navigation drive completed");
            driving = false;
        }
    } else {
        // Calibrate using camera
        bool cam_success;
        position_t robot_pos;
        if (arucoCam1.getRobotPos(robot_pos.x, robot_pos.y, robot_pos.a, cam_success)){
            if (cam_success){
                drive.setCoordinates(robot_pos);
                tableStatus.resetCalibrationAge();
                LOG_GREEN_INFO("Camera calibration successful, new position: { x = ", robot_pos.x, " y = ", robot_pos.y, " a = ", robot_pos.a, " }");
            }
            else{
                LOG_WARNING("Camera did not have a good position estimate, skipping calibration");
            }
            driving = true;
            return NAV_DONE;
        }
    }
    return NAV_IN_PROCESS;
}

nav_return_t navigationGoTo(position_t pos, bool turnEnd, bool useAStar){
    if (current_pos_target.x != pos.x || 
        current_pos_target.y != pos.y || 
        (current_pos_target.a != pos.a && turnEnd) || 
        current_use_astar != useAStar){
        LOG_INFO("New navigation target: { x = ", pos.x, " y = ", pos.y, " a = ", pos.a, " }, useAStar = ", useAStar);
        current_pos_target = pos;
        current_use_astar = useAStar;
    }
    return navigationGo();
}

void navigation_path_json(json& j){
    j = json::array();
    j.push_back({{"x", drive.position.x}, {"y", drive.position.y}});
    for (int i = 0; i < currentPathLength; i++){
        j.push_back({{"x", currentPath[i].x}, {"y", currentPath[i].y}});
    }
}

void navigationOpponentDetection(){

    return; // Disabled for now

    bool isEndangered = false;

    if (true /* TODO dir != Direction::NONE */){
        // Using the braking distance to calculate a point in front of the robot andh checking if the opponent is in the way
        double brakingDistance = 500;
        /*
        if (dir == Direction::BACKWARD){
            brakingDistance = -brakingDistance;
        }
            */
        // Check if the opponent is in the way
        isEndangered = opponent_collide_lidar(lidar.data, lidar.count, 300, brakingDistance, OPPONENT_ROBOT_RADIUS);
        if (isEndangered)
            LOG_INFO("Opponent is in the way");
        else
            LOG_DEBUG("No opponent in the way");
    }
    // stop the robot if it is endangered
    if (isEndangered && !is_robot_stalled){
        LOG_GREEN_INFO("Opponent is in the way, stopping the robot");
        is_robot_stalled = true;
        robot_stall_start_time = _millis();
    }
    else if (!isEndangered && is_robot_stalled){
        LOG_GREEN_INFO("Opponent is no longer in the way, resuming the robot");
        is_robot_stalled = false;
    }
}