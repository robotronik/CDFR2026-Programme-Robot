#include "navigation/navigation.h"
#include "main.hpp"
#include "defs/constante.h" // DISTANCESTOP and DISTANCESTART
#include "utils/logger.hpp"
#include "lidar/lidarAnalize.h"
#include "navigation/pathfind.h"


static bool is_robot_stalled = false;
static unsigned long robot_stall_start_time;

static position_t currentPath[1024];
static int currentPathLenght = 0;
static int pointAlongPathIndex = 0;

nav_return_t navigationDrive(){
    if (currentPathLenght == 0 || pointAlongPathIndex >= currentPathLenght)
        return NAV_DONE;
    bool done = drive.drive(currentPath + pointAlongPathIndex, currentPathLenght - pointAlongPathIndex);

    if (done){
        pointAlongPathIndex += 1;
        if (pointAlongPathIndex >= currentPathLenght){
            // Done with navigation
            return NAV_DONE;
        }
    }
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

void fillCurrentPath(position_t path[], int pathLength) {
    memcpy(currentPath, path, sizeof(position_t) * pathLength);
    currentPathLenght = pathLength;
}

nav_return_t navigationGoTo(position_t pos, bool turnEnd, bool useAStar){
    if (useAStar){
        currentPathLenght = pathfind(drive.position, pos, currentPath);
        if (currentPathLenght == -1){
            LOG_WARNING("No path found");
            if (!is_robot_stalled){
                is_robot_stalled = true;
                robot_stall_start_time = _millis();
            }
        }
        else
            LOG_WARNING("Path found!");
        return (navigationPath(currentPath, currentPathLenght, turnEnd));
    }
    else 
        return (navigationPath(&pos, 1, turnEnd));
}

nav_return_t navigationPath(position_t path[], int pathLenght, bool turnEnd){
    bool is_same_path = memcmp(currentPath, path, sizeof(position_t) * pathLenght) == 0;
    if (is_same_path && is_robot_stalled){
        // Drive Break
        return NAV_PAUSED;
    }

    if (!is_same_path){
        for (int i = 0; i < pathLenght; i++){
            currentPath[i] = path[i];
        }
        currentPathLenght = pathLenght;
        pointAlongPathIndex = 0;
    }
    return navigationGo();
}

void navigation_path_json(json& j){
    j = json::array();
    j.push_back({{"x", drive.position.x}, {"y", drive.position.y}});
    for (int i = 0; i < currentPathLenght; i++){
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