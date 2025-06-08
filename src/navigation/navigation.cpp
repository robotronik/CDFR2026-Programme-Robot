#include <functional> // For std::hash
#include "navigation/navigation.h"
#include "main.hpp"
#include "defs/constante.h" // DISTANCESTOP and DISTANCESTART
#include "utils/logger.hpp"
#include "lidar/lidarAnalize.h"


static bool is_robot_stalled = false;
static unsigned long robot_stall_start_time;
typedef std::size_t nav_hash;
static nav_hash currentInstructionHash;

static position_t currentPath[512];
static int currentPathLenght = 0;

nav_hash createHash(position_t pos);

void fillCurrentPath(position_t path[], int pathLength) {
    memcpy(currentPath, path, sizeof(position_t) * pathLength);
    currentPathLenght = pathLength;
}

nav_return_t navigationGoTo(position_t pos, bool turnEnd, bool useAStar){
    if (useAStar){
        /*
        currentPathLenght = AStar_calculate(drive.position, pos, currentPath);
        if (currentPathLenght == -1)
            LOG_WARNING("No path found");
        return (navigationPath(currentPath, currentPathLenght));
        */
       return NAV_IN_PROCESS;
    }
    else 
        return (navigationPath(&pos, 1));
}


nav_return_t navigationPath(position_t path[], int pathLenght, bool turnEnd){
    nav_hash hashValue = 0;
    for (int i = 0; i < pathLenght; i++){
        hashValue += createHash(path[i]);
    }
    nav_return_t ireturn = NAV_IN_PROCESS;
    if (hashValue == currentInstructionHash && is_robot_stalled){
        drive.drive(&drive.position, 1);
        return NAV_PAUSED;
    }

    if (hashValue != currentInstructionHash){
        for (int i = 0; i < pathLenght; i++){
            currentPath[i] = path[i];
        }
        currentPathLenght = pathLenght;
        currentInstructionHash = hashValue;
    }
    return drive.drive(currentPath, currentPathLenght) ? NAV_DONE : NAV_IN_PROCESS;
}

void navigation_path_json(json& j){
    j = json::array();
    j.push_back({{"x", drive.position.x}, {"y", drive.position.y}});
    for (int i = 0; i < currentPathLenght; i++){
        j.push_back({{"x", currentPath[i].x}, {"y", currentPath[i].y}});
    }
}

void navigationOpponentDetection(){

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

// Function to calculate the hash of the navigation instruction
nav_hash createHash(position_t pos) {
    // Combine all values simply by adding their hashes
    return std::hash<int>{}(pos.x) + std::hash<int>{}(pos.y) + std::hash<int>{}(pos.a);
}
