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

nav_hash createHash(position_t pos, Direction direction, Rotation rotationLookAt, Rotation rotation);

nav_return_t navigationGoTo(position_t pos, Direction direction, Rotation rotationLookAt, Rotation rotation){
    return (navigationPath(&pos, 1, direction, rotationLookAt, rotation));
}

nav_return_t navigationGoToNoTurn(position_t pos, Direction direction, Rotation rotationLookAt){
    return navigationPathNoTurn(&pos, 1, direction, rotationLookAt);
}


nav_return_t navigationPath(position_t path[], int pathLenght, Direction direction, Rotation rotationLookAt, Rotation rotation){
    nav_hash hashValue = createHash({0, 0, 0}, direction, rotationLookAt, (Rotation)0);
    for (int i = 0; i < pathLenght; i++){
        hashValue += createHash(path[i], direction, rotationLookAt, (Rotation)0);
    }
    nav_return_t ireturn = NAV_IN_PROCESS;
    if (hashValue == currentInstructionHash && is_robot_stalled){
        return NAV_PAUSED;
    }

    if (hashValue != currentInstructionHash){
        LOG_DEBUG("Following path");

        //  TODO
        for (int i = 0; i < pathLenght; i++){
            asserv.go_to_point(path[i].x, path[i].y, i == 0 ? rotationLookAt : Rotation::SHORTEST, direction);


            currentPath[i] = path[i];
        }
        asserv.consigne_angulaire(path[pathLenght - 1].a, rotationLookAt);


        currentPathLenght = pathLenght;
        currentInstructionHash = hashValue;
    }
    else{
        ireturn = asserv.get_moving_is_done()
                     ? NAV_DONE : NAV_IN_PROCESS;
    }
    return ireturn;
}

nav_return_t navigationPathNoTurn(position_t path[], int pathLenght, Direction direction, Rotation rotationLookAt){
    nav_hash hashValue = createHash({0, 0, 0}, direction, rotationLookAt, (Rotation)0);
    for (int i = 0; i < pathLenght; i++){
        hashValue += createHash(path[i], direction, rotationLookAt, (Rotation)0);
    }
    nav_return_t ireturn = NAV_IN_PROCESS;
    if (hashValue == currentInstructionHash && is_robot_stalled){
        return NAV_PAUSED;
    }

    if (hashValue != currentInstructionHash){
        LOG_DEBUG("Following path");
        
        // TODO
        for (int i = 0; i < pathLenght; i++){
            asserv.go_to_point(path[i].x, path[i].y, i == 0 ? rotationLookAt : Rotation::SHORTEST, direction);

            currentPath[i] = path[i];
        }


        currentInstructionHash = hashValue;
        currentPathLenght = pathLenght;
    }
    else{
        ireturn = asserv.get_moving_is_done()
                     ? NAV_DONE : NAV_IN_PROCESS;
    }
    return ireturn;
}

void navigation_path_json(json& j){
    j = json::array();
    j.push_back({{"x", tableStatus.robot.pos.x}, {"y", tableStatus.robot.pos.y}});
    for (int i = 0; i < currentPathLenght; i++){
        j.push_back({{"x", currentPath[i].x}, {"y", currentPath[i].y}});
    }
}

void navigationOpponentDetection(){

    Direction dir = (Direction)(tableStatus.robot.direction_side);
    bool isEndangered = false;

    if (dir != Direction::NONE){
        // Using the braking distance to calculate a point in front of the robot andh checking if the opponent is in the way
        double brakingDistance = 500;
        if (dir == Direction::BACKWARD){
            brakingDistance = -brakingDistance;
        }
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
        asserv.pause();
        is_robot_stalled = true;
        robot_stall_start_time = _millis();
    }
    else if (!isEndangered && is_robot_stalled){
        LOG_GREEN_INFO("Opponent is no longer in the way, resuming the robot");
        asserv.resume();
        is_robot_stalled = false;
    }
}

// Function to calculate the hash of the navigation instruction
nav_hash createHash(position_t pos, Direction direction, Rotation rotationLookAt, Rotation rotation) {
    // Combine all values simply by adding their hashes
    return std::hash<int>{}(pos.x) ^
           std::hash<int>{}(pos.y) ^
           std::hash<int>{}(pos.a) ^
           std::hash<int>{}(static_cast<int>(direction)) ^
           std::hash<int>{}(static_cast<int>(rotationLookAt)) ^
           std::hash<int>{}(static_cast<int>(rotation));
}
