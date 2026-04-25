#include "navigation/navigation.h"
#include "main.hpp"
#include "defs/constante.h" // DISTANCESTOP and DISTANCESTART
#include "utils/logger.hpp"
#include "lidar/lidarAnalize.h"
#include "navigation/pathfind.h"

static bool is_robot_stalled = false;  // Because of opponent in direction of movement
static bool is_robot_stuck = false;  // Because of no path found
static unsigned long robot_stall_start_time;
static unsigned long robot_stuck_start_time;
bool forced_slow_mode = false;

static position_t current_pos_target;
static bool current_use_astar;
static bool current_slow_mode;
static bool current_complete_stop = true;

static position_t currentPath[1024];
static int currentPathLength = 0;

void navigationOpponentDetection();

nav_return_t navigationDrive(){
    // Calculate the path
    if (current_use_astar){
        double len;
        currentPathLength = pathfind(drive.position, current_pos_target, currentPath, len);
        //LOG_GREEN_INFO("Path length: ", len);
        if (currentPathLength <= 0){
            LOG_ERROR("No path found");
            return NAV_ERROR;
        }
        else {
            // LOG_GREEN_INFO("Path found!");
        }
    }
    else{
        currentPathLength = 1;
        currentPath[0] = current_pos_target;
    }
    bool done = drive.drive(currentPath, currentPathLength, current_slow_mode, current_complete_stop);
    if (done) return NAV_DONE;
    navigationOpponentDetection(); // Check if its safe
    return NAV_IN_PROCESS;
}

nav_return_t navigationGo(){
    // FSM which does drive and calibration
    static bool driving = true;
    static position_t last_pos = {0,0,0};
    static unsigned long stuck_start = 0;
    if (driving){
        nav_return_t result = navigationDrive();

        double dist = position_distance(drive.position, current_pos_target);
        double move = position_distance(drive.position, last_pos);
        //LOG_DEBUG("NAV: Distance to target: ", dist, "mm, movement since last check: ", move, "mm");
        if (dist > 10 && move < 4){
            if (stuck_start == 0) stuck_start = _millis();
            if (_millis() - stuck_start > 200.0)
                LOG_WARNING("NAV: Robot might be stuck, distance to target: ", dist, "mm, movement since last check: ", move, "mm, time stuck: ", _millis() - stuck_start, "ms");
            

            if (_millis() - stuck_start > 1000){
                LOG_ERROR("NAV: Robot stuck");
                stuck_start = 0;
                return NAV_ERROR;
            }
        }else{
            stuck_start = 0;
        }

        last_pos = drive.position;

        if (result == NAV_DONE){
            LOG_EXTENDED_DEBUG("Navigation drive completed");
            if (current_complete_stop) // If came to a complete stop, calibrate using camera, else nav is done
                driving = false;
            else
                return NAV_DONE;
        } else if (result == NAV_ERROR){
            LOG_ERROR("Navigation drive error");
            return NAV_ERROR;
        }
        if (is_robot_stalled && (_millis() - robot_stall_start_time > 1000)){
            LOG_WARNING("Robot has been stalled for more than 1 second, returning NAV_ERROR");
            return NAV_ERROR; // We are stuck for too long
        }
        else if (is_robot_stalled)
            return NAV_PAUSED;
    } else {
        // Calibrate using camera
        bool cam_success;
        position_t robot_pos;
        if (arucoCam1.getRobotPos(robot_pos.x, robot_pos.y, robot_pos.a, cam_success)){
            if (cam_success){
                drive.setCoordinates(robot_pos);
                tableStatus.resetCalibrationAge();
                LOG_GREEN_INFO("Camera calibration during move successful, new position: { x = ", robot_pos.x, " y = ", robot_pos.y, " a = ", robot_pos.a, " }");
            }
            else{
                LOG_EXTENDED_DEBUG("Camera did not have a good position estimate, skipping calibration");
            }
            driving = true;
            return NAV_DONE;
        }
    }
    return NAV_IN_PROCESS;
}

nav_return_t navigationGoTo(position_t pos, bool useAStar, bool slow_mode, bool complete_stop){
    if (current_pos_target.x != pos.x || 
        current_pos_target.y != pos.y || 
        current_pos_target.a != pos.a || 
        current_use_astar != useAStar){
        LOG_INFO("New navigation target: { x = ", pos.x, " y = ", pos.y, " a = ", pos.a, " }, useAStar = ", useAStar);
        current_pos_target = pos;
        current_use_astar = useAStar;
    }
    current_slow_mode = slow_mode;
    if (forced_slow_mode)
        current_slow_mode = true;
    
    current_complete_stop = complete_stop;

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
    bool isCloseToEnnemy = false;
    // Check if the opponent is in the way
    isCloseToEnnemy = opponent_is_close(tableStatus.pos_opponent, drive.position, 800); // If opponent is closer than 800mm, we consider it close and activate slow mode

    if (isCloseToEnnemy && !forced_slow_mode){
        LOG_WARNING("Opponent is close to us, activating slow mode");
        forced_slow_mode = true;
    }else if (!isCloseToEnnemy && forced_slow_mode){
        LOG_WARNING("Opponent is no longer close to us, deactivating slow mode");
        forced_slow_mode = false;
    }
}