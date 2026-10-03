#include "navigation/navigation.h"
#include "utils/logger.hpp"
#include "lidar/lidarAnalize.h"
#include "navigation/pathfind.h"
#include "navigation/driveControl.h"
#include "defs/tableState.hpp"
#include "vision/ArucoCam.hpp"

Navigation::Navigation(DriveControl* drive, TableState* tableStatus, ArucoCam* arucoCam)
    : drive(drive), tableStatus(tableStatus), arucoCam(arucoCam){
}

nav_return_t Navigation::driveStep(){
    // Calculate the path
    if (current_use_astar){
        double len;
        currentPathLength = pathfind(drive->position, current_pos_target, currentPath, len);
        if (currentPathLength <= 0){
            LOG_ERROR("No path found");
            return NAV_ERROR;
        }
    }
    else{
        currentPathLength = 1;
        currentPath[0] = current_pos_target;
    }
    bool done = drive->drive(currentPath, currentPathLength, current_slow_mode, current_complete_stop);
    if (done) return NAV_DONE;
    opponentDetection(); // Check if its safe
    return NAV_IN_PROCESS;
}

nav_return_t Navigation::go(){
    // FSM which does drive and calibration
    if (driving){
        nav_return_t result = driveStep();

        double dist = position_distance(drive->position, current_pos_target);
        double move = position_distance(drive->position, last_pos);
        if (dist > 20 && move < 4){
            if (stuck_start == 0) stuck_start = _millis();
            if (_millis() - stuck_start > 500.0)
                LOG_WARNING("NAV: Robot might be stuck, distance to target: ", dist, "mm, movement since last check: ", move, "mm, time stuck: ", _millis() - stuck_start, "ms");

            if (_millis() - stuck_start > 1600){
                LOG_ERROR("NAV: Robot stuck");
                drive->stopMotion();
                stuck_start = 0;
                return NAV_ERROR;
            }
        }else{
            stuck_start = 0;
        }

        last_pos = drive->position;

        if (result == NAV_DONE){
            LOG_EXTENDED_DEBUG("Navigation drive completed");
            if (current_complete_stop){ // If came to a complete stop, calibrate using camera, else nav is done
                driving = false;
                drive->setBrakeState(true);
            }
            else {
                stuck_start = 0;
                prev_final_pos_otos = drive->position;
                return NAV_DONE;
            }
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
        // Calibrate using camera. An emulated camera has no fix, so skip.
        position_t camera_pos = {0.0, 0.0, 0.0};
        const bool localised = arucoCam->getLocalisation(camera_pos);
        if (localised || arucoCam->isEmulated()){
            if (localised){
                // The camera is not the robot: convert to the robot's frame.
                const position_t robot_pos = cameraToRobot(camera_pos);
                prev_final_pos_cam = robot_pos;
                prev_final_pos_otos = drive->position;
                drive->setCoordinates(robot_pos);
                tableStatus->resetCalibrationAge();
                LOG_GREEN_INFO("Camera calibration during move successful, new position: { x = ", robot_pos.x, " y = ", robot_pos.y, " a = ", robot_pos.a, " }");
            }
            else{
                LOG_EXTENDED_DEBUG("Camera emulated, skipping calibration");
            }
            driving = true;
            drive->setBrakeState(false);
            stuck_start = 0;
            return NAV_DONE;
        }
    }
    return NAV_IN_PROCESS;
}

nav_return_t Navigation::goTo(position_t pos, bool useAStar, bool slow_mode, bool complete_stop){
    if (current_pos_target.x != pos.x ||
        current_pos_target.y != pos.y ||
        current_pos_target.a != pos.a ||
        current_use_astar != useAStar){
        LOG_INFO("New navigation target: { x = ", pos.x, " y = ", pos.y, " a = ", pos.a, " }, useAStar = ", useAStar);
        current_pos_target = pos;
        current_use_astar = useAStar;
        stuck_start = 0;
    }
    current_slow_mode = slow_mode;
    if (forced_slow_mode || (_millis() - tableStatus->startTime > 90000))
        current_slow_mode = true;

    current_complete_stop = complete_stop;

    return go();
}

void Navigation::pathJson(json& j){
    j = json::array();
    j.push_back({{"x", drive->position.x}, {"y", drive->position.y}});
    for (int i = 0; i < currentPathLength; i++){
        j.push_back({{"x", currentPath[i].x}, {"y", currentPath[i].y}});
    }
}

void Navigation::opponentDetection(){
    bool isCloseToEnnemy = false;
    // Check if the opponent is in the way
    isCloseToEnnemy = opponent_is_close(tableStatus->pos_opponent, drive->position, 800); // If opponent is closer than 800mm, we consider it close and activate slow mode

    if (isCloseToEnnemy && !forced_slow_mode){
        LOG_WARNING("Opponent is close to us, activating slow mode");
        forced_slow_mode = true;
    }else if (!isCloseToEnnemy && forced_slow_mode){
        LOG_WARNING("Opponent is no longer close to us, deactivating slow mode");
        forced_slow_mode = false;
    }
}
