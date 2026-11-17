#include "navigation/navigation.h"
#include "utils/logger.hpp"
#include "lidar/lidarAnalize.h"
#include "navigation/pathfind.h"
#include "navigation/driveControl.h"
#include "defs/tableState.hpp"
#include "actions/RobotControl.hpp"
#include "vision/Cam.hpp"

// The camera localisation is retried this many times before giving up.
// A failed calibration is not critical, so keep it short.
constexpr int kMaxCameraFixAttempts = 5;

Navigation::Navigation(RobotControl* robot, TableState* tableStatus, Cam* cam)
    : robot(robot), tableStatus(tableStatus), cam(cam){
}

NavReturn Navigation::driveStep(){
    // Calculate the path
    if (current_use_astar){
        double len;
        currentPathLength = pathfind(robot->drive.position, current_pos_target, currentPath, len);
        if (currentPathLength <= 0){
            LOG_ERROR("No path found");
            return NavReturn::Error;
        }
    }
    else{
        currentPathLength = 1;
        currentPath[0] = current_pos_target;
    }
    bool done = robot->drive.drive(currentPath, currentPathLength, current_slow_mode, current_complete_stop);
    if (done) return NavReturn::Done;
    opponentDetection(); // Check if its safe
    return NavReturn::InProcess;
}

NavReturn Navigation::go(){
    // FSM which does drive and calibration
    // TODO: NavReturn::Paused is not a return value because it wasnt used
    if (driving){
        NavReturn result = driveStep();

        if (result == NavReturn::Done){
            LOG_EXTENDED_DEBUG("Navigation drive completed");
            if (current_complete_stop){ // If came to a complete stop, calibrate using camera, else nav is done
                driving = false;
                //robot->drive.setBrakeState(true);
            }
            else {
                return NavReturn::Done;
            }
        } else if (result == NavReturn::Error){
            LOG_ERROR("Navigation drive error");
            return NavReturn::Error;
        }
    } else {
        // Calibrate using camera. An emulated camera has no fix, so skip.
        position_t camera_pos = {0.0, 0.0, 0.0};
        // If position found, update the robot position and reset the calibration age
        if (cam->getLocalisation(camera_pos)){
            camera_fix_attempts = 0;
            // Convert to the robot's frame.
            const position_t robot_pos = cameraToRobot(camera_pos);
            // Apply the new position
            robot->drive.setCoordinates(robot_pos);
            tableStatus->resetCalibrationAge();
            LOG_GREEN_INFO("Camera calibration during move successful, new position: { x = ", robot_pos.x, " y = ", robot_pos.y, " a = ", robot_pos.a, " }");
            driving = true;
            return NavReturn::Done;
        }

        camera_fix_attempts++;
        if (camera_fix_attempts >= kMaxCameraFixAttempts){
            // A failed camera fix is not fatal: the robot did reach its target, it just keeps
            // its odometry estimate. Report Done and give the next call a fresh budget.
            LOG_WARNING("Camera localisation failed ", kMaxCameraFixAttempts, " times, giving up for now");
            camera_fix_attempts = 0;
            return NavReturn::Done;
        }

        LOG_INFO("Camera localisation failed (", camera_fix_attempts, "/", kMaxCameraFixAttempts, "), retrying");
        return NavReturn::InProcess;
    }
    return NavReturn::InProcess;
}

NavReturn Navigation::goTo(position_t pos, bool useAStar, bool slow_mode, bool complete_stop){
    if (current_pos_target.x != pos.x ||
        current_pos_target.y != pos.y ||
        current_pos_target.a != pos.a ||
        current_use_astar != useAStar){
        LOG_INFO("New navigation target: { x = ", pos.x, " y = ", pos.y, " a = ", pos.a, " }, useAStar = ", useAStar);
        current_pos_target = pos;
        current_use_astar = useAStar;
        // A new target is a new occasion to localise: restart the attempts
        camera_fix_attempts = 0;
    }
    current_slow_mode = slow_mode;
    // startTime == 0 means the match clock is not armed (pre-match states): the endgame rule does not apply.
    if (forced_slow_mode || (tableStatus->startTime != 0 && _millis() - tableStatus->startTime > 90000))
        current_slow_mode = true;

    current_complete_stop = complete_stop;

    return go();
}

void Navigation::pathJson(json& j){
    j = json::array();
    j.push_back({{"x", robot->drive.position.x}, {"y", robot->drive.position.y}});
    for (int i = 0; i < currentPathLength; i++){
        j.push_back({{"x", currentPath[i].x}, {"y", currentPath[i].y}});
    }
}

void Navigation::opponentDetection(){
    bool isCloseToEnnemy = false;
    // Check if the opponent is in the way
    isCloseToEnnemy = opponent_is_close(tableStatus->pos_opponent, robot->drive.position, 800); // If opponent is closer than 800mm, we consider it close and activate slow mode

    if (isCloseToEnnemy && !forced_slow_mode){
        LOG_WARNING("Opponent is close to us, activating slow mode");
        forced_slow_mode = true;
    }else if (!isCloseToEnnemy && forced_slow_mode){
        LOG_WARNING("Opponent is no longer close to us, deactivating slow mode");
        forced_slow_mode = false;
    }
}
