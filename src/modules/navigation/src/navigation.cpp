#include "navigation/navigation.h"
#include "utils/logger.hpp"
#include "lidar/lidarAnalize.h"
#include "navigation/pathfind.h"
#include "navigation/driveControl.h"
#include "defs/tableState.hpp"
#include "vision/Cam.hpp"

Navigation::Navigation(DriveControl* drive, TableState* tableStatus, Cam* cam)
    : drive(drive), tableStatus(tableStatus), cam(cam) {
}

NavReturn Navigation::driveStep() {
    // Calculate the path
    if (current_use_astar) {
        double len;
        currentPathLength = pathfind(drive->position, current_pos_target, currentPath, len);
        if (currentPathLength <= 0) {
            LOG_ERROR("No path found");
            return NavReturn::Error;
        }
    } else {
        currentPathLength = 1;
        currentPath[0] = current_pos_target;
    }
    bool done = drive->drive(currentPath, currentPathLength, current_slow_mode, current_complete_stop);
    if (done) return NavReturn::Done;
    opponentDetection(); // Check if its safe
    return NavReturn::InProcess;
}

NavReturn Navigation::go() {
    // FSM which does drive and calibration
    // TODO: NavReturn::Paused is not a return value because it wasnt used
    if (driving) {
        NavReturn result = driveStep();

        if (result == NavReturn::Done) {
            LOG_EXTENDED_DEBUG("Navigation drive completed");
            if (current_complete_stop) { // If came to a complete stop, calibrate using camera, else nav is done
                driving = false;
                //drive->setBrakeState(true);
            } else {
                return NavReturn::Done;
            }
        } else if (result == NavReturn::Error) {
            LOG_ERROR("Navigation drive error");
            return NavReturn::Error;
        }
    } else {
        // Single camera fix attempt. A failure is not fatal: the robot did reach its target,
        // it just keeps its odometry estimate and the caller can ask again.
        position_t camera_pos = {0.0, 0.0, 0.0};
        if (cam->getLocalisation(camera_pos)) {
            // Convert to the robot's frame and apply the new position
            const position_t robot_pos = cameraToRobot(camera_pos);
            drive->setCoordinates(robot_pos);
            tableStatus->resetCalibrationAge();
            LOG_GREEN_INFO("Camera calibration during move successful, new position: { x = ", robot_pos.x, " y = ", robot_pos.y, " a = ", robot_pos.a, " }");
        } else {
            LOG_WARNING("Camera localisation failed, keeping the odometry estimate");
        }
        driving = true;
        return NavReturn::Done;
    }
    return NavReturn::InProcess;
}

NavReturn Navigation::goTo(position_t pos, bool useAStar, bool slow_mode, bool complete_stop) {
    if (current_pos_target.x != pos.x ||
        current_pos_target.y != pos.y ||
        current_pos_target.a != pos.a ||
        current_use_astar != useAStar) {
        LOG_INFO("New navigation target: { x = ", pos.x, " y = ", pos.y, " a = ", pos.a, " }, useAStar = ", useAStar);
        current_pos_target = pos;
        current_use_astar = useAStar;
    }
    current_slow_mode = slow_mode;
    if (forced_slow_mode)
        current_slow_mode = true;
    current_complete_stop = complete_stop;

    return go();
}

void Navigation::pathJson(json& j) {
    j = json::array();
    j.push_back({{"x", drive->position.x}, {"y", drive->position.y}});
    for (int i = 0; i < currentPathLength; i++) {
        j.push_back({{"x", currentPath[i].x}, {"y", currentPath[i].y}});
    }
}

void Navigation::opponentDetection() {
    bool isCloseToOpponent = false;
    // Check if the opponent is in the way
    isCloseToOpponent = opponent_is_close(tableStatus->pos_opponent, drive->position, 800); // If opponent is closer than 800mm, we consider it close and activate slow mode

    if (isCloseToOpponent && !forced_slow_mode) {
        LOG_WARNING("Opponent is close to us, activating slow mode");
        forced_slow_mode = true;
    } else if (!isCloseToOpponent && forced_slow_mode) {
        LOG_WARNING("Opponent is no longer close to us, deactivating slow mode");
        forced_slow_mode = false;
    }
}
