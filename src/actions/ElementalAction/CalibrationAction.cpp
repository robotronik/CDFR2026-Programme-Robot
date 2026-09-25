#include <math.h>
#include "actions/ElementalAction/CalibrationAction.hpp"
#include "utils/logger.hpp"
#include "navigation/pathfind.h"
#include "defs/constante.h"
#include "main.hpp" // for tableStatus, drive

CalibrationAction::CalibrationAction(){
    nom = "Calibration";
    duree = 0;
    calibrationState = FSM_CALCULATION;
}

bool CalibrationAction::stop(){
    return true;
}

void CalibrationAction::reset(){
    calibrationState = FSM_CALCULATION;
}

ReturnFSM_t CalibrationAction::run(){
    switch (calibrationState){
        case FSM_CALCULATION:
            calibrationTarget_ = calculateClosestArucoPosition(drive.position);
            calibrationState = FSM_CALIBRATION_NAV;
            break;
        case FSM_CALIBRATION_NAV:
        {
            // Look towards the closest aruco marker to recalibrate the position
            nav_ret = navigationGoTo(calibrationTarget_, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_CALIBRATION_NAV: Nav done");
                if (tableStatus.calibrationAge){
                    // Si calibrationAge != 0 la calibration a échoué
                    errorManagement(NAV_DONE);
                    return FSM_RETURN_ERROR;
                }
                return FSM_RETURN_DONE;
            }
            else if (nav_ret == NAV_ERROR){
                errorManagement(NAV_ERROR);
                return FSM_RETURN_ERROR;
            }
            break;
        }
    }
    return FSM_RETURN_WORKING;
}

bool CalibrationAction::errorManagement(nav_return_t error_code){
    tableStatus.calibrationAge -= 1;
    if (error_code == NAV_DONE){
        LOG_WARNING("ACTION_CALIBRATION: Failed nav to calibration action");
    }else if (error_code == NAV_ERROR)
    {
        LOG_WARNING("ACTION_CALIBRATION: Failed calibration action on aruco scan");
    }
    return true;
}

bool CalibrationAction::successManagement(){
    LOG_GREEN_INFO("ACTION_CALIBRATION: calibration on aruco tag sucess");
    return true;
}

float CalibrationAction::available(){
    return 0.0f;
}

bool CalibrationAction::fullBlock(){
    return true;
}

bool CalibrationAction::mouvementBlock(){
    return true;
}

// Return the closest position to look at an aruco marker
position_t CalibrationAction::calculateClosestArucoPosition(position_t currentPos){
    position_t outPos = currentPos;
    position_t closestPos = ARUCO_POSITIONS_TABLE[0];
    double minDistance = position_distance(currentPos, ARUCO_POSITIONS_TABLE[0]);
    for (int i = 1; i < 4; i++){
        double d = position_distance(currentPos, ARUCO_POSITIONS_TABLE[i]);
        if (d < minDistance){
            minDistance = d;
            closestPos = ARUCO_POSITIONS_TABLE[i];
        }
    }
    LOG_ERROR("Distance to closest aruco marker: ", minDistance);
    const double target_distance_min = 350.0; // mm
    const double target_distance_max = 550.0; // mm

    if (minDistance < target_distance_min || minDistance > target_distance_max){
        LOG_WARNING("Not in valid range, moving to preset position");
        // Calculate the closest valid position using predetermined pos
        outPos = ARUCO_CALIB_POSITIONS[0];
        double minTargetDistance = 1e6;
        for (int i = 0; i < ARUCO_CALIB_POSITIONS_COUNT; i++){
            // Blue side
            double d = position_distance(currentPos, ARUCO_CALIB_POSITIONS[i]);
            if (d < minTargetDistance){
                minTargetDistance = d;
                outPos = ARUCO_CALIB_POSITIONS[i];
            }
            // Yellow side (mirrored)
            position_t mirroredPos = ARUCO_CALIB_POSITIONS[i];
            mirroredPos.y = -mirroredPos.y;
            d = position_distance(currentPos, mirroredPos);
            if (d < minTargetDistance){
                minTargetDistance = d;
                outPos = mirroredPos;
            }
        }
    }
    else {
        LOG_DEBUG("Good distance → no movement");
    }
    outPos.a = RAD_TO_DEG * position_angle(outPos, closestPos) + OFFSET_CAM_A;

    return outPos;
}