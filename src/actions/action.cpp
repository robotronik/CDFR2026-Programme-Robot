#include <string>
#include <exception>
#include "actions/action.hpp"
#include "utils/logger.hpp"
#include "main.hpp"
#include "navigation/navigation.h"
#include "actions/functions.h"
#include "actions/strats.hpp"
#include "defs/tableState.hpp"
#include "defs/constante.h"

ActionFSM::ActionFSM(){
    Reset();
}

ActionFSM::~ActionFSM(){}

void ActionFSM::Reset(){
    runState = FSM_ACTION_GATHER;
    gatherStockState = FSM_GATHER_NAV;
}

bool ActionFSM::RunFSM(){
    ReturnFSM_t ret;
    switch (runState)
    {
    //****************************************************************
    case FSM_ACTION_GATHER:
        ret = GatherStock();
        if (ret == FSM_RETURN_DONE)
            runState = FSM_ACTION_NAV_HOME;
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("Couldn't gather");
            // TODO Handle error
        }
        break;
    //****************************************************************
    case FSM_ACTION_NAV_HOME:
        if (returnToHome()){
            runState = FSM_ACTION_GATHER;
            return true; // Robot is done
        }
        break;
    }
    return false;
}


ReturnFSM_t ActionFSM::GatherStock(){
    nav_return_t nav_ret;
    switch (gatherStockState){
    case FSM_GATHER_NAV:
        // TODO Astarts should be enabled for some takes
        position_t pos;
        nav_ret = navigationGoTo(pos, false);
        if (nav_ret == NAV_DONE){
            gatherStockState = FSM_GATHER_COLLECT;
            LOG_INFO("Nav done and RevolverPrepareLowBarrel done for FSM_GATHER_NAV, going to FSM_GATHER_MOVE");
        }
        else if (nav_ret == NAV_ERROR){
            // TODO get another stock
            return FSM_RETURN_ERROR;
        }
        break;
    case FSM_GATHER_COLLECT:
        // Collect the stock
        if (true){ //takeStockPlatforms()
            gatherStockState = FSM_GATHER_NAV;
            LOG_INFO("taking stock for FSM_GATHER_COLLECT");
            return FSM_RETURN_DONE;
        }
        break;
    }
    return FSM_RETURN_WORKING;
}

position_t calculateClosestArucoPosition(position_t currentPos, position_t& outPos){
    outPos = currentPos;
    position_t arucoPos_20 = {-400.0, -900.0, 0.0};
    position_t arucoPos_21 = {-400.0, 900.0, 0.0};
    position_t arucoPos_22 = {400.0, -900.0, 0.0};
    position_t arucoPos_23 = {400.0, 900.0, 0.0};
    position_t closestPos = arucoPos_20;
    double dist20 = position_distance(currentPos, arucoPos_20);
    double dist21 = position_distance(currentPos, arucoPos_21);
    double dist22 = position_distance(currentPos, arucoPos_22);
    double dist23 = position_distance(currentPos, arucoPos_23);
    double minDistance = dist20;
    if (dist21 < minDistance){
        minDistance = dist21;
        closestPos = arucoPos_21;
    }
    if (dist22 < minDistance){
        minDistance = dist22;
        closestPos = arucoPos_22;
    }
    if (dist23 < minDistance){
        minDistance = dist23;
        closestPos = arucoPos_23;
    }
    // Check if we are above the aruco marker
    const double minimal_distance = 200.0; // 20cm
    if (minDistance < minimal_distance){
        LOG_WARNING("Above aruco marker, need to move away first");
        double displacement = minimal_distance - minDistance + 1; //+margin
        position_t tmp = position_vector(closestPos, currentPos);
        position_normalize(tmp);
        tmp.x *= displacement;
        tmp.y *= displacement;
        outPos.x += tmp.x;
        outPos.y += tmp.y;
    }
    outPos.a = position_angle(drive.position, closestPos);

    return closestPos;
}

ReturnFSM_t ActionFSM::Calibrate(){
    nav_return_t nav_ret;
    static unsigned long start_time;
    switch (calibrationState){
    case FSM_CALIBRATION_NAV:
    {
        // Look towards the closest aruco marker by only spinning in place
        position_t target_;
        position_t arucoPos = calculateClosestArucoPosition(drive.position, target_);
        nav_ret = navigationGoTo(target_, true, true);
        if (nav_ret == NAV_DONE){
            calibrationState = FSM_CALIBRATION_CALIBRATE;
            LOG_INFO("Nav done for FSM_CALIBRATION_NAV, going to FSM_CALIBRATION_CALIBRATE");
            start_time = _millis();
        }
        else if (nav_ret == NAV_ERROR){
            return FSM_RETURN_ERROR;
        }
    }
    break;
    
    case FSM_CALIBRATION_CALIBRATE:
        // Calibrate
        if (_millis() > start_time + 1000){ // Timeout after 1s
            LOG_ERROR("Calibration timeout");
            return FSM_RETURN_ERROR;
        }
        position_t pos_;
        if (arucoCam1.getPos(pos_.x, pos_.y, pos_.a)){
            drive.setCoordinates(pos_);
            calibrationState = FSM_CALIBRATION_NAV;
            LOG_INFO("Calibrating for FSM_CALIBRATION_CALIBRATE");
            return FSM_RETURN_DONE;
        }
    break;
    }
    return FSM_RETURN_WORKING;
}