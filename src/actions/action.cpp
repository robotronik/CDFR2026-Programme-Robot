#include <string>
#include <exception>
#include "actions/action.hpp"


ActionFSM::ActionFSM(){
    Reset();
}

ActionFSM::~ActionFSM(){}

void ActionFSM::Reset(){
    runState = FSM_ACTION_GATHER;
    gatherStockState = FSM_GATHER_NAV;
    dropStockState = FSM_DROP_NONE;
    stock_num = -1;
    offset = 0;
    
    // TODO reset other states (num,offset, etc.)
}

bool ActionFSM::RunFSM(){
    ReturnFSM_t ret;
    switch (runState)
    {
    //****************************************************************
    case FSM_ACTION_GATHER:
        ret = TakeStock();
        if (ret == FSM_RETURN_DONE){
            runState = FSM_ACTION_DROP;
            LOG_INFO("Finished gathering stock ", stock_num, ", going to FSM_ACTION_DROP");
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("Couldn't gather");
            // TODO Handle error
        }
        break;
    //****************************************************************
    case FSM_ACTION_DROP:
        ret = DropStock();
        if (ret == FSM_RETURN_DONE){
            runState = FSM_ACTION_GATHER;//TODO fct strategy choosing action
            LOG_INFO("Finished dropping stock ", stock_num, ", going to FSM_ACTION_NAV_HOME");
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("Couldn't drop");
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


ReturnFSM_t ActionFSM::TakeStock(){
    if (stock_num == -1 || gatherStockState == FSM_GATHER_NAV){
        LOG_DEBUG("Getting next stock to take");
        if (!chooseStockStrategy(stock_num, offset)){
            LOG_INFO("No more stocks to take, exiting GatherStock");
            stock_num = -1;
            gatherStockState = FSM_GATHER_NAV;
            return FSM_RETURN_DONE;
        }
        LOG_INFO("Next stock to take: ", stock_num, " offset: ", offset);
    }

    position_t stockPos = STOCK_POSITIONS_TABLE[stock_num];
    position_t stockOff = STOCK_OFFSETS[STOCK_OFFSET_MAPPING[stock_num][offset]];
    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
            LOG_INFO("Navigating to stock ", stock_num, " at position (", targetPos.x, ",", targetPos.y, ") with angle ", targetPos.a);

            nav_ret = navigationGoTo(targetPos, true);
            if (nav_ret == NAV_DONE){
                if (lowerClaws()){
                    gatherStockState = FSM_GATHER_MOVE;
                    LOG_INFO("Nav done FSM_GATHER_NAV, going to FSM_GATHER_MOVE");
                }
            }
            else if (nav_ret == NAV_ERROR){
                gatherStockState = FSM_GATHER_NAV;
                LOG_WARNING("Navigation error while going to stock ", stock_num);
                // TODO get another stock
                return FSM_RETURN_ERROR;
            }
            }
            break;

        case FSM_GATHER_MOVE:
            nav_ret = navigationGoTo(position_t {stockPos.x + int(stockOff.x * 0.9), stockPos.y + int(stockOff.y * 0.9), angle}, true);
            if (nav_ret == NAV_DONE){
                gatherStockState = FSM_GATHER_COLLECT;
                LOG_INFO("Nav done FSM_GATHER_MOVE, going to FSM_GATHER_COLLECT");
            }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the stock
            if (rotateTwoBlocks(false)){ // TODO fermer claw puis partir sans attendre fin rotateTwoBlocks (timer)
                LOG_INFO("Stock", stock_num, " collected");
                setStockAsRemoved(stock_num);
                gatherStockState = FSM_GATHER_COLLECTED;
                return FSM_RETURN_DONE;
            }
            break;
        case FSM_GATHER_COLLECTED:
            // Wait for the stock to be collected before doing anything else (like navigating to dropzone), to avoid dropping the stock on the way
            LOG_INFO("GATHER_COLLEDTED");
            return FSM_RETURN_DONE;
            break;
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::DropStock(){
    switch (dropStockState){
        case FSM_DROP_NONE:
            dropzone_num = GetBestDropZone(drive.position);
            LOG_DEBUG("best drop zone for stock ", stock_num, " is ", dropzone_num);
            dropzonePos = getBestDropZonePosition(dropzone_num, drive.position);
            LOG_DEBUG("Dropzone position for stock ", stock_num, " is (", dropzonePos.x, ", ", dropzonePos.y, ", ", dropzonePos.a * RAD_TO_DEG, ")");
            dropStockState = FSM_DROP_NAV;
            break;
        case FSM_DROP_NAV:
            {   
            // Navigate to dropzone
            nav_ret = navigationGoTo(dropzonePos, true);
            LOG_INFO("Navigating to stock ", stock_num, " at position (", dropzonePos.x, ",", dropzonePos.y, ") with angle ", dropzonePos.a);

            if (nav_ret == NAV_DONE){ // We consider that we are at the dropzone if we are close enough, to avoid navigation errors
                LOG_INFO("Nav done FSM_DROP_NAV, going to FSM_DROP");
                setDropzoneState(dropzone_num, (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_YELLOW : TableState::DROPZONE_BLUE); // Mark dropzone as occupied
                dropStockState = FSM_DROP;
            }
            else if (nav_ret == NAV_ERROR){

                LOG_WARNING("Navigation error while going to dropzone for stock ", stock_num);
                setDropzoneAsError(dropzone_num);
                
                int dropzone_temp = GetBestDropZone(drive.position);
                if(dropzone_temp == -1){
                    LOG_ERROR("No more dropzone available, cannot drop stock ", stock_num);
                    return FSM_RETURN_ERROR;
                }else{
                    setDropzoneState(dropzone_num, TableState::DROPZONE_EMPTY); // Reset previous dropzone state
                    dropzone_num = dropzone_temp;
                    dropzonePos = getBestDropZonePosition(dropzone_num, drive.position);
                    LOG_INFO("Dropzone position for stock ", stock_num, " is (", dropzonePos.x, ",", dropzonePos.y, ")");
                }

                dropStockState = FSM_DROP_NAV;
                return FSM_RETURN_ERROR;
            }
            }
            break;

        case FSM_DROP:
            // Drop the stock
            if (dropBlock()){
                LOG_INFO("Stock ", stock_num, "dropped");
                gatherStockState = FSM_GATHER_NAV;
                dropStockState = FSM_DROP_NONE;
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