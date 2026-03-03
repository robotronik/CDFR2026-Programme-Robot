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
    // ACTION PRINCIPALE *************************************************
    case FSM_ACTION_GATHER:
        ret = TakeStock();
        if (ret == FSM_RETURN_DONE){
            runState = GetBestAction(drive.position);
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
        if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("Couldn't drop");
            // TODO Handle error
        }else if (ret == FSM_RETURN_DONE){
            LOG_INFO("Finished dropping stock ", stock_num);
            runState = GetBestAction(drive.position);
        }
        break;
    //****************************************************************
    //****************************************************************

    case FSM_ACTION_CURSOR:
        ret = Cursor();
        if (ret == FSM_RETURN_DONE){
            runState = GetBestAction(drive.position);
            LOG_INFO("Finished cursor action, going to state ", runState);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("Couldn't do cursor action");
            // TODO Handle error
        }
        break;

    case FSM_ACTION_NAV_HOME:
        if (returnToHome()){
            runState = FSM_ACTION_GATHER;
            return true; // Robot is done
        }
        break;
    //*******************************************************************
    case FSM_CENTER_CALIBRATION:
        ret = GetRobotCenter();
        if(ret == FSM_RETURN_DONE){
            LOG_INFO("Finished camera calibration step ", calibrationCameraState);
            runState = FSM_ACTION_NAV_HOME;
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("Error during camera calibration step ", calibrationCameraState);
            // TODO Handle error
        }
        break;
    }
    return false;
}


ReturnFSM_t ActionFSM::TakeStock(){
    //LOG_INFO("TakeStock state: ", gatherStockState, " stock_num: ", stock_num);
    if (stock_num == -1 || gatherStockState == FSM_GATHER_NAV){
        //LOG_DEBUG("Getting next stock to take");
        if (!chooseStockStrategy(stock_num, offset)){
            LOG_INFO("No more stocks to take, exiting GatherStock");
            stock_num = -1;
            gatherStockState = FSM_GATHER_NAV;
            return FSM_RETURN_DONE;
        }
        //LOG_INFO("Next stock to take: ", stock_num, " offset: ", offset);
    }

    position_t stockPos = STOCK_POSITIONS_TABLE[stock_num];
    position_t stockOff = STOCK_OFFSETS[offset];
    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
            nav_ret = navigationGoTo(targetPos, true);
            if (nav_ret == NAV_DONE){
                //LOG_INFO("Nov Done to stock ", stock_num, "lowering claws");
                if (lowerClaws()){
                    LOG_INFO("Claws lowered for stock ", stock_num);
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
            nav_ret = navigationGoTo(position_t {stockPos.x + int(stockOff.x * 0.66), stockPos.y + int(stockOff.y * 0.66), angle}, true);
            //LOG_INFO("Moving to stock ", stock_num, " at position (", stockPos.x + int(stockOff.x * 0.7), ",", stockPos.y + int(stockOff.y * 0.7), ") with angle ", angle);
            if (nav_ret == NAV_DONE){
                gatherStockState = FSM_GATHER_COLLECT;
                LOG_INFO("Nav done FSM_GATHER_MOVE, going to FSM_GATHER_COLLECT");
            }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the stock
            if (rotateTwoBlocks()){ // TODO fermer claw puis partir sans attendre fin rotateTwoBlocks (timer)
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
            if (dropzone_num == -1) {
                LOG_ERROR("No more dropzone available, cannot drop stock ", stock_num);
                return FSM_RETURN_ERROR;
            }
            dropzonePos = getBestDropZonePosition(dropzone_num, drive.position);
            LOG_DEBUG("Dropzone position for stock ", stock_num, " is (", dropzonePos.x, ", ", dropzonePos.y, ", ", dropzonePos.a , ")");
            dropStockState = FSM_DROP_NAV;
            break;
        case FSM_DROP_NAV:
            {   
            // Navigate to dropzone
            nav_ret = navigationGoTo(dropzonePos, true);
            //LOG_INFO("Navigating to stock ", stock_num, " at position (", dropzonePos.x, ",", dropzonePos.y, ") with angle ", dropzonePos.a);

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
                    LOG_INFO("New dropzone position for stock ", stock_num, " is (", dropzonePos.x, ",", dropzonePos.y, ")");
                }

                dropStockState = FSM_DROP_NAV;
                return FSM_RETURN_WORKING;
            }
            }
            break;

        case FSM_DROP:
            // Drop the stock
            if (dropBlock()){
                LOG_INFO("Stock ", stock_num, "dropped");
                gatherStockState = FSM_GATHER_NAV;
                dropStockState = FSM_DROP_NONE;
                stock_num = -1;
                offset = 0;  
                return FSM_RETURN_DONE; 
            }
            break;
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::Cursor(){
    int sign = (tableStatus.colorTeam == BLUE) ? 1 : -1;
    position_t targetPos = {625, 1220 * sign,  45*sign };

    switch (CursorState){
        case FSM_CURSOR_NAV:
            {
            nav_ret = navigationGoTo(targetPos, true);
            if (nav_ret == NAV_DONE){
                if (lowerClaws()){
                LOG_INFO("Nav done FSM_CURSOR_NAV, going to FSM_CURSOR");
                CursorState = FSM_CURSOR_MOVE;
                }
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("Navigation error while going to cursor position");
                return FSM_RETURN_ERROR;
            }
            }
            break;
        case FSM_CURSOR_MOVE:
            nav_ret = navigationGoTo(position_t {targetPos.x, targetPos.y -sign * 330, 0}, true);
            LOG_INFO("Claws lowered at cursor position");
            if (nav_ret == NAV_DONE){
                if (raiseClaws()){
                    LOG_INFO("Nav done FSM_CURSOR_MOVE, going to FSM_CURSOR");
                    CursorState = FSM_CURSOR_END;
                }
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("Navigation error while moving to cursor position");
                return FSM_RETURN_ERROR;
            }
            
            break;

        case FSM_CURSOR_END:
            nav_ret = navigationGoTo(position_t {targetPos.x - 200, targetPos.y - sign * 280, 0}, true);
            LOG_INFO("Claws raised at cursor position");
            if (nav_ret == NAV_DONE){
                LOG_INFO("Nav done FSM_CURSOR_END, cursor action complete");
                CursorState = FSM_CURSOR_NAV;
                return FSM_RETURN_DONE;
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("Navigation error while moving to final cursor position");
                return FSM_RETURN_ERROR;
            }

    }
    return FSM_RETURN_WORKING;
}

position_t calculateClosestArucoPosition(position_t currentPos, position_t& outPos){
    outPos = currentPos;
    position_t closestPos = ARUCO_POSITIONS_TABLE[0];
    double dist20 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[0]);
    double dist21 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[1]);
    double dist22 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[2]);
    double dist23 = position_distance(currentPos, ARUCO_POSITIONS_TABLE[3]);
    double minDistance = dist20;
    if (dist21 < minDistance){
        minDistance = dist21;
        closestPos = ARUCO_POSITIONS_TABLE[1];
    }
    if (dist22 < minDistance){
        minDistance = dist22;
        closestPos = ARUCO_POSITIONS_TABLE[2];
    }
    if (dist23 < minDistance){
        minDistance = dist23;
        closestPos = ARUCO_POSITIONS_TABLE[3];
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
    outPos.a = RAD_TO_DEG * position_angle(drive.position, closestPos) + OFFSET_ANGLE_CAM;

    return closestPos;
}

ActionFSM::StateRun_t ActionFSM::GetBestAction(position_t position){
    if(runState == FSM_ACTION_GATHER){
        return FSM_ACTION_DROP;
    }
    if(runState == FSM_ACTION_DROP){
        return FSM_ACTION_GATHER;
    }
}     
ReturnFSM_t ActionFSM::Calibrate(){
    nav_return_t nav_ret;
    static unsigned long start_time;
    static position_t target_;
    static position_t arucoPos;

    switch (calibrationState){
    case FSM_CALIBRATION_NAV:
        {
        // Look towards the closest aruco marker by only spinning in place
        position_t target_;
        position_t arucoPos = calculateClosestArucoPosition(drive.position, target_);
        LOG_DEBUG("Calibrating, closest aruco marker is at (", arucoPos.x, ", ", arucoPos.y, ", ", arucoPos.a, ")");
        nav_ret = navigationGoTo(target_, true);
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
        bool cam_success;
        if (arucoCam1.getRobotPos(pos_.x, pos_.y, pos_.a, cam_success)){
            if (cam_success){
                drive.setCoordinates(pos_);
                calibrationState = FSM_CALCULATION;
                LOG_INFO("Calibrating for FSM_CALIBRATION_CALIBRATE");
            }
            else{
                LOG_WARNING("Camera did not have a good position estimate, skipping calibration");
            }
            return FSM_RETURN_DONE;
        }
        break;
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::GetRobotCenter(){
    nav_return_t nav_ret;
    static unsigned long start_time;
    static position_t aruco1;
    static position_t aruco2;
    static position_t target_ = {drive.position.x, drive.position.y, drive.position.a + 180}; // Look in the opposite direction to find the second aruco marker
    switch (calibrationCameraState){
        case FSM_ARUCO_1:
            {
            bool sucess;
            if(arucoCam1.getPos(aruco1.x, aruco1.y, aruco1.a, sucess) && sucess){
                calibrationCameraState = FSM_ARUCO_NAV;
                LOG_INFO("Found first aruco marker at (", aruco1.x, ", ", aruco1.y, ", ", aruco1.a, "), going to FSM_ARUCO_2");
            }
            }
            break;
        
        case FSM_ARUCO_2:
            {
            // Look towards the next aruco marker by only spinning in place
            bool sucess;
            if (arucoCam1.getPos(aruco2.x, aruco2.y, aruco2.a, sucess) && sucess){
                position_t center = {(aruco1.x + aruco2.x) / 2, (aruco1.y + aruco2.y) / 2, aruco2.a};
                position_t offset = {aruco1.x - center.x, aruco1.y - center.y, 0};
                LOG_INFO("Calculated offset between cam and center for aruco1: (", offset.x, ", ", offset.y, ")");
                offset = {aruco2.x - center.x, aruco2.y - center.y, 0};
                LOG_INFO("Calculated offset between cam and center for aruco2: (", offset.x, ", ", offset.y, ")");
                drive.setCoordinates(center);
                return FSM_RETURN_DONE;
            }
            }
            break;
        case FSM_ARUCO_NAV:
            {
            nav_ret = navigationGoTo(target_, true);
            if (nav_ret == NAV_DONE){
                LOG_INFO("Nav done for FSM_ARUCO_NAV, going to FSM_ARUCO_2");
                calibrationCameraState = FSM_ARUCO_2;
            }
            else if (nav_ret == NAV_ERROR){
                return FSM_RETURN_ERROR;
            }
            }
             break;
    }
    return FSM_RETURN_WORKING;
}