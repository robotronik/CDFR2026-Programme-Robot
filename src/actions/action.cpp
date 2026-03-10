#include <string>
#include <exception>
#include "actions/action.hpp"
#include "actions/functions.h"
#include "actions/strats.hpp"
#include "defs/tableState.hpp"
#include "utils/logger.hpp"
#include "main.hpp"
#include "defs/constante.h"


ActionFSM::ActionFSM(){
    Reset();
}

ActionFSM::~ActionFSM(){}


void ActionFSM::Reset(){
    runState = FSM_ACTION_CALIBRATION;

    /****** RESET OF FSM STATES *******/
    gatherStockState = FSM_GATHER_NAV;
    dropStockState = FSM_DROP_NONE;
    CursorState = FSM_CURSOR_NAV;
    calibrationCameraState = FSM_ARUCO_1;
    calibrationState = FSM_CALCULATION;

    /*RESET OF ACTION ID*/
    dropzone_num = 0;
    stock_num = -1;
    offset = 0;
    targetStockPos = position_t{0,0,0};
    dropzonePos = position_t{0,0,0};
    CursorPos = {625, 1220, 45};
    if (tableStatus.colorTeam == YELLOW) position_robot_flip(CursorPos);

    stockOrder[0] = false; stockOrder[1]=false; stockOrder[2]=false; stockOrder[3]=false;
    setCursorIsDone(false);
    
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
            SetBestAction(drive.position);
            LOG_INFO("ACTION_GATHER: Finished gathering stock ", stock_num);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_GATHER: Couldn't gather");
            // TODO Handle error
        }
        break;
    //****************************************************************
    case FSM_ACTION_DROP:
        ret = DropStock();
        if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_DROP: Couldn't drop");
            // TODO Handle error
        }else if (ret == FSM_RETURN_DONE){
            LOG_INFO("ACTION_DROP: Finished dropping stock ", stock_num);
            SetBestAction(drive.position);
        }
        break;
    //****************************************************************
    //****************************************************************

    case FSM_ACTION_CURSOR:
        ret = Cursor();
        if (ret == FSM_RETURN_DONE){
            setCursorIsDone(true);
            SetBestAction(drive.position);
            LOG_INFO("ACTION_CURSOR: Finished cursor action");
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_CURSOR: Couldn't do cursor action");
            // TODO Handle error
        }
        break;

    case FSM_ACTION_NAV_HOME:
        if (returnToHome()){
            LOG_INFO("ACTION_NAV_HOME: Finished going home");
            runState = FSM_ACTION_GATHER;
            return true; // Robot is done
        }
        break;
    //*******************************************************************
    case FSM_ACTION_CALIBRATION:
        ret = Calibrate();
        if (ret == FSM_RETURN_DONE){
            tableStatus.resetCalibrationAge();
            SetBestAction(drive.position);
            LOG_INFO("ACTION_CALIBRATION: Finished calibration action");
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_CALIBRATION: Couldn't do calibration action");
            // TODO Handle error
        }
        break;

    case FSM_CENTER_CALIBRATION:
        ret = GetRobotCenter();
        if(ret == FSM_RETURN_DONE){
            LOG_INFO("CENTER_CALIBRATION: Finished center calibration step ", calibrationCameraState);
            runState = FSM_ACTION_NAV_HOME;
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("CENTER_CALIBRATION: Error during center calibration step ", calibrationCameraState);
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
            LOG_WARNING("ACTION_GATHER: No more stocks to take, exiting GatherStock");
            stock_num = -1;
            gatherStockState = FSM_GATHER_NAV;
            runState = FSM_ACTION_NAV_HOME; // si plus de stock, on return home
            return FSM_RETURN_DONE;
        }
        LOG_EXTENDED_DEBUG("ACTION_GATHER: Next stock to take: ", stock_num, " offset: ", offset);
    }

    position_t stockPos = STOCK_POSITIONS_TABLE[stock_num];
    position_t stockOff = STOCK_OFFSETS[offset];

    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
            nav_ret = navigationGoTo(targetPos, true, true); // Enabeling A*
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_GATHER_NAV: moved to stock now searching for blocks");
                gatherStockState = FSM_GATHER_DETECT;
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_GATHER_NAV: Navigation error while going to stock ", stock_num);
                stock_num = -1;
                gatherStockState = FSM_GATHER_NAV; // TO TEST avec adversaire 
                return FSM_RETURN_ERROR;
            }
            }
            break;
        case FSM_GATHER_DETECT:
            {
            double x = drive.position.x;
            double y = drive.position.y;
            double a = drive.position.a;
            bool sucess = false;

            if(arucoCam1.getObjectInfoColors(stockOrder,x,y,a,sucess)){
                if(sucess){
                    LOG_EXTENDED_DEBUG("FSM_GATHER_DETECT: Detection sucess calibration on blocks");
                    targetStockPos = position_t{x + int(stockOff.x * 0.66), y + int(stockOff.y * 0.66), angle};
                }else{
                    //TODO handle error
                    LOG_WARNING("FSM_GATHER_DETECT: Detection failed calibration on map");
                    targetStockPos = position_t{stockPos.x + int(stockOff.x * 0.66), stockPos.y + int(stockOff.y * 0.66), angle};
                }
                gatherStockState = FSM_GATHER_CLAWS;
            }
            }
            break;
        case FSM_GATHER_CLAWS:
            if (snapClaws(false,false) && lowerClaws()){
                LOG_EXTENDED_DEBUG("FSM_GATHER_CLAWS: Claws lowered and snap for stock ", stock_num);
                gatherStockState = FSM_GATHER_MOVE;
            }
            break;
        case FSM_GATHER_MOVE:
            {
            
            nav_ret = navigationGoTo(targetStockPos, true);
            //LOG_INFO("Moving to stock ", stock_num, " at position (", stockPos.x + int(stockOff.x * 0.7), ",", stockPos.y + int(stockOff.y * 0.7), ") with angle ", angle);
            if (nav_ret == NAV_DONE){
                gatherStockState = FSM_GATHER_COLLECT;
                LOG_EXTENDED_DEBUG("FSM_GATHER_MOVE: Moving to stock ", stock_num, " at position (", targetStockPos.x, ",", targetStockPos.y, ") with angle ", targetStockPos.a);
            }
            }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the stock
            if (closeClaws()){
                LOG_EXTENDED_DEBUG("FSM_GATHER_COLLECT: Stock", stock_num, " collected");
                setStockAsRemoved(stock_num);
                gatherStockState = FSM_GATHER_COLLECTED;
                return FSM_RETURN_DONE;
            }
            break;
        case FSM_GATHER_COLLECTED:
            // Wait for the stock to be collected before doing anything else (like navigating to dropzone), to avoid dropping the stock on the way
            LOG_WARNING("GATHER_COLLEDTED: trying to run take stock but stock already in claws");
            return FSM_RETURN_DONE;
            break;
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::DropStock(){
    switch (dropStockState){
        case FSM_DROP_NONE:
            dropzone_num = GetBestDropZone(drive.position);
            LOG_DEBUG("FSM_DROP_NONE: Best drop zone for stock ", stock_num, " is ", dropzone_num);
            if (dropzone_num == -1) {
                LOG_ERROR("FSM_DROP_NONE: No more dropzone available, cannot drop stock ", stock_num);
                return FSM_RETURN_ERROR;
            }
            dropzonePos = getBestDropZonePosition(dropzone_num, drive.position);
            LOG_EXTENDED_DEBUG("FSM_DROP_NONE: Dropzone position for stock ", stock_num, " is (", dropzonePos.x, ", ", dropzonePos.y, ", ", dropzonePos.a , ")");
            dropStockState = FSM_DROP_NAV;
            break;
        case FSM_DROP_NAV:
            {   
            // Navigate to dropzone
            nav_ret = navigationGoTo(dropzonePos, true, true);
            //LOG_INFO("Navigating to stock ", stock_num, " at position (", dropzonePos.x, ",", dropzonePos.y, ") with angle ", dropzonePos.a);

            if ((nav_ret == NAV_DONE) & rotateTwoBlocks(stockOrder)){ // We consider that we are at the dropzone if we are close enough, to avoid navigation errors
                setDropzoneState(dropzone_num, (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_YELLOW : TableState::DROPZONE_BLUE); // Mark dropzone as occupied
                dropStockState = FSM_DROP;
                LOG_EXTENDED_DEBUG("FSM_DROP_NAV: Finished Drop Nav and rotate 2 blocks");
            }
            else if (nav_ret == NAV_ERROR){

                LOG_WARNING("FSM_DROP_NAV(NAV_ERROR): Navigation error while going to dropzone for stock ", stock_num);
                setDropzoneAsError(dropzone_num);
                
                int dropzone_temp = GetBestDropZone(drive.position);
                if(dropzone_temp == -1){
                    LOG_ERROR("FSM_DROP_NAV(NAV_ERROR): No more dropzone available, cannot drop stock ", stock_num);
                    return FSM_RETURN_ERROR;
                }else{
                    setDropzoneState(dropzone_num, TableState::DROPZONE_EMPTY); // Reset previous dropzone state
                    dropzone_num = dropzone_temp;
                    dropzonePos = getBestDropZonePosition(dropzone_num, drive.position);
                    LOG_EXTENDED_DEBUG("FSM_DROP_NAV(NAV_ERROR): New dropzone position for stock ", stock_num, " is (", dropzonePos.x, ",", dropzonePos.y, ")");
                }

                dropStockState = FSM_DROP_NAV;
                return FSM_RETURN_WORKING;
            }
            }
            break;

        case FSM_DROP:
            // Drop the stock
            if (dropBlock()){
                LOG_EXTENDED_DEBUG("FSM_DROP: Stock ", stock_num, "dropped");
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
    position_t navTarget = {625.0, 1220.0, 45.0};
    position_t moveTarget = navTarget;
    moveTarget.y -= 280.0;
    moveTarget.a = 0.0;

    position_t endTarget = navTarget;
    endTarget.x -= 50.0;
    endTarget.y -= 280.0;
    endTarget.a = 0.0;

    if (tableStatus.colorTeam == YELLOW){
        position_robot_flip(navTarget);
        position_robot_flip(moveTarget);
        position_robot_flip(endTarget);
    }

    switch (CursorState){
        case FSM_CURSOR_NAV:
            nav_ret = navigationGoTo(navTarget, true, true);
            if (raiseClaws() & nav_ret == NAV_DONE){ 
                LOG_EXTENDED_DEBUG("FSM_CURSOR_NAV: Nav done and Claws lowered, going to FSM_CURSOR");
                CursorState = FSM_CURSOR_LOW_CLAW;
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_NAV: Navigation error while going to cursor position for lowerClaws");
                return FSM_RETURN_ERROR;
            }
            break;
        case FSM_CURSOR_LOW_CLAW:
            if (lowerClaws()){
                LOG_INFO("Nav done FSM_CURSOR_LOW_CLAW, going to FSM_CURSOR_MOVE");
                CursorState = FSM_CURSOR_MOVE;
            }
            break;

        case FSM_CURSOR_MOVE:
            nav_ret = navigationGoTo(moveTarget, true);
            //LOG_INFO("Claws lowered at cursor position");
            if (nav_ret == NAV_DONE){
                if (raiseClaws()){
                    LOG_EXTENDED_DEBUG("FSM_CURSOR_MOVE: Nav done and raised claws, going to FSM_CURSOR");
                    CursorState = FSM_CURSOR_END;
                }
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_MOVE: Navigation error while moving to cursor position for raiseClaws");
                return FSM_RETURN_ERROR;
            }
            break;

        case FSM_CURSOR_END:
            nav_ret = navigationGoTo(endTarget, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_CURSOR_END: Nav done, cursor action complete");
                CursorState = FSM_CURSOR_NAV;
                return FSM_RETURN_DONE;
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_END: Navigation error while moving to final cursor position");
                return FSM_RETURN_ERROR;
            }
            break;

    }
    return FSM_RETURN_WORKING;
}

void ActionFSM::SetBestAction(position_t position){
    // TODO Etienne - refacto de cette fonction en switch case pour lisibilité

    if(_millis() > tableStatus.startTime + 95000){ // After 95 seconds, switch to NAV_HOME to be sure to be in the arrival zone at the end of the match, even if we are late on the strategy
        LOG_GREEN_INFO("95 seconds passed, switching to NAV_HOME");
        runState = FSM_ACTION_NAV_HOME;
        return;
    }else if(tableStatus.calibrationAge >= CALIBRATION_DEPLETION_TIME){
        runState = FSM_ACTION_CALIBRATION;
        LOG_GREEN_INFO("Calibration aged is greater than 2 going for forced calibration");
        return;
    }
    if((!cursorIsDone()) && (position_distance(position, CursorPos) < 300 || stock_num == (tableStatus.colorTeam == YELLOW ? 5 : 1))){ // If we are close to the cursor position or if we are at stock 
        LOG_GREEN_INFO("Going for cursor action");
        setCursorIsDone(true);
        runState = FSM_ACTION_CURSOR;
        return;
    }

    if(runState == FSM_ACTION_GATHER || stock_num != -1){
        runState = FSM_ACTION_DROP;
        tableStatus.calibrationAge += 1;
        LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to drop a stock, going to FSM_ACTION_DROP");
        return;
    }
    if(runState == FSM_ACTION_DROP || runState == FSM_ACTION_CURSOR || runState == FSM_ACTION_CALIBRATION){
        runState = FSM_ACTION_GATHER;
        LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to gather a stock, going to FSM_ACTION_GATHER");
        return;
    }
}
   
ReturnFSM_t ActionFSM::Calibrate(){
    //TODO Etienne - Le robot se déplace sur le tag aruco au lieu de s'en éloigner lorsqu'il est trop proche du tag
    // maybe bug dans calculateClosestArucoPosition sens du vecteur
    // ajout de log à tester
    
    nav_return_t nav_ret;
    static position_t Calibrationtarget_;
    static position_t arucoPos;

    switch (calibrationState){
        case FSM_CALCULATION:
            arucoPos = calculateClosestArucoPosition(drive.position, Calibrationtarget_);
            calibrationState = FSM_CALIBRATION_RAISE;
            LOG_DEBUG("FSM_CALCULATION: closest aruco marker is at (", arucoPos.x, ", ", arucoPos.y, ", ", arucoPos.a, ")");
            break;
        case FSM_CALIBRATION_NAV:
            {
            // Look towards the closest aruco marker by only spinning in place
            nav_ret = navigationGoTo(Calibrationtarget_, true);
            if (nav_ret == NAV_DONE){
                calibrationState = FSM_CALCULATION;
                LOG_EXTENDED_DEBUG("FSM_CALIBRATION_NAV: Nav done, going to FSM_CALIBRATION_RAISE");
                return FSM_RETURN_DONE;
            }
            else if (nav_ret == NAV_ERROR){
                return FSM_RETURN_ERROR;
            }
            }
            break;
        case FSM_CALIBRATION_RAISE:
            if(raiseClaws()){
                LOG_EXTENDED_DEBUG("FSM_CALIBRATION_RAISE: Raised claws for vision");
                calibrationState = FSM_CALIBRATION_NAV;
            }
            break;
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::GetRobotCenter(){
    nav_return_t nav_ret;
    static position_t aruco1;
    static position_t aruco2;
    static position_t target_ = {drive.position.x, drive.position.y, drive.position.a + 180}; // Look in the opposite direction to find the second aruco marker
    switch (calibrationCameraState){
        case FSM_ARUCO_1:
            {
            bool sucess;
            if (arucoCam1.getPos(aruco1.x, aruco1.y, aruco1.a, sucess) && sucess){
                calibrationCameraState = FSM_ARUCO_NAV;
                LOG_EXTENDED_DEBUG("FSM_ARUCO_1: Found first aruco marker at (", aruco1.x, ", ", aruco1.y, ", ", aruco1.a, "), going to FSM_ARUCO_2");
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
                LOG_GREEN_INFO("FSM_ARUCO_2: Calculated offset between cam and center for aruco1: (", offset.x, ", ", offset.y, ")");
                offset = {aruco2.x - center.x, aruco2.y - center.y, 0};
                LOG_GREEN_INFO("FSM_ARUCO_2: Calculated offset between cam and center for aruco2: (", offset.x, ", ", offset.y, ")");
                drive.setCoordinates(center);
                return FSM_RETURN_DONE;
            }
            }
            break;
        case FSM_ARUCO_NAV:
            {
            nav_ret = navigationGoTo(target_, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_ARUCO_NAV: Nav done for FSM_ARUCO_NAV, going to FSM_ARUCO_2");
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