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
    runState = FSM_ACTION_GATHER;

    /****** RESET OF FSM STATES *******/
    gatherStockState = FSM_GATHER_NAV;
    gatherIsolatedStockState = FSM_GATHER_DETECT;
    dropStockState = FSM_DROP_NONE;
    CursorState = CURSOR_RAISE_CLAW;
    calibrationCameraState = FSM_ARUCO_1;
    calibrationState = FSM_CALCULATION;

    /*RESET OF ACTION ID*/
    dropzone_num = 0;
    stock_num = -1;
    offset = 0;
    targetStockPos = position_t{0,0,0};
    dropzonePos = position_t{0,0,0};
    targetStockFirstPos = position_t{0,0,0};
    for(size_t _ = 0; _<4 ; _++){
        stockOrder[_] = (tableStatus.colorTeam == YELLOW) ? false : true;
    }
    
    // TODO reset other states (num,offset, etc.)
}

bool ActionFSM::RunFSM(){
    ReturnFSM_t ret;
    switch (runState)
    {
    /*
        Action Gather:

    */
    case FSM_ACTION_GATHER:
        ret = TakeStock();
        if (ret == FSM_RETURN_DONE){
            LOG_INFO("ACTION_GATHER: Finished gathering stock ", stock_num);
            SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_GATHER: Couldn't gather");
            // TODO Handle error
        }
        break;
    case FSM_ACTION_GATHER_ISOLATED:
        ret = TakeIsolatedStock();
        if (ret == FSM_RETURN_DONE){
            LOG_INFO("FSM_ACTION_GATHER_ISOLATED: Finished gathering");
            return true;
            //SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("FSM_ACTION_GATHER_ISOLATED: Couldn't gather");
            // TODO Handle error
        }
        break;
    /*
        Action drop block in zone
        Error make the action be postponned
    */
    case FSM_ACTION_DROP:
        ret = DropStock();
        if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_DROP: Couldn't drop");
            if(dropzone_num == -1){
                LOG_ERROR("ACTION_DROP: dropzone_num == -1, Should not be the case");
            }else{
                LOG_ERROR("ACTION_DROP: last dropzone was not available");
                SetBestAction(drive.position); // Going for next action, try again later
            }

        }else if (ret == FSM_RETURN_DONE){
            LOG_INFO("ACTION_DROP: Finished dropping stock: stock_num is now: ", stock_num);
            SetBestAction(drive.position);
        }
        break;
    
    /*
        Action Curseur
        En cas d'erreur on lance une nouvelle action et retentera plus tard
    */
    case FSM_ACTION_CURSOR:
        ret = Cursor();
        if (ret == FSM_RETURN_DONE){
            LOG_INFO("ACTION_CURSOR: Finished cursor action");
            tableStatus.setCursorIsDone(true); // Place le curseur comme virtuellement fait
            SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_CURSOR: Couldn't do cursor action");
            tableStatus.setCursorIsDone(true); // Place le curseur comme virtuellement fait
            SetBestAction(drive.position); // Choisit une nouvelle action, le curseur étant indisponible
            tableStatus.setCursorIsDone(false); // Rend le curseur de nouveau disponible
        }
        break;

    /*
        Action retour sur zone de départ
        n'est run que si plus rien n'est possible sur la table ou si le temps est écoulé
    */
    case FSM_ACTION_NAV_HOME:
        if (returnToHome()){
            LOG_INFO("ACTION_NAV_HOME: Finished going home");
            runState = FSM_ACTION_GATHER;
            return true; // Robot is done
        }
        break;
    /*
        Action forçant la calibration 
    */
    case FSM_ACTION_CALIBRATION:
        ret = Calibrate();
        if (ret == FSM_RETURN_DONE){
            // Si calibrationAge != 0 la calibration a échoué
            // et on se donne une action supplémentaire avant de retenter
            if(tableStatus.calibrationAge) tableStatus.calibrationAge -=1; 
            LOG_INFO("ACTION_CALIBRATION: Finished calibration action");
            SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_CALIBRATION: Couldn't do calibration action, going for next action");
            tableStatus.calibrationAge -=1; // On se donne une action supplémentaire avant de retenter la calibration
            SetBestAction(drive.position);
        }
        break;
    /*
        Action de calibration ne sera pas éxecutée en match
        En cas d'erreur return true pour mettre fin au match
    */
    case FSM_CENTER_CALIBRATION:
        ret = GetRobotCenter();
        if(ret == FSM_RETURN_DONE){
            LOG_INFO("CENTER_CALIBRATION: Finished center calibration step ", calibrationCameraState);
            runState = FSM_ACTION_NAV_HOME;
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("CENTER_CALIBRATION: Error during center calibration step ", calibrationCameraState);
            return true; // Robot is done
        }
        break;
    }
    return false;
}

ReturnFSM_t ActionFSM::TakeStock(){
    //LOG_INFO("TakeStock state: ", gatherStockState, " stock_num: ", stock_num);
    if (stock_num == -1 && gatherStockState == FSM_GATHER_NAV){
        //LOG_DEBUG("Getting next stock to take");
        if (!chooseStockStrategy(stock_num, offset)){
            LOG_ERROR("ACTION_GATHER: No more stocks to take, exiting GatherStock");//Should never be catch
            stock_num = -1;
            gatherStockState = FSM_GATHER_NAV;
            return FSM_RETURN_DONE;
        }
        LOG_GREEN_INFO("ACTION_GATHER: Next stock to take: ", stock_num, " offset: ", offset);
    }

    position_t stockPos = STOCK_POSITIONS_TABLE[stock_num];
    position_t stockOff = STOCK_OFFSETS[offset];

    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            //LOG_DEBUG("entering FSM_GATHER_NAV");
            position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
            nav_ret = navigationGoTo(targetPos, true); // Enabeling A*
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_GATHER_NAV: moved to stock at postition (",targetPos.x,", ",targetPos.y, ") now searching for blocks");
                gatherStockState = FSM_GATHER_DETECT;
            }else if(nav_ret == NAV_IN_PROCESS){
                snapClaws(false,false);
            }
            else if (nav_ret == NAV_ERROR){
                LOG_ERROR("FSM_GATHER_NAV: Navigation error while going to stock ", stock_num);
                stock_num = -1;
                gatherStockState = FSM_GATHER_NAV;
                return FSM_RETURN_DONE;
            }
            }
            break;
        case FSM_GATHER_DETECT:
            {
            double x = drive.position.x;
            double y = drive.position.y;
            double a = drive.position.a;
            int sucess = -1;

            if(arucoCam1.getObjectInfoColors(stockOrder,x,y,a,sucess)){
                if(!sucess){
                    //LOG_GREEN_INFO("pos aruco = ", x ," / ", y," / ",  a);
                    LOG_EXTENDED_DEBUG("FSM_GATHER_DETECT: Detection sucess calibration on blocks");
                    targetStockPos = position_t{x, y, a};
                    targetStockFirstPos = toFirstStockPos(targetStockPos);
                }else if(sucess == 1){
                    LOG_WARNING("FSM_GATHER_DETECT: Drop Zone is empty");
                    gatherStockState = FSM_GATHER_NAV;
                    tableStatus.setStockAsRemoved(stock_num);
                    stock_num = -1;
                    return FSM_RETURN_DONE;
                }else{
                    LOG_ERROR("FSM_GATHER_DETECT: Camera Error don't know what to do");
                    gatherStockState = FSM_GATHER_NAV;
                    stock_num = -1;
                    return FSM_RETURN_ERROR;
                }
                gatherStockState = FSM_GATHER_CLAWS;
            }
            }
            break;
        case FSM_GATHER_CLAWS:
            if (lowerClaws()){
                LOG_EXTENDED_DEBUG("FSM_GATHER_CLAWS: Claws lowered and snap for stock ", stock_num);
                gatherStockState = FSM_GATHER_PREMOVE;
            }
            break;
        case FSM_GATHER_PREMOVE:
            {
            
            nav_ret = navigationGoTo(targetStockFirstPos, false, true, false); // Slow mode for more precision
            //LOG_INFO("Moving to stock ", stock_num, " at position (", stockPos.x + int(stockOff.x * 0.7), ",", stockPos.y + int(stockOff.y * 0.7), ") with angle ", angle);
            if (nav_ret == NAV_DONE){
                if (moveServoAndWait(SERVO_NUM_6, 180, 200)){
                    gatherStockState = FSM_GATHER_MOVE;
                    LOG_EXTENDED_DEBUG("FSM_GATHER_PREMOVE: Pre-Moving to stock ", stock_num, " at position (", targetStockFirstPos.x, ",", targetStockFirstPos.y, ") with angle ", targetStockFirstPos.a);
                    //target stock pos = targetStockFirstPos 30cm à gauche (du robot)
                    targetStockPos = position_t{targetStockFirstPos.x - int(300 * sin(DEG_TO_RAD*targetStockFirstPos.a)), targetStockFirstPos.y - int(300 * cos(DEG_TO_RAD*targetStockFirstPos.a)), targetStockFirstPos.a};
                } 
            }
            }
            break;
        case FSM_GATHER_MOVE:
            {
            nav_ret = navigationGoTo(targetStockPos, false, true); // Slow mode for more precision
            //LOG_INFO("Moving to stock ", stock_num, " at position (", stockPos.x + int(stockOff.x * 0.7), ",", stockPos.y + int(stockOff.y * 0.7), ") with angle ", angle);
            if (nav_ret == NAV_DONE){
                gatherStockState = FSM_GATHER_COLLECT;
                LOG_EXTENDED_DEBUG("FSM_GATHER_MOVE: Moving to stock ", stock_num, " at position (", targetStockPos.x, ",", targetStockPos.y, ") with angle ", targetStockPos.a);
            }
            }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the stock
            if (closeClaws() && moveServoAndWait(SERVO_NUM_6, 90, 200)){
                LOG_EXTENDED_DEBUG("FSM_GATHER_COLLECT: Stock", stock_num, " collected");
                tableStatus.setStockAsRemoved(stock_num);
                gatherStockState = FSM_GATHER_COLLECTED;
                return FSM_RETURN_DONE;
            }
            break;
        case FSM_GATHER_COLLECTED:
            // Wait for the stock to be collected before doing anything else (like navigating to dropzone), to avoid dropping the stock on the way
            LOG_WARNING("GATHER_COLLECTED: trying to run take stock but stock already in claws");
            return FSM_RETURN_DONE;
            break;
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::TakeIsolatedStock(){
    static position_t targetPos_; 
    switch (gatherIsolatedStockState)
    {   
        case FSM_GATHER_COLLECTED:
            return FSM_RETURN_ERROR;

        case FSM_GATHER_DETECT:
        {
            double x = drive.position.x;
            double y = drive.position.y;
            double a = drive.position.a;
            bool sucess = false;
            if(arucoCam1.getBestIsolatedObject(x,y,a,sucess)){
                if(sucess){
                    targetPos_ = getBestIsolatedPosition(position_t{x,y,a}, drive.position);
                    gatherIsolatedStockState = FSM_GATHER_NAV;
                    LOG_EXTENDED_DEBUG("FSM_GATHER_DETECT: Found an isolated object");
                }else{
                    LOG_ERROR("FSM_GATHER_DETECT: Failed to find an isolated object");
                    return FSM_RETURN_ERROR;
                }
            }
            break;
        }
        case FSM_GATHER_NAV:
            nav_ret = navigationGoTo(targetPos_, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_GATHER_NAV: moved to stock at postition (",targetPos_.x,", ",targetPos_.y, ")");
                gatherIsolatedStockState = FSM_GATHER_COLLECT;
                break;
            }
            break;
        case FSM_GATHER_COLLECT:
            //TODO add utilisation de la ventouse
            gatherIsolatedStockState = FSM_GATHER_MOVE;
            break;
        case FSM_GATHER_MOVE:
            //TODO add virer les blocks restant dans la zone
            gatherIsolatedStockState = FSM_GATHER_CLAWS;
            break;
        case FSM_GATHER_CLAWS:
            //TODO add drop du bon block
            gatherIsolatedStockState = FSM_GATHER_DETECT;
            return FSM_RETURN_DONE;
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::DropStock(){
    switch (dropStockState){
        case FSM_DROP_NONE:
            rotate_done = false;
            dropzone_num = GetBestDropZone(drive.position);
            LOG_GREEN_INFO("FSM_DROP_NONE: Best drop zone for stock ", stock_num, " is ", dropzone_num);
            if (dropzone_num == -1) {
                LOG_ERROR("FSM_DROP_NONE: No more dropzone available, cannot drop stock ", stock_num);
                return FSM_RETURN_ERROR;
            }
            dropzonePos = getBestDropZonePosition(dropzone_num, drive.position);
            LOG_EXTENDED_DEBUG("FSM_DROP_NONE: Dropzone position for stock ", stock_num, " is (", dropzonePos.x, ", ", dropzonePos.y, ", ", dropzonePos.a , ")");
            LOG_EXTENDED_DEBUG("FSM_DROP_NONE -> FSM_DROP_NAV");
            dropStockState = FSM_DROP_NAV;
            break;
        case FSM_DROP_NAV:
        {               
            nav_ret = navigationGoTo(dropzonePos, true);
    
            if (!rotate_done) rotate_done = rotateTwoBlocks(stockOrder);
        
            if ((nav_ret == NAV_DONE) && rotate_done ) {
                dropStockState = FSM_DROP;
                LOG_EXTENDED_DEBUG("FSM_DROP_NAV: Finished Drop Nav and rotate 2 blocks");
            }
            else if (nav_ret == NAV_ERROR){

                LOG_WARNING("FSM_DROP_NAV(NAV_ERROR): Navigation error while going to dropzone for stock ", stock_num);
                tableStatus.setDropzoneAsError(dropzone_num);
                
                int dropzone_temp = GetBestDropZone(drive.position);
                if(dropzone_temp == -1){
                    LOG_ERROR("FSM_DROP_NAV(NAV_ERROR): No more dropzone available, cannot drop stock ", stock_num);
                    tableStatus.setDropzoneState(dropzone_num, TableState::DROPZONE_EMPTY); // Reset previous dropzone state
                    return FSM_RETURN_ERROR;
                }else{
                    tableStatus.setDropzoneState(dropzone_num, TableState::DROPZONE_EMPTY); // Reset previous dropzone state
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
                LOG_EXTENDED_DEBUG("FSM_DROP: Stock ", stock_num, " dropped");
                dropStockState = FSM_DROP_NAV_BACK;
                backPos = drive.position;
                backPos.x -= 100 * cos(DEG_TO_RAD * drive.position.a);
                backPos.y -= 100 * sin(DEG_TO_RAD * drive.position.a);

                //No more stock in hand
                stock_num = -1;
                gatherStockState = FSM_GATHER_NAV;
                tableStatus.setDropzoneState(dropzone_num, (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_YELLOW : TableState::DROPZONE_BLUE);  

            }
            break;
        case FSM_DROP_NAV_BACK:
        {
            // We don't come to a complete stop when backing up
            nav_ret = navigationGoTo(backPos, false, false, false);
        
            if (nav_ret == NAV_DONE) {
                dropStockState = FSM_DROP_NONE;
                offset = 0;  
                rotate_done = false;
                LOG_EXTENDED_DEBUG("FSM_DROP_NAV_BACK: Finished Drop Nav Back");
                return FSM_RETURN_DONE; 
            }
        }
            break;
            
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::Cursor(){
    position_t navTarget = {800.0, 1300.0, -180.0};
    position_t navTargetRot = navTarget;
    navTargetRot.a = -120.0;

    position_t moveTarget = navTargetRot;
    moveTarget.y -= 500.0;
    
    if (tableStatus.colorTeam == YELLOW){
        position_robot_flip(navTarget);
        position_robot_flip(navTargetRot);
        navTargetRot.a = -120.0;
        position_robot_flip(moveTarget);
        moveTarget.a = -120.0;
    }

    switch (CursorState){
        case CURSOR_RAISE_CLAW:
            if (rotateTwoBlocks(stockOrder)){
                LOG_EXTENDED_DEBUG("CURSOR_RAISE_CLAW: going to FSM_CURSOR_NAV");
                CursorState = FSM_CURSOR_NAV;
            }
            break;
        case FSM_CURSOR_NAV:
            nav_ret = navigationGoTo(navTarget, false); //false sinon il va pas vouloir y aller car trop proche du mur
            if ((nav_ret == NAV_DONE)){ 
                LOG_EXTENDED_DEBUG("FSM_CURSOR_NAV: Nav done, going to FSM_CURSOR");
                if (enableCursor(true)){
                    LOG_EXTENDED_DEBUG("FSM_CURSOR_NAV: Ventouse lowered at cursor position, going to FSM_CURSOR_LOW_CLAW");
                    CursorState = FSM_CURSOR_LOW_CLAW;
                }
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_NAV: Navigation error while going to cursor position for lowerClaws");
                return FSM_RETURN_ERROR;
            }
            break;
        case FSM_CURSOR_LOW_CLAW:
            nav_ret = navigationGoTo(navTargetRot, false, true); //false sinon il va pas vouloir y aller car trop proche du mur
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_CURSOR_LOW_CLAW: Nav done for lowerClaws, going to FSM_CURSOR_MOVE");
                CursorState = FSM_CURSOR_MOVE;
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_LOW_CLAW: Navigation error while going to cursor position for lowerClaws");
                return FSM_RETURN_ERROR;
            }       
            break;

        case FSM_CURSOR_MOVE:
            nav_ret = navigationGoTo(moveTarget, false, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_CURSOR_MOVE: Nav done , going to FSM_CURSOR");
                CursorState = FSM_CURSOR_END;
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_MOVE: Navigation error while moving to cursor position for raiseClaws");
                return FSM_RETURN_ERROR;
            }
            break;

        case FSM_CURSOR_END:
            if (enableCursor(false)){
                LOG_EXTENDED_DEBUG("FSM_CURSOR_END: Cursor action done");
                CursorState = CURSOR_RAISE_CLAW;
                return FSM_RETURN_DONE;
            }
            break;

    }
    return FSM_RETURN_WORKING;
}

/*
    Plus l'action est prioritaire plus elle apparaît tôt dans le code.
        Ex: le retour êtant prioritaire sur toutes les autres actions on fera toujours le retour si les conditions sont remplies
    Priorités actuelles:
        - Retour
        - Calibration
        - Curseur
        - Drop
        - Take
*/
void ActionFSM::SetBestAction(position_t position){

    /********************* CONDITIONS POUR LE RETURN HOME ***********************/
    if(_millis() > tableStatus.startTime + 95000){ // After 95 seconds, switch to NAV_HOME to be sure to be in the arrival zone at the end of the match, even if we are late on the strategy
        LOG_GREEN_INFO("95 seconds passed, switching to NAV_HOME");
        runState = FSM_ACTION_NAV_HOME;
        return;
    }

    /************************** CONDITIONS SUR LA CALIBRATION *************************/
    if(tableStatus.calibrationAge >= CALIBRATION_DEPLETION_TIME){
        runState = FSM_ACTION_CALIBRATION;
        LOG_GREEN_INFO("Calibration aged is greater than 2 going for forced calibration");
        return;
    }

    /*********************** CONDITIONS POUR FAIRE LE CURSEUR ************************/
    if((false && !tableStatus.cursorIsDone()) && (position_distance(position, tableStatus.CursorPos) < 300 || stock_num == (tableStatus.colorTeam == YELLOW ? 5 : 1))){ // If we are close to the cursor position or if we are at stock 
        LOG_GREEN_INFO("Going for cursor action");
        runState = FSM_ACTION_CURSOR;
        return;
    }

    /**************************** CONDITIONS POUR DROP UN STOCK ***************************************/
    if(tableStatus.remainingDropExist() && stock_num != -1){ // On peut DROP à partir du moment où on a un stock et qu'il reste des drop zones
        runState = FSM_ACTION_DROP;
        tableStatus.calibrationAge += 1;
        LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to drop a stock, going to FSM_ACTION_DROP");
        return;
    }

    /*****************CONDITIONS POUR RECUPERER UN STOCK **********************************/
    if(tableStatus.remainingStocksExist() && stock_num == -1){// On ne se base plus sur l'état précédent mais sur la possibilité de réaliser l'action
        runState = FSM_ACTION_GATHER;
        LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to gather a stock, going to FSM_ACTION_GATHER");
        return;
    }

}

/*
    Force la calibration en se tournant vers un tag ou s'éloignant d'un tag aruco
    Si la navigation échoue la calibration est considérée échouée et sera retentée une action plus tard
*/
ReturnFSM_t ActionFSM::Calibrate(){
    nav_return_t nav_ret;
    static position_t Calibrationtarget_;

    switch (calibrationState){
        case FSM_CALCULATION:
            Calibrationtarget_ = calculateClosestArucoPosition(drive.position);
            calibrationState = FSM_CALIBRATION_NAV;
            break;
        case FSM_CALIBRATION_NAV:
            {
            // Look towards the closest aruco marker to recalibrate the position
            nav_ret = navigationGoTo(Calibrationtarget_, true);
            if (nav_ret == NAV_DONE){
                calibrationState = FSM_CALCULATION;
                LOG_EXTENDED_DEBUG("FSM_CALIBRATION_NAV: Nav done, going to FSM_CALCULATION");
                return FSM_RETURN_DONE;
            }
            else if (nav_ret == NAV_ERROR){
                return FSM_RETURN_ERROR;
            }
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
            nav_ret = navigationGoTo(target_);
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