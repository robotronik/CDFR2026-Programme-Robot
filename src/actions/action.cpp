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
    /****** RESET OF FSM STATES *******/
    gatherStockState = FSM_GATHER_NAV;
    stealStockState = FSM_GATHER_NAV;
    dropStockState = FSM_DROP_NONE;
    CursorState = FSM_CURSOR_NAV;
    calibrationCameraState = FSM_ARUCO_1;
    calibrationState = FSM_CALCULATION;

    /*RESET OF ACTION ID*/
    dropzone_num = -1;
    stock_num = -1;
    steal_count = 0;
    offset = 0;
    targetStockPos = position_t{0,0,0};
    dropzonePos = position_t{0,0,0};
    targetStockFirstPos = position_t{0,0,0};
    closestStock = INFINITY;
    closestSteal = INFINITY;
    stockPos = position_t{0,0,0};
    stockOff = position_t{0,0,0};
    for(size_t _ = 0; _<4 ; _++){
        stockOrder[_] = (tableStatus.colorTeam == YELLOW) ? false : true;
    }
    runState = FSM_ACTION_SAFESTART;
    //SetBestAction(drive.position);
    // TODO reset other states (num,offset, etc.)
}

bool ActionFSM::RunFSM(){
    ReturnFSM_t ret;
    static long unsigned startTime = 0;

    switch (runState)
    {
    case FSM_ACTION_SAFESTART:
        ret = SafeStart();

        if (ret == FSM_RETURN_DONE){
            LOG_INFO("SAFE_START: completed, switching to normal FSM");
            SetBestAction(drive.position);
            runState = FSM_ACTION_GATHER;
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("SAFE_START: failed, forcing normal FSM start");
            SetBestAction(drive.position);
            runState = FSM_ACTION_GATHER;

        }
        break;
    case FSM_ACTION_GATHER:
        ret = TakeStock();
        if (ret == FSM_RETURN_DONE){
            LOG_INFO("ACTION_GATHER: Finished gathering stock ", stock_num);
            SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_GATHER: Couldn't gather stock : ", stock_num);
            tableStatus.setStockAsRemoved(stock_num);
            gatherStockState = FSM_GATHER_NAV;
            stock_num = -1;
            navigationGoTo(drive.position);
            if (raiseClaws()) SetBestAction(drive.position);
        }
        break;
    case FSM_ACTION_STEAL:
        ret = StealStock();
        if (ret == FSM_RETURN_DONE){
            LOG_INFO("FSM_ACTION_STEAL: Finished stealing");
            SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("FSM_ACTION_STEAL: Couldn't steal zone : ", dropzone_num);
            tableStatus.setDropzoneState(dropzone_num,TableState::DROPZONE_EMPTY);
            stealStockState = FSM_GATHER_NAV;
            dropzone_num = -1;
            navigationGoTo(drive.position);
            if (raiseClaws()) SetBestAction(drive.position);

        }
        break;
  
    case FSM_ACTION_DROP:
        ret = DropStock();
        if (ret == FSM_RETURN_DONE){
            LOG_INFO("ACTION_DROP: Finished dropping stock: stock_num is now: ", stock_num);
            SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_DROP: Couldn't drop stock : ", stock_num);
            if(dropzone_num == -1){
                LOG_ERROR("ACTION_DROP: dropzone_num == -1, Should not be the case");
            }
            tableStatus.setDropzoneState(dropzone_num,TableState::DROPZONE_ERROR);
            dropStockState = FSM_DROP_NONE;
            navigationGoTo(drive.position);
            if (raiseClaws()) SetBestAction(drive.position);


        }
        break;
    case FSM_ACTION_WAIT:
        if (startTime == 0) startTime = _millis();
        if (_millis() - startTime > 2000){
            navigationGoTo(drive.position);
            SetBestAction(drive.position);
            startTime = 0;
        }
        break;
    /*
        Action Curseur
        En cas d'erreur on lance une nouvelle action
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
            enableCursor(false);
            SetBestAction(drive.position); // Choisit une nouvelle action, le curseur étant indisponible
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

ReturnFSM_t ActionFSM::SafeStart(){
    static bool init = false;
    static position_t safePos;

    if (!init){
        safePos = position_t{-400, 1075, 45};

        if (tableStatus.colorTeam == YELLOW)
            position_robot_flip(safePos);
        init = true;
    }

    nav_ret = navigationGoTo(safePos, false, false, false);

    if (nav_ret == NAV_DONE){
        LOG_INFO("SAFE_START: Arrived at safe position");
        init = false;
        runState = FSM_ACTION_GATHER;
        return FSM_RETURN_DONE;
    }
    else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;
    

    return FSM_RETURN_WORKING;
}
ReturnFSM_t ActionFSM::TakeStock(){
    //LOG_INFO("TakeStock state: ", gatherStockState, " stock_num: ", stock_num);
    if (stock_num == -1 && gatherStockState == FSM_GATHER_NAV){
        LOG_ERROR("No stock to take");
        return FSM_RETURN_ERROR;
    }

    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            //LOG_DEBUG("entering FSM_GATHER_NAV");
            position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
            nav_ret = navigationGoTo(targetPos, true); // Enabeling A*
            snapClaws(false,false);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_GATHER_NAV: moved to stock at postition (",targetPos.x,", ",targetPos.y, ") now searching for blocks");
                gatherStockState = FSM_GATHER_DETECT;
            }
            else if (nav_ret == NAV_ERROR){ //TODO marche pas si c'est le stock le plus proche est qu'on essaye de le prendre en boucle
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
                if(sucess >=0){
                    //LOG_GREEN_INFO("pos aruco = ", x ," / ", y," / ",  a);
                    LOG_EXTENDED_DEBUG("FSM_GATHER_DETECT: Detection sucess calibration on ", sucess, " blocks");
                    targetStockPos = position_t{x, y, a};
                    targetStockFirstPos = toFirstStockPos(targetStockPos);
                }else if(sucess == -2){
                    LOG_WARNING("FSM_GATHER_DETECT: Stock is empty");
                    gatherStockState = FSM_GATHER_NAV;
                    tableStatus.setStockAsRemoved(stock_num);
                    stock_num = -1;
                    return FSM_RETURN_DONE;
                }else if(sucess == -1){
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
            {
            if (lowerClaws()){
                LOG_EXTENDED_DEBUG("FSM_GATHER_CLAWS: Claws lowered and snap for stock ", stock_num);
                gatherStockState = FSM_GATHER_PREMOVE;
            }
            }
            break;
        case FSM_GATHER_PREMOVE:
            {
            
            nav_ret = navigationGoTo(targetStockFirstPos, false, false, false); // Slow mode for more precision
            //LOG_INFO("Moving to stock ", stock_num, " at position (", stockPos.x + int(stockOff.x * 0.7), ",", stockPos.y + int(stockOff.y * 0.7), ") with angle ", angle);
            if (nav_ret == NAV_DONE){
                gatherStockState = FSM_GATHER_MOVE;
                LOG_EXTENDED_DEBUG("FSM_GATHER_PREMOVE: Pre-Moving to stock ", stock_num, " at position (", targetStockFirstPos.x, ",", targetStockFirstPos.y, ") with angle ", targetStockFirstPos.a);
            }
            else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;

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
            else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;

            }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the stock
            if (closeClaws()){
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

ReturnFSM_t ActionFSM::StealStock(){
    static position_t targetPos_; 
    if (dropzone_num == -1 && stealStockState == FSM_GATHER_NAV){
        //LOG_DEBUG("Getting next stock to take");
        LOG_ERROR("ACTION_STEAL: No dropZone to steal, exiting GatherStock");//Should never be catch
        return FSM_RETURN_ERROR;
    }

    switch (stealStockState)
    {   

        case FSM_GATHER_NAV:
            {
            nav_ret = navigationGoTo(dropzonePos, true); // Enabeling A*
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_GATHER_NAV: moved to dropZone at postition (",dropzonePos.x,", ",dropzonePos.y, ") now searching for blocks");
                stealStockState = FSM_GATHER_DETECT;
            }else if(nav_ret == NAV_IN_PROCESS){
                snapClaws(false,false);
            }
            else if (nav_ret == NAV_ERROR){//marche pas si le stock c'est le plus proche et qu'on essaye de le prendre en boucle
                LOG_ERROR("FSM_GATHER_NAV: Navigation error while going to dropzone ", dropzone_num);
                dropzone_num = -1;
                stealStockState = FSM_GATHER_NAV;
                return FSM_RETURN_DONE;
            }
            }
            break;

        case FSM_GATHER_DETECT:
        {
            double x = drive.position.x;
            double y = drive.position.y;
            double a = drive.position.a;
            if(arucoCam1.getObjectInfoColors(stockOrder, x, y, a, steal_count)){
                if(steal_count>=0){
                    targetPos_ = position_t{x,y,a};
                    stealStockState = FSM_GATHER_CLAWS;
                    LOG_EXTENDED_DEBUG("FSM_GATHER_DETECT: Found ", steal_count, " objects to steal");
                }else if (steal_count == -2){
                    LOG_WARNING("FSM_GATHER_DETECT: DropZone was empty");
                    stealStockState = FSM_GATHER_NAV;
                    tableStatus.setDropzoneState(dropzone_num,TableState::DROPZONE_EMPTY);
                    dropzone_num = -1;
                    return FSM_RETURN_DONE;
                }else if(steal_count == -1){
                    LOG_ERROR("FSM_GATHER_DETECT: Camera Error don't know what to do");
                    stealStockState = FSM_GATHER_NAV;
                    dropzone_num = -1;
                    return FSM_RETURN_ERROR;
                }
            }
            break;
        }
        case FSM_GATHER_CLAWS:
            {
            if (lowerClaws()){
                LOG_EXTENDED_DEBUG("FSM_GATHER_CLAWS: Claws lowered and snap for dropZone ", dropzone_num);
                stealStockState = FSM_GATHER_MOVE;
            }
            }
            break;
        case FSM_GATHER_PREMOVE:
            {
            
            nav_ret = navigationGoTo(targetStockFirstPos, false, false, false); // Slow mode for more precision
            if (nav_ret == NAV_DONE){
                gatherStockState = FSM_GATHER_MOVE;
                LOG_EXTENDED_DEBUG("FSM_GATHER_PREMOVE: Pre-Moving to stock ", stock_num, " at position (", targetStockFirstPos.x, ",", targetStockFirstPos.y, ") with angle ", targetStockFirstPos.a);
            }
            else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;

            }
            break;
        case FSM_GATHER_MOVE:
        {
            nav_ret = navigationGoTo(targetPos_, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_GATHER_NAV: moved to dropZone at postition (",targetPos_.x,", ",targetPos_.y, ")");
                stealStockState = FSM_GATHER_COLLECT;
                break;
            }
            else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;

        }
            break;
        case FSM_GATHER_COLLECT:
            // Collect the steal
        {
            if (closeClaws()){
                LOG_EXTENDED_DEBUG("FSM_GATHER_COLLECT: dropZone", dropzone_num, " collected");
                stealStockState = FSM_GATHER_COLLECTED;
            }
        }
            break;
        case FSM_GATHER_COLLECTED:
        {
            if(steal_count == 4){
                LOG_DEBUG("FSM_GATHER_COLLECTED: pas de déplacement de blocks tous les blocks ont été pris");// Si plus que 4 blocks?
            }else{
                //TODO tej les blocks restant.
                LOG_ERROR("FSM_GATHER_COLLECTED: Tej des blocks pas encore implémenté");
            }
            // Force le drop dans la même zone
            dropStockState = FSM_DROP_NAV;
            dropzonePos = targetPos_; // à changer en cas de virage de blocks
            rotate_done = false;
            stock_num = steal_count; // marking random value to pass Best Action condition on drop action
            steal_count = -1;
            tableStatus.setDropzoneState(dropzone_num, TableState::DROPZONE_EMPTY);
            return FSM_RETURN_DONE;
        }
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::DropStock(){
    switch (dropStockState){
        case FSM_DROP_NONE:
        {
            rotate_done = false;
            getBestDropZonePosition(dropzone_num, dropzonePos);
            if (dropzone_num == -1) {
                LOG_ERROR("FSM_DROP_NONE: No more dropzone available, cannot drop stock ", stock_num);
                return FSM_RETURN_ERROR;
            }else{
                LOG_GREEN_INFO("FSM_DROP_NONE: Best drop zone for stock ", stock_num, " is ", dropzone_num);
                LOG_EXTENDED_DEBUG("FSM_DROP_NONE: Dropzone position for stock ", stock_num, " is (", dropzonePos.x, ", ", dropzonePos.y, ", ", dropzonePos.a , ")");
                LOG_EXTENDED_DEBUG("FSM_DROP_NONE -> FSM_DROP_NAV");
            }
            dropStockState = FSM_DROP_NAV;
        }
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
                
                int dropzone_temp = -1; 
                getBestDropZonePosition(dropzone_temp, dropzonePos);
                if(dropzone_temp == -1){
                    LOG_ERROR("FSM_DROP_NAV(NAV_ERROR): No more dropzone available, cannot drop stock ", stock_num);
                    tableStatus.setDropzoneState(dropzone_num, TableState::DROPZONE_EMPTY); // Reset previous dropzone state
                    return FSM_RETURN_ERROR;
                }else{
                    tableStatus.setDropzoneState(dropzone_num, TableState::DROPZONE_EMPTY); // Reset previous dropzone state
                    dropzone_num = dropzone_temp;
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

                gatherStockState = FSM_GATHER_NAV;
                stealStockState = FSM_GATHER_NAV;
                tableStatus.setDropzoneState(dropzone_num, (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_BLUE : TableState::DROPZONE_YELLOW);  

                //No more stock in hand
                stock_num = -1;
                dropzone_num = -1;
                steal_count = -1;
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
            else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;

        }
            break;
            
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::Cursor(){
    static long unsigned startTime = 0;
    position_t navTarget = {780.0, 1280.0, -135.0};
    position_t navTargetRot = navTarget;
    navTargetRot.a = -180.0;
    position_t moveTarget = navTargetRot;
    moveTarget.y -= 350.0;
    position_t moveSafeTarget = moveTarget;
    moveSafeTarget.x -= 250.0;
    
    if (tableStatus.colorTeam == YELLOW){
        position_robot_flip(navTarget);
        position_robot_flip(navTargetRot);
        position_robot_flip(moveTarget);
        position_robot_flip(moveSafeTarget);
    }

    switch (CursorState){
        case FSM_CURSOR_NAV:
            nav_ret = navigationGoTo(navTarget, true);
            if ((nav_ret == NAV_DONE)){ 
                LOG_EXTENDED_DEBUG("FSM_CURSOR_NAV: Nav done, going to FSM_CURSOR");
                if (enableCursor(true)){
                    LOG_EXTENDED_DEBUG("FSM_CURSOR_NAV: Cursor enabled, going to FSM_CURSOR_LOW_CLAW");
                    startTime = _millis();
                    CursorState = FSM_CURSOR_ROT_NAV;
                }
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_NAV: Navigation error while going to cursor position for lowerClaws");
                return FSM_RETURN_ERROR;
            }
            break;
        case FSM_CURSOR_ROT_NAV:
            if (_millis() - startTime > 250){
                nav_ret = navigationGoTo(navTargetRot, true, true, false);
                if (nav_ret == NAV_DONE){
                    LOG_EXTENDED_DEBUG("FSM_CURSOR_LOW_CLAW: Nav done for lowerClaws, going to FSM_CURSOR_MOVE");
                            CursorState = FSM_CURSOR_MOVE;
                }    
                else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;

            }
            break;

        case FSM_CURSOR_MOVE:
            nav_ret = navigationGoTo(moveTarget, true, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_CURSOR_MOVE: Nav done , going to FSM_CURSOR");
                CursorState = FSM_CURSOR_END;
            }
            else if (nav_ret == NAV_ERROR){
                LOG_WARNING("FSM_CURSOR_MOVE: Navigation error while going to cursor position for lowerClaws");
                return FSM_RETURN_ERROR;
            }
            break;

        case FSM_CURSOR_END:
            nav_ret = navigationGoTo(moveSafeTarget, false, true);
            if (nav_ret == NAV_DONE){
                LOG_EXTENDED_DEBUG("FSM_CURSOR_END: Nav done for move safe target");
                if (enableCursor(false)){
                    LOG_EXTENDED_DEBUG("FSM_CURSOR_END: Cursor action done");
                    CursorState = FSM_CURSOR_NAV;
                    startTime = 0;
                    return FSM_RETURN_DONE;
            }
            }    
            else if (nav_ret == NAV_ERROR)return FSM_RETURN_ERROR;

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

    if (_millis() > tableStatus.startTime + 50000)
            tableStatus.startTime = _millis();
    /*********************** RESET DES DISTANCES POUR BEST ACTIONS *********************/
    closestStock = INFINITY;
    closestSteal = INFINITY;

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

    /**************************** CONDITIONS POUR DROP UN STOCK ***************************************/
    if(tableStatus.remainingDropExist() && stock_num != -1){ // On peut DROP à partir du moment où on a un stock et qu'il reste des drop zones
        runState = FSM_ACTION_DROP;
        tableStatus.calibrationAge += 1;
        LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to drop a stock, going to FSM_ACTION_DROP");
        return;
    }//TODO cas pas de drop mais blocks dans la main -> poser 2 stocks même zone ou poser dans une zone adverse

    /********** CALCUL BEST STOCK ************/
    if (stock_num == -1 && gatherStockState == FSM_GATHER_NAV){
        //LOG_DEBUG("Getting next stock to take");
        closestStock = chooseStockStrategy(stock_num, offset);
        if (stock_num == -1){
            LOG_WARNING("ACTION_GATHER: No more stocks to take");
            stock_num = -1;
            gatherStockState = FSM_GATHER_NAV;
            closestStock = INFINITY;
        }else{
            stockPos = STOCK_POSITIONS_TABLE[stock_num];
            stockOff = STOCK_OFFSETS[offset];
        }
    }

    /************** CALCUL BEST STEAL *****************/
    if (dropzone_num == -1 && stealStockState == FSM_GATHER_NAV){
        //LOG_DEBUG("Getting next stock to take");
        closestSteal = getBestStealZonePosition(dropzone_num, dropzonePos);
        if (dropzone_num == -1){
            LOG_ERROR("ACTION_STEAL: No dropZone to steal, exiting GatherStock");//Should never be catch
            dropzone_num = -1;
            stealStockState = FSM_GATHER_NAV;
            closestSteal = INFINITY;
        }
    }

    if(closestSteal == INFINITY && closestStock == INFINITY){
        /*********************** CONDITIONS POUR FAIRE LE CURSEUR ************************/
        if(!tableStatus.cursorIsDone()){ 
            LOG_GREEN_INFO("Going for cursor action");
            runState = FSM_ACTION_CURSOR;
            return;
        }
        LOG_ERROR("Nothing else to do waiting");
        runState = FSM_ACTION_WAIT;
    }else{
        /*********************** CONDITION POUR VOLER UN STOCK OU TAKE STOCK ****************************/
        if(closestSteal < closestStock){
            LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to steal a drop, going to FSM_ACTION_STEAL");
            LOG_GREEN_INFO("ACTION_STEAL: Next dropZone to steal: ", dropzone_num);
            runState = FSM_ACTION_STEAL;
            stock_num = -1;
            return;
        }else{
            runState = FSM_ACTION_GATHER;
            dropzone_num = -1;
            LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to gather a stock, going to FSM_ACTION_GATHER");
            LOG_GREEN_INFO("ACTION_GATHER: Next stock to take: ", stock_num, " offset: ", offset);
            return;
        }
    }

}


/*
********************************************************************************************
*  SECTION CALIBRATION
*
********************************************************************************************
*/

/*
    Force la calibration en se tournant vers un tag ou s'éloignant d'un tag aruco
    Si la navigation échoue la calibration est considérée échouée et sera retentée une action plus tard
*/
ReturnFSM_t ActionFSM::Calibrate(){
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
            else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;
            
            }
            break;
    }
    return FSM_RETURN_WORKING;
}


/*
* FSM de récupération du centre du robot
*/
ReturnFSM_t ActionFSM::GetRobotCenter(){
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
            else if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;
            
            }
             break;
    }
    return FSM_RETURN_WORKING;
}