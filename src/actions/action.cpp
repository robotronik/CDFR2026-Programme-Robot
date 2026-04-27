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
    sweepState = FSM_SWEEP_INIT;
    vidangeState = FSM_VIDANGE_INIT;
    dropStockState = FSM_DROP_NONE;
    CursorState = CURSOR_RAISE_CLAW;
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
    runState = FSM_ACTION_GATHER;
    SetBestAction(drive.position);
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
    case FSM_ACTION_STEAL:
        ret = StealStock();
        if (ret == FSM_RETURN_DONE){
            LOG_INFO("FSM_ACTION_STEAL: Finished stealing");
            SetBestAction(drive.position);
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("FSM_ACTION_STEAL: Couldn't steal");
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
    case FSM_ACTION_WAIT:
        SetBestAction(drive.position);
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
            else if (nav_ret == NAV_ERROR){
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
                //drive.is_slow_mode = true;
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
        }
            break;
        case FSM_GATHER_COLLECT:
            //TODO add utilisation de la ventouse ou pince
            // Collect the steal
        {
            if (closeClaws()){
                //drive.is_slow_mode = false;
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
        }
            break;
            
    }
    return FSM_RETURN_WORKING;
}

ReturnFSM_t ActionFSM::BalayageSteal(position_t targetPos, double angle, double distanceBalayage){
    //targetPos1 = position premier block à voler
    double margeBalayage = 50.0;
    distanceBalayage += margeBalayage;
    
    static double cosinus, sinus;
    static position_t targetPos1,targetPos2, targetPos3, targetPos4;
    static double startTime = 0;
    static bool needToGoToWall = false, claws_done = false;

    switch(sweepState){

        case FSM_SWEEP_INIT: //
        {
            LOG_INFO("SWEEP: init");
            startTime = 0;
            needToGoToWall = false, claws_done = false;
            sweepState = FSM_SWEEP_DETECT;
            break;
        }

        case FSM_SWEEP_DETECT:
        {
            LOG_DEBUG("FSM_SWEEP_DETECT: Detection sucess calibration on blocks");
            targetPos1 = targetPos;
            cosinus = cos(DEG_TO_RAD * angle);
            sinus   = sin(DEG_TO_RAD * angle);
            
            // Se décaler distanceBalayage mm à gauche du stock (va reculer un peu si trop proche du mur avec NearestValidZone())
            targetPos2.y = targetPos1.y + distanceBalayage * cosinus;
            targetPos2.x = targetPos1.x - distanceBalayage * sinus;
            targetPos2.a = angle;

            startTime = 0;
            bool tp2 = NearestValidZone(&targetPos2);
            if (NearestValidZone(&targetPos1) || tp2){
                needToGoToWall = true;
            }
            // Se reculer pour prendre le stock de 20 et se décaler de 50mm à gauche
            targetPos3.y = targetPos2.y - 20.0 * sinus + 50.0 * cosinus;
            targetPos3.x = targetPos2.x - 20.0 * cosinus - 50.0 * sinus;
            targetPos3.a = targetPos2.a + 10.0;

            //S'avance de 20 mm pour collect
            targetPos4.y = targetPos3.y + 20.0 * sinus;
            targetPos4.x = targetPos3.x + 20.0 * cosinus; 
            targetPos4.a = targetPos3.a - 10.0;

            sweepState = FSM_SWEEP_NAV_RIGHT;
            break;
        }
        case FSM_SWEEP_NAV_RIGHT:
        {
            nav_ret = navigationGoTo(targetPos1, false, false, false); //First Move
            snapClaws(false,false);
            moveServoAndWait(SERVO_NUM_6, 170, 200);
            if (!claws_done) claws_done = lowerClaws();
    
            if (nav_ret == NAV_DONE && claws_done){
                LOG_DEBUG("FSM_SWEEP_NAV_RIGHT: Moving to right of the stock at position (", targetPos1.x, ",", targetPos1.y, ") with angle ", angle);

                if (needToGoToWall){
                    targetPos1.y += 50.0 * sinus;
                    targetPos1.x += 50.0 * cosinus;
                    startTime = _millis();
                    needToGoToWall = false;
                    sweepState = FSM_SWEEP_WALL;
                    break;
                }
                sweepState = FSM_SWEEP_NAV_LEFT;
            }
            break;
        }
        case FSM_SWEEP_WALL:
        {
            nav_ret = navigationGoTo(targetPos1, false, true, false); // Go slowly to the wall
            if (nav_ret == NAV_DONE || ((_millis() - startTime > 1000) && startTime != 0)){ // If stuck > 1 second, we are against the wall
                LOG_INFO("SWEEP: arrived at wall");
                sweepState = FSM_SWEEP_NAV_LEFT;
            }
            break;
        }
        case FSM_SWEEP_NAV_LEFT:
        {
            nav_ret = navigationGoTo(targetPos2, false, true, false); // Second Move, Slow mode
            if (nav_ret == NAV_DONE){
                startTime = _millis();
                sweepState = FSM_SWEEP_PRE_COLLECT;
                LOG_DEBUG("FSM_SWEEP_NAV_LEFT: Moving to left of the stock " " at position (", targetPos2.x, ",", targetPos2.y, ") with angle ", targetPos2.a);
            }
            break;
        }
        case FSM_SWEEP_PRE_COLLECT:
        {
            nav_ret = navigationGoTo(targetPos3, false, true, false);
            if ((nav_ret == NAV_DONE) || ((_millis() - startTime > 2000) && startTime != 0)) {
                if (openClaws()){
                    snapClaws(false,false);
                    LOG_EXTENDED_DEBUG("FSM_SWEEP_PRE_COLLECT: Opened and closed claws to prepare for collection");
                    startTime = 0;
                    sweepState = FSM_SWEEP_COLLECT;
                }
                
            }
            break;
        }
        case FSM_SWEEP_COLLECT:
        {
            nav_ret = navigationGoTo(targetPos4, false, true, false);
            if ((nav_ret == NAV_DONE)) {
                moveServoAndWait(SERVO_NUM_6, 90, 200);
                if (rotateTwoBlocks(stockOrder)){
                    LOG_EXTENDED_DEBUG("FSM_SWEEP_COLLECT: Claws rotated for collection");
                    LOG_DEBUG("FSM_SWEEP_COLLECT: Stock collected");
                    startTime = 0;
                    sweepState = FSM_SWEEP_INIT; // reset
                    return FSM_RETURN_DONE;
                }
            }
            break;
        } 
    }
    return FSM_RETURN_WORKING;
}


//Si FSM_RETURN_ERROR faire openClaws(), moveServoAndWait(SERVO_NUM_6, 90, 200) et raiseClaws() pour lacher les blocs (seront proche ou dans dropzone)
ReturnFSM_t ActionFSM::VidangeDropZone(int dropzone){
    static position_t vidangeTarget;
    static position_t targetPos1,targetPos2, targetPos3, targetPos4;
    static double cosinus, sinus;
    static bool claws_done;

    switch(vidangeState){

        case FSM_VIDANGE_INIT:
        {
            LOG_INFO("VIDANGE : init");
            if (dropzone == -1){
                LOG_ERROR("VIDANGE: invalid dropzone");
                return FSM_RETURN_ERROR;
            }
            claws_done = false;
            getBestDropZonePosition(dropzone,vidangeTarget); // ou équivalent

            cosinus = cos(DEG_TO_RAD * vidangeTarget.a);
            sinus   = sin(DEG_TO_RAD * vidangeTarget.a);

            //va à 150mm à gauche de la dropZone
            targetPos1.x = vidangeTarget.x + 150.0 * cosinus;
            targetPos1.y = vidangeTarget.y - 150.0 * sinus;
            targetPos1.a = vidangeTarget.a;

            //va à 20mm à droite de la dropZone pour être sur d'avoir enlever tout les blocks de l'adversaire
            targetPos2.x = vidangeTarget.x - 20.0*cosinus;
            targetPos2.y = vidangeTarget.y + 20.0*sinus;
            targetPos2.a = vidangeTarget.a;

            //Se reculer de 100mm pour pas taper le stock
            targetPos3.x = vidangeTarget.x - 100.0*sinus;
            targetPos3.y = vidangeTarget.y - 100.0*cosinus;
            targetPos3.a = vidangeTarget.a;

            vidangeState = FSM_VIDANGE_NAV;
            break;
        }
        case FSM_VIDANGE_NAV:
        {
            nav_ret = navigationGoTo(targetPos1, true, false, false);
            moveServoAndWait(SERVO_NUM_6, 170, 200); // lower the little arm
            if (!claws_done) claws_done = lowerClaws();
    
            if (nav_ret == NAV_DONE && claws_done){
                LOG_DEBUG("FSM_VIDANGE_NAV done: lowered claws at dropzone ", dropzone, " (", targetPos1.x, ",", targetPos1.y, ",", targetPos1.a);
                claws_done = false;
                vidangeState = FSM_VIDANGE_NAV_CLEAR;
            }
            if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;
            break;
        }
        case FSM_VIDANGE_NAV_CLEAR:
        {
            nav_ret = navigationGoTo(targetPos2, true, false, false);
            if (nav_ret == NAV_DONE){
                moveServoAndWait(SERVO_NUM_6, 90, 200); //raise the little arm
                if (openClaws()){//Drop our bloc 
                    LOG_DEBUG("FSM_VIDANGE_NAV_CLEAR done");
                    vidangeState = FSM_VIDANGE_NAV_BACK;
                }
            }
            if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;
            break;
        }
        case FSM_VIDANGE_NAV_BACK:
        {
            nav_ret = navigationGoTo(targetPos3, true, false, false);
            if (!claws_done) claws_done = lowerClaws();

            if (nav_ret == NAV_DONE && claws_done){
                LOG_DEBUG("FSM_VIDANGE_NAV_BACK done");
                claws_done = false;
                vidangeState = FSM_VIDANGE_INIT;
                return FSM_RETURN_DONE;
            }
            if (nav_ret == NAV_ERROR) return FSM_RETURN_ERROR;
            break;
        }
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
    if (dropzone_num == -1 && stealStockState == FSM_GATHER_NAV && tableStatus.dropToStealExist()){
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
        LOG_ERROR("Nothing else to do waiting");
        runState = FSM_ACTION_WAIT;
    }else{
        /*********************** CONDITION POUR VOLER UN STOCK OU TAKE STOCK ****************************/
        if(closestSteal < closestStock){
            LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to steal a drop, going to FSM_ACTION_STEAL");
            LOG_GREEN_INFO("ACTION_STEAL: Next dropZone to steal: ", dropzone_num);
            runState = FSM_ACTION_STEAL;
            stock_num = -1;
            /*********************TESTING OPTION **********************************/
            tableStatus.startTime = _millis();
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


/*
* FSM de récupération du centre du robot
*/
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