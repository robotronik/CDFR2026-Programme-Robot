#include <string>
#include <exception>
#include "actions/action.hpp"
#include "actions/functions.h"
#include "actions/strats.hpp"
#include "defs/tableState.hpp"
#include "utils/logger.hpp"
#include "main.hpp"
#include "defs/constante.h"
#include "mat/mat.hpp"

ActionFSM::ActionFSM(){
    Reset();
}

ActionFSM::~ActionFSM(){}

void ActionFSM::Reset(){
    /****** RESET OF FSM STATES *******/
    gatherStockState = FSM_GATHER_NAV;
    dropStockState = FSM_DROP_NONE;
    calibrationState = FSM_CALCULATION;

    /*RESET OF ACTION ID*/
    dropzone_num = -1;
    stock_num = -1;
    offset = 0;
    distToAction = 0;
    noStockCalibrationDone = false;
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
    //SetBestAction(drive.position);
}

bool ActionFSM::RunFSM(){
    ReturnFSM_t ret;
    static long unsigned startTime = 0;

    switch (runState)
    {
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
            tableStatus.calibrationAge += CALIBRATION_DEPLETION_TIME;
            stock_num = -1;
            drive.stopMotion();
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
            tableStatus.calibrationAge += CALIBRATION_DEPLETION_TIME;
            drive.stopMotion();
            if (raiseClaws()) SetBestAction(drive.position);

        }
        break;
    case FSM_ACTION_WAIT:
    {
        if (startTime == 0) startTime = _millis();
        if (_millis() - startTime > 500){
            SetBestAction(drive.position);
            startTime = 0;
        }
        break;
    }
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
    }
    return false;
}

ReturnFSM_t ActionFSM::TakeStock(){
    //LOG_INFO("TakeStock state: ", gatherStockState, " stock_num: ", stock_num);
    if (stock_num == -1 && gatherStockState == FSM_GATHER_NAV){
        LOG_ERROR("No stock to take");
        return FSM_RETURN_DONE;
    }

    double angle = RAD_TO_DEG*  position_angle(position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, stockOff.a} , stockPos);

    switch (gatherStockState){
        case FSM_GATHER_NAV:
            {
            //LOG_DEBUG("entering FSM_GATHER_NAV");
            position_t targetPos = position_t {stockPos.x + stockOff.x, stockPos.y + stockOff.y, angle};
            distToAction = position_distance(drive.position, targetPos);
            if (distToAction > D_THRESHOLD_LATERAL) targetPos.a = 0;

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

ReturnFSM_t ActionFSM::DropStock(){
    static bool drop2StockInOne = false;
    switch (dropStockState){
        case FSM_DROP_NONE:
        {
            rotate_done = false;
            drop2StockInOne = false;
            getBestDropZonePosition(dropzone_num, dropzonePos);
            if (dropzone_num == -1){
                getBestDropZonePosition(dropzone_num, dropzonePos, false, true);
                if (dropzone_num == -1) return FSM_RETURN_ERROR;
                drop2StockInOne = true;
            }
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
            position_t PretargetPos = dropzonePos;
            distToAction = position_distance(drive.position, dropzonePos);
            if (distToAction > D_THRESHOLD_LATERAL) PretargetPos.a = 0;

            if(distToAction <= AS_THRESHOLD && ADVERSARY_THRESH > position_distance(dropzonePos, tableStatus.pos_opponent)){
                nav_ret = navigationGoTo(PretargetPos, false);
            }else{
                nav_ret = navigationGoTo(PretargetPos, true);
            }
            
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
                if (dropzone_num == 3)  tableStatus.granaryAlreadyTaken[0] = true;
                if (dropzone_num == 8)  tableStatus.granaryAlreadyTaken[1] = true;
                if (dropzone_num == 10) tableStatus.granaryAlreadyTaken[2] = true;
                if (dropzone_num == 11) tableStatus.granaryAlreadyTaken[3] = true;
                LOG_EXTENDED_DEBUG("FSM_DROP: Stock ", stock_num, " dropped");
                backPos = drive.position;
                backPos.x -= 100 * cos(DEG_TO_RAD * drive.position.a);
                backPos.y -= 100 * sin(DEG_TO_RAD * drive.position.a);

                gatherStockState = FSM_GATHER_NAV;
                tableStatus.setDropzoneState(dropzone_num, (tableStatus.colorTeam == BLUE) ? TableState::DROPZONE_BLUE : TableState::DROPZONE_YELLOW);  

                //No more stock in hand
                stock_num = -1;
                dropzone_num = -1;
                for(size_t _ = 0; _<4 ; _++){
                    stockOrder[_] = (tableStatus.colorTeam == YELLOW) ? false : true;
                }
                dropStockState = FSM_DROP_NAV_BACK;
                if (drop2StockInOne){
                    drop2StockInOne = false;
                    dropStockState = FSM_DROP_NAV_FRONT;
                }

            }
            break;
        case FSM_DROP_NAV_FRONT:
        {
            position_t frontPos = dropzonePos;
            frontPos.x += 250 * cos(DEG_TO_RAD * dropzonePos.a);
            frontPos.y += 250 * sin(DEG_TO_RAD * dropzonePos.a);

            nav_ret = navigationGoTo(frontPos, false, true, false);
        
            if (nav_ret == NAV_DONE || nav_ret == NAV_ERROR) {
                LOG_EXTENDED_DEBUG("FSM_DROP_NAV_FRONT: Finished Drop Nav Front");
                backPos = drive.position;
                backPos.x -= 200 * cos(DEG_TO_RAD * drive.position.a);
                backPos.y -= 200 * sin(DEG_TO_RAD * drive.position.a);
                dropStockState = FSM_DROP_NAV_BACK;
            }

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
    //updateGranaryStock();
    std::vector<bool> stockStatus;
    std::vector<std::pair<int, int>> dropZoneStatus;
    if (tableStatus.mastStatus) {
        LOG_WARNING("Updating map status with mast information");
        if(getMapStatus(stockStatus, dropZoneStatus)){
            tableStatus.updateMapStatus(stockStatus, dropZoneStatus);
        }else{
            LOG_ERROR("Failed to get map status from mast");
            tableStatus.mastStatus = false; // Don't try to get mast information for the rest of the match
        }
    }
    //ENDLESSMODE
    if (tableStatus.strategy == 4){
        if (_millis() > tableStatus.startTime + 50000) tableStatus.startTime = _millis();
    }
    /*********************** RESET DES DISTANCES POUR BEST ACTIONS *********************/
    closestStock = INFINITY;
    closestSteal = INFINITY;

    /********************* CONDITIONS POUR LE RETURN HOME ***********************/
    if(_millis() > tableStatus.startTime + 80000){ // After 95 seconds, switch to NAV_HOME to be sure to be in the arrival zone at the end of the match, even if we are late on the strategy
        LOG_GREEN_INFO("80 seconds passed, switching to NAV_HOME");
        runState = FSM_ACTION_NAV_HOME;
        return;
    }

    /************************** CONDITIONS SUR LA CALIBRATION *************************/
    if(tableStatus.calibrationAge >= CALIBRATION_DEPLETION_TIME || ( !noStockCalibrationDone && !tableStatus.remainingStocksExist() && stock_num == -1)){
        if ( !noStockCalibrationDone && !tableStatus.remainingStocksExist()) noStockCalibrationDone = true;
        runState = FSM_ACTION_CALIBRATION;
        LOG_GREEN_INFO("Calibration aged is greater than 2 going for forced calibration");
        return;
    }
    /**************************** CONDITIONS POUR DROP UN STOCK ***************************************/
    LOG_ERROR("stock_num ; ", stock_num);
    if(stock_num != -1){ // On peut DROP à partir du moment où on a un stock
        runState = FSM_ACTION_DROP;
        tableStatus.calibrationAge += 1;
        LOG_GREEN_INFO("Best action for position (", position.x, ", ", position.y, ") is to drop a stock, going to FSM_ACTION_DROP");
        return;
    }

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

    if(closestSteal == INFINITY && closestStock == INFINITY){
        LOG_ERROR("Nothing else to do waiting");
        runState = FSM_ACTION_WAIT;
        return;
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