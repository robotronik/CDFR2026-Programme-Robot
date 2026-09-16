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
}

bool ActionFSM::RunFSM(){
    ReturnFSM_t ret;
    static long unsigned startTime = 0;

    switch (runState)
    {
    case FSM_ACTION_WAIT:
    {
        if (startTime == 0) startTime = _millis();
        if (_millis() - startTime > 500){
            SetBestAction();
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
            SetBestAction();
        }
        else if (ret == FSM_RETURN_ERROR){
            LOG_ERROR("ACTION_CALIBRATION: Couldn't do calibration action, going for next action");
            tableStatus.calibrationAge -=1; // On se donne une action supplémentaire avant de retenter la calibration
            SetBestAction();
        }
        break;

    /*
        Action retour sur zone de départ
        n'est run que si plus rien n'est possible sur la table ou si le temps est écoulé
    */
    case FSM_ACTION_NAV_HOME:
        if (returnToHome()){
            LOG_INFO("ACTION_NAV_HOME: Finished going home");
            runState = FSM_ACTION_WAIT;
            return true; // Robot is done
        }
        break;
    }
    return false;
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
void ActionFSM::SetBestAction(){
    if (tableStatus.mastStatus) {
        LOG_WARNING("Updating map status with mast information");
        if(getMapStatus()){ // Getting data from mast
            tableStatus.updateMapStatus();
        }else{
            LOG_ERROR("Failed to get map status from mast");
            tableStatus.mastStatus = false; // Don't try to get mast information for the rest of the match
        }
    }
    //ENDLESSMODE
    if (tableStatus.strategy == 4){
        if (_millis() > tableStatus.startTime + 50000) tableStatus.startTime = _millis();
    }

    /********************* CONDITIONS POUR LE RETURN HOME ***********************/
    if(_millis() > tableStatus.startTime + 80000){ // After 95 seconds, switch to NAV_HOME to be sure to be in the arrival zone at the end of the match, even if we are late on the strategy
        LOG_GREEN_INFO("80 seconds passed, switching to NAV_HOME");
        runState = FSM_ACTION_NAV_HOME;
        return;
    }

    /************************** CONDITIONS SUR LA CALIBRATION *************************/
    if(tableStatus.calibrationAge >= CALIBRATION_DEPLETION_TIME){
        runState = FSM_ACTION_CALIBRATION;
        LOG_GREEN_INFO("Calibration aged is greater than 2 going for forced calibration");
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