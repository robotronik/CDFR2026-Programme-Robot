#include <string>
#include "actions/MainActionFSM.hpp"
#include "actions/ElementalAction/WaitAction.hpp"
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "utils/logger.hpp"
#include "main.hpp"
#include "defs/constante.h"
#include "mat/mat.hpp"

ActionFSM::ActionFSM(){
    Reset();
}

ActionFSM::ActionFSM(VirtualStrategy* strategy)
    : currentStrategy(strategy)
{
    Reset();
}

ActionFSM::~ActionFSM(){}

void ActionFSM::setStrategy(VirtualStrategy* strategy){
    currentStrategy = strategy;
    // On abandonne toute action de stratégie en cours issue de l'ancienne
    // stratégie : elle n'a plus de sens une fois la stratégie changée.
    strategyAction.reset();
}

void ActionFSM::Reset(){
    /****** RESET OF FSM STATES *******/
    waitAction.reset();
    calibrationAction.reset();
    navHomeAction.reset();
    strategyAction.reset();

    if (currentStrategy != nullptr){
        // Réactive la stratégie si elle avait été stoppée (stop()).
        // NB: si la stratégie a besoin de reconstruire son pool d'actions
        // (cf. ExempleStrat::reset()), c'est à l'appelant de le faire
        // explicitement, VirtualStrategy n'imposant pas cette méthode.
        currentStrategy->resume();
    }

    // On démarre par une calibration forcée
    currentAction = &calibrationAction;
}

/*
    Boucle principale : on récupère la meilleure action à exécuter
    (si aucune n'est en cours) puis on la lance.
*/
bool ActionFSM::RunFSM(){
    if (currentAction == nullptr || currentAction == &waitAction){
        currentAction = SetBestAction();
    }

    ReturnFSM_t ret = currentAction->run();

    if (ret == FSM_RETURN_DONE){
        if (currentAction == &navHomeAction){
            LOG_INFO("ACTION_NAV_HOME: Finished going home");
            currentAction = nullptr;
            return true; // Robot is done
        }
        //TODO handle database
        currentAction = SetBestAction();
    }
    else if (ret == FSM_RETURN_ERROR){
        // TODO handle database
        currentAction = SetBestAction();
    }

    return false;
}

/*
    Plus l'action est prioritaire plus elle apparaît tôt dans le code.
        Ex: le retour êtant prioritaire sur toutes les autres actions on fera toujours le retour si les conditions sont remplies
    Priorités actuelles:
        - Retour
        - Calibration
        - Meilleure action de la stratégie courante (currentStrategy)
        - Attente (si rien d'autre n'est disponible)
*/
VirtualAction* ActionFSM::SetBestAction(){
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
        strategyAction.reset(); // on abandonne l'action de stratégie en cours, si il y en a une
        return &navHomeAction;
    }

    /************************** CONDITIONS SUR LA CALIBRATION *************************/
    if(tableStatus.calibrationAge >= CALIBRATION_DEPLETION_TIME){
        LOG_GREEN_INFO("Calibration aged is greater than 2 going for forced calibration");
        return &calibrationAction;
    }

    /************************** DEMANDE À LA STRATÉGIE COURANTE *************************/
    if (currentStrategy != nullptr){
        strategyAction = currentStrategy->bestAction();
        if (strategyAction != nullptr){
            LOG_GREEN_INFO("ActionFSM: exécution de l'action de stratégie '",
                            strategyAction->getNom().c_str(), "'");
            return strategyAction.get();
        }
        // bestAction() a renvoyé nullptr : stratégie arrêtée (stop()) ou
        // pool d'actions épuisé. On retombe sur l'attente ci-dessous.
    }

    /************************** SINON, ON ATTEND *************************/
    return &waitAction;
}