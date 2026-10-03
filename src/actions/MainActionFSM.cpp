#include <string>
#include "actions/MainActionFSM.hpp"
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "utils/logger.hpp"

ActionFSM::ActionFSM(DriveControl* drive, TableState* tableState)
    : driveControl(drive), tableState(tableState)
{
    Reset();
}

ActionFSM::ActionFSM(VirtualStrategy* strategy, DriveControl* drive, TableState* tableState)
    : driveControl(drive), tableState(tableState), currentStrategy(strategy)
{
    Reset();
}

ActionFSM::~ActionFSM(){}

void ActionFSM::setStrategy(VirtualStrategy* strategy){
    currentStrategy = strategy;
    // On abandonne l'action en cours : elle appartient à l'ancienne
    // stratégie, qui peut disparaître à tout moment.
    currentAction = nullptr;
}

void ActionFSM::Reset(){
    /****** RESET OF FSM STATES *******/
    if (currentStrategy != nullptr){
        // Réactive la stratégie si elle avait été stoppée (stop()).
        // NB: si la stratégie a besoin de reconstruire son pool d'actions
        // (cf. ExempleStrat::reset()), c'est à l'appelant de le faire
        // explicitement, VirtualStrategy n'imposant pas cette méthode.
        currentStrategy->resume();
    }

    // On démarre par une calibration forcée
    currentAction = nullptr;
}

/*
    Boucle principale : on récupère la meilleure action à exécuter
    (si aucune n'est en cours) puis on la lance.
    TODO: une seule action à la fois pour l'instant une parallélisation est envisageable à réfléchir
*/
bool ActionFSM::RunFSM(){
    // Sélection d'une nouvelle action : au démarrage, ou tant qu'on tourne sur
    // l'action de temporisation (on resonde alors la stratégie à chaque tick
    // pour voir si elle propose mieux). Réassigner la même action ne relance
    // pas le chrono : il s'agit de la même exécution qui se poursuit.
    if (currentAction == nullptr || currentAction == currentStrategy->tempAction()){
        VirtualAction* best = SetBestAction();
        if (best != currentAction){
            currentAction = best;
            actionStartTime = _millis();
        }
    }

    if (currentAction == nullptr){
        // Aucune action à exécuter : pas de stratégie branchée, ou pas
        // même d'action de temporisation disponible.
        return false;
    }

    ReturnFSM_t ret = currentAction->run();

    if (ret == FSM_RETURN_DONE){
        // Action menée à son terme : on enregistre le temps qu'elle a
        // réellement pris.
        durationDB.record(currentAction->getNom(), _millis() - actionStartTime);
        currentAction = SetBestAction();
        actionStartTime = _millis();
    }
    else if (ret == FSM_RETURN_ERROR){
        // TODO handle database
        currentAction = SetBestAction();
        actionStartTime = _millis();
    }

    return false;
}

/*
    Wrapper pour déterminer la meilleure action à exécuter
    Permet d'ajouter des modes comme endless mode
*/
VirtualAction* ActionFSM::SetBestAction(){
    //ENDLESSMODE
    if (endlessMod){
        if (_millis() > tableState->startTime + 50000) tableState->startTime = _millis();
    }

    /************************** DEMANDE À LA STRATÉGIE COURANTE *************************/
    if (currentStrategy != nullptr){
        // Pointeur non-possédant : l'action reste la propriété de la
        // stratégie, le FSM ne la libère jamais.
        VirtualAction* best = currentStrategy->bestAction();
        if (best != nullptr){
            LOG_GREEN_INFO("ActionFSM: exécution de l'action de stratégie '",
                            best->getNom().c_str(), "'");
            return best;
        }
        // bestAction() a renvoyé nullptr : stratégie arrêtée (stop()) ou
        // aucune action disponible. On retombe sur l'attente ci-dessous.
    }

    /************************** SINON, ON ATTEND *************************/
    if (currentStrategy == nullptr) return nullptr;
    return currentStrategy->tempAction();
}