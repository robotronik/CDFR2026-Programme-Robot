#include <string>
#include "actions/MainActionFSM.hpp"
#include "actions/Robot.hpp"
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "utils/logger.hpp"

ActionFSM::ActionFSM(Robot* robot, TableState* tableState)
    : robotControl(robot), tableState(tableState) {
    Reset();
}

ActionFSM::~ActionFSM(){}

void ActionFSM::syncStrategy() {
    VirtualStrategy* strategy = robotControl ? robotControl->currentStrategy : nullptr;
    if (strategy == currentStrategy) return;
    // Abandon the running action: it belongs to the previous strategy, which
    // may disappear at any time.
    currentStrategy = strategy;
    currentAction = nullptr;
}

void ActionFSM::Reset(){
    /****** RESET OF FSM STATES *******/
    syncStrategy();
    if (currentStrategy != nullptr){
        // Resume the strategy if it had been stopped (stop()).
        // NB: if the strategy needs to rebuild its action pool (cf.
        // ExempleStrat::reset()), the caller must do it explicitly;
        // VirtualStrategy does not require this method.
        currentStrategy->resume();
    }

    currentAction = nullptr;
}

/*
    Main loop: pick the best action to run (if none is running) then run it.
    TODO: one action at a time for now; parallelism could be considered.
*/
bool ActionFSM::RunFSM(){
    syncStrategy();
    // Select a new action: at startup, or while running the temporary action
    // (the strategy is then re-polled every tick to see if it offers better).
    // Reassigning the same action does not restart the timer: it is the same
    // execution carrying on.
    if (currentAction == nullptr || currentAction == currentStrategy->tempAction()){
        VirtualAction* best = SetBestAction();
        if (best != currentAction){
            currentAction = best;
            actionStartTime = _millis();
        }
    }

    if (currentAction == nullptr){
        // No action to run: no strategy attached, or no temporary action
        // available either.
        return false;
    }

    ReturnFSM ret = currentAction->run();

    if (ret == ReturnFSM::Done){
        // Action completed: record how long it actually took.
        //durationDB.record(currentAction->getNom(), _millis() - actionStartTime);
        currentAction = SetBestAction();
        actionStartTime = _millis();
    }
    else if (ret == ReturnFSM::Error){
        currentAction = SetBestAction();
        actionStartTime = _millis();
    }

    return false;
}

/*
    Wrapper to pick the best action to run.
    Lets us add modes such as endless mode.
*/
VirtualAction* ActionFSM::SetBestAction(){
    //ENDLESSMODE
    if (isEndlessMode){
        if (_millis() > tableState->startTime + 50000) tableState->startTime = _millis();
    }

    /************************** ASK THE CURRENT STRATEGY *************************/
    if (currentStrategy != nullptr){
        // Non-owning pointer: the action stays owned by the strategy; the FSM
        // never frees it.
        VirtualAction* best = currentStrategy->bestAction();
        if (best != nullptr){
            LOG_GREEN_INFO("ActionFSM: running strategy action '",
                           best->getNom().c_str(), "'");
            return best;
        }
        // bestAction() returned nullptr: strategy stopped (stop()) or no action
        // available. Fall back to waiting below.
    }

    /************************** OTHERWISE, WAIT *************************/
    if (currentStrategy == nullptr) return nullptr;
    return currentStrategy->tempAction();
}