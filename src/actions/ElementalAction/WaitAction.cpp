#include "actions/ElementalAction/WaitAction.hpp"
#include "main.hpp" // for _millis()
//#include "utils/logger.hpp"

WaitAction::WaitAction(){
    nom = "Wait";
    duree = WAIT_DURATION_MS;
    startTime = 0;
}

ReturnFSM_t WaitAction::run(){
    if (startTime == 0) startTime = _millis();
    if (_millis() - startTime > WAIT_DURATION_MS){
        startTime = 0;
        return FSM_RETURN_DONE;
    }
    return FSM_RETURN_WORKING;
}

bool WaitAction::successManagement(){
    return true;
}

bool WaitAction::errorManagement(){
    return true;
}

bool WaitAction::stop(){
    startTime = 0;
    return true;
}

void WaitAction::reset(){
    startTime = 0;
}

float WaitAction::available(){
    // L'attente est toujours disponible, coût nul
    return 0.0f;
}

/*Wait is not a full block we could imagine positionning to block/anticipate adversary action*/
bool WaitAction::fullBlock(){
    return false;
}

bool WaitAction::mouvementBlock(){
    return false;
}