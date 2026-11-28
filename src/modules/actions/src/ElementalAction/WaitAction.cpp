#include "actions/ElementalAction/WaitAction.hpp"
#include "utils/utils.h"
//#include "utils/logger.hpp"

WaitAction::WaitAction(unsigned long tps){
    nom = "Wait";
    duree = (float)tps;
    wait = tps;
    startTime = 0;
}

ReturnFSM WaitAction::run(){
    if (startTime == 0) startTime = _millis();
    if (_millis() - startTime > WAIT_DURATION_MS){
        startTime = 0;
        return ReturnFSM::Done;
    }
    return ReturnFSM::Working;
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

bool WaitAction::available(float &reward){
    // Waiting is always available, but its cost is high since it is not worthwhile
    reward = -duree;
    return true;
}

/*Wait is not a full block we could imagine positionning to block/anticipate opponent action*/
bool WaitAction::fullBlock(){
    return false;
}

bool WaitAction::movementBlock(){
    return false;
}