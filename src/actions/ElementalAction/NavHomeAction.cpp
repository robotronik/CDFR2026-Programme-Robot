#include "ElementalAction/NavHomeAction.hpp"
#include "actions/functions.h"
#include "utils/logger.hpp"

NavHomeAction::NavHomeAction(){
    nom = "NavHome";
    duree = 0;
}

ReturnFSM_t NavHomeAction::run(){
    if (returnToHome()){
        return FSM_RETURN_DONE;
    }
    return FSM_RETURN_WORKING;
}

bool NavHomeAction::stop(){
    return true;
}

void NavHomeAction::reset(){
    // Rien à réinitialiser pour l'instant
}

float NavHomeAction::available(){
    return 0.0f;
}

bool NavHomeAction::fullBlock(){
    return true;
}

bool NavHomeAction::mouvementBlock(){
    return true;
}