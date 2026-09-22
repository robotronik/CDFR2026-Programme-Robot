#include "actions/ElementalAction/NavHomeAction.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h" //For nav_return_t & position_t
#include "main.hpp" // for tableStatus

NavHomeAction::NavHomeAction(){
    nom = "NavHome";
    duree = 0;
}

ReturnFSM_t NavHomeAction::run(){
    nav_return_t res = navigationGoTo(homePos, true);
    if (res == NAV_ERROR){
        errorManagement();
    }
    if (res == NAV_DONE){
        successManagement();
        return FSM_RETURN_DONE;
    }
    return FSM_RETURN_WORKING;
}

bool NavHomeAction::errorManagement(){
    LOG_ERROR("RETURN_TO_HOME: Navigation error");
    homePos.y += (tableStatus.colorTeam == BLUE) ? 50 : -50; // recule un peu et retente
    return true;
}

bool NavHomeAction::successManagement(){
    LOG_GREEN_INFO("RETURN_TO_HOME: Done");
    return true;
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