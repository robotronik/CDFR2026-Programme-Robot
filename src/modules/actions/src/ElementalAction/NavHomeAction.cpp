#include "actions/ElementalAction/NavHomeAction.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h" //For nav_return_t & position_t
#include "navigation/pathfind.h"

NavHomeAction::NavHomeAction(TableState* tableState, DriveControl* drive){
    nom = "NavHome";
    duree = 2000.0f;
    this->tableState = tableState;
    this->drive = drive;
}

ReturnFSM_t NavHomeAction::run(){
    nav_return_t res = navigation.goTo(homePos, true);
    if (res == NAV_ERROR){
        errorManagement();
    }
    if (res == NAV_DONE){
        successManagement();
        done = true;
        return FSM_RETURN_DONE;
    }
    return FSM_RETURN_WORKING;
}

bool NavHomeAction::errorManagement(){
    LOG_ERROR("RETURN_TO_HOME: Navigation error");
    //homePos.y += (tableState->colorTeam == BLUE) ? 50 : -50; // recule un peu et retente
    return true;
}

bool NavHomeAction::successManagement(){
    LOG_GREEN_INFO("RETURN_TO_HOME: Done");
    return true;
}

bool NavHomeAction::stop(){
    drive->stopMotion();
    return true;
}

void NavHomeAction::reset(){
    done = false;
}

bool NavHomeAction::available(float &reward){
    // Retour déjà effectué : plus candidate tant qu'elle n'a pas été réarmée.
    if (done) return false;
    double path_length_mm;
    position_t path[100];
    if(!pathfind(drive->getPosition(), homePos, path, path_length_mm)){
        return false;
    }
    reward = duree/(float)value;
    return true;
}

bool NavHomeAction::fullBlock(){
    return true;
}

bool NavHomeAction::mouvementBlock(){
    return true;
}