#include "actions/ElementalAction/NavHomeAction.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h" //For NavReturn & position_t
#include "navigation/pathfind.h"

NavHomeAction::NavHomeAction(TableState* tableState, DriveControl* drive){
    nom = "NavHome";
    duree = 2000.0f;
    this->tableState = tableState;
    this->drive = drive;
}

ReturnFSM NavHomeAction::run(){
    NavReturn res = navigation.goTo(homePos, true);
    if (res == NavReturn::Error){
        errorManagement();
    }
    if (res == NavReturn::Done){
        successManagement();
        done = true;
        return ReturnFSM::Done;
    }
    return ReturnFSM::Working;
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
    position_t path[1024];
    if(!pathfind(drive->getPosition(), homePos, path, path_length_mm)){
        return false;
    }
    reward = (float)value/(float)path_length_mm;
    return true;
}

bool NavHomeAction::fullBlock(){
    return true;
}

bool NavHomeAction::mouvementBlock(){
    return true;
}