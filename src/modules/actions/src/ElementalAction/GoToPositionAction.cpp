#include "actions/ElementalAction/GoToPositionAction.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h" // pour nav_return_t & position_t
#include "navigation/pathfind.h"

GoToPositionAction::GoToPositionAction(const std::string& name, position_t target, DriveControl* drive)
    : target(target), drive(drive), moving(false)
{
    nom = name;
    duree = 20; // durée estimée, à ajuster
}

ReturnFSM_t GoToPositionAction::run(){
    nav_return_t res = navigation.goTo(target, true);

    switch (res) {
        case NAV_PAUSED:
            LOG_DEBUG("GoToPositionAction: nav is paused");
            moving = false;
            break;
        case NAV_ERROR:
            errorManagement();
            return FSM_RETURN_ERROR;
        case NAV_DONE:
            successManagement();
            return FSM_RETURN_DONE;
        case NAV_IN_PROCESS:
            moving = true;
            break;
    }
    return FSM_RETURN_WORKING;
}

bool GoToPositionAction::errorManagement(){
    LOG_ERROR("GoToPositionAction: erreur de navigation vers ", nom.c_str());
    reset();
    return true;
}

bool GoToPositionAction::successManagement(){
    LOG_GREEN_INFO("GoToPositionAction: arrivé à ", nom.c_str());
    moving = false;
    done = true;
    return true;
}

bool GoToPositionAction::stop(){
    if(moving){
        drive->stopMotion();
        moving = false;
    }
    return true;
}

void GoToPositionAction::reset(){
    stop();
    done = false;                  // l'action redevient candidate
}

bool GoToPositionAction::available(float &reward){
    // Déplacement déjà terminé (succès ou échec) : plus candidate tant
    // qu'elle n'a pas été réarmée par reset().
    if (done) return false;
    double path_length_mm;
    position_t path[100]; // Assuming a maximum path length
    if(!pathfind(drive->getPosition(), target, path, path_length_mm)){
        return false;
    }
    reward = -path_length_mm;
    return true;
}

bool GoToPositionAction::fullBlock(){
    return false;
}

bool GoToPositionAction::mouvementBlock(){
    return moving;
}