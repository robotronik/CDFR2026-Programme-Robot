#include "actions/ElementalAction/GoToPositionAction.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h" // pour nav_return_t & position_t
#include "navigation/pathfind.h"

GoToPositionAction::GoToPositionAction(const std::string& name, position_t target, DriveControl* drive)
    : target(target), drive(drive), moving(false)
{
    nom = name;
    duree = 2000; // durée estimée, à ajuster
}

ReturnFSM_t GoToPositionAction::run(){
    nav_return_t res = navigationGoTo(target, true);

    switch (res) {
        case NAV_PAUSED:
            LOG_DEBUG("GoToPositionAction: nav is paused");
            moving = false;
            break;
        case NAV_ERROR:
            errorManagement();
            reset();
            return FSM_RETURN_ERROR;
        case NAV_DONE:
            successManagement();
            moving = false;
            return FSM_RETURN_DONE;
        case NAV_IN_PROCESS:
        default:
            moving = true;
            break;
    }
    return FSM_RETURN_WORKING;
}

bool GoToPositionAction::errorManagement(){
    LOG_ERROR("GoToPositionAction: erreur de navigation vers ", nom.c_str());
    return true;
}

bool GoToPositionAction::successManagement(){
    LOG_GREEN_INFO("GoToPositionAction: arrivé à ", nom.c_str());
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
    target = drive->getPosition(); // Reset target to current position
}

float GoToPositionAction::available(){
    // Un déplacement est toujours jouable tant que le contexte
    // (couleur/stratégie) est valide.
    double path_length_mm;
    position_t path[100]; // Assuming a maximum path length
    pathfind(drive->getPosition(), target, path, path_length_mm);
    return path_length_mm/ duree;
}

bool GoToPositionAction::fullBlock(){
    return false;
}

bool GoToPositionAction::mouvementBlock(){
    return moving;
}