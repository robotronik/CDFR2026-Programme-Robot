#include "actions/ElementalAction/NavHomeAction.hpp"
#include "actions/RobotControl.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h" //For NavReturn & position_t
#include "navigation/pathfind.h"

NavHomeAction::NavHomeAction(TableState* tableState, RobotControl* robot, position_t homePos)
    : homePos(homePos) {
    nom = "NavHome";
    duree = 2000.0f;
    this->tableState = tableState;
    this->robot = robot;
}

ReturnFSM NavHomeAction::run() {
    NavReturn res = navigation.goTo(homePos, false);
    if (res == NavReturn::Error) {
        errorManagement();
    }
    if (res == NavReturn::Done) {
        successManagement();
        done = true;
        return ReturnFSM::Done;
    }
    return ReturnFSM::Working;
}

bool NavHomeAction::errorManagement() {
    LOG_ERROR("RETURN_TO_HOME: Navigation error");
    return true;
}

bool NavHomeAction::successManagement() {
    LOG_GREEN_INFO("RETURN_TO_HOME: Done");
    return true;
}

bool NavHomeAction::stop() {
    robot->drive.stopMotion();
    return true;
}

void NavHomeAction::reset() {
    done = false;
    homePos = position_t{0, 0, 0};
}

bool NavHomeAction::available(float& reward) {
    // Retour déjà effectué : plus candidate tant qu'elle n'a pas été réarmée.
    if (done) return false;
    if (_millis() - tableState->startTime < 90000) return false;
    double path_length_mm;
    position_t path[1024];
    LOG_INFO("position du robot x=%lf, y=%lf, a=%lf, position homePos x=%lf y=%lf a=%lf",
             robot->drive.getPosition().x, robot->drive.getPosition().y, robot->drive.getPosition().a,
             homePos.x, homePos.y, homePos.a);
    if (!pathfind(robot->drive.getPosition(), homePos, path, path_length_mm)) {
        return false;
    }
    reward = (float)value / (float)path_length_mm;
    return true;
}

bool NavHomeAction::fullBlock() {
    return true;
}

bool NavHomeAction::movementBlock() {
    return true;
}