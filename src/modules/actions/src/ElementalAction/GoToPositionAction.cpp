#include "actions/ElementalAction/GoToPositionAction.hpp"
#include "actions/RobotControl.hpp"
#include "defs/tableState.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h" // pour NavReturn & position_t
#include "navigation/pathfind.hpp"

GoToPositionAction::GoToPositionAction(const std::string& name, position_t target, RobotControl* robot)
    : target(target), robot(robot), isMoving(false) {
    nom = name;
    value = 100;
    duree = 20; // durée estimée, à ajuster
}

GoToPositionAction::GoToPositionAction(const std::string& name, TableState* tableState, RobotControl* robot)
    : target({0, 0, 0}), robot(robot), tableState(tableState), needsTarget(true), isMoving(false) {
    nom = name;
    value = 100;
    duree = 20; // durée estimée, à ajuster
}

bool GoToPositionAction::findClosestElement(const position_t& from, position_t& target) const {
    const std::vector<MatGameElement>& elements = tableState->elements;
    if (elements.empty()) {
        return false;
    }

    const MatGameElement* closest = &elements.front();
    double minDistance = position_distance(from, {closest->x, closest->y, closest->a});

    for (const MatGameElement& element : elements) {
        const double distance = position_distance(from, {element.x, element.y, element.a});
        if (distance < minDistance) {
            minDistance = distance;
            closest = &element;
        }
    }

    target.x = closest->x;
    target.y = closest->y;
    target.a = closest->a;
    return true;
}

bool GoToPositionAction::hasResolvedTarget() {
    if (!needsTarget) {
        return true; // cible fixe, ou déjà résolue pour cette exécution
    }
    if (!findClosestElement(robot->drive.getPosition(), target)) {
        return false; // aucun élément de jeu connu
    }
    needsTarget = false;
    return true;
}

ReturnFSM GoToPositionAction::run() {
    if (!hasResolvedTarget()) {
        LOG_WARNING("GoToPositionAction: aucun élément de jeu connu pour ", nom.c_str());
        return ReturnFSM::Error;
    }

    NavReturn res = navigation.goTo(target, true);

    switch (res) {
    case NavReturn::Paused:
        LOG_DEBUG("GoToPositionAction: nav is paused");
        isMoving = false;
        break;
    case NavReturn::Error:
        errorManagement();
        return ReturnFSM::Error;
    case NavReturn::Done:
        successManagement();
        return ReturnFSM::Done;
    case NavReturn::InProcess:
        isMoving = true;
        break;
    }
    return ReturnFSM::Working;
}

bool GoToPositionAction::errorManagement() {
    LOG_ERROR("GoToPositionAction: erreur de navigation vers ", nom.c_str());
    reset();
    return true;
}

bool GoToPositionAction::successManagement() {
    LOG_GREEN_INFO("GoToPositionAction: arrivé à ", nom.c_str());
    isMoving = false;
    done = true;
    return true;
}

bool GoToPositionAction::stop() {
    if (isMoving) {
        robot->drive.stopMotion();
        isMoving = false;
    }
    return true;
}

void GoToPositionAction::reset() {
    stop();
    done = false; // l'action redevient candidate
    needsTarget = (tableState != nullptr); // re-cible l'élément le plus proche
}

bool GoToPositionAction::available(float& reward) {
    // Déplacement déjà terminé (succès ou échec) : plus candidate tant
    // qu'elle n'a pas été réarmée par reset().
    if (done) return false;
    // Cible dynamique : sans élément connu, l'action n'est pas jouable.
    if (!hasResolvedTarget()) return false;
    double path_length_mm;
    position_t path[1024]; // Assuming a maximum path length
    if (!pathfind(robot->drive.getPosition(), target, path, path_length_mm)) {
        return false;
    }
    reward = value / path_length_mm;
    return true;
}

bool GoToPositionAction::fullBlock() {
    return false;
}

bool GoToPositionAction::movementBlock() {
    return isMoving;
}