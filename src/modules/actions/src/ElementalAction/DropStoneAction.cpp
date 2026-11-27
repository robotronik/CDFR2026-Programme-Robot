#include "actions/ElementalAction/DropStoneAction.hpp"
#include "utils/logger.hpp"
#include "utils/utils.h"
#include <cmath>

//#include "utils/logger.hpp"

DropStoneAction::DropStoneAction(TableState* tableState, RobotControl* robot)
    : tableState(tableState), robot(robot) {
    this->nom = "DropStone";
    reset();
}

ReturnFSM DropStoneAction::run() {
    switch (state) {
    case DropStoneState::Moving: {
        NavReturn nav_result = navigation.goTo(target, true);
        if (nav_result == NavReturn::Done) {
            stop();
            if (!selected_face->wDrop()) return ReturnFSM::Error; // In practice can't fail
            releaseStartTime = _millis();
            state = DropStoneState::Releasing;
        } else if (nav_result == NavReturn::Error) {
            LOG_ERROR("Drop Error");
            reset();
            return ReturnFSM::Error;
        }
        break;
    }
    case DropStoneState::Releasing:
        // Wait for the release to complete
        if (_millis() - releaseStartTime >= RELEASE_WAIT_MS) {
            // we close the air circuit
            // In practice can't fail
            if (!selected_face->wDisableElectro()) return ReturnFSM::Error;
            successManagement();
            LOG_GREEN_INFO("Drop Success");
            return ReturnFSM::Done;
        }
        break;
    }
    return ReturnFSM::Working;
}

bool DropStoneAction::successManagement() {
    selected_zone->nb = 3;
    reset();
    return true;
}

bool DropStoneAction::stop() {
    robot->drive.stopMotion();
    return true;
}

void DropStoneAction::reset() {
    state = DropStoneState::Moving;
    releaseStartTime = 0;
    target = {0, 0, 0};
    best_path_length = INFINITY;
    selected_face = &robot->gate1; // TODO implement a way to chose face
    selected_zone = nullptr;
}

bool DropStoneAction::available(float& reward) {
    if (!selected_face->hasStone()) return false;
    if (findBestConstructionZone()) {
        moveTargetToConstructionPosition();
        reward = (float)static_cast<int>(ConstructionType::Wall) / best_path_length;
        return true;
    } else {
        reward = 0.0f;
        return false;
    }
}

bool DropStoneAction::fullBlock() {
    // Implementation for checking if the stone drop action is a full block
    return true;
}

bool DropStoneAction::movementBlock() {
    // Implementation for checking if the stone drop action is a movement block
    return true;
}

bool DropStoneAction::findBestConstructionZone() {
    // TODO check if the robot carries stones to drop
    if (!tableState->hasEmptyWall()) return false;
    if (!tableState->countMissingWalls()) return false;
    position_t robot_pos = robot->drive.getPosition();
    selected_zone = nullptr;

    best_path_length = INFINITY;
    position_t path[1024];
    const int side = tableState->colorTeam == ColorTeam::Blue ? 1 : -1;
    for (int i = 0; i < WALL_ZONE_COUNT; i++) {
        wallZone_t* zone = &tableState->wallZone[i];
        if (zone->nb != 0 || zone->colorTeam != tableState->colorTeam) continue;

        double zone_path_length;
        if (!pathfind(robot_pos, zone->pos, path, zone_path_length)) continue;

        if (selected_zone == nullptr || side * zone->pos.y > side * selected_zone->pos.y ||
            (zone->pos.y == selected_zone->pos.y && zone_path_length < best_path_length)) {
            selected_zone = zone;
            best_path_length = zone_path_length;
        }
    }
    if (selected_zone != nullptr) {
        target = selected_zone->pos;
    }
    return selected_zone != nullptr;
}

bool DropStoneAction::moveTargetToConstructionPosition() {
    // TODO change for proper offset depending on actuator
    // TODO 2 placement in area should be more dependent on strategy and not fix
    target.a = target.a - selected_face->getOffset().a;
    target.x = target.x - selected_face->getOffset().x * cos(target.a * DEG_TO_RAD) + selected_face->getOffset().y * sin(target.a * DEG_TO_RAD);
    target.y = target.y - selected_face->getOffset().x * sin(target.a * DEG_TO_RAD) - selected_face->getOffset().y * cos(target.a * DEG_TO_RAD);
    return true;
}
