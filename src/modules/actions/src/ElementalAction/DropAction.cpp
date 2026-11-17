#include "actions/ElementalAction/DropAction.hpp"
#include "utils/logger.hpp"
#include "utils/utils.h"
#include <cmath>


//#include "utils/logger.hpp"

DropAction::DropAction(TableState* tableState, RobotControl* robot)
    : tableState(tableState), robot(robot) 
{
    this->nom = "DropAction";
    reset();
}

ReturnFSM DropAction::run() {
    switch (state) {
        case DropState::Moving:
            {
            NavReturn nav_result = navigation.goTo(target, true);
            if(nav_result == NavReturn::Done) {
                stop();
                robot->wall1.wDrop();
                releaseStartTime = _millis();
                state = DropState::Releasing;
            }else if (nav_result == NavReturn::Error) {
                LOG_ERROR("Drop Error");
                reset();
                return ReturnFSM::Error;
            }
            break;
            }
        case DropState::Releasing:
            // Wait for the release to complete
            if (_millis() - releaseStartTime >= RELEASE_WAIT_MS) {
                successManagement();
                LOG_GREEN_INFO("Drop Success");
                return ReturnFSM::Done;
            }
            break;
    }
    return ReturnFSM::Working;
}

bool DropAction::successManagement(){
    selected_face->rocks = 0;
    selected_drop->nb = 3;
    return true;
}

bool DropAction::stop() {
    robot->drive.stopMotion();
    return true;
}

void DropAction::reset() {
    stop();
    state = DropState::Moving;
    releaseStartTime = 0;
    target = {0, 0, 0};
    best_path_length = INFINITY;
    selected_face = &robot->wall1;// TODO implement a way to chose face
    selected_drop = nullptr;
}

bool DropAction::available(float &reward) {
    reset();
    if(selected_face->rocks == 0) return false;
    if (getBestDropZone()) {
        toBestDropPosition();
        reward = (float)static_cast<int>(ConstructionType::Wall)/best_path_length;
        return true;
    }else{
        reward = 0.0f;
        return false;
    }
}

bool DropAction::fullBlock() {
    // Implementation for checking if the drop action is a full block
    return true;
}

bool DropAction::mouvementBlock() {
    // Implementation for checking if the drop action is a movement block
    return true;
}

bool DropAction::getBestDropZone(){
    // TODO check if has block to drop
    if (!tableState->remainEmptyWall()) return false;
    if(!tableState->needWall()) return false;
    position_t robot_pos = robot->drive.getPosition();
    selected_drop = nullptr;

    best_path_length = INFINITY;
    position_t path[1024];
    const int side = tableState->colorTeam == ColorTeam::Blue ? 1 : -1;
    for(int i = 0; i < WALL_ZONE_COUNT; i++) {
        wallZone_t* zone = &tableState->wallZone[i];
        if (zone->nb != 0 || zone->colorTeam != tableState->colorTeam) continue;

        double zone_path_length;
        if (!pathfind(robot_pos, zone->pos, path, zone_path_length)) continue;

        if (selected_drop == nullptr || side * zone->pos.y > side * selected_drop->pos.y ||
            (zone->pos.y == selected_drop->pos.y && zone_path_length < best_path_length)) {
            selected_drop = zone;
            best_path_length = zone_path_length;
        }
    }
    if(selected_drop != nullptr){
        target = selected_drop->pos;
    }
    return selected_drop != nullptr;
}

bool DropAction::toBestDropPosition(){
    // TODO change for proper offset depending on actuator
    // TODO 2 placement in area should be more dependent on strategy and not fix
    target.a = target.a - selected_face->getOffset().a;
    target.x = target.x - selected_face->getOffset().x * cos(target.a * DEG_TO_RAD) + selected_face->getOffset().y * sin(target.a * DEG_TO_RAD);
    target.y = target.y - selected_face->getOffset().x * sin(target.a * DEG_TO_RAD) - selected_face->getOffset().y * cos(target.a * DEG_TO_RAD);
    return true;
}
