#pragma once
#include "actions/ElementalAction/DropAction.hpp"


//#include "utils/logger.hpp"

DropAction::DropAction(TableState* state, DriveControl* drive, ActuatorsControl* actuators) {
    this->tableState = state;
    this->drive = drive;
    this->actuators = actuators;
    this->nom = "DropAction";
    reset();
}

ReturnFSM_t DropAction::run() {
    switch (state) {
        case DROP_STATE_MOVING:
            {
            nav_return_t nav_result = navigation.goTo(target, true);
            if(nav_result == NAV_DONE) {
                actuators->setPumpState(false);
                return FSM_RETURN_DONE;
            }else if (nav_result == NAV_ERROR) {
                return FSM_RETURN_ERROR;
            }
            break;
            }
    }
    return FSM_RETURN_WORKING;
}


bool DropAction::stop() {
    reset();
    return true;
}

void DropAction::reset() {
    target = {0, 0, 0};
    best_path_length = 1e6;
}

bool DropAction::available(float &reward) {
    reset();
    if (getBestDropZone()) {
        reward = (float)(WALL + 3)/best_path_length;
    }else{
        reward = 0.0f;
    }
    return true;
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
    position_t robot_pos = drive->getPosition();
    wallZone_t bestZone = tableState->wallZone[0];

    best_path_length = 1e6;
    position_t path[1024];
    bool valid = false;
    for(int i = 0; i < WALL_ZONE_COUNT; i++) {
        if (tableState->wallZone[i].nb == 0) {
            wallZone_t zone = tableState->wallZone[i];
            if((tableState->colorTeam == BLUE ? 1 : -1) * zone.pos.y > (tableState->colorTeam == BLUE ? 1 : -1) * bestZone.pos.y
                && pathfind(robot_pos, zone.pos, path, best_path_length)) {
                bestZone = zone;
                valid = true;
            }else if(zone.pos.y == bestZone.pos.y) {
                double zone_path_length;
                if(!pathfind(robot_pos, zone.pos, path, zone_path_length)){
                    zone_path_length = 1e6;
                }
                if(!valid && !pathfind(robot_pos, bestZone.pos, path, best_path_length)){
                    best_path_length = 1e6;
                }
                if(zone_path_length < best_path_length && zone_path_length != 1e6) {
                    bestZone = zone;
                    valid = true;
                    best_path_length = zone_path_length;
                }else if(best_path_length != 1e6){
                    valid = true;
                }
            }else if(!valid){
                if(pathfind(robot_pos, zone.pos, path, best_path_length)){
                    bestZone = zone;
                    valid = true;
                }
            }
        }
    }
    if(valid){
        target = bestZone.pos;
    }
    return valid;
}

bool DropAction::toBestDropPosition(){
    // TODO change for proper offset depending on actuator
    // TODO 2 placement in area should be more dependent on strategy and not fix
    target.x = target.x + DROP_OFFSET.x * cos(target.a * DEG_TO_RAD) - DROP_OFFSET.y * sin(target.a * DEG_TO_RAD);
    target.y = target.y + DROP_OFFSET.x * sin(target.a * DEG_TO_RAD) + DROP_OFFSET.y * cos(target.a * DEG_TO_RAD);
    target.a = target.a + DROP_OFFSET.a;
    return true;
}
