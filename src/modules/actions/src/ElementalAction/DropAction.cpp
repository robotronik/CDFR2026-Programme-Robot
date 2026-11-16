#include "actions/ElementalAction/DropAction.hpp"


//#include "utils/logger.hpp"

DropAction::DropAction(TableState* tableState, RobotControl* robot) {
    this->tableState = tableState;
    this->robot = robot;
    this->nom = "DropAction";
    // reset() s'appuie sur robot/tableState : il doit être appelé après
    // l'affectation des membres.
    reset();
}

ReturnFSM DropAction::run() {
    switch (state) {
        case DropState::Moving:
            {
            NavReturn nav_result = navigation.goTo(target, true);
            if(nav_result == NavReturn::Done) {
                robot->wall1.wDrop();
                selected_face->rocks = 0;
                return ReturnFSM::Done;
            }else if (nav_result == NavReturn::Error) {
                return ReturnFSM::Error;
            }
            break;
            }
    }
    return ReturnFSM::Working;
}


bool DropAction::stop() {
    reset();
    return true;
}

void DropAction::reset() {
    state = DropState::Moving;
    target = {0, 0, 0};
    best_path_length = 1e6;
    selected_face = &robot->wall1;// TODO implement a way to chose face
    selected_drop = nullptr;
}

bool DropAction::available(float &reward) {
    reset();
    if(selected_face->rocks == 0) return false;
    if (getBestDropZone()) {
        reward = (float)static_cast<int>(ConstructionType::Wall)/best_path_length;
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
    position_t robot_pos = robot->drive.getPosition();
    selected_drop = &tableState->wallZone[0];

    best_path_length = 1e6;
    position_t path[1024];
    bool valid = false;
    for(int i = 0; i < WALL_ZONE_COUNT; i++) {
        if (tableState->wallZone[i].nb == 0) {
            wallZone_t* zone = &tableState->wallZone[i];
            if((tableState->colorTeam == ColorTeam::Blue ? 1 : -1) * zone->pos.y > (tableState->colorTeam == ColorTeam::Blue ? 1 : -1) * selected_drop->pos.y
                && pathfind(robot_pos, zone->pos, path, best_path_length)) {
                selected_drop = zone;
                valid = true;
            }else if(zone->pos.y == selected_drop->pos.y) {
                double zone_path_length;
                if(!pathfind(robot_pos, zone->pos, path, zone_path_length)){
                    zone_path_length = 1e6;
                }
                if(!valid && !pathfind(robot_pos, selected_drop->pos, path, best_path_length)){
                    best_path_length = 1e6;
                }
                if(zone_path_length < best_path_length && zone_path_length != 1e6) {
                    selected_drop = zone;
                    valid = true;
                    best_path_length = zone_path_length;
                }else if(best_path_length != 1e6){
                    valid = true;
                }
            }else if(!valid){
                if(pathfind(robot_pos, zone->pos, path, best_path_length)){
                    selected_drop = zone;
                    valid = true;
                }
            }
        }
    }
    if(valid){
        target = selected_drop->pos;
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
