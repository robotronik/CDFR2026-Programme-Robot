#include "actions/ElementalAction/TakeStone.hpp"
#include "defs/constante.h"
#include "defs/structs.hpp"
#include "defs/tableState.hpp"
#include "actions/RobotControl.hpp"
#include "drive_interface.h"
#include "navigation/driveControl.h" // pour position_t
#include "navigation/navigation.h" // pour nav_return_t & position_t
#include "navigation/pathfind.h"
#include "utils/logger.hpp"

TakeStone::TakeStone(TableState* tableState, RobotControl* robot)
    : tableState(tableState), robot(robot)
{
    nom = "TakeStone";
    reset();
}

float TakeStone::getBestQuarry(const position_t& from) {
    if (!tableState->remainQuarry()) {
        return -1;
    }

    quarry_t* best = nullptr;
    position_t bestTarget{0, 0, 0};
    int bestApproach = 0;
    int bestStones = -1;
    double minDistance = INFINITY;
    double minCost = INFINITY;

    position_t path[1024]; // Assuming a maximum path length
    for (int i = 0; i < QUARRY_ZONE_COUNT; i++) {
        quarry_t* candidate = &tableState->quarry[i];
        if (!candidate->nb) {
            continue;
        }

        position_t targets[2];
        const int targetCount = BestPositionToTake(candidate->pos, targets);
        for (int j = 0; j < targetCount; j++) {
            double distance;
            if (!pathfind(from, targets[j], path, distance)) {
                continue;
            }
            position_t stone = candidate->pos;
            if (j == 1) stone.a -= 180;
            if (distance < minCost && candidate->nb >= bestStones) {
                minCost = distance;
                minDistance = distance;
                best = candidate;
                bestTarget = targets[j];
                bestApproach = j;
                bestStones = candidate->nb;
            }
        }
    }

    if (best != nullptr) {
        quarry = best;
        target = bestTarget;
        position_t stone = best->pos;
        if (bestApproach == 1) stone.a -= 180;
        diffpos =RobotPosSuctionCupOffset (RobotTakeOffset(stone, from));//J'ai le droit
        return (float)minDistance;
    }
    return -1;
}

//TODO ajouter offset de 5.5cm du bloc fait

position_t TakeStone::RobotTakeOffset(const  position_t stone, const position_t& from) const {
    const int offsetStone = -150;//offset a regler
    position_t stonePos;
    double angle;
    if( stone.y == 0 ) {
        angle = (stone.a + 180) * DEG_TO_RAD;
    }
    else {
        angle = stone.a * DEG_TO_RAD;
    }
    stonePos.a = stone.a;
    stonePos.x = stone.x - offsetStone * cos(angle);
    stonePos.y = stone.y + offsetStone * sin(angle);
    return stonePos;
}
position_t TakeStone::RobotPosSuctionCupOffset(const position_t& stone) const {
    const position_t offsetface = selected_face->getOffset();
    position_t face;
    face.a = stone.a - offsetface.a;
    const double angle = face.a * DEG_TO_RAD;
    face.x = stone.x - offsetface.x * cos(angle) + offsetface.y * sin(angle);
    face.y = stone.y - offsetface.x * sin(angle) - offsetface.y * cos(angle);
    return face;
}

int TakeStone::BestPositionToTake(const position_t& stone, position_t targets[2]) const {
    const position_t face = RobotPosSuctionCupOffset(stone);

    targets[0] = face;
    if (stone.x == 950) {
        targets[0].x -= OFFSET_TARGET.x;
        return 1;
    }
    if (stone.x == -950) {
        targets[0].x += OFFSET_TARGET.x;
        return 1;
    }
    else {
       targets[0].y += OFFSET_TARGET.y;
        position_t oppositeStone = stone;
        oppositeStone.a -= 180;
        targets[1] = RobotPosSuctionCupOffset(oppositeStone);
        targets[1].y -= OFFSET_TARGET.y;
        return 2;  
    }
}

float TakeStone::BestTake(const position_t& from) {
    return getBestQuarry(from);//TODO ajouter le vole des bloques ou la recuperation des bloques hors carriere
}

ReturnFSM TakeStone::run(){
    switch (state) {
        case TakeStoneState::MOVE1: {
            NavReturn res1 = navigation.goTo(target, true, false, false);
            switch (res1) {
                case NavReturn::Paused:
                    LOG_DEBUG("TakeStone: nav is paused");
                    break;
                case NavReturn::Error:
                    errorManagement();
                    return ReturnFSM::Error;
                case NavReturn::Done:
                    approachStartTime = _millis();
                    state = TakeStoneState::MOVE2;
                    break;
                case NavReturn::InProcess:
                    break;
            }
            break;
        }
        case TakeStoneState::MOVE2:
            if (_millis() - approachStartTime >= 1500UL) {
                successManagement();
                return ReturnFSM::Done;
            }
            selected_face->wTake(); //TODO need to implement face selection
            NavReturn res2 = navigation.goTo(diffpos, false, true, true);//TODO fast mode
            switch (res2) {
                case NavReturn::Paused:
                    LOG_DEBUG("TakeStone: nav is paused");
                    break;
                case NavReturn::Error:
                    errorManagement();
                    return ReturnFSM::Error;
                case NavReturn::Done:
                    successManagement();
                    return ReturnFSM::Done;
                case NavReturn::InProcess:
                    break;

            } 
            break;
    }
    return ReturnFSM::Working;

}


bool TakeStone::stop(){
    robot->drive.stopMotion();
    return true;
}

bool TakeStone::errorManagement(){
    LOG_ERROR("TakeStone: erreur de navigation vers ", nom.c_str());
    reset();
    return true;
}

bool TakeStone::successManagement(){
    LOG_GREEN_INFO("TakeStone: arrivé à ", nom.c_str());
    quarry->nb = 0;
    selected_face->stones = 3;
    reset();
    return true;
}

void TakeStone::reset(){
    stop();
    selected_face = &robot->wall1;
    value = 23; // TODO change to be max value of points the opponent can make with 3 stones
    state = TakeStoneState::MOVE1;
    approachStartTime = 0;
    quarry = nullptr;
    target = {0, 0, 0};
    diffpos = {0, 0, 0};
}

bool TakeStone::available(float &reward){
    if(selected_face->stones!=0) return false;
    bool hasDropSpace = false;
    for (const auto& zone : tableState->wallZone) {
        if (zone.colorTeam == tableState->colorTeam && zone.nb == 0) {
            hasDropSpace = true;
            break;
        }
    }
    if (!hasDropSpace) return false;
    float dist = BestTake(robot->drive.getPosition());
    if (dist != -1) {
        reward = value/dist;
        return true;
    }
    return false;
}

bool TakeStone::fullBlock(){
    return false;
}

bool TakeStone::mouvementBlock(){
    return true; //the action always blocks the movement of the opponent
}
