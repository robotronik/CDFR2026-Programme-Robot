#include "actions/ElementalAction/TakeStock.hpp"
#include "defs/constante.h"
#include "defs/structs.hpp"
#include "defs/tableState.hpp"
#include "actions/RobotControl.hpp"
#include "navigation/driveControl.h" // pour position_t
#include "navigation/navigation.h" // pour nav_return_t & position_t
#include "navigation/pathfind.h"
#include "utils/logger.hpp"

TakeStock::TakeStock(TableState* tableState, RobotControl* robot)
    : tableState(tableState), robot(robot)
{
    nom = "TakeStock";
    reset();
}

float TakeStock::getBestQuarry(const position_t& from) {
    if (!tableState->remainQuarry()) {
        return -1;
    }

    quarry_t* best = nullptr;
    position_t bestTarget{0, 0, 0};
    int bestRocks = -1;
    double minDistance = INFINITY;

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
            if (distance < minDistance && candidate->nb >= bestRocks) {
                minDistance = distance;
                best = candidate;
                bestTarget = targets[j];
                bestRocks = candidate->nb;
            }
        }
    }

    if (best != nullptr) {
        quarry = best;
        target = bestTarget;
        diffpos = getFacePosition(best->pos);
        return (float)minDistance;
    }
    return -1;
}

position_t TakeStock::getFacePosition(const position_t& stock) const {
    const position_t offset = selected_face->getOffset();
    position_t face;
    face.a = stock.a - offset.a;
    const double angle = face.a * DEG_TO_RAD;
    face.x = stock.x - offset.x * cos(angle) + offset.y * sin(angle);
    face.y = stock.y - offset.x * sin(angle) - offset.y * cos(angle);
    return face;
}

int TakeStock::BestPositionToTake(const position_t& stock, position_t targets[2]) const {
    const position_t face = getFacePosition(stock);

    targets[0] = face;
    if (stock.x == 950) {
        targets[0].x -= OFFSET_TARGET.x;
        return 1;
    }
    if (stock.x == -950) {
        targets[0].x += OFFSET_TARGET.x;
        return 1;
    }
    targets[0].y += OFFSET_TARGET.y;
    targets[1] = face;
    targets[1].y -= OFFSET_TARGET.y;
    return 2;
}

float TakeStock::BestTake(const position_t& from) {
    return getBestQuarry(from);//TODO ajouter le vole des bloques ou la recuperation des bloques hors carriere
}

ReturnFSM TakeStock::run(){
    switch (state) {
        case TakeStockState::MOVE1: {
            NavReturn res1 = navigation.goTo(target, true, false, false);
            switch (res1) {
                case NavReturn::Paused:
                    LOG_DEBUG("TakeStock: nav is paused");
                    break;
                case NavReturn::Error:
                    errorManagement();
                    return ReturnFSM::Error;
                case NavReturn::Done:
                    approachStartTime = _millis();
                    state = TakeStockState::MOVE2;
                    break;
                case NavReturn::InProcess:
                    break;
            }
            break;
        }
        case TakeStockState::MOVE2:
            if (_millis() - approachStartTime >= 2000UL) {
                successManagement();
                return ReturnFSM::Done;
            }
            selected_face->wTake(); //TODO need to implement face selection
            NavReturn res2 = navigation.goTo(diffpos, false, true, true);//TODO fast mode
            switch (res2) {
                case NavReturn::Paused:
                    LOG_DEBUG("TakeStock: nav is paused");
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


bool TakeStock::stop(){
    robot->drive.stopMotion();
    return true;
}

bool TakeStock::errorManagement(){
    LOG_ERROR("TakeStock: erreur de navigation vers ", nom.c_str());
    reset();
    return true;
}

bool TakeStock::successManagement(){
    LOG_GREEN_INFO("TakeStock: arrivé à ", nom.c_str());
    quarry->nb = 0;
    selected_face->rocks = 3;
    reset();
    return true;
}

void TakeStock::reset(){
    stop();
    selected_face = &robot->wall1;
    value = 23; // TODO change to be max value of points adv can make with 3 rocks
    state = TakeStockState::MOVE1;
    approachStartTime = 0;
    quarry = nullptr;
    target = {0, 0, 0};
    diffpos = {0, 0, 0};
}

bool TakeStock::available(float &reward){
    if(selected_face->rocks!=0) return false;
    float dist = BestTake(robot->drive.getPosition());
    if (dist != -1) {
        reward = value/dist;
        return true;
    }
    return false;
}

bool TakeStock::fullBlock(){
    return false;
}

bool TakeStock::mouvementBlock(){
    return true; //the action always blocks the movement of adversary
}
