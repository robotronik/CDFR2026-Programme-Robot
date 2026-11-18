#include "actions/ElementalAction/TakeStock.hpp"
#include "defs/constante.h"
#include "defs/structs.hpp"
#include "defs/tableState.hpp"
#include "actions/RobotControl.hpp"
#include "drive_interface.h"
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
    int bestApproach = 0;
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
                bestApproach = j;
                bestRocks = candidate->nb;
            }
        }
    }

    if (best != nullptr) {
        quarry = best;
        target = bestTarget;
        position_t stock = best->pos;
        if (bestApproach == 1) stock.a -= 180;
        diffpos =RobotPosSuctionCupOffset (RobotTakeOffset(stock, from));//J'ai le droit
        return (float)minDistance;
    }
    return -1;
}

//TODO ajouter offset de 5.5cm du bloc fait

position_t TakeStock::RobotTakeOffset(const  position_t stock, const position_t& from) const {
    const int offsetBlock = -150;//offset a regler
    position_t block;
    double angle;
    if( stock.y == 0 ) {
        angle = (stock.a + 180) * DEG_TO_RAD;
    }
    else {
        angle = stock.a * DEG_TO_RAD;
    }
    block.a = stock.a;
    block.x = stock.x - offsetBlock * cos(angle);
    block.y = stock.y + offsetBlock * sin(angle);
    return block;
}
position_t TakeStock::RobotPosSuctionCupOffset(const position_t& stock) const {
    const position_t offsetface = selected_face->getOffset();
    position_t face;
    face.a = stock.a - offsetface.a;
    const double angle = face.a * DEG_TO_RAD;
    face.x = stock.x - offsetface.x * cos(angle) + offsetface.y * sin(angle);
    face.y = stock.y - offsetface.x * sin(angle) - offsetface.y * cos(angle);
    return face;
}

int TakeStock::BestPositionToTake(const position_t& stock, position_t targets[2]) const {
    const position_t face = RobotPosSuctionCupOffset(stock);

    targets[0] = face;
    if (stock.x == 950) {
        targets[0].x -= OFFSET_TARGET.x;
        return 1;
    }
    if (stock.x == -950) {
        targets[0].x += OFFSET_TARGET.x;
        return 1;
    }
    else {
       targets[0].y += OFFSET_TARGET.y;
        position_t oppositeStock = stock;
        oppositeStock.a -= 180;
        targets[1] = RobotPosSuctionCupOffset(oppositeStock);
        targets[1].y -= OFFSET_TARGET.y;
        return 2;  
    }
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

bool TakeStock::fullBlock(){
    return false;
}

bool TakeStock::mouvementBlock(){
    return true; //the action always blocks the movement of adversary
}
