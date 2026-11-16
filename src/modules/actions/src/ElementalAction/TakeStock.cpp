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
    state = MOVE1;
    selected_face = &robot->wall1;
    quarry = nullptr;
    nom = "TakeStock";
}

float TakeStock::getBestQuarry(const position_t& from) {
    if (!tableState->remainQuarry()) {
        return -1;
    }

    quarry_t* closest = &tableState->quarry[0];
    double minDistance=INFINITY;

    position_t path[1024]; // Assuming a maximum path length
    if(!pathfind(from, {closest->pos.x, closest->pos.y, closest->pos.a}, path, minDistance)) {
        minDistance = INFINITY;
    }
    for (int i = 0; i < QUARRY_ZONE_COUNT; i++) {
        quarry_t* quarry = &tableState->quarry[i];
        double distance; 
        if(!pathfind(from, {quarry->pos.x, quarry->pos.y, quarry->pos.a}, path, distance)) {
            distance = INFINITY;
        }
        if (distance < minDistance  && quarry->nb>=closest->nb) {
            minDistance = distance;
            closest = quarry;
        }
    }
    if (minDistance != INFINITY) {
        quarry = closest;
        target = closest->pos;
        return (float)minDistance;
    }
    return -1;
}

void TakeStock::BestPositionToTake(const position_t& from) {
    const position_t stock = target;
    const position_t offset = selected_face->getOffset();
    diffpos.a = stock.a - offset.a;
    const double angle = diffpos.a * DEG_TO_RAD;
    diffpos.x = stock.x - offset.x * cos(angle) + offset.y * sin(angle);
    diffpos.y = stock.y - offset.x * sin(angle) - offset.y * cos(angle);

    target = diffpos;
    if (stock.x == 950) {
        target.x -= OFFSET_TARGET.x;
    }
    else if (stock.x == -950) {
        target.x += OFFSET_TARGET.x;
    }
    else if (from.y >= stock.y) {
        target.y += OFFSET_TARGET.y;
    }
    else {
        target.y -= OFFSET_TARGET.y;
    }
}

float TakeStock::BestTake(const position_t& from) {
    return getBestQuarry(from);//TODO ajouter le vole des bloques ou la recuperation des bloques hors carriere
}

ReturnFSM TakeStock::run(){
    switch (state) {
        case MOVE1: {
            NavReturn res1 = navigation.goTo(target, true, false, false);
            switch (res1) {
                case NavReturn::Paused:
                    LOG_DEBUG("TakeStock: nav is paused");
                    break;
                case NavReturn::Error:
                    errorManagement();
                    return ReturnFSM::Error;
                case NavReturn::Done:
                    state = MOVE2;
                    break;
                case NavReturn::InProcess:
                    break;
            }
            break;
        }
        case MOVE2:
            selected_face->wTake(); //TODO need to implement face selection
            NavReturn res2 = navigation.goTo(diffpos, true, true, true);//TODO fast mode
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
    if(robot->drive.velocity.a != 0 || robot->drive.velocity.x != 0 || robot->drive.velocity.y != 0){
        robot->drive.stopMotion();
    }
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
    value = 23; // TODO change to be max value of points adv can make with 3 rocks
    state = MOVE1;

    target = {0, 0, 0};
    diffpos = {0, 0, 0};
}

bool TakeStock::available(float &reward){
    if(selected_face->rocks!=0) return false;
    float dist = BestTake(robot->drive.getPosition());
    BestPositionToTake(robot->drive.getPosition());
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
