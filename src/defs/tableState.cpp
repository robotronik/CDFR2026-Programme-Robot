#include "defs/tableState.hpp"
#include "actions/functions.h"

TableState::TableState(DriveControl* drive){
    this->drive = drive;
    pos_opponent.x = 3000; pos_opponent.y = 0; //si on detect pas l'adversaire, on se mettrait en slow mode proche de 0,0
    colorTeam = NONE;
    strategy = 1;
    startTime = 0;
    calibrationAge = 0;
    reset();
}

TableState::~TableState(){}

void TableState::reset(){
    resetCalibrationAge();
    /* reset of specific The legend of Camelot */

}

int TableState::getScore()
{
    int totalScore = 0;
    // TODO, should be "completely inside" and not just "in"
    if (isRobotInArrivalZone((position_t)drive->getPosition()))
        totalScore += 5;
    return totalScore;
}

// Serialize tableState
void to_json(json& j, const TableState& ts) {
    j = json{
        {"pos_opponent", ts.pos_opponent},
        {"startTime", ts.startTime},
        {"colorTeam", ts.colorTeam},
        {"strategy", ts.strategy}
    };
}

void TableState::updateMapStatus(){
    // TODO use data from mast to update tableStatus

}