#include "defs/tableState.hpp"
#include "actions/functions.h"
#include "utils/logger.hpp"

TableState::TableState(){
    pos_opponent.x = 0; pos_opponent.y = 0;
    robot.pos = {0, 0, 0};
    robot.colorTeam = NONE;
    strategy = 1;
    startTime = 0;

    reset();
}

TableState::~TableState(){}

void TableState::reset(){
    
    /* data Winter is comming */
}

int TableState::getScore()
{
    int totalScore = 0;
    if (isRobotInArrivalZone(robot.pos))
        totalScore += 10;
    return totalScore;
}

// Serialize tableState
void to_json(json& j, const TableState& ts) {
    j = json{
        {"pos_opponent", ts.pos_opponent},
        {"startTime", ts.startTime},
        {"robot", ts.robot},
        {"strategy", ts.strategy}
    };
}