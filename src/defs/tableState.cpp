#include "defs/tableState.hpp"
#include "actions/functions.h"
#include "utils/logger.hpp"
#include "main.hpp"

TableState::TableState(){
    pos_opponent.x = 0; pos_opponent.y = 0;
    colorTeam = NONE;
    strategy = 1;
    startTime = 0;

    reset();
}

TableState::~TableState(){}

void TableState::reset(){
    
    /* data Winter is comming */
    for (int i = 0; i < STOCK_COUNT; i++)
        avail_stocks[i] = true;
}

int TableState::getScore()
{
    int totalScore = 0;
    // TODO, should be "completely inside" and not just "in"
    if (isRobotInArrivalZone((position_t)drive.position))
        totalScore += 5;
    if (isRobotInArrivalZone((position_t)drive.position))
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