#include "defs/tableState.hpp"
#include "actions/functions.h"
#include "utils/logger.hpp"
#include "main.hpp"

TableState::TableState(){
    pos_opponent.x = 3000; pos_opponent.y = 0; //si on detect pas l'adversaire, on se mettrait en slow mode proche de 0,0
    colorTeam = NONE;
    strategy = 1;
    startTime = 0;
    calibrationAge = 0;
    setCursorIsDone(false);
    CursorPos = {625, 1220, 45};
    if (tableStatus.colorTeam == YELLOW) position_robot_flip(CursorPos);
    reset();
}

TableState::~TableState(){}

void TableState::reset(){
    /* data Winter is comming */

    setCursorIsDone(true);
    CursorPos = {625, 1220, 45};
    if (tableStatus.colorTeam == YELLOW) position_robot_flip(CursorPos);

    resetCalibrationAge();
    for (int i = 0; i < STOCK_COUNT; i++)
        avail_stocks[i] = true;

    // Initialize all drop zones to the empty state
    for (int i = 0; i < DROPZONE_COUNT; i++)
        dropzone_states[i] = DROPZONE_EMPTY;
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

void TableState::setStockAsRemoved(int num){
    tableStatus.avail_stocks[num] = false;
    LOG_EXTENDED_DEBUG("Removed stock ", num);
}

void TableState::setDropzoneState(int dropzoneNum, TableState::dropzone_state_t state){
    if (dropzoneNum < 0 || dropzoneNum >= DROPZONE_COUNT) {
        LOG_EXTENDED_DEBUG("Attempted to set dropzone state with invalid index ", dropzoneNum);
        return;
    }
    dropzone_states[dropzoneNum] = state;
    LOG_EXTENDED_DEBUG("Set dropzone ", dropzoneNum, " state to ", state);
}

void TableState::setDropzoneAsError(int dropzoneNum){
    setDropzoneState(dropzoneNum, TableState::DROPZONE_ERROR);
}