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
    //mastStatus = false;

    setCursorIsDone(true);
    CursorPos = {780.0, 190.0, -180.0};
    if (tableStatus.colorTeam == YELLOW) position_robot_flip(CursorPos);

    resetCalibrationAge();
    for (int i = 0; i < STOCK_COUNT; i++){
        avail_stocks[i] = TIME_TO_TAKE;
    }

    colorTeamDropZone = (colorTeam == BLUE) ? DROPZONE_BLUE : DROPZONE_YELLOW;
    colorTeamDropZoneOpponent = (colorTeam == BLUE) ? DROPZONE_YELLOW : DROPZONE_BLUE;
    // Initialize all drop zones to the empty state
    for (int i = 0; i < DROPZONE_COUNT; i++){
        dropzone_states[i] = DROPZONE_EMPTY;
        dropzone_proba[i] = 0;
    }
    
    //Don't Drop on the Zone in front of the granary
    if (colorTeam == BLUE){
        dropzone_states[3] = colorTeamDropZone;
    }else{
        dropzone_states[8] = colorTeamDropZone;
    }

    for(int i = 0; i < 4; i++){
        granaryAlreadyTaken[i] = false;
    }
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
    tableStatus.avail_stocks[num] = 0;
    LOG_EXTENDED_DEBUG("Removed stock ", num);
}

void TableState::setDropzoneState(int dropzoneNum, TableState::dropzone_state_t state){
    if (dropzoneNum < 0 || dropzoneNum >= DROPZONE_COUNT) {
        LOG_EXTENDED_DEBUG("Attempted to set dropzone state with invalid index ", dropzoneNum);
        return;
    }
    if((tableStatus.colorTeam == BLUE && state == TableState::DROPZONE_BLUE) || (tableStatus.colorTeam == YELLOW && state == TableState::DROPZONE_YELLOW)){
        tableStatus.dropzone_proba[dropzoneNum] = 0;
    }else{
        tableStatus.dropzone_proba[dropzoneNum] += (tableStatus.dropzone_proba[dropzoneNum] < TIME_TO_DROP) ? 1 : 0;
    }

    // Condition de vérification pour le passage à une zone controllée par l'adversaire 
    bool teamCheck = (state == TableState::DROPZONE_BLUE && tableStatus.colorTeam == YELLOW) 
                    || (state == TableState::DROPZONE_YELLOW && tableStatus.colorTeam == BLUE);
    if(!teamCheck){
        dropzone_states[dropzoneNum] = state;
        LOG_EXTENDED_DEBUG("Set dropzone ", dropzoneNum, " state to ", state);
    }else if(tableStatus.dropzone_proba[dropzoneNum] >= MIN_DROPZONE_TIME){
        dropzone_states[dropzoneNum] = state;
        LOG_EXTENDED_DEBUG("Set dropzone ", dropzoneNum, " state to ", state);
    }else{
        LOG_EXTENDED_DEBUG("Opponent detected at dropZone but not for long enough");
    }
}

void TableState::setDropzoneAsError(int dropzoneNum){
    setDropzoneState(dropzoneNum, TableState::DROPZONE_ERROR);
}

void TableState::updateMapStatus(const std::vector<bool>& stock, const std::vector<std::pair<int, int>>& dropzone){

    for(size_t i = 0; i < stock.size() && i < STOCK_COUNT; i++){
        if(!stock[i]){
            if(avail_stocks[i] <= TIME_TO_TAKE - MIN_TAKEZONE_TIME){
                avail_stocks[i] = 0;
            }else{
                avail_stocks[i] = TIME_TO_TAKE / 2;
            }
        }else{
            avail_stocks[i] = TIME_TO_TAKE;
        }
    }
    for(size_t i = 0; i < dropzone.size() && i < DROPZONE_COUNT; i++){
        if(std::get<0>(dropzone[i]) == 0 && std::get<1>(dropzone[i]) == 0){
            continue;
        }else if(std::get<1>(dropzone[i]) > std::get<0>(dropzone[i])){
            setDropzoneState(i, DROPZONE_YELLOW);
            if(tableStatus.colorTeam == BLUE){
                dropzone_proba[i] = TIME_TO_DROP + 50 * std::get<0>(dropzone[i]);
            }
        }else if(std::get<1>(dropzone[i]) < std::get<0>(dropzone[i])){
            setDropzoneState(i, DROPZONE_BLUE); 
            if(colorTeam == YELLOW){
                dropzone_proba[i] = TIME_TO_DROP + 50 * std::get<1>(dropzone[i]); 
            }
        }else{
            setDropzoneState(i, colorTeamDropZoneOpponent); // Si égalité, on considère que c'est l'adversaire qui contrôle la zone pour être plus prudent
            dropzone_proba[i] = TIME_TO_DROP;
        }
        
    }

}
