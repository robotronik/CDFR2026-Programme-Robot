#pragma once
#include "defs/structs.hpp"
#include "defs/constante.h"
#include <utils/json.hpp>
using json = nlohmann::json;

class TableState
{
    public:

        TableState();
        ~TableState();

        void reset();
        int getScore();
        
        /* common data */
        position_t pos_opponent;
        unsigned long startTime;

        colorTeam_t colorTeam;
        int strategy;
        int calibrationAge;
        bool cursorStatus = false;
        bool cursorIsDone(){ return cursorStatus; }
        void setCursorIsDone(bool val){ cursorStatus = val; }
        position_t CursorPos = {625, 1220, 45};
        
        /* data Winter is comming */
        bool avail_stocks[STOCK_COUNT];     // Is stock available
        typedef enum
        {
            DROPZONE_EMPTY,
            DROPZONE_YELLOW,
            DROPZONE_BLUE,
            DROPZONE_ERROR
        } dropzone_state_t;

        dropzone_state_t dropzone_states[DROPZONE_COUNT];
        void resetCalibrationAge(){ calibrationAge = 0;}
        void setStockAsRemoved(int num);
        void setDropzoneState(int dropzoneNum, TableState::dropzone_state_t state);
        void setDropzoneAsError(int dropzoneNum);

};

// Serialize tableState
void to_json(json& j, const TableState& ts);
