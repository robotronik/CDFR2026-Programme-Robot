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
};

// Serialize tableState
void to_json(json& j, const TableState& ts);